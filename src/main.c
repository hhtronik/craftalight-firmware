/**
 * Craftalight / USB-Powered LED light for (hand)bags
 */
/* SPDX-License-Identifier: GPL-3.0-only */
/* Copyright © 2020 Staudt Technologies GmbH / Yannic Staudt */

#include <stdint.h>
#include <stddef.h>
#include "stm8s003.h"
#include "systick.h"
#include "button.h"
#include "config.h"
#include "usb_blip.h"

// available state-machine states
typedef enum
{
  WAITING = 0,
  IN_SETUP,
  LIGHT_ON,
  DIMMING
} MainState;

//--------------------------------------------------------------
// global state variables, defines etc

volatile MainState mainLoopState = WAITING;
volatile MainState mainLoopTargetState = WAITING;
volatile uint32_t last_wake = 0;           // last millis() we woke up
volatile uint32_t setup_activity_time = 0; // last millis() there was activity in the setup mode
volatile int8_t setup_dimming_dir = 1;     // dimming direction in setup mode
volatile uint16_t set_pwm_value = PWM_MAX; // last set TIM1 pwm value

// waiting time between main loop itterations
#define MAIN_LOOP_STEP_DURATION 5

// keep setup active for SETUP_TIMEOUT_MS after last activity
#define SETUP_TIMEOUT_MS 10000

// how long dimming ON/OFF takes
#define DIMMING_ON_DURATION_MS 250
#define DIMMING_OFF_DURATION_MS 1000

// speed of the setup mode activity indication dimming
#define DIMMING_CYCLE_SETUP_MS 350
#define DIMMING_STEP_SETUP ((PWM_MAX - PWM_MIN_SETTABLE) / (DIMMING_CYCLE_SETUP_MS / MAIN_LOOP_STEP_DURATION))

//--------------------------------------------------------------
// fn/method prototypes
void init_sysclock(void);
void init_TIM1(void);
void init_low_power(void);
void init_gpio(void);
void init_option_bytes(void);
void TIM1_set(uint16_t count);

// blink the power LEDs N times (with specified duty cycle)
void blink_leds_delay(uint8_t times, uint16_t delayOn, uint16_t delayOff);
void blink_leds(uint8_t times) { blink_leds_delay(times, 250, 250); }

// resets the EEPROM-storage configuration values
void user_reset_config(void);

// read the hall sensor and apply the configured value inversion logic
bool hall_sensor_read_active(void);

// every xxx_tick() method is called in the main loop
// depending on the state-machine's state at roughly
// a MAIN_LOOP_STEP_DURATION step duration
void hall_sensor_tick(void);       // samples the Hall-Effect sensor and applies debouncing
void main_loop_dimming_tick(void); // dims the LEDs ON/OFF when mainLoopState == DIMMING
void main_loop_setup_tick(void);   // takes care of showing that the setup is active + exits it after timing out

// Button action callbacks
// Options regarding debouncing & press lengths in button.h/button.c
void user_button_click(void);
void user_button_doubleclick(void);
void user_button_longpress_start(void);

//--------------------------------------------------------------
// main method
void main()
{
  __disable_interrupts(); // we have to disable interrupts prior to configuring some peripherals
  init_sysclock();
  systick_enable();
  init_TIM1();
  init_low_power();
  init_option_bytes();  

  // enable interrupts (on both edge) for Port B and D
  EXTI_CR1_bit.PBIS = EXTI_CRx_PxIS_RisingAndFallingEdge; // Interrupt on both edge.
  EXTI_CR1_bit.PDIS = EXTI_CRx_PxIS_RisingAndFallingEdge; // Interrupt on both edge.
  EXTI_CR1_bit.PCIS = EXTI_CRx_PxIS_RisingAndFallingEdge; // Interrupt on both edge.

  __enable_interrupts();

  config_load();
  init_gpio();
  usb_blip_reconfigure();

  // test for user input during power up
  // => resets config if required
  user_reset_config();

  // attach the button callbacks
  button_attach(SINGLE_CLICK, user_button_click);
  button_attach(DOUBLE_CLICK, user_button_doubleclick);
  button_attach(LONG_PRESS_START, user_button_longpress_start);

  // the main loop
  while (1)
  {
    usb_blip_tick();
    button_tick();
    hall_sensor_tick();

    if (mainLoopState != mainLoopTargetState)
    {
      // we have to DIM to get from waiting to Lights on and vice-versa
      if ((mainLoopState == WAITING && mainLoopTargetState == LIGHT_ON) || (mainLoopState == LIGHT_ON && mainLoopTargetState == WAITING))
      {
        mainLoopState = DIMMING;

        if (mainLoopTargetState == LIGHT_ON)
          set_pwm_value = PWM_MAX;
        else
          set_pwm_value = Config.Brightness;
      }

      // jumping right into the setup
      if (mainLoopTargetState == IN_SETUP)
      {
        mainLoopState = IN_SETUP;
        setup_activity_time = millis();
        TIM1_CR1_bit.CEN = 1; // TIM1 might be disabled

        // proper entry conditions
        setup_dimming_dir = 1;
        set_pwm_value = PWM_MAX;
      }

      // exiting setup
      if (mainLoopState == IN_SETUP && mainLoopTargetState == WAITING)
      {
        mainLoopState = DIMMING;
        last_wake = millis();
      }
    }

    switch (mainLoopState)
    {
    case DIMMING:
      main_loop_dimming_tick();
      break;

    case IN_SETUP:
      main_loop_setup_tick();
      break;

    default:
      break;
    }

    // go to (active)halt depending on the state
    if (last_wake + 1500 <= millis() && mainLoopState == WAITING && usb_blip_can_sleep())
    {
      __halt();
      last_wake = millis();
    }
    else
    {
      delayms(MAIN_LOOP_STEP_DURATION);
    }
  }
}

// On the STM8S003 we have to use the OPT2 option byte to "remap"
// the TIM1_CH1 PWM output to pin C6. To do this we set the AFR0 bit
// in OPT2 to 1 and it's complementary in NOPT2 to zero.
//
// This is done by writing to the flash memory, so we have to unlock that first.
// To not wear out the flash memory we check the config prior to doing anything
void init_option_bytes(void)
{
  // option bit already set, move on
  if(OPT2_bit.AFR0 == 1) return;

  // unnprotect the flash area by writing the unlock key
  FLASH_DUKR = FLASH_DUKR_KEY1;
  FLASH_DUKR = FLASH_DUKR_KEY2;

  // wait for the unlocking to be complete
  while (!(FLASH_IAPSR_bit.DUL));

  // allow us writing the option bytes
  FLASH_CR2_bit.OPT = 1;
  FLASH_NCR2_bit.NOPT = 0;

  // change the config
  OPT2_bit.AFR0 = 1;
  NOPT2_bit.NAFR0 = 0;

  // wait for the end of the programming
  while (!(FLASH_IAPSR_bit.EOP));
    
  // re-lock the flash
  FLASH_IAPSR_bit.DUL = 0;
}

void init_gpio(void)
{
  //power LEDs / NMOS gate on on PC6
  PC_DDR_bit.DDR6 = 1; // output
  PC_CR1_bit.C16 = 1;  // fast mode

  //
  // Hall effect sensor on PB5
  // 
  // NOTES:
  // Sensor: SI7201-B-07-IVR has an open drain output, hence why we 
  // want to enable the internal pull-up resistor on the GPIO or use
  // SI7201-B-05-IVR instead which has a push-pull output driver
  PB_DDR_bit.DDR5 = 0; // input
  //PB_CR1_bit.C15 = 1;   // weak pull up enabled not available on PB5 :/ TODO: reroute to PC3!
  PB_CR2_bit.C25 = 1; // interrupt enabled

  // USB load MOSFET gate on PA3
  PA_DDR_bit.DDR3 = 1; // output
  PA_CR1_bit.C13 = 1;  // push-pull mode
  PA_ODR_bit.ODR3 = 0; // set the pin to LOW to switch the mosfet OFF

  // User button on PC7 
  button_init();
}

void init_low_power(void)
{
  CLK_ICKR_bit.REGAH = 1;  // disable MVR regulator (active)-halt mode
  CLK_ICKR_bit.FHWU = 0;   // disable fast wake-up from halt
  FLASH_CR1_bit.HALT = 0;  // Switch the Flash to power down in halt state, increses wakeup by few us but saves power
  FLASH_CR1_bit.AHALT = 1; // Switch the Flash to power down in active halt state, increse wakeup by few us
}

void init_TIM1(void)
{
  //
  // for the LED driver PWM (TIM1 / Ch 1 / PC 6)
  // NOTE: take care of having the alternate function AFR0 set
  // in the chip's option bytes
  TIM1_ARRH = (PWM_MAX & 0xff00) >> 8; // Reload counter = 255 (8 bits)
  TIM1_ARRL = (PWM_MAX & 0x00ff);
  TIM1_PSCRH = 0x00; // Prescalar = 0 (i.e. 1)
  TIM1_PSCRL = 0x00;
  TIM1_CR1_bit.DIR = 1;                 // Down counter.
  TIM1_CR1_bit.CMS = 1;                 // Edge aligned counter.
  TIM1_RCR = 0;                         // Repetition count.
  TIM1_CCMR1_out_bit.OC1M = 7;          // PWM mode 2
  TIM1_CCER1_bit.CC1E = 1;              // Output is enabled
  TIM1_CCER1_bit.CC1P = 0;              // Active level: high
  TIM1_CCR1H = (PWM_MAX & 0xff00) >> 8; //  Start with the PWM signal off.
  TIM1_CCR1L = (PWM_MAX & 0x00ff);

  TIM1_BKR_bit.MOE = 1;
  TIM1_CR1_bit.CEN = 0; // disable the peripheral
}

void init_sysclock(void)
{
  CLK_ICKR = 0;           // Reset the Internal Clock Register.
  CLK_ICKR_bit.HSIEN = 1; // Enable the HSI (high speed internal osc.)
  CLK_ICKR_bit.LSIEN = 1; // Enable the LSI (low speed internal osc.) to allow for low power active halt
  CLK_ECKR = 0;           // Disable the external clock.

  // clock speed divider
  // 0 = System clock source /1 (16MHz from HSI)
  // 1 = System clock source /2 (8MHz from HSI)
  // 5 = System clock source /32 (0.5MHz from HSI)
  // 7 = System clock source /128 (125kHz from HSI)
  CLK_CKDIVR = 0; //  Ensure the clocks are running at full speed.

  CLK_CCOR = 0;     //  Turn off Configurable Clock Ouput.
  CLK_HSITRIMR = 0; //  Turn off any HSIU trimming.
  CLK_SWIMCCR = 0;  //  Set SWIM to run at clock / 2.
  CLK_SWR = 0xe1;   //  Use HSI as the clock source.
  CLK_SWCR = 0;     //  Reset the clock switch control register.

  CLK_SWCR_bit.SWEN = 1; //  Enable switching.
  while (CLK_SWCR_bit.SWBSY)
    ; // wait for clock switch to complete

  // setup / distribute the clock to selected peripherals
  CLK_PCKENR1 = 0;
  CLK_PCKENR1_bit.PCKEN17 = 1; // TIM1 (used as PWM driver)
  CLK_PCKENR1_bit.PCKEN16 = 0; // TIM3
  CLK_PCKENR1_bit.PCKEN15 = 0; // TIM2
  CLK_PCKENR1_bit.PCKEN14 = 1; // TIM4 (used for systick)
  CLK_PCKENR1_bit.PCKEN13 = 0; // UART1
  CLK_PCKENR1_bit.PCKEN12 = 0; // UART2
  CLK_PCKENR1_bit.PCKEN11 = 0; // SPI
  CLK_PCKENR1_bit.PCKEN10 = 0; // I2C1

  CLK_PCKENR2 = 0;
  CLK_PCKENR2_bit.PCKEN27 = 0; // Reserved
  CLK_PCKENR2_bit.PCKEN25 = 0; // Reserved
  CLK_PCKENR2_bit.PCKEN24 = 0; // Reserved
  CLK_PCKENR2_bit.PCKEN23 = 0; // ADC
  CLK_PCKENR2_bit.PCKEN22 = 0; // AWU (might be [re]configured in usb_blip.c)
  CLK_PCKENR2_bit.PCKEN21 = 0; // Reserved
  CLK_PCKENR2_bit.PCKEN20 = 0; // Reserved
}

//
// bit-shift debouncing for the hall sensor
// sensor is considered switch on/off
// if the state is steady of 8 samples
//
// 0b11111111 == ON
// 0b00000000 == OFF
//
// The sensor state is shifted to the left every hall_sensor_tick()
// and the read state is appended.
//
volatile uint8_t _hall_sensor_state = 0;

void hall_sensor_tick(void)
{
  _hall_sensor_state = (_hall_sensor_state << 1);
  bool sensorHigh = hall_sensor_read_active();

  if (sensorHigh) 
    _hall_sensor_state |= 1;

  // feed the state machine if necessary
  if (mainLoopState == WAITING && _hall_sensor_state == 255)
  {
    mainLoopTargetState = LIGHT_ON;
  }
  else if (mainLoopState == LIGHT_ON && _hall_sensor_state == 0)
  {
    mainLoopTargetState = WAITING;
  }
}

//
// transparently handles the "inversion" by configuration
bool hall_sensor_read_active(void)
{
  return ((PB_IDR_bit.IDR5 == 0) ^ (Config.InvertMagneticSwitch));
}

// for very narrow brightness jumps we use a /10 substep mechanism
// where the PWM value gets incremented/decremented only every 10 calls
// of main_loop_dimming_tick()
volatile uint8_t dimming_substep_counter = 0;

void main_loop_dimming_tick(void)
{
  uint16_t step = (PWM_MAX - Config.Brightness) / (DIMMING_ON_DURATION_MS / MAIN_LOOP_STEP_DURATION);
  if (step < 1 && dimming_substep_counter > 10)
  {
    step = 1;
    dimming_substep_counter = 0;
  }
  else
  {
    dimming_substep_counter++;
  }

  TIM1_set(set_pwm_value);

  if (mainLoopTargetState == LIGHT_ON)
  {
    TIM1_CR1_bit.CEN = 1;
    set_pwm_value -= step;

    // we've finished dimming
    if (set_pwm_value < Config.Brightness)
    {
      TIM1_set(Config.Brightness); // we jump one step eventually in some cases
      mainLoopState = LIGHT_ON;
    }
  }
  else if (mainLoopTargetState == WAITING)
  {
    set_pwm_value += step;

    // we've finished dimming
    if (set_pwm_value >= PWM_MAX)
    {
      TIM1_set(PWM_MAX); // we jump one step eventually in some cases
      mainLoopState = WAITING;
      TIM1_CR1_bit.CEN = 0; // disable Timer1
    }
  }
}

//
// shows that the setup mode is active by dimming on/off rapidly
// + feeds the state machine when the (in)activity timer runs out
void main_loop_setup_tick(void)
{
  // fade in/out to show that the menu is active:
  if (set_pwm_value > PWM_MAX)
    setup_dimming_dir = -1;
  else if (set_pwm_value <= PWM_MIN_SETTABLE)
    setup_dimming_dir = 1;

  set_pwm_value += setup_dimming_dir * DIMMING_STEP_SETUP;
  TIM1_set(set_pwm_value);

  // exit the menu after timeout ms elapsed
  if (millis() - setup_activity_time > SETUP_TIMEOUT_MS)
    mainLoopTargetState = WAITING;
}

//--------------------------------------------------------------
// Button action implementations
// naming conventions: 
//
// button_*actionName*() 
//  -> for the handlers registered with button.c/button.h
//  -> basically dispatches calls depending on the state of
//  -> the state machine
//
// button_*actionName*_*StateMachineStateName*()
//  -> for implementations of state specific actions



//
// when a button click happens when LIGHT_ON we change the 
// set brightness and store the new configuration
void button_click_LIGHT_ON(void)
{
  Config.Brightness = Config.Brightness + ((PWM_MAX - PWM_MIN_SETTABLE) / 6);

  // wrap to minimum value at end of range
  if (Config.Brightness > PWM_MAX || Config.Brightness <= PWM_MIN_SETTABLE)
    Config.Brightness = PWM_MIN_SETTABLE;

  config_save();

  set_pwm_value = Config.Brightness;
  TIM1_set(set_pwm_value);
}

//
// IN_SETUP a button click changes the "InvertMagneticSwitch" configuration
// of the Hall-Effect sensor. Configuration is persisted
void button_click_IN_SETUP(void)
{
  Config.InvertMagneticSwitch = !Config.InvertMagneticSwitch;
  _hall_sensor_state = 0; // reset the state...
  config_save();

  if (Config.InvertMagneticSwitch)
    blink_leds_delay(2, 150, 500);
  else
    blink_leds_delay(2, 500, 150);

  setup_activity_time = millis();
}

// actual button callback single click changes the LED brightness
void user_button_click(void)
{
  switch (mainLoopState)
  {
  case WAITING:
    // todo: should we switch on the light for a short period of time?
    break;

  case IN_SETUP:
    button_click_IN_SETUP();
    break;

  case LIGHT_ON:
    button_click_LIGHT_ON();
    break;

  default:
    break;
  }
}

//
// IN_SETUP a double click toggles the "USB Power-Blip" feature
// again, configuration is persisted
void button_doubleclick_IN_SETUP(void)
{
  Config.EnableUSBPowerBlip = !Config.EnableUSBPowerBlip;
  config_save();
  usb_blip_reconfigure();

  if (Config.EnableUSBPowerBlip)
    blink_leds_delay(5, 150, 800);
  else
    blink_leds_delay(10, 150, 150);

  setup_activity_time = millis();
}

void user_button_doubleclick(void)
{
  switch (mainLoopState)
  {
  case WAITING:
  case LIGHT_ON:
    // any idea what to use this one for? _/\o/\_
    break;

  case IN_SETUP:
    // => toggle usb blip mode
    button_doubleclick_IN_SETUP();
    break;

  default:
    break;
  }
}

void user_button_longpress_start(void)
{
  switch (mainLoopState)
  {

  case IN_SETUP:
    // any idea what to use this one for? _/\o/\_
    break;

  case WAITING:
  case LIGHT_ON:
    // => enter the setup mode
    mainLoopTargetState = IN_SETUP;
    break;

  default:
    break;
  }
}

//--------------------------------------------------------------
// helpers
void TIM1_set(uint16_t count)
{
  TIM1_CCR1H = (count & 0xff00) >> 8;
  TIM1_CCR1L = count & 0x00ff;
}

void blink_leds_delay(uint8_t times, uint16_t delayOn, uint16_t delayOff)
{
  // automatically handle the state of TIM1 to have a stateless fn
  bool toggleTIM1 = (TIM1_CR1_bit.CEN == 0);

  if (toggleTIM1)
    TIM1_CR1_bit.CEN = 1;

  for (uint8_t tick = 0; tick < times; tick++)
  {
    TIM1_set(PWM_MAX);
    delayms(delayOff);
    TIM1_set(PWM_MIN_SETTABLE);
    delayms(delayOn);
  }

  TIM1_set(PWM_MAX);

  if (toggleTIM1)
    TIM1_CR1_bit.CEN = 1;
}

// called on startup. If user holds down the button
// we will reset the config and blink 3 times for confirmation
void user_reset_config(void)
{
  uint8_t pinstate = 255;
  unsigned int pinStateCounter = 0;

  // simple windows debouncing
  while (pinstate > 0 && pinStateCounter < 50)
  {
    pinstate = (pinstate << 1);
    pinstate |= PC_IDR_bit.IDR7;
    pinStateCounter++;
    delayms(5);
  }

  // button was pressed in our books
  // => reset config and show user we did something
  if (pinstate == 0)
  {
    config_reset();
    blink_leds_delay(5, 100, 100);
  }
}

//--------------------------------------------------------------
// IRQ prototypes
//  /!\ SDCC required they are defined in the same file as main()
// /_!_\ Actual implementation can be spread over the application then.
//
void TIM4_UIF_IRQHandler() __interrupt(ISRV_TIM4); // Timer4 (=> systick.c)
void AWU_IRQHandler() __interrupt(ISRV_AWU);       // Auto Wake Unit (=> usb_blip.c)

void PORTC_IRQHandler(void) __interrupt(ISRV_EXTI2_PORTC)
{
  /* nothing to do here right now */
}

void PORTB_IRQHandler(void) __interrupt(ISRV_EXTI1_PORTB)
{
  /* nothing to do here right now */
}

void PORTA_IRQHandler(void) __interrupt(ISRV_EXTI0_PORTA)
{
  /* nothing to do here right now */
}