/* SPDX-License-Identifier: GPL-3.0-only */
/* Copyright © 2020 Staudt Technologies GmbH / Yannic Staudt */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm8s003.h"
#include "button.h"

typedef struct Button
{
  uint16_t ticks;
  uint8_t repeat : 4;
  uint8_t event : 4;
  uint8_t state : 3;
  uint8_t debounce_cnt : 3;
  uint8_t current_state : 1;
  BtnCallback cb[PressEvents_count];
} Button;

volatile Button userBtn;

// macro as we use that line a lot... makes code more readable
// actually just calls the right event callback of the usrBtn struct
// if its defined
#define CALL_BTN_CALLBACK(ev) \
  if (userBtn.cb[ev])         \
    userBtn.cb[ev]()

// read PC7 state
uint8_t read_hardwareButton_GPIO() { return PC_IDR_bit.IDR7; }

void button_init(void)
{
  // GPIO initialization
  PC_DDR_bit.DDR7 = 0; // input
  PC_CR1_bit.C17 = 1;  // pull up
  PC_CR2_bit.C27 = 1;  // interrupt enabled to allow waking from halt

  userBtn.event = (uint8_t)NOT_PRESSED;
  userBtn.current_state = read_hardwareButton_GPIO();
}

PressEvent get_button_event()
{
  return userBtn.event;
}

void button_attach(PressEvent event, BtnCallback cb)
{
  userBtn.cb[event] = cb;
}

void button_tick()
{
  uint8_t read_btn_state = read_hardwareButton_GPIO();

  // increment the ticks if there's an active state
  // enables us to enter long press etc...
  if ((userBtn.state) > 0)
    userBtn.ticks++;

  // debounce the button
  if (read_btn_state != userBtn.current_state)
  {
    if (++(userBtn.debounce_cnt) >= DEBOUNCE_TICKS)
    {
      userBtn.current_state = read_btn_state;
      userBtn.debounce_cnt = 0;
    }
  }
  else
  {
    userBtn.debounce_cnt = 0;
  }

  switch (userBtn.state)
  {
  // start press down
  case 0:
    if (userBtn.current_state == LOW)
    {
      userBtn.event = (uint8_t)PRESS_DOWN;
      CALL_BTN_CALLBACK(PRESS_DOWN);
      userBtn.ticks = 0;
      userBtn.repeat = 1;
      userBtn.state = 1;
    }
    else
    {
      userBtn.event = (uint8_t)NOT_PRESSED;
    }
    break;

    // released / press up
    // or long press start depending on the lenght of holding
  case 1:
    if (userBtn.current_state == HIGH)
    {
      userBtn.event = (uint8_t)PRESS_UP;
      CALL_BTN_CALLBACK(PRESS_UP);
      userBtn.ticks = 0;
      userBtn.state = 2;
    }
    else if (userBtn.ticks > LONG_TICKS)
    {
      userBtn.event = (uint8_t)LONG_PRESS_START;
      CALL_BTN_CALLBACK(LONG_PRESS_START);
      userBtn.state = 5;
    }
    break;

  // repeated press down
  case 2:
    if (userBtn.current_state == LOW)
    {
      userBtn.event = (uint8_t)PRESS_DOWN;
      CALL_BTN_CALLBACK(PRESS_DOWN);
      userBtn.repeat++;
      if (userBtn.repeat == 2)
      {
        CALL_BTN_CALLBACK(DOUBLE_CLICK); // repeat hit
      }
      CALL_BTN_CALLBACK(PRESS_REPEAT); // repeat hit
      userBtn.ticks = 0;
      userBtn.state = 3;
    }
    else if (userBtn.ticks > SHORT_TICKS)
    { // released timeout
      if (userBtn.repeat == 1)
      {
        userBtn.event = (uint8_t)SINGLE_CLICK;
        CALL_BTN_CALLBACK(SINGLE_CLICK);
      }
      else if (userBtn.repeat == 2)
      {
        userBtn.event = (uint8_t)DOUBLE_CLICK;
      }
      userBtn.state = 0;
    }
    break;

  // released again
  case 3:
    if (userBtn.current_state == HIGH)
    { 
      userBtn.event = (uint8_t)PRESS_UP;
      CALL_BTN_CALLBACK(PRESS_UP);
      if (userBtn.ticks < SHORT_TICKS)
      {
        userBtn.ticks = 0;
        userBtn.state = 2; //repeat press
      }
      else
      {
        userBtn.state = 0;
      }
    }
    break;

  // ...still holding or just released after
  // a longer period
  case 5:
    if (userBtn.current_state == LOW)
    {
      // NOTE: LONG_PRESS_HOLD is issued on every tick
      // as-long-as we keep pressing the button
      userBtn.event = (uint8_t)LONG_PRESS_HOLD;
      CALL_BTN_CALLBACK(LONG_PRESS_HOLD);
    }
    else
    { 
      userBtn.event = (uint8_t)LONG_PRESS_UP;
      CALL_BTN_CALLBACK(LONG_PRESS_UP);
      userBtn.state = 0; //reset
    }
    break;
  }
}
