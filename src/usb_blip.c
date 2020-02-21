/* SPDX-License-Identifier: GPL-3.0-only */
/* Copyright © 2020 Staudt Technologies GmbH / Yannic Staudt */

#include <stdbool.h>
#include "usb_blip.h"
#include "stm8s003.h"
#include "config.h"
#include "systick.h"

void usb_blip_reconfigure( void )
{
    // disable the peripheral & clock before reconfiguring!
    AWU_CSR1_bit.AWUEN = 0;
    CLK_PCKENR2_bit.PCKEN22 = 0;	

    if(Config.EnableUSBPowerBlip)
    {
        // supply clock to the peripheral
        CLK_PCKENR2_bit.PCKEN22 = 1;	

        // this sets the timeout to ~14s +-12% (precision of the LSI _/\Ö/\_)
        // which seems to work fine with many power banks
        AWU_APR_bit.APR = 22;
        AWU_TB_bit.AWUTB = 15;

        // enable the wake-up unit
        AWU_CSR1_bit.AWUEN = 1; 
    }    
}

volatile bool powerBlipOnNextTick = false;
volatile  bool powerBlipOn = false;
volatile uint32_t powerBlipStart = 0;

bool usb_blip_can_sleep( void )
{
    return !powerBlipOn && !powerBlipOnNextTick;
}

void usb_blip_tick( void )
{
    if(powerBlipOn && (millis() - powerBlipStart > BLIP_DURATION_MS))
    {
        PA_ODR_bit.ODR3 = 0;
        powerBlipOn = false;
    }

    if(powerBlipOnNextTick) 
    {
        powerBlipOnNextTick = false;
        powerBlipStart = millis();
        powerBlipOn = true;
        PA_ODR_bit.ODR3 = 1;
    }


#ifdef BLIP_IN_RUN_MODE
    // blip every 15s basically if feature is enabled
    if(Config.EnableUSBPowerBlip 
      && !powerBlipOn 
      && (millis() - powerBlipStart > 14000))
    {
        powerBlipOnNextTick = true;
    }    

#endif
}

//
// the auto-wakeup happens only when the device is actually in
// __halt() (actually active halt state which is chosen automatically)
// by the hardware when the auto-wakeup unit is enabled
void AWU_IRQHandler() __interrupt(ISRV_AWU)
{
    powerBlipOnNextTick = true;    
    volatile uint8_t reg;
    reg = AWU_CSR1; // reading the register to clear the interrupt flag
}