/* SPDX-License-Identifier: GPL-3.0-only */
/* Copyright © 2020 Staudt Technologies GmbH / Yannic Staudt */

#ifndef H_CONFIG
#define H_CONFIG

#include <stdint.h>
#include <stdbool.h>

#define CONFIGURATIONID 2137
#define PWM_MAX 400
#define PWM_MIN_SETTABLE 35

struct _Configuration
{ 
    uint16_t ConfigurationId;       // fingerprint for this configuration
    uint16_t Brightness;            // configured PWM value
    bool EnableUSBPowerBlip;        // if enabled the firmware loads the USB bus with a 100R every few seconds
    bool InvertMagneticSwitch;      // set to true to invert the function of the magnetic switch
};

extern volatile struct _Configuration Config;


void config_load( void );
void config_save( void );
void config_reset( void );

#endif
