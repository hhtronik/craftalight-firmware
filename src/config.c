/* SPDX-License-Identifier: GPL-3.0-only */
/* Copyright © 2020 Staudt Technologies GmbH / Yannic Staudt */

#include <stdbool.h>
#include <string.h>
#include "stm8s003.h"
#include "config.h"

// Defaulf configuration values
volatile struct _Configuration Config = {
  .ConfigurationId = CONFIGURATIONID,
  .Brightness = PWM_MIN_SETTABLE,
  .EnableUSBPowerBlip = true,
  .InvertMagneticSwitch = true
};

uint16_t crc16(uint16_t sum, uint8_t *p, uint32_t len)
{
    uint8_t i;
    uint8_t byte;
    uint16_t oSum;

    while (len--)
    {
        byte = *(p++);
        for (i = 0; i < 8; ++i)
        {
            oSum = sum;
            sum <<= 1;
            if (byte & 0x80)
                sum |= 1;

            if (oSum & 0x8000)
                sum ^= 0x1021;  //CRC-CCITT

            byte <<= 1;
        }
    }
    return sum;
}

void config_load( void )
{
    // buffer for the config + CRC16
    uint8_t data[sizeof(Config) + 2];
    
    //  EEPROM base address.
    uint8_t *address = (uint8_t *) 0x4000;
    
    // read the data
    for(uint8_t i = 0; i < sizeof(data); i++)
        data[i] = *address++;
    
    // calculate the checksum of the read data
    uint16_t calcedCrc = crc16(0, data, (uint32_t)sizeof(Config));
    uint16_t savedCrc = (data[sizeof(Config)] << 8) + data[sizeof(Config) + 1];

    // get the config id
    unsigned int configId = (data[0] << 8) + data[1];
    
    if(calcedCrc == savedCrc && CONFIGURATIONID == configId)
    {        
        memcpy(&Config, data, sizeof(Config));
    }
    else
    {
        // write the default stuff...
        config_save();
    }
}


void config_save( void )
{
    // unnprotect the flash area by writing the unlock key
    if (FLASH_IAPSR_bit.DUL == 0)
    {
        FLASH_DUKR = 0xAE;
        FLASH_DUKR = 0x56;
    }

    //  Write the data to the EEPROM.
    uint8_t *address = (uint8_t *) 0x4000;
    uint8_t *configPtr = (uint8_t *)&Config;
    
    uint16_t crc = crc16(0, configPtr, (uint32_t)sizeof(Config));
    
    // copy the data to the eeprom
    for (uint8_t i = 0; i < sizeof(Config); i++)
        *address++ = *configPtr++;
    
    // write the CRC
    *address++ = (uint8_t)(crc >> 8);
    *address++ = (uint8_t)(crc & 0xff);
    
    // lock the EEPROM
    FLASH_IAPSR_bit.DUL = 0;
}

void config_reset( void )
{
    Config.ConfigurationId = CONFIGURATIONID;
    Config.Brightness = PWM_MIN_SETTABLE;
    Config.EnableUSBPowerBlip = true;
    Config.InvertMagneticSwitch = true;

    config_save();
}
