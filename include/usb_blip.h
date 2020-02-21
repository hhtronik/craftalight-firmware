/* SPDX-License-Identifier: GPL-3.0-only */
/* Copyright © 2020 Staudt Technologies GmbH / Yannic Staudt */

#ifndef H_USB_BLIP
#define H_USB_BLIP

#include <stdbool.h>

//
// USB_BLIP takes care of loading the USB power bank for a few 
// milliseconds at a defined interval to make sure it doesn't 
// go to standby.
//
// It's wastefull but should help in most cases

// this value should be as low as possible for least waste-of
// power. Exact duration is aligned on a 5ms itteration window
// of the main loop so it won't be suuuuper accurate
#define BLIP_DURATION_MS 350

// some power banks don't see the lower power settings as
// a "worthy" load either, so 
#define BLIP_IN_RUN_MODE 

// configures the AWU according to user settings
void usb_blip_reconfigure( void );

// should be called in a relatively short intervall 
// by the main loop
void usb_blip_tick( void );

// returns true when we're not in the middle of a power blip
bool usb_blip_can_sleep( void );
#endif
