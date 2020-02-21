/* SPDX-License-Identifier: GPL-3.0-only */
/* Copyright © 2020 Staudt Technologies GmbH / Yannic Staudt */

#ifndef H_BUTTON
#define H_BUTTON

#define TICKS_INTERVAL    5	// in ms
#define DEBOUNCE_TICKS    3	// MAX 8
#define SHORT_TICKS       (150 / TICKS_INTERVAL)
#define LONG_TICKS        (900 / TICKS_INTERVAL)

typedef void (*BtnCallback)( void );

typedef enum {
	PRESS_DOWN = 0,
	PRESS_UP,
	PRESS_REPEAT,
	SINGLE_CLICK,
	DOUBLE_CLICK,
	LONG_PRESS_START,
	LONG_PRESS_HOLD,
    LONG_PRESS_UP,	
	NOT_PRESSED,
	PressEvents_count
}PressEvent;

void button_init( void );
void button_attach(PressEvent event, BtnCallback cb);
void button_tick( void );
PressEvent get_button_event( void );

#endif
