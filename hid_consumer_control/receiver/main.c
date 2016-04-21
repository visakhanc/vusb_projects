/* Name: main.c
 * Project: hid-mouse, a very simple HID example
 * Author: Christian Starkjohann
 * Creation Date: 2008-04-07
 * Tabsize: 4
 * Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 */

/*
This example should run on most AVRs with only little changes. No special
hardware resources except INT0 are used. You may have to change usbconfig.h for
different I/O pins for USB. Please note that USB D+ must be the INT0 pin, or
at least be connected to INT0 as well.

We use VID/PID 0x046D/0xC00E which is taken from a Logitech mouse. Don't
publish any hardware using these IDs! This is for demonstration only!
*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
#include "oddebug.h"        /* This is also an example for using debug macros */
#include "rfm70.h"
#include "ir_nec.h"

#define RED_LED					PC1
#define RED_LED_DDR				DDRC
#define RED_LED_PORT			PORTC

#define RED_LED_OUT()			(RED_LED_DDR |= (1 << RED_LED))
#define RED_LED_ON()			(RED_LED_PORT |= (1 << RED_LED))
#define RED_LED_OFF()			(RED_LED_PORT &= ~(1 << RED_LED))
#define RED_LED_TOGGLE()		(RED_LED_PORT ^= (1 << RED_LED))

#define 	SEND_STATE_KB			1
#define 	SENT_STATE_KB			2
#define 	SEND_STATE_MM			3
#define 	SENT_STATE_MM			4
#define 	WAIT_STATE				0

/* Bits for Multimedia key mapping */
#define 	NO_KEYS				0
#define 	VOL_UP_KEY			(1 << 5)
#define 	VOL_DOWN_KEY		(1 << 6)
#define 	MUTE_KEY			(1 << 4)
#define 	STOP_KEY			(1 << 2)
#define 	PLAY_KEY			(1 << 3)
#define 	NEXT_KEY			(1 << 0)
#define 	PREV_KEY			(1 << 1)

/* Standard Keyboard usage codes */
#define 	KEYBOARD_RIGHT		0x4F
#define 	KEYBOARD_LEFT		0x50
#define 	KEYBOARD_UP			0x52
#define 	KEYBOARD_DOWN		0x51
#define 	KEYBOARD_F4			0x3D
#define 	KEYBOARD_ENTER		0x58
#define 	KEYBOARD_TAB		0x2B
#define 	KEYBOARD_APP		0x65
#define 	KEYBOARD_ESC		0x29

#define 	KEYBOARD_MOD_ALT	(1 << 2)
#define 	KEYBOARD_MOD_CTRL	(1 << 0)
#define 	KEYBOARD_MOD_SHIFT	(1 << 1)
#define 	KEYBOARD_MOD_GUI	(1 << 3)

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

/* Keyboard report descriptor*/
PROGMEM const char usbHidReportDescriptor[104] = { /* USB report descriptor, size must match usbconfig.h */
	
	0x05, 0x01,						// USAGE PAGE(Generic desktop)
	0x09, 0x06,						// USAGE (Keyboard)
	0xa1, 0x01,						// COllection (application)
	0x85, 0x01,						// 	  Usage ID(1)
	0x05, 0x07,                    //     USAGE_PAGE (Keycodes)
    0x19, 0xE0,                    //     USAGE_MINIMUM (224)
    0x29, 0xE7,                    //     USAGE_MAXIMUM (231)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x08,                    //     REPORT_COUNT (8)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs) ->Modifier byte
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x81, 0x01,                    //     INPUT (Const) -> Reserved byte
    0x95, 0x05,                    //     REPORT_COUNT (5)
    0x75, 0x01,                    //     REPORT_SIZE (1)
	0x05, 0x08,                    //     USAGE_PAGE (LEDs)
//	0x85, 0x01,						// 		ReportID(1)
    0x19, 0x01,                    //     USAGE_MINIMUM (1)
    0x29, 0x05,                    //     USAGE_MAXIMUM (5)
	0x91, 0x02,					   //	  Output(Data, variable, Abs) -> LED report
    0x95, 0x01,						// 		REPORT_COUNT (1)
	0x75, 0x03,						//		REPORT_SIZE (3)
	0x91, 0x01,						// 		Output(Const)
	0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x06,                    //     REPORT_COUNT (6)
    0x15, 0x00,						//		Logical Min(0)
	0x25, 0x65,						// 		Logical Max(101)
	0x05, 0x07,						// 		Usage Page (Keycodes)
	0x19, 0x00,						//		Usage Min(0)
	0x29, 0x65,						//		Usage Max(101)
	0x81, 0x00,                    //     INPUT (Data,Array)
    0xC0,                          // END COLLECTION

    0x05, 0x0C,                    // USAGE_PAGE (Consumer)
    0x09, 0x01,                    // USAGE (Consumer control)
    0xa1, 0x01,                    // COLLECTION (Application)
	0x85, 0x02,						// 	ReportID(2)
	0x05, 0x0C,						// USAGE_PAGE (Consumer)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //     REPORT_SIZE (1)
	0x95, 0x07,                    //     REPORT_COUNT (7)
	0x09, 0xB5,						// 		NEXT TRACK (BIT 0)
	0x09, 0xB6,						// 		PREV TRACK (BIT 1)
	0x09, 0xB7,						// 		STOP (BIT 2)
	0x09, 0xCD,						//		PLAY/PAUSE(BIT 3)
	0x09, 0xE2,						// 		MUTE (BIT 4)
	0x09, 0xE9,						//		VOL INCREMENT (BIT 5)
	0x09, 0xEA,						//		VOL DECREMENT (BIT 6)
	0x81, 0x02,						// 		INPUT(Data, variable, absolute)
	0x95, 0x01,						// 	  REPORT COUNT(1)
	0x81, 0x01,						// 		INPUT(Constant)
    0xC0                          // END COLLECTION
};

/* Multimedia keys report */
typedef struct{
	uint8_t reportId;
	uint8_t key_byte;
}multi_report_t;

/* Keyboard report */
typedef struct{
	uint8_t reportId;
	uint8_t modifier;
	uint8_t _reserved;
	uint8_t keycode[6];
}keyboard_report_t;


static multi_report_t multi_report_buf = { .reportId = 2};
static keyboard_report_t keyboard_report_buf = { .reportId = 1 };
static uint8_t state = WAIT_STATE;
static uint8_t idleRate;
static uint8_t ledState;
static uint8_t bytes_remaining;
static uint8_t *send_buf;

static uint8_t rx_buf[8];
static uint8_t rfm70_addr[CONFIG_RFM70_ADDR_LEN] = CONFIG_RFM70_ADDRESS;
/* ------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            if(rq->wValue.bytes[0] == 1) { /* report id: 1 (keyboard)*/
				keyboard_report_buf.modifier = 0;
				keyboard_report_buf.keycode[0] = 0;
				usbMsgPtr = (void *)&keyboard_report_buf;
				return sizeof(keyboard_report_buf);
			}
			else if(rq->wValue.bytes[0] == 2) { /* report id: 2 (multimedia keys)*/
				multi_report_buf.key_byte = 0;
				usbMsgPtr = (void *)&multi_report_buf;
				return sizeof(multi_report_buf);
			}
        }
		else if(rq->bRequest == USBRQ_HID_SET_REPORT) {
			if(rq->wLength.word == 1) { /* set keyboard LEDs */
				return USB_NO_MSG; /* pass on to usbFunctionWrite() */
			}
		}else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
    return 0;   /* default for not implemented requests: return no data back to host */
}

usbMsgLen_t	usbFunctionWrite(uint8_t *data, uint8_t len)
{
	if(ledState == data[0]) { /* same value as present */
		return 1; 
	}
	else {
		ledState = data[0];
	}
	/* LED for Capslock */
	if(ledState & 0x2) {
		RED_LED_ON();
	} 
	else {
		RED_LED_OFF();
	}
	
	return 1;
}
/* ------------------------------------------------------------------------- */



static void Receive_Packet(void)
{
	uint8_t len = 0;
	uint8_t ir_code; 
	uint8_t key_byte;  /* Pressd Multimedia key */
	
	rfm70_receive_packet(rx_buf, &len);
	if(0 != len) {
		RED_LED_TOGGLE();
		key_byte = NO_KEYS;
		ir_code = rx_buf[1];
		switch(ir_code) {
			case RC_VOL_UP: 
				key_byte = VOL_UP_KEY;
				break;
			case RC_VOL_DOWN:
				key_byte = VOL_DOWN_KEY;
				break;
			case RC_NEXT:
				key_byte = NEXT_KEY;
				break;
			case RC_PREV:
				key_byte = PREV_KEY;
				break;
			case RC_STOP:
				key_byte = STOP_KEY;
				break;
			case RC_PLAY:
				key_byte = PLAY_KEY;
				break;
			case RC_MUTE:
				key_byte = MUTE_KEY;
				break;
			case RC_SETUP:
				keyboard_report_buf.keycode[0] = 0;
				keyboard_report_buf.modifier = KEYBOARD_MOD_GUI;
				break;
			case RC_STEP:
				keyboard_report_buf.keycode[0] = KEYBOARD_TAB;
				break;
			case RC_SEARCH:
				keyboard_report_buf.keycode[0] = KEYBOARD_APP;
				break;
			case RC_CLEAR:
				keyboard_report_buf.keycode[0] = KEYBOARD_ESC;
				break;
			case RC_LEFT:
				keyboard_report_buf.keycode[0] = KEYBOARD_LEFT;
				break;
			case RC_RIGHT:
				keyboard_report_buf.keycode[0] = KEYBOARD_RIGHT;
				break;
			case RC_UP:
				keyboard_report_buf.keycode[0] = KEYBOARD_UP;
				break;
			case RC_DOWN:
				keyboard_report_buf.keycode[0] = KEYBOARD_DOWN;
				break;
			case RC_OK:
				keyboard_report_buf.keycode[0] = KEYBOARD_ENTER;
				break;
			case RC_POWER:
				keyboard_report_buf.keycode[0] = KEYBOARD_F4;
				keyboard_report_buf.modifier = KEYBOARD_MOD_ALT;
				break;
			default:
				key_byte = NO_KEYS;
				break;
		}
		
		if(key_byte != NO_KEYS) {  /* Multimedia report to be sent */
			multi_report_buf.key_byte = key_byte;
			send_buf = (uint8_t *)&multi_report_buf;
			bytes_remaining = sizeof(multi_report_buf);
			state = SEND_STATE_MM;
		}
		else if(keyboard_report_buf.keycode[0] || keyboard_report_buf.modifier) { /* Keyboard report to be sent */
			send_buf = (uint8_t *)&keyboard_report_buf;
			bytes_remaining = sizeof(keyboard_report_buf);
			state = SEND_STATE_KB;
		}
	}
}

int __attribute__((noreturn)) main(void)
{
uchar   i;

    wdt_enable(WDTO_1S);
    /* Even if you don't use the watchdog, turn it off here. On newer devices,
     * the status of the watchdog (on/off, period) is PRESERVED OVER RESET!
     */
    /* RESET status: all port bits are inputs without pull-up.
     * That's the way we need D+ and D-. Therefore we don't need any
     * additional hardware initialization.
     */
	rfm70_init(RFM70_MODE_PRX, rfm70_addr);
    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();
	RED_LED_OUT();
    sei();
    for(;;){                /* main event loop */
        wdt_reset();
        usbPoll();
		Receive_Packet();
        if(usbInterruptIsReady()){
			switch(state) {
				case SEND_STATE_MM:					
					usbSetInterrupt(send_buf, bytes_remaining);
					multi_report_buf.key_byte = NO_KEYS; /* Empty the report (no keys pressed) */
					state = SENT_STATE_MM;
					break;
				case SEND_STATE_KB:
					usbSetInterrupt((void *)send_buf, (bytes_remaining > 8)? 8 : bytes_remaining);
					if(bytes_remaining > 8) {
						send_buf += 8;
						bytes_remaining -= 8;
						state = SEND_STATE_KB;
					}
					else {
						send_buf = (uint8_t *)&keyboard_report_buf;
						bytes_remaining = sizeof(keyboard_report_buf);
						keyboard_report_buf.modifier = 0;
						keyboard_report_buf.keycode[0] = 0;
						state = SENT_STATE_KB;
					}
					break;
				case SENT_STATE_MM:
					usbSetInterrupt((void *)&multi_report_buf, sizeof(multi_report_buf));
					state = WAIT_STATE;
					break;
				case SENT_STATE_KB:
					usbSetInterrupt((void *)send_buf, (bytes_remaining > 8)? 8 : bytes_remaining);
					if(bytes_remaining > 8) {
						send_buf += 8;
						bytes_remaining -= 8;
						state = SENT_STATE_KB;
					}
					else {
						state = WAIT_STATE;
					}
					break;
				case WAIT_STATE:
				default:
					break;	
			}
		}
	
    }
}

/* ------------------------------------------------------------------------- */
