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



#define RED_LED					PC1
#define RED_LED_DDR				DDRC
#define RED_LED_PORT			PORTC

#define RED_LED_OUT()			(RED_LED_DDR |= (1 << RED_LED))
#define RED_LED_ON()			(RED_LED_PORT |= (1 << RED_LED))
#define RED_LED_OFF()			(RED_LED_PORT &= ~(1 << RED_LED))
#define RED_LED_TOGGLE()		(RED_LED_PORT ^= (1 << RED_LED))

#define 	SEND				1
#define 	NO_KEYS				0

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

/* Keyboard report descriptor*/
PROGMEM const char usbHidReportDescriptor[63] = { /* USB report descriptor, size must match usbconfig.h */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
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
};

/* Keyboard report */
typedef struct{
    uint8_t modifier;
	uint8_t reserved;
	uint8_t keys[6];
}report_t;

static report_t reportBuffer;
static uchar    idleRate;   /* repeat rate for keyboards, never used for mice */
static uint8_t	ledState;

static uint8_t state = NO_KEYS;
static uint16_t counter;
/* ------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    /* The following requests are never used. But since they are required by
     * the specification, we implement them in this example.
     */
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one report type, so don't look at wValue */
			reportBuffer.modifier = 0; /* Send empty report (no keys pressed) */
			reportBuffer.keys[0] = 0;
            usbMsgPtr = (void *)&reportBuffer;
            return sizeof(reportBuffer);
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
        if(usbInterruptIsReady()){
			if(state == SEND) {
				reportBuffer.keys[0] = 0x52; /* 'UP' */
				//reportBuffer.modifier |= (1 << 2); /* SHIFT */
				state = NO_KEYS;
				counter = 0;
			}
			usbSetInterrupt((void *)&reportBuffer, sizeof(reportBuffer));
			reportBuffer.keys[0] = 0; /* Empty the report (no keys pressed) */
			reportBuffer.modifier = 0;
		}
		counter++;
		if(counter == 30000) {
			state = SEND;
		}
    }
}

/* ------------------------------------------------------------------------- */
