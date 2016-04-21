#include <avr/io.h>
#include <stdbool.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */
#include <avr/pgmspace.h>   /* required by usbdrv.h */

#include "usbdrv.h"
#include "HidSensorSpec_1.h"

#define RED_LED					PC0
#define RED_LED_DDR				DDRC
#define RED_LED_PORT			PORTC

#define GREEN_LED				PC1
#define GREEN_LED_DDR			DDRC
#define GREEN_LED_PORT			PORTC


#define RED_LED_OUT()			(RED_LED_DDR |= (1 << RED_LED))
#define RED_LED_OFF()			(RED_LED_PORT |= (1 << RED_LED))
#define RED_LED_ON()			(RED_LED_PORT &= ~(1 << RED_LED))
#define RED_LED_TOGGLE()		(RED_LED_PORT ^= (1 << RED_LED))

#define GREEN_LED_OUT()			(GREEN_LED_DDR |= (1 << GREEN_LED))
#define GREEN_LED_OFF()			(GREEN_LED_PORT |= (1 << GREEN_LED))
#define GREEN_LED_ON()			(GREEN_LED_PORT &= ~(1 << GREEN_LED))
#define GREEN_LED_TOGGLE()		(GREEN_LED_PORT ^= (1 << GREEN_LED))



// 3D Gyrometer
PROGMEM const char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
	HID_USAGE_PAGE_SENSOR,
	HID_USAGE_SENSOR_TYPE_MOTION_GYROMETER_3D,
	HID_COLLECTION(Physical),
		
	//feature reports (xmit/receive)
	HID_USAGE_PAGE_SENSOR,
	
	HID_USAGE_SENSOR_PROPERTY_SENSOR_CONNECTION_TYPE,  // NAry
	HID_LOGICAL_MIN_8(0),
	HID_LOGICAL_MAX_8(2),
	HID_REPORT_SIZE(8),
	HID_REPORT_COUNT(1),
	HID_COLLECTION(Logical),
	HID_USAGE_SENSOR_PROPERTY_CONNECTION_TYPE_PC_INTEGRATED_SEL,
	HID_USAGE_SENSOR_PROPERTY_CONNECTION_TYPE_PC_ATTACHED_SEL_SEL,
	HID_USAGE_SENSOR_PROPERTY_CONNECTION_TYPE_PC_EXTERNAL_SEL_SEL,
	HID_FEATURE(Data_Arr_Abs),
	HID_END_COLLECTION,
	
	HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE,
	HID_LOGICAL_MIN_8(0),
	HID_LOGICAL_MAX_8(5),
			HID_REPORT_SIZE(8),
			HID_REPORT_COUNT(1),
		HID_COLLECTION(Logical),
	HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE_NO_EVENTS_SEL_SEL, 
	HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE_ALL_EVENTS_SEL_SEL,
	HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE_THRESHOLD_EVENTS_SEL_SEL,
	HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE_NO_EVENTS_SEL_WAKE_SEL,
	HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE_ALL_EVENTS_SEL_WAKE_SEL,
	HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE_THRESHOLD_EVENTS_WAKE_SEL_SEL,
			HID_FEATURE(Data_Arr_Abs),
			HID_END_COLLECTION,
	
	HID_USAGE_SENSOR_PROPERTY_POWER_STATE,
	HID_LOGICAL_MIN_8(0),
	HID_LOGICAL_MAX_8(5),
	HID_REPORT_SIZE(8),
	HID_REPORT_COUNT(1),
	HID_COLLECTION(Logical),
	HID_USAGE_SENSOR_PROPERTY_POWER_STATE_UNDEFINED_SEL,           
	HID_USAGE_SENSOR_PROPERTY_POWER_STATE_D0_FULL_POWER_SEL,       
	HID_USAGE_SENSOR_PROPERTY_POWER_STATE_D1_LOW_POWER_SEL,        
	HID_USAGE_SENSOR_PROPERTY_POWER_STATE_D2_STANDBY_WITH_WAKE_SEL,
	HID_USAGE_SENSOR_PROPERTY_POWER_STATE_D3_SLEEP_WITH_WAKE_SEL,  
	HID_USAGE_SENSOR_PROPERTY_POWER_STATE_D4_POWER_OFF_SEL,        
	HID_FEATURE(Data_Arr_Abs),
	HID_END_COLLECTION,
	
	HID_USAGE_SENSOR_STATE,
	HID_LOGICAL_MIN_8(0),
	HID_LOGICAL_MAX_8(6),
	HID_REPORT_SIZE(8),
	HID_REPORT_COUNT(1),
	HID_COLLECTION(Logical),
		HID_USAGE_SENSOR_STATE_UNKNOWN_SEL_SEL,
		HID_USAGE_SENSOR_STATE_READY_SEL_SEL,
		HID_USAGE_SENSOR_STATE_NOT_AVAILABLE_SEL_SEL,
		HID_USAGE_SENSOR_STATE_NO_DATA_SEL_SEL,
		HID_USAGE_SENSOR_STATE_INITIALIZING_SEL_SEL,
		HID_USAGE_SENSOR_STATE_ACCESS_DENIED_SEL_SEL,
		HID_USAGE_SENSOR_STATE_ERROR_SEL_SEL,
	HID_FEATURE(Data_Arr_Abs),
	HID_END_COLLECTION,
	
	HID_USAGE_SENSOR_PROPERTY_REPORT_INTERVAL,
	HID_LOGICAL_MIN_8(0),
	HID_LOGICAL_MAX_32(0xFF,0xFF,0xFF,0xFF),
	HID_REPORT_SIZE(32),
	HID_REPORT_COUNT(1),
	HID_UNIT_EXPONENT(0), 
	HID_FEATURE(Data_Var_Abs),
	    
	HID_USAGE_SENSOR_PROPERTY_MINIMUM_REPORT_INTERVAL,  
    HID_LOGICAL_MIN_32(0x00,0x00,0x00,0x00),
    HID_LOGICAL_MAX_32(0xFF,0xFF,0xFF,0xFF),
    HID_REPORT_SIZE(32),
    HID_REPORT_COUNT(1),
    HID_UNIT_EXPONENT(0),
    HID_FEATURE(Data_Var_Abs),
	
	HID_USAGE_SENSOR_DATA(HID_USAGE_SENSOR_DATA_MOTION_ANGULAR_VELOCITY,HID_USAGE_SENSOR_DATA_MOD_CHANGE_SENSITIVITY_ABS),
	HID_LOGICAL_MIN_8(0),
	HID_LOGICAL_MAX_16(0xFF,0xFF),
	HID_REPORT_SIZE(16),
	HID_REPORT_COUNT(1),
	HID_UNIT_EXPONENT(0x0E), // scale default unit to provide 2 digits past decimal point
	HID_FEATURE(Data_Var_Abs),
	
	HID_USAGE_SENSOR_DATA(HID_USAGE_SENSOR_DATA_MOTION_ANGULAR_VELOCITY,HID_USAGE_SENSOR_DATA_MOD_MAX),
	HID_LOGICAL_MIN_16(0x01,0x80), //    LOGICAL_MINIMUM (-32767)
	HID_LOGICAL_MAX_16(0xFF,0x7F), //    LOGICAL_MAXIMUM (32767)
	HID_REPORT_SIZE(16), 
	HID_REPORT_COUNT(1), 
	HID_UNIT_EXPONENT(0x0E), // scale default unit to provide 2 digits past decimal point
	HID_FEATURE(Data_Var_Abs),
	
	HID_USAGE_SENSOR_DATA(HID_USAGE_SENSOR_DATA_MOTION_ANGULAR_VELOCITY,HID_USAGE_SENSOR_DATA_MOD_MIN),
	HID_LOGICAL_MIN_16(0x01,0x80), //    LOGICAL_MINIMUM (-32767)
	HID_LOGICAL_MAX_16(0xFF,0x7F), //    LOGICAL_MAXIMUM (32767)
	HID_REPORT_SIZE(16), 
	HID_REPORT_COUNT(1), 
	HID_UNIT_EXPONENT(0x0E), // scale default unit to provide 2 digits past decimal point
	HID_FEATURE(Data_Var_Abs),
		
	//input reports (transmit)
	HID_USAGE_PAGE_SENSOR,
	
	HID_USAGE_SENSOR_STATE,
	HID_LOGICAL_MIN_8(0),
	HID_LOGICAL_MAX_8(6),
			HID_REPORT_SIZE(8),
			HID_REPORT_COUNT(1),
	HID_COLLECTION(Logical),
			HID_USAGE_SENSOR_STATE_UNKNOWN_SEL_SEL,
			HID_USAGE_SENSOR_STATE_READY_SEL_SEL,
			HID_USAGE_SENSOR_STATE_NOT_AVAILABLE_SEL_SEL,
			HID_USAGE_SENSOR_STATE_NO_DATA_SEL_SEL,
			HID_USAGE_SENSOR_STATE_INITIALIZING_SEL_SEL,
			HID_USAGE_SENSOR_STATE_ACCESS_DENIED_SEL_SEL,
			HID_USAGE_SENSOR_STATE_ERROR_SEL_SEL,
	HID_INPUT(Data_Arr_Abs),
	HID_END_COLLECTION,
	
	HID_USAGE_SENSOR_EVENT,
	HID_LOGICAL_MIN_8(0),
	HID_LOGICAL_MAX_8(5),
			HID_REPORT_SIZE(8),
			HID_REPORT_COUNT(1),
		HID_COLLECTION(Logical),
	HID_USAGE_SENSOR_EVENT_UNKNOWN_SEL_SEL,
	HID_USAGE_SENSOR_EVENT_STATE_CHANGED_SEL_SEL,
	HID_USAGE_SENSOR_EVENT_PROPERTY_CHANGED_SEL_SEL,
	HID_USAGE_SENSOR_EVENT_DATA_UPDATED_SEL_SEL,
	HID_USAGE_SENSOR_EVENT_POLL_RESPONSE_SEL_SEL,
	HID_USAGE_SENSOR_EVENT_CHANGE_SENSITIVITY_SEL_SEL,
	HID_INPUT(Data_Arr_Abs),
			HID_END_COLLECTION,
	
	HID_USAGE_SENSOR_DATA_MOTION_ANGULAR_VELOCITY_X_AXIS,
	HID_LOGICAL_MIN_16(0x01,0x80), //    LOGICAL_MINIMUM (-32767)
	HID_LOGICAL_MAX_16(0xFF,0x7F), //    LOGICAL_MAXIMUM (32767)
	HID_REPORT_SIZE(16), 
	HID_REPORT_COUNT(1), 
	HID_UNIT_EXPONENT(0x0E), // scale default unit to provide 2 digits past decimal point
	HID_INPUT(Data_Var_Abs),
	
	HID_USAGE_SENSOR_DATA_MOTION_ANGULAR_VELOCITY_Y_AXIS,
	HID_LOGICAL_MIN_16(0x01,0x80), //    LOGICAL_MINIMUM (-32767)
	HID_LOGICAL_MAX_16(0xFF,0x7F), //    LOGICAL_MAXIMUM (32767)
	HID_REPORT_SIZE(16), 
	HID_REPORT_COUNT(1), 
	HID_UNIT_EXPONENT(0x0E), // scale default unit to provide 2 digits past decimal point
	HID_INPUT(Data_Var_Abs),
	
	HID_USAGE_SENSOR_DATA_MOTION_ANGULAR_VELOCITY_Z_AXIS,
	HID_LOGICAL_MIN_16(0x01,0x80), //    LOGICAL_MINIMUM (-32767)
	HID_LOGICAL_MAX_16(0xFF,0x7F), //    LOGICAL_MAXIMUM (32767)
	HID_REPORT_SIZE(16), 
	HID_REPORT_COUNT(1), 
	HID_UNIT_EXPONENT(0x0E), // scale default unit to provide 2 digits past decimal point
	HID_INPUT(Data_Var_Abs),

	HID_END_COLLECTION
};

typedef struct _GYRO3_FEATURE_REPORT
{
    //common properties
    //uint8_t   ucReportId;
    uint8_t   ucConnectionType;
    uint8_t   ucReportingState;
    uint8_t   ucPowerState;
    uint8_t   ucSensorState;
    uint32_t   ulReportInterval;
	uint32_t   ulMinReportInterval;
	
    //properties specific to this sensor
    uint16_t  usGyroChangeSensitivity;
    uint16_t   sGyroMaximum;
    uint16_t   sGyroMinimum;

} GYRO3_FEATURE_REPORT, *PGYRO3_FEATURE_REPORT;

typedef struct _GYRO3_INPUT_REPORT
{
    //common values
    //HID_UCHAR   ucReportId;
    uint8_t   ucSensorState;
    uint8_t   ucEventType;

    //values specific to this sensor
    uint16_t   sGyroXValue;
    uint16_t   sGyroYValue;
    uint16_t   sGyroZValue;

} GYRO3_INPUT_REPORT, *PGYRO3_INPUT_REPORT;




static GYRO3_FEATURE_REPORT	feature_report_buf = {
	//1,  // Report ID
	HID_USAGE_SENSOR_PROPERTY_CONNECTION_TYPE_PC_ATTACHED_SEL_ENUM,
	HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE_NO_EVENTS_SEL_ENUM,
	HID_USAGE_SENSOR_PROPERTY_POWER_STATE_D0_FULL_POWER_ENUM,
	HID_USAGE_SENSOR_STATE_READY_SEL_ENUM,
	1000, /* reporting interval (ms)*/
	100, /* Min reporting interval (ms)*/
	
	10,    /* Change sensitivity = 0.1 */
	1000,	/* Max  = 10 */
	0,		/* Min = 0 */
};	
	
static GYRO3_INPUT_REPORT	input_report_buf = {
	//1,  // Report ID
	HID_USAGE_SENSOR_STATE_READY_SEL_ENUM,
	HID_USAGE_SENSOR_EVENT_POLL_RESPONSE_SEL_ENUM,
	
	1,	// x = 0.01
	2, 	// y = 0.02
	3	// z = 0.03
};

uint8_t *write_buf = (void*)0;
uint16_t bytesRemaining;
bool get_input_requested = false;

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  
			/* wValue: ReportType (highbyte), ReportID (lowbyte) */
            if(rq->wValue.bytes[1] == 0x03) { /* Get Feature report */
				usbMsgPtr = (void *)&feature_report_buf;
				return sizeof(feature_report_buf);
			}
			else if (rq->wValue.bytes[1] == 0x01) { /* Get Input report */
				get_input_requested = true;
			}
        }else if(rq->bRequest == USBRQ_HID_SET_REPORT){
			if(rq->wValue.bytes[1] == 0x03) { /* Set Feature report */
				
				bytesRemaining = rq->wLength.word;
				write_buf = (void*)&feature_report_buf;
				if(rq->wLength.bytes[0] == (sizeof(feature_report_buf))) {
					GREEN_LED_TOGGLE();
				}
				if(bytesRemaining > sizeof(feature_report_buf)) {
					bytesRemaining = sizeof(feature_report_buf);
				}
			}
			return USB_NO_MSG; /* Use usbFuntionWrite() to get data from host */
        }
    }else{
        /* no vendor specific requests implemented */
    }
    return 0;   /* default for not implemented requests: return no data back to host */
}


uchar   usbFunctionWrite(uchar *data, uchar len)
{
    uint8_t  i;
	if(len > bytesRemaining)
        len = bytesRemaining;
	for(i = 0; i < len; i++)
		write_buf[i] = data[i];
	write_buf += len;
    bytesRemaining -= len;
    return bytesRemaining == 0; /* return 1 if this was the last chunk */
}

int main(void)
{
	uint8_t i;
	bool timeout = false;
	
	RED_LED_OUT();
	GREEN_LED_OUT();
	RED_LED_OFF();
	GREEN_LED_OFF();
	
	usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();
    sei();
	
	for(;;){                /* main event loop */
        wdt_reset();
        usbPoll();
        if((get_input_requested == true) || ((timeout == true) && (feature_report_buf.ucReportingState != 
											 HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE_NO_EVENTS_SEL_ENUM)) ) {
			if(get_input_requested == true) {
				get_input_requested = false;
				input_report_buf.ucEventType = HID_USAGE_SENSOR_EVENT_POLL_RESPONSE_SEL_ENUM;
			}
			else if(timeout == true) {
				timeout = false;
				input_report_buf.ucEventType = HID_USAGE_SENSOR_EVENT_DATA_UPDATED_SEL_ENUM;
				input_report_buf.sGyroXValue++;
			}
			RED_LED_TOGGLE();
            /* called after every poll of the interrupt endpoint */
            usbSetInterrupt((void *)&input_report_buf, sizeof(input_report_buf));
        }
		_delay_ms(2);
		if(i++ == 0) {
			timeout = true;
		}
    }
}
