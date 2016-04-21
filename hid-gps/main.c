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


 PROGMEM const char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
	HID_USAGE_PAGE_SENSOR, 
	HID_USAGE_SENSOR_TYPE_LOCATION_GPS, 
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
		HID_FEATURE(Const_Arr_Abs), 
		HID_END_COLLECTION,
		
	HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE, 
	HID_LOGICAL_MIN_8(0), 
	HID_LOGICAL_MAX_8(5), 
    HID_REPORT_SIZE(8), 
    HID_REPORT_COUNT(1), 
	HID_COLLECTION(Logical), 
		HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE_NO_EVENTS_SEL_ENUM,
		HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE_ALL_EVENTS_SEL_ENUM,
		HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE_THRESHOLD_EVENTS_SEL_ENUM,
		HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE_NO_EVENTS_SEL_WAKE_ENUM,
		HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE_ALL_EVENTS_SEL_WAKE_ENUM,       
		HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE_THRESHOLD_EVENTS_WAKE_SEL_ENUM, 
		HID_FEATURE(Data_Arr_Abs), 
		HID_END_COLLECTION, 
  
	HID_USAGE_SENSOR_PROPERTY_SENSOR_STATUS, 
	HID_LOGICAL_MIN_8(0), 
	HID_LOGICAL_MAX_32(0xFF,0xFF,0xFF,0xFF), 
	HID_REPORT_SIZE(32), 
	HID_REPORT_COUNT(1), 
	HID_FEATURE(Data_Var_Abs), // up to VT_UI4 worth of status info 

	HID_USAGE_SENSOR_PROPERTY_REPORT_INTERVAL, 
	HID_LOGICAL_MIN_8(0), 
	HID_LOGICAL_MAX_32(0xFF,0xFF,0xFF,0xFF), 
	HID_REPORT_SIZE(32), 
	HID_REPORT_COUNT(1), 
	// HID_USAGE_SENSOR_UNITS_MILLISECOND, 
	HID_UNIT_EXPONENT(0),  
	HID_FEATURE(Data_Var_Abs), 
	
	HID_USAGE_SENSOR_PROPERTY_MINIMUM_REPORT_INTERVAL,  
    HID_LOGICAL_MIN_8(0),
    HID_LOGICAL_MAX_32(0xFF,0xFF,0xFF,0xFF),
    HID_REPORT_SIZE(32),
    HID_REPORT_COUNT(1),
    HID_UNIT_EXPONENT(0),
    HID_FEATURE(Data_Var_Abs),
	
	HID_USAGE_SENSOR_STATE, 
	HID_LOGICAL_MIN_8(0), 
	HID_LOGICAL_MAX_8(6), 
	HID_REPORT_SIZE(8), 
	HID_REPORT_COUNT(1), 
	HID_COLLECTION(Logical), 
		HID_USAGE_SENSOR_STATE_UNKNOWN_SEL_ENUM,        
		HID_USAGE_SENSOR_STATE_READY_SEL_ENUM,          
		HID_USAGE_SENSOR_STATE_NOT_AVAILABLE_SEL_ENUM,  
		HID_USAGE_SENSOR_STATE_NO_DATA_SEL_ENUM,        
		HID_USAGE_SENSOR_STATE_INITIALIZING_SEL_ENUM,   
		HID_USAGE_SENSOR_STATE_ACCESS_DENIED_SEL_ENUM,  
		HID_USAGE_SENSOR_STATE_ERROR_SEL_ENUM,          
		HID_FEATURE(Const_Arr_Abs), 
		HID_END_COLLECTION, 

	HID_USAGE_SENSOR_PROPERTY_CHANGE_SENSITIVITY_ABS, 
	HID_LOGICAL_MIN_8(0), 
	HID_LOGICAL_MAX_16(0xFF,0xFF), 
	HID_REPORT_SIZE(16), 
	HID_REPORT_COUNT(1), 
	  // HID_USAGE_SENSOR_UNITS_METER, 
	HID_UNIT_EXPONENT(0x0E), // scale default unit “meter” to provide 2 digits past the decimal point 
	HID_FEATURE(Data_Var_Abs), 

	//input reports (transmit) 
	HID_USAGE_PAGE_SENSOR, 
	HID_USAGE_SENSOR_STATE, 
	HID_LOGICAL_MIN_8(0), 
	HID_LOGICAL_MAX_8(6), 
	HID_REPORT_SIZE(8), 
	HID_REPORT_COUNT(1), 
	HID_COLLECTION(Logical), 
		HID_USAGE_SENSOR_STATE_UNKNOWN_SEL_ENUM,        
		HID_USAGE_SENSOR_STATE_READY_SEL_ENUM,          
		HID_USAGE_SENSOR_STATE_NOT_AVAILABLE_SEL_ENUM,  
		HID_USAGE_SENSOR_STATE_NO_DATA_SEL_ENUM,        
		HID_USAGE_SENSOR_STATE_INITIALIZING_SEL_ENUM,   
		HID_USAGE_SENSOR_STATE_ACCESS_DENIED_SEL_ENUM,  
		HID_USAGE_SENSOR_STATE_ERROR_SEL_ENUM,          
		HID_INPUT(Const_Arr_Abs), 
		HID_END_COLLECTION, 
    
	HID_USAGE_SENSOR_EVENT, 
	HID_LOGICAL_MIN_8(0), 
	HID_LOGICAL_MAX_8(16), 
    HID_REPORT_SIZE(8), 
    HID_REPORT_COUNT(1), 
	HID_COLLECTION(Logical), 
		HID_USAGE_SENSOR_EVENT_UNKNOWN_SEL_ENUM, 
		HID_USAGE_SENSOR_EVENT_STATE_CHANGED_SEL_ENUM, 
		HID_USAGE_SENSOR_EVENT_PROPERTY_CHANGED_SEL_ENUM, 
		HID_USAGE_SENSOR_EVENT_DATA_UPDATED_SEL_ENUM, 
		HID_USAGE_SENSOR_EVENT_POLL_RESPONSE_SEL_ENUM, 
		HID_USAGE_SENSOR_EVENT_CHANGE_SENSITIVITY_SEL_ENUM, 
		HID_USAGE_SENSOR_EVENT_MAX_REACHED_ENUM, 
		HID_USAGE_SENSOR_EVENT_MIN_REACHED_ENUM, 
		HID_USAGE_SENSOR_EVENT_HIGH_THRESHOLD_CROSS_UPWARD_ENUM, 
		HID_USAGE_SENSOR_EVENT_HIGH_THRESHOLD_CROSS_DOWNWARD_ENUM, 
		HID_USAGE_SENSOR_EVENT_LOW_THRESHOLD_CROSS_UPWARD_ENUM, 
		HID_USAGE_SENSOR_EVENT_LOW_THRESHOLD_CROSS_DOWNWARD_ENUM, 
		HID_USAGE_SENSOR_EVENT_ZERO_THRESHOLD_CROSS_UPWARD_ENUM, 
		HID_USAGE_SENSOR_EVENT_ZERO_THRESHOLD_CROSS_DOWNWARD_ENUM, 
		HID_USAGE_SENSOR_EVENT_PERIOD_EXCEEDED_ENUM, 
		HID_USAGE_SENSOR_EVENT_FREQUENCY_EXCEEDED_ENUM, 
		HID_USAGE_SENSOR_EVENT_COMPLEX_TRIGGER_ENUM, 
		HID_INPUT(Const_Arr_Abs), 
		HID_END_COLLECTION, 
		
	HID_USAGE_SENSOR_DATA_LOCATION_LATITUDE, 
	HID_LOGICAL_MIN_32(0xFF,0xFF,0x01,0x00), // LOGICAL_MINIMUM (-2147483647) 
	HID_LOGICAL_MAX_32(0xFF,0x7F,0xFF,0xFF), // LOGICAL_MAXIMUM (2147483647) 
	HID_REPORT_SIZE(32),  
	HID_REPORT_COUNT(1),  
	//HID_USAGE_SENSOR_UNITS_DEGREES, 
	HID_UNIT_EXPONENT(0x09), // scale unit to provide 7 digits past the decimal point 
	HID_INPUT(Const_Var_Abs), 
	
	HID_USAGE_SENSOR_DATA_LOCATION_LONGITUDE, 
	HID_LOGICAL_MIN_32(0xFF,0xFF,0x01,0x00), // LOGICAL_MINIMUM (-2147483647) 
	HID_LOGICAL_MAX_32(0xFF,0x7F,0xFF,0xFF), // LOGICAL_MAXIMUM (2147483647) 
	HID_REPORT_SIZE(32),  
	HID_REPORT_COUNT(1),  
	//HID_USAGE_SENSOR_UNITS_DEGREES, 
	HID_UNIT_EXPONENT(0x09), // scale unit to provide 7 digits past the decimal point 
	HID_INPUT(Const_Var_Abs), 
  
	HID_USAGE_SENSOR_DATA_LOCATION_ERROR_RADIUS, 
	HID_LOGICAL_MIN_8(0), 
	HID_LOGICAL_MAX_32(0xFF,0xFF,0xFF,0xFF), 
	HID_REPORT_SIZE(32),  
	HID_REPORT_COUNT(1),  
	// HID_USAGE_SENSOR_UNITS_METER, 
	HID_UNIT_EXPONENT(0x09), // scale default unit “meter” to provide 7 digits past the decimal point 
	HID_INPUT(Const_Var_Abs), 
	
	HID_USAGE_SENSOR_DATA_LOCATION_ALTITUDE_SEALEVEL, 
	HID_LOGICAL_MIN_32(0xFF,0xFF,0x01,0x00), // LOGICAL_MINIMUM (-2147483647) 
	HID_LOGICAL_MAX_32(0xFF,0x7F,0xFF,0xFF), // LOGICAL_MAXIMUM (2147483647) 
	HID_REPORT_SIZE(32),  
	HID_REPORT_COUNT(1),  
	//HID_USAGE_SENSOR_UNITS_DEGREES, 
	HID_UNIT_EXPONENT(0x0E), // scale unit to provide 2 digits past the decimal point 
	HID_INPUT(Const_Var_Abs),
 
	HID_USAGE_SENSOR_DATA_LOCATION_ALTITUDE_SEALEVEL_ERROR, 
	HID_LOGICAL_MIN_8(0), 
	HID_LOGICAL_MAX_32(0xFF,0xFF,0xFF,0xFF), 
	HID_REPORT_SIZE(32),  
	HID_REPORT_COUNT(1),  
	//HID_USAGE_SENSOR_UNITS_DEGREES, 
	HID_UNIT_EXPONENT(0x0E), // scale unit to provide 2 digits past the decimal point 
	HID_INPUT(Const_Var_Abs), 
 
	HID_END_COLLECTION 
}; 



typedef struct _GPS_FEATURE_REPORT
{
    //uint8_t   ucReportId;
	uint8_t ucConnectionType;// HID_USAGE_SENSOR_PROPERTY_SENSOR_CONNECTION_TYPE
	uint8_t ucReportingState;	//HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE
	uint32_t ulSensorStatus;	//HID_USAGE_SENSOR_PROPERTY_SENSOR_STATUS
	uint32_t ulReportInterval;	//HID_USAGE_SENSOR_PROPERTY_REPORT_INTERVAL
	uint32_t ulMinReportInterval; 
	uint8_t   ucSensorState;	// HID_USAGE_SENSOR_STATE
	uint16_t  usTempChangeSensitivity; // HID_USAGE_SENSOR_PROPERTY_CHANGE_SENSITIVITY_ABS
	
} GPS_FEATURE_REPORT, *PGPS_FEATURE_REPORT;

typedef struct _GPS_INPUT_REPORT
{
    //uint8_t   ucReportId;
	uint8_t   ucSensorState;	// HID_USAGE_SENSOR_STATE
    uint8_t   ucEventType;		// HID_USAGE_SENSOR_EVENT
	
	uint32_t ulLocationLatitude; // HID_USAGE_SENSOR_DATA_LOCATION_LATITUDE
	uint32_t ulLocationLongitude; // HID_USAGE_SENSOR_DATA_LOCATION_LONGITUDE
	uint32_t ulErrorRadius; // HID_USAGE_SENSOR_DATA_LOCATION_ERROR_RADIUS
	uint32_t ulAltitude; // HID_USAGE_SENSOR_DATA_LOCATION_ALTITUDE_SEALEVEL
	uint32_t ulAltitudeError; // HID_USAGE_SENSOR_DATA_LOCATION_ALTITUDE_SEALEVEL_ERROR

} GPS_INPUT_REPORT, *PGPS_INPUT_REPORT;


static GPS_FEATURE_REPORT	feature_report_buf = {
	//1,  // Report ID
	HID_USAGE_SENSOR_PROPERTY_CONNECTION_TYPE_PC_ATTACHED_SEL_ENUM,
	HID_USAGE_SENSOR_PROPERTY_REPORTING_STATE_NO_EVENTS_SEL_ENUM,
	1,  // Sensor Status
	1000, /* reporting interval (ms)*/
	1000, /* Minimum reporting interval */
	HID_USAGE_SENSOR_STATE_READY_SEL_ENUM,
	10
};	
	
static GPS_INPUT_REPORT	input_report_buf = {
	//1,  // Report ID
	HID_USAGE_SENSOR_STATE_READY_SEL_ENUM,
	HID_USAGE_SENSOR_EVENT_POLL_RESPONSE_SEL_ENUM,
	
	129273600, 	// latitude x1e-7
	776072900, 	// longitude x1e-7
	50000000, 	// error radius x1e-7
	90010,		// Altitude x0.01
	1000		// Altitude error x0.01
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
				GREEN_LED_TOGGLE();
				usbMsgPtr = (void *)&feature_report_buf;
				return sizeof(feature_report_buf);
			}
			else if (rq->wValue.bytes[1] == 0x01) { /* Get Input report */
				get_input_requested = true;
			}
			else {
				RED_LED_TOGGLE();
			}
        }else if(rq->bRequest == USBRQ_HID_SET_REPORT){
			if(rq->wValue.bytes[1] == 0x03) { /* Set Feature report */
				RED_LED_TOGGLE();
				bytesRemaining = rq->wLength.word;
				write_buf = (void*)&feature_report_buf;
				if(rq->wLength.bytes[0] == (sizeof(feature_report_buf))) {
					//GREEN_LED_TOGGLE();
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
			}
			//RED_LED_TOGGLE();
            /* called after every poll of the interrupt endpoint */
            usbSetInterrupt((void *)&input_report_buf, sizeof(input_report_buf));
        }
		_delay_ms(2);
		if(i++ == 0) {
			timeout = true;
		}
    }
}
