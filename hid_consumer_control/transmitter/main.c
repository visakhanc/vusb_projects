#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "ir_nec.h"
#include "rfm70.h"

#define RED_LED					PB0
#define RED_LED_DDR				DDRB
#define RED_LED_PORT			PORTB

#define RED_LED_OUT()			(RED_LED_DDR |= (1 << RED_LED))
#define RED_LED_ON()			(RED_LED_PORT |= (1 << RED_LED))
#define RED_LED_OFF()			(RED_LED_PORT &= ~(1 << RED_LED))
#define RED_LED_TOGGLE()		(RED_LED_PORT ^= (1 << RED_LED))

static rc_code_t  	code;
static uint8_t 		rfm70_addr[CONFIG_RFM70_ADDR_LEN] = CONFIG_RFM70_ADDRESS;
static uint8_t		tx_buf[8];

int main(void)
{
	int8_t i;
	
	rc_init();
	rfm70_init(RFM70_MODE_PTX, rfm70_addr);
	RED_LED_OUT();
	RED_LED_OFF();
	sei();
	
	while(1) {
		rc_get_code(&code);
		//RED_LED_TOGGLE();
		
		tx_buf[0] = code.addr;
		tx_buf[1] = code.data;
#if 0		
		for(i = 7; i >= 0; i--)
		{
			if(code.data & (1 << i)) {
				RED_LED_ON();
				_delay_ms(50);
				RED_LED_OFF();
				_delay_ms(50);
			}
			RED_LED_ON();
			_delay_ms(50);
			RED_LED_OFF();
			_delay_ms(50);
				
			_delay_ms(500);
		}
#endif
	
		if(0 == rfm70_transmit_packet(tx_buf,sizeof(tx_buf))) {
			RED_LED_TOGGLE();
		}	

	
		_delay_ms(70);
	
	}
}

