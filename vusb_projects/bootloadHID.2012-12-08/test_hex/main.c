#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define RED_LED					PC0
#define RED_LED_DDR				DDRC
#define RED_LED_PORT			PORTC

#define RED_LED_OUT()			(RED_LED_DDR |= (1 << RED_LED))
#define RED_LED_ON()			(RED_LED_PORT |= (1 << RED_LED))
#define RED_LED_OFF()			(RED_LED_PORT &= ~(1 << RED_LED))
#define RED_LED_TOGGLE()		(RED_LED_PORT ^= (1 << RED_LED))



int main(void)
{	
	
	RED_LED_OUT();
	RED_LED_OFF();
	
	while(1)
	{
		RED_LED_ON();
		_delay_ms(500);
		RED_LED_OFF();
		_delay_ms(500);
	}
}

