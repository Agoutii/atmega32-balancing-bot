#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h> 
#include <math.h>

#include "uart.h"
#include "adc.h"
#include "kalman.h"

// Number of characters to accept on one line
#define LBUFSIZE 32

extern volatile circBuffer *txBuffP, *rxBuffP;
volatile unsigned char kalmanTick;
volatile unsigned char kalmanTimer;
volatile unsigned int dispTimer=1;


ISR(TIMER0_OVF_vect)
{
	if(++kalmanTimer==0) // 28.8/256 = 112.5 Hz. Doesn't set at first cycle.
	{
		kalmanTick++;
/*		if(dispTimer>0)
		{
			if(++dispTimer>1000)
			{
				dispTimer=0;
			}
			else
			{
				//kalman_print();
			}

			
			//adc_print();
		}
		else if(dispTimer>0)
		{
		}*/
	} 

}
void init(void)
{
	//clk 14,745,600 Hz
	// motor PWM prescaler to 1:1=28kHz, 2:1=14kHz

	// Phase correct PWM, (28.798828125)kHz
	
	TCCR0 = (0<<WGM01)|(1<<WGM00)|(1<<COM01)|(1<<CS00);
	TCCR2 = (0<<WGM21)|(1<<WGM20)|(1<<COM21)|(1<<CS20);

	DDRC |= (1<<DDC0)|(1<<DDC1);
	PORTC &= ~(1<<PORTC1);
	PORTC |= (1<<PORTC0);

	DDRD |= (1<<DDD7);
	DDRB |= (1<<DDB3);


	TIMSK |= (1<<TOIE0);

	//TCCR2 = (1<<WGM00);

	//Using timer 0 to generate the 100Hz (ish) tick
	

	//set timers up
	//timer 0 and 2 for PWM, 

}
int main(void)
{
	char c;
	unsigned int bufCount = 0;
	char linebuf[LBUFSIZE]; //Our normal fill & clear buffer for commands
	linebuf[0] = '\0';
	_delay_ms(500);/*stretch*/
	init();
	uart_init(); 	// init USART
	adc_init();
	sei();  		// enable interrupts
//	kalman_print_headers();
	// Wait a second at startup 
	adc_start();

	// Send initial string

	for/*ever*/(;/*and ever*/;)
	{ 
		// Have a kit kat
		//_delay_ms(250);
		if(kalmanTick>0)
		{
			doKalman();
			if(--kalmanTick>3)
				printf_P(PSTR("Missing Calcs!"));
		}
		if(char_avail(rxBuffP)) 
		{ //*ding* You've got mail
			
		c=char_dequeue(rxBuffP); //*yoink* grab the next char out of the buffer
			if (bufCount < LBUFSIZE)
			{
				linebuf[bufCount++] = c;
				//printf_P(PSTR("%02X"), c);
			}
			else
			{
				bufCount = 0;
				linebuf[0] = '\0';
				printf_P(PSTR("\nMax line length exceeded (nice going)\n"));
			}
		//	uart_putc(c); //Send it back (sharing is caring)
			if (c == '\n' || c == '\r') //The beginning of a bright new line
			{ 
				if(bufCount>1)
				{
					char *subStr;
					linebuf[bufCount-1] = '\0';		// terminate the string. Arnie, enter stage left.
					if (!strncmp_P(linebuf,PSTR("help"),4))
					{
						printf_P(PSTR("Mwahahahaha.... not even the gods can help you now >:D\n\r"));
					}
//					else if (!strncmp_P(linebuf,PSTR("print"),4))
//					{
//						kalman_print_headers();
//						dispTimer=1;
//					}
					else if ((subStr=strstr_P(linebuf,PSTR("="))))
					{
						setPVar(linebuf,subStr);
					}
					else if ((strstr_P(linebuf,PSTR("?"))))
					{
						printf_P(PSTR("%G\n"),getPVar(linebuf));
					}
					else
					{
						printf_P(PSTR("Unknown command: \"%s\"\n"),linebuf);
					}
				}
				// reset linebuf
				bufCount = 0;
				linebuf[0] = '\0';
			}
		}
		
	}
	return 0;
}

