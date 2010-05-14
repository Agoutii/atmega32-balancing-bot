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
volatile unsigned char dispTimer;


ISR(TIMER0_OVF_vect)
{
	if(++kalmanTimer==0) // 28.8/256 = 112.5 Hz. Doesn't set at first cycle.
	{
		kalmanTick++;
		if(++dispTimer>12)
		{
			dispTimer=0;
			kalman_print();
			//adc_print();
		}
	}

}
void init(void)
{
	//clk 14,745,600 Hz
	// motor PWM prescaler to 1:1=28kHz, 2:1=14kHz

	// Phase correct PWM, (28.798828125)kHz

	TCCR0 = (1<<WGM00)|(1<<CS00);
	TIMSK |= (1<<TOIE0);


	//TCCR2 = (1<<WGM00);

	//Using timer 0 to generate the 100Hz (ish) tick
	

	//set timers up
	//timer 0 and 2 for PWM, 

}
int main(void)
{
	int x = 0;
	char c;
	unsigned int bufCount = 0;
	char linebuf[LBUFSIZE]; //Our normal fill & clear buffer for commands
	linebuf[0] = '\0';
	_delay_ms(1000);/*stretch*/
	init();
	uart_init(); 	// init USART
	adc_init();
	sei();  		// enable interrupts

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
				printf_P(PSTR("Missing Kalman Calcs!"));
		}
		if(char_avail(rxBuffP)) 
		{ //*ding* You've got mail
			
			c=char_dequeue(rxBuffP); //*yoink* grab the next char out of the buffer
			if (c == '\b')			// backspace (different from frontspace)
			{
				bufCount--;			
			}
			else
			{
				if (bufCount < LBUFSIZE)
				{
					linebuf[bufCount++] = c;
					//printf_P(PSTR("%02X"), c);
				}
				else
				{
					bufCount = 0;
					linebuf[0] = '\0';
					printf_P(PSTR("\n\rMax line length exceeded (nice going)\n\r"));
				}
			}
		//	uart_putc(c); //Send it back (sharing is caring)
			if (c == '\n' || c == '\r') //The beginning of a bright new line
			{ 
				if(bufCount>1)
				{
					linebuf[bufCount-1] = '\0';		// terminate the string. Arnie, enter stage left.
					if (!strncmp_P(linebuf,PSTR("help"),4))
					{
						printf_P(PSTR("Mwahahahaha.... not even the gods can help you now >:D\n\r"));
					}
					else if (!strncmp_P(linebuf,PSTR("x="),2))
					{
						x = atof(&linebuf[2]);
						printf_P(PSTR("x set to %i\n\r"),x);
					}
					else if (!strncmp_P(linebuf,PSTR("x?"),2))
					{
						printf_P(PSTR("x is %i\n\r"),x);
					}
					else
					{
						printf_P(PSTR("Unknown command: \"%s\"\n\r"),linebuf);
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

