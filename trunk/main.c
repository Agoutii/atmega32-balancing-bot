//Kel AVR buffer test 04/10
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h> 

#include "uart.h"

// Number of characters to accept on one line
#define LBUFSIZE 32

extern volatile circBuffer *txBuffP, *rxBuffP;

ISR(USART_RXC_vect)			// USART RX interrupt
{
	if(!char_queue(UDR, rxBuffP))
	{ //Oh noes, receive buffer full... we broke it :(
		printf_P(PSTR("Zee buffer, it does nahsink"));
	}
		
}

ISR(USART_UDRE_vect) // USART transmit buffer empty interrupt
{
	if (char_avail(txBuffP))
	{
		// If we have a char waiting, send it on it's merry way.
		UDR = char_dequeue(txBuffP);
	}
	else
	{
		// Otherwise disable interrupt so we don't immediately retrigger forever (and where's the fun in that?)
		UCSRB &= ~(1 << UDRIE);
	}
}



int main(void)
{
	int x = 0;
	char c;
	unsigned int bufCount = 0;
	char linebuf[LBUFSIZE]; //Our normal fill & clear buffer for commands
	linebuf[0] = '\0';
	
	uart_init(); 	// init USART
	sei();  		// enable interrupts

	// Wait a second at startup 
	_delay_ms(1000);/*stretch*/

	// Send initial string
	printf_P(PSTR("What is your bidding, master?\n\r"));

	for/*ever*/(;/*and ever*/;)
	{ 
		// Have a kit kat
		//_delay_ms(250);
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
			uart_putc(c); //Send it back (sharing is caring)
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

