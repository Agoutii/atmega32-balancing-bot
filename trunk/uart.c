#include "uart.h"
////// Should these be in uart.h?//////
volatile circBuffer txBuff, rxBuff;
volatile circBuffer *txBuffP=&txBuff, *rxBuffP=&rxBuff;
///////////////////////////////////////

// Define and initialise stream used for printing to uart
FILE uart_str = FDEV_SETUP_STREAM(uart_printchar, NULL, _FDEV_SETUP_RW);


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

int uart_putc(char c)
{
	// Disable UDRE interrupt
	UCSRB &= ~(1 << UDRIE);

	// Queue up the character
	int result = char_queue(c, txBuffP);

	// Enable UDRE interrupt
	UCSRB |= (1 << UDRIE);
	return result;
}

int uart_printchar(char c, FILE *stream)
{
	uart_putc(c);

	return 0;
	//////// how to return an error (e.g buffer full)? Is this possible to the stream?
}

int uart_puts (char *s) //Add a string to the transmit buffer.
{	//If this gets called in an interupt, may need a UDRIE check to make sure an instruction doesn't injected.
	//If the string isn't going to fit, break now before half an instruction is queued.
	if(strlen(s) > (BUFSIZE - char_avail(txBuffP))) return 0; 

	// Disable UDRE interrupt, so the buffer doesn't get messed with.
	UCSRB &= ~(1 << UDRIE);

	// Loop until end of string (must be null terminated)
	while (*s)
	{
		if(!char_queue(*s++, txBuffP)) //Check anyway, just in case
		{
			//Oh noes! Buffer full.... somehow..... no point trying to send the rest
			UCSRB |= (1 << UDRIE);
			return 0;
		}
	}

	// String buffered successfully - enable UDRE interrupt and return
	UCSRB |= (1 << UDRIE);
	return 1;
}

void uart_init(void)
{
	// Set baud rate
	UBRRH = (uint8_t)(UART_CALC_BAUDRATE(UART_BAUD_RATE)>>8);
	UBRRL = (uint8_t)UART_CALC_BAUDRATE(UART_BAUD_RATE);

#ifdef UART_DOUBLESPEED
	UCSRA = (1<<U2X);
#endif

	// Enable receiver and transmitter; enable RX interrupt
	UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);

	// Asynchronous 8N1
	UCSRC = (1 << URSEL) | (3 << UCSZ0);
//	txBuffP = &txBuff;
//	rxBuffP = &rxBuff;
	
	// Redirect stdout, so that printf() knows where to go
	stdout = &uart_str;
}

	
