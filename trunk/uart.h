#ifndef UART_H
#define UART_H

#include <avr/io.h>
#include <stdio.h>
#include <ctype.h> 
#include <avr/interrupt.h>
#include <string.h>
#include "circBuffer.h"


#define UART_BAUD_RATE 38400

//#define UART_DOUBLESPEED

#ifdef UART_DOUBLESPEED
  #define UART_CALC_BAUDRATE(baudRate) ((uint32_t)((F_CPU) + ((uint32_t)baudRate * 4UL)) / ((uint32_t)(baudRate) * 8UL) - 1)
#else
  #define UART_CALC_BAUDRATE(baudRate) ((uint32_t)((F_CPU) + ((uint32_t)baudRate * 8UL)) / ((uint32_t)(baudRate) * 16UL) - 1)
#endif

#ifndef BUFSIZE
  #define BUFSIZE 80
#endif



/////Special functions for adding to the Tx buffer (the interupt needs playing with)
int uart_putc(char c);
int uart_printchar(char c, FILE *stream);
int uart_puts (char *s);
////////////////////////////////////////////
void uart_init(void);



#endif
