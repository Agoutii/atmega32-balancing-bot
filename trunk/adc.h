#ifndef ADC_INIT
#define ADC_INIT

#include <avr/io.h>
#include <stdio.h>
#include <ctype.h> 
#include <avr/interrupt.h>
#include <math.h>

void adc_init(void);
void adc_start(void);
char adc_fetch(char adc_chan);
#endif
