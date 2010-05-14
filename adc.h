#ifndef ADC_INIT
#define ADC_INIT

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h> 
#include <math.h>

void adc_init(void);
void adc_start(void);
int adc_fetch(int adc_chan);
void adc_print(void);
#endif
