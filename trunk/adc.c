#include "adc.h"


static const unsigned char adc_enable = (1<<4)|(1<<2)|(1<<1); //enabled adc channels in binary (bit 0 = chan 0, bit 1 = chan 1 etc)

static const unsigned char adc_size=5; //for chan 0 only adc_size = 1, for chan 1 only adc_size = 2 etc

volatile unsigned int adc_result[5];  //total number of elements = highest index +1

//const unsigned char adc_peice_count = sizeof(adc_enable)/sizeof(adc_enable[0]);

//char str[] = PSTR("hello world");


ISR(ADC_vect)
{ //Only sampling desired channels may not be needed... not sure how it would interfere with ADC pins set to other things otherwise.
	unsigned int adc_count = (ADMUX&(7<<MUX0));
	// ADIF is cleared automatically if the interupt is enabled
	adc_result[adc_count] = ADCL;
	adc_result[adc_count] += (ADCH<<8); //Put ADC result in adc_result[chan]
	ADMUX &= ~(7<<MUX0); //clear MUX;
	//find the next enabled channel - THIS WILL LOCK IF NONE ARE ENABLED!
	// ADC *cannot* be started or used without using the adc_enable to poll.
	// adc_init will not start the adc loop if no channels are enabled.
	while(!( (adc_count = (adc_count+1)%adc_size)&&(adc_enable|(1<<adc_count)) )){} 
	ADMUX |= ((adc_count)<<MUX0); //Set MUX
	ADCSRA |= (1<<ADSC); //Start the next conversion
	//printf_P(PSTR("ADC conversion done!:"));
}

void adc_init(void)
{
	ADMUX = (0<<ADLAR);
	ADCSRA = (1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1);

}
void adc_start(void) 
{//if no channels are enabled this should not start the ADC loop.
	unsigned char adc_count = 0;

	ADMUX &= ~(7<<MUX0); //clear MUX;
	while(++adc_count<adc_size)
	{	
		if((adc_enable|(1<<adc_count)))
		{
			ADMUX |= ((adc_count)<<MUX0); //Set MUX
			ADCSRA |= (1<<ADSC); //Start the conversion
			//printf_P(PSTR("ADC is starting on pin %i"), adc_count);

			break;
		}
	}


}


int adc_fetch(int adc_chan)
{
//	if(adc_chan<=adc_size-1)
//	{
		return adc_result[adc_chan];
//	}
//	return 0;
}

void adc_print(void)
{
	printf_P(PSTR("1:%i,2:%i,4:%i\n"),adc_result[1],adc_result[2],adc_result[4]);

}
