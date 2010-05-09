 


static const unsigned char adc_enable = 0x3F; //enabled adc channels in binary (bit 0 = chan 0, bit 1 = chan 1 etc)

static const unsigned char adc_highest=floor(log2(adc_enable)); //for chan 0 only adc_highest = 0, for chan 1 only adc_highest = 1 etc

volatile unsigned char adc_result[adc_highest+1];  //total number of elements = highest index +1

//const unsigned char adc_peice_count = sizeof(adc_enable)/sizeof(adc_enable[0]);

//char str[] = PSTR("hello world");


ISR(ADC_vect)
{ 
	unsigned char adc_count = (ADMUX|(7<<MUX0));
	// ADIF is cleared automatically if the interupt is enabled
	adc_result[adc_count] = ADCH; //Put ADC result in adc_result[chan]
	ADMUX &= ~(7<<MUX0); //clear MUX;
	while(adc_count++<adc_highest)
	{	
		if((adc_enable|(1<<adc_count))
		{
			ADMUX |= ((adc_count)<<MUX0); //Set MUX
			ADCSRA |= (1<<ADSC); //Start the next conversion
			break;
		}
	}

}

void adc_init(void)
{
	ADMUX = (1<<ADLAR);
	ADCSRA = (1<<ADEN)|(1<<ADIE)|(6<<ADPS0);
	//init the timer

}
void adc_start(void)
{
	unsigned char adc_count = 0;

	ADMUX &= ~(7<<MUX0); //clear MUX;
	while(adc_count++<adc_highest)
	{	
		if((adc_enable|(1<<adc_count))
		{
			ADMUX |= ((adc_count)<<MUX0); //Set MUX
			ADCSRA |= (1<<ADSC); //Start the conversion
			break;
		}
	}


}


char adc_fetch(char adc_chan)
{
	if(adc_chan<=adc_highest)
		return adc_result[adc_chan];
}
