 
static unsigned char adc_count;

static unsigned char adc_enable[] = {0, 1, 2, 3, 4, 5};
unsigned char adc_result[sizeof(adc_enable)/sizeof(adc_enable[0])];

//char str[] = PSTR("hello world");


ISR(ADC_vect)
{ // ADIF is cleared automatically if the interupt is enabled
	adc_result[(ADMUX|(7<<MUX0))] = ADCH;
	if(adc_count<sizeof(adc_enable)/sizeof(adc_enable[0])-1)
	{
		//sanity check
		//if(adc_enable[adc_count]<7)
		//{
		ADMUX &= ~(7<<MUX0); //clear MUX;
		ADMUX |= (adc_enable[adc_count]<<MUX0); //Set MUX

		adc_count++; //incriment at the end.
		if(adc_count<sizeof(adc_enable)/sizeof(adc_enable[0])-1)
		{
			ADCSRA |= (1<<ADSC); //Start the next conversion
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
	adc_count = 0;
	ADCSRA |= (1<<ADSC);

}


char adc_fetch(char adc_chan)
{
	for(i=0;i<sizeof(adc_enable)/sizeof(adc_enable[0])-1;i++)
	{
		if(adc_enable[i]==adc_chan)
		{
			return adc_result[i];
		}
	}
	
	return 0x00;
}
