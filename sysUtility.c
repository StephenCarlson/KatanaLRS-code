#include "sysUtil.h"


static FILE uart_io = FDEV_SETUP_STREAM(putUARTchar, NULL, _FDEV_SETUP_WRITE);

void updateVolts(uint8_t fastMode){
	uint16_t lipoly = 0; // An array or struct would be more condusive?
	uint16_t sysVin = 0;
	uint16_t atMegaVolt = 0;
	
	
	for(uint8_t j=0; j<4; j++){
		lipoly += readADC(ADC_VBAT);
		sysVin += readADC(ADC_VIN);
		for(uint8_t i=0; i<4; i++) readADC(ADC_VSYS); // Pre-heat the VSYS ADC Input
		atMegaVolt += (fastMode)? readADC(ADC_VSYS) : readAdcNoiseReduced(ADC_VSYS); // [4092:0]
	}
	lipoly >>= 2; // [1023:0]
	sysVin >>= 2;
	atMegaVolt >>= 2;
	
	// printf("Raw: %u\t%u\t%u\n",lipoly,sysVin,atMegaVolt);
	
	// 3.300 System Voltage, 50% divider
	// (sample / 1023) * 2 * 3.3v
	// value = sample * 2 * atMegaVolt / 1023
	// voltSample = ((voltSample<<2)+(voltSample<<1)+(voltSample>>1));
	
	
	// atMegaVolt = (atMegaVolt < 225)? 4999 : (uint16_t)( (1125300)/((uint32_t) atMegaVolt) ); // 1125300 from (1100*1023)
	
	// sysVin = (sysVin * atMegaVolt * 2 )/ 1023;
	// sysVin = (uint16_t)( (uint32_t)( ((uint32_t)sysVin) * ((uint32_t)atMegaVolt) ) >> 9 );
	
	// atMegaADC = 1023 * 1.1v / ATMEGAv, battADC = 1023 * BATTv * .5 / ATMEGAv
	// ATMEGAv = 1023 * 1.1v / atMegaADC, BATTv = battADC * ATMEGAv * 2 / 1023
	// BATTv = battADC * 1.1v * 2 / atMegaADC
	// lipoly = lipoly (which is [1023:0]) * 1100 * 2 / atMegaVolt , Max is 2.2 Mil, non-determinined divide eats time bad
	// Repeated for sysVin
	// Is this next version more lossy? :
	// BATTv = battADC[1023:0] * ATMEGAv[4999:0] * 2 /1023 (appprox as >> 9) , Max is 5.1 Mil, divide is an easy right shift to within .1% actual
	// Both version require 32-bit ints, so might as well do the one with only one divide and 3 multiplies. First was 3 mult + 3 div
	
	atMegaVolt = (atMegaVolt < 225)? 4999 : (uint16_t)( (1125300)/((uint32_t) atMegaVolt) );
	sysVin = (uint16_t)( (uint32_t)( (uint32_t)sysVin * (uint32_t)atMegaVolt ) >> 9 );
	lipoly = (uint16_t)( (uint32_t)( (uint32_t)lipoly * (uint32_t)atMegaVolt ) >> 9 );
	
	sys.powerState = (sysVin > VIN_CUTOFF)? 1 : 0;
	sys.batteryState = (lipoly > LIPOLY_CUTOFF)? 1 : 0;
	
	// printf("Lipoly: %u\tVoltIn: %u\tATmega: %u\n",lipoly,sysVin,atMegaVolt);
	
	volt.lipoly = lipoly;
	volt.sysVin = sysVin;
	volt.atMega = atMegaVolt;
	
	//return ((uint16_t) voltSample);
}

static int putUARTchar(char c, FILE *stream){
    if (c == '\n') putUARTchar('\r', stream);
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
	// _delay_us(500);
	//transferSPI(c);
    return 0;
}

uint8_t getUARTchar(void){
    while( !(UCSR0A & (1<<RXC0)));
    return(UDR0);
}

uint16_t readADC(uint8_t adcChannel){
	ADMUX 	= adcChannel; //(1<<REFS0) |
	ADCSRA 	|= (1<<ADSC);
	while (ADCSRA & (1 << ADSC));
	return (ADCL + ((uint16_t) ADCH << 8));
}

uint16_t readAdcNoiseReduced(uint8_t adcChannel){
	ADMUX 	= adcChannel; //(1<<REFS0) |
	
	set_sleep_mode(SLEEP_MODE_ADC);
	sleep_enable();
	sleep_bod_disable();
	sei();
	sleep_cpu();
	
	sleep_disable();
	return (ADCL + ((uint16_t) ADCH << 8));
}

void flashOrangeLED(uint8_t count, uint8_t high, uint8_t low){
	for(;count>0; count--){
		LED_OR = HIGH;
		_delay_ms(high);
		LED_OR = LOW;
		_delay_ms(low);
	}
}

void flashBlueLED(uint8_t count, uint8_t high, uint8_t low){
	for(;count>0; count--){
		LED_BL = HIGH;
		_delay_ms(high);
		LED_BL = LOW;
		_delay_ms(low);
	}
}






