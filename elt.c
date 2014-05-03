#include "elt.h"

volatile uint16_t eltPulseCount;
// static volatile struct{ // Huge mess, fix later
	// union{
	// }
	// uint8_t eltAfskPacket[6]; 
// } eltAfskPacket;

// volatile uint8_t eltAfskPacket[6] = {0b10101100, 0b01100000, 0b11000010, 0b00111011, 0b10010010, 0b01010111};
volatile uint8_t eltAfskPacket[6] = {'K', 'E', '7', 'Z', 'L', 'H'};


// 40.396323,-111.758359
// 0110 00001100 00100011
// 1011 10010010 01010111

// 40 bits @ (440 Hz / 11 pulses/bit or 932.328 Hz / 60 pulses/bit) = 1 sec

// 20 bits of Lat and Lon each
// 40 bits/sec
// 1 sec total
// If 440 and 880: 11 and 22 pulses, 22 and 44 transitions
// 089999999 Lat
// 179999999 Lon
//    ^^^^^^		Send these digits, 20 bits captures 1M
// 32-bits only captures 9 Decimal digits



ISR(TIMER2_COMPA_vect){ // OC2A --> MOSI Pin
	// FORCE_MOSI = eltPulseCount&0x01;
	eltPulseCount++;
}
// ISR(TIMER2_COMPB_vect){ // OC2B not connected externally
	// Signal the end of this bit
	// OCR2A
	// OCR2B
	// TCNT2
// }



// f_OC2A_PCPWM = f_clk_io/(N*510) with N= 1,8,32,64,128,256,1024

	// TCCR2A = 
	// TCCR2B = _BV(CS22)|_BV(CS20); // clk/128
	// OCR2A = 125; // 1.0 ms
	// TIMSK2 = (1<<TIOE2); //(1<<OCIE2B);

void systemIdle(void);

void systemIdle(void){
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();
	sleep_bod_disable();
	sei();
	sleep_cpu();
	sleep_disable();
}

void eltFullSequence(void){
	rfmWriteReg(0x75, 0x53);	// Freq Band		<>
	rfmWriteReg(0x76, 0x64);	// Freq Carrier 1	<>
	rfmWriteReg(0x77, 0x00);	// Freq Carrier 0	<>
	rfmWriteReg(RFM_CONTROL_1, RFM_xton);
	eltTransmit_AFSK();
	_delay_ms(1);
	eltTransmit_Packet();
	_delay_ms(1);
	eltTransmit_Beacon();
	_delay_ms(1);
	rfmWriteReg(RFM_CONTROL_1, 0x00);
}


void eltTransmit_AFSK(void){
	// Bell 202 is 1200 Hz for a Mark '1', 2200 Hz for Space '0', 1200 bps
	// I think I'll do a prime orthogonal set
	// Also, lets do 10 to 20 


	if((rfmReadReg(0x07)& RFM_xton) != RFM_xton){
		for(uint8_t i=0; (i<20) && rfmWriteReg(RFM_CONTROL_1, RFM_xton); i++){
			_delay_ms(1);
			if(i==20) printf("2B\n");
		}
	}
	
	rfmWriteReg(0x71, 0x12);		// FSK Async Mode, 
	rfmWriteReg(0x72, 7);			// Frequency deviation is 625 Hz * value (Centered, so actual peak-peak deviation is 2x)
	rfmWriteReg(0x6D, 0x08 | AFSK_TX_POWER);
	
	rfmWriteReg(RFM_CONTROL_1, RFM_txon);
	_delay_ms(1);
	SPCR = 0;
	TCCR2A = _BV(COM2A0)|_BV(WGM21); //|_BV(WGM20); //WGM21 WGM20
	TCCR2B = _BV(CS22)|_BV(CS20); //|_BV(WGM22); // clk/128 //WGM22
	TIMSK2 = (1<<OCIE2A);
	
	// 440 and 920 (A4 and about A#5)
	// Possible bauds for 440 Hz (2^3, 5, 11 are factors): 40, 88, 55
	// Prefered Baudrates to be cool: 4800, 2400, 1200, 600, 300, 110 (110 is in list of common bauds)
	// As I'm totally breaking with the Bell 103 and 202 specs, forget a standard baud, I'm doing 40 bps
	// The Space representation needs to factor nicly to 40 bps
	// At 440, 11 iterations is one mark, 11 is prime, choose a prime for the space.
	// 25 ms per bit, lest do either 17 or 19, which yields 680 and 760 Hz
	// 23 gives 920
	for(uint16_t i=0; i<sizeof(eltAfskPacket)*8; i++){
		uint8_t symbol = (0x01 & eltAfskPacket[i>>3]>>(7 - i%8)); // MSB First
		uint8_t period = (symbol)? 142 : 68; // Pulse period (actually, half the period of the freq)
		uint8_t reps   = (symbol)?  11 : 23; // Number of repetitions/cycles
		
		OCR2A = period; // Mark (1) is 440 Hz, Space (0) is 920 Hz
		eltPulseCount = 0;
		while(eltPulseCount< reps){
			systemIdle();
		}
	}
	
	TCCR2A = 0;
	TCCR2B = 0;
	TIMSK2 = 0;
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
	_delay_ms(1);
	
	rfmWriteReg(RFM_CONTROL_1, RFM_xton);
	for(uint8_t i=0; (i<200) && (rfmReadReg(0x07)&0x08); i++){
		_delay_ms(1);
		if(i==200) printf("1B\n");
	}
}

void eltTransmit_Packet(void){ //uint8_t *targetArray, uint8_t count){
	
	uint8_t index = 0;
	
	rfmWriteReg(0x08,0x01);		// FIFO Clear Sequence
	//_delay_ms(1);					
	rfmWriteReg(0x08,0x00);		
	rfmWriteReg(0x6E,0x27);		// Tx Data Rate 1	<>					
	rfmWriteReg(0x6F,0x52);		// Tx Data Rate 0	9600 Baud	
	rfmWriteReg(0x71, 0x23);		// GFSK, FIFO Used
	rfmWriteReg(0x72, 20);		// ~20kHz Peak-Peak Deviation
	rfmWriteReg(0x6D, 0x08 | 7);	// Max Power
	
	if((rfmReadReg(0x07)& RFM_xton) != RFM_xton ){
		for(uint8_t i=0; (i<20) && rfmWriteReg(RFM_CONTROL_1, RFM_xton); i++){
			_delay_ms(1);
			if(i==20) printf("2C\n");
		}
	}
	
	//		FCC ID, Lat, Long, UTC Fix, # Sat's, HDOP, Altitude, LiPoly, System In, AtMega
	//		Slightly Reordered from $GPGGA. Want Lat/Long in front, incase of clock skew, battery lag
	snprintf(dataBufferA,BUFFER_SIZE,"KE7ZLH,%+.9li,%+.9li,%.6lu,%u,%u,%+.4i,%u,%u,%u*\n", 
		(int32_t)gps.lat,(int32_t)gps.lon,(uint32_t)gps.time,gps.sats,gps.hdop,gps.alt,volt.lipoly,volt.sysVin,volt.atMega);
	
	//printf("%s",dataBufferA);
	
	CS_RFM = LOW;
		transferSPI((RFM_WRITE<<7) | 0x7F);
		transferSPI(0x00);
		for(uint8_t i=0; i<4; i++){
			transferSPI(0xAA);
		}
		transferSPI(0x09);
		for(; index<50; index++){ // String, obvious consequences if there is no \0 present
			if(dataBufferA[index] == '\0') break;
			transferSPI(dataBufferA[index]);
			//if(i == BUFFER_SIZE) printf("Fail on String\n");
		}
	CS_RFM = HIGH;
	_delay_us(1);
	
	rfmWriteReg(RFM_CONTROL_1, RFM_txon);

	for(uint8_t i=0; i<200; i++){
		_delay_ms(1);
		if(rfmReadReg(0x03)&0x20) break;
		if(i==200) printf("1C\n");
	}
	
	CS_RFM = LOW;
		transferSPI((RFM_WRITE<<7) | 0x7F);
		for(; index<BUFFER_SIZE; index++){
			if(dataBufferA[index] == '\0') break;
			transferSPI(dataBufferA[index]);
		}
	CS_RFM = HIGH;
	_delay_us(1);
	
	for(uint8_t i=0; (i<200) && (rfmReadReg(0x07)&0x08); i++){
		_delay_ms(1);
		if(i==200) printf("3C\n");
	}
	
	rfmWriteReg(RFM_CONTROL_1, RFM_xton);
}

void eltTransmit_Beacon(void){
	if((rfmReadReg(0x07)& RFM_xton) != RFM_xton ){
		for(uint8_t i=0; (i<20) && rfmWriteReg(RFM_CONTROL_1, RFM_xton); i++){
			_delay_ms(1);
			if(i==20) printf("2A\n");
		}
	}
	
	rfmWriteReg(0x71, 0x12);		// FSK Async Mode, 
	rfmWriteReg(0x72, 7);			// Frequency deviation is 625 Hz * value (Centered, so actual peak-peak deviation is 2x)
	
	//_delay_ms(1);
	
	for(uint8_t n=1; n<BEACON_NOTES; n++){ // n=0 for no AFSK, but as the AFSK packet is A4/A5 notes, it replaces A4 here
		rfmWriteReg(0x6D, 0x08 | beaconNotes[n][2]);
		rfmWriteReg(RFM_CONTROL_1, RFM_txon);
		_delay_ms(1);
		SPCR = 0;
		CS_RFM = HIGH;
		_delay_ms(1);
		
		// CTC Mode with Toggle on OC2A (SPI MOSI)
		TCCR2A = _BV(COM2A0)|_BV(WGM21); //|_BV(WGM20); //WGM21 WGM20
		TCCR2B = _BV(CS22)|_BV(CS20); //|_BV(WGM22); // clk/128 //WGM22
		TIMSK2 = (1<<OCIE2A); //(1<<OCIE2B);
		
		OCR2A = beaconNotes[n][0]; // Poor Resolution for 8-bit range
		// 142.0, 112.7,  94.8, 106.4,  84.5, 71.0
		// 440.1, 553.1, 657.9, 589.6, 744.0, 880.3
		// 440    554.4  659.3  587.3  740    880
		// ok     -1.3   -1.4   +2.3   +4.0   ok
		
		// 
		// 440.14	558.04	664.89
		// Considered using Timer 0, timer10ms, both won't work well
		// Timer0 is 15 kHz but would roll often as it is 8-bit
		// timer10ms is driven by the Timer0 Interrupt. Resolution would be unstable
		eltPulseCount = 0;
		while(eltPulseCount<beaconNotes[n][1]){
			systemIdle();
		}

		
		TCCR2A = 0;
		TCCR2B = 0;
		TIMSK2 = 0;
		SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
		_delay_ms(1);
	}
	
	rfmWriteReg(RFM_CONTROL_1, RFM_xton);
	for(uint8_t i=0; (i<200) && (rfmReadReg(0x07)&0x08); i++){
		_delay_ms(1);
		if(i==200) printf("1A\n");
	}
}