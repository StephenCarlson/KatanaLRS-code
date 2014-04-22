#include "elt.h"

volatile uint16_t pulseCount;
// static volatile struct{ // Huge mess, fix later
	// union{
	// }
	// uint8_t afskPacket[6]; 
// } afskPacket;

volatile uint8_t afskPacket[6] = {0b10101100, 0b01100000, 0b11000010, 0b00111011, 0b10010010, 0b01010111};


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



ISR(TIMER2_COMPA_vect){ // OC2A --> MOSI Pin
	// FORCE_MOSI = pulseCount&0x01;
	pulseCount++;
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





void transmitELT(void){
	radioWriteReg(0x75, 0x53);
	radioWriteReg(0x76, 0x64);
	radioWriteReg(0x77, 0x00);
	radioWriteReg(OPCONTROL1_REG, (1<<RFM_xton));
	transmitELT_AFSK();
	_delay_ms(1);
	transmitELT_Packet();
	_delay_ms(1);
	//transmitELT_Beacon();
	//_delay_ms(1);
	radioWriteReg(OPCONTROL1_REG, 0x00);
}


void transmitELT_Beacon(void){
	if((radioReadReg(0x07)&(1<<RFM_xton)) != (1<<RFM_xton) ){
		for(uint8_t i=0; (i<20) && radioWriteReg(OPCONTROL1_REG, (1<<RFM_xton)); i++){
			_delay_ms(1);
			if(i==20) printf("2A\n");
		}
	}
	
	radioWriteReg(0x71, 0x12);		// FSK Async Mode, 
	radioWriteReg(0x72, 7);			// Frequency deviation is 625 Hz * value (Centered, so actual peak-peak deviation is 2x)
	
	radioWriteReg(OPCONTROL1_REG, (1<<RFM_txon));
	//_delay_ms(1);
	
	for(uint8_t n=0; n<BEACON_NOTES; n++){ // n=0 for no AFSK, but as the AFSK packet is A4/A5 notes, it replaces A4 here
		radioWriteReg(0x6D, beaconNotes[n][2]);
		_delay_ms(1);
		SPCR = 0;
		CS_RFM = HIGH;
		_delay_ms(1);
		
		// CTC Mode with Toggle on OC2A (SPI MOSI)
		TCCR2A = _BV(COM2A0)|_BV(WGM21); //|_BV(WGM20); //WGM21 WGM20
		TCCR2B = _BV(CS22)|_BV(CS20); //|_BV(WGM22); // clk/128 //WGM22
		TIMSK2 = (1<<OCIE2A); //(1<<OCIE2B);
		
		OCR2A = beaconNotes[n][0]>>3; // Poor Resolution for 8-bit range
		// 142.0, 112.7,  94.8, 106.4,  84.5, 71.0
		// 440.1, 553.1, 657.9, 589.6, 744.0, 880.3
		// 440    554.4  659.3  587.3  740    880
		// ok     -1.3   -1.4   +2.3   +4.0   ok
		
		// 
		// 440.14	558.04	664.89
		// Considered using Timer 0, timer10ms, both won't work well
		// Timer0 is 15 kHz but would roll often as it is 8-bit
		// timer10ms is driven by the Timer0 Interrupt. Resolution would be unstable
		pulseCount = 0;
		while(pulseCount<beaconNotes[n][1]);

		
		TCCR2A = 0;
		TCCR2B = 0;
		TIMSK2 = 0;
		SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
		_delay_ms(1);
	}
	
	radioWriteReg(OPCONTROL1_REG, (1<<RFM_xton));
	for(uint8_t i=0; (i<200) && (radioReadReg(0x07)&0x08); i++){
		_delay_ms(1);
		if(i==200) printf("1A\n");
	}
}


void transmitELT_AFSK(void){
	// Bell 202 is 1200 Hz for a Mark '1', 2200 Hz for Space '0', 1200 bps
	// I think I'll do a prime orthogonal set
	// Also, lets do 10 to 20 


	if((radioReadReg(0x07)&(1<<RFM_xton)) != (1<<RFM_xton) ){
		for(uint8_t i=0; (i<20) && radioWriteReg(OPCONTROL1_REG, (1<<RFM_xton)); i++){
			_delay_ms(1);
			if(i==20) printf("2B\n");
		}
	}
	
	radioWriteReg(0x71, 0x12);		// FSK Async Mode, 
	radioWriteReg(0x72, 7);			// Frequency deviation is 625 Hz * value (Centered, so actual peak-peak deviation is 2x)
	radioWriteReg(0x6D, AFSK_TX_POWER);
	
	radioWriteReg(OPCONTROL1_REG, (1<<RFM_txon));
	//_delay_ms(1);
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
	for(uint16_t i=0; i<sizeof(afskPacket)*8; i++){
		uint8_t symbol = (0x01 & afskPacket[i>>3]>>(7 - i%8)); // MSB First
		uint8_t period = (symbol)? 142 : 68; // Pulse period (actually, half the period of the freq)
		uint8_t reps   = (symbol)?  11 : 23; // Number of repetitions/cycles
		
		OCR2A = period; // Mark (1) is 440 Hz, Space (0) is 920 Hz
		pulseCount = 0;
		while(pulseCount< reps);
	}
	
	TCCR2A = 0;
	TCCR2B = 0;
	TIMSK2 = 0;
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
	_delay_ms(1);
	
	radioWriteReg(OPCONTROL1_REG, (1<<RFM_xton));
	for(uint8_t i=0; (i<200) && (radioReadReg(0x07)&0x08); i++){
		_delay_ms(1);
		if(i==200) printf("1B\n");
	}
}


void transmitELT_Packet(void){ //uint8_t *targetArray, uint8_t count){
	
	
	radioWriteReg(0x08,0x01);		// FIFO Clear Sequence
	//_delay_ms(1);					
	radioWriteReg(0x08,0x00);		
	radioWriteReg(0x71, 0x23);		// GFSK, FIFO Used
	radioWriteReg(0x72, 20);		// ~20kHz Peak-Peak Deviation
	radioWriteReg(0x6D, 7);			// Max Power
	
	if((radioReadReg(0x07)&(1<<RFM_xton)) != (1<<RFM_xton) ){
		for(uint8_t i=0; (i<20) && radioWriteReg(OPCONTROL1_REG, (1<<RFM_xton)); i++){
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
		for(uint8_t i=0; i<BUFFER_SIZE; i++){ // String, obvious consequences if there is no \0 present
			if(dataBufferA[i] == '\0') break;
			transferSPI(dataBufferA[i]);
			//if(i == BUFFER_SIZE) printf("Fail on String\n");
		}
	CS_RFM = HIGH;
	_delay_us(1);
	
	radioWriteReg(OPCONTROL1_REG, (1<<RFM_txon));

	for(uint8_t i=0; (i<200) && (radioReadReg(0x07)&0x08); i++){
		_delay_ms(1);
		if(i==200) printf("1C\n");
	}
	radioWriteReg(OPCONTROL1_REG, (1<<RFM_xton));
}




































// void audioBlip(uint8_t tone, uint8_t count, uint8_t interval){
	// Make a tone on the FPV audio line using the I2C SDA output
	// Use Timer 2, as this is dedicated to audio tones anyway
	// This makes the most sense, as it is usually High-Z with a 5k pullup
	// Putting this through a capacitor to the mic line should work well
	// The harness should also include a voltage divider, as 3.3v on a 1Vpp mic is harsh
	// Actually, I wonder what it looks like when the SDA DDRC bit is toggled with its PORTC bit high?
	// That might be more workable with no need for a fancy buffering harness, just a capacitor.
	// What would be really nice is no pull-up resistors, then just toggle High-Z / asserted low.
	// This would be fine, as the mic hangs out at ~ 0.5v
	
	// Anyhow, this is all feature bloat, add this after the system is working well
	
	
	
	
// }





/*
void transmitELT_Beacon(void){
	if((radioReadReg(0x07)&(1<<RFM_xton)) != (1<<RFM_xton) ){
		for(uint8_t i=0; (i<20) && radioWriteReg(OPCONTROL1_REG, (1<<RFM_xton)); i++) _delay_ms(1);
		//printf("Fail on Preset: Beacon\n");
		//_delay_ms(2);
	}
	
	radioWriteReg(0x71, 0x12);		// FSK Async Mode, 
	radioWriteReg(0x72, 7);			// Frequency deviation is 625 Hz * value (Centered, so actual peak-peak deviation is 2x)
	
	radioWriteReg(OPCONTROL1_REG, (1<<RFM_txon));
	//_delay_ms(1);
	
	for(uint8_t n=0; n<BEACON_NOTES; n++){ // n=0 for no AFSK, but as the AFSK packet is A4/A5 notes, it replaces A4 here
		radioWriteReg(0x6D, beaconNotes[n][2]);
		_delay_ms(1);
		SPCR = 0;
		CS_RFM = HIGH;
		_delay_ms(1);
		for(uint16_t d=0; d<beaconNotes[n][1]; d++){
			FORCE_MOSI = d&0x01;
			_delay_us(beaconNotes[n][0]); // Getting 416 Hz A4 w/o correction, means its adding 66 uS 
		}
		SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
	}
	radioWriteReg(OPCONTROL1_REG, (1<<RFM_xton));
}
*/

/*
void transmitELT_AFSK(void){
	// Bell 202 is 1200 Hz for a Mark '1', 2200 Hz for Space '0', 1200 bps
	// I think I'll do a prime orthogonal set
	// Also, lets do 10 to 20 


	if((radioReadReg(0x07)&(1<<RFM_xton)) != (1<<RFM_xton) ){
		for(uint8_t i=0; (i<20) && radioWriteReg(OPCONTROL1_REG, (1<<RFM_xton)); i++) _delay_ms(1);
		//printf("Fail on Preset: Beacon\n");
		//_delay_ms(2);
	}
	
	radioWriteReg(0x71, 0x12);		// FSK Async Mode, 
	radioWriteReg(0x72, 7);			// Frequency deviation is 625 Hz * value (Centered, so actual peak-peak deviation is 2x)
	radioWriteReg(0x6D, AFSK_TX_POWER);
	
	radioWriteReg(OPCONTROL1_REG, (1<<RFM_txon));
	//_delay_ms(1);
	SPCR = 0;
	TCCR2A = 0; //WGM21 WGM20; 
	TCCR2B = _BV(CS22)|_BV(CS21)|_BV(CS20); // WGM22
	TIMSK2 = (1<<OCIE2A)|(1<<OCIE2B); // TOV2
	pulseCount = 0;
	
	while(pulseCount<sizeof(message)){
		
	}
	
	TCCR2A = 0;
	TCCR2B = 0;
	TIMSK2 = 0;
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}
*/

