#include "elt.h"

volatile uint16_t pulseCount;
volatile uint8_t messageBuffer[40]; // 40 bits @ (1200 Hz / 30 pulses/bit or 2400 Hz / 60 pulses/bit) = 1 sec

// 20 bits of Lat, Lon
// 40 bits/sec
// If 440 and 880: 11 and 22 pulses, 22 and 44 transitions
// For Timer2, 16 MHz / 1024 is 15625 kHz, 16 ms max delay
// 089999999 Lat
// 179999999 Lon
//    ^^^^^^		Send these digits, 20 bits captures 1M


/*
ISR(TIMER2_COMPA_vect){
	FORCE_MOSI = pulseCount&0x01;
	pulseCount++;
}
ISR(TIMER2_COMPB_vect){
	// Signal the end of this bit
	//OCR2A
	//OCR2B
	// TCNT2
}

*/


void transmitELT(void){
	radioWriteReg(0x75, 0x53);
	radioWriteReg(0x76, 0x64);
	radioWriteReg(0x77, 0x00);
	//radioWriteReg(OPCONTROL1_REG, (1<<RFM_xton));
	//transmitELT_AFSK();
	_delay_ms(1);
	transmitELT_Packet();
	_delay_ms(1);
	transmitELT_Beacon();
	_delay_ms(1);
	radioWriteReg(OPCONTROL1_REG, 0x00);
}

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
}

void transmitELT_Packet(void){ //uint8_t *targetArray, uint8_t count){
	
	
	radioWriteReg(0x08,0x01);		// FIFO Clear Sequence
	//_delay_ms(1);					
	radioWriteReg(0x08,0x00);		
	radioWriteReg(0x71, 0x23);		// GFSK, FIFO Used
	radioWriteReg(0x72, 20);		// ~20kHz Peak-Peak Deviation
	radioWriteReg(0x6D, 7);			// Max Power
	
	if((radioReadReg(0x07)&(1<<RFM_xton)) != (1<<RFM_xton) ){
		for(uint8_t i=0; (i<20) && radioWriteReg(OPCONTROL1_REG, (1<<RFM_xton)); i++) _delay_ms(1);
		// printf("Fail on Preset: Packet\n");
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

	for(uint8_t i=0; (i<200) && (radioReadReg(0x07)&0x08); i++) _delay_ms(1);
}

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

