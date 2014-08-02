// RFM22B and RFM23BP Transceiver Library, by Stephen Carlson, April 2014
// Use Notepad++ with Tab=4


#include "rfm22b.h"

#define IDLE_STANDBY	0		// Very Low Power
#define IDLE_READY		1		// Fast Response
#define IDLE_TUNE		2		// Best Response Time
#define RX_FHSS_LRS		3		// 
#define RX_TONE_LDC		4
#define TX_AFSK			5
#define TX_PACKET_4800	6
//#define IDLE_SLEEP Same as LDC


#define RFM_READ		(0<<7) // RFM Direction Flags
#define RFM_WRITE		(1<<7)


uint8_t rfmReset(void){
	uint8_t failed = 0;
	// for(uint8_t i=0; i<5; i++){
		rfmWriteReg(0x07, 0x80);		// Reset the Chip
	//	_delay_us(200);
	// }
	for(uint8_t i=0; i<200; i++){
		if((rfmReadReg(0x04)&0x02) == 0x02) break;
		if(i==200) failed = 1;
		_delay_ms(1);
	}
	return failed;
}

uint8_t rfmMode(uint8_t mode){
	// Returns non-zero if the the system is in the correct state, zero otherwise
	static uint8_t currentMode = 99;
	
	if(currentMode == mode){ // If we already have commanded this mode, poll for completed transition
		switch(mode){
			case IDLE_TUNE:
				return !((rfmReadReg(0x62)>>5)^0x03); // TUNE
				break;
			case IDLE_READY:
				return !((rfmReadReg(0x62)>>5)^0x01); // RDY
				break;
			case RX_FHSS_LRS:
				return !((rfmReadReg(0x62)>>5)^0x07); // RX
				break;
			case RX_TONE_LDC:
				return !( ((rfmReadReg(0x62)>>5)^0x00)|((rfmReadReg(0x08)&0x04)^0x04) ); // LP and LDC
				break;
			case TX_AFSK:
				return !((rfmReadReg(0x62)>>5)^0x03); // TUNE //TX
				break;
			case TX_PACKET_4800:
				return !((rfmReadReg(0x62)>>5)^0x03); // TUNE
				break;
			default:
				return !( ((rfmReadReg(0x62)>>5)^0x00)|((rfmReadReg(0x08)&0x04)^0x00) ); // LP and no LDC
		}
	} else{
		mode = (currentMode == 99)? IDLE_STANDBY : mode;
		switch(mode){
			case IDLE_TUNE:
				rfmSetRxTxSw(0);
				rfmWriteReg(0x07, 0x02);
				break;
				
			case IDLE_READY:
				rfmSetRxTxSw(0);
				rfmWriteReg(0x07, 0x01);
				break;
				
			case RX_FHSS_LRS: // 9600 Baud across 5 MHz
				for(uint8_t i=0; i<sizeof(rfmConfig_ModeConfigs)/4;i++) \
					rfmWriteReg(rfmConfig_ModeConfigs[i][0],rfmConfig_ModeConfigs[i][RFMCFG_FHSS]);
				for(uint8_t i=0; i<sizeof(rfmConfig_FhssConfig)/2;i++) \
					rfmWriteReg(rfmConfig_FhssConfig[i][0],rfmConfig_FhssConfig[i][1]);
				rfmClearRxFIFO();
				rfmSetRxTxSw(RFM_rxon);
				rfmWriteReg(0x07, RFM_rxon);		// Idle Standby
				rfmWriteReg(0x05, 0x02);	// Valid Packet
				rfmWriteReg(0x06, 0);
				rfmWriteReg(0x08, 0);		// LDC Off
				break;
			
			case RX_TONE_LDC: // 3500 "Baud" on 434.000
				for(uint8_t i=0; i<sizeof(rfmConfig_ModeConfigs)/4;i++) \
					rfmWriteReg(rfmConfig_ModeConfigs[i][0],rfmConfig_ModeConfigs[i][RFMCFG_AFSK]);
				rfmSetRxTxSw(RFM_rxon);			// Set the RF Switch to RX
				rfmWriteReg(0x07, 0);			// Idle Standby
				rfmWriteReg(0x05, 0);		
				rfmWriteReg(0x06, 0x80);		// Valid Sync
				rfmWriteReg(0x08, (1<<2));		// Enable LDC Mode
				rfmWriteReg(0x71, 0x22);		// Set for FIFO, needed for Sync to work?
				rfmWriteReg(0x33,0x08);
				rfmWriteReg(0x34,0x30);
				rfmWriteReg(0x35,0x30);
				rfmWriteReg(0x36,0x55);
				break; // As this and next are similiar/cloned, could remove break.
			
			case TX_AFSK: // 440 and 920 Hz tokens at 40 Baud on 434.000
				for(uint8_t i=0; i<sizeof(rfmConfig_ModeConfigs)/4;i++) \
					rfmWriteReg(rfmConfig_ModeConfigs[i][0],rfmConfig_ModeConfigs[i][RFMCFG_AFSK]);
				// rfmSetRxTxSw(RFM_txon);
				rfmWriteReg(0x07, 0x02);	
				rfmWriteReg(0x05, 0);	
				rfmWriteReg(0x06, 0);	
				rfmWriteReg(0x08, 0);			// Disable LDC
				rfmWriteReg(0x6D, 0x08);		// Max Power  | MAX_TX_POWER
				rfmWriteReg(0x71, 0x12);		// Direct Mode
				break;
				
			case TX_PACKET_4800: // 4800 Baud on 434.000
				for(uint8_t i=0; i<sizeof(rfmConfig_ModeConfigs)/4;i++) \
					rfmWriteReg(rfmConfig_ModeConfigs[i][0],rfmConfig_ModeConfigs[i][RFMCFG_4800]);
				rfmClearTxFIFO();
				rfmSetRxTxSw(0);
				rfmWriteReg(0x07, 0x02);
				rfmWriteReg(0x05, 0);	
				rfmWriteReg(0x06, 0);	
				// rfmWriteReg(0x08, 0x08);		// Disable LDC, Packet Auto-Tx
				rfmWriteReg(0x08, 0);			// Disable LDC
				rfmWriteReg(0x6D, 0x08);		// Max Power | MAX_TX_POWER
				break;
			
			case IDLE_STANDBY: // Drop thru to default
			default:
				mode = IDLE_STANDBY;
				for(uint8_t i=0; i<sizeof(rfmConfig_Core)/2;i++){
					rfmWriteReg(rfmConfig_Core[i][0],rfmConfig_Core[i][1]);
				}
				rfmSetRxTxSw(0);
				rfmWriteReg(0x07, 0);
				rfmWriteReg(0x05, 0);	
				rfmWriteReg(0x06, 0);
				rfmWriteReg(0x08, 0);
				rfmWriteReg(0x62, 0x04);
				rfmWriteReg(0x6D, 0x08);
		}
		currentMode = mode;
		return(0);
	}
}

// uint8_t rfmTxPacket(void){ Very bad coding
	// uint8_t state = rfmReadReg(RFM_CONTROL_1)&RFM_txon;
	// if(!state) rfmWriteReg(RFM_CONTROL_1, RFM_txon); // Bad, may activate Tx after packet sent
	// return(!(state));
// }

// void rfmTxDirect(uint8_t active){
	// rfmWriteReg(RFM_CONTROL_1, (active)? RFM_txon : 0);
// }

void rfmSetRxTxSw(uint8_t state){
	#if defined(RFM23BP)
		// Critical Note! The RFM23BP is LOW-Asserted for the RF switch
		// Thus, Rx->0 (low) and Tx-> 1 (high) for Rx, backward from RFM22B
		rfmWriteReg(RFM_GPIO_0,RFM_p_DDO);
		rfmWriteReg(RFM_GPIO_1,RFM_p_DDO);
		if(state&RFM_rxon){
			rfmWriteReg(RFM_IO_PORT,RFM_dio1);	// GPIO0 Low, GPIO1 High
		} else if(state&RFM_txon){
			rfmWriteReg(RFM_IO_PORT,RFM_dio0);	// GPIO0 High, GPIO1 Low
		} else{
			rfmWriteReg(RFM_IO_PORT,RFM_dio1 | RFM_dio0);
		}
	
	
	#endif // RFM23BP
	
	// rfmWriteReg(RFM_CONTROL_1, (state&RFM_rxon)? RFM_rxon : (state&RFM_txon)? RFM_txon : 0);
}


uint8_t rfmGetRxTx(uint8_t mask){
	return rfmReadReg(RFM_CONTROL_1)&(RFM_txon | RFM_rxon)&mask;
}

/*
void rfmSetInterrupts(uint8_t mode, uint8_t rssiThresh){
	if(mode == ENABLED){
		
		// Configure for 1750 Hz Tone Detection
		// for(uint8_t i=0; i<sizeof(rfmConfig_AccessTone)/2;i++){
			// rfmWriteReg(rfmConfig_AccessTone[i][0],rfmConfig_AccessTone[i][1]);
		// }
		
		
		//rfmWriteReg(0x05, (1<<1)); // RX FIFO Almost Full
		rfmWriteReg(0x06, (1<<7)); // RX FIFO Almost Full
		//rfmWriteReg(0x05, (1<<1)); // Enable Valid Packet Received Interrupt
		// rfmWriteReg(0x05, (1<<4)); // Enable Valid Packet Received Interrupt
		//rfmWriteReg(0x06, (1<<7)|(1<<6)); // (1<<4)| Enable RSSI Interrupt
		//rfmWriteReg(0x27, (rssiThresh)); // Configure RSSI for +30dBm Level Threshold
		// {0x07, (1<<5)},	// Wake Up Timer Enabled
		
	} else{
		rfmWriteReg(0x05, 0);
		rfmWriteReg(0x06, 0);
		rfmWriteReg(0x07, 0);
	}
}
*/

uint16_t rfmGetInterrupts(void){
	uint16_t rfmIntList;
	
	CS_RFM = LOW;
		transferSPI(0x03);
		rfmIntList = transferSPI(0x00)<<8;	// Int Stat 1
		rfmIntList |= transferSPI(0x00);	// Int Stat 2
	CS_RFM = HIGH;
	return rfmIntList;
}

void rfmSetLrsChannel(uint8_t channel){
	// fcarrier = (fb+24+(fc+fo)/64000) * 10000 * (hbsel+1) + (fhch * fhs * 10) [kHz]
	// fb=19 (10 MHz Increments), fo=32 (+5 kHz Up Tick), fc=3200 (500 kHz)
	// DL uses 27 kHz spacing
	// 27 kHz -> 172.8
	// 3200 
	// 1:6.4, or .15625
	
	// uint16_t value = 3200 + channel*172.8;
	// #define DL_OFFSET	3200
	// #define DL_STEP(ch)	(ch*172 + ch*4/5) // Broken
	// #define DL_STEP(ch)	(uint16_t)(((uint32_t)(ch*1728)) / 10) + ((((uint32_t)(ch*1728)) % 10) > 4)? 1:0  // Broken-ish
	// #define DL_STEP(ch)	(uint16_t)(((uint32_t)(ch*1728) + 5) / 10) // Way better, half the size in compiled assembly
	// uint16_t value = DL_OFFSET + DL_STEP(channel);
	uint16_t value = 3200 + (uint16_t)(((uint32_t)(ch*1728) + 5) / 10); // 3144
	
	rfmWriteReg(0x76,value>>8);	// Freq Carrier 1	Upper Byte
	rfmWriteReg(0x77,value&0xFF);	// Freq Carrier 0	Lower Byte
}

uint8_t rfmSetManualFreq(uint16_t manualFreq){
	rfmWriteReg(0x76,manualFreq>>8);		// Freq Carrier 1	Upper Byte
	rfmWriteReg(0x77,manualFreq&0xFF);	// Freq Carrier 0	Lower Byte
	return(rfmReadReg(0x07)&0x02); // TUNE Mode
}

void rfmSetTxPower(uint8_t power){
	rfmWriteReg(0x6D, 0x08 | (power&0x07));
}

uint8_t rfmGetRSSI(void){
	uint8_t rssiMeasure = 0;
	/*
	if((rfmReadReg(RFM_CONTROL_1)& RFM_rxon) != RFM_rxon){
		// printf("%X\n",rfmReadReg(RFM_CONTROL_1));
		rfmWriteReg(RFM_CONTROL_1, RFM_rxon);
		
		_delay_ms(10);
	
		// for(uint8_t i=0; i<255; i++){
			// if((rfmReadReg(0x02)&0x01) != 1){
				// printf("Not Rx@ %u\n",i);
				// //break;
			// }
			// _delay_ms(1);
		// }
		// for(uint8_t i=0; i<255; i++){
			// if((rfmReadReg(0x02)&0x01) == 0){
				// printf("Break@ %u\n",i);
				// break;
			// }
			// _delay_ms(1);
		// }
		rssiMeasure = rfmReadReg(0x26);
		rfmWriteReg(RFM_CONTROL_1, 0); //RFM_xton);
	} else rssiMeasure = rfmReadReg(0x26);
	*/
	rssiMeasure = rfmReadReg(0x26);
	
	// Huge warning!!! <steve> Do not Write-Read cycle the 0x07 register, it triggers a soft reset
	
	return rssiMeasure; //
}

uint8_t rfmReadReg(uint8_t regAddress){
	CS_RFM = LOW;
		transferSPI(regAddress);
		_delay_us(1);
		uint8_t value = transferSPI(0xFF); // Seems that the line likes to rest high
	CS_RFM = HIGH;
	_delay_us(1);
	return value;
}

uint8_t rfmWriteReg(uint8_t regAddress, uint8_t regValue){
	uint8_t readBack = rfmReadReg(regAddress);
	CS_RFM = LOW;
		transferSPI(RFM_WRITE | regAddress);
		_delay_us(1);
		transferSPI(regValue);
	CS_RFM = HIGH;
	_delay_us(2);
	readBack = rfmReadReg(regAddress)^(regValue);
	if(readBack) LED_OR = HIGH;
	rfmWriteErrors += (readBack)? 1 : 0;
	// for(uint8_t i=0; (i<10) && (readBack); i++){
	//if(readBack){
		/*
		CS_RFM = LOW;
			transferSPI(RFM_WRITE | regAddress);
			//_delay_us(1);
			transferSPI(regValue);
		CS_RFM = HIGH;
		_delay_us(50);
		*/
		// readBack = rfmReadReg(regAddress)^(regValue);
		
		// LED_OR = HIGH;
		// _delay_us(50);
	// }
	LED_OR = LOW;
	return readBack; //(readBack)? readBack : 0; Seems I check for not working more than working.
}

void rfmReadFIFO(uint8_t *array){
	CS_RFM = LOW;
		transferSPI(0x7F); //(RFM_READ<<7) | 
		for(uint8_t i=0; i<64; i++){
			array[i] = transferSPI(i); //0x00);
		}
	CS_RFM = HIGH;
	_delay_us(1);
}

void rfmReadFIFOn(uint8_t *array, uint8_t n){
	n = (n>64)? 64:n;
	CS_RFM = LOW;
		transferSPI(0x7F); //(RFM_READ<<7) | 
		for(uint8_t i=0; i<n; i++){
			array[i] = transferSPI(i); //0x00);
		}
	CS_RFM = HIGH;
	_delay_us(1);
}

uint8_t rfmWriteFIFOArray(uint8_t *array, uint8_t n){
	uint8_t count = 0;
	n = (n>64)? 64:n;
	CS_RFM = LOW;
		transferSPI(0xFF);
		for(uint8_t i=0; i<n; i++){
			transferSPI(array[i]);
			count++;
		}
	CS_RFM = HIGH;
	_delay_us(1);
	return count;
}

uint8_t rfmWriteFIFOStr(char *array){ //, uint8_t index){
	uint8_t count = 0;
	// index = (index>64)? 64 : index;
	CS_RFM = LOW;
		transferSPI(0xFF);
		// for(uint8_t i=index; i<(64-index); i++){
		for(uint8_t i=0; i<64; i++){
			if(array[i] == '\0') break; // Bad idea if the string carries non-ASCII data
			transferSPI(array[i]);
			count++;
		}
	CS_RFM = HIGH;
	_delay_us(1);
	return(count);
}

uint8_t rfmGetTxFIFOEmpty(void){ // Totally Irrelevant, can do a macro instead
	return rfmReadReg(0x03)&0x20;
}

void rfmClearTxFIFO(void){
	rfmWriteReg(0x08,0x01);		// FIFO Clear Sequence
	//_delay_ms(1);					
	rfmWriteReg(0x08,0x00);	
}

void rfmClearRxFIFO(void){
	rfmWriteReg(0x08, (1<<1));
	rfmWriteReg(0x08, 0);	
}

// uint8_t rfmTxPacket(uint8_t *array, uint8_t n){
	// uint8_t count = rfmWriteFIFOArray(array,n);
	// return count;
// }









