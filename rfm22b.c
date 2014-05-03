// RFM22B and RFM23BP Transceiver Library, by Stephen Carlson, April 2014
// Use Notepad++ with Tab=4


#include "rfm22b.h"

void rfmSetMode(uint8_t mode){
	
	for(uint8_t i=0; i<sizeof(rfmConfig_Core)/2;i++){
		rfmWriteReg(rfmConfig_Core[i][0],rfmConfig_Core[i][1]);
	}
	
	// for(uint8_t i=0; i<sizeof(rfmConfig_DragonLinkCompat)/2;i++){
		// rfmWriteReg(rfmConfig_DragonLinkCompat[i][0],rfmConfig_DragonLinkCompat[i][1]);
	// }
	
	for(uint8_t i=0; i<sizeof(rfmConfig_AccessTone)/2;i++){
		rfmWriteReg(rfmConfig_AccessTone[i][0],rfmConfig_AccessTone[i][1]);
	}
	
	
	
	
	if(mode == SLEEP){
		rfmWriteReg(0x08, (1<<1));
		rfmWriteReg(0x08, 0);
		rfmWriteReg(0x07, 0); //(1<<2)|(1<<1));
		rfmWriteReg(0x08, (1<<2));
	}
}

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

uint16_t rfmGetInterrupts(void){
	uint16_t rfmIntList = rfmReadReg(0x03);
	rfmIntList |= rfmReadReg(0x04)<<8;
	return rfmIntList;
}

void rfmSetDlChannel(uint8_t channel){
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

uint8_t rfmGetRSSI(void){
	uint8_t rssiMeasure = 0;
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
	}
	
	return rssiMeasure; //
}

uint8_t rfmReadReg(uint8_t regAddress){
	CS_RFM = LOW;
		transferSPI(regAddress);
		uint8_t value = transferSPI(0x00);
	CS_RFM = HIGH;
	_delay_us(1);
	return value;
}

uint8_t rfmWriteReg(uint8_t regAddress, uint8_t regValue){
	CS_RFM = LOW;
		transferSPI((RFM_WRITE<<7) | regAddress);
		transferSPI(regValue);
	CS_RFM = HIGH;
	_delay_us(2);
	uint8_t readBack = rfmReadReg(regAddress)^(regValue);
	rfmWriteErrors += (readBack)? 1 : 0;
	if(readBack){
		LED_OR = HIGH;
		_delay_us(10);
		LED_OR = LOW;
	}
	return readBack; //(readBack)? readBack : 0; Seems I check for not working more than working.
}

void rfmReadFIFO(uint8_t *array){
	for(uint8_t i=0; i<64; i++){
		array[i] = rfmReadReg(0x7F);
	}
}



