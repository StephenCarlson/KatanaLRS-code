#ifndef RFM22B_H
#define RFM22B_H

//#include "KatanaLRS.h"

#define OPCONTROL1_REG	0x07
#define RFM_txon		3
#define RFM_rxon		2
#define RFM_pllon		1
#define RFM_xton		0

#define OP_CONT2_REG	0x08

#define GPIO_0_CFG		0x0B
#define GPIO_1_CFG		0x0C
#define GPIO_2_CFG		0x0D
#define GPIO_INPUT		0b00000011
#define GPIO_OUTPUT		0b00001010
#define GPIO_TXST		0b00010010
#define GPIO_RXST		0b00010101
#define GPIO_PMBLDET	0b00011001

#define IO_CFG_REG		0x0E
#define DIO_2			2
#define DIO_1			1
#define DIO_0			0

void radioMode(uint8_t);
uint8_t radioWriteReg(uint8_t, uint8_t);
uint8_t radioReadReg(uint8_t);
uint8_t radioReadRSSI(void);
void rfmReadFIFO(uint8_t *array);
uint16_t rfmReadIntrpts(void);
void rfmIntConfig(uint8_t, uint8_t);



void rfmIntConfig(uint8_t mode, uint8_t noiseFloor){
	if(mode == ENABLED){
		radioWriteReg(0x05, (1<<1)); // Enable Valid Packet Received Interrupt
		radioWriteReg(0x06, (1<<4)); // Enable RSSI Interrupt
		radioWriteReg(0x27, (noiseFloor+60)); // Configure RSSI for +30dBm Level Threshold
		
		// For 3500 bit/sec @ 5 kHz Dev, for detecting a 1750 Hz Tone
		radioWriteReg(0x1C, 0x2B);		// IF Filter Bandwidth
		radioWriteReg(0x1D, 0x44);		// AFC Loop Gearshift Override
		radioWriteReg(0x1E, 0x0A);		// AFC Timing Control
		radioWriteReg(0x1F, 0x03);		// Clock Recovery Gearshift Override
		radioWriteReg(0x20, 0x1E);		// Clock Recovery Oversampling Rate
		radioWriteReg(0x21, 0x20);		// Clock Recovery Offset 2
		radioWriteReg(0x22, 0x72);		// Clock Recovery Offset 1
		radioWriteReg(0x23, 0xB0);		// Clock Recovery Offset 0
		radioWriteReg(0x24, 0x00);		// Clock Recovery Timing Loop Gain 1
		radioWriteReg(0x25, 0xA2);		// Clock Recovery Timing Loop Gain 0
		radioWriteReg(0x2A, 0x1D);		// AFC Limiter
		radioWriteReg(0x34, 0x40);		// 64 nibble = 32 byte preamble
		radioWriteReg(0x35, 0x20);		// 0x35 need to detect 20bit preamble
		radioWriteReg(0x65, 0x60);		// Clock Recovery Timing Loop Gain 0
		radioWriteReg(0x71, 0x12);		// FSK via SDI
		radioWriteReg(0x72, 8);			// Frequency deviation setting to 5 kHz, total 10 kHz deviation, 5000/625
		
		radioWriteReg(0x07, 0);
		radioWriteReg(0x14, 7);			// R in T_WUT = 4 * M * 2^R / 32768, .015625*M ms for R=7 (2^7=128)
		radioWriteReg(0x15, 0);			// M[15:8]
		radioWriteReg(0x16, 128);		// M[7:0]
		radioWriteReg(0x19, 1);			// LDC in T_LDC_ON = 4 * LDC * 2^R / 32768
		radioWriteReg(0x07, (1<<5));	// Wake Up Timer Enabled
		
		// Preamble is 1750 -- 3500 bps, .032 sec -> 56 bits, can capture 16 bits in that time
		// .016 TLDC is .015625 * 32768 /4 
		
		EICRA = 0;
		EIMSK = (1<<INT0); //|(1<<INT0);
	} else{
		radioWriteReg(0x05, 0);
		radioWriteReg(0x06, 0);
		radioWriteReg(0x07, 0);
		EIMSK = 0;
	}
}



void radioMode(uint8_t mode){
	radioWriteReg(GPIO_0_CFG, GPIO_TXST);
	radioWriteReg(GPIO_1_CFG, GPIO_RXST);
	radioWriteReg(GPIO_2_CFG, GPIO_PMBLDET);
	//																								Add		R/W	Function/Desc		[7]			[6]			[5]			[4]			[3]			[2]			[1]			[0]		Reset Value
	radioWriteReg(0x06, 0x00);		// Disable all interrupts										06		R/W	Interrupt Enable 2	enswdet		enpreaval	enpreainval	enrssi		enwut		enlbd		enchiprdy	enpor	03h
	radioWriteReg(0x07, 0x01);		// Set READY mode
	radioWriteReg(0x09, 0x7F);		// Cap = 12.5pF
	radioWriteReg(0x0A, 0x05);		// Clk output is 2MHz

	radioWriteReg(0x0F, 0x70);		// NO ADC used
	radioWriteReg(0x10, 0x00);		// no ADC used
	radioWriteReg(0x12, 0x00);		// No temp sensor used
	radioWriteReg(0x13, 0x00);		// no temp sensor used

	radioWriteReg(0x70, 0x20);		// No manchester code, no data whiting, data rate < 30Kbps

	radioWriteReg(0x1C, 0x1D);		// IF filter bandwidth
	radioWriteReg(0x1D, 0x40);		// AFC Loop
	// radioWriteReg(0x1E, 0x0A);	// AFC timing

	radioWriteReg(0x20, 0xA1);		// clock recovery
	radioWriteReg(0x21, 0x20);		// clock recovery
	radioWriteReg(0x22, 0x4E);		// clock recovery
	radioWriteReg(0x23, 0xA5);		// clock recovery
	radioWriteReg(0x24, 0x00);		// clock recovery timing
	radioWriteReg(0x25, 0x0A);		// clock recovery timing

	// radioWriteReg(0x2A, 0x18);	// AFC Limiter
	radioWriteReg(0x2C, 0x00);		// OOK Counter
	radioWriteReg(0x2D, 0x00);		// OOK Counter
	radioWriteReg(0x2E, 0x00);		// Slicer Peak Hold

	radioWriteReg(0x6E, 0x27);		// 0x27 for 4800, 0x4E for 9600
	radioWriteReg(0x6F, 0x52);		// 0x52 for 4800, 0xA5 for 9600

	radioWriteReg(0x30, 0x00);		// Data access control <steve> 0x8C

	radioWriteReg(0x32, 0xFF);		// Header control

	radioWriteReg(0x33, 0x42);		// Header 3, 2, 1, 0 used for head length, fixed packet length, synchronize word length 3, 2,

	radioWriteReg(0x34, 64);		// 64 nibble = 32 byte preamble
	radioWriteReg(0x35, 0x20);		// 0x35 need to detect 20bit preamble
	radioWriteReg(0x36, 0x2D);		// synchronize word
	radioWriteReg(0x37, 0xD4);
	radioWriteReg(0x38, 0x00);
	radioWriteReg(0x39, 0x00);
	radioWriteReg(0x3A, '*');		// set tx header 3
	radioWriteReg(0x3B, 'E');		// set tx header 2
	radioWriteReg(0x3C, 'W');		// set tx header 1
	radioWriteReg(0x3D, 'S');		// set tx header 0
	// radioWriteReg(0x3E, 17);		// set packet length to 17 bytes (max size: 255 bytes)
	radioWriteReg(0x3E, 50);	// set packet length to PKTSIZE bytes (max size: 255 bytes)

	radioWriteReg(0x3F, '*');		// set rx header
	radioWriteReg(0x40, 'E');
	radioWriteReg(0x41, 'W');
	radioWriteReg(0x42, 'S');
	radioWriteReg(0x43, 0xFF);		// check all bits
	radioWriteReg(0x44, 0xFF);		// Check all bits
	radioWriteReg(0x45, 0xFF);		// check all bits
	radioWriteReg(0x46, 0xFF);		// Check all bits

	// radioWriteReg(0x56, 0x02);		// <steve> Something to do with I/Q Swapping

	radioWriteReg(0x6D, 0x00);		// Tx power to max

	radioWriteReg(0x79, 0x00);		// no frequency hopping
	radioWriteReg(0x7A, 0x00);		// no frequency hopping

	radioWriteReg(0x71, 0x12);		// FSK Async Mode, 

	radioWriteReg(0x72, 8);			// Frequency deviation setting to 5 kHz, total 10 kHz deviation, 5000/625

	radioWriteReg(0x73, 32);		// Frequency offset in 156.25 Hz Increments, Crawls between 4840 and 5040 Hz, 5740 May be a better
	radioWriteReg(0x74, 0x00);		// Frequency offset

	radioWriteReg(0x75, 0x53);		// frequency set to 434MHz
	radioWriteReg(0x76, 0x64);		// frequency set to 434MHz
	radioWriteReg(0x77, 0x00);		// frequency set to 434Mhz

	// radioWriteReg(0x5A, 0x7F);
	// radioWriteReg(0x59, 0x40);
	// radioWriteReg(0x58, 0x80);		// cpcuu[7:0], whatever this is

	// radioWriteReg(0x6A, 0x0B);
	// radioWriteReg(0x68, 0x04);
	radioWriteReg(0x1F, 0x03);		// Clock Recovery Value
	
	
	#if defined(RFM22B)

	#endif
}


uint8_t radioWriteReg(uint8_t regAddress, uint8_t regValue){
	CS_RFM = LOW;
		transferSPI((RFM_WRITE<<7) | regAddress);
		transferSPI(regValue);
	CS_RFM = HIGH;
	_delay_us(1);
	uint8_t readBack = radioReadReg(regAddress)^(regValue);
	rfmWriteErrors += (readBack)? 1 : 0;
	if(readBack){
		LED_OR = HIGH;
		_delay_us(10);
		LED_OR = LOW;
	}
	return readBack; //(readBack)? readBack : 0; Seems I check for not working more than working.
}




uint8_t radioReadReg(uint8_t regAddress){
	CS_RFM = LOW;
		transferSPI(regAddress);
		uint8_t value = transferSPI(0x00);
	CS_RFM = HIGH;
	_delay_us(1);
	return value;
}

uint8_t radioReadRSSI(void){
	if((radioReadReg(OPCONTROL1_REG)&(1<<RFM_rxon)) != (1<<RFM_rxon)){
		// printf("%X\n",radioReadReg(OPCONTROL1_REG));
		radioWriteReg(OPCONTROL1_REG, (1<<RFM_rxon));
		
		_delay_ms(10);
		
		// for(uint8_t i=0; i<255; i++){
			// if((radioReadReg(0x02)&0x01) != 1){
				// printf("Not Rx@ %u\n",i);
				// //break;
			// }
			// _delay_ms(1);
		// }
		// for(uint8_t i=0; i<255; i++){
			// if((radioReadReg(0x02)&0x01) == 0){
				// printf("Break@ %u\n",i);
				// break;
			// }
			// _delay_ms(1);
		// }
		
		radioWriteReg(OPCONTROL1_REG, 0); //(1<<RFM_xton));
	}
	
	return radioReadReg(0x26);
}

void rfmReadFIFO(uint8_t *array){
	for(uint8_t i=0; i<16; i++){
		array[i] = radioReadReg(0x7F);
	}
}

uint16_t rfmReadIntrpts(void){
	uint16_t rfmIntList = radioReadReg(0x03);
	rfmIntList |= radioReadReg(0x04)<<8;
	return rfmIntList;
}





























#ifdef NO_WE_ARE_NOT_COMPILING_THIS
	write(0x06, 0x00);		// Disable all interrupts
	write(0x07, 0x01);		// Set READY mode
	write(0x09, 0x7F);		// Cap = 12.5pF
	write(0x0A, 0x05);		// Clk output is 2MHz
	
	write(0x0B, 0xF4);		// GPIO0 is for RX data output
	write(0x0C, 0xEF);		// GPIO1 is TX/RX data CLK output
	write(0x0D, 0x00);		// GPIO2 for MCLK output
	write(0x0E, 0x00);		// GPIO port use default value
	
	write(0x0F, 0x70);		// NO ADC used
	write(0x10, 0x00);		// no ADC used
	write(0x12, 0x00);		// No temp sensor used
	write(0x13, 0x00);		// no temp sensor used
	
	write(0x70, 0x20);		// No manchester code, no data whiting, data rate < 30Kbps
	
	write(0x1C, 0x1D);		// IF filter bandwidth
	write(0x1D, 0x40);		// AFC Loop
	//write(0x1E, 0x0A);	// AFC timing
	
	write(0x20, 0xA1);		// clock recovery
	write(0x21, 0x20);		// clock recovery
	write(0x22, 0x4E);		// clock recovery
	write(0x23, 0xA5);		// clock recovery
	write(0x24, 0x00);		// clock recovery timing
	write(0x25, 0x0A);		// clock recovery timing
	
	//write(0x2A, 0x18);
	write(0x2C, 0x00);
	write(0x2D, 0x00);
	write(0x2E, 0x00);
	
	write(0x6E, 0x27);		// TX data rate 1
	write(0x6F, 0x52);		// TX data rate 0
	
	write(0x30, 0x8C);		// Data access control
	
	write(0x32, 0xFF);		// Header control
	
	write(0x33, 0x42);		// Header 3, 2, 1, 0 used for head length, fixed packet length, synchronize word length 3, 2,
	
	write(0x34, 64);		// 64 nibble = 32 byte preamble
	write(0x35, 0x20);		// 0x35 need to detect 20bit preamble
	write(0x36, 0x2D);		// synchronize word
	write(0x37, 0xD4);
	write(0x38, 0x00);
	write(0x39, 0x00);
	write(0x3A, '*');		// set tx header 3
	write(0x3B, 'E');		// set tx header 2
	write(0x3C, 'W');		// set tx header 1
	write(0x3D, 'S');		// set tx header 0
	//write(0x3E, 17);		// set packet length to 17 bytes (max size: 255 bytes)
	write(0x3E, PKTSIZE);	// set packet length to PKTSIZE bytes (max size: 255 bytes)
	
	write(0x3F, '*');		// set rx header
	write(0x40, 'E');
	write(0x41, 'W');
	write(0x42, 'S');
	write(0x43, 0xFF);		// check all bits
	write(0x44, 0xFF);		// Check all bits
	write(0x45, 0xFF);		// check all bits
	write(0x46, 0xFF);		// Check all bits
	
	write(0x56, 0x01);
	
	write(0x6D, 0x07);		// Tx power to max
	
	write(0x79, 0x00);		// no frequency hopping
	write(0x7A, 0x00);		// no frequency hopping
	
	write(0x71, 0x22);		// GFSK, fd[8]=0, no invert for TX/RX data, FIFO mode, txclk-->gpio
	
	write(0x72, 0x48);		// Frequency deviation setting to 45K=72*625
	
	write(0x73, 0x00);		// No frequency offset
	write(0x74, 0x00);		// No frequency offset
	
	write(0x75, 0x53);		// frequency set to 434MHz
	write(0x76, 0x64);		// frequency set to 434MHz
	write(0x77, 0x00);		// frequency set to 434Mhz
	
	write(0x5A, 0x7F);
	write(0x59, 0x40);
	write(0x58, 0x80);
	
	write(0x6A, 0x0B);
	write(0x68, 0x04);
	write(0x1F, 0x03);
	
	
	
	
 ItStatus1 = _spi_read(0x03); // read status, clear interrupt   
 ItStatus2 = _spi_read(0x04); 
  _spi_write(0x06, 0x00);    // no wakeup up, lbd, 
  _spi_write(0x07, RF22B_PWRSTATE_READY);      // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode 
  _spi_write(0x09, 0x7f);  // c = 12.5p   
  _spi_write(0x0a, 0x05); 
  _spi_write(0x0b, 0x12);    // gpio0 TX State
  _spi_write(0x0c, 0x15);    // gpio1 RX State 

  _spi_write(0x0d, 0xfd);    // gpio 2 micro-controller clk output 
  _spi_write(0x0e, 0x00);    // gpio    0, 1,2 NO OTHER FUNCTION. 
  
  _spi_write(0x70, 0x2C);    // disable manchest 
  
       // 9.6Kbps data rate
  _spi_write(0x6e, 0x27); //case RATE_384K 
  _spi_write(0x6f, 0x52); //case RATE_384K
  
  _spi_write(0x1c, 0x1A); // case RATE_384K
  _spi_write(0x20, 0xA1);//  0x20 calculate from the datasheet= 500*(1+2*down3_bypass)/(2^ndec*RB*(1+enmanch)) 
  _spi_write(0x21, 0x20); // 0x21 , rxosr[10--8] = 0; stalltr = (default), ccoff[19:16] = 0; 
  _spi_write(0x22, 0x4E); // 0x22    ncoff =5033 = 0x13a9 
  _spi_write(0x23, 0xA5); // 0x23 
  _spi_write(0x24, 0x00); // 0x24 
  _spi_write(0x25, 0x1B); // 0x25 
  _spi_write(0x1D, 0x40); // 0x25 
  _spi_write(0x1E, 0x0A); // 0x25 
  
  _spi_write(0x2a, 0x1e); 
   

  _spi_write(0x30, 0x8c);    // enable packet handler, msb first, enable crc, 

  _spi_write(0x32, 0xf3);    // 0x32address enable for headere byte 0, 1,2,3, receive header check for byte 0, 1,2,3 
  _spi_write(0x33, 0x42);    // header 3, 2, 1,0 used for head length, fixed packet length, synchronize word length 3, 2, 
  _spi_write(0x34, 0x06);    // 7 default value or   // 64 nibble = 32byte preamble 
  _spi_write(0x36, 0x2d);    // synchronize word 
 _spi_write(0x37, 0xd4); 
 _spi_write(0x38, 0x00); 
 _spi_write(0x39, 0x00); 
 _spi_write(0x3a, RF_Header[0]);    // tx header 
 _spi_write(0x3b, RF_Header[1]); 
 _spi_write(0x3c, RF_Header[2]); 
 _spi_write(0x3d, RF_Header[3]); 
 _spi_write(0x3e, 8);    // total tx 3 byte 
 
  
  
    //RX HEADER
 _spi_write(0x3f, RF_Header[0]);   // check hearder 
 _spi_write(0x40, RF_Header[1]); 
 _spi_write(0x41, RF_Header[2]); 
 _spi_write(0x42, RF_Header[3]); 
 _spi_write(0x43, 0xff);    // all the bit to be checked 
 _spi_write(0x44, 0xff);    // all the bit to be checked 
 _spi_write(0x45, 0xff);    // all the bit to be checked 
 _spi_write(0x46, 0xff);    // all the bit to be checked 
  

  
  #if (BOOSTER == 0)
  _spi_write(0x6d, 0x07); // 7 set power max power 
  #else
  _spi_write(0x6d, 0x06); // 6 set power 50mw for booster 
  #endif
  
  _spi_write(0x79, 0x00);    // no hopping 
  
  #if (BAND== 0)
  _spi_write(0x7a, 0x06);    // 60khz step size (10khz x value) // no hopping 
  #else
  _spi_write(0x7a, 0x05); // 50khz step size (10khz x value) // no hopping 
  #endif

  _spi_write(0x71, 0x23); // Gfsk, fd[8] =0, no invert for Tx/Rx data, fifo mode, txclk -->gpio 
  _spi_write(0x72, 0x30); // frequency deviation setting to 19.6khz (for 38.4kbps)
 
  _spi_write(0x73, 0x00);   
  _spi_write(0x74, 0x00);    // no offset 
 

  //band 435.000
 
 #if (BAND== 0)
 _spi_write(0x75, 0x53);
 #else 
_spi_write(0x75, 0x55);  //450 band 
 #endif
 
 _spi_write(0x76, 0x7D);    
 _spi_write(0x77, 0x00); 
	
	
	
	
	
	char configArray[] = {					// 21 Values
		0xC0,		// 	1D	THRESH_TAP	12g		62.5 mg/LSB unsigned 0xFF = +16 g
		0x00,		// 	1E	OFSX					15.6 mg/LSB signed 0x7F = +2 g
		0x00,		// 	1F	OFSY
		0x00,		// 	20	OFSZ
		0x50,		// 	21	DUR			60ms	625 uS/LSB		Max time/width of tap peak
		0x28,		// 	22	Latent		50ms	1.25 ms/LSB		No other peak until after this
		0x50,		// 	23	Window		100ms	1.25 ms/LSB 	Period after latent to make a second peak
		0x40,		// 	24	THRESH_ACT	1.5g		62.5 mg/LSB unsigned	Exceed value to flag activity
		0x04,		// 	25	THRESH_INACT	.25g	62.5 mg/LSB unsigned	Stay below for TIME_INACT for inactivity
		0x05,		// 	26	TIME_INACT	5sec	1 sec/LSB
		0b11111111,	// 	27	ACT_INACT_CTL 		ACT[dc/AC][X|Y|Z] INACT[dc/AC][X|Y|Z]
		0x08,		// 	28	THRESH_FF	500mg	62.5 mg/LSB unsigned sqrt(x^2+y^2+z^2)
		0x14,		// 	29	TIME_FF		100ms	5 ms/LSB
		0b00001111,	// 	2A	TAP_AXES				0[7:4], [Suppress] Enable[X|Y|Z]
		0x00,		// 	2B	ACT_TAP_STATUS READ-ONLY [0] Activity[X|Y|Z] [Asleep] Tap[X|Y|Z]
		ADXL_RATE,	// 	2C	BW_RATE				0[7:5], [Low Power] RateCode[3:0]
		0b00100000,	// 	2D	POWER_CTL			0[7:6], [Link][AutoSleep][Measure][Sleep] WakeRate[1:0]
		0b00000000,	// 	2E	INT_ENABLE			[DataReady][1 Tap][2 Taps][Activity][Inactivity][FreeFall][Watermark][OverRun]
		0b10000011,	// 	2F	INT_MAP				[DataReady][1 Tap][2 Taps][Activity][Inactivity][FreeFall][Watermark][OverRun]
		0x00,		// 	30	INT_SOURCE READ-ONLY
		0b00101011	// 	31	DATA_FORMAT  FULL_RES bit set
	};


	
	
	
	
	
	
	
	
	
void beacon_send(void)
{
  Green_LED_ON
  ItStatus1 = spiReadRegister(0x03);   // read status, clear interrupt
  ItStatus2 = spiReadRegister(0x04);
  spiWriteRegister(0x06, 0x00);    // no wakeup up, lbd,
  spiWriteRegister(0x07, RF22B_PWRSTATE_READY);      // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode
  spiWriteRegister(0x09, 0x7f);  // (default) c = 12.5p
  spiWriteRegister(0x0a, 0x05);
  spiWriteRegister(0x0b, 0x12);    // gpio0 TX State
  spiWriteRegister(0x0c, 0x15);    // gpio1 RX State
  spiWriteRegister(0x0d, 0xfd);    // gpio 2 micro-controller clk output
  spiWriteRegister(0x0e, 0x00);    // gpio    0, 1,2 NO OTHER FUNCTION.

  spiWriteRegister(0x70, 0x2C);    // disable manchest

  spiWriteRegister(0x30, 0x00);    //disable packet handling

  spiWriteRegister(0x79, 0);    // start channel

  spiWriteRegister(0x7a, 0x05);   // 50khz step size (10khz x value) // no hopping

  spiWriteRegister(0x71, 0x12);   // trclk=[00] no clock, dtmod=[01] direct using SPI, fd8=0 eninv=0 modtyp=[10] FSK
  spiWriteRegister(0x72, 0x02);   // fd (frequency deviation) 2*625Hz == 1.25kHz

  spiWriteRegister(0x73, 0x00);
  spiWriteRegister(0x74, 0x00);    // no offset

  rfmSetCarrierFrequency(bind_data.beacon_frequency);

  spiWriteRegister(0x6d, 0x07);   // 7 set max power 100mW

  delay(10);
  spiWriteRegister(0x07, RF22B_PWRSTATE_TX);    // to tx mode
  delay(10);
  beacon_tone(500, 1);

  spiWriteRegister(0x6d, 0x04);   // 4 set mid power 15mW
  delay(10);
  beacon_tone(250, 1);

  spiWriteRegister(0x6d, 0x00);   // 0 set min power 1mW
  delay(10);
  beacon_tone(160, 1);

  spiWriteRegister(0x07, RF22B_PWRSTATE_READY);
  Green_LED_OFF
}
	
	
void setModemRegs(struct rfm22_modem_regs* r)
{

  spiWriteRegister(0x1c, r->r_1c);
  spiWriteRegister(0x1d, r->r_1d);
  spiWriteRegister(0x1e, r->r_1e);
  spiWriteRegister(0x20, r->r_20);
  spiWriteRegister(0x21, r->r_21);
  spiWriteRegister(0x22, r->r_22);
  spiWriteRegister(0x23, r->r_23);
  spiWriteRegister(0x24, r->r_24);
  spiWriteRegister(0x25, r->r_25);
  spiWriteRegister(0x2a, r->r_2a);
  spiWriteRegister(0x6e, r->r_6e);
  spiWriteRegister(0x6f, r->r_6f);
  spiWriteRegister(0x70, r->r_70);
  spiWriteRegister(0x71, r->r_71);
  spiWriteRegister(0x72, r->r_72);
}



void rfmSetCarrierFrequency(uint32_t f)
{
  uint16_t fb, fc, hbsel;
  if (f < 480000000) {
    hbsel = 0;
    fb = f / 10000000 - 24;
    fc = (f - (fb + 24) * 10000000) * 4 / 625;
  } else {
    hbsel = 1;
    fb = f / 20000000 - 24;
    fc = (f - (fb + 24) * 20000000) * 2 / 625;
  }
  spiWriteRegister(0x75, 0x40 + (hbsel?0x20:0) + (fb & 0x1f));
  spiWriteRegister(0x76, (fc >> 8));
  spiWriteRegister(0x77, (fc & 0xff));
}

void init_rfm(uint8_t isbind)
{
  ItStatus1 = spiReadRegister(0x03);   // read status, clear interrupt
  ItStatus2 = spiReadRegister(0x04);
  spiWriteRegister(0x06, 0x00);    // disable interrupts
  spiWriteRegister(0x07, RF22B_PWRSTATE_READY); // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode
  spiWriteRegister(0x09, 0x7f);   // c = 12.5p
  spiWriteRegister(0x0a, 0x05);
  spiWriteRegister(0x0b, 0x12);    // gpio0 TX State
  spiWriteRegister(0x0c, 0x15);    // gpio1 RX State
  spiWriteRegister(0x0d, 0xfd);    // gpio 2 micro-controller clk output
  spiWriteRegister(0x0e, 0x00);    // gpio    0, 1,2 NO OTHER FUNCTION.

  if (isbind) {
    setModemRegs(&bind_params);
  } else {
    setModemRegs(&modem_params[bind_data.modem_params]);
  }

  // Packet settings
  spiWriteRegister(0x30, 0x8c);    // enable packet handler, msb first, enable crc,
  spiWriteRegister(0x32, 0x0f);    // no broadcast, check header bytes 3,2,1,0
  spiWriteRegister(0x33, 0x42);    // 4 byte header, 2 byte synch, variable pkt size
  spiWriteRegister(0x34, 0x0a);    // 10 nibbles (40 bit preamble)
  spiWriteRegister(0x35, 0x2a);    // preath = 5 (20bits), rssioff = 2
  spiWriteRegister(0x36, 0x2d);    // synchronize word 3
  spiWriteRegister(0x37, 0xd4);    // synchronize word 2
  spiWriteRegister(0x38, 0x00);    // synch word 1 (not used)
  spiWriteRegister(0x39, 0x00);    // synch word 0 (not used)

  uint32_t magic = isbind ? BIND_MAGIC : bind_data.rf_magic;
  for (uint8_t i=0; i<4; i++) {
    spiWriteRegister(0x3a + i, (magic >> 24) & 0xff);   // tx header
    spiWriteRegister(0x3f + i, (magic >> 24) & 0xff);   // rx header
    magic = magic << 8; // advance to next byte
  }

  spiWriteRegister(0x43, 0xff);    // all the bit to be checked
  spiWriteRegister(0x44, 0xff);    // all the bit to be checked
  spiWriteRegister(0x45, 0xff);    // all the bit to be checked
  spiWriteRegister(0x46, 0xff);    // all the bit to be checked

  if (isbind) {
    spiWriteRegister(0x6d, BINDING_POWER);
  } else {
    spiWriteRegister(0x6d, bind_data.rf_power);
  }

  spiWriteRegister(0x79, 0);

  spiWriteRegister(0x7a, bind_data.rf_channel_spacing);   // channel spacing

  spiWriteRegister(0x73, 0x00);
  spiWriteRegister(0x74, 0x00);    // no offset

  rfmSetCarrierFrequency(isbind ? BINDING_FREQUENCY : bind_data.rf_frequency);

}














#endif // NO_WE_ARE_NOT_COMPILING_THIS
	
#endif // RFM22B_H