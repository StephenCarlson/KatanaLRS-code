// Katana Long-Range-System, by Stephen Carlson, March 2014
// Use Notepad++ with Tab=4

// Pins Configuration:
//    -- Package --		  --PCB Label--
// PDIP-28		TQFP-32		K-LRSv1	Name	Function(s)		Prime Assignment			Code Label
//    14		   12		LED_OR	PB0		LED/PPM*		ICP1 Capture if Tx Module	LED_OR, PPM_INPUT
//    15		   13		LED_BL	PB1		LED/PPM*		OC1A Waveform PPM if Rx		LED_BL, PPM_OUTPUT 
//    16		   14		/CS_RF	PB2		/SS				SPI CS for RFM				CS_RFM
//    17		   15		MOSI	PB3		MOSI			SPI MOSI
//    18		   16		MISO	PB4		MISO			SPI MISO
//    19		   17		SCK		PB5		SCK				SPI SCK
//     9		   --		-		PB6		XTAL1
//    10		   --		-		PB7		XTAL2
//    23		   23		P1		PC0		PWM/GPIO*/ADC0	Pin 1						
//    24		   24		P2		PC1		PWM/GPIO*/ADC1	Pin 2						
//    25		   25		P3		PC2		PWM/GPIO*/ADC2	Pin 3						
//    26		   26		P4		PC3		PWM/GPIO*/ADC3	Pin 4						
//    27		   27		SDA		PC4		SDA/GPIO^/ADC4	I2C SDA
//    28		   28		SCL		PC5		SCL/GPIO^/ADC5	I2C SCL
//    1			   29		/RESET	PC6		/Reset			Reset
//    2			   30		RX		PD0		RXD/GPIO*		UART Rx
//    3			   31		TX		PD1		TXD/GPIO		UART Tx
//    4			   32		RF_INT	PD2		INT0			RFM Interrupt to AVR		RFM_INT
//    5			   1		PMBL_D	PD3		INT1			RFM Preamble Detect to AVR	RFM_PMBL
//    6			   2		P5		PD4		PWM/GPIO*		Pin 5						
//    11		   9		P6		PD5		PWM/GPIO*		Pin 6 w/OC0B				
//    12		   10		P7		PD6		PWM/GPIO*		Pin 7 w/OC0A				
//    13		   11		P8		PD7		PWM/GPIO*		Pin 8						
//    --		   19		ADC6	ADC6	ADC6			Battery Voltage 50% 		ADC_VBAT
//    --		   22		ADC7	ADC7	ADC7			Input Voltage 50%			ADC_VIN
//											* = 1k Series Resistor
//											^ = 4.7k Pull-up Resistor

//           76543210	7		6		5		4		3		2		1		0
// DDRB = 0b --SsSSLL	XTAL2	XTAL1	SCK		MISO	MOSI	CS_RFM	LED_BL	LED_OR
// DDRC = 0b -0CC____	--		Reset	SCL		SDA		P4		P3		P2		P1
// DDRD = 0b ____ii10	P8		P7		P6		P5		RFM_PBL	RF_INT	TXD		RXD

// Legend: - N/A		_ PWM/GPIO Available	S/s SPI		C I2C		i Interrupt		1/0 Output/Input

// =======================================================================


// Timers 					Description
// Timer 0 (8-bit)			System Time Counter (1 ms)
// 		OC0A	PD6			Pin 7, not really available
// 		OC0B	PD5			Pin 6, not really available

// Timer 1 (16-bit)			R/C Output Generator, could be used as system time counter?
//		OC1A	PB1			PPM Output Line
//		OC1B	PB2			/CS Line, can't use

// Timer 2 (8-bit)			AFSK on ELT
//		OC2A	PB3			MOSI, can create tones for ELT on this
//		OC2B	PD3			Unconnected spare line, use for piezo buzzer? Free otherwise


#include "KatanaLRS.h"


// Interrupt Vectors (Listed in Priority Order)
// RFM Module Interrupt Input
ISR(INT0_vect){ // Configured for Falling-Edge
	// LED_OR = HIGH;
	sys.intSrc.rfm = 1;
	timestampPacketRxd = timer1us_p + (TCNT1>>1);
	// dlChannel = (dlChannel < (sizeof(dlFreqList)-1))? dlChannel+1 : 0; 
	//EIMSK = 0;
}

// Sleep-Immune UART Wake Mechanism
ISR(PCINT2_vect){
	sys.intSrc.uart = 1;
	PCICR = 0;
	PCMSK2 = 0;
}

ISR(WDT_vect){
	sys.intSrc.wdt = 1;
}

// ELT Audio Generator - See elt.c for this handler, included here for reference
// ISR(TIMER2_COMPA_vect){}

// LRS Packet Timer
// ISR(TIMER2_COMPA_vect  OVF_vect?){
	
// }

// PPM Input Parser
// ISR(TIMER1_CAPT_vect){
	// This is where the Input PPM steam is parsed
	
	
// }

// Servo PPM/PWM Generator
ISR(TIMER1_OVF_vect ){
	timer1us_p += (ICR1>>1);
	switch(ch){
		case 0:
			PWM_1 = HIGH;
			ICR1 = pwmValues[ch];
			pwmFrameSum = pwmValues[ch]; // Note the subtle difference, =, not +=
			ch+=1;
			break;
		case 1:
			PWM_1 = LOW;
			PWM_2 = HIGH;
			ICR1 = pwmValues[ch];
			pwmFrameSum += pwmValues[ch];
			ch+=1;
			break;
		case 2:
			PWM_2 = LOW;
			PWM_3 = HIGH;
			ICR1 = pwmValues[ch];
			pwmFrameSum += pwmValues[ch];
			ch+=1;
			break;
		case 3:
			PWM_3 = LOW;
			PWM_4 = HIGH;
			ICR1 = pwmValues[ch];
			pwmFrameSum += pwmValues[ch];
			ch+=1;
			break;
		case 4:
			PWM_4 = LOW;
			PWM_5 = HIGH;
			ICR1 = pwmValues[ch];
			pwmFrameSum += pwmValues[ch];
			ch+=1;
			break;
		case 5:
			PWM_5 = LOW;
			PWM_6 = HIGH;
			ICR1 = pwmValues[ch];
			pwmFrameSum += pwmValues[ch];
			ch+=1;
			break;
		case 6:
			PWM_6 = LOW;
			PWM_7 = HIGH;
			ICR1 = pwmValues[ch];
			pwmFrameSum += pwmValues[ch];
			ch+=1;
			break;
		case 7:
			PWM_7 = LOW;
			PWM_8 = HIGH;
			ICR1 = pwmValues[ch];
			pwmFrameSum += pwmValues[ch];
			ch+=1;
			// printf("Sum: %u\n",pwmFrameSum); Getting 14000
			break;
		case 8:
			PORTC &= ~(0x0F);
			PORTD &= ~(0xF0);
			// pwmFrameSum = (pwmFrameSum > 40000)? 0 : pwmFrameSum;
			// ICR1 = (40000 - pwmFrameSum); // Hazard if negative!
			ICR1 = (pwmFrameSum<36000)? (40000 - pwmFrameSum) : 4000; // Prototype, test this sometime
			// pwmFrameSum = 0;
			ch = 0;
			OCR1A = 800;	// Produce a 400 us low pulse
			TCCR1A |= _BV(COM1A1)|_BV(COM1A0); // Including COM1A0 Flips to behavior to Idle-High
			break;
		default:
			ch = 8;
	}
	// LED_OR = LOW;
}

// System Timer - 1ms Resolution
ISR(TIMER0_COMPA_vect){
	//LED_OR = HIGH;
	sys.intSrc.timer0 = 1;
	// timer1min += (timer1ms >= 60000)? 1 : 0; // Rolls at 1000 hours
	// timer1ms = (timer1ms >= 60000)? 0 : timer1ms+1; // 60 sec
	timer1ms += 1;
	//LED_OR = LOW;
}

// UART Command Handler
ISR(USART_RX_vect){
	sys.intSrc.uart = 1;
	sys.monitorMode = 0;
	
	uint8_t command = UDR0;
	
	switch(command){
		case 'B':
			printf("Battery: %u\n", volt.atMega);
			break;
		case '+':
			freqOffset += 5;
			break;
		case '-':
			freqOffset -= 5;
			break;
		case 'M':
			sys.monitorMode = 1;
			break;
		case '?':
			printHelpInfo();
			break;
		case 'J':
			printRegisters();
			break;
		case '1':
			configFlags.wdtSlpEn ^= 1;
			printf("Sleep: ");
			if(configFlags.wdtSlpEn) printf("Enabled\n");
			else printf("Disabled\n");
			break;
		case '`':
			eeprom_update_byte((uint8_t*)EEPROM_START,(*(uint8_t*) &configFlags));
			break;
		default:
			break;
	}	
}

// Low-Noise ADC Exit Handler
ISR(ADC_vect){
	sleep_disable();
}


// Main Program
int main(void){
	setup();

	while(1){		
		loop();
	}
}

void setup(void){
	uint8_t startStatus = atMegaInit();								// 
	sys.state = ACTIVE;
	
	// Restart Peripherals											// ~105 ms
	rfmReset();
	rfmMode(IDLE_STANDBY);
	
	// Tasks and Routines
	printf("\n\nKatanaLRS v1\nBy Steve Carlson %s %s\n\n",__TIME__,__DATE__);
	printf("Reset Source: "); //%X\n",startStatus); //%X\n", startStatus);
	if(startStatus&WDRF) printf("WatchDog\t"); // From iom328p.h in AVR Include Folder
	if(startStatus&BORF) printf("BrownOut\t");
	if(startStatus&EXTRF) printf("External\t");
	if(startStatus&PORF) printf("PowerOn\t");
	printf("\n\n");
	//	WDRF BORF EXTRF PORF
	
	flashOrangeLED(5,10,40); 										// 250 ms
	sys.monitorMode = 1;
	
	updateVolts(SLOW);
	
	printf("Device ID Check: ");
	if(deviceIdCheck()){
		printf("OK\n");
	} else{
		printf("FAILED!\n");
		for(uint8_t i=0; i<10; i++){
			flashOrangeLED(5,10,40);
			rfmReset();
			rfmMode(IDLE_STANDBY);
			if(deviceIdCheck()) break;
			// if(i >= 9) sys.state = FAILSAFE; 
		}
	}	
	
	*((uint8_t*) &configFlags) = eeprom_read_byte((const uint8_t*) EEPROM_START);
	eeprom_read_block(pwmFailsafes,(void *)(EEPROM_START+1),sizeof(pwmFailsafes));
	
	// Application Warm-up
	TIMSK1 = _BV(TOIE1);
	printf("\nFailsafes: ");
	for(uint8_t i=0; i<CHANNELS; i++){
		// pwmFailsafes[i] = (pwmFailsafes[i]<1720)? 1720 : (pwmFailsafes[i]>4275)? 4275 : pwmFailsafes[i]; // [1720:4275]
		pwmFailsafes[i] = (pwmFailsafes[i]<1720 || pwmFailsafes[i]>4275)? ((i==2)? 2000 : 3000) : pwmFailsafes[i];
		pwmValues[i] = 1500<<1;
		printf("%4d ",pwmFailsafes[i]>>1);
	}
	printf("\n");
	
	gps.lat  = 89999999;
	gps.lon  = 179999999;
	gps.time = 235959;
	gps.alt  = 1000;
	gps.sats = 8;
	gps.hdop = 15;
	
	// Console Usage Hints
	printHelpInfo();
	
	
	for(uint8_t i=0; i<100 && !rfmMode(RX_FHSS_LRS); i++) _delay_us(100);
	
	// for(uint8_t i=0; i<20 && !rfmMode(TX_PACKET_4800); i++) _delay_ms(1);
	// for(uint8_t i=0; i<sizeof(rfmConfig_FhssConfig)/2;i++) \
		// rfmWriteReg(rfmConfig_FhssConfig[i][0],rfmConfig_FhssConfig[i][1]);
	
	rfmWriteReg(0x05, 0x02);	
	// rfmWriteReg(0x05, 0x04);	
	rfmWriteReg(0x06, 0);	
	
	dlChannel = 5;
	rfmSetLrsChannel(dlFreqList[dlChannel]);
	
	// rfmSetManualFreq(0x6400);
	
	for(uint8_t i=0; i<64; i++){
		rfmFIFO[i] = i; //(i&0x01)? 0x00:0xFF;
	}
	
	// Development
	
	// EICRA = _BV(ISC01); // Falling-Edge
	// EIMSK = _BV(INT0);
}

void loop(void){
	static uint8_t eltTransmitCount = 0;
	// static uint16_t failsafeCounter = 0;
	
	static uint16_t manualFreq = 19800;
	
	static uint32_t time1sec;
	static uint32_t time5sec;
	static uint32_t timeoutFhssPkt;
	// static uint32_t periodFhssPkt = 22222;
	const uint32_t periodFhssPkt = 22222;
	// static int16_t freqOffset = 0;
	static int16_t afcValue = 0;
	
	uint16_t rfmIntList = 0;
	#define RFM_INT_VALID_SYNC (1<<(7))
	#define RFM_INT_PKT_RXED	(1<<(1))<<8
	
	
	if(sys.intSrc.wdt){
		sys.intSrc.wdt = 0;
		_delay_ms(1);
	}
		
	switch(sys.state){
	/*
		case DOWN:
				wdtIntConfig(DISABLED,0);
				noiseFloor = rfmGetRSSI();
				updateVolts(SLOW);
				rfmIntList = rfmGetInterrupts();
				if(rfmIntList&RFM_INT_VALID_SYNC){
					printf("Rx in DOWN Works!\t%u\t%X\n",noiseFloor,rfmIntList);
					sys.state = BEACON;
				} else sys.state = (sys.batteryState || sys.powerState)? SLEEP : DOWN;
				if(sys.state != DOWN){
					printState();
					break;
				}
				rfmMode(RX_TONE_LDC);
				rcOutputs(DISABLED);
				sys.statusLEDs = ENABLED;
				uartIntConfig(DISABLED);
				systemSleep(9);
			break;
		case SLEEP:
				wdtIntConfig(ENABLED,9);
				updateVolts(SLOW);
				eltFullSequence();
				_delay_ms(2);
				rfmIntList = rfmGetInterrupts();
				if(rfmIntList&RFM_INT_VALID_SYNC){
					sys.state = BEACON;
				} else sys.state = (sys.batteryState || sys.powerState)? SLEEP : DOWN;
				if(sys.state != SLEEP){
					printState();
					break;
				}
				
				rfmMode(RX_TONE_LDC);
				rcOutputs(DISABLED);
				sys.statusLEDs = ENABLED;
				uartIntConfig(DISABLED);
				systemSleep(9);
			break;
		case BEACON:
				wdtIntConfig(ENABLED,8);
				updateVolts(FAST);
				eltFullSequence();
				_delay_ms(2);
				sys.state = (eltTransmitCount > 10)? SLEEP : BEACON;
				if(sys.state != BEACON){
					eltTransmitCount = 0;
					printState();
					_delay_ms(30);
					break;
				}
				eltTransmitCount += 1;
				rfmMode(IDLE_TUNE);
				rcOutputs((sys.powerState)? ENABLED : DISABLED);
				sys.statusLEDs = ENABLED;
				uartIntConfig(DISABLED);
				
				_delay_ms(30);
			break;
			
			*/
		case ACTIVE:
				wdtIntConfig(ENABLED, 5); // 0.5 sec timeout
				//sys.state = ((sys.powerState == 0))? DOWN : SLEEP; //FAILSAFE; //sticksCentered() && 
				//if(sys.state != ACTIVE){
				//	printState();
				//	_delay_ms(30);
				//	break;
				//}
				cli();
					// uint32_t currentTime1ms = timer1ms; // Avoid Race Condition, two bytes, one can change if interrupted in this process.
					uint32_t currentTime1us = timer1us_p + (TCNT1>>1);
					// uint32_t packetRxTimestamp = timestampPacketRxd; // To eliminate possible race conditions?, and speedup the system by copying to a register.
				sei();
				uint32_t currentTime1ms = currentTime1us/1000;
				
				if(currentTime1ms > time5sec){
					time5sec = currentTime1ms + 30000;
					// manualFreq = 3200 + (uint16_t)((((uint32_t)dlFreqList[dlChannel]*1728) + 5) / 10);
					// dlChannel = (dlChannel < (sizeof(dlFreqList)-1))? dlChannel+1 : 0;
					// rfmSetLrsChannel(dlFreqList[dlChannel]);
					
					// printf("DL %d\n",dlChannel);
					// printf("~%X,%X,%X\n",rfmReadReg(0x02),rfmReadReg(0x04),rfmReadReg(0x07));
				}
				
				if(0){ // currentTime1ms > time1sec){
					time1sec = currentTime1ms+2000;
					RFM_PMBL = HIGH;
					// time1sec = currentTime1ms+500;
					// updateVolts(FAST);
					rcOutputs(ENABLED);
					sys.statusLEDs = ENABLED;
					
					// Stability Check, register checked inspired by OpenLRSng
					if(rfmReadReg(0x2D) != 0x00){
						printf("RFM Bricked, Resetting!\n");
						
						rfmReset();
						for(uint8_t i=0; (i<200) && !rfmMode(IDLE_STANDBY); i++) _delay_us(100);
						for(uint8_t i=0; (i<100) && !rfmMode(RX_FHSS_LRS); i++) _delay_us(100);
						rfmWriteReg(0x05, 0x02);
						rfmWriteReg(0x06, 0);
						rfmSetLrsChannel(dlFreqList[dlChannel]);
					}
					
					
					// Receive Enable, Offset Shifting
					rfmSetRxTxSw(RFM_rxon);
					rfmWriteReg(0x07, RFM_rxon | 0x02);
					
					
					
					// uint16_t value = 3200 + (uint16_t)((((uint32_t)dlFreqList[dlChannel]*1728) + 5) / 10) + afcValue;
					// manualFreq += ((value>>5) - (manualFreq>>5));
					// manualFreq += ((currentTime1ms > timestampPacketRxd) && (currentTime1ms-timestampPacketRxd)<2000)? ((afcValue>>1)) : 0; // - (manualFreq>>2));
					// rfmSetManualFreq(manualFreq);
					// printf("FQ\t%u\n",manualFreq);
					// manualFreq = 3200 + (uint16_t)((((uint32_t)dlFreqList[dlChannel]*1728) + 5) / 10);
					freqOffset += ((currentTime1us > timestampPacketRxd) && (currentTime1us-timestampPacketRxd)<2000000)? ((afcValue>>2)) : 0;
					rfmSetManualFreq(manualFreq+freqOffset);
					printf("FQ\t%d\n",freqOffset);
					
					/*
					// Transmit Dragonlink Packet
					rfmSetRxTxSw(0);
					rfmSetTxPower(0);
					rfmClearTxFIFO();
					rfmWriteReg(0x05, 0x04);
					rfmWriteFIFOArray(rfmFIFO,20);
					rfmWriteReg(0x3E,20);
					rfmSetRxTxSw(RFM_txon);
					rfmWriteReg(0x07, RFM_txon | 0x02);
					for(uint8_t i=0; (i<200) && !RFM_INT; i++){ // Counting on FHSS Config to set Reg 0x05 to 0x02
						// rfmGetInterrupts();
						// rfmGetRxTx(RFM_txon);
						_delay_ms(1);
						if(i==199) printf("P3\n");
					}
					rfmSetRxTxSw(0);
					rfmWriteReg(0x07, 0x02);
					rfmGetInterrupts();
					*/
					

					RFM_PMBL = LOW;
				}
				
				if(RFM_INT){ //0){ //
					RFM_PMBL = HIGH;
					
					// Receive
					rfmIntList = rfmGetInterrupts();
					dlChannel = (dlChannel < (sizeof(dlFreqList)-1))? dlChannel+1 : 0; // Incremented in interrupt handler
					manualFreq = 3200 + (uint16_t)((((uint32_t)dlFreqList[dlChannel]*1728) + 5) / 10);
					rfmSetManualFreq(manualFreq+freqOffset);
					// rfmSetLrsChannel(dlFreqList[dlChannel]);
					uint8_t payloadSize = rfmReadReg(0x4B);
					rfmReadFIFOn(rfmFIFO,15);
					// int16_t afcValue = (((int16_t)((int8_t)rfmReadReg(0x2B)))<<2) | (rfmReadReg(0x2C)>>6);
					afcValue = (((int16_t)((int8_t)rfmReadReg(0x2B)))<<2) | (rfmReadReg(0x2C)>>6);
					// printf("DL %d\tAFC %d\nD ",dlChannel,afcValue);
					// for(uint8_t i=0; i<20; i++){
						// printf("%X ",rfmFIFO[i]); //0x00);
					// }
					// printf("\n");
					rfmClearRxFIFO();
					
					// Clean Up
					rfmWriteReg(0x05, 0x02);
					rfmSetRxTxSw(RFM_rxon);
					rfmWriteReg(0x07, RFM_rxon | 0x02);
					
					// periodFhssPkt += (timestampPacketRxd) - periodFhssPkt;
					timeoutFhssPkt = timestampPacketRxd + periodFhssPkt + (periodFhssPkt>>2);
					
					// Process Array - Extract Commands
					// Media			sS012345 67sS0123 4567sS01 234567sS 01234567 sS01234567sS01234567sS
					// rfmFIFO			543210Ss.3210Ss76.10Ss7654.Ss765432.76543210.
					// dlReadArray		76543210/76543210/76543210/76543210
					// DL Field Tags	MSBs     CH1      CH2      CH3
					// 
					// const uint8_t DL_BYTES = 10;
					#define DL_BYTES 10
					uint8_t dlReadArray[DL_BYTES];
					// uint8_t k=0;
					// for(uint8_t j=0; j<(DL_BYTES-4); j++){
						// dlReadArray[k]   = ((rfmFIFO[j*5  ]&0xFC)>>2 | rfmFIFO[j*5+1]<<6);
						// dlReadArray[k+1] = ((rfmFIFO[j*5+1]&0xF0)>>4 | rfmFIFO[j*5+2]<<4);
						// dlReadArray[k+2] = ((rfmFIFO[j*5+2]&0xC0)>>6 | rfmFIFO[j*5+3]<<2);
						// dlReadArray[k+3] =                             rfmFIFO[j*5+4];
						// k+=4;
					// }
					for(uint8_t i=0; i<DL_BYTES; i++){
						uint8_t offset = (i>>2)+i;
						uint8_t shift =  ((i&0x03)+1)<<1;
						dlReadArray[i] = (rfmFIFO[offset]>>(shift)) | (rfmFIFO[offset+1]<<(8-shift));
					}
					#define DL_CHANNELS 7
					for(uint8_t i=0; i<DL_CHANNELS; i++){
						// pwmValues[i] = ( ((dlReadArray[0]<<(8-i))&0x100) | dlReadArray[i] )*4 + 2000 - 22; // for [989:2011]us
						pwmValues[i] = ( ((dlReadArray[0]<<(8-i))&0x100) | dlReadArray[i+1] )*5 + 2000 - 280; // [860:2137]us; [1000:2000]us -> [56:456]
						printf("%4u ",pwmValues[i]>>1);
					}
					
					/*
					// printf("%X,",dlReadArray[0]);
					for(uint8_t i=0; i<7; i++){
						//pwmValues[i] = (( ((dlReadArray[0]>>i)&0x01)<<8 | dlReadArray[i+1] )<<1+1000)<<1;;
						pwmValues[i] = (((uint16_t) dlReadArray[i+1]<<2) + 1000)<<1;
						printf("%u,",pwmValues[i]>>1);
					} */
					
					// printf("CH %2u\t%3d\t%lu\n",(dlChannel),afcValue,periodFhssPkt);
					printf("CH %2u\t%3d\n",(dlChannel),afcValue);
					RFM_PMBL = LOW;
				}
				
				if(currentTime1us  > timeoutFhssPkt){
					dlChannel = (dlChannel < (sizeof(dlFreqList)-1))? dlChannel+1 : 0; // Incremented in interrupt handler
					manualFreq = 3200 + (uint16_t)((((uint32_t)dlFreqList[dlChannel]*1728) + 5) / 10);
					rfmSetManualFreq(manualFreq+freqOffset);
					
					LED_OR = HIGH;
					timeoutFhssPkt = ((currentTime1us-timestampPacketRxd)>500000)? (currentTime1us + 500000) : (timeoutFhssPkt + periodFhssPkt); // + 100;
					LED_OR = LOW;
				}
				
			break;
		case FAILSAFE:
				wdtIntConfig(ENABLED, 5); // 0.5 sec timeout
				if(timer1ms > 8000){
					updateVolts(FAST);
					sys.state = (sys.powerState)? BEACON : SLEEP;
					timer1ms = 0;
				}
				if(sys.state != FAILSAFE){
					printState();
					break;
				}
				rcOutputs(ENABLED);
				sys.statusLEDs = ENABLED;
				for(uint8_t i=0; i<CHANNELS; i++) pwmValues[i] = pwmFailsafes[i];
			break;
		default:
			sys.state = ACTIVE;
			// Refresh information
			// Determine nextState using refreshed information
			// Continue if remaining in current state
			// Assert Outputs
			// Configure for next loop and continue
			
	}

	// LED_OR = LOW;
}

void printState(){
	printf("State: %s\tLipoly: %u\tVoltIn: %u\tATmega: %u\tRSSI: %u\tErrors: %u\n",
			(sys.state == 0)? "DOWN" :(sys.state == 1)? "SLEEP" :(sys.state == 2)? "BEACON" :
			(sys.state == 3)? "ACTIVE" : "FAILSAFE",volt.lipoly,volt.sysVin,volt.atMega,noiseFloor, rfmWriteErrors);

}

void rcOutputs(uint8_t mode){
	TIMSK1 = (mode == ENABLED)? _BV(TOIE1) : 0;
	TCCR1A = (mode == ENABLED)? TCCR1A | _BV(COM1A1) : TCCR1A & (~_BV(COM1A1));
	
	/*
	if(mode == ENABLED){
		DDRC |= 0x0F;
		DDRD |= 0xF0;
		// PORTC |=0x0F;
		// PORTD |=0xF0;
	} else{
		DDRC &= ~(0x0F);
		DDRD &= ~(0xF0);
		// PORTC &= ~(0x0F);
		// PORTD &= ~(0xF0);
	}
	*/
}

void uartIntConfig(uint8_t mode){
	if(mode == ENABLED){
		PCICR = (1<<PCIE2);
		PCMSK2 = (1<<PCINT16);
	} else{
		PCICR = 0; //(1<<PCIE2);
		PCMSK2 = 0; //(1<<PCINT16);
	}
}

void wdtIntConfig(uint8_t mode, uint8_t interval){
	uint8_t value = (uint8_t)( _BV(WDIE) | _BV(WDE) | (interval & 0x08? (1<<WDP3): 0x00) | (interval & 0x07) );
	if(mode){
		WDTCSR |= (1<<WDCE)|(1<<WDE);
		WDTCSR = value;
	} else{
		WDTCSR |= (1<<WDCE)|(1<<WDE);
		WDTCSR = 0;
	}
}

void printRegisters(void){
	
	
	//printf("\n\t");
	// for(uint8_t c=0; c<16; c++)	printf("%X\t",c);
	printf("\n");
	for(uint8_t j=0; j<8; j++){
		// printf("%X\t",j);
		CS_RFM = LOW;
			transferSPI(16*j);
			for(uint8_t k=0; k<16; k++){
				uint8_t response = transferSPI(0x00);
				// printf("%X\t",response);
				printf("%X\n",response);
			}
		CS_RFM = HIGH;
		_delay_us(100);
		// printf("\n");
	}
	//printf("\n");
	
	//printf("_T\t%u",TCNT1);


}

uint8_t systemSleep(uint8_t interval){
	//	Interval	0	1	2	3	4	5	6	7	8	9
	//	Time in ms	16	32	64	128	256	512	1k	2k	4k	8k
	
	//LED = LOW;
	
	cli();
	
	TWCR = 0;
	TWSR = 0;
	SPCR = 0;
	ADMUX = 0;
	ADCSRA = 0;
	DIDR0 = (1<<ADC5D)|(1<<ADC4D)|(1<<ADC3D)|(1<<ADC2D)|(1<<ADC1D)|(1<<ADC0D);
	DIDR1 = (1<<AIN1D)|(1<<AIN0D);
	UCSR0B =0;
	TCCR1B = 0;
	DDRB = DDRC = DDRD = 0;
	PORTB = PORTC = PORTD = 0; // May want to comment this out to clean up waveforms
	
	//MPU_VLOGIC = LOW;
	power_all_disable();
	
	//wdt_reset();
	// uint8_t value = (uint8_t)( ((configFlags.wdtSlpEn)<<WDIE) | (interval & 0x08? (1<<WDP3): 0x00) | (interval & 0x07) );
	uint8_t value = (uint8_t)( (WDTCSR &(_BV(WDIE) | _BV(WDE))) | (interval & 0x08? (1<<WDP3): 0x00) | (interval & 0x07) );
	MCUSR = 0;
	WDTCSR |= (1<<WDCE)|(1<<WDE);
	WDTCSR = value; //_BV(WDIE) | _BV(WDE) | _BV(WDP3) | _BV(WDP0);
	
	// PCMSK2 = (1<<PCINT16);
	// PCICR = (1<<PCIE2);

	EICRA = 0;
	// EIMSK = (1<<INT1); //|(1<<INT0);
	
	
	// if(sys.state == DOWN)			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	// else if(sys.state == SLEEP) 	set_sleep_mode(SLEEP_MODE_STANDBY);
	// else										set_sleep_mode(SLEEP_MODE_IDLE);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	sleep_bod_disable();
	sei();
	sleep_cpu();
	
	sleep_disable();
	uint8_t systemReturnState = atMegaInit();
	
	//LED = HIGH;
	
	return systemReturnState;
}

uint8_t atMegaInit(void){
	uint8_t startupStatus = MCUSR; //wdt_init(); //
	MCUSR = 0;
	WDTCSR |= _BV(WDCE) | _BV(WDE); // Three Options Below:
	// Was this WDTCSR = _BV(WDIE) | _BV(WDP2) | _BV(WDP1) | _BV(WDE); // Hardwire the WDT for 1 Sec
	//WDTCSR = _BV(WDE) | _BV(WDP3) | _BV(WDP0);
	//WDTCSR = 0;
	WDTCSR = _BV(WDIE) | _BV(WDE) | _BV(WDP3) | _BV(WDP0);
	wdt_reset();
	
	// System
	//MCUCR |= (1<<PUD);		// Pull-up Disable
	MCUCR = 0;
	PRR = 0;

	// Timers
	TCCR0A = _BV(WGM01); // CTC Mode
	TCCR0B = (1<<CS01)|(1<<CS00); // clk/64
	OCR0A = 249; // 1ms
	TIMSK0 = (1<<OCIE0A);
	
	
	TCCR1A = _BV(WGM11); //_BV(COM1A1)|
	TCCR1B = _BV(WGM13)|_BV(WGM12)|(1<<CS11); // 2 MHz
	ICR1 = 50000; // 25 ms
	OCR1A = 800; // 400 uS
	
	// TCCR2A = 
	// TCCR2B = _BV(CS22)|_BV(CS20); // clk/128
	// OCR2A = 125; // 1.0 ms
	// TIMSK2 = (1<<OCIE2A);
	
	
	// IO Ports
	// 0: Input (Hi-Z) 1: Output
	//        76543210		7		6		5		4		3		2		1		0
	PORTB |=0b00000100;	//	XTAL2	XTAL1	SCK		MISO	MOSI	CS_RFM	LED_BL	LED_OR
	PORTC |=0b00001111;	//	--		Reset	SCL		SDA		P4		P3		P2		P1
	PORTD |=0b11110010;	//	P8		P7		P6		P5		RFM_PBL	RF_INT	TXD		RXD
	DDRB |= 0b00101111;	
    DDRC |= 0b00001111;	
    DDRD |= 0b11111010;	
	// <steve> TODO! Add a statement here that disables the output ports for the servos
	// when the RC Outputs are suppose to be disabled. Likely, the RC Outputs state will be a flag
	// in the global config struct.
	// Actually, this just makes life hard. 
	// Servos jump on every sleep entry, would probably do the same in the full implementation.

	
	// Serial Port
	UBRR0H = UART_UBRR >> 8;
    UBRR0L = UART_UBRR;
    UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
    stdout = &uart_io; //= stdin 
	
	//SPI
	SPCR	= (1<<SPE)|(1<<MSTR)|(1<<SPR1); // |(1<<CPOL)|(1<<CPHA) SPR0
	// SPCR	= (1<<SPE)|(1<<MSTR)|(1<<SPR0); // |(1<<CPOL)|(1<<CPHA) SPR0
	
	//I2C
	// TWCR = (1<<TWEN) | (1<<TWEA);
	// TWSR &= ~((1<<TWPS1) | (1<<TWPS0));
	// TWBR = ((F_CPU / I2C_FREQ) - 16) / 2;
	
	// ADC
	ADMUX 	= 0; //(1<<REFS0);	// AVcc Connected
	ADCSRA 	= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADIE);
	DIDR0 	= (1<<ADC5D)|(1<<ADC4D)|(1<<ADC3D)|(1<<ADC2D)|(1<<ADC1D)|(1<<ADC0D);
	
	// Interrupts
	//PCICR = 0; //(1<<PCIE2);
	//PCMSK2 = 0; //(1<<PCINT16);
	
	EICRA = _BV(ISC01);
	EIMSK = (1<<INT0); //(1<<INT1)|(1<<INT0);
	
	sei();
	
	return startupStatus;
}

char deviceIdCheck(void){
	// printRegisters();
	CS_RFM = LOW;
		transferSPI(0x00);
		uint8_t rfmDevType = transferSPI(0x00);
		uint8_t rfmVerCode = transferSPI(0x00);
	CS_RFM = HIGH;
	
	printf("\n%X\t%X\n",rfmDevType,rfmVerCode);
	
	
	rfmDevType ^= 0b00001000;
	rfmVerCode ^= 0b00000110;
	
	if(rfmDevType==0 && rfmVerCode==0) return (1);
	return 0;
}

void printHelpInfo(void){
	printf("\nConsole Useage:\n"
		"B\tBattery mV\n"
		"M\tMonitor\n"
		"1 to 3\tToggle Config Flags\n"
		"`\tWrite Toggles to EEPROM and Review\n"
		"?\tConsole Useage\n\n");
}



