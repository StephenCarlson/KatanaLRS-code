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
// Timer 0 (8-bit)			System Time Counter (10 ms), not needed given Timer 1?
// 		OC0A	PD6			Pin 7, not really available
// 		OC0B	PD5			Pin 6, not really available

// Timer 1 (16-bit)			R/C Output Generator, could be used as system time counter?
//		OC1A	PB1			PPM Output Line
//		OC1B	PB2			/CS Line, can't use

// Timer 2 (8-bit)			Would like to use for AFSK on ELT
//		OC2A	PB3			MOSI, can create tones for ELT on this
//		OC2B	PD3			Unconnected spare line, use for piezo buzzer? Free otherwise








#include "KatanaLRS.h"

// Function Prototypes
void setup(void);
void loop(void);
void printState(void);
void rcOutputs(uint8_t);
void uartIntConfig(uint8_t);
void wdtIntConfig(uint8_t, uint8_t);
void printRegisters(void);
uint8_t systemSleep(uint8_t);
uint8_t atMegaInit(void);
char deviceIdCheck(void);
void printHelpInfo(void);



// Interrupt Vectors (Listed in Priority Order)
ISR(INT0_vect){
	LED_OR = HIGH;
	sys.intSrc.rfm = 1;
	timestamp = timer1ms;
	
	dlChannel = (dlChannel < (sizeof(dlFreqList)-1))? dlChannel+1 : 0; 
	//rfmWriteReg(0x05,0);
	//rfmWriteReg(0x07,(1<<1));
	//EIMSK = 0;
}

ISR(PCINT2_vect){
	sys.intSrc.uart = 1;
	PCICR = 0;
	PCMSK2 = 0;
}

ISR(WDT_vect){
	sys.intSrc.wdt = 1;
}

// ISR(TIMER1_CAPT_vect){
	// This is where the Input PPM steam is parsed
	
	
// }

ISR(TIMER1_OVF_vect ){ // May want to redo as if/else structure, more efficient?
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
			ICR1 = pwmValues[ch]; // Not Relevant: Loss of precision in favor of uSec representation in pwmValues[]
			pwmFrameSum += pwmValues[ch]; // 65535 is the max for pwmFrameSum, max ch PWM width is 8192 ~ 4096 uS
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
			pwmFrameSum = (pwmFrameSum > 40000)? 0 : pwmFrameSum;
			ICR1 = (40000 - pwmFrameSum); // Hazard if negative!
			ch = 0;
			OCR1A = 800;	// Produce a 400 us low pulse
			TCCR1A |= _BV(COM1A1)|_BV(COM1A0); // COM1A0 Flips to Idle-High
			break;
		default:
			ch = 8;
	}
	LED_OR = LOW;
}

ISR(TIMER0_COMPA_vect){
	//LED_OR = HIGH;
	sys.intSrc.timer0 = 1;
	// timer1min += (timer1ms >= 60000)? 1 : 0; // Rolls at 1000 hours
	// timer1ms = (timer1ms >= 60000)? 0 : timer1ms+1; // 60 sec
	timer1ms += 1;
	//LED_OR = LOW;
}

ISR(USART_RX_vect){
	sys.intSrc.uart = 1;
	sys.monitorMode = 0;
	
	uint8_t command = UDR0;
	
	switch(command){
		case 'B':
			printf("Battery: %u\n", volt.atMega);
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
	for(uint8_t i=0; i<5; i++){
		rfmWriteReg(0x07, 0x80);		// Reset the Chip
		_delay_ms(1);
	}
	for(uint8_t i=0; i<200; i++){
		if((rfmReadReg(0x04)&0x02) == 0x02) break;
		_delay_ms(1);
	}
	rfmSetMode(ACTIVE);
	
	// Tasks and Routines
	printf("\n\nKatanaLRS v1\nBy Steve Carlson %s\n\n",__DATE__);
	printf("Reset Source: "); //%X\n",startStatus); //%X\n", startStatus);
	if(startStatus&WDRF) printf("WatchDog\t"); // From iom328p.h in AVR Include Folder
	if(startStatus&BORF) printf("BrownOut\t");
	if(startStatus&EXTRF) printf("External\t");
	if(startStatus&PORF) printf("PowerOn\t");
	printf("\n\n");
	//	WDRF BORF EXTRF PORF
	
	flashOrangeLED(5,10,40); 										// 250 ms
	sys.monitorMode = 1;
	
	updateVolts(0);
	
	printf("Device ID Check: ");
	if(deviceIdCheck()){
		printf("OK\n");
		// eltFullSequence();
		// eltTransmit_Packet();										// ~100 ms
	} else{
		printf("FAILED!\n");
		for(uint8_t i=0; i<10; i++){
			flashOrangeLED(5,10,40);
			if(deviceIdCheck()) break;
			if(i >= 9) sys.state = FAILSAFE; 
		}
	}	
	
	*((uint8_t*) &configFlags) = eeprom_read_byte((const uint8_t*) EEPROM_START);
	
	// Application Warm-up
	TIMSK1 = _BV(TOIE1);
	for(uint8_t i=0; i<CHANNELS; i++){
		pwmValues[i] = 1500<<1;
	}
	
	gps.lat  = 89999999;
	gps.lon  = 179999999;
	gps.time = 235959;
	gps.alt  = 1465;
	gps.sats = 8;
	gps.hdop = 15;
	
	// Console Usage Hints
	printHelpInfo();
	
	
	// Development
	
	rfmSetInterrupts(ENABLED,100);
	EICRA = _BV(ISC01); // Falling-Edge
	EIMSK = _BV(INT0);
	rfmWriteReg(RFM_CONTROL_1, RFM_rxon);
}

void loop(void){
	static uint8_t eltTransmitCount = 0;
	static uint16_t failsafeCounter = 0;
	
	static uint16_t manualFreq = 3000;
	
	static uint16_t secLoop;
	
	uint16_t rfmIntList = 0;
	uint8_t rfmFIFO[64];
	#define RFM_INT_VALID_PACKET_RX (1<<(1))
	#define RFM_INT_RSSI_THRESH ((1<<(4))<<8)
	#define DL_BIND_FLAG (1<<7)
	
	// Assert Concurrent Outputs (Outputs not tied to a FSM State, but merely from inputs)
	// if(sys.statusLEDs) LED_OR = HIGH; //flashOrangeLED(2,5,5); // Solve Delay timing issue
	
	// if(RFM_INT) LED_OR = HIGH;
	
	// transferSPI(sys.state);
	// _delay_us(1);
	
	if(sys.intSrc.wdt){ // Wow! Race Condition! Should only check this in a single function
		//if(sys.statusLEDs) LED_OR = HIGH;
		// updateVolts(1);
		sys.intSrc.wdt = 0;
		_delay_ms(1);
		// printf("State: %s\tLipoly: %u\tVoltIn: %u\tATmega: %u\tRSSI: %u\tErrors: %u\n",
			// (sys.state == 0)? "DOWN" :(sys.state == 1)? "SLEEP" :(sys.state == 2)? "BEACON" :
			// (sys.state == 3)? "ACTIVE" : "FAILSAFE",volt.lipoly,volt.sysVin,volt.atMega,noiseFloor, rfmWriteErrors);

	}
		
	// Carry out the current State processes and determine next state
	switch(sys.state){ // Native State Machine
		case DOWN:	// This is the worst-case state, and must be power-optimized as much as possible.
					// Getting to this point means the battery is at 70% or so. The ATmega must power-down hard
					// and configure the RFM for Low-Duty-Cycle mode Receive. This mode needs to allow for several 
					// weeks of standby monitoring time, months if possible. A valid interrogation jumps the 
					// state machine to SLEEP to emit at least one ELT burst, regardless of battery condition.
					// It would make sense to have the LDC Rx frequencies to be the regular LRS list, as the LRS
					// Transmitter is already a stellar preformer in the roll of interrogator. The Rx freq should
					// roll on the list, shifting to the next entry every minute / 10th wake or so. This will allow
					// for noise and locked-out channels.
					// As mentioned in SLEEP, is having the ATmega go totally down necessary? Also, is it 
					// dangerous to have the entire system rest on the RFM if the ATmega disables its WDT and does
					// hard sleep?
					// If GPS information is flowing, parse it.
				wdtIntConfig(ENABLED,9); //DISABLED,0);
			// Refresh information
				updateVolts(0);
				noiseFloor = rfmGetRSSI();
				//rfmIntList = rfmGetInterrupts();
			// Determine nextState using refreshed information
				if(noiseFloor>100){ //rfmIntList&RFM_INT_RSSI_THRESH){ //&RFM_INT_VALID_PACKET_RX){
					printf("Rx in DOWN Works!\t%u\t%X\n",noiseFloor,rfmIntList);
					// rfmReadFIFO(rfmFIFO);
					// if(rfmFIFO[0]&(DL_BIND_FLAG)) sys.state = SLEEP;
					sys.state = SLEEP;
					// else sys.state = ACTIVE;
				} else sys.state = (sys.batteryState || sys.powerState)? SLEEP : DOWN;
			// Continue if remaining in current state
				if(sys.state != DOWN){
					printState();
					break;
				}
			// Assert Outputs
				rcOutputs(DISABLED);
				sys.statusLEDs = ENABLED; //DISABLED;
				uartIntConfig(DISABLED);
			// Configure for next loop and continue
				//rfmSetInterrupts(DISABLED,100);
				//EIMSK = (1<<INT0);
				systemSleep(9);
			break;
		case SLEEP:	// Power-optimized 8-second beacon bursts, until the battery is at 70% or so.
					// This and the DOWN state are exclusive to battery-only operation. If there is no good power
					// from the 5v bus, SLEEP operates until the battery is consumed to the point we discontinue.
					// If a valid packet/interrogation is received, the state machine jumps to BEACON for several cycles.
					// A question to answer is whether the LDC mode on the RFM is worth using, or it having the ATmega wake 
					// and configure the RFM to Rx manually is more power conservative. As it stands are present, the ATmega
					// checks the RSSI at the end of the ELT burst, as the RFM is already active.
					// If GPS information is flowing, parse it.
				wdtIntConfig(ENABLED,9);
			// Refresh information
				updateVolts(0);
				eltFullSequence();
				_delay_ms(2);
				noiseFloor = rfmGetRSSI();
				//rfmIntList = rfmGetInterrupts();
			// Determine nextState using refreshed information
				if(noiseFloor>100){ //rfmIntList&RFM_INT_RSSI_THRESH){ //&RFM_INT_VALID_PACKET_RX){
					//printf("Rx in SLEEP Works!\t%u\t%X\n",noiseFloor,rfmIntList);
					// rfmReadFIFO(rfmFIFO);
					// if(rfmFIFO[0]&(DL_BIND_FLAG)) sys.state = BEACON;
					sys.state = BEACON;
					// else sys.state = ACTIVE;
					// sys.state = ACTIVE;
				} else sys.state = (sys.batteryState || sys.powerState)? SLEEP : DOWN;
				// } else sys.state = ((sys.powerState == 0))? ((sys.batteryState)? SLEEP : DOWN) : ACTIVE; //sticksCentered() && 

			// Continue if remaining in current state
				if(sys.state != SLEEP){
					printState();
					break;
				}
			// Assert Outputs
				rcOutputs(DISABLED);
				sys.statusLEDs = ENABLED; //DISABLED;
				uartIntConfig(DISABLED);
			// Configure for next loop and continue
				//rfmSetInterrupts(DISABLED,100);
				//EIMSK = (1<<INT0);
				_delay_ms(30);
				systemSleep(9);
			break;
		case BEACON:	// Stay here until the aircraft main battery is depleted. 
						// As the KatanaLRS is a minor consumer even in Tx, no problem in continuous operation, 
						// won't hasten the main battery demise too much.
						// Makes sense that the ELT would be full-duty-cycle immediately after a crash.
						// This is also visited when the module is interrogated by a valid packet/tone
						// Hence, this state needs to play the same way as the SLEEP state to reduce battery power
						// and to preserve stealth. No R/C Outputs or LEDs if visited from SLEEP.
						// If GPS information is flowing, parse it.
				wdtIntConfig(ENABLED,8);
				//rfmSetInterrupts(DISABLED,100);
				//EIMSK = (1<<INT0);
			// Refresh information
				updateVolts(1);
				//rfmIntList = rfmGetInterrupts();
				_delay_ms(2);
			// Determine nextState using refreshed information
				if(0){ //rfmIntList&RFM_INT_VALID_PACKET_RX){
					// rfmReadFIFO(rfmFIFO);
					// if(rfmFIFO[0]&(DL_BIND_FLAG)) sys.state = BEACON;
					// else sys.state = ACTIVE;
				} else sys.state = (eltTransmitCount > 10)? SLEEP : BEACON;
			// Continue if remaining in current state
				if(sys.state != BEACON){
					eltTransmitCount = 0;
					printState();
					_delay_ms(30);
					break;
				}
				eltTransmitCount += 1;
			// Assert Outputs
				rcOutputs((sys.powerState)? ENABLED : DISABLED);
				sys.statusLEDs = ENABLED; //(sys.powerState)? ENABLED : DISABLED;
				uartIntConfig(DISABLED);
				eltFullSequence();
				
			// Configure for next loop and continue
				_delay_ms(30);
			break;
		case ACTIVE:	// Includes the first state of Failsafe: poor/lossy or momentary lost link.
						// Hopefully, this is the only mode ever really used. Optimized for R/C servo output.
						// This mode would also parse the optional GPS input and interact with the Flight Controller.
						// For normal discontinued operation (flight is over, main battery disconnected), this state
						// has to determine that there is no emergency when link is lost. (May want to shift this to
						// FAILSAFE.) The mission is over when sticks are centered and 5v input power is already gone.
						// An emergency would occur if the aircraft either impacts terrain (lossing 5v power and 
						// possibly antenna) or if link is lost (control inputs would probably no be centered, and 
						// 5v should be good). Sticks Centered and 5v absent might still occur in a emergency, but 
						// ultimately, the LRS never disengages from monitoring: DOWN is the "ground state".
				wdtIntConfig(ENABLED, 5); // 0.5 sec timeout
			// Refresh information
				//updateVolts(1);
			// Determine nextState using refreshed information
				//if(timer10ms > 600){ // 10 Misses in 20 hops (Fix this)
					//timer10ms = 0;
					//failsafeCounter = 0;
					//updateVolts(1); // Very Dangerous. Perhaps just checking for the powerState component?
					// sys.state = ((sys.powerState == 0))? DOWN : FAILSAFE; //sticksCentered() && 
					// sys.state = ((sys.powerState == 0))? SLEEP : ACTIVE; //sticksCentered() && 
				//  } else sys.state = ACTIVE;
			// Continue if remaining in current state
				if(sys.state != ACTIVE){
					printState();
					_delay_ms(30);
					break;
				}
			// Assert Outputs
				// rcOutputs(ENABLED);
				// for(uint8_t i=0; i<CHANNELS; i++){
					// pwmValues[i] = 1700<<1;
				// }
				sys.statusLEDs = ENABLED;
				
				
				
				
				// if((rfmReadReg(RFM_CONTROL_1)& RFM_rxon) != RFM_rxon){
					// rfmSetInterrupts(ENABLED,100);
					//EIMSK = (1<<INT0);
					// rfmWriteReg(RFM_CONTROL_1, RFM_rxon);
					// // rfmWriteReg(0x27, 100);
					// printf("Enabled Rx\n");
				// }
				
				
				if(sys.intSrc.rfm || RFM_INT){
					
					// sys.intSrc.rfm = 0; Move to end to prevent double cycling
					//rfmSetDlChannel(dlFreqList[dlChannel]);
					
					uint8_t rssi = rfmReadReg(0x26);
					int8_t afcMeasure = rfmReadReg(0x2B); // Is manualFreq>>2 Offset. This 4x + manualFreq is best center
					uint8_t rfmIntReg1 = rfmReadReg(0x02);
					uint8_t rfmIntReg2 = rfmReadReg(0x03);
					uint8_t rfmIntReg3 = rfmReadReg(0x04);
					uint8_t rfmModeReg = rfmReadReg(0x07);
					uint8_t rfmHeader3 = rfmReadReg(0x47);
					uint8_t rfmHeader2 = rfmReadReg(0x48);
					uint8_t rfmHeader1 = rfmReadReg(0x49);
					
					
					
					CS_RFM = LOW;
						transferSPI(0x7F);
						for(uint8_t i=0; i<32; i++){
							rfmFIFO[i] = transferSPI(0x00);
						}
					CS_RFM = HIGH;
					/*
					const uint8_t DL_BYTES = 10;
					uint8_t dlReadArray[DL_BYTES];
					
					uint8_t k=0;
					for(uint8_t j=0; j<(DL_BYTES-4); j++){
						dlReadArray[k]   = ((rfmFIFO[j*5  ]&0xFC)>>2 | rfmFIFO[j*5+1]<<6);
						dlReadArray[k+1] = ((rfmFIFO[j*5+1]&0xF0)>>4 | rfmFIFO[j*5+2]<<4);
						dlReadArray[k+2] = ((rfmFIFO[j*5+2]&0xC0)>>6 | rfmFIFO[j*5+3]<<2);
						dlReadArray[k+3] =  rfmFIFO[j*5+4];
						k+=4;
					}
					
					
					// 5C,41,44,57,79,61,40,60,43,C6
					printf("%X,",dlReadArray[0]);
					for(uint8_t i=0; i<7; i++){
						//pwmValues[i] = (( ((dlReadArray[0]>>i)&0x01)<<8 | dlReadArray[i+1] )<<1+1000)<<1;;
						pwmValues[i] = (((uint16_t) dlReadArray[i+1]<<2) + 1000)<<1;
						printf("%u,",pwmValues[i]>>1);
					}
					*/
					
					// for(uint8_t i=0; i<DL_BYTES; i++){
						// printf("%X,",dlReadArray[i]);
					// }
					
					
					rfmWriteReg(0x08, (1<<1));
					rfmWriteReg(0x08, 0);
					rfmWriteReg(0x07, 0); //(1<<2)|(1<<1));
					rfmWriteReg(0x08, (1<<2));
					// rfmWriteReg(0x70, (1<<5)|(1<<4));
					
					printf("%u,%u,%d,%X,%X,%X,%X,%X,%X\t",dlChannel,rssi,afcMeasure,rfmIntReg1,rfmIntReg2,rfmIntReg3,rfmHeader3,rfmHeader2,rfmHeader1);
					printf("\n");
					
					//printf("%u,%u,%d\n",dlChannel,rssi,afcMeasure);
					
					// rfmWriteReg(0x05,(1<<4));
					//EIMSK = (1<<INT0);
					
					sys.intSrc.rfm = 0;
				}
				
				if(timer1ms > timestamp+5000){ // Still have issues at 60 seconds intervals with this!!!!!! 
					//LED_OR = HIGH;
					timestamp = timer1ms;
					
					//dlChannel = (dlChannel > 200)? 0 : dlChannel+1;
					
					dlChannel = (dlChannel < (sizeof(dlFreqList)-1))? dlChannel+1 : 0; 
					//rfmSetDlChannel(dlFreqList[dlChannel]);
					
					// manualFreq = (manualFreq >= 34600)? 3000 : manualFreq + 64;
					// rfmWriteReg(0x76,manualFreq>>8);		// Freq Carrier 1	Upper Byte
					// rfmWriteReg(0x77,manualFreq&0xFF);	// Freq Carrier 0	Lower Byte
					
					//printf("%u\n",manualFreq);
				
					// rfmWriteReg(0x08, (1<<1));
					// rfmWriteReg(0x08, 0);
					
					
					printf("~%X,%X,%X\n",rfmReadReg(0x02),rfmReadReg(0x04),rfmReadReg(0x07));
				}
				
				if(timer1ms > secLoop+1000){
					updateVolts(1);
					rcOutputs(ENABLED);
				}
	
				//printState();
				
				// printf("Rx in DOWN Works!\t%u\t%X\n",noiseFloor,rfmIntList);
				
			// Configure for next loop and continue
				// Insert Timer0 Coder here later
			break;
		case FAILSAFE:	// Entered if contact has been lost for long enough that servos must be driven to safe values.
						// This must be the most stable state? Don't parse GPS?
						// This state 
				wdtIntConfig(ENABLED, 5); // 0.5 sec timeout
			// Refresh information
				// if(sys.intSrc.wdt){ // These should be a separate timer, no WDT
					// failsafeCounter += 1;
					// updateVolts(1);
					// sys.intSrc.wdt = 0;
				// }
			// Determine nextState using refreshed information
				if(timer1ms > 8000){
					updateVolts(1);
					sys.state = (sys.powerState)? BEACON : SLEEP;
					timer1ms = 0;
				}
			// Continue if remaining in current state
				if(sys.state != FAILSAFE){
					printState();
					break;
				}
			// Assert Outputs
				rcOutputs(ENABLED);
				sys.statusLEDs = ENABLED;
				for(uint8_t i=0; i<CHANNELS; i++){
					pwmValues[i] = 1000<<1;
				}
			// Configure for next loop and continue
				
			break;
		default:
			sys.state = FAILSAFE;
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
	
	
	printf("\n\t");
	for(uint8_t c=0; c<16; c++)	printf("%X\t",c);
	printf("\n");
	for(uint8_t j=0; j<8; j++){
		printf("%X\t",j);
		CS_RFM = LOW;
			transferSPI(16*j);
			for(uint8_t k=0; k<16; k++){
				uint8_t response = transferSPI(0x00);
				printf("%X\t",response);
			}
		CS_RFM = HIGH;
		printf("\n");
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

	// EICRA = 0;
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
    DDRD |= 0b11110010;	
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
	SPCR	= (1<<SPE)|(1<<MSTR)|(1<<SPR0); // |(1<<CPOL)|(1<<CPHA)
	
	//I2C
	TWCR = (1<<TWEN) | (1<<TWEA);
	TWSR &= ~((1<<TWPS1) | (1<<TWPS0));
	TWBR = ((F_CPU / I2C_FREQ) - 16) / 2;
	
	// ADC
	ADMUX 	= 0; //(1<<REFS0);	// AVcc Connected
	ADCSRA 	= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADIE);
	DIDR0 	= (1<<ADC5D)|(1<<ADC4D)|(1<<ADC3D)|(1<<ADC2D)|(1<<ADC1D)|(1<<ADC0D);

	//PCICR = 0; //(1<<PCIE2);
	//PCMSK2 = 0; //(1<<PCINT16);
	
	EICRA = 0;
	EIMSK = 0; //(1<<INT1)|(1<<INT0);
	
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



