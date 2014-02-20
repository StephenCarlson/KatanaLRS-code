// Katana Long-Range-System, by Stephen Carlson, May 2013
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

// Behavioral Switches
// #define TRANSMITTER
#define RECEIVER

#ifdef TRANSMITTER
#define RFM23BP

#elif defined(RECEIVER)
#define RFM22B
#define BATTERY
#endif


// Debug Switches
//#define DEBUG_MAIN_LOOP

// Behavioral Parameters
#define SLEEP_INT		7 // Don't go over 7

// System Parameters
//#define F_CPU			16000000UL
//#define BAUD			115200 //19200
#define UART_UBRR		8 //(((((F_CPU * 10) / (16L * BAUD)) + 5) / 10) - 1)
#define I2C_FREQ		400000L
#define LOOP_PERIOD		2500	// (16000000 Hz / 64) / 100 Hz
#define BUFFER_SIZE 	128
#define EEPROM_START	10
#define LIPOLY_CUTOFF	3400
#define VIN_CUTOFF		3800

// System Constants
#define DOWN 			0 // Configure RFM/SI4432 to interrupt on Rx Packet or RSSI, only wake on this interrupt
#define SLEEP			1 // Sleep, wake on watchdog, measure RSSI, transmit packet and tones
#define BEACON	 		2 // Same as SLEEP, but continuous. Activated on RX RSSI, Resume SLEEP in 2 Minutes
#define ACTIVE 			3 // Normal Operation as an LRS Receiver, no transmission behaviors
#define FAILSAFE		7 // Failsafe Condition, entered on loss of LRS packets
#define HIGH			1
#define LOW				0
#define ENABLED			1
#define DISABLED		0
#define RFM_READ		0 // RFM Direction Flags
#define RFM_WRITE		1 
#define I2C_READ		1 // I2C Direction Flags
#define I2C_WRITE		0
// #define SINGLE			0
// #define MULTI			1


// Port Definitions and Macros
typedef struct{
  unsigned int bit0:1;
  unsigned int bit1:1;
  unsigned int bit2:1;
  unsigned int bit3:1;
  unsigned int bit4:1;
  unsigned int bit5:1;
  unsigned int bit6:1;
  unsigned int bit7:1;
} _io_reg; 
#define REGISTER_BIT(rg,bt) ((volatile _io_reg*)&rg)->bit##bt

#define LED_OR		REGISTER_BIT(PORTB,0)
#define LED_BL		REGISTER_BIT(PORTB,1)
#define CS_RFM		REGISTER_BIT(PORTB,2)
#define FORCE_MOSI	REGISTER_BIT(PORTB,3)
#define CHANNELS	8
#define PWM_1		REGISTER_BIT(PORTC,0)
#define PWM_2		REGISTER_BIT(PORTC,1)
#define PWM_3		REGISTER_BIT(PORTC,2)
#define PWM_4		REGISTER_BIT(PORTC,3)
#define PWM_5		REGISTER_BIT(PORTD,4)
#define PWM_6		REGISTER_BIT(PORTD,5)
#define PWM_7		REGISTER_BIT(PORTD,6)
#define PWM_8		REGISTER_BIT(PORTD,7)
#define RFM_INT		(!(PIND &(1<<2)))
#define RFM_PMBL	(PIND &(1<<3))
#define ADC_VBAT	6 // For ADC Read Channel Selection
#define ADC_VIN		7
#define ADC_VSYS	14

// Included Headers
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/eeprom.h>

#include "i2c.c"
#include "spi.c"


#if defined(RFM22B)
	#include "rfm22b.c"
#elif defined(RFM23BP)
	#include "rfm23bp.c"
#else
	#error "No Radio Transceiver Selected!"
#endif

// Function Prototypes
void setup(void);
void loop(void);
void rcOutputs(uint8_t);
void uartIntConfig(uint8_t);
void wdtIntConfig(uint8_t, uint8_t);
void rfmIntConfig(uint8_t, uint8_t);
void printRegisters(void);
uint8_t systemSleep(uint8_t);
uint8_t atMegaInit(void);
void radioMode(uint8_t);
void radioWriteReg(uint8_t, uint8_t);
uint8_t radioReadReg(uint8_t);
uint8_t radioReadRSSI(void);
void rfmReadFIFO(uint8_t *array);
uint16_t rfmReadIntrpts(void);
void transmitELT(void);
void transmitELT_Beacon(void);
void transmitELT_Packet(void); //uint8_t *,uint8_t);
void updateVolts(uint8_t);

char deviceIdCheck(void);
void printHelpInfo(void);

static int putUARTchar(char c, FILE *stream);
uint8_t getUARTchar(void);
uint16_t readADC(uint8_t);
uint16_t readAdcNoiseReduced(uint8_t);
void flashOrangeLED(uint8_t, uint8_t, uint8_t);
void flashBlueLED(uint8_t, uint8_t, uint8_t);

// Global Variables
static FILE uart_io = FDEV_SETUP_STREAM(putUARTchar, NULL, _FDEV_SETUP_WRITE);
static char dataBufferA[BUFFER_SIZE]; //volatile

static volatile uint16_t hopGlitchCount = 0;

static volatile uint8_t ch;
static volatile uint16_t pwmValues[CHANNELS] = {1500,1300,1000,1200,1800,1900,1100,1450};
static volatile uint16_t pwmFrameSum;

static struct{
	uint16_t ch1:10;
	uint16_t ch2:10;
	uint16_t ch3:10;
	uint16_t ch4:10;
	uint16_t ch5:10;
	uint16_t ch6:10;
	uint16_t ch7:10;
	uint16_t ch8:10;
} rcCommands;


// FCC ID, Lat, Long, UTC Fix, # Sat's, HDOP, Altitude, LiPoly, System In, AtMega
const uint8_t fccId[] = "KE7ZLH";

static struct{
	int32_t lat:30;		// 536870912 > 089999999
	int32_t lon:30;		// 536870912 > 179999999
	uint32_t time:20;	// 262143 > 235959
	int16_t alt;			// 16384 Max
	uint8_t sats;		
	uint8_t hdop;		
} gps;

#define BEACON_NOTES 6
static const uint16_t beaconNotes[BEACON_NOTES][3] = {{1067,704,7},{833,222,4},{684,264,3},{782,235,2},{605,296,1},{498,352,0}};
//	Note	A4		C#5 	E5 		D5 		F#5 	A5
//	Freq	440		554.4	659.3	587.3	740		880
//	uS		2273	1804	1517	1703	1351    1136
//	Halve these values to actually get the note, as the for loop times the half-wave gaps, not peak-peak wave shape
//	uS/2	1136	902		758		851		675		568
//	Times	0.8		0.2		0.2		0.2		0.2		0.2
//	TxPwr	7		4		3		2		1		0
//	In dBm	+20		+11		+8		+5		+2		+1
//	In mW	100		12.6	6.3		3.2		1.6		1.3
//	Actual	438.7	552.4	656.0	584.9	735.5	874.1 // With -66 uS already asserted
//	uS/2	1067	833		684		782		605		498 // 684 and 498 are slightly sharp and flat, respective
//	cycles	704		222		264		235		296		352 // Keep Cycle counts tied to actual periods, not corrected ones

static struct{
	uint8_t sleepInterval:3;
	uint8_t wdtSlpEn:1;
} configFlags;
typedef struct{
	uint8_t rfm:1;
	uint8_t wdt:1;
	uint8_t uart:1;
	uint8_t timer0:1;
} intSrcType;
static volatile struct{
	uint8_t state:3;
	intSrcType intSrc;
	uint8_t monitorMode:1;
	uint8_t statusLEDs:1;
	uint8_t powerState:1;
	uint8_t batteryState:1;
} sys;
static struct{
	uint16_t lipoly;
	uint16_t sysVin;
	uint16_t atMega;
} volt;

// Interrupt Vectors (Listed in Priority Order)
ISR(INT0_vect){
	sys.intSrc.rfm = 1;
	EIMSK = 0;
}

ISR(PCINT2_vect){
	sys.intSrc.uart = 1;
	PCICR = 0;
	PCMSK2 = 0;
}

ISR(WDT_vect){
	sys.intSrc.wdt = 1;
}

// ISR(TIMER1_COMPA_vect){
	// sys.intSrc.timer0 = 1;
	// hopGlitchCount += 1;
// }

// ISR(TIMER1_COMPB_vect){
// }

ISR(TIMER0_COMPA_vect){
	sys.intSrc.timer0 = 1;
	hopGlitchCount += 1;
}

ISR(TIMER2_COMPA_vect){ // Explicitly Sloppy until sequence nail down, optimize later
	TCNT2 = 1;
	switch(ch){
		case 0:
			PWM_1 = HIGH;
			OCR2A = pwmValues[ch]>>3;
			ch+=1;
			pwmFrameSum = pwmValues[ch]>>3;
			break;
		case 1:
			PWM_1 = LOW;
			PWM_2 = HIGH;
			OCR2A = pwmValues[ch]>>3;
			ch+=1;
			pwmFrameSum += pwmValues[ch]>>3;
			break;
		case 2:
			PWM_2 = LOW;
			PWM_3 = HIGH;
			OCR2A = pwmValues[ch]>>3;
			ch+=1;
			pwmFrameSum += pwmValues[ch]>>3;
			break;
		case 3:
			PWM_3 = LOW;
			PWM_4 = HIGH;
			OCR2A = pwmValues[ch]>>3;
			ch+=1;
			pwmFrameSum += pwmValues[ch]>>3;
			break;
		case 4:
			PWM_4 = LOW;
			PWM_5 = HIGH;
			OCR2A = pwmValues[ch]>>3;
			ch+=1;
			pwmFrameSum += pwmValues[ch]>>3;
			break;
		case 5:
			PWM_5 = LOW;
			PWM_6 = HIGH;
			OCR2A = pwmValues[ch]>>3;
			ch+=1;
			pwmFrameSum += pwmValues[ch]>>3;
			break;
		case 6:
			PWM_6 = LOW;
			PWM_7 = HIGH;
			OCR2A = pwmValues[ch]>>3;
			ch+=1;
			pwmFrameSum += pwmValues[ch]>>3;
			break;
		case 7:
			PWM_7 = LOW;
			PWM_8 = HIGH;
			OCR2A = pwmValues[ch]>>3;
			ch+=1;
			pwmFrameSum += pwmValues[ch]>>3;
			// printf("Sum: %u\n",pwmFrameSum); Getting 14000
			break;
		case 8:
			PORTC &= ~(0x0F);
			PORTD &= ~(0xF0);
			if((pwmFrameSum+25) > 2500){
				pwmFrameSum = 0;
				ch = 0;
			} else{
				pwmFrameSum += 25;
				ch = 8;
			}
			OCR2A = 25;
			//TIMSK2 = 0;
			break;
		default:
			TIMSK2 = 0; //ch = 8;
	}
	// ch = ((ch >= CHANNELS) && )? 0 : ch+1;
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
	uint8_t startStatus = atMegaInit();
	sys.state = ACTIVE;
	
	// Restart Peripherals
	for(uint8_t i=0; i<5; i++){
		radioWriteReg(0x07, 0x80);		// Reset the Chip
		_delay_ms(10);
	}
	for(uint8_t i=0; i<0xFF; i++){
		if((radioReadReg(0x05)&0x02) == 0x02) break;
		_delay_ms(1);
	}
	radioMode(ACTIVE);
	
	// Tasks and Routines
	printf("\n\nKatanaLRS v1\nBy Steve Carlson May 2013\n\n");
	printf("Reset Source: "); //%X\n",startStatus); //%X\n", startStatus);
	if(startStatus&WDRF) printf("WatchDog\t"); // From iom328p.h in AVR Include Folder
	if(startStatus&BORF) printf("BrownOut\t");
	if(startStatus&EXTRF) printf("External\t");
	if(startStatus&PORF) printf("PowerOn\t");
	printf("\n\n");
	//	WDRF BORF EXTRF PORF
	
	flashOrangeLED(10,10,40);
	sys.monitorMode = 1;
	
	printf("Device ID Check: ");
	if(deviceIdCheck()){
		printf("OK\n");
		transmitELT();
	} else{
		printf("FAILED!\n");
	}	
	
	*((uint8_t*) &configFlags) = eeprom_read_byte((const uint8_t*) EEPROM_START);
	
	// Application Warm-up
	// for(uint8_t i=0; i<CHANNELS; i++){
		// pwmValues[i] = 1000;
	// }
	TIMSK2 = (1<<OCIE2A);
	
	// Console Usage Hints
	printHelpInfo();
}

void loop(void){
	static uint8_t noiseFloor = 60; // Start with ~ -80 dBm, will work down from this
	static uint8_t eltTransmitCount = 0;
	static uint16_t failsafeCounter = 0;
	
	uint16_t rfmIntList = 0;
	uint8_t rfmFIFO[16];
	#define RFM_INT_VALID_PACKET_RX (1<<(1))
	#define DL_BIND_FLAG (1<<7)
	
	// Assert Concurrent Outputs (Outputs not tied to a FSM State, but merely from inputs)
	OCR1A = (sys.statusLEDs && sys.powerState)? (volt.lipoly-2800)<<5 : 0; //[3000:4200] -> [10k;60k]
	if(sys.statusLEDs) LED_OR = HIGH; //flashOrangeLED(2,5,5); // Solve Delay timing issue
	
	
	
	if(sys.intSrc.wdt){ // Wow! Race Condition! Should only check this in a single function
		sys.intSrc.wdt = 0;
		_delay_ms(1);
		printf("State: %s\tLipoly: %u\tVoltIn: %u\tATmega: %u\tRSSI: %u\n",
			(sys.state == 0)? "DOWN" :(sys.state == 1)? "SLEEP" :(sys.state == 2)? "BEACON" :
			(sys.state == 3)? "ACTIVE" : "FAILSAFE",volt.lipoly,volt.sysVin,volt.atMega,noiseFloor);

	}
		
	// Carry out the current State processes and determine next state
	switch(sys.state){ // Native State Machine
		case DOWN:
			// Refresh information
				updateVolts(0);
				rfmIntList = 0; //rfmReadIntrpts();
			// Determine nextState using refreshed information
				if(rfmIntList&RFM_INT_VALID_PACKET_RX){
					// rfmReadFIFO(rfmFIFO);
					// if(rfmFIFO[0]&(DL_BIND_FLAG)) sys.state = SLEEP;
					// else sys.state = ACTIVE;
				} else sys.state = (sys.batteryState)? SLEEP : DOWN;
			// Continue if remaining in current state
				if(sys.state != DOWN) break;
			// Assert Outputs
				rcOutputs(DISABLED);
				sys.statusLEDs = ENABLED; //DISABLED;
				uartIntConfig(DISABLED);
			// Configure for next loop and continue
				// rfmIntConfig(ENABLED,noiseFloor);
				wdtIntConfig(DISABLED,0);
				systemSleep(9);
			break;
		case SLEEP:
			// Refresh information
				updateVolts(0);
				rfmIntList = 0; //rfmReadIntrpts();
			// Determine nextState using refreshed information
				if(rfmIntList&RFM_INT_VALID_PACKET_RX){
					// rfmReadFIFO(rfmFIFO);
					// if(rfmFIFO[0]&(DL_BIND_FLAG)) sys.state = BEACON;
					// else sys.state = ACTIVE;
				} else sys.state = (sys.batteryState)? SLEEP : DOWN;
			// Continue if remaining in current state
				if(sys.state != SLEEP) break;
			// Assert Outputs
				rcOutputs(DISABLED);
				sys.statusLEDs = ENABLED; //DISABLED;
				uartIntConfig(DISABLED);
				transmitELT();
			// Configure for next loop and continue
				// rfmIntConfig(ENABLED,noiseFloor);
				wdtIntConfig(ENABLED,9);
				systemSleep(9);
			break;
		case BEACON:
				wdtIntConfig(ENABLED,9);
			// Refresh information
				updateVolts(1);
				rfmIntList = 0; //rfmReadIntrpts();
				_delay_ms(2);
			// Determine nextState using refreshed information
				if(rfmIntList&RFM_INT_VALID_PACKET_RX){
					// rfmReadFIFO(rfmFIFO);
					// if(rfmFIFO[0]&(DL_BIND_FLAG)) sys.state = BEACON;
					// else sys.state = ACTIVE;
				} else sys.state = (eltTransmitCount > 10)? SLEEP : BEACON;
			// Continue if remaining in current state
				if(sys.state != BEACON){
					eltTransmitCount = 0;
					break;
				}
				eltTransmitCount += 1;
			// Assert Outputs
				rcOutputs((sys.powerState)? ENABLED : DISABLED);
				sys.statusLEDs = ENABLED; //(sys.powerState)? ENABLED : DISABLED;
				uartIntConfig(DISABLED);
				transmitELT();
				
			// Configure for next loop and continue
				_delay_ms(30);
			break;
		case ACTIVE:
			// Refresh information
				// if(sys.intSrc.wdt){ // These should be a separate timer, no WDT
					// hopGlitchCount = 0;
					// sys.intSrc.wdt = 0;
				// }
			// Determine nextState using refreshed information
				if(0){ //hopGlitchCount > 10){ // 10 Misses in 20 hops (Fix this)
					failsafeCounter = 0;
					updateVolts(1); // Very Dangerous. Perhaps just checking for the powerState component?
					sys.state = ((sys.powerState == 0))? DOWN : FAILSAFE; //sticksCentered() && 
				} else sys.state = ACTIVE;
			// Continue if remaining in current state
				if(sys.state != ACTIVE) break;
			// Assert Outputs
				rcOutputs(ENABLED);
				sys.statusLEDs = ENABLED;
			// Configure for next loop and continue
				wdtIntConfig(ENABLED, 5); // 0.5 sec timeout
				// Insert Timer0 Coder here later
			break;
		case FAILSAFE:
			// Refresh information
				// if(sys.intSrc.wdt){ // These should be a separate timer, no WDT
					// failsafeCounter += 1;
					// updateVolts(1);
					// sys.intSrc.wdt = 0;
				// }
			// Determine nextState using refreshed information
				if(failsafeCounter > 10) sys.state = (sys.powerState)? BEACON : SLEEP;
			// Continue if remaining in current state
				if(sys.state != FAILSAFE) break;
			// Assert Outputs
				rcOutputs(ENABLED);
				sys.statusLEDs = ENABLED;
			// Configure for next loop and continue
				wdtIntConfig(ENABLED, 5); // 0.5 sec timeout
			break;
		default:
			sys.state = FAILSAFE;
			// Refresh information
			// Determine nextState using refreshed information
			// Continue if remaining in current state
			// Assert Outputs
			// Configure for next loop and continue
			
	}

	
	LED_OR = LOW;
}

void rcOutputs(uint8_t mode){
	TIMSK2 = (mode)? (1<<OCIE2A) : 0;
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

void rfmIntConfig(uint8_t mode, uint8_t noiseFloor){
	if(mode == ENABLED){
		radioWriteReg(0x05, (1<<1)); // Enable Valid Packet Received Interrupt
		radioWriteReg(0x06, (1<<4)); // Enable RSSI Interrupt
		radioWriteReg(0x27, (noiseFloor+60)); // Configure RSSI for +30dBm Level Threshold
		EICRA = 0;
		EIMSK = (1<<INT0); //|(1<<INT0);
	} else{
		radioWriteReg(0x05, 0);
		radioWriteReg(0x06, 0);
		EIMSK = 0;
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
	TCCR0B = (1<<CS02)|(1<<CS00); // clk/1024
	OCR0A = 156; // 10 ms
	TIMSK0 = (1<<OCIE0A);
	
	TCCR1A = _BV(COM1A1)|_BV(WGM11)|_BV(WGM13);
	TCCR1B = (1<<CS12); //(1<<CS11)|(1<<CS10); //
	ICR1 = 0xFFFF;
	
	// TCCR2A = 
	TCCR2B = _BV(CS22)|_BV(CS20); // clk/128
	OCR2A = 125; // 1.0 ms
	// TIMSK2 = (1<<OCIE2A);
	
	
	// IO Ports
	// 0: Input (Hi-Z) 1: Output
	//        76543210		7		6		5		4		3		2		1		0
	PORTB |=0b00000100;	//	XTAL2	XTAL1	SCK		MISO	MOSI	CS_RFM	LED_BL	LED_OR
	PORTC |=0b00001111;	//	--		Reset	SCL		SDA		P4		P3		P2		P1
	PORTD |=0x11110010;	//	P8		P7		P6		P5		RFM_PBL	RF_INT	TXD		RXD
	DDRB |= 0b00101111;	
    DDRC |= 0b00001111;	
    DDRD |= 0b11110010;	

	
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

void radioWriteReg(uint8_t regAddress, uint8_t regValue){
	CS_RFM = LOW;
		transferSPI((RFM_WRITE<<7) | regAddress);
		transferSPI(regValue);
	CS_RFM = HIGH;
}

uint8_t radioReadReg(uint8_t regAddress){
	CS_RFM = LOW;
		transferSPI(regAddress);
		uint8_t value = transferSPI(0x00);
	CS_RFM = HIGH;
	return value;
}

uint8_t radioReadRSSI(void){
	if((radioReadReg(OPCONTROL1_REG)&(1<<RFM_rxon)) != (1<<RFM_rxon)){
		// printf("%X\n",radioReadReg(OPCONTROL1_REG));
		radioWriteReg(OPCONTROL1_REG, (1<<RFM_rxon));
		
		_delay_ms(3);
		
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
		
		radioWriteReg(OPCONTROL1_REG, (1<<RFM_xton));
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

void transmitELT(void){
	radioWriteReg(OPCONTROL1_REG, (1<<RFM_xton));
	_delay_ms(1);
	transmitELT_Packet(); // Want to send packet before battery sags in worst case
	_delay_ms(1);
	transmitELT_Beacon();
	radioWriteReg(OPCONTROL1_REG, 0x00);
}

void transmitELT_Beacon(void){
	if((radioReadReg(0x07)&(1<<RFM_xton)) != (1<<RFM_xton) ){
		radioWriteReg(OPCONTROL1_REG, (1<<RFM_xton));
		//printf("Fail on Preset: Beacon\n");
		//_delay_ms(2);
	}
	
	radioWriteReg(0x71, 0x12);		// FSK Async Mode, 
	radioWriteReg(0x72, 7);		// Frequency deviation is 625 Hz * value (Centered, so actual peak-peak deviation is 2x)
	
	radioWriteReg(OPCONTROL1_REG, (1<<RFM_txon));
	//_delay_ms(1);
	
	for(uint8_t n=0; n<BEACON_NOTES; n++){
		radioWriteReg(0x6D, beaconNotes[n][2]);
		_delay_ms(1);
		SPCR = 0;
		CS_RFM = HIGH;
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
		radioWriteReg(OPCONTROL1_REG, (1<<RFM_xton));
		// printf("Fail on Preset: Packet\n");
		_delay_ms(1);
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
	
	radioWriteReg(OPCONTROL1_REG, (1<<RFM_txon));

	for(uint8_t i=0; i<255; i++){
		if((radioReadReg(0x07)&0x08) == 0){
			// printf("Break@ %u\n",i);
			break;
		}
		_delay_ms(1);
	}
}

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

static int putUARTchar(char c, FILE *stream){
    if (c == '\n') putUARTchar('\r', stream);
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
	_delay_us(500);
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


/*
Dragon Link Packet Structures


00001100 10000010 10110000 11111110 11111110 01001000 11111110 00000110 10000011 11111110 00010101 10000110 11100001 01001101
A   B      C        D        E1       E2       E3       E4       E5       E6       E7       E8       E9       F1       F2

A	Flags: Bind and Failsafe Inactive
B	Byte Count: 12 Bytes beyond ID byte (D)
C	System ID: 0x82
D	MSB's for lower 8 channels
En	9 Channels Payload
F1	Upper Byte of CRC-16 IBM/ARC
F2	Lower Byte of CRC-16 IBM/ARC

BIND Behavior:
01000011 01001111 00000111 10000100 00100110
A   B      C        D        E        F

A	FailsafeSet Flag is [7], Bind Flag is [6]
B	Number of payload bytes after System ID Byte
C	System ID
D	Number of Channels
E	Upper CRC-16 Byte
F	Lower CRC-16 Byte





*/










/*
State Machine Notes

Repeat the State Machine Defines here:
#define DOWN 			0 // Configure RFM/SI4432 to interrupt on Rx Packet or RSSI, only wake on this interrupt
#define SLEEP			1 // Sleep, wake on watchdog, measure RSSI, transmit packet and tones
#define BEACON	 		2 // Same as SLEEP, but continuous. Activated on RX RSSI, Resume SLEEP in 2 Minutes
#define ACTIVE 			3 // Normal Operation as an LRS Receiver, no transmission behaviors


ACTIVE (3):
	If we are receiving valid R/C LRS Packets, and the System Power-in is good, then we stick with this mode.
	If we lose R/C Packets for longer than the time-out threshold, we activate the failsafes for the R/C channels.
	We start timing how long it has been since last valid packet, and do not switch modes until ~30 seconds from last packet
	Despite switching modes, we will continue to supply Failsafe setpoints as long as powerState is good (R/C servos are still powered)
	When the powerState flag is cleared, then we stop all PPM, Serial, and/or PWM generation, as the plane is effectively dead, and all electronics presumed also.
	If we have good R/C packets but the powerState is cleared, this means that we've either crashed hard enough to disconnect system power, 
		or the flight is over as the pilot has disconnected the battery by hand.
	Creative Part: How do we know which case is which? My solution at the moment: If the R/C commands are Centered Sticks, Throttle low, then this is end-of-flight.
		To avoid draining the battery with the beacon mode unnecessairly, we transition to DOWN, which is listening only.
		
		
		
DOWN (0):
	This is Listening-Only. The RFM/SI4432 is configured to interrup on a packet match, RSSI threshold, or both.
	This raises the question: Does the system transition out of this mode if it received a packet, or if there is a RSSI peak?
		If the interrupt is for a received valid packet, this means that the LRS is back. Transition to ACTIVE. There, if powerState is bad, it just hangs out I guess.
			Otherwise, if powerState is good, it will resume PPM/PWM/Serial regular operation.


SLEEP (1):
	Every WatchDog cycle, the system wakes, measures RSSI and checks for packet, and transmits the ELT Packet and Tones.
	If RSSI, Transition to BEACON
	If Packet, Transition to ACTIVE
	
BEACON (2):
	Sets the WatchDog to 2 Sec, or, alternatively, simply disables it and runs the loop continuously.
	Starts a count-down timer. On timing out, returns to SLEEP mode.
	Minor Issue: Does an RSSI or Packet event clear the timer? The danger is that a FHSS packet system is lit-up near by, and thus tripping the RSSI.
		Because BEACON is sampling the RSSI much faster then SLEEP, it might exhaust the LiPoly quickly while hanging in this mode.
		So, lets simply make this mode something that has to be re-entered after each time-out. Good move?
	
	
Concurrent Items:
batteryState: LiPoly is over 3.4v or so
	If this ever clears, if powerState is good, then no transition/change
	If powerState is cleared, we transition to DOWN. The problem is that this disables transmitting modes (1 and 2).
	If we get RSSI or Packet in DOWN, then it needs to be able to respond, and to do so, has to transition to either SLEEP or BEACON.
	So, batteryState should be enforced in some manner that allows wandering into other states for a bit.
	
powerState: KatanaLRS is being powered by the aircraft, sufficient to charge the LiPoly
	If cleared, disable PPM/PWM/Serial? Bad move, because if the receiver merely is disconnected, then the plane might be good while we shunt it.

RSSI: RFM/SI4432 measures a RSSI that is 30 dBm above noiseFloor
	Means that a 70cm HT is being keyed. Only applicable it no LRS Packets are being received and we are not ACTIVE mode.
	Actually, no LRS packets should drive whether we are ACTIVE or not, right?
	
Packet: RFM/SI4432 has captured a valid LRS packet
	Means that the LRS is active. If ACTIVE, this signals the start of processing a new frame. Otherwise, ....



Lets try in the same style as KatanaLRS-Statemachine.ods


		State ->			(Mealy/Concurrent)									DOWN								SLEEP(WDT=8sec)					BEACON							ACTIVE
		Substate ->																																									failsafe==0							failsafe==1							
Outputs:
	PPM/PWM/Serial																Disabled							Disabled						powerState? Enabled:Off			Enabled								Enabled
	ELT Transmissions															Disabled, Only Rx					Every WDT Wake					Continuous						Disabled							Disabled
	Indicator LEDs																Disabled							Disabled						Enabled? Want Stealth?			Enabled								Enabled
Inputs:					
	RFM/Si4432 Int or Read													
		RSSI Thresh	(70cm HT Keyed)												Ignore?								BEACON, TIMER=15sec				-								-									-
		BIND Packet																Delay ~8sec; SLEEP					BEACON, TIMER=1min												-									BEACON, TIMER=1min
		Valid PACKET															ACTIVE, TIMER=2sec					ACTIVE, TIMER=2sec				ACTIVE, TIMER=2sec				TIMER=2sec							failsafe=0, TIMER=0.2sec

	TIMER Expires / Default Case												WDT Disabled						batteryState? SLEEP:			powerState?						sticksCentered&&(!powerState)? 		powerState? BEACON:SLEEP
	|																													DOWN							(BEACON,TIMER=1min):			DOWN :
	|																																					SLEEP							failsafe=1, TIMER=20min

	
	GPS UART Sentence															Ignore, as not powered				Ignore, as not powered			powerState? Valid? Keep			Valid? Keep							Valid? Keep
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	

	
	
	
	

*/

