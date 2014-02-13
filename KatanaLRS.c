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
#define TRANSMITTER
//#define RECEIVER

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
#define DOWN 			0 // Only Wake on Interrupts
#define SLEEP			1 // Sleep, waking using Watchdog
#define STANDBY 		2 // Non-Transmitting State
#define ACTIVE 			3 // Fully-Active Mode
#define INT_SRC_CLEAR	0
#define INT_SRC_INTx	1
#define INT_SRC_WDT		2
#define INT_SRC_UART	3
#define INT_SRC_TIMER	4
#define HIGH			1
#define LOW				0
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
void printRegisters(void);
uint8_t systemSleep(uint8_t);
uint8_t atMegaInit(void);
void radioMode(uint8_t);
void radioWriteReg(uint8_t, uint8_t);
void updateVolts(void);

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
//static uint8_t dataBufferA[BUFFER_SIZE]; //volatile
static struct{
	uint8_t sleepInterval:3;
	uint8_t wdtSlpEn:1;
} configFlags;
static volatile struct{
	uint8_t systemState:2;
	uint8_t intSource:3;
	uint8_t monitorMode:1;
	uint8_t powerState:1;
	uint8_t batteryState:1;
} stateFlags;
static struct{
	uint16_t lipoly;
	uint16_t sysVin;
	uint16_t atMega;
} volt;

// Interrupt Vectors (Listed in Priority Order)
ISR(INT0_vect){
	stateFlags.intSource = INT_SRC_INTx;
	EIMSK = 0;
}

ISR(INT1_vect){
	stateFlags.intSource = INT_SRC_INTx;
	EIMSK = 0;
}

ISR(PCINT2_vect){
	stateFlags.intSource = INT_SRC_UART;
	PCICR = 0;
	PCMSK2 = 0;
}

ISR(WDT_vect){
	stateFlags.intSource = INT_SRC_WDT;
}

ISR(TIMER1_COMPA_vect){
	stateFlags.intSource = INT_SRC_TIMER;
}

ISR(TIMER1_COMPB_vect){
}

ISR(TIMER0_OVF_vect){
}

ISR(USART_RX_vect){
	stateFlags.intSource = INT_SRC_UART;
	stateFlags.monitorMode = 0;
	
	uint8_t command = UDR0;
	
	switch(command){
		case 'B':
			printf("Battery: %u\n", volt.atMega);
			break;
		case 'M':
			stateFlags.monitorMode = 1;
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
	stateFlags.systemState = ACTIVE;
	
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
	
	printf("Device ID Check: ");
	if(deviceIdCheck()){
		printf("OK\n");
	} else{
		printf("FAILED!\n");
	}	
	
	*((uint8_t*) &configFlags) = eeprom_read_byte((const uint8_t*) EEPROM_START);
	
	// Console Usage Hints
	printHelpInfo();
}

void loop(void){
	if(stateFlags.intSource == INT_SRC_WDT){
		stateFlags.intSource = INT_SRC_CLEAR;
		
		updateVolts();
		
		stateFlags.monitorMode = 1;
		#ifdef TRANSMITTER
		if(stateFlags.monitorMode==1){
			flashOrangeLED(2,5,5);
			printf("Lipoly: %u\tVoltIn: %u\tATmega: %u\n",volt.lipoly,volt.sysVin,volt.atMega);
		}
		#endif // TRANSMITTER
		#ifdef RFM22B
		if(stateFlags.monitorMode==1 && stateFlags.batteryState==1){
			flashOrangeLED(2,5,5);
			printf("Lipoly: %u\tVoltIn: %u\tATmega: %u\n",volt.lipoly,volt.sysVin,volt.atMega);
			
			uint8_t regValue = 0;
			
			regValue = (1<<RFM_xton); // (1<<RFM_rxon) | 
			CS_RFM = LOW;
				transferSPI((RFM_WRITE<<7) | OPCONTROL1_REG);
				transferSPI(regValue); // 0b00000100
			CS_RFM = HIGH;
			
			_delay_ms(1);
			// radioWriteReg(0x08, 0x03);	// FIFO reset
			// radioWriteReg(0x08, 0x00);	// Clear FIFO
			// radioWriteReg(0x34, 64);	// preamble = 64nibble
			// radioWriteReg(0x3E, 50);
			
			radioWriteReg(0x6D, 0x07);
			
			regValue = (1<<RFM_txon) | (1<<RFM_xton);
			CS_RFM = LOW;
				transferSPI((RFM_WRITE<<7) | OPCONTROL1_REG);
				transferSPI(regValue); // 0b00000100
			CS_RFM = HIGH;
			
			_delay_ms(2);
			for(uint16_t d=0; d<=470; d++){ // 600
				//uint8_t fillData = ((d&0x04)==0x04)? 0xFF : 0x00;
				transferSPI(d&0x01); // 0b00000100
				_delay_us(1910); // 1875
			}
			_delay_ms(2);
			radioWriteReg(0x6D, 0x04);
			_delay_ms(2);
			for(uint16_t d=0; d<=360; d++){ // 800
				//uint8_t fillData = ((d&0x04)==0x04)? 0xFF : 0x00;
				transferSPI(d&0x01); // 0b00000100
				_delay_us(1520); // 1295
			}
			_delay_ms(2);
			radioWriteReg(0x6D, 0x00);
			_delay_ms(2);
			for(uint16_t d=0; d<=620; d++){ // 1000
				//uint8_t fillData = ((d&0x04)==0x04)? 0xFF : 0x00;
				transferSPI(d&0x01); // 0b00000100
				_delay_us(640);
			}
			
			radioWriteReg(OPCONTROL1_REG, 0x00);
		} else {
			radioWriteReg(OPCONTROL1_REG, 0x00);
		}
		#endif // RFM22B
		
		uint8_t tempReg = WDTCSR;
		tempReg |= _BV(WDIE);
		WDTCSR |= (1<<WDCE)|(1<<WDE);
		WDTCSR = tempReg;
	}
	if(stateFlags.intSource == INT_SRC_TIMER){
	}
	
	#ifdef RECEIVER
	if(stateFlags.powerState){
		OCR1A = volt.lipoly;
	} else{
		OCR1A = 0;
		systemSleep(8);
	}
	#endif // RECEIVER
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
	PORTB = PORTC = PORTD = 0; // 
	DDRB = DDRC = DDRD = 0;
	
	//MPU_VLOGIC = LOW;
	power_all_disable();
	
	//wdt_reset();
	//uint8_t value = (uint8_t)( ((configFlags.wdtSlpEn)<<WDIE) | (interval & 0x08? (1<<WDP3): 0x00) | (interval & 0x07) );
	MCUSR = 0;
	WDTCSR |= (1<<WDCE)|(1<<WDE);
	WDTCSR = _BV(WDIE) | _BV(WDE) | _BV(WDP3) | _BV(WDP0);
	
	// PCMSK2 = (1<<PCINT16);
	// PCICR = (1<<PCIE2);

	// EICRA = 0;
	// EIMSK = (1<<INT1); //|(1<<INT0);
	
	
	// if(stateFlags.systemState == DOWN)			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	// else if(stateFlags.systemState == SLEEP) 	set_sleep_mode(SLEEP_MODE_STANDBY);
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
	TCCR1A = _BV(COM1A1)|_BV(WGM11)|_BV(WGM13);
	TCCR1B = (1<<CS12); //(1<<CS11)|(1<<CS10); //
	ICR1 = 0xFFFF;
	
	// IO Ports
	// 0: Input (Hi-Z) 1: Output
	//        76543210		7		6		5		4		3		2		1		0
	DDRB |= 0b00101111; //	XTAL2	XTAL1	SCK		MISO	MOSI	CS_RFM	LED_BL	LED_OR
    DDRC |= 0b00001111; //	--		Reset	SCL		SDA		P4		P3		P2		P1
    DDRD |= 0b00000010; //	P8		P7		P6		P5		RFM_PBL	RF_INT	TXD		RXD
	// PORTC |=0b00000000;
	
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
	
	radioWriteReg(0x06, 0x00);		// Disable all interrupts
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
	//radioWriteReg(0x1E, 0x0A);	// AFC timing

	radioWriteReg(0x20, 0xA1);		// clock recovery
	radioWriteReg(0x21, 0x20);		// clock recovery
	radioWriteReg(0x22, 0x4E);		// clock recovery
	radioWriteReg(0x23, 0xA5);		// clock recovery
	radioWriteReg(0x24, 0x00);		// clock recovery timing
	radioWriteReg(0x25, 0x0A);		// clock recovery timing

	//radioWriteReg(0x2A, 0x18);
	radioWriteReg(0x2C, 0x00);
	radioWriteReg(0x2D, 0x00);
	radioWriteReg(0x2E, 0x00);

	radioWriteReg(0x6E, 0x27);		// TX data rate 1
	radioWriteReg(0x6F, 0x52);		// TX data rate 0

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
	//radioWriteReg(0x3E, 17);		// set packet length to 17 bytes (max size: 255 bytes)
	radioWriteReg(0x3E, 50);	// set packet length to PKTSIZE bytes (max size: 255 bytes)

	radioWriteReg(0x3F, '*');		// set rx header
	radioWriteReg(0x40, 'E');
	radioWriteReg(0x41, 'W');
	radioWriteReg(0x42, 'S');
	radioWriteReg(0x43, 0xFF);		// check all bits
	radioWriteReg(0x44, 0xFF);		// Check all bits
	radioWriteReg(0x45, 0xFF);		// check all bits
	radioWriteReg(0x46, 0xFF);		// Check all bits

	radioWriteReg(0x56, 0x02);		// <steve> Something to do with I/Q Swapping

	radioWriteReg(0x6D, 0x00);		// Tx power to max

	radioWriteReg(0x79, 0x00);		// no frequency hopping
	radioWriteReg(0x7A, 0x00);		// no frequency hopping

	radioWriteReg(0x71, 0x12);		// GFSK, fd[8]=0, no invert for TX/RX data, FIFO mode, txclk-->gpio

	radioWriteReg(0x72, 0x48);		// Frequency deviation setting to 45K=72*625

	radioWriteReg(0x73, 0x00);		// No frequency offset
	radioWriteReg(0x74, 0x00);		// No frequency offset

	radioWriteReg(0x75, 0x53);		// frequency set to 434MHz
	radioWriteReg(0x76, 0x64);		// frequency set to 434MHz
	radioWriteReg(0x77, 0x00);		// frequency set to 434Mhz

	radioWriteReg(0x5A, 0x7F);
	radioWriteReg(0x59, 0x40);
	radioWriteReg(0x58, 0x80);

	radioWriteReg(0x6A, 0x0B);
	radioWriteReg(0x68, 0x04);
	radioWriteReg(0x1F, 0x03);
	
	
	#if defined(RFM22B)

	#endif
}

void radioWriteReg(uint8_t regAddress, uint8_t regValue){
	CS_RFM = LOW;
		transferSPI((RFM_WRITE<<7) | regAddress);
		transferSPI(regValue);
	CS_RFM = HIGH;
}

void updateVolts(void){
	uint16_t lipoly = 0; // An array or struct would be more condusive?
	uint16_t sysVin = 0;
	uint16_t atMegaVolt = 0;
	
	
	for(uint8_t j=0; j<4; j++){
		lipoly += readADC(ADC_VBAT);
		sysVin += readADC(ADC_VIN);
		for(uint8_t i=0; i<4; i++) readADC(ADC_VSYS);
		atMegaVolt += readAdcNoiseReduced(ADC_VSYS); // [4092:0]
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
	
	stateFlags.powerState = (sysVin > VIN_CUTOFF)? 1 : 0;
	stateFlags.batteryState = (lipoly > LIPOLY_CUTOFF)? 1 : 0;
	
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
