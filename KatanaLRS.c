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
//    --		   19		ADC6	ADC6	ADC6			Battery Voltage 50% 		VBAT
//    --		   22		ADC7	ADC7	ADC7			Input Voltage 50%			VIN
//											* = 1k Series Resistor
//											^ = 4.7k Pull-up Resistor

//           76543210	7		6		5		4		3		2		1		0
// DDRB = 0b --SsSSLL	XTAL2	XTAL1	SCK		MISO	MOSI	CS_RFM	LED_BL	LED_OR
// DDRC = 0b -0CC____	--		Reset	SCL		SDA		P4		P3		P2		P1
// DDRD = 0b ____ii10	P8		P7		P6		P5		RFM_PBL	RF_INT	TXD		RXD

// Legend: - N/A		_ PWM/GPIO Available	S/s SPI		C I2C		i Interrupt		1/0 Output/Input

// =======================================================================

// Behavioral Switches
//#define RFM22B
//#define RFM23BP
//#define TRANSMITTER
//#define RECEIVER
//#define BATTERY

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

// System Constants
#define DOWN 			0 // Only Wake on Interrupts
#define SLEEP			1 // Sleep, waking using Watchdog
#define STANDBY 		2 // Non-Transmitting State
#define ACTIVE 			3 // Fully-Active Mode
#define INT_SRC_CLEAR	0
#define INT_SRC_INTx	1
#define INT_SRC_WDT		2
#define INT_SRC_UART	3
#define HIGH			1
#define LOW				0
#define READ			1 // Common SPI and I2C Direction Flags
#define WRITE			0
#define SINGLE			0
#define MULTI			1


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
#define VBAT		6 // For ADC Read Channel Selection
#define VIN			7

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
void printMonitor(void);
uint8_t systemSleep(uint8_t);
uint8_t atMegaInit(void);
void radioMode(uint8_t);

uint16_t getSystemV(void);
uint16_t getLipolyV(void);
uint16_t getATmegaV(void);
char deviceIdCheck(void);
void printHelpInfo(void);

static int putUARTchar(char c, FILE *stream);
uint8_t getUARTchar(void);
uint16_t readADC(uint8_t);
void flashLED(uint8_t, uint8_t, uint8_t);

// Global Variables
static FILE uart_io = FDEV_SETUP_STREAM(putUARTchar, NULL, _FDEV_SETUP_WRITE);
static uint8_t dataBufferA[BUFFER_SIZE]; //volatile
static struct{
	uint8_t sleepInterval:3;
	uint8_t wdtSlpEn:1;
} configFlags;
static volatile struct{
	uint8_t systemState:2;
	uint8_t intSource:2;
	uint8_t monitorMode:1;
} stateFlags;

// Interrupt Vectors
ISR(WDT_vect){
	stateFlags.intSource = INT_SRC_WDT;
}

ISR(PCINT2_vect){
	stateFlags.intSource = INT_SRC_UART;
	PCICR = 0;
	PCMSK2 = 0;
}

ISR(INT0_vect){
	stateFlags.intSource = INT_SRC_INTx;
	EIMSK = 0;
}

ISR(INT1_vect){
	stateFlags.intSource = INT_SRC_INTx;
	EIMSK = 0;
}

ISR(USART_RX_vect){
	stateFlags.intSource = INT_SRC_UART;
	stateFlags.monitorMode = 0;
	
	uint8_t command = UDR0;
	
	switch(command){
		case 'B':
			printf("Battery: %u\n", getBatt());
			break;
		case 'M':
			stateFlags.monitorMode = 1;
			break;
		case '?':
			printHelpInfo();
			printTriggerSources();
			break;
		case '1':
			configFlags.onOneTap ^= 1;
			printf("Trigger on 1 Tap: ");
			if(configFlags.onOneTap) printf("Enabled\n");
			else printf("Disabled\n");
			break;
		case '`':
			eeprom_update_byte((uint8_t*)EEPROM_START,(*(uint8_t*) &configFlags));
			break;
		default:
			break;
	}	
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
	printf("\n\nKatanaLRS v1\nBy Steve Carlson\n\n");
	printf("Reset Source: "); //%X\n", startStatus);
	if(startStatus|WDRF) printf("WatchDog\n"); // From iom328p.h in AVR Include Folder
	if(startStatus|BORF) printf("BrownOut\n");
	if(startStatus|EXTRF) printf("External\n");
	if(startStatus|PORF) printf("PowerOn\n\n");
	//	WDRF BORF EXTRF PORF
	
	flashLED(10,10,40);
	
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
	//static uint8_t wdtIntCntDwn = 0;
	
	LED_OR = HIGH;
	_delay_us(5);
	LED_OR = LOW;
	_delay_ms(1);

}

void printMonitor(void){
	
	printf("_T\t%u",TCNT1);

	printf("\n");
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
	
	wdt_reset();
	//uint8_t value = (uint8_t)( ((configFlags.wdtSlpEn)<<WDIE) | (interval & 0x08? (1<<WDP3): 0x00) | (interval & 0x07) );
	MCUSR = 0;
	WDTCSR |= (1<<WDCE)|(1<<WDE);
	WDTCSR = 0; //value;
	
	PCMSK2 = (1<<PCINT16);
	PCICR = (1<<PCIE2);

	EICRA = 0;
	EIMSK = (1<<INT1); //|(1<<INT0);
	
	
	// if(stateFlags.systemState == DOWN)			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	// else if(stateFlags.systemState == SLEEP) 	set_sleep_mode(SLEEP_MODE_STANDBY);
	// else										set_sleep_mode(SLEEP_MODE_IDLE);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	sleep_bod_disable();
	sei();
	sleep_cpu();
	
	sleep_disable();
	wdt_reset();
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
	WDTCSR = 0;
	wdt_reset();
	
	// System
	//MCUCR |= (1<<PUD);		// Pull-up Disable
	MCUCR = 0;
	PRR = 0;

	// Timers
	TCCR1A = 0;
	TCCR1B = (1<<CS12); //(1<<CS11)|(1<<CS10); //
	
	// IO Ports
	// 0: Input (Hi-Z) 1: Output
	//        76543210		7		6		5		4		3		2		1		0
	DDRB |= 0b00101111; //	XTAL2	XTAL1	SCK		MISO	MOSI	CS_RFM	LED_BL	LED_OR
    DDRC |= 0b00000000; //	--		Reset	SCL		SDA		P4		P3		P2		P1
    DDRD |= 0b00000010; //	P8		P7		P6		P5		RFM_PBL	RF_INT	TXD		RXD
	//PORTB |=0b00111111;
	
	// Serial Port
	UBRR0H = UART_UBRR >> 8;
    UBRR0L = UART_UBRR;
    UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
    stdout = &uart_io; //= stdin 
	
	//SPI
	SPCR	= (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA)|(1<<SPR0);
	
	//I2C
	TWCR = (1<<TWEN) | (1<<TWEA);
	TWSR &= ~((1<<TWPS1) | (1<<TWPS0));
	TWBR = ((F_CPU / I2C_FREQ) - 16) / 2;
	
	// ADC
	ADMUX 	= (1<<REFS0);	// AVcc Connected
	ADCSRA 	= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);
	DIDR0 	= (1<<ADC5D)|(1<<ADC4D)|(1<<ADC3D)|(1<<ADC2D)|(1<<ADC1D)|(1<<ADC0D);

	//PCICR = 0; //(1<<PCIE2);
	//PCMSK2 = 0; //(1<<PCINT16);
	
	EICRA = 0;
	EIMSK = 0; //(1<<INT1)|(1<<INT0);
	
	sei();
	
	return startupStatus;
}

void radioMode(uint8_t mode){
	#if defined(RFM22B)

	#endif
}

uint16_t getSystemV(void){
	uint16_t voltSample = readADC(VIN);
	// 3.300 System Voltage, 50% divider
	// (sample / 1023) * 2 * 3.3v
	voltSample = ((voltSample<<2)+(voltSample<<1)+(voltSample>>1));
	return voltSample;
}

uint16_t getLipolyV(void){
	uint16_t voltSample = readADC(VBAT);
	voltSample = ((voltSample<<2)+(voltSample<<1)+(voltSample>>1));
	return voltSample;
}

uint16_t getATmegaV(void){
	uint16_t voltSample = readADC(14);
	//voltSample = 1100*1023/voltSample;
	voltSample = 5353 - ((voltSample<<2)+(voltSample<<1)+(voltSample>>2));
	return voltSample;
}

char deviceIdCheck(void){
	return 1;
	//return 0;
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
	ADMUX 	= (1<<REFS0) | adcChannel;
	ADCSRA 	|= (1<<ADSC);
	while (ADCSRA & (1 << ADSC));
	return (ADCL + ((uint16_t) ADCH << 8));
}

void flashLED(uint8_t count, uint8_t high, uint8_t low){
	for(;count>0; count--){
		LED = HIGH;
		_delay_ms(high);
		LED = LOW;
		_delay_ms(low);
	}
}
