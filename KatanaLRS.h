#ifndef KATANALRS_DEF_H
#define KATANALRS_DEF_H


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


// Global Variables
static char dataBufferA[BUFFER_SIZE]; //volatile
static volatile uint8_t timer10min = 0; // Rolls at 42.5 Hours
static volatile uint16_t timer10ms = 0; // Reset at 10 min

static uint16_t rfmWriteErrors;
static uint8_t noiseFloor = 60; // Start with ~ -80 dBm, will work down from this


static volatile uint8_t ch;
static volatile uint16_t pwmValues[CHANNELS] = {1200,1200,1200,1200,1200,1200,1200,1200}; // In uSec, 
static volatile uint16_t pwmFrameSum; // Must be able to contain the accumulated sum of pwmValues[]

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

static struct{
	int32_t lat:30;		// 536870912 > 089999999
	int32_t lon:30;		// 536870912 > 179999999
	uint32_t time:20;	// 262143 > 235959
	int16_t alt;			// 16384 Max
	uint8_t sats;		
	uint8_t hdop;		
} gps;

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

#include "i2c.c"
#include "spi.c"

#include "sysUtility.c"



#if defined(RFM22B)
	#include "rfm22b.c"
#elif defined(RFM23BP)
	#include "rfm23bp.c"
#else
	#error "No Radio Transceiver Selected!"
#endif


#include "elt.c"

#endif // KATANALRS_DEF_H