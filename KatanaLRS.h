#ifndef KATANALRS_DEF_H
#define KATANALRS_DEF_H

//* Preprocessor Directives
// Behavioral Switches
// #define RFM23BP
#define RFM22B

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
#define LIPOLY_CUTOFF	3900
#define VIN_CUTOFF		3900

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
#define I2C_READ		1 // I2C Direction Flags
#define I2C_WRITE		0
// #define SINGLE			0
// #define MULTI			1
#define FAST			1
#define SLOW			0


// Port Definitions and Macros
typedef struct{ // AVR Freaks: JohanEkdahl Aug 5 2008.
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
#define RFM_PMBL	REGISTER_BIT(PORTD,3)
#define RFM_INT		(!(PIND &(1<<2)))
//#define RFM_PMBL	(PIND &(1<<3))
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


//* Global Variables
// General and Timing
static char dataBufferA[BUFFER_SIZE]; //volatile
static volatile uint32_t timer1ms = 0; 
static volatile uint32_t timer1us_p = 0;

// PPM & PWM 
static volatile uint8_t ch;
static volatile uint16_t pwmValues[CHANNELS] = {2000,2000,2000,2000,2000,2000,2000,2000}; // In uSec, 
static volatile uint16_t pwmFrameSum; // Must be able to contain the accumulated sum of pwmValues[]
static uint16_t pwmFailsafes[CHANNELS];

// Radio
const uint8_t dlFreqList[] = { 24,142,169,133,32,96,58,125,87,77,141,177,55,42,121,78,159,138,175,35,86,36};
static volatile uint8_t dlChannel = 0;
static volatile uint32_t timestampPacketRxd = 0;
static volatile int16_t freqOffset = 180;
static uint8_t rfmFIFO[64];

// Radio Statistics
static uint16_t rfmWriteErrors;
static uint8_t noiseFloor = 60; // Start with ~ -80 dBm, will work down from this

// Application
static struct{
	int32_t lat:30;		// 536870912 > 089999999
	int32_t lon:30;		// 536870912 > 179999999
	uint32_t time:20;	// 262143 > 235959
	int16_t alt;		// 16384 Max
	uint8_t sats;		
	uint8_t hdop;		
} gps;

// System and State Machine
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

// typedef enum {DOWN=0, SLEEP=1, BEACON=2, ACTIVE=3, FAILSAFE=7} StateList;			
const uint8_t wdtIntCfgList[] 	= {DISABLED,		ENABLED,		ENABLED,		ENABLED,		ENABLED		};
const uint8_t wdtTimeCfgList[] 	= {0,				9,				8,				5,				5			};
const uint8_t adcSaRateList[] 	= {SLOW,			SLOW,			FAST,			FAST,			FAST		};
const uint8_t rfmModeList[] 	= {RX_TONE_LDC,		RX_TONE_LDC,	IDLE_TUNE,		RX_FHSS_LRS,	RX_FHSS_LRS	};
const uint8_t sleepList[] 		= {9,				9,				0,				0,				0			};

// Final Preprocessor Directives and Includes
#include "i2c.c"
#include "spi.c"

#include "sysUtility.c"

#if defined(RFM22B) || defined(RFM23BP)
	#include "rfm22b.c"
#else
	#error "No Radio Transceiver Selected!"
#endif

#include "elt.c"


//* Function Prototypes
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


#endif // KATANALRS_DEF_H



/*

To-Do:
	The Repeater Access Tone used in the amateur radio world is 1750 Hz, if not CTCSS, which is ~100 Hz.
		Would appear as 3500 bps preamble
		Get Direct Rx Mode working
	It therefore makes sense to add a frequency counter that detect this tone if the RSSI is strong in a sleep mode.
	This way, the device will ignore random signals during DOWN/SLEEP.
	Need to test all beacon states/behaviors.
	Add Packet interaction/decoding:
		CRC-16
		Convert 8-bit SPI to 10-bit UART
		Get Sync Word detection working
		Get the selective bit pattern thing working in the RFM
	Get Bind working once the packet mechanism works
	Get the frequency table implimented, take one final shot and dissecting the DragonLink pattern
	Get WDT to keep things stable. Continue to reduce startup and aquire time.
	Go on a fox hunt with the full beacon states implemented.
	















*/


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

