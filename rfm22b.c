// RFM22B and RFM23BP Transceiver Library, by Stephen Carlson, April 2014
// Use Notepad++ with Tab=4

// Master List
	// Address		Reset Value			   Description		Setting					R/W		R/O Bits
	// 0x00			0x08				// Device Type								R
	// 0x01			-					// Device Version	0x06 for Rev B1			R
	// 0x02			-					// Device Status							R
	// 0x03			-					// Int Status 1								R
	// 0x04			-					// Int Status 2								R
	// 0x05			0x00				// Int Enable 1								RW
	// 0x06			0x03				// Int Enable 2		Chip Ready and POR		RW
	// 0x07			0x01				// Control 1		Xtal On (Ready)			RW
	// 0x08			0x00				// Control 2								RW
	// 0x09			0x7F				// Xtal Cap 		Default @ 12.5pF		RW
	// 0x0A			0x06				// uC Output Clk	1 MHz					RW		xx
	// 0x0B			0x00				// GPIO1 			POR						RW
	// 0x0C			0x00				// GPIO2 			/POR					RW
	// 0x0D			0x00				// GPIO3 			uC Clk Output			RW
	// 0x0E			0x00				// I/O Ports								RW
	// 0x0F			0x00				// ADC Config 								RW
	// 0x10			0x00				// ADC Offset								RW		xxxx
	// 0x11			-					// ADC Value								R
	// 0x12			0x20				// Temp Cal			Celsius					RW
	// 0x13			0x00				// Temp Offset								RW
	// 0x14			0x03				// Wake Set 1		R=3						RW		xxx
	// 0x15			0x00				// Wake Set 2		T_WUT=4*M*2^R / 2^15	RW
	// 0x16			0x01				// Wake Set 3		M=1 					RW
	// 0x17			-					// Wake Val 1								R
	// 0x18			-					// Wake Val 2								R
	// 0x19			0x00				// Low Cyc Dur		LDC/M Wake Ratio		RW
	// 0x1A			0x14				// Low Batt			2.7v (1.7+20*50mV)		RW		xxx
	// 0x1B			-					// Batt Level								R
	// 0x1C			0x01				// IF B/W			75.2 kHz				RW
	// 0x1D			0x40				// AFC Gear			AFC On, 0dB pass		RW
	// 0x1E			0x0A				// AFC Timing		<>						RW		xx
	// 0x1F			0x03				// Clk Gear			<>						RW
	// 0x20			0x64				// Clk Ratio		<>						RW
	// 0x21			0x01				// Clk Offset 2		<>						RW
	// 0x22			0x47				// Clk Offset 1		<>						RW
	// 0x23			0xAE				// Clk Offset 0		<>						RW
	// 0x24			0x02				// Clk Gain 1		<>						RW
	// 0x25			0x8F				// Clk Gain 0		<>						RW
	// 0x26			-					// RSSI										R
	// 0x27			0x1E				// RSSI Thresh		30 -> ~ -110 dBm		RW
	// 0x28			-					// Ant Divsty 1								R
	// 0x29			-					// Ant Divsty 2								R
	// 0x2A			0x00				// AFC Limit								RW
	// 0x2B			0x00				// AFC Correct								R
	// 0x2C			0x18				// OOK Control		Freeze, Peak Detr On	RW
	// 0x2D			0xBC				// OOK Counter		<>						RW
	// 0x2E			0x26				// OOK Slicer		<>						RW
	// 0x2F								// Reserved									-
	// 0x30			0x8D				// Data Ctrl		Rx/Tx Pkt'ing, CRC On	RW
	// 0x31			-					// ExMAC Status								R
	// 0x32			0x0C				// Header Ctrl 1	Rx'd Hdr is Byte 3,2	RW
	// 0x33			0x22				// Header Ctrl 2	Hdr 3,2; Sync 3,2		RW
	// 0x34			0x08				// Preamble Length	32 bits / 4 bytes		RW
	// 0x35			0x2A				// Preamble Detect	20 bits, +8dB RSSI		RW
	// 0x36			0x2D				// Sync Word 3		0010110111010100		RW
	// 0x37			0xD4				// Sync Word 2								RW
	// 0x38			0x00				// Sync Word 1								RW
	// 0x39			0x00				// Sync Word 0								RW
	// 0x3A			0x00				// Tx Header 3								RW
	// 0x3B			0x00				// Tx Header 2								RW
	// 0x3C			0x00				// Tx Header 1								RW
	// 0x3D			0x00				// Tx Header 0								RW
	// 0x3E			0x00				// Tx Pkt Length	No data sent in pkt		RW
	// 0x3F			0x00				// Check Header 3							RW
	// 0x40			0x00				// Check Header 2							RW
	// 0x41			0x00				// Check Header 1							RW
	// 0x42			0x00				// Check Header 0							RW
	// 0x43			0xFF				// Header Enable 3	All bits compared		RW
	// 0x44			0xFF				// Header Enable 2	All bits compared		RW
	// 0x45			0xFF				// Header Enable 1	All bits compared		RW
	// 0x46			0xFF				// Header Enable 0	All bits compared		RW
	// 0x47			-					// Rx'd Header 3							R
	// 0x48			-					// Rx'd Header 2							R
	// 0x49			-					// Rx'd Header 1							R
	// 0x4A			-					// Rx'd Header 0							R
	// 0x4B			-					// Rx'd Pkt Length							R
	// 0x4C	- 0x4E						// Reserved									-
	// 0x4F			0x10				// ADC8 Temp Set	<>						RW
	// 0x50 - 0x5F						// Reserved									-
	// 0x60			0x00				// Preamble	Thresh							RW
	// 0x61								// Reserved									-
	// 0x62			0x04				// Xtal / Test		2x Amplify				RW		xxx
	// 0x63	- 0x68						// Reserved									-
	// 0x69			0x20				// AGC Override		AGC Enabled				RW
	// 0x6A	- 0x6C						// Reserved									-
	// 0x6D			0x18				// Tx Power			LNA Sw Active, -1dBm	RW		x
	// 0x6E			0x0A				// Tx Data Rate 1	<>						RW
	// 0x6F			0x3D				// Tx Data Rate 0	40 kbps					RW
	// 0x70			0x0C				// Modulation 1		Normal					RW
	// 0x71			0x00				// Modulation 2		No Modulation			RW
	// 0x72			0x20				// Freq Deviation	<>						RW
	// 0x73			0x00				// Freq Offset 1							RW
	// 0x74			0x00				// Freq Offset 2							RW
	// 0x75			0x75				// Freq Band		<>						RW
	// 0x76			0xBB				// Freq Carrier 1	<>						RW
	// 0x77			0x80				// Freq Carrier 0	<>						RW
	// 0x78								// Reserved									-
	// 0x79			0x00				// FHSS Channel								RW
	// 0x7A			0x00				// FHSS Step Size							RW
	// 0x7B								// Reserved									-
	// 0x7C			0x37				// Tx FIFO Ctrl 1	Tx Almost Full @ 55		RW
	// 0x7D			0x04				// Tx FIFO Ctrl 2	Tx Almost Empt @ 4		RW
	// 0x7E			0x37				// Rx FIFO Ctrl		Rx Almost Full @ 55		RW
	// 0x7F			-					// FIFO				FIFOs are 64B each		RW
	


// By Type and Feature:  // All line gaps intentional
// Read Only
	// 0x00			0x08				// Device Type								R
	// 0x01			-					// Device Version	0x06 for Rev B1			R
	
	// 0x02			-					// Device Status							R
	// 0x03			-					// Int Status 1								R
	// 0x04			-					// Int Status 2								R
	// 0x17			-					// Wake Val 1								R
	// 0x18			-					// Wake Val 2								R
	
	// 0x26			-					// RSSI										R
	// 0x28			-					// Ant Divsty 1								R
	// 0x29			-					// Ant Divsty 2								R
	// 0x2B			0x00				// AFC Correct								R
	
	// 0x31			-					// ExMAC Status								R
	// 0x47			-					// Rx'd Header 3							R
	// 0x48			-					// Rx'd Header 2							R
	// 0x49			-					// Rx'd Header 1							R
	// 0x4A			-					// Rx'd Header 0							R
	// 0x4B			-					// Rx'd Pkt Length							R

	// 0x11			-					// ADC Value								R
	// 0x1B			-					// Batt Level								R
	

// RFM IC Configuration
	// 0x09			0x7F				// Xtal Cap 		Default @ 12.5pF		RW

// RFM Ports, GPIO, and uC Interface
	// 0x0B			0x00				// GPIO1 			POR						RW
	// 0x0C			0x00				// GPIO2 			/POR					RW
	// 0x0D			0x00				// GPIO3 			uC Clk Output			RW
	// 0x0E			0x00				// I/O Ports								RW
	// 0x05			0x00				// Int Enable 1								RW
	// 0x06			0x03				// Int Enable 2		Chip Ready and POR		RW

// RFM Top-Level State Control
	// 0x07			0x01				// Control 1		Xtal On (Ready)			RW
	// 0x08			0x00				// Control 2								RW


// Accessories
	// 0x0A			0x06				// uC Output Clk	1 MHz					RW
	// 0x0F			0x00				// ADC Config 								RW
	// 0x10			0x00				// ADC Offset								RW
	// 0x12			0x20				// Temp Calc		Celsius					RW
	// 0x13			0x00				// Temp Offset								RW
	// 0x4F			0x10				// ADC8 Temp Set	<>						RW
	// 0x1A			0x14				// Low Batt			2.7v (1.7+20*50mV)		RW
	
	// Low-Duty-Cycle Module
	// 0x14			0x03				// Wake Set 1		R=3						RW
	// 0x15			0x00				// Wake Set 2		T_WUT=4*M*2^R / 2^15	RW
	// 0x16			0x01				// Wake Set 3		M=1 					RW
	// 0x19			0x00				// Low Cyc Dur		LDC/M Wake Ratio		RW



// Modem RF Configuration - Primatives and Low-Level
	// 0x1C			0x01				// IF B/W			75.2 kHz				RW
	// 0x1F			0x03				// Clk Gear			<>						RW
	// 0x20			0x64				// Clk Ratio		<>						RW
	// 0x21			0x01				// Clk Offset 2		<>						RW
	// 0x22			0x47				// Clk Offset 1		<>						RW
	// 0x23			0xAE				// Clk Offset 0		<>						RW
	// 0x24			0x02				// Clk Gain 1		<>						RW
	// 0x25			0x8F				// Clk Gain 0		<>						RW
	// 0x1D			0x40				// AFC Gear			AFC On, 0dB pass		RW
	// 0x1E			0x0A				// AFC Timing		<>						RW
	// 0x2A			0x00				// AFC Limit								RW
	
	// Amplifier
	// 0x62			0x24				// Xtal / Test		2x Amplify				RW
	// 0x69			0x20				// AGC Override		AGC Enabled				RW
	
	// OOK Module
	// 0x2C			0x18				// OOK Control		Freeze, Peak Detr On	RW
	// 0x2D			0xBC				// OOK Counter		<>						RW
	// 0x2E			0x26				// OOK Slicer		<>						RW
	
// Modem RF Configuration - RSSI Related
	// 0x27			0x1E				// RSSI Thresh		30 -> ~ -110 dBm		RW


// Packet Configuration and Handling ( Can be skipped for Tx Only)
	// 0x30			0x8D				// Data Ctrl		Rx/Tx Pkt'ing, CRC On	RW
	// 0x32			0x0C				// Header Ctrl 1	Rx'd Hdr is Byte 3,2	RW
	// 0x33			0x22				// Header Ctrl 2	Hdr 3,2; Sync 3,2		RW
	// 0x34			0x08				// Preamble Length	32 bits / 4 bytes		RW
	// 0x35			0x2A				// Preamble Detect	20 bits, +8dB RSSI		RW
	// 0x60			0x00				// Preamble	Thresh							RW
	// 0x36			0x2D				// Sync Word 3		0010110111010100		RW
	// 0x37			0xD4				// Sync Word 2								RW
	// 0x38			0x00				// Sync Word 1								RW
	// 0x39			0x00				// Sync Word 0								RW
	// 0x3A			0x00				// Tx Header 3								RW
	// 0x3B			0x00				// Tx Header 2								RW
	// 0x3C			0x00				// Tx Header 1								RW
	// 0x3D			0x00				// Tx Header 0								RW
	// 0x3E			0x00				// Tx Pkt Length	No data sent in pkt		RW
	// 0x3F			0x00				// Check Header 3							RW
	// 0x40			0x00				// Check Header 2							RW
	// 0x41			0x00				// Check Header 1							RW
	// 0x42			0x00				// Check Header 0							RW
	// 0x43			0xFF				// Header Enable 3	All bits compared		RW
	// 0x44			0xFF				// Header Enable 2	All bits compared		RW
	// 0x45			0xFF				// Header Enable 1	All bits compared		RW
	// 0x46			0xFF				// Header Enable 0	All bits compared		RW

// Radio Modulation and FHSS
	// 0x6D			0x18				// Tx Power			LNA Sw Active, -1dBm	RW
	// 0x6E			0x0A				// Tx Data Rate 1	<>						RW
	// 0x6F			0x3D				// Tx Data Rate 0	40 kbps					RW
	// 0x70			0x0C				// Modulation 1		Normal					RW
	// 0x71			0x00				// Modulation 2		No Modulation			RW
	// 0x72			0x20				// Freq Deviation	<>						RW
	// 0x73			0x00				// Freq Offset 1							RW
	// 0x74			0x00				// Freq Offset 2							RW
	// 0x75			0x75				// Freq Band		<>						RW
	// 0x76			0xBB				// Freq Carrier 1	<>						RW
	// 0x77			0x80				// Freq Carrier 0	<>						RW
	// 0x79			0x00				// FHSS Channel								RW
	// 0x7A			0x00				// FHSS Step Size							RW
	
// FIFO and Data I/O
	// 0x7C			0x37				// Tx FIFO Ctrl 1	Tx Almost Full @ 55		RW
	// 0x7D			0x04				// Tx FIFO Ctrl 2	Tx Almost Empt @ 4		RW
	// 0x7E			0x37				// Rx FIFO Ctrl		Rx Almost Full @ 55		RW
	// 0x7F			-					// FIFO				FIFOs are 64B each		RW





#ifndef RFM22B_H
#define RFM22B_H

#include "KatanaLRS.h"

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


const uint8_t configCore[][2] = { // Start the device inert but ready, interrupts set
	// Set critical components in priority order
	{0x09,0x7F},		// Xtal Cap 		Default @ 12.5pF
	{0x0B,0b00010010},	// GPIO1 			Tx
	{0x0C,0b00010101},	// GPIO2 			Rx
	{0x0D,0b00010100},	// GPIO3 			Sync=11011 RxData=10100 Preamble=11001 RxFifoFull=10110 RxState=10101 WUT=00001
	{0x0E,0},			// I/O Ports		None
	{0x05,0x00},		// Int Enable 1		
	{0x06,0x03},		// Int Enable 2		Chip Ready and POR
	{0x07,0x01},		// Control 1		Xtal On (Ready)
	{0x08,0},			// Control 2		
	
	// Disable OOK Module
	{0x2C,0},			// OOK Control		Disabled
	{0x2D,0},			// OOK Counter		Disabled
	{0x2E,0},			// OOK Slicer		Disabled
	
	// Disable various auxiliary devices that are not used
	{0x0A,0x07},		// uC Output Clk	32 kHz
	{0x0F,0x00},		// ADC Config 		Disabled
	{0x10,0x00},		// ADC Offset		None
	{0x12,0x20},		// Temp Calc		Celsius
	{0x13,0x00},		// Temp Offset		None
	{0x4F,0x10},		// ADC8 Temp Set	<>
	{0x1A,0x14},		// Low Batt			2.7v (1.7+20*50mV)
	
	// Configure LDC Mechanism for 16 ms every 2 sec
	{0x14,7},			// Wake Set 1		R=7
	{0x15,0},			// Wake Set 2		T_WUT=4*M*2^R / 2^15
	{0x16,128},			// Wake Set 3		M=128
	{0x19,1},			// Low Cyc Dur		LDC/M Wake Ratio
	
	// 430 MHz Band, +5kHz Calibration, FHSS Off
	{0x73,0},			// Freq Offset 1	+5 kHz Calibration					
	{0x74,0x00},		// Freq Offset 2						
	{0x75,0x53},		// Freq Band		430 MHz Band	
	{0x79,0x00},		// FHSS Channel							
	{0x7A,0x00},		// FHSS Step Size							

};

const uint8_t configDlCompat[][2] = {
	// Freq Range between 430.5 and 434.950 MHz, Center is 432.725
	// 9600 Baud
	// 5.4 kHz Deviation
	// Gaussian Shift
	// fcarrier = (fb+24+(fc+fo)/64000) * 10000 * (hbsel+1) + (fhch * fhs * 10) [kHz]
	// fb=19 (10 MHz Increments), fo=32 (+5 kHz Up Tick), fc=3200 (500 kHz)
	// fo can only shift by +/-80 kHz, 2's comp
	// Use fc as channel select mechanism, unsigned for 10 MHz span
	
	// ..11111101010101010101 00000100 10100001 01000011 11011000 0.....
	// Idle   | Preamble    |
	
	// 04A1 or 2085
	// 10000010 01010000
	// 8250 or 410A
	
	// 0A E1 00001010 11100001
	
	// Actual bit order, LSB is transmitted first
	// ...111111 10 10101010 10 10100000 10 01010000 10 10000111 10 110000....
	//              55          05          0A          E1          MSBs
	// ...111111101010101010101000001001010000101000011110110000....
	//          ||55      ||05      ||0A      ||E1      ||MSBs
	//                        10000010 01010000 10100001 1110
	//                                                          
	//                         00000100 10100001 01000011 110
	//                                                        
	//                          00001001 01000010 10000111 10
	//                              ||0A       || E1       ||MSBs
	//                   *      90       42       E1
	// Bind Packet               
	//          PPPPPPPPPPPPPPPPSSSSSS####  BF  NNNNNNNN  DDDDDDD.  
	// ....11111101010101010101000001011000010101000011110111000....
	//          ||55      ||05      ||43      ||E1      ||Stuff
	//                          00001011 00001010 10000111 10
	//                   *      D0       50       E1
	//     Alt: PPPPPPPPPP.SSSSSSSS.HHHHLLLL.HHBFCCCC.CCCCCCCC.DDDDDDDD
	// ....111111010101010.10101000.00101100.00101010.00011110.111000.
	// P=Preamble, S=Sync, H=HeaderFiller, L=LengthBits, B=BindFlag, F=FailSafeFlag, C=CheckBits, D=Data
	//            2nibs    15       x4       5y       78
	//                     15       34       54       78
	//       relevant               04       53       78
	//       bitmask                0F       F3       FF
	// 
	// What I'm getting with the DO50E1 Sync Packet and Bind mode:
	// !	1D,84,9F,D1,FF,FF,FF,FF,FF,FF.....
	// Actual bit field:
	// 111111010101010101010000010110000101010000111 101110000010000111111001100010 1111111111111111
	//      ||55      ||05      ||43      ||E1       ||07      ||F8      ||46       |
	//                                               10111000 00100001 11111001 100010 11 111
	//                                               1D       84       9F       D1
	// Total Mess, here are the final rules for descrambling the byte stream:
	// Media Rep:	ss012345 67ss0123 4567ss01 234567ss 01234567
	// Output Rep:	543210ss 3210ss76 10ss7654 ss765432 76543210
	// Group:		aaaaaa   bbbb  aa cc  bbbb   cccccc dddddddd
	
	// Reversed to match normal byte bit order MSB first
	// 00001100 10000010 10110000 11111110 11111110 01001000 11111110 00000110 10000011 11111110 00010101 10000110 11100001 01001101
	// A   B      C        D        E1       E2       E3       E4       E5       E6       E7       E8       E9       F1       F2

	// A	Flags: Bind and Failsafe Inactive
	// B	Byte Count: 12 Bytes beyond ID byte (D)
	// C	System ID: 0x82
	// D	MSB's for lower 8 channels
	// En	9 Channels Payload
	// F1	Upper Byte of CRC-16 IBM/ARC
	// F2	Lower Byte of CRC-16 IBM/ARC
	
	
	// Timing (for 0xE1)
	// 45.0 Packets per second, from Saleae swipes
	// 0.30 ms for DL to write a new frequency, from CS down to CS up
	// 10 Packets after 0xE1, or 100 bits, 12.5 bytes
	// Each channel is visited ever 489.2 ms, meaning there are 22 channels
	// For 0xE1, there are 22 channels. Others are less, but all are 45 Hz.
	
	// Frequency Configuration
	{0x1C,0x27},		// IF B/W			75.2 kHz	// Only Value Freq Sensitive
	{0x1F,0x03},		// Clk Gear			<>	
	{0x20,0x68},		// Clk Ratio		<>	
	{0x21,0x01},		// Clk Offset 2		<>	
	{0x22,0x3A},		// Clk Offset 1		<>	
	{0x23,0x93},		// Clk Offset 0		<>	
	{0x24,0x04},		// Clk Gain 1		<>	Max.Rb Error affects this: rxncocomp
	{0x25,0x62},		// Clk Gain 0		<>	
	{0x1D,0x04},		// AFC Gear			AFC On, 0dB pass
	{0x1E,0x0A},		// AFC Timing		<>				
	{0x2A,0x1D},		// AFC Limit		Was 0x1D			
	
	// Amplifier
	// Allow to default {0x62,0x04},		// Xtal / Test		2x Amplify	
	{0x69,0x60},		// AGC Override		AGC Enabled		
	
	
	// 00001100 10000010 10110000 11111110 11111110 01001000 11111110 00000110 10000011 11111110 00010101 10000110 11100001 01001101
	// A   B      C        D        E1       E2       E3       E4       E5       E6       E7       E8       E9       F1       F2
	// 
	// A	Flags: Bind and Failsafe Inactive
	// B	Byte Count: 12 Bytes beyond ID byte (D)
	// C	System ID: 0x82
	// D	MSB's for lower 8 channels
	// En	9 Channels Payload
	// F1	Upper Byte of CRC-16 IBM/ARC
	// F2	Lower Byte of CRC-16 IBM/ARC
	
	// Packet and FIFO
	{0x30,0xC0},		// Data Ctrl		No PH, as data is 10-bit UART, must chew on it in uC. LSB First
	{0x32,0x07},		// Header Ctrl 1	Rx'd Hdr is Byte 3,2
	{0x33,0x38},		// Header Ctrl 2	Hdr Defined by 3E ; Sync 3,2,0	
	{0x34,0x03},		// Preamble Length	3 nibbles
	{0x35,0x18},		// Preamble Detect	3 nibbles
	{0x60,0x00},		// Preamble	Thresh						
	{0x36,0x15},		// Sync Word 3		90, D0
	{0x37,0x00},		// Sync Word 2		42, 50					
	{0x38,0x00},		// Sync Word 1		Always E1					
	{0x39,0x00},		// Sync Word 0							
	{0x3A,0x00},		// Tx Header 3							
	{0x3B,0x00},		// Tx Header 2							
	{0x3C,0x00},		// Tx Header 1							
	{0x3D,0x00},		// Tx Header 0							
	{0x3E,0x0D},		// Tx Pkt Length	13 Bytes per packet
	{0x3F,0x34},		// Check Header 3						
	{0x40,0x54},		// Check Header 2						
	{0x41,0x78},		// Check Header 1						
	{0x42,0x00},		// Check Header 0						
	{0x43,0x0F},		// Header Enable 3	All bits compared	
	{0x44,0xF3},		// Header Enable 2	All bits compared	
	{0x45,0xFF},		// Header Enable 1	All bits compared	
	{0x46,0x00},		// Header Enable 0	All bits compared	
	{0x7C,0x37},		// Tx FIFO Ctrl 1	Tx Almost Full @ 55	
	{0x7D,0x04},		// Tx FIFO Ctrl 2	Tx Almost Empt @ 4	
	{0x7E,0x20},		// Rx FIFO Ctrl		Rx Almost Full @ 32	
	
	// Modulation
	{0x6D,0x08},		// Tx Power			LNA Sw Active, -1dBm
	{0x6E,0x4E},		// Tx Data Rate 1	<>					
	{0x6F,0xA5},		// Tx Data Rate 0	9600 Baud				
	{0x70,0x20},		// Modulation 1		Normal				
	{0x71,0x23},		// Modulation 2		FIFO GFSK		
	{0x72,0x09},		// Freq Deviation	<>					
	{0x76,0x0C},		// Freq Carrier 1	<>					
	{0x77,0x80},		// Freq Carrier 0	<>					
};

const uint8_t configRxTone1750[][2] = {
	// Frequency: 434 MHz 3500 bit/sec @ 5 kHz Dev, for detecting a 1750 Hz Tone
	{0x1C, 0x2B},		// IF Filter Bandwidth
	{0x1F, 0x03},		// Clock Recovery Gearshift Override
	{0x20, 0x1E},		// Clock Recovery Oversampling Rate
	{0x21, 0x20},		// Clock Recovery Offset 2
	{0x22, 0x72},		// Clock Recovery Offset 1
	{0x23, 0xB0},		// Clock Recovery Offset 0
	{0x24, 0x00},		// Clock Recovery Timing Loop Gain 1
	{0x25, 0xA2},		// Clock Recovery Timing Loop Gain 0
	{0x1D, 0x44},		// AFC Loop Gearshift Override
	{0x1E, 0x0A},		// AFC Timing Control
	{0x2A, 0x30},		// AFC Limiter
	{0x69,0x60},		// AGC Override		AGC Enabled	
	
	// Packet
	{0x30,0x80},		// Packet Handler on, 
	{0x32,0x01},		// 1 Headers
	{0x33,0x18},		// 1 Headers, 1 Sync
	{0x34,0x20},		// 64 nibble = 32 byte preamble
	{0x35,0x20},		// 0x35 need to detect 20bit preamble
	{0x36,0x55},
	//{0x37,0x55},
	//{0x38,0x55},
	//{0x39,0x55},
	{0x3E,0x0F},
	{0x3F,0x55},
	//{0x40,0x55},
	{0x43,0xFF},
	//{0x44,0xFF},
	{0x7E,0x20},
	
	{0x60,0x00},		// Preamble	Thresh
	// ...
	
	// Modulation
	{0x6D,0x08},		// Tx Power			LNA Sw Active, -1dBm
	{0x6E,0x1C},		// Tx Data Rate 1	<>					
	{0x6F,0xAC},		// Tx Data Rate 0	40 kbps				
	{0x70,0x20},		// Modulation 1		Normal				
	{0x71,0x23},		// Modulation 2		No Modulation		
	{0x72,0x08},		// Freq Deviation	<>								
	{0x76,0x64},		// Freq Carrier 1	434 MHz					
	{0x77,0x00},		// Freq Carrier 0	<>					
	
	// LDC
	{0x14, 7},			// R in T_WUT = 4 * M * 2^R / 32768, .015625*M ms for R=7 (2^7=128)
	{0x15, 0},			// M[15:8]
	{0x16, 128},		// M[7:0]
	{0x19, 1},			// LDC in T_LDC_ON = 4 * LDC * 2^R / 32768
	
	// Device State
	// {0x07, RFM_xton},
	
	// Preamble is 1750 -- 3500 bps, .032 sec -> 56 bits, can capture 16 bits in that time
	// .016 TLDC is .015625 * 32768 /4 
};

void rfmIntConfig(uint8_t mode, uint8_t rssiThresh){
	if(mode == ENABLED){
		
		// Configure for 1750 Hz Tone Detection
		// for(uint8_t i=0; i<sizeof(configRxTone1750)/2;i++){
			// radioWriteReg(configRxTone1750[i][0],configRxTone1750[i][1]);
		// }
		
		
		//radioWriteReg(0x05, (1<<1)); // RX FIFO Almost Full
		radioWriteReg(0x06, (1<<7)); // RX FIFO Almost Full
		//radioWriteReg(0x05, (1<<1)); // Enable Valid Packet Received Interrupt
		// radioWriteReg(0x05, (1<<4)); // Enable Valid Packet Received Interrupt
		//radioWriteReg(0x06, (1<<7)|(1<<6)); // (1<<4)| Enable RSSI Interrupt
		//radioWriteReg(0x27, (rssiThresh)); // Configure RSSI for +30dBm Level Threshold
		// {0x07, (1<<5)},	// Wake Up Timer Enabled
		
	} else{
		radioWriteReg(0x05, 0);
		radioWriteReg(0x06, 0);
		radioWriteReg(0x07, 0);
	}
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
	
	radioWriteReg(0x76,value>>8);	// Freq Carrier 1	Upper Byte
	radioWriteReg(0x77,value&0xFF);	// Freq Carrier 0	Lower Byte
}

void radioMode(uint8_t mode){
	
	for(uint8_t i=0; i<sizeof(configCore)/2;i++){
		radioWriteReg(configCore[i][0],configCore[i][1]);
	}
	
	// for(uint8_t i=0; i<sizeof(configDlCompat)/2;i++){
		// radioWriteReg(configDlCompat[i][0],configDlCompat[i][1]);
	// }
	
	for(uint8_t i=0; i<sizeof(configRxTone1750)/2;i++){
		radioWriteReg(configRxTone1750[i][0],configRxTone1750[i][1]);
	}
}

uint8_t radioWriteReg(uint8_t regAddress, uint8_t regValue){
	CS_RFM = LOW;
		transferSPI((RFM_WRITE<<7) | regAddress);
		transferSPI(regValue);
	CS_RFM = HIGH;
	_delay_us(2);
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
	uint8_t rssiMeasure = 0;
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
		rssiMeasure = radioReadReg(0x26);
		radioWriteReg(OPCONTROL1_REG, 0); //(1<<RFM_xton));
	}
	
	return rssiMeasure; //
}

void rfmReadFIFO(uint8_t *array){
	for(uint8_t i=0; i<64; i++){
		array[i] = radioReadReg(0x7F);
	}
}

uint16_t rfmReadIntrpts(void){
	uint16_t rfmIntList = radioReadReg(0x03);
	rfmIntList |= radioReadReg(0x04)<<8;
	return rfmIntList;
}



#endif // RFM22B_H





#if defined(Notes_for_here)

// Pesudo-code


uint16_t rfmLearnHopSeq(uint8_t type){
	// Find the master list, the set of frequencies that have channels
	for each channel in the list of possible channels for given type:
		observe the channel for at least 1 sec
		if RSSI breaks threshold inside of that second, we've found a valid channel
			Add the channel to the candidate list, with timestamp
			if we've got 5+ entries in the candidate list:
				add this channel the master list, exit to next channel in for loop
			otherwise, repeat the 1 second observation period
			
	now, with the master list
		
		
		
		
		
		
		
		if(timeout <1 sec && ) // dwell on channel for about a second:
			if RSSI exceeds the threshould:
				make entry in list with timestamp
				timeout = 0, such that we persist on this channel
			if there are 5 entries in the array:
				if they are evenly spaced in time, +/- a bit:
					We have observed the periodic cycle for the FHSS pattern
					Mark this channel as valid, append to FHSS list
			
				
	for each channel on the FHSS list from previously:
		The moment the RSSI for the index i frequency breaks threshould:
			Switch frequency to index i++, i=0 if we saturated
			If RSSI shows a hit at +/- 10% expected packet arrival:
				.......
				Basically, learn the sequence via bubble-sort basically
				Or is it another sort of sort?








}


#endif




