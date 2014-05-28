#ifndef RFM22B_H
#define RFM22B_H


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
	// 0x0B			0x00				// GPIO0 			POR						RW
	// 0x0C			0x00				// GPIO1 			/POR					RW
	// 0x0D			0x00				// GPIO2 			uC Clk Output			RW
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
	// 0x0B			0x00				// GPIO0 			POR						RW
	// 0x0C			0x00				// GPIO1 			/POR					RW
	// 0x0D			0x00				// GPIO2 			uC Clk Output			RW
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




#include "KatanaLRS.h"

#define RFM_DEV_TYPE		0x00
#define RFM_dt(v)				(v)&0x1F

#define RFM_DEV_VERSION		0x01
#define RFM_vc(v)				(v)&0x1F

#define RFM_DEV_STATUS		0x02
#define RFM_ffovfl				_BV(7)
#define RFM_ffunfl				_BV(6)
#define RFM_rxffem				_BV(5)
#define RFM_header				_BV(4)
#define RFM_freqerr				_BV(3)
#define RFM_cps(v)				(v)&0x03

#define RFM_INT_STATUS_1	0x03
#define RFM_ifferr				_BV(7)
#define RFM_itxffafull			_BV(6)
#define RFM_ixtffaem			_BV(5)
#define RFM_irxffafull			_BV(4)
#define RFM_iext				_BV(3)
#define RFM_ipksent				_BV(2)
#define RFM_ipkvalid			_BV(1)
#define RFM_icrerror			_BV(0)

#define RFM_INT_STATUS_2	0x04
#define RFM_iswdet				_BV(7)
#define RFM_ipreaval			_BV(6)
#define RFM_ipreainval			_BV(5)
#define RFM_irssi				_BV(4)
#define RFM_iwut				_BV(3)
#define RFM_ilbd				_BV(2)
#define RFM_ichiprdy			_BV(1)
#define RFM_ipor				_BV(0)

#define RFM_INT_ENABLE_1	0x05
#define RFM_enfferr				_BV(7)
#define RFM_entxffafull			_BV(6)
#define RFM_entxffaem			_BV(5)
#define RFM_enrxffafull			_BV(4)
#define RFM_enext				_BV(3)
#define RFM_enpksent			_BV(2)
#define RFM_enpkvalid			_BV(1)
#define RFM_encrcerror			_BV(0)


#define RFM_INT_ENABLE_2	0x06
#define RFM_enswdet				_BV(7)
#define RFM_enpreaval			_BV(6)
#define RFM_enpreainval			_BV(5)
#define RFM_enrssi				_BV(4)
#define RFM_enwut				_BV(3)
#define RFM_enlbdi				_BV(2)
#define RFM_enchiprdy			_BV(1)
#define RFM_enpor				_BV(0)


#define RFM_CONTROL_1		0x07
#define RFM_swres				_BV(7)
#define RFM_enlbd				_BV(6)
#define RFM_enwt				_BV(5)
#define RFM_x32ksel				_BV(4)
#define RFM_txon				_BV(3)
#define RFM_rxon				_BV(2)
#define RFM_pllon				_BV(1)
#define RFM_xton				_BV(0)

#define RFM_CONTROL_2		0x08
#define RFM_antdiv(v)			(v<<5)&0xE0
#define RFM_rxmpk				_BV(4)
#define RFM_autotx				_BV(3)
#define RFM_enldm				_BV(2)
#define RFM_ffclrrx				_BV(1)
#define RFM_ffclrtx				_BV(0)

#define RFM_CRYSTAL_CAP		0x09
#define RFM_xtalshft			_BV(7)
#define RFM_xlc(v)				(v)&0x7F

#define RFM_CLOCK_OUT		0x0A
#define RFM_clkt(v)				(v<<4)&0x30
#define RFM_enlfc				_BV(3)
#define RFM_mclk(v)				(v)&0x07

#define RFM_GPIO_0			0x0B
#define RFM_gpio0drv(v)			(v<<6)&0xC0
#define RFM_pup0				_BV(5)
#define RFM_gpio0(v)			(v)&0x1F

#define RFM_GPIO_1			0x0C
#define RFM_gpio1drv(v)			(v<<6)&0xC0
#define RFM_pup1				_BV(5)
#define RFM_gpio1(v)			(v)&0x1F

#define RFM_GPIO_2			0x0D
#define RFM_gpio2drv(v)			(v<<6)&0xC0
#define RFM_pup2				_BV(5)
#define RFM_gpio2(v)			(v)&0x1F

#define RFM_p_WUT			0b00001	// Wake-Up Timer: 1 when WUT has expired (output)
#define RFM_p_LBD			0b00010	// Low Battery Detect: 1 when battery is below threshold setting (output)
#define RFM_p_DDI			0b00011	// Direct Digital Input
#define RFM_p_EXT_F			0b00100	// External Interrupt, falling edge (input)
#define RFM_p_EXT_R			0b00101	// External Interrupt, rising edge (input)
#define RFM_p_EXT_C			0b00110	// External Interrupt, state change (input)
#define RFM_p_ADC			0b00111	// ADC Analog Input
#define RFM_p_DDO			0b01010	// Direct Digital Output
#define RFM_p_REF_V			0b01110	// Reference Voltage (output)
#define RFM_p_CLK			0b01111	// TX/RX Data CLK output to be used in conjunction with TX/RX Data pin (output)
#define RFM_p_TX_IN			0b10000	// TX Data input for direct modulation (input)
#define RFM_p_RETX			0b10001	// External Retransmission Request (input)
#define RFM_p_TX_ST			0b10010	// TX State (output)
#define RFM_p_TX_FL			0b10011	// TX FIFO Almost Full (output)
#define RFM_p_RX_OUT		0b10100	// RX Data (output)
#define RFM_p_RX_ST			0b10101	// RX State (output)
#define RFM_p_RX_FL			0b10110	// RX FIFO Almost Full (output)
#define RFM_p_ANT1			0b10111	// Antenna 1 Switch used for antenna diversity (output)
#define RFM_p_ANT2			0b11000	// Antenna 2 Switch used for antenna diversity (output)
#define RFM_p_PMBL			0b11001	// Valid Preamble Detected (output)
#define RFM_p_NVPMBL		0b11010	// Invalid Preamble Detected (output)
#define RFM_p_SYNC			0b11011	// Sync Word Detected (output)
#define RFM_p_CCA			0b11100	// Clear Channel Assessment (output)
#define RFM_p_VDD			0b11101	// VDD, GND for all other values


#define RFM_IO_PORT			0x0E
#define RFM_extitst(v)			(v<<4)&0x70
#define RFM_itsdo				_BV(3)
#define RFM_dio2				_BV(2)
#define RFM_dio1				_BV(1)
#define RFM_dio0				_BV(0)

#define RFM_ADC_CONFIG		0x0F
#define RFM_adcstartdone		_BV(7)
#define RFM_adcsel(v)			(v<<4)&0x70
#define RFM_adcref(v)			(v<<2)&0x0C
#define RFM_adcgain(v)			(v)&0x03

#define RFM_ADC_AMPOFFSET	0x10
#define RFM_adcoffs(v)			(v)&0x0F

#define RFM_ADC_VALUE		0x11
//#define RFM_adc(v)				(v)&0xFF

#define RFM_TEMP_CONFIG		0x12
#define RFM_tsrange(v)			(v<<6)&0xC0
#define RFM_entsoffs			_BV(5)
#define RFM_entstrim			_BV(4)
#define RFM_tstrim(v)			(v)&0x0x0F

#define RFM_TEMP_OFFSET		0x13
#define RFM_tvoffs(v)			(v)&0xFF

#define RFM_WUT_PERIOD_1	0x14
#define RFM_wtr(v)				(v)&0x1F

#define RFM_WUT_PERIOD_2	0x15
#define RFM_wtm_H(v)			(v>>8)&0xFF

#define RFM_WUT_PERIOD_3	0x16
#define RFM_wtm_L(v)			(v)&0xFF

#define RFM_WUT_VALUE_1		0x17
#define RFM_wtv_H(v)			(v>>8)&0xFF

#define RFM_WUT_VALUE_2		0x18
#define RFM_wtv_L(v)			(v)&0xFF

#define RFM_LDC_			0x19
//#define RFM_ldc(v)				(v)&0xFF

#define RFM_BATT_THRESH		0x1A
#define RFM_lbdt(v)				(v)&0x1F

#define RFM_BATT_VALUE		0x1B
#define RFM_vbat(v)				(v)&0x1F

#define RFM_IF_FILTER_BW	0x1C
#define RFM_dwn3_bypass			_BV(7)
#define RFM_ndec(v)				(v<<4)&0x70
#define RFM_filset(v)			(v)&0x0F

#define RFM_AFC_GEARSHIFT	0x1D
#define RFM_afcbd				_BV(7)
#define RFM_enafc				_BV(6)
#define RFM_afcgearh(v)			(v<<3)&0x38
#define RFM_1p5bypass			_BV(2)
#define RFM_matap				_BV(1)
#define RFM_ph0size				_BV(0)

#define RFM_AFC_TIMING		0x1E
#define RFM_swait_timer(v)		(v<<6)&0xC0
#define RFM_shwait(v)			(v<<3)&0x38
#define RFM_anwait(v)			(v)&0x07

#define RFM_CLK_GEARSHIFT	0x1F
#define RFM_crfast(v)			(v<<3)&0x38
#define RFM_crslow(v)			(v)&0x07

#define RFM_CLK_OVERSAMP	0x20
#define RFM_rxosr_L(v)			(v)&0xFF

#define RFM_CLK_OFFSET_2	0x21
#define RFM_rxosr_H(v)			(v>>3)&0xE0
#define RFM_stallctrl			_BV(4)
#define RFM_ncoff_H(v)			(v>>16)&0x0F

#define RFM_CLK_OFFSET_1	0x22
#define RFM_ncoff_M(v)			(v>>8)&0xFF

#define RFM_CLK_OFFSET_0	0x23
#define RFM_ncoff_L(v)			(v)&0xFF

#define RFM_CLK_GAIN_1		0x24
#define RFM_rxncocomp 			_BV(4)
#define RFM_crgain2x 			_BV(3)
#define RFM_crgain_H(v)			(v>>8)&0x07

#define RFM_CLK_GAIN_0		0x25
#define RFM_crgain_L(v)			(v)&0xFF

#define RFM_RSSI_VALUE		0x26
//#define RFM_rssi(v)				(v)&0xFF

#define RFM_RSSI_THRESH		0x27
//#define RFM_rssith(v)			(v)&0xFF

#define RFM_ANT_DIV_1		0x28
//#define RFM_adrssia(v)			(v)&0xFF

#define RFM_ANT_DIV_2		0x29
//#define RFM_adrssib(v)			(v)&0xFF

#define RFM_AFC_LIMIT		0x2A
//#define RFM_afclim(v)			(v)&0xFF

#define RFM_AFC_CORRECT		0x2B
#define RFM_afc_corr_H(v)		(v>>2)&0xFF


#define RFM_OOK_COUNT_1		0x2C
#define RFM_afc_corr_L(v)		(v<<6)&0xC0
#define RFM_ookfrzen			_BV(5)
#define RFM_peakdeten			_BV(4)
#define RFM_madeten				_BV(3)
#define RFM_ookcnt_H(v)			(v>>8)&0x03

#define RFM_OOK_COUNT_2		0x2D
#define RFM_ookcnt_L(v)			(v)&0xFF

#define RFM_OOK_SLICER		0x2E
#define RFM_attack(v)			(v<<4)&0x70
#define RFM_decay(v)			(v)&0x0F

#define RFM_DATA_CONTROL	0x30
#define RFM_enpacrx 			_BV(7)
#define RFM_lsbfrst 			_BV(6)
#define RFM_crcdonly 			_BV(5)
#define RFM_skip2ph 			_BV(4)
#define RFM_enpactx 			_BV(3)
#define RFM_encrc 				_BV(2)
#define RFM_crc(v)				(v)&0x03

#define RFM_EZMAC			0x31
#define RFM_rxcrc1 				_BV(6)
#define RFM_pksrch 				_BV(5)
#define RFM_pkrx 				_BV(4)
#define RFM_pkvalid 			_BV(3)
#define RFM_crcerror 			_BV(2)
#define RFM_pktx 				_BV(1)
#define RFM_pksent				_BV(0)

#define RFM_HEADER_CTRL_1	0x32
#define RFM_bcen(v)				(v<<4)&0xF0
#define RFM_hdch(v)				(v)&0x0F

#define RFM_HEADER_CTRL_2	0x33
#define RFM_skipsyn				_BV(7)
#define RFM_hdlen(v)			(v<<4)&0x70
#define RFM_fixpklen			_BV(3)
#define RFM_synclen(v)			(v<<1)&0x06
#define RFM_prealen_H(v)		(v>>8)&0x01

#define RFM_PRMBL_LEN		0x34
#define RFM_prealen_L(v)		(v)&0xFF

#define RFM_PRMBL_DETECT	0x35
#define RFM_preath(v)			(v<<3)&0xF8
#define RFM_rssi_offset(v)		(v)&0x07

#define RFM_SYNCWORD_3		0x36
#define RFM_SYNCWORD_2		0x37
#define RFM_SYNCWORD_1		0x38
#define RFM_SYNCWORD_0		0x39
#define RFM_HEADER_TX_3		0x3A
#define RFM_HEADER_TX_2		0x3B
#define RFM_HEADER_TX_1		0x3C
#define RFM_HEADER_TX_0		0x3D
#define RFM_PACKET_LEN		0x3E
#define RFM_HEADER_CHK_3	0x3F
#define RFM_HEADER_CHK_2	0x40
#define RFM_HEADER_CHK_1	0x41
#define RFM_HEADER_CHK_0	0x42
#define RFM_HEADER_EN_3		0x43
#define RFM_HEADER_EN_2		0x44
#define RFM_HEADER_EN_1		0x45
#define RFM_HEADER_EN_0		0x46
#define RFM_HEADER_RX_3		0x47
#define RFM_HEADER_RX_2		0x48
#define RFM_HEADER_RX_1		0x49
#define RFM_HEADER_RX_0		0x4A
#define RFM_PACKET_RX_LEN	0x4B
#define RFM_ADC8			0x4F
#define RFM_adc8(v)				(v)&0x7F

#define RFM_CH_FILTER		0x60

#define RFM_XTAL_PWR		0x62
#define RFM_pwst(v)				(v<<5)&0xE0
#define RFM_clkhyst				_BV(4)
#define RFM_enbias2x			_BV(3)
#define RFM_enamp2x				_BV(2)
#define RFM_bufovr				_BV(1)
#define RFM_enbuf				_BV(0)

#define RFM_AGC_OVERRIDE	0x69
#define RFM_sgin				_BV(6)
#define RFM_agcen				_BV(5)
#define RFM_lnagain				_BV(4)
#define RFM_pga(v)				(v)&0x0F

#define RFM_TX_POWER		0x6D
#define RFM_lna_sw				_BV(3)
#define RFM_txpow(v)			(v)&0x07

#define RFM_TX_RATE_1		0x6E
#define RFM_txdr_H(v)			(v>>8)&0xFF

#define RFM_TX_RATE_0		0x6F
#define RFM_txdr_L(v)			(v)&0xFF


#define RFM_MOD_MODE_1		0x70
#define RFM_txdtrtscale			_BV(5)
#define RFM_enphpwdn			_BV(4)
#define RFM_manppol				_BV(3)
#define RFM_enmaninv			_BV(2)
#define RFM_enmanch				_BV(1)
#define RFM_enwhite				_BV(0)

#define RFM_MOD_MODE_0		0x71
#define RFM_trclk(v)			(v<<6)&0xC0
#define RFM_dtmod(v)			(v<<4)&0x30
#define RFM_eninv				_BV(3)
#define RFM_fd_H(v)				(v<<2)&0x04
#define RFM_modtyp(v)			(v)&0x03

#define RFM_FREQ_DEVI		0x72
#define RFM_fd_L(v)				(v)&0xFF

#define RFM_FREQ_OFFSET_1	0x73
#define RFM_fo_L(v)				(v)&0xFF

#define RFM_FREQ_OFFSET_2	0x74
#define RFM_fo_H(v)				(v>>8)0x03

#define RFM_FREQ_BAND		0x75
#define RFM_sbsel				_BV(6)
#define RFM_hbsel				_BV(5)
#define RFM_fb(v)				(v)&0x1F

#define RFM_FREQ_NOM_1		0x76
#define RFM_fc_H(v)				(v>>8)&0xFF

#define RFM_FREQ_NOM_0		0x77
#define RFM_fc_L(v)				(v)&0xFF

#define RFM_FHSS_CH			0x79
#define RFM_FHSS_STEP		0x7A

#define RFM_TX_FIFO_1		0x7C
#define RFM_txafthr(v)			(v)&0x3F

#define RFM_TX_FIFO_2		0x7D
#define RFM_txaethr(v)			(v)&0x3F


#define RFM_RX_FIFO			0x7E
#define RFM_rxafthr(v)			(v)&0x3F

#define RFM_FIFO			0x7F



#define MAX_TX_POWER	7

#define RFMCFG_FHSS		1
#define RFMCFG_4800		2
#define RFMCFG_AFSK		3

const uint8_t rfmConfig_Core[][2] = { // Start the device inert but ready, interrupts set
	// Set critical components in priority order
	{0x09,	0x7F},			// Xtal Cap 		Default @ 12.5pF
	#if defined(RFM23BP)
	{RFM_GPIO_0,	RFM_p_DDO},		// GPIO0 			/Rx (Low-Asserted)
	{RFM_GPIO_1,	RFM_p_DDO},		// GPIO1 			/Tx (Low-Asserted)
	{0x0E,(RFM_dio1 | RFM_dio0)},	// I/O Ports		Both high for idle, may also be High-Z
	#else
	{RFM_GPIO_0,	RFM_p_RX_ST},	// GPIO0 			Rx
	{RFM_GPIO_1,	RFM_p_TX_ST},	// GPIO1 			Tx
	{0x0E,	0x00},					// I/O Ports		None
	#endif
	{RFM_GPIO_2,	RFM_p_WUT},	// GPIO2 		RFM_p_PMBL RFM_p_RX_OUT RFM_p_WUT	Sync=11011 RxData=10100 Preamble=11001 RxFifoFull=10110 RxState=10101 WUT=00001
	{0x05,	0x00},			// Int Enable 1		
	{0x06,	0x00},			// Int Enable 2		
	{0x07,	0x00},			// Control 1		Standby
	{0x08,	0x00},			// Control 2		
	
	// Disable OOK Module
	{0x2C,0x00},			// OOK Control		Disabled
	{0x2D,0x00},			// OOK Counter		Disabled
	{0x2E,0x00},			// OOK Slicer		Disabled
	
	// Disable various auxiliary devices that are not used
	{0x0A,0x07},			// uC Output Clk	32 kHz
	{0x0F,0x00},			// ADC Config 		Disabled
	{0x10,0x00},			// ADC Offset		None
	{0x12,0x20},			// Temp Calc		Celsius
	{0x13,0x00},			// Temp Offset		None
	{0x4F,0x10},			// ADC8 Temp Set	<>
	{0x1A,0x14},			// Low Batt			2.7v (1.7+20*50mV)
	
	// Configure LDC Mechanism for 16 ms every 2 sec
	{0x14,7},			// Wake Set 1		R=7
	{0x15,0},			// Wake Set 2		T_WUT=4*M*2^R / 2^15
	{0x16,128},			// Wake Set 3		M=128
	{0x19,1},			// Low Cyc Dur		LDC/M Wake Ratio
	
	// 430 MHz Band, +5kHz Calibration, FHSS Off
	{0x73,0},			// Freq Offset 1	+5 kHz Calibration					
	{0x74,0},			// Freq Offset 2						
	{0x75,0x53},		// Freq Band		430 MHz Band	
	{0x79,0x00},		// FHSS Channel							
	{0x7A,0x00},		// FHSS Step Size							

};
const uint8_t rfmConfig_ModeConfigs[][4] = {
//	 Addr,FHSS,4800,3500/AFSK								FHSS		4800		3500/AFSK
	{0x1C,0x27,0x2B,0x2B},		// IF B/W					
	{0x1F,0x03,0x03,0x03},      // Clk Gear					
	{0x20,0x68,0xD0,0x1E},      // Clk Ratio				
	{0x21,0x01,0x00,0x20},      // Clk Offset 2				
	{0x22,0x3A,0x9D,0x72},      // Clk Offset 1				
	{0x23,0x93,0x49,0xB0},      // Clk Offset 0				
	{0x24,0x04,0x00,0x00},      // Clk Gain 1				
	{0x25,0x62,0x99,0xA2},      // Clk Gain 0				
	{0x1D,0x04,0x44,0x44},      // AFC Gear					
	{0x1E,0x0A,0x0A,0x0A},      // AFC Timing				
	{0x2A,0x1D,0x1D,0x30},      // AFC Limit				+/-18kHz	+/-18kHz	+/-30kHz
	{0x69,0x60,0x60,0x60},		// AGC Override				
	
	{0x6D,0x08,0x08,0x08},		// Tx Power					All: Direct-Tie Config, Lowest Power
	{0x6E,0x4E,0x27,0x1C},      // Tx Data Rate 1			9600bps		4800bps		"3500bps"
	{0x6F,0xA5,0x52,0xAC},      // Tx Data Rate 0			
	{0x70,0x20,0x20,0x20},      // Modulation 1				All: PH Disabled in Sleep, sub-30kbps
	{0x71,0x23,0x23,0x12},      // Modulation 2				GFSK FIFO	GFSK FIFO	FSK Direct
	{0x72,0x09,0x14,0x08},      // Freq Deviation			+/-5.6kHz	12.5kHz		+/-5kHz
	{0x76,0x0C,0x64,0x64},      // Freq Carrier 1			430.500		434.000		434.000
	{0x77,0x80,0x00,0x00},      // Freq Carrier 0			
	
	// {0x08,0x00,0x00,0x00},	// Low-Duty-Cycle			LDC Off		LDC Off		LDC On for 3500
	{0x30,0xC0,0x80,0x00},		// Packet Handler			PH, LSB		PH			None
};
const uint8_t rfmConfig_FhssConfig[][2] = {
	// Freq Range between 430.5 and 434.950 MHz, Center is 432.725
	// 9600 Baud
	// 5.4 kHz Deviation
	// Gaussian Shift
	// fcarrier = (fb+24+(fc+fo)/64000) * 10000 * (hbsel+1) + (fhch * fhs * 10) [kHz]
	// fb=19 (10 MHz Increments), fo=32 (+5 kHz Up Tick), fc=3200 (500 kHz)
	// fo can only shift by +/-80 kHz, 2's comp
	// Use fc as channel select mechanism, unsigned for 10 MHz span
	
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
	{0x30,0x88},		// Data Ctrl		Packet Handler LSB First
	{0x32,0x00},		// Header Ctrl 1	Rx'd Hdr is Byte 3,2
	{0x33,0x0A},		// Header Ctrl 2	Hdr Defined by 3E ; Sync 3,2,0	
	{0x34,0x08},		// Preamble Length	3 nibbles
	{0x35,0x28},		// Preamble Detect	3 nibbles
	// {0x60,0x00},		// Preamble	Thresh						
	{0x36,0b11010010},	// Sync Word 3		90, D0
	{0x37,0b11000110},	// Sync Word 2		42, 50					
	{0x38,0x00},		// Sync Word 1		Always E1					
	{0x39,0x00},		// Sync Word 0							
	{0x3A,0x00},		// Tx Header 3							
	{0x3B,0x00},		// Tx Header 2							
	{0x3C,0x00},		// Tx Header 1							
	{0x3D,0x00},		// Tx Header 0							
	{0x3E,0x14},		// Tx Pkt Length	13 Bytes per packet
	{0x3F,0x00},		// Check Header 3						
	{0x40,0x00},		// Check Header 2						
	{0x41,0x00},		// Check Header 1						
	{0x42,0x00},		// Check Header 0						
	{0x43,0x00},		// Header Enable 3	All bits compared	
	{0x44,0x00},		// Header Enable 2	All bits compared	
	{0x45,0x00},		// Header Enable 1	All bits compared	
	{0x46,0x00},		// Header Enable 0	All bits compared	
	{0x7C,4},		// T// Tx FIFO Ctrl 1	Tx Almost Full @ 60	
	{0x7D,1},		// T// Tx FIFO Ctrl 2	Tx Almost Empt @ 4	
	{0x7E,0x20},		// Rx FIFO Ctrl		Rx Almost Full @ 32	
	
};
/*
// const uint8_t rfmConfig_ToneConfig[][2] = {
	{0x30,0x00},		// Packet Handler off, 
	// {0x30,0x80},		// Packet Handler on, 
	// {0x32,0x01},		// 1 Headers
	// {0x33,0x18},		// 1 Headers, 1 Sync
	{0x34,0x20},		// 64 nibble = 32 byte preamble
	{0x35,0x20},		// 0x35 need to detect 20bit preamble
	// {0x60,0x00},		// Preamble	Thresh
	{0x36,0x55},
	// {0x3E,0x0F},
	// {0x3F,0x55},
	// {0x40,0x55},
	// {0x43,0xFF},
	// {0x44,0xFF},
	// {0x7E,0x20},
// }
*/

uint8_t rfmReset(void);
uint8_t rfmMode(uint8_t);
// uint8_t rfmTxPacket(void);
// void rfmTxDirect(uint8_t);
void rfmSetRxTxSw(uint8_t);
uint8_t rfmGetRxTx(uint8_t);
void rfmSetInterrupts(uint8_t, uint8_t);
uint16_t rfmGetInterrupts(void);
void rfmSetLrsChannel(uint8_t);
uint8_t rfmSetManualFreq(uint16_t);
void rfmSetTxPower(uint8_t);
uint8_t rfmGetRSSI(void);
uint8_t rfmReadReg(uint8_t);
uint8_t rfmWriteReg(uint8_t, uint8_t);
void rfmReadFIFO(uint8_t *array);
uint8_t rfmWriteFIFOStr(char *array);
uint8_t rfmGetTxFIFOEmpty(void);
void rfmClearTxFIFO(void);
void rfmClearRxFIFO(void);



#endif // RFM22B_H








#if defined(Notes_for_here)


	
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
	
	/*
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
	*/

	/*
	// Modulation
	{0x6D,0x08},		// Tx Power			LNA Sw Active, -1dBm
	{0x6E,0x4E},		// Tx Data Rate 1	<>					
	{0x6F,0xA5},		// Tx Data Rate 0	9600 Baud				
	{0x70,0x30},		// Modulation 1		Normal, no PH in LDC				
	{0x71,0x23},		// Modulation 2		FIFO GFSK		
	{0x72,0x09},		// Freq Deviation	<>					
	{0x76,0x0C},		// Freq Carrier 1	<>					
	{0x77,0x80},		// Freq Carrier 0	<>					
	*/



// const uint8_t rfmConfig_AccessTone[][2] = {
	/*
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
	*/
	
	// Packet
	//{0x30,0x00},		// Packet Handler off, 
	/*
	{0x30,0x80},		// Packet Handler on, 
	{0x32,0x01},		// 1 Headers
	{0x33,0x18},		// 1 Headers, 1 Sync
	{0x34,0x20},		// 64 nibble = 32 byte preamble
	{0x35,0x20},		// 0x35 need to detect 20bit preamble
	{0x60,0x00},		// Preamble	Thresh
	{0x36,0x55},
	// {0x37,0x55},
	// {0x38,0x55},
	// {0x39,0x55},
	{0x3E,0x0F},
	{0x3F,0x55},
	// {0x40,0x55},
	{0x43,0xFF},
	// {0x44,0xFF},
	{0x7E,0x20},
	
	*/
	/*
	// Modulation
	{0x6D,0x08},		// Tx Power			LNA Sw Active, -1dBm
	{0x6E,0x1C},		// Tx Data Rate 1	<>					
	{0x6F,0xAC},		// Tx Data Rate 0	3500 bit/s				
	{0x70,0x30},		// Modulation 1		Normal, PH off in LDC		
	{0x71,0x23},		// Modulation 2		GFSK		
	{0x72,0x08},		// Freq Deviation	<>								
	{0x76,0x64},		// Freq Carrier 1	434 MHz					
	{0x77,0x00},		// Freq Carrier 0	<>					
	*/
	// Preamble is 1750 -- 3500 bps, .032 sec -> 56 bits, can capture 16 bits in that time
	// .016 TLDC is .015625 * 32768 /4 
// };









// Pesudo-code

/*
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

*/
#endif


