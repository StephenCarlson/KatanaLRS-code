#ifndef RFM23BP_H
#define RFM23BP_H

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


#endif