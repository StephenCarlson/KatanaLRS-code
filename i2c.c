#ifndef I2C_H
#define I2C_H

//#devine DEBUG_I2C

#define ACK				1
#define NACK			0

uint8_t startI2C(uint8_t, uint8_t);
void stopI2C(void);
uint8_t writeI2C(uint8_t);
uint8_t readI2C(uint8_t);


uint8_t startI2C(uint8_t address, uint8_t intent){ // i.e. ITG3200ADDR, WRITE
	uint16_t time = TCNT1;
	//while( !(TWCR &(1<<TWINT)));			// Avoid Crashing
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTA);	// Send Start
	while( !(TWCR &(1<<TWINT))){
		if(TCNT1== (time+50)) break;
	} //printf("ST1: %X\n",TWSR);			// Wait
#if defined(DEBUG_I2C)
	if(TW_STATUS != TW_START) printf("BadStart\n");
#endif
	TWDR = ((address<<1) | (intent & 0x01));			// Hail Slave Device
	TWCR = (1<<TWINT)|(1<<TWEN);			// Engage
	while( !(TWCR &(1<<TWINT))){
		if(TCNT1== (time+50)) break;
	} //; //printf("ST2: %X\n",TWSR);			// Wait
#if defined(DEBUG_I2C)
	if(TW_STATUS == TW_NO_INFO || TW_STATUS == TW_BUS_ERROR) printf("BadHold\n");
#endif
	return TW_STATUS;
}

void stopI2C(void){
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);	// Send Stop
	while(TWCR &(1<<TWSTO)); //printf("SP: %X\n",TWSR);
}

uint8_t writeI2C(uint8_t data){
	uint16_t time = TCNT1;
	TWDR = data;							// Data
	TWCR = (1<<TWINT)|(1<<TWEN);			// Enable
	while( !(TWCR &(1<<TWINT))){
		if(TCNT1== (time+50)) break;
	} //; //printf("WT: %X\n",TWSR);			// Wait
#if defined(DEBUG_I2C)
	if(TW_STATUS != TW_MT_DATA_ACK) printf("BadFrame\n");
#endif
	return TW_STATUS;
}

uint8_t readI2C(uint8_t ackType){
	uint16_t time = TCNT1;
	//printf("TWCR: %X", (1<<TWINT)|(1<<TWEN)|(ackType<<TWEA));
	TWCR = (1<<TWINT)|(1<<TWEN)|(ackType<<TWEA);
	while( !(TWCR &(1<<TWINT))){
		if(TCNT1== (time+50)) break;
	} //; //printf("RV: %X\n",TWSR);
	return TWDR;
}


#endif
