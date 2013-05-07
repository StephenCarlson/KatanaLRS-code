#ifndef SPI_H
#define SPI_H


uint8_t transferSPI(uint8_t);

uint8_t transferSPI(uint8_t data){
	SPDR = data;
	while(!(SPSR & _BV(SPIF)));
	return SPDR;
}


#endif