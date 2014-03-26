#ifndef SYS_UTILITY_H
#define SYS_UTILITY_H



void updateVolts(uint8_t);

char deviceIdCheck(void);
void printHelpInfo(void);

static int putUARTchar(char c, FILE *stream);
uint8_t getUARTchar(void);
uint16_t readADC(uint8_t);
uint16_t readAdcNoiseReduced(uint8_t);
void flashOrangeLED(uint8_t, uint8_t, uint8_t);
void flashBlueLED(uint8_t, uint8_t, uint8_t);




#endif // SYS_UTILITY_H