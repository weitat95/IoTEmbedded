


#include <stdint.h>


int write(char * data, unsigned long data_size);
void initEmitter(void);
void to_manchester(unsigned char data, unsigned long int * data_manchester);
void manToAnalogue(unsigned long int *data_manchester, unsigned long *buffer, unsigned long logicHigh, unsigned long logicLow, unsigned short oversampling);
