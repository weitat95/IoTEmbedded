


#include <stdint.h>


int write(char * data, unsigned long data_size);
void initEmitter(void);
void to_manchester(unsigned char data, unsigned long int * data_manchester);