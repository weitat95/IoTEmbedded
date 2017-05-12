#include <stdint.h>

void DelayMsSearching(uint32_t n);
void DelayMs(uint32_t n);
int HC05_Default(void);
void HC05_DisableRXInterrupt(void);
void HC05_EnableRXInterrupt(void);
void HC05_InitUART(void);
void HC05_Init(void);
void HC05_PrintChar(char input);
int HC05_SetMaster(void);
void HC05SendCommand(const char* inputString);
void SearchCheck(char letter);
void SearchStart(char *pt);
void ServerResponseSearchCheck(char letter);
void ServerResponseSearchStart(void);
void UART3_Handler(void);