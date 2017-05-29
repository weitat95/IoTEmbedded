//*********************** hc05.c  ***********************
// Program written by:
// - Corentin Dugue  c.dugue@outlook.com
// Modified by:
// - Wei Tat Lee weitat95@live.com
// Brief desicription of program:
// - Initializes an HC-05 module to act as a Bluetooth client
//
//*********************************************************
/*
 Hardware connections
 /---------------------------\
 |              chip      1  |
 | Ant                    2  |
 | enna       processor   3  |
 |                        4  | 
 |                        5  |
 |                        6  |
 \---------------------------/ 
HC05        TM4C123 
  1 EN                  Enable pin
  2 VCC     +3.3V       +3.3V 
  3 GND     GND         Ground
  4 UTxD    PC6(U3Rx)   UART Out of HC05,  38,400 baud
  5 URxD    PC7(U3Tx)   UART In of HC05,  38,400 baud
  6 State               State if connected
 */


#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "../inc/tm4c123gh6pm.h"
#include "hc05.h"
#include "FIFO.h"
#include "uart.h"
#define BUFFER_SIZE 1024
#define MAXTRY 10
// prototypes for functions defined in startup.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode


#define UART_FR_TXFF            0x00000020  // UART Transmit FIFO Full
#define UART_FR_RXFE            0x00000010  // UART Receive FIFO Empty
#define UART_LCRH_WLEN_8        0x00000060  // 8 bit word length
#define UART_LCRH_FEN           0x00000010  // UART Enable FIFOs
#define UART_CTL_UARTEN         0x00000001  // UART Enable

#define FIFOSUCESS 1
#define FIFOFAIL 0
// e.g.,
// AddIndexFifo(Tx,32,unsigned char, 1,0)
// SIZE must be a power of two
// creates TxFifo_Init() TxFifo_Get() and TxFifo_Put()
AddIndexFifo(RxHC, 32 ,unsigned char,FIFOSUCESS,FIFOFAIL)

//------------------- HC05InitUART-------------------
// - intializes uart needed to communicate with HC05
// Configure UART3 for 38,400 baud rate operation
// Inputs: none
// Outputs: none
void HC05_InitUART(void){ 
  volatile int delay;
  SYSCTL_RCGCUART_R |= 0x08; // Enable UART3
  while((SYSCTL_PRUART_R&0x08)==0){};
  SYSCTL_RCGCGPIO_R |= 0x04; // Enable PORT C clock gating
  while((SYSCTL_PRGPIO_R&0x04)==0){}; 
  GPIO_PORTC_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port C
  GPIO_PORTC_CR_R = 0xFF;           // allow changes to Port C
  GPIO_PORTC_AFSEL_R |= 0xC0; // PC7,PC6 UART3
  GPIO_PORTC_PCTL_R =(GPIO_PORTC_PCTL_R&0x00FFFFFF)|0x11000000;
  GPIO_PORTC_DEN_R   |= 0xC0; //PC6,PC7 UART3 
  UART3_CTL_R &= ~UART_CTL_UARTEN;                  // Clear UART3 enable bit during config
  UART3_IBRD_R = 130;                    // IBRD = int(80,000,000 / (16 * 38,400)) = int(130.2083)
  UART3_FBRD_R = 13;                     // FBRD = int(0.2083 * 64 + 0.5) = 13
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)    
  UART3_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);  // 8 bit word length, 1 stop, no parity, FIFOs enabled
  UART3_IFLS_R &= ~0x3F;                            // Clear TX and RX interrupt FIFO level fields
  UART3_IFLS_R += UART_IFLS_RX1_8 ;                 // RX FIFO interrupt threshold >= 1/8th full
  UART3_IM_R  |= UART_IM_RXIM | UART_IM_RTIM;       // Enable interupt on RX and RX transmission end
  UART3_CTL_R |= UART_CTL_UARTEN;                   // Set UART3 enable bit    
}
void HC05FIFOtoBuffer(void);
// -----------UART3_Handler-----------
// called on one receiver data input followed by timeout
// or     on going from 1 to 2 data input characters
void UART3_Handler(void){
  if(UART3_RIS_R & UART_RIS_RXRIS){   // rx fifo >= 1/8 full
    UART3_ICR_R = UART_ICR_RXIC;      // acknowledge interrupt
    HC05FIFOtoBuffer();
  }
  if(UART3_RIS_R & UART_RIS_RTRIS){   // receiver timed out
    UART3_ICR_R = UART_ICR_RTIC;      // acknowledge receiver time
    HC05FIFOtoBuffer();
  }
}
//--------HC06_EnableRXInterrupt--------
// - enables uart3 rx interrupt
// Inputs: none
// Outputs: none
void HC05_EnableRXInterrupt(void){
  NVIC_EN1_R = 1<<27; // interrupt 59 33-1
}

//--------HC06_DisableRXInterrupt--------
// - disables uart6 rx interrupt
// Inputs: none
// Outputs: none
void HC05_DisableRXInterrupt(void){
  NVIC_DIS1_R = 1<<27; // interrupt 59    
}

//--------HC06_PrintChar--------
// prints a character to the HC06 via uart
// Inputs: character to transmit
// Outputs: none
// busy-wait synchronization
void HC05_PrintChar(char input){
  while((UART3_FR_R&UART_FR_TXFF) != 0) {};
  UART3_DR_R = input;
  UART_OutChar(input); // echo debugging
}

//----------HC05FIFOtoBuffer----------
// - copies uart fifo to RXBuffer, using a circular MACQ to store the last data
// - NOTE: would probably be better to use a software defined FIFO here
// - LastReturnIndex is index to previous \n
// Inputs: none
// Outputs:none
void HC05FIFOtoBuffer(void){
  char letter;
  while((UART3_FR_R&UART_FR_RXFE) == 0){
    letter = UART3_DR_R;        // retrieve char from FIFO
    //if(ESP8266_EchoResponse){
      UART_OutCharNonBlock(letter); // echo
      if(letter=='\r'){
        UART_OutCharNonBlock('\n');
      }
   // }
      
    RxHCFifo_Put(letter);  
    /*
    if(RxHCBufferIndex >= BUFFER_SIZE){
      RXBufferIndex = 0; // store last BUFFER_SIZE received
    }
    RXBuffer[RXBufferIndex] = letter; // put char into buffer
    RXBufferIndex++; // increment buffer index 
    SearchCheck(letter);               // check for end of command
    ServerResponseSearchCheck(letter); // check for server response
    if(letter == '\n' || letter == '\r'){
      LastReturnIndex = CurrentReturnIndex;
      CurrentReturnIndex = RXBufferIndex;
      RXBufferIndex=0;
    }
  */      
  }
}

char HC05_InChar(void){
  char letter;
  while(RxHCFifo_Get(&letter) == FIFOFAIL){};
  return(letter);
}


//---------HC05SendCommand-----
// - sends a string to the HC05 module
// uses busy-wait
// however, hardware has 16 character hardware FIFO
// Inputs: string to send (null-terminated)
// Outputs: none
void HC05SendCommand(const char* inputString){
  int index = 0;
  while(inputString[index] != 0){
    HC05_PrintChar(inputString[index++]);
  }
}




/*
=======================================================================
==========          HC05 PUBLIC FUNCTIONS                 ==========
=======================================================================
*/
/*
void HC05_Init(void){
  HC05_InitUART();
  HC05_EnableRXInterrupt();
  SearchLooking = false;
  SearchFound = false;
  ServerResponseSearchLooking = 0; // not looking for "+IPD"
  ServerResponseSearchFinished = 0;
  EnableInterrupts();
  printf("HC05 Initialization:\n\r");
  if(HC05_CheckVersion()==0){
    printf("Read Firmware Version Failed.\n\r");
  }
}
//---------HC05_Default----------
// Restore to default: slave mode, pin code: 1234, device name: H-C-2010-06-01, baud: 38400 b/s
// Input: none
// output: 1 if success, 0 if fail
int HC05_Default(void){
  int try=MAXTRY;
  SearchStart("ok");
  while(try){
    HC05SendCommand("AT+ORGL\r\n");
    DelayMsSearching(5000);
    if(SearchFound) return 1; // success
    try--;
  }
  return 0; // fail
}

//---------HC05_SetMaster----------
// Set the HC-05 to act as a master
// Input: none
// output: 1 if success, 0 if fail
int HC05_SetMaster(void){
  int try=MAXTRY;
  SearchStart("ok");
  while(try){
    HC05SendCommand("AT+ROLE=1\r\n");
    DelayMsSearching(5000);
    if(SearchFound) return 1; // success
    try--;
  }
  return 0; // fail
}

//---------HC05_CheckVersion----------
// Set the HC-05 to act as a master
// Input: none
// output: 1 if success, 0 if fail
int HC05_CheckVersion(void){
  int try=MAXTRY;
  SearchStart("ok");
  while(try){
    HC05SendCommand("AT+VERSION?\r\n");
    DelayMsSearching(5000);
    if(SearchFound) return 1; // success
    try--;
  }
  return 0; // fail
}
*/