//***********************  main.c  ***********************
// Program written by:
// - Steven Prickett  steven.prickett@gmail.com
//
// Brief desicription of program:
// - Initializes an ESP8266 module to act as a WiFi client
//   and fetch weather data from openweathermap.org
//
//*********************************************************
/* Modified by Jonathan Valvano
 Sept 14, 2016
 Out of the box: to make this work you must
 Step 1) Set parameters of your AP in lines 59-60 of esp8266.c
 Step 2) Change line 39 with directions in lines 40-42
 Step 3) Run a terminal emulator like Putty or TExasDisplay at
         115200 bits/sec, 8 bit, 1 stop, no flow control
 Step 4) Set line 50 to match baud rate of your ESP8266 (9600 or 115200)
 */
 
 /*
 Modified by Wei Tat Lee
 May 11, 2017
 Used example by Prof. Valvano for esp8266 and modified to send AT commands to HC05 bluetooth module
 Clock Rate 80MHz
 The HC05 module works as a slave at default. 
 Use main3 to configure the HC05 module such as changing it to master module.
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
  2 VCC     +5V       +3.6V-6V 
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
#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))

#include "../inc/tm4c123gh6pm.h"

#include "pll.h"
#include "UART.h"
#include "hc05.h"
#include "LED.h"
#include "FIFO.h"
// prototypes for functions defined in startup.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

/*
 * Used to configure HC05 bluetooth module using AT+ commands
 */
extern volatile bool SearchFound;
char data[40];

int main3(void){

  DisableInterrupts();
  PLL_Init(Bus80MHz);
  LED_Init();  
  Output_Init();       // UART0 only used for debugging
  printf("\n\r-----------\n\rSystem starting...\n\r");
  HC05_Init();
  
  while(1){
    UART_InString(data,40);
    int try=1;
    SearchStart("ok");
    while(try){
      HC05SendCommand(data);
      HC05SendCommand("\r\n");
      DelayMsSearching(5000);
      if(SearchFound) break; // success
      try--;
    }
    for(int i=0;i<40;i++){
      data[i]=0;
    }    
  }
  
}

/*
 * The Bluetooth module is set as slave module. Used an android app to set  the LEDs.
 */
char character;
extern char RXBuffer[1024];
int main(void){
  DisableInterrupts();
  PLL_Init(Bus80MHz);
  LED_Init();  
  Output_Init();       // UART0 only used for debugging
  printf("\n\r-----------\n\rSystem starting...\n\r");
  HC05_InitUART();
  HC05_EnableRXInterrupt();
  EnableInterrupts();
  printf("HC05 Initialization:\n\r");
  
  while(1){
    if(RXBuffer[0]=='H'){
      PF3^=0x08;
      RXBuffer[0]='h';
    }
    if(RXBuffer[0]=='B'){
      PF2^=0x04;
      RXBuffer[0]='b';
    }
  }
}