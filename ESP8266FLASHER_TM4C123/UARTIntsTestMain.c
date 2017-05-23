// UARTIntsTestMain.c
// Runs on LM4F120/TM4C123
// Tests the UART0 to implement bidirectional data transfer to and from a
// computer running HyperTerminal.  This time, interrupts and FIFOs
// are used.
// Daniel Valvano
// September 12, 2013

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
   Program 5.11 Section 5.6, Program 3.10

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// U0Rx (VCP receive) connected to PA0
// U0Tx (VCP transmit) connected to PA1


/*
Used to flash the esp8266 using XCOM-Util inside flasher folder.
VCC to 3.3V
GND to ground
CH_PD to 3.3V
TXD to PB0
RXD to PB1
GPIO0 to ground (for the duration of firmware upgrading. After all the upgrades have been loaded, it needs to be disconnected)
https://developer.mbed.org/users/sschocke/code/WiFiLamp/wiki/Updating-ESP8266-Firmware
*/
#include <stdint.h>
#include "PLL.h"
#include "UART.h"
#include "../inc/tm4c123gh6pm.h"

//---------------------OutCRLF---------------------
// Output a CR,LF to UART to go to a new line
// Input: none
// Output: none
void OutCRLF(void){
  UART_OutChar(CR);
  UART_OutChar(LF);
}

void Interpreter(void){
  char string[20];  // global to assist in debugging
  
  
  
}
void ESP8266_InitUART(void){ 
  SYSCTL_RCGCUART_R |= 0x02; // Enable UART1
  while((SYSCTL_PRUART_R&0x02)==0){};
  SYSCTL_RCGCGPIO_R |= 0x02; // Enable PORT B clock gating
  while((SYSCTL_PRGPIO_R&0x02)==0){}; 
  GPIO_PORTB_AFSEL_R |= 0x03;
  GPIO_PORTB_DIR_R |= 0x20; // PB5 output to reset
  GPIO_PORTB_PCTL_R =(GPIO_PORTB_PCTL_R&0xFF0FFF00)|0x00000011;
  GPIO_PORTB_DEN_R   |= 0x23; //23 
  GPIO_PORTB_DATA_R |= 0x20; // reset high
  UART1_CTL_R &= ~UART_CTL_UARTEN;                  // Clear UART1 enable bit during config
  //UART1_IBRD_R = 5000000/baud;   
  //UART1_FBRD_R = ((64*(5000000%baud))+baud/2)/baud;      
  UART1_IBRD_R = 43;       // IBRD = int(80,000,000 / (16 * 115,200)) = int(43.403)
  UART1_FBRD_R = 26;       // FBRD = round(0.4028 * 64 ) = 26

  UART1_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);  // 8 bit word length, 1 stop, no parity, FIFOs enabled
  UART1_IFLS_R &= ~0x3F;                            // Clear TX and RX interrupt FIFO level fields
  UART1_IFLS_R += UART_IFLS_RX1_8 ;                 // RX FIFO interrupt threshold >= 1/8th full
  UART1_IM_R  |= UART_IM_RXIM | UART_IM_RTIM;       // Enable interupt on RX and RX transmission end
  UART1_CTL_R |= UART_CTL_UARTEN;                   // Set UART1 enable bit    
  NVIC_EN0_R = 1<<6; // interrupt 6

}
void ESP8266FIFOtoBuffer(void){
  char letter;
  while((UART1_FR_R&UART_FR_RXFE) == 0){
    letter = UART1_DR_R;        // retrieve char from FIFO
    UART_OutChar(letter); 
  }
}
// -----------UART1_Handler-----------
// called on one receiver data input followed by timeout
// or     on going from 1 to 2 data input characters
void UART1_Handler(void){
  if(UART1_RIS_R & UART_RIS_RXRIS){   // rx fifo >= 1/8 full
    UART1_ICR_R = UART_ICR_RXIC;      // acknowledge interrupt
    ESP8266FIFOtoBuffer();
  }
  if(UART1_RIS_R & UART_RIS_RTRIS){   // receiver timed out
    UART1_ICR_R = UART_ICR_RTIC;      // acknowledge receiver time
    ESP8266FIFOtoBuffer();
  }
}
void ESP8266_PrintChar(char input){
  while((UART1_FR_R&UART_FR_TXFF) != 0) {};
  UART1_DR_R = input;
//  UART_OutChar(input); // echo debugging
}
//debug code
int main(void){
  char i;
  char string[20];  // global to assist in debugging
  uint32_t n;

  PLL_Init(Bus80MHz);       // set system clock to 50 MHz
  UART_Init();              // initialize UART

  ESP8266_InitUART();
  while(1){
    char out;
    out=UART_InChar();
   // UART_OutChar(out);
    ESP8266_PrintChar(out);
  }
}
