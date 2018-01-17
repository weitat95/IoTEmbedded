/*
*  gpio.c
*
*  This file represents the initialization funtions used to handle the GPIO (General
*  Purpose Input and Output) on the Tiva C Series TM4C123 hardware.
*  This project started in the class EE445M - EMBEDDED AND REAL-TIME SYSTEMS - Spring 2017
*  - University of Texas at Austin.
*  ----------------------------------------------------------------------------
*  SOFTWARE DISCLAIMER
*
*  THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
*  OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
*  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
*  THE AUTHOR(S) SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,
*  INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
*  ----------------------------------------------------------------------------
*  SOFTWARE UPDATES:
*
*      NAME            DATE/TIME              COMMENTS
*  PEGASUS TEAM        April, 10 2017         Initial Version
*
*  NOTES:
*  - Professor Spring 2017 class: Prof. Jonathan W. Valvano
*  - TA Spring 2017             : Kishore Punniyamurthy
*
*/

#include <stdint.h>
#include "gpio.h"
#include "inc/tm4c123gh6pm.h"

void GPIO_PortE_Init(void)
{ 
  volatile uint32_t delay;
  
  SYSCTL_RCGC2_R      |= 0x10;        // activate port E
  delay               = SYSCTL_RCGC2_R;        
  delay               = SYSCTL_RCGC2_R;   
  delay               = SYSCTL_RCGC2_R;        
  delay               = SYSCTL_RCGC2_R;   
  GPIO_PORTE_DIR_R    |= 0x0F;        // make PE3-0 output heartbeats
  GPIO_PORTE_AFSEL_R  &= ~0x0F;       // disable alt funct on PE3-0
  GPIO_PORTE_DEN_R    |= 0x0F;        // enable digital I/O on PE3-0
  GPIO_PORTE_PCTL_R   = ~0x0000FFFF;
  GPIO_PORTE_AMSEL_R  &= ~0x0F;;      // disable analog functionality on PF
}

void GPIO_PortF_Init(void)
{
  SYSCTL_RCGCGPIO_R |= 0x20;            // activate port F
  while((SYSCTL_PRGPIO_R&0x20) == 0){}; // ready?
  GPIO_PORTF_LOCK_R = 0x4C4F434B;       // unlock GPIO Port F
  GPIO_PORTF_CR_R = 0xFF;               // allow changes to PF4-0
  GPIO_PORTF_DIR_R = 0x0E;              // make PF3-1 output (PF3-1 built-in LEDs)
  GPIO_PORTF_AFSEL_R = 0;               // disable alt funct
  GPIO_PORTF_DEN_R = 0x1F;              // enable digital I/O on PF4-0
  GPIO_PORTF_PUR_R = 0x11;              // enable pullup on inputs
  GPIO_PORTF_PCTL_R = 0x00000000;
  GPIO_PORTF_AMSEL_R = 0;               // disable analog functionality on PF
}

void GPIO_PortB_Init(uint32_t dir)
{ 
  volatile uint32_t delay;
  
  SYSCTL_RCGC2_R      |= 0x02;       // activate port B
  delay               = SYSCTL_RCGC2_R;        
  delay               = SYSCTL_RCGC2_R;   
  delay               = SYSCTL_RCGC2_R;        
  delay               = SYSCTL_RCGC2_R;  
  GPIO_PORTB_DIR_R    |= dir;        // make PB5-2 output heartbeats
  GPIO_PORTB_AFSEL_R  &= ~0x3C;       // disable alt funct on PB5-2
  GPIO_PORTB_DEN_R    |= 0x3C;        // enable digital I/O on PB5-2
  GPIO_PORTB_PCTL_R   = ~0x00FFFF00;
  GPIO_PORTB_AMSEL_R  &= ~0x3C;       // disable analog functionality on PB
}
