// Name: ADC.c
// Authors: Corentin Dugue & Wei Tat Lee
// Creation: 1/23/2017
// Description: Provide functions for using ADC(timer-triggered sampling)
// Lab 1
// TA: Kishore Punniyamurthy
// Last modified: 02/06/2017
// Hardware connections: none specific


/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015

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

 #define NVIC_EN0_INT17          0x00020000  // Interrupt 17 enable

 #define TIMER_CFG_16_BIT        0x00000004  // 16-bit timer configuration,
                                             // function is controlled by bits
                                             // 1:0 of GPTMTAMR and GPTMTBMR
 #define TIMER_TAMR_TACDIR       0x00000010  // GPTM Timer A Count Direction
 #define TIMER_TAMR_TAMR_PERIOD  0x00000002  // Periodic Timer mode
 #define TIMER_CTL_TAOTE         0x00000020  // GPTM TimerA Output Trigger
                                             // Enable
 #define TIMER_CTL_TAEN          0x00000001  // GPTM TimerA Enable
 #define TIMER_IMR_TATOIM        0x00000001  // GPTM TimerA Time-Out Interrupt
                                             // Mask
 #define TIMER_TAILR_TAILRL_M    0x0000FFFF  // GPTM TimerA Interval Load
                                             // Register Low

 #define ADC_ACTSS_ASEN3         0x00000008  // ADC SS3 Enable
 #define ADC_RIS_INR3            0x00000008  // SS3 Raw Interrupt Status
 #define ADC_IM_MASK3            0x00000008  // SS3 Interrupt Mask
 #define ADC_ISC_IN3             0x00000008  // SS3 Interrupt Status and Clear
 #define ADC_EMUX_EM3_M          0x0000F000  // SS3 Trigger Select mask
 #define ADC_EMUX_EM3_TIMER      0x00005000  // Timer
 #define ADC_SSPRI_SS3_4TH       0x00003000  // fourth priority
 #define ADC_SSPRI_SS2_3RD       0x00000200  // third priority
 #define ADC_SSPRI_SS1_2ND       0x00000010  // second priority
 #define ADC_SSPRI_SS0_1ST       0x00000000  // first priority
 #define ADC_PSSI_SS3            0x00000008  // SS3 Initiate
 #define ADC_SSCTL3_TS0          0x00000008  // 1st Sample Temp Sensor Select
 #define ADC_SSCTL3_IE0          0x00000004  // 1st Sample Interrupt Enable
 #define ADC_SSCTL3_END0         0x00000002  // 1st Sample is End of Sequence
 #define ADC_SSCTL3_D0           0x00000001  // 1st Sample Diff Input Select
 #define ADC_SSFIFO3_DATA_M      0x00000FFF  // Conversion Result Data mask
 #define ADC_PC_SR_M             0x0000000F  // ADC Sample Rate
 #define ADC_PC_SR_125K          0x00000001  // 125 ksps
 #define SYSCTL_RCGCGPIO_R4      0x00000010  // GPIO Port E Run Mode Clock
                                             // Gating Control
 #define SYSCTL_RCGCGPIO_R3      0x00000008  // GPIO Port D Run Mode Clock
                                             // Gating Control
 #define SYSCTL_RCGCGPIO_R1      0x00000002  // GPIO Port B Run Mode Clock
                                             // Gating Control
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "ADC.h"

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

// ***************** adcChannelPortInit ****************
// Initialize the channel for adc 
// Inputs:  channel number 0-11
// Outputs: int 1: success, 0: failure
int adcChannelPortInit(uint32_t channelNum,uint16_t ADC_Number){
  volatile uint32_t delay;
  if(ADC_Number==0){
    SYSCTL_RCGC0_R |= SYSCTL_RCGC0_ADC0;
  }else{
    SYSCTL_RCGC0_R |= SYSCTL_RCGC0_ADC1;
  }
  // **** GPIO pin initialization ****
  switch(channelNum){             // 1) activate clock
    case 0:
    case 1:
    case 2:
    case 3:
    case 8:
    case 9:                       //    these are on GPIO_PORTE
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; break;
    case 4:
    case 5:
    case 6:
    case 7:                       //    these are on GPIO_PORTD
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3; break;
    case 10:
    case 11:                      //    these are on GPIO_PORTB
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; break;
    default: return 0;              //    0 to 11 are valid channels on the LM4F120
  }
  delay = SYSCTL_RCGCGPIO_R;      // 2) allow time for clock to stabilize
  delay = SYSCTL_RCGCGPIO_R;
  switch(channelNum){
    case 0:                       //      Ain0 is on PE3
      GPIO_PORTE_DIR_R &= ~0x08;  // 3.0) make PE3 input
      GPIO_PORTE_AFSEL_R |= 0x08; // 4.0) enable alternate function on PE3
      GPIO_PORTE_DEN_R &= ~0x08;  // 5.0) disable digital I/O on PE3
      GPIO_PORTE_AMSEL_R |= 0x08; // 6.0) enable analog functionality on PE3
      break;
    case 1:                       //      Ain1 is on PE2
      GPIO_PORTE_DIR_R &= ~0x04;  // 3.1) make PE2 input
      GPIO_PORTE_AFSEL_R |= 0x04; // 4.1) enable alternate function on PE2
      GPIO_PORTE_DEN_R &= ~0x04;  // 5.1) disable digital I/O on PE2
      GPIO_PORTE_AMSEL_R |= 0x04; // 6.1) enable analog functionality on PE2
      break;
    case 2:                       //      Ain2 is on PE1
      GPIO_PORTE_DIR_R &= ~0x02;  // 3.2) make PE1 input
      GPIO_PORTE_AFSEL_R |= 0x02; // 4.2) enable alternate function on PE1
      GPIO_PORTE_DEN_R &= ~0x02;  // 5.2) disable digital I/O on PE1
      GPIO_PORTE_AMSEL_R |= 0x02; // 6.2) enable analog functionality on PE1
      break;
    case 3:                       //      Ain3 is on PE0
      GPIO_PORTE_DIR_R &= ~0x01;  // 3.3) make PE0 input
      GPIO_PORTE_AFSEL_R |= 0x01; // 4.3) enable alternate function on PE0
      GPIO_PORTE_DEN_R &= ~0x01;  // 5.3) disable digital I/O on PE0
      GPIO_PORTE_AMSEL_R |= 0x01; // 6.3) enable analog functionality on PE0
      break;
    case 4:                       //      Ain4 is on PD3
      GPIO_PORTD_DIR_R &= ~0x08;  // 3.4) make PD3 input
      GPIO_PORTD_AFSEL_R |= 0x08; // 4.4) enable alternate function on PD3
      GPIO_PORTD_DEN_R &= ~0x08;  // 5.4) disable digital I/O on PD3
      GPIO_PORTD_AMSEL_R |= 0x08; // 6.4) enable analog functionality on PD3
      break;
    case 5:                       //      Ain5 is on PD2
      GPIO_PORTD_DIR_R &= ~0x04;  // 3.5) make PD2 input
      GPIO_PORTD_AFSEL_R |= 0x04; // 4.5) enable alternate function on PD2
      GPIO_PORTD_DEN_R &= ~0x04;  // 5.5) disable digital I/O on PD2
      GPIO_PORTD_AMSEL_R |= 0x04; // 6.5) enable analog functionality on PD2
      break;
    case 6:                       //      Ain6 is on PD1
      GPIO_PORTD_DIR_R &= ~0x02;  // 3.6) make PD1 input
      GPIO_PORTD_AFSEL_R |= 0x02; // 4.6) enable alternate function on PD1
      GPIO_PORTD_DEN_R &= ~0x02;  // 5.6) disable digital I/O on PD1
      GPIO_PORTD_AMSEL_R |= 0x02; // 6.6) enable analog functionality on PD1
      break;
    case 7:                       //      Ain7 is on PD0
      GPIO_PORTD_DIR_R &= ~0x01;  // 3.7) make PD0 input
      GPIO_PORTD_AFSEL_R |= 0x01; // 4.7) enable alternate function on PD0
      GPIO_PORTD_DEN_R &= ~0x01;  // 5.7) disable digital I/O on PD0
      GPIO_PORTD_AMSEL_R |= 0x01; // 6.7) enable analog functionality on PD0
      break;
    case 8:                       //      Ain8 is on PE5
      GPIO_PORTE_DIR_R &= ~0x20;  // 3.8) make PE5 input
      GPIO_PORTE_AFSEL_R |= 0x20; // 4.8) enable alternate function on PE5
      GPIO_PORTE_DEN_R &= ~0x20;  // 5.8) disable digital I/O on PE5
      GPIO_PORTE_AMSEL_R |= 0x20; // 6.8) enable analog functionality on PE5
      break;
    case 9:                       //      Ain9 is on PE4
      GPIO_PORTE_DIR_R &= ~0x10;  // 3.9) make PE4 input
      GPIO_PORTE_AFSEL_R |= 0x10; // 4.9) enable alternate function on PE4
      GPIO_PORTE_DEN_R &= ~0x10;  // 5.9) disable digital I/O on PE4
      GPIO_PORTE_AMSEL_R |= 0x10; // 6.9) enable analog functionality on PE4
      break;
    case 10:                      //       Ain10 is on PB4
      GPIO_PORTB_DIR_R &= ~0x10;  // 3.10) make PB4 input
      GPIO_PORTB_AFSEL_R |= 0x10; // 4.10) enable alternate function on PB4
      GPIO_PORTB_DEN_R &= ~0x10;  // 5.10) disable digital I/O on PB4
      GPIO_PORTB_AMSEL_R |= 0x10; // 6.10) enable analog functionality on PB4
      break;
    case 11:                      //       Ain11 is on PB5
      GPIO_PORTB_DIR_R &= ~0x20;  // 3.11) make PB5 input
      GPIO_PORTB_AFSEL_R |= 0x20; // 4.11) enable alternate function on PB5
      GPIO_PORTB_DEN_R &= ~0x20;  // 5.11) disable digital I/O on PB5
      GPIO_PORTB_AMSEL_R |= 0x20; // 6.11) enable analog functionality on PB5
      break;
  }
  return 1;
  // ****End of GPIO pin initialization ****
}

// *************** ADC_OPEN ********************
// Initializes the ADC channel(0-11) and sampling is triggered by software
// Uses ADC_In(void) to sample accordingly.
// Inputs:  uint32_t channelNum: Specify the channel number to initialze.
//          ADC Channels with their I/O ports in TM4C123GH6PM
//          Ain0 : PE3
//          Ain1 : PE2
//          Ain2 : PE1
//          Ain3 : PE0
//          Ain4 : PD3
//          Ain5 : PD2
//          Ain6 : PD1
//          Ain7 : PD0
//          Ain8 : PE5
//          Ain9 : PE4
//          Ain10: PB4
//          Ain11: PB5
//SS3 samples at a time:1 Depth of FIFO:1
//SS2                   4               4
//SS1                   4               4
//SS0                   8               8

// Outputs: int : returns 1 if success, 0 otherwise
int ADC_Init(uint32_t channelNum){
  if(!adcChannelPortInit(channelNum,1)){
    return 0;
  }
  // *** ADC configuration ***
  ADC1_PC_R &= ~0xF;              // 7) clear max sample rate field
  ADC1_PC_R |= 0x1;               // 8) configure for 125K samples/sec
  ADC1_SSPRI_R = 0x3210;          // 9) Sequencer 3 is lowest priority
  ADC1_ACTSS_R &= ~0x0008;        // 10) disable sample sequencer 3
  ADC1_EMUX_R &= ~0xF000;         // 11) seq3 is software trigger
  ADC1_SSMUX3_R &=~ADC_SSMUX3_MUX0_M;
  ADC1_SSMUX3_R += (channelNum<<ADC_SSMUX3_MUX0_S);    // 12) set channels for SS3
  ADC1_SSCTL3_R = 0x0006;         // 13) 0110 :
                                  // TS(temperature) IE(interrupt) END(last sample of the sequence?) D(differential)
  ADC1_IM_R &= ~0x0008;           // 14) disable SS2 interrupts
  ADC1_ACTSS_R |= 0x0008;         // 15) enable sample sequencer 3

  return 1;
  // *** End of ADC Configuration ***
}

// *************** ADC_IN ********************
// Busy-wait Analog to digital conversion
// Input: none
// Output: 12-bit result of ADC conversions
// Samples ADC at previously initiated ADC channel using SS3
// 125k max sampling
// software trigger, busy-wait sampling
uint16_t ADC_In(void){
  uint16_t data;
  ADC1_PSSI_R = 0x0008;            // 1) initiate SS3
  while((ADC1_RIS_R&0x08)==0){};   // 2) wait for conversion done
  data = ADC1_SSFIFO3_R&0xFFF;     // 3) read first result
  ADC1_ISC_R = 0x0008;             // 4) acknowledge completion
  return data;
}
int status=0;

// *************** ADC_Status ********************
// Hardware interupted ADC sampling
// Input: none
// Output: int status:1->Success 0->Fail
int ADC_Status(void){
  return status;
}

uint32_t counter;
uint16_t *buffptr;
uint32_t n_samples;

// *************** ADC_Collect ********************
// Hardware interupted ADC sampling
// Input: uint32_t channelNum
//        uint32_t fs: the sampling frequency
//        uint16_t buffer[]: the buffer to stores the samples sampled
//        uint32_t numberOfSamples
// Output: int 1:success 0:fail
void (*producer)(unsigned long data);

int ADC_Collect(uint32_t channelNum, uint32_t fs,void(*task)(unsigned long data),uint32_t numberOfSamples){

  if(!adcChannelPortInit(channelNum,0)){
    return 0;
  }
  volatile uint32_t delay;
  counter=0;
  DisableInterrupts();
  SYSCTL_RCGCADC_R |= 0x01;     // activate ADC0 
  SYSCTL_RCGCTIMER_R |= 0x04;   // activate timer2
  delay = SYSCTL_RCGCTIMER_R;   // allow time to finish activating
  TIMER2_CTL_R = 0x00000000;    // disable timer2A during setup
  TIMER2_CTL_R |= 0x00000020;   // enable timer2A trigger to ADC
  TIMER2_CFG_R = 0;             // configure for 32-bit timer mode
  TIMER2_TAMR_R = 0x00000002;   // configure for periodic mode, default down-count settings
  TIMER2_TAPR_R = 0;            // prescale value for trigger
  TIMER2_TAILR_R = 80000000/fs-1;    // start value for trigger
  TIMER2_IMR_R &= ~0x00000001;    // disable all interrupts
  TIMER2_CTL_R |= 0x00000001;   // enable timer2A 32-b, periodic, no interrupts
  ADC0_ACTSS_R &= ~0x08;    // disable sample sequencer 3  
  ADC0_PC_R = 0x01;         // configure for 125K samples/sec
  ADC0_SSPRI_R = 0x3210;    // sequencer 0 is highest, sequencer 3 is lowest
  ADC0_EMUX_R = (ADC0_EMUX_R&0xFFFF0FFF)+0x5000; // timer trigger event
  ADC0_SSMUX3_R &=~ ADC_SSMUX3_MUX0_M;
  ADC0_SSMUX3_R += (channelNum<<ADC_SSMUX3_MUX0_S);    // 12) set channels for SS3
  ADC0_SSCTL3_R = 0x06;          // set flag and end                       
  ADC0_IM_R |= 0x08;             // enable SS3 interrupts
  ADC0_ACTSS_R |= 0x08;          // enable sample sequencer 3
  NVIC_PRI4_R = (NVIC_PRI4_R&0xFFFF00FF)|0x00004000; //priority 2
  NVIC_EN0_R = 1<<17;              // enable interrupt 17 in NVIC
  producer=task;
  //buffptr=buffer;
  n_samples=numberOfSamples;
  EnableInterrupts();
  status=1;
  return 1;
}

void ADC0Seq3_Handler(void){
  ADC0_ISC_R = ADC_ISC_IN3; //acknowledge ADC sequence 3 completion
  
  (*producer)(ADC0_SSFIFO3_R&ADC_SSFIFO3_DATA_M); //pass to foreground
  //buffptr++;
  counter++;
  if(counter>n_samples){
      TIMER2_CTL_R = 0x00000000;    // disable timer 2
      ADC0_ACTSS_R &= ~0x08;    // disable sample sequencer 3  
      status=0;
  }
}
