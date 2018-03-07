// Name: ADC.h
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
 int ADC_Init(uint32_t channelNum);
 
 
 // *************** ADC_IN ********************
 // Busy-wait Analog to digital conversion
 // Input: none
 // Output: 12-bit result of ADC conversions
 // Samples ADC at previously initiated ADC channel using SS3
 // 125k max sampling
 // software trigger, busy-wait sampling
 uint16_t ADC_In(void);

// *************** ADC_Collect ********************
// Hardware interupted ADC sampling
// Input: uint32_t channelNum
//        uint32_t fs: the sampling frequency
//        uint16_t buffer[]: the buffer to stores the samples sampled
//        uint32_t numberOfSamples
// Output: int 1:success 0:fail
int ADC_Collect(uint32_t channelNum, uint32_t fs,void (*task)(unsigned long data),uint32_t numberOfSamples);
 
 // *************** ADC_Status ********************
// Hardware interupted ADC sampling
// Input: none
// Output: int status:1->Success 0->Fail
 int ADC_Status(void);
