/*
*  uart_fifo.h
*
*  Provides functions that initialize a FIFO, put data in, get data out,
*  and return the current size.  The file includes a transmit FIFO
*  using index implementation and a receive FIFO using pointer
*  implementation.  Other index or pointer implementation FIFOs can be
*  created using the macros supplied at the end of the file. The functions in
*  this file also include synchronization primitives, like semaphores, to
*  maintain the Fifo's data reliable. The file used for the class EE445M -
*  EMBEDDED AND REAL-TIME SYSTEMS. EE445M - Spring 2017 - University of Texas
*  at Austin.
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
*  Jonathan Valvano    June  16, 2011         Initial/Base version (CopyRight)
*  PEGASUS TEAM        April, 10 2017         Initial Version
*
*  NOTES:
*  - Professor Spring 2017 class: Prof. Jonathan W. Valvano
*  - TA Spring 2017             : Kishore Punniyamurthy
*
*/

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to the Arm Cortex M3",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2011
   Programs 3.7, 3.8., 3.9 and 3.10 in Section 3.7

 Copyright 2011 by Jonathan W. Valvano, valvano@mail.utexas.edu
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
 
#ifndef UART_FIFO_H
#define UART_FIFO_H

typedef char tx_UARTDataType;
typedef char rx_UARTDataType;

// initialize index FIFO
void Tx_UARTFifo_Init(void);
// add element to end of index FIFO
// return TXFIFOSUCCESS if successful
int Tx_UARTFifo_Put(tx_UARTDataType data);
// remove element from front of index FIFO
// return TXFIFOSUCCESS if successful
int Tx_UARTFifo_Get(tx_UARTDataType *datapt);
// number of elements in index FIFO
// 0 to TXFIFOSIZE-1
unsigned short Tx_UARTFifo_Size(void);

// initialize pointer FIFO
void Rx_UARTFifo_Init(void);
// add element to end of pointer FIFO
// return RXFIFOSUCCESS if successful
int Rx_UARTFifo_Put(rx_UARTDataType data);
// remove element from front of pointer FIFO
// return RXFIFOSUCCESS if successful
int Rx_UARTFifo_Get(rx_UARTDataType *datapt);
// number of elements in pointer FIFO
// 0 to RXFIFOSIZE-1
unsigned short Rx_UARTFifo_Size(void);

#endif
