/*
*  gpio.h
*
*  This file contains access to GPIO featutes of TIVA C Series
*  This project started in the class EE445M - EMBEDDED AND REAL-TIME SYSTEMS
*  - Spring 2017 - University of Texas at Austin.
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
*  Hilgad Montelo      February 13, 2017      Initial Version
*  Zhongqi Wang        February 13, 2017      Modified for debug/demo
*
*  NOTES:
*  - Professor Spring 2017 class: Prof. Jonathan W. Valvano
*  - TA Spring 2017             : Kishore Punniyamurthy
*
*/

#include <stdint.h>

#ifndef __GPIO_H__
#define __GPIO_H__

/**
 * @details It initialize the GPIO Port E
 * @return none
 */
void GPIO_PortE_Init(void);

/**
 * @details It initialize the GPIO Port F
 * @return none
 */
void GPIO_PortF_Init(void);

/**
 * @details It initialize the GPIO Port B
 * @return none
 */
void GPIO_PortB_Init(uint32_t dir);

#endif //__GPIO_H__

