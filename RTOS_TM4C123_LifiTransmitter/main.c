/*
*  main.c
*
*  The file used for the class EE445M -
*  EMBEDDED AND REAL-TIME SYSTEMS. EE445M - Spring 2017 - University of Texas
*  at Austin. It runs on LM4F120/TM4C123.
*  It contains user programs, File system, stream data onto disk.
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

//*****************************************************************************

/*
To program the emitter for the LIFI please define emitter below
to program the receiver for the LIFI please define receiver below
*/
//#define EMITTER
#define RECEIVER


#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "os.h"
#include "adct0atrigger.h"
#include "uart.h"
#include "pll.h"
#include "gpio.h"

#ifdef RECEIVER
#include "ST7735.h"
#endif

#include "inc/tm4c123gh6pm.h"

#ifdef EMITTER
#include "LifiEmitter.h"
#endif
#ifdef RECEIVER
#include "LifiReceiver.h"
#endif

//#define DEBUG
#define SYMBOLRATE 50000 //1kHz


#define TIME_SLICE (2*TIME_1MS)
#define ST7735             
// PORT B - Memory Mapped GPIO
#define PB3       (*((volatile unsigned long *)0x40005020))

// PORT F - Memory Mapped GPIO
#define PF0       (*((volatile uint32_t *)0x40025004))
#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))
#define PF4       (*((volatile uint32_t *)0x40025040))

uint32_t NumCreated;   // number of foreground threads created
uint32_t NumSamples;   // incremented every sample
uint32_t DataLost;     // data sent by Producer, but not received by Consumer
uint32_t Idlecount;
uint8_t  Running;                // true while OS is running
uint32_t count1;

//Color Coding for Launchpad LED
#define WHITE PF1=0x02; PF2=0x04; PF3=0x08;
#define RED PF1=0x02; PF2=0; PF3=0;
#define GREEN PF1=0; PF2=0x00; PF3=0x08;
#define BLUE PF1=0; PF2=0x04; PF3=0;
#define YELLOW PF1=0x02; PF2=0x00; PF3=0x08;
#define PURPLE  PF1=0x02; PF2=0x04; PF3=0;
#define CYAN PF1=0; PF2=0x04; PF3=0x08;

#define SPACEFORMATTING UART_OutChar(CR); UART_OutChar(LF);


//******** IdleTask  ***************
// foreground thread, runs when no other work needed
// never blocks, never sleeps, never dies
// inputs:  none
// outputs: none

void IdleTask(void){
  
  while(TRUE){
    //GREEN
    Idlecount++;        // debugging
    //WaitForInterrupt();
  }
}
void thread1(void){
  
  while(TRUE){
    
    count1++;
  }
}
#ifdef RECEIVER
unsigned long fail_counter=0;
unsigned long producer_counter=0;
unsigned long buffer[100];
static const unsigned long stabuff[] = {1700,1700,100,100,1700,1700,100,100,1700,1700,100,100,1700,1700,100,100,1700,1700,100,100,
																				1700,1700,100,100,1700,1700,100,100,1700,1700,100,100,1700,1700,100,100,1700,1700,100,100,
																				1700,1700,100,100,1700,1700,100,100,1700,1700,100,100,1700,1700,100,100,1700,1700,100,100,
																				1700,1700,100,100,1700,1700,100,100,1700,1700,100,100,1700,1700,100,100,1700,1700,100,100,
																				1700,1700,100,100,1700,1700,100,100,1700,1700,100,100,1700,1700,100,100,1700,1700,100,100};
																				// TODO: Write a High Level Function to convert a byte of Data (0xD5) into binary with wrapping 
																				//        Taking the Oversampling rate into account as well
static const unsigned long dataFrame1[] = {1700,100, 1700,100,100,1700, 1700,100,100,1700, 1700,100,100,1700, 1700,100,100,1700, 100,1700, // 0xAA
																					1700,100, 1700,100,100,1700, 1700,100,100,1700, 1700,100,100,1700, 1700,100,100,1700, 100,1700,  // 0xAA
																					1700,100, 1700,100,100,1700, 1700,100,100,1700, 1700,100,100,1700, 1700,100,100,1700, 100,1700,	// 0xAA
																					1700,100, 1700,100,100,1700, 1700,100,100,1700, 1700,100,100,1700, 1700,100,100,1700, 100,1700,	// 0xAA
																					1700,100, 100,1700,1700,100, 100,1700,1700,100, 100,1700,1700,100, 100,1700,100,1700, 100,1700 // 0xD5 SYNC
																					};
int k=0;
//************************************* Debugging Threads *************************
void producerTask(unsigned long data){
  if(k<100){
    buffer[k]=data;
    k++;
  }
}
void resetProducerTask(void){
  k=0;
}

// Debugging the receiver functionality with hardcoded values in a buffer
void consumerFromBuffer(){
	for(int m=0;m<100;m++){
		if(OS_Fifo_Put(stabuff[m])==0){
			fail_counter++;
		}
		producer_counter++;
	}
	OS_Kill();
}


//************************************** Final Threads *****************************
void producerTaskFifo(unsigned long data){
	
	//if(k<100){
	//	buffer[k]=data;
	//	k++;
	//}
	if(OS_Fifo_Put(data)==0){
		fail_counter++;
	}
	producer_counter++;
}

void consumerTaskFifo(void){
	while(1){
		unsigned long data = OS_Fifo_Get();
		sample_signal_edge(data);
	}
}

void wordDetectedTask(void){
	while(1){
		getDataFrame();
	}
}

#endif

#ifdef EMITTER

int sendData(char* data){
  return write(data,strlen(data));
}
char comBuffer[32];
void sendPeriodicDataTask(void){
  int n=0;
  int counter=-99;
  while(1){
    while(sendData(comBuffer)!=0){
			strcpy(comBuffer,"1234567890123456789012345678901");
      OS_Sleep(500);
      counter++;
    }
    printf("Data Sent:%u",n);
    SPACEFORMATTING
    n++;
  }
  printf("Data Sending Task Done (sent:%u, Wait Counter:%u)",n,counter);
  SPACEFORMATTING
  OS_Kill();
}
#endif
char string[32];

void Interpreter(void) {
  char str[32];
  char *token;
  UART_Init();
#ifdef EMITTER  
  initEmitter();
#endif
  UART_OutChar(CR);
  UART_OutChar(LF);
  while(1){
    UART_OutChar('-');
    UART_OutChar('-');
    UART_OutChar('>');
    UART_InString(string,29);
    strcpy(str,string);
    token =strtok(str,"-");
//**************************************** RECEIVER ************************************
#ifdef RECEIVER
    if(strcmp(token,"ADCIN")==0){
      SPACEFORMATTING
      resetProducerTask();
      ADC_Collect(7,SYMBOLRATE*OVERSAMPLING,&producerTask); //Symboltime== Based on system Clock 80000
    }
		if(strcmp(token,"ADC2IN")==0){
      SPACEFORMATTING
      resetProducerTask();
      ADC_Collect(7,SYMBOLRATE,&producerTask); //Symboltime== Based on system Clock 80000
    }
		if(strcmp(token,"INPUTFROMBUFFER")==0){
      SPACEFORMATTING
			NumCreated += OS_AddThread(&consumerFromBuffer,128,4);
    }
#endif
		
		
//**************************************** TRANSMITTER ************************************

#ifdef EMITTER
    if(strcmp(token,"SEND")==0){
#ifdef DEBUG
			resetCounter();
#endif
      token=strtok(NULL,"-");
      UART_OutChar(CR);
      UART_OutChar(LF);
      OS_StartCritical();
      int ret=write(token,strlen(token));
      OS_EndCritical();
      if(ret==0)  UART_OutString("Data sent");
      else UART_OutString("Data not sent");
    }
    if(strcmp(token,"SENDCONT")==0){ 
      token=strtok(NULL,"-");
      strcpy(comBuffer,token);
      SPACEFORMATTING
      NumCreated += OS_AddThread(&sendPeriodicDataTask,128,4);
    }
#endif

    UART_OutChar(CR);
    UART_OutChar(LF);
  }
}

void emit_half_bit(void);
int main(void){
  
  OS_Init();        // OS Initialization
  GPIO_PortF_Init();
#ifdef RECEIVER
	ST7735_InitR(INITR_REDTAB);
#endif
  Running    = 0;        // Log not running
  DataLost   = 0;        // lost data between producer and consumer
  NumSamples = 0;
  Idlecount  = 0;
  count1=0;
  NumCreated = 0;

#ifdef EMITTER
  OS_AddPeriodicThread(&emit_half_bit,TIME_1S/SYMBOLRATE,3);
	NumCreated += OS_AddThread(&Interpreter,128,4);
	//strcpy(comBuffer,"ABCD");
	//NumCreated +=OS_AddThread(&sendPeriodicDataTask,128,4);
#endif
#ifdef RECEIVER
	OS_Fifo_Init(2048);
	initLifiReceiver();
  //ADC_Collect(7,SYMBOLRATE*OVERSAMPLING,&producerTaskFifo); //Symboltime== Based on system Clock 80000
	NumCreated += OS_AddThread(&consumerTaskFifo,128,4);
	NumCreated += OS_AddThread(&wordDetectedTask,128,5);
	NumCreated += OS_AddThread(&Interpreter,128,5);							//Used for debugging of ADC sampling 

#endif
  // create initial foreground threads
  NumCreated += OS_AddThread(&IdleTask,128,6);  // runs when nothing useful to do
  //NumCreated += OS_AddThread(&thread1,128,5);
  //NumCreated += OS_AddThread(&bufferReader,128,4);
  OS_Launch(TIME_SLICE); // doesn't return, interrupts enabled in here

  return -1;             // this never executes
}
