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
#define BUFFERINPUT 1 //If wants buffer input instead of sampling through PD0

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
	
#if BUFFERINPUT

#include "LifiEmitter.h"

#endif
	
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
#if BUFFERINPUT
#define HIGH 1700
#define LOW 100

static const unsigned long idleFrame[] = {HIGH,LOW, HIGH,LOW,LOW,HIGH, HIGH,LOW,LOW,HIGH, HIGH,LOW,LOW,HIGH, HIGH,LOW,LOW,HIGH, LOW,HIGH}; //0xAA IDLE
static const unsigned long preambleFrame[] = {HIGH,LOW, LOW,HIGH,HIGH,LOW, LOW,HIGH,HIGH,LOW, LOW,HIGH,HIGH,LOW, LOW,HIGH,LOW,HIGH, LOW,HIGH}; //0xD5 SYNC
static const unsigned long startFrame[] = {HIGH,LOW, HIGH,LOW,LOW,HIGH, HIGH,LOW,HIGH,LOW, HIGH,LOW,HIGH,LOW, HIGH,LOW,HIGH,LOW, LOW,HIGH}; //0x02 STX
static const unsigned long endFrame[] = {HIGH,LOW, LOW,HIGH,LOW,HIGH, HIGH,LOW,HIGH,LOW, HIGH,LOW,HIGH,LOW, HIGH,LOW,HIGH,LOW, LOW,HIGH}; //0x03 ETX
unsigned long dataFrame1[] = {HIGH,LOW, LOW,HIGH,HIGH,LOW, HIGH,LOW,HIGH,LOW, HIGH,LOW,HIGH,LOW, LOW,HIGH,HIGH,LOW, LOW,HIGH}; //0x41 'A'
#endif
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
#if BUFFERINPUT

// Helper function
void inputFrameIntoOSFifo(const unsigned long * frame){
	int n=0;
	for(int m=0;m<20*OVERSAMPLING;m++){
		if(n==20){
			while(1){
				; //n shouldn't be more than frame size
			}
		}
		if(OS_Fifo_Put(frame[n])==0){
			fail_counter++;
		}
		if( (m+1)%OVERSAMPLING==0 || OVERSAMPLING==1   ){
			n++;
		}
		producer_counter++;
	}		
}

void changeDataFrame1(char character){
	unsigned long int dataa=0;
	to_manchester(character, &dataa);
	manToAnalogue(&dataa, dataFrame1, HIGH, LOW ,1);
}

// Debugging the receiver functionality with hardcoded values in a buffer
void consumerFromBuffer(){
	inputFrameIntoOSFifo(idleFrame);
	inputFrameIntoOSFifo(preambleFrame);
	inputFrameIntoOSFifo(startFrame);
	inputFrameIntoOSFifo(dataFrame1);
	inputFrameIntoOSFifo(dataFrame1);
	inputFrameIntoOSFifo(dataFrame1);
	inputFrameIntoOSFifo(endFrame);
	OS_Kill();
}

#endif

//************************************** Final Threads *****************************
void producerTaskFifo(unsigned long data){

	if(OS_Fifo_Put(data)==0){
		fail_counter++;
	}
	producer_counter++;
}
int consumerTaskCounter=0;
void consumerTaskFifo(void){
	while(1){
		consumerTaskCounter++;
		unsigned long data = OS_Fifo_Get();
		sample_signal_edge(data);
	}
}
int wordDetectedTaskCounter=0;
void wordDetectedTask(void){
	while(1){
		wordDetectedTaskCounter++;
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
unsigned long bufferInput[100];
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
#if BUFFERINPUT
		if(strcmp(token,"INPUTFROMBUFFER")==0){
      SPACEFORMATTING
			NumCreated += OS_AddThread(&consumerFromBuffer,128,4);
    }
		
		if(strcmp(token,"CONVERT")==0){
			unsigned long int dataa=0;
			SPACEFORMATTING
			token=strtok(NULL,"-");
			to_manchester(token[0], &dataa);
			manToAnalogue(&dataa, bufferInput, 1, 0 ,1);
			for(int i=0;i<20;i++){
				if(bufferInput[i]==1){
					UART_OutChar('1');
				}else {
					UART_OutChar('0');
				}
				
				UART_OutChar(' ');
			  if(i==1 || i==5 || i==9 ||i==13 ||i==17){
					UART_OutChar(',');
				}
			}
		}
		if(strcmp(token,"CHANGEBUFFER")==0){
			SPACEFORMATTING
			token=strtok(NULL,"-");
			changeDataFrame1(token[0]);
		}
#endif
		
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
	// Final Thread
#if !BUFFERINPUT
	ADC_Collect(7,SYMBOLRATE*OVERSAMPLING,&producerTaskFifo); //Symboltime== Based on system Clock 80000
#endif
	//Debugging Thread
	NumCreated += OS_AddThread(&Interpreter,128,5);							//Used for debugging of ADC sampling 
	// Core Threads
	NumCreated += OS_AddThread(&consumerTaskFifo,128,4);
	NumCreated += OS_AddThread(&wordDetectedTask,128,3);
#endif
  // create initial foreground threads
  NumCreated += OS_AddThread(&IdleTask,128,6);  // runs when nothing useful to do
  //NumCreated += OS_AddThread(&thread1,128,5);
  //NumCreated += OS_AddThread(&bufferReader,128,4);
  OS_Launch(TIME_SLICE); // doesn't return, interrupts enabled in here

  return -1;             // this never executes
}
