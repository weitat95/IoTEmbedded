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
#define AUDIOOUT 0 //Outputing audio to output (GPIO PORT TO BE DEFINED)
#define SDDEBUG 1 //Enable SD Card Debugging 

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "os.h"
#include "adct0atrigger.h"
#include "uart.h"
#include "pll.h"
#include "gpio.h"
#include "ff.h"
#include "efile.h"
#include "eDisk.h"
#ifdef RECEIVER
#include "ST7735.h"
#include "DAC.h"
#endif

#include "inc/tm4c123gh6pm.h"

#ifdef EMITTER

#include "LifiEmitter.h"

#endif

#ifdef RECEIVER 
int oversampling = 2;
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
char str[50];
void diskError(char* errtype, unsigned long n){
  UART_OutString(errtype);
  sprintf(str," disk error %u",n);
  UART_OutString(str);
  OS_Kill();
}

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
unsigned char bufferFile[512];
#if BUFFERINPUT
#define HIGH 1700
#define LOW 100

static const unsigned long idleFrame[] = {HIGH,LOW, HIGH,LOW,LOW,HIGH, HIGH,LOW,LOW,HIGH, HIGH,LOW,LOW,HIGH, HIGH,LOW,LOW,HIGH, LOW,HIGH}; //0xAA IDLE
static const unsigned long preambleFrame[] = {HIGH,LOW, LOW,HIGH,HIGH,LOW, LOW,HIGH,HIGH,LOW, LOW,HIGH,HIGH,LOW, LOW,HIGH,LOW,HIGH, LOW,HIGH}; //0xD5 SYNC
static const unsigned long startFrame[] = {HIGH,LOW, HIGH,LOW,LOW,HIGH, HIGH,LOW,HIGH,LOW, HIGH,LOW,HIGH,LOW, HIGH,LOW,HIGH,LOW, LOW,HIGH}; //0x02 STX
static const unsigned long endFrame[] = {HIGH,LOW, LOW,HIGH,LOW,HIGH, HIGH,LOW,HIGH,LOW, HIGH,LOW,HIGH,LOW, HIGH,LOW,HIGH,LOW, LOW,HIGH}; //0x03 ETX
unsigned long dataFrame1[] = {HIGH,LOW, LOW,HIGH,HIGH,LOW, HIGH,LOW,HIGH,LOW, HIGH,LOW,HIGH,LOW, LOW,HIGH,HIGH,LOW, LOW,HIGH}; //0x41 'A'
#endif
	
#if AUDIOOUT
#define NUM_ELEMENTS 1008
BYTE data[NUM_ELEMENTS] = {
142, 145, 138, 137, 129, 113, 108, 108, // 0-7
111, 118, 126, 123, 129, 134, 140, 142, // 8-15
140, 132, 123, 122, 119, 117, 115, 112, // 16-23
109, 116, 120, 123, 126, 121, 119, 121, // 24-31
135, 157, 184, 170, 156, 137, 120, 117, // 32-39
121, 107,  91,  90,  89, 113, 132, 148, // 40-47
145, 146, 149, 153, 158, 151, 130, 110, // 48-55
104,  98, 107, 112, 108, 110, 118, 134, // 56-63
148, 160, 154, 145, 138, 137, 131, 127, // 64-71
116, 101, 100, 109, 120, 130, 139, 138, // 72-79
141, 148, 150, 145, 135, 120, 110, 111, // 80-87
112, 113, 115, 115, 118, 128, 135, 136, // 88-95
134, 127, 122, 120, 119, 110, 108, 116, // 96-103
149, 170, 164, 162, 136, 126, 128, 133, // 104-111
113, 109,  90,  90, 109, 128, 139, 138, // 112-119
141, 137, 148, 158, 153, 134, 121, 107, // 120-127
103, 117, 114, 112, 110, 115, 125, 144, // 128-135
153, 149, 142, 136, 135, 134, 134, 119, // 136-143
108, 105, 106, 117, 127, 129, 129, 137, // 144-151
143, 149, 152, 147, 133, 124, 118, 111, // 152-159
113, 111, 108, 114, 122, 130, 139, 140, // 160-167
133, 131, 126, 124, 120, 114, 101,  98, // 168-175
117, 155, 162, 165, 154, 131, 129, 141, // 176-183
134, 120, 109,  85,  93, 112, 130, 134, // 184-191
135, 133, 132, 150, 159, 152, 137, 123, // 192-199
105, 111, 120, 115, 114, 109, 109, 122, // 200-207
143, 149, 148, 142, 134, 131, 137, 136, // 208-215
123, 115, 105, 104, 116, 126, 128, 129, // 216-223
130, 132, 147, 153, 148, 138, 126, 116, // 224-231
116, 120, 115, 115, 111, 114, 123, 136, // 232-239
139, 138, 134, 127, 127, 130, 127, 119, // 240-247
109, 102, 110, 140, 151, 154, 149, 133, // 248-255
125, 136, 139, 130, 124, 104,  98, 107, // 256-263
123, 131, 135, 132, 124, 132, 144, 149, // 264-271
145, 137, 118, 112, 119, 120, 124, 121, // 272-279
115, 112, 123, 134, 142, 145, 139, 130, // 280-287
129, 132, 131, 132, 125, 113, 111, 115, // 288-295
122, 129, 133, 129, 128, 130, 135, 139, // 296-303
141, 134, 125, 121, 119, 121, 125, 125, // 304-311
121, 122, 127, 130, 138, 140, 135, 129, // 312-319
128, 124, 126, 129, 123, 120, 118, 117, // 320-327
121, 128, 131, 131, 131, 128, 128, 131, // 328-335
131, 129, 130, 128, 126, 128, 129, 128, // 336-343
128, 126, 124, 125, 127, 127, 130, 130, // 344-351
129, 128, 129, 128, 129, 130, 127, 125, // 352-359
125, 125, 125, 128, 128, 127, 128, 127, // 360-367
128, 129, 130, 129, 129, 128, 126, 127, // 368-375
127, 127, 127, 127, 126, 127, 128, 128, // 376-383
128, 129, 128, 128, 128, 128, 127, 128, // 384-391
127, 127, 127, 127, 127, 127, 128, 128, // 392-399
129, 129, 129, 129, 129, 130, 130, 130, // 400-407
129, 127, 125, 123, 123, 124, 125, 125, // 408-415
126, 126, 125, 125, 124, 121, 123, 132, // 416-423
135, 141, 143, 137, 131, 128, 122, 123, // 424-431
126, 124, 123, 121, 118, 118, 123, 128, // 432-439
131, 136, 136, 132, 131, 130, 128, 131, // 440-447
131, 127, 126, 122, 119, 119, 122, 124, // 448-455
128, 130, 130, 129, 130, 130, 130, 133, // 456-463
133, 131, 130, 126, 123, 123, 123, 124, // 464-471
126, 126, 126, 126, 126, 127, 129, 131, // 472-479
132, 132, 132, 129, 128, 127, 125, 126, // 480-487
127, 126, 126, 126, 125, 125, 126, 127, // 488-495
129, 130, 130, 130, 130, 129, 128, 128, // 496-503
128, 128, 128, 127, 126, 125, 125, 125, // 504-511
126, 127, 128, 129, 129, 128, 128, 128, // 512-519
128, 128, 129, 129, 128, 128, 127, 126, // 520-527
126, 126, 126, 127, 127, 127, 128, 128, // 528-535
128, 128, 128, 128, 129, 129, 128, 128, // 536-543
128, 127, 127, 127, 127, 127, 127, 127, // 544-551
127, 127, 127, 127, 128, 128, 128, 128, // 552-559
128, 128, 128, 128, 127, 127, 127, 127, // 560-567
127, 127, 127, 127, 127, 127, 127, 127, // 568-575
128, 128, 128, 128, 128, 128, 128, 128, // 576-583
128, 128, 128, 128, 128, 127, 127, 127, // 584-591
127, 127, 127, 127, 127, 128, 128, 128, // 592-599
128, 128, 128, 128, 128, 128, 128, 128, // 600-607
128, 128, 127, 127, 127, 127, 127, 127, // 608-615
127, 128, 128, 128, 128, 128, 128, 128, // 616-623
128, 128, 128, 128, 128, 128, 128, 127, // 624-631
127, 127, 127, 127, 127, 128, 128, 128, // 632-639
128, 128, 129, 129, 128, 128, 128, 127, // 640-647
127, 127, 126, 126, 126, 126, 126, 127, // 648-655
127, 127, 127, 128, 128, 128, 128, 128, // 656-663
128, 128, 128, 127, 127, 127, 127, 127, // 664-671
128, 128, 129, 129, 128, 128, 128, 127, // 672-679
127, 127, 128, 128, 128, 128, 128, 127, // 680-687
127, 127, 127, 127, 127, 127, 128, 128, // 688-695
128, 128, 128, 128, 128, 128, 127, 128, // 696-703
128, 128, 128, 128, 128, 127, 127, 127, // 704-711
127, 127, 127, 127, 127, 128, 128, 128, // 712-719
127, 127, 127, 127, 127, 127, 127, 127, // 720-727
126, 126, 126, 126, 127, 128, 130, 131, // 728-735
132, 131, 131, 130, 128, 127, 126, 125, // 736-743
125, 126, 126, 127, 127, 127, 127, 127, // 744-751
126, 126, 126, 126, 127, 128, 128, 129, // 752-759
130, 130, 130, 129, 128, 128, 127, 126, // 760-767
126, 126, 127, 127, 127, 128, 128, 128, // 768-775
127, 127, 127, 127, 127, 127, 128, 128, // 776-783
128, 128, 128, 128, 128, 128, 127, 127, // 784-791
127, 127, 127, 127, 128, 128, 128, 129, // 792-799
129, 129, 129, 128, 128, 127, 126, 126, // 800-807
126, 125, 126, 126, 126, 127, 127, 128, // 808-815
128, 128, 128, 128, 127, 127, 127, 126, // 816-823
126, 126, 127, 128, 129, 130, 131, 132, // 824-831
132, 131, 130, 128, 127, 126, 125, 125, // 832-839
125, 125, 126, 126, 127, 127, 128, 128, // 840-847
128, 127, 127, 127, 127, 128, 128, 129, // 848-855
129, 129, 129, 129, 129, 128, 127, 127, // 856-863
126, 126, 126, 126, 126, 127, 127, 128, // 864-871
128, 129, 129, 129, 129, 128, 128, 128, // 872-879
128, 127, 127, 127, 126, 126, 126, 126, // 880-887
126, 126, 126, 127, 127, 127, 127, 127, // 888-895
128, 128, 127, 127, 126, 126, 127, 127, // 896-903
128, 129, 130, 131, 132, 132, 132, 131, // 904-911
130, 128, 127, 125, 125, 124, 124, 124, // 912-919
125, 125, 126, 127, 128, 128, 128, 129, // 920-927
129, 129, 128, 128, 128, 128, 128, 128, // 928-935
128, 128, 128, 128, 128, 128, 127, 127, // 936-943
127, 127, 127, 127, 128, 128, 128, 127, // 944-951
127, 127, 127, 127, 127, 127, 127, 127, // 952-959
127, 127, 127, 127, 127, 127, 127, 127, // 960-967
127, 127, 126, 126, 127, 126, 128, 129, // 968-975
130, 131, 132, 133, 133, 132, 131, 129, // 976-983
128, 126, 125, 124, 123, 123, 123, 124, // 984-991
125, 126, 127, 128, 128, 129, 129, 129, // 992-999
129, 129, 129, 129, 128, 128, 128};// 33048-33053

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
#if SDDEBUG


#endif
#if BUFFERINPUT

// Helper function
void inputFrameIntoOSFifo(const unsigned long * frame){
	int n=0;
	for(int m=0;m<20*oversampling;m++){
		if(n==20){
			while(1){
				; //n shouldn't be more than frame size
			}
		}
		if(OS_Fifo_Put(frame[n])==0){
			fail_counter++;
		}
		if( (m+1)%oversampling==0 || oversampling==1   ){
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
#if AUDIOOUT
void audioOutThread(void){
	while(1){
		for(int i=0;i<NUM_ELEMENTS;i++){
			
		}
	}
}
#endif

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

//Globals for File Systems
static FATFS g_sFatFs;
FIL Handle,Handle2;
FRESULT fresult;

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
//**************************************** SD CARD DEBUGGING ***************************

#if SDDEBUG
		if(strcmp(token,"SDREAD")==0){
			int i;  UINT n;			
			SPACEFORMATTING
			fresult = f_mount(0, &g_sFatFs);
			UART_OutString("Mounted FS\n\r");
			if(fresult) diskError("f_mount",0);
			// assumes there is a short text file named file5.txt
			fresult = f_open(&Handle,"file5.txt", FA_READ);
			if(fresult) diskError("f_open",0);
			UART_OutString("Opened file5.txt\n\r");
			i=0;
			UART_OutString("Start Reading..\n\r");
			do{
				fresult = f_read(&Handle,&bufferFile[i],1,&n);
				if(fresult) diskError("f_read",0);
				i++;
			} while((n==1)&&(i<511));
			bufferFile[i] = 0; // null termination
			UART_OutString("Done Reading into Buffer\n\r");
			fresult = f_close(&Handle);
			if(fresult) diskError("f_close",0);
			UART_OutString("File Closed\n\r");
			fresult = f_mount(0,0); //Unmount
			if(fresult) diskError("f_unmount",0);
			UART_OutString("Unmount Fs\n\r");
			UART_OutString("Done ReadTest Task\n\r");
			token=strtok(NULL,"-");
			ST7735_OutString(0,0,token,ST7735_WHITE);

			continue;
		}
#endif
//**************************************** RECEIVER ************************************
#ifdef RECEIVER
    if(strcmp(token,"ADCIN")==0){
      SPACEFORMATTING
      resetProducerTask();
      ADC_Collect(7,SYMBOLRATE*oversampling,&producerTask); //Symboltime== Based on system Clock 80000
			while(k<100){
				;
			}
			for(int i=0;i<100;i++){
				printf("\n\r%d",buffer[i]);
			}
			continue;
    }
		if(strcmp(token,"ADC2IN")==0){
      SPACEFORMATTING
      resetProducerTask();
      ADC_Collect(7,SYMBOLRATE,&producerTask); //Symboltime== Based on system Clock 80000
			continue;
		}
#if BUFFERINPUT
		if(strcmp(token,"INPUTFROMBUFFER")==0){
      SPACEFORMATTING
			NumCreated += OS_AddThread(&consumerFromBuffer,128,4);
			continue;
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
			continue;
		}
		if(strcmp(token,"CHANGEBUFFER")==0){
			SPACEFORMATTING
			token=strtok(NULL,"-");
			changeDataFrame1(token[0]);
			continue;
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

void SW1Push1(void){
  
}

//******************************************* MAIN FUNCTION ******************************

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
	OS_AddPeriodicThread(&disk_timerproc,10*TIME_1MS,0);   // time out routines for disk

#ifdef EMITTER
  OS_AddPeriodicThread(&emit_half_bit,TIME_1S/SYMBOLRATE,3);
	NumCreated += OS_AddThread(&Interpreter,128,4);
	//strcpy(comBuffer,"ABCD");
	//NumCreated +=OS_AddThread(&sendPeriodicDataTask,128,4);
#endif
#ifdef RECEIVER
	OS_Fifo_Init(2048);
	initLifiReceiver(oversampling);

	//Debugging Thread
	NumCreated += OS_AddThread(&Interpreter,128,5);							//Used for debugging of ADC sampling 
	// Core Threads
	NumCreated += OS_AddThread(&consumerTaskFifo,128,4);
	NumCreated += OS_AddThread(&wordDetectedTask,128,3);
#if AUDIOOUT
	NumCreated += OS_AddThread(&audioOutThread,128,5);
#endif



#endif
  // create initial foreground threads
	//OS_AddSW1Task(&SW1Push1,2);
  NumCreated += OS_AddThread(&IdleTask,128,6);  // runs when nothing useful to do
  //NumCreated += OS_AddThread(&thread1,128,5);
  //NumCreated += OS_AddThread(&bufferReader,128,4);
	// Final Thread
	#ifdef RECEIVER
#if !BUFFERINPUT
	ADC_Collect(7,SYMBOLRATE*oversampling,&producerTaskFifo); //Symboltime== Based on system Clock 80000
#endif
#endif
  OS_Launch(TIME_SLICE); // doesn't return, interrupts enabled in here

  return -1;             // this never executes
}
