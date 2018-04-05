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
//***************************** Connections ***********************************
/*
******Transmitter******
LED Input - GPIO PF2
******Receiver*********
ADCSampler- GPIO PD2 
******ST7735***
VCC 			- 3.3V
GND 			- GND
CS  			- PA3
RESET 		- PA7
A0(DATA)	- PA6
SDA(MOSI)	- PA5
SCK				-PA2
LED				-3.3V
*******SDCARD***
SDCS      - PB0
SDMOSI		- PA5
SDMISO		- PA4
SDSCK			- PA2
******DAC*******
SSI1Clk (SCLK, pin 4) connected to PD0
SSI1Fss (!CS, pin 2) connected to PD1
SSI1Tx (DIN, pin 3) connected to PD3
******Switch****

*/

//*****************************************************************************

/*
To program the emitter for the LIFI please define emitter below
to program the receiver for the LIFI please define receiver below
*/
//#define EMITTER
#define RECEIVER
#define BUFFERINPUT 0 //If wants buffer input instead of sampling
#define AUDIOOUT 0 //Outputing audio to output (GPIO PORT TO BE DEFINED)
#define SDDEBUG 0 //Enable SD Card Debugging 

#ifdef RECEIVER
#define EDGETIMERMODE 1
//Edge capture is used instead of using ADC sampling
#if (EDGETIMERMODE)&&(!AUDIOOUT)
#define EDGETIMERDEBUG 0 //using PD0 to debug edge timer,
												 //DAC is also using PD0
												 //Cannot activate both at the same time
#endif

#define AUTODETECTSPEED 1 //Auto Detect Transmitting Frequency

#endif

#define ARM_ADS

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
#include "inc/tm4c123gh6pm.h"

#ifdef RECEIVER
	#include "EdgeCount.h"
	#include "ST7735.h"
	#include "DAC.h"
	#include "LifiReceiver.h"
	#define RECINPUT 5 //Channel 5 PD2 (Channel defined in adct0atrigger.c)
	int oversampling = 1;
	#if BUFFERINPUT
		#include "LifiEmitter.h"
	#endif
	#if AUDIOOUT
		#include "mp3/fixpt/mp3dec.h"
		#include "mp3/fixpt/mp3common.h"
	#endif
#endif


#ifdef EMITTER

#include "LifiEmitter.h"

#endif


//#define DEBUG
#define SYMBOLRATE 1000 //1kHz


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
unsigned long buffer[500];
unsigned char bufferFile[512];
#if (BUFFERINPUT||EDGETIMERDEBUG)
#define HIGH 4000
#define LOW 100
static const unsigned long sleepFrame[] = {HIGH,HIGH, HIGH,HIGH,HIGH, HIGH,HIGH,HIGH, HIGH,HIGH,HIGH, HIGH,HIGH,HIGH, HIGH,HIGH}; //
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
#if !EDGETIMERMODE
void producerTaskFifo(unsigned long data){
		if(OS_Fifo_Put(data)==0){
			fail_counter++;
		}
		producer_counter++;
}
void startSampling(void){
	OS_Sleep(5000);
	ADC_Collect(RECINPUT,SYMBOLRATE*oversampling,&producerTaskFifo); //Symboltime== Based on system Clock 80000
	OS_Kill();
}
#elif EDGETIMERMODE
#define DEFTIMETHRESH (160000)*75/100 //1kHz **Changing Symbol have to change threshold as well**
uint32_t timeThresh=DEFTIMETHRESH;
int myedge=0;
unsigned long mytime;
int myi=0;
#define BUFFFSIZE 1024
unsigned long bufff[BUFFFSIZE];
//Main task for edge capture on PB6
//1 Rising Edge
//2 Rising Edge Long
//3 Falling Edge
//4 Falling Edge Long
void edgeTask(int edge,unsigned long time){
	myedge=edge;
	mytime=time;
	if(myedge==1){
		if(mytime>timeThresh){
			OS_Fifo_Put(2);
		}else{OS_Fifo_Put(1);}
	}else if(myedge==-1){
		if(mytime>timeThresh){
			OS_Fifo_Put(4);
		}else{OS_Fifo_Put(3);}
	}
	//To get the time threshold
	 
	if(myi>=BUFFFSIZE){myi=0;}
	bufff[myi]=mytime;
	myi++;
	
}
#if EDGETIMERDEBUG
void putFrame(const unsigned long * frame){
	for(int i=0;i<20;i++){
		if(frame[i]==HIGH){
			GPIO_PORTD_DATA_R |= 0x01;
			OS_SleepUs(20);
		}else if(frame[i]==LOW){
			GPIO_PORTD_DATA_R &= ~0x01;
			OS_SleepUs(20);
		}
	}
}
//Debugging task to simulate square waves in PB6 by toggleling PD0 since they are connected by R9;
void pulsePD0(void){
	int n=0;
	while(n<100){
		putFrame(sleepFrame);
		n++;
	}
	n=0;
	while(n<3){
		putFrame(idleFrame);
		n++;
	}
		putFrame(preambleFrame);
		putFrame(startFrame);
		putFrame(dataFrame1);
		putFrame(dataFrame1);
		putFrame(dataFrame1);
		putFrame(endFrame);
		putFrame(idleFrame);
		OS_Kill();
}
#endif

#endif

int consumerTaskCounter=0;
int bufffi=0;
uint32_t unstableTimeCounter=0;
int consumerBigTimeCounter=0;
void consumerTaskFifo(void){
	while(1){
		consumerTaskCounter++;
		unsigned long data = OS_Fifo_Get();
		/*if(k<500){
		buffer[k]=data;
		k++;}
		else if(k==500){	
			for(int i=0;i<500;i++){
				
				printf("%d\n\r",buffer[i]);
			}
			OS_Kill();
		}*/
		if(data==2 || data==4){
			consumerBigTimeCounter++;
		}
		//Auto Detect FrequencyChange
		if(bufff[bufffi]>=timeThresh*100*125/2/75/100||bufff[bufffi]<=timeThresh*85*100/2/75/100){
			unstableTimeCounter++;
		}else{
			unstableTimeCounter=0;
		}
#if AUTODETECTSPEED
		if(unstableTimeCounter>0x2000){ //Around 5 Seconds to trigger frequency change
			uint32_t averageVal=0;
			uint32_t freqDetected=0;
			//Take Average Value of thresh
			if(bufffi<9){
				bufffi=BUFFFSIZE;
			}
			for(int i=0;i<10;i++){
			  averageVal+=bufff[bufffi-i];
			}
			averageVal=averageVal/10;
			//freqDetected=(167894-averageVal*2)*19/150;
			timeThresh=averageVal*2*75/100;
			//timeThresh=125921-averageVal*1125/190;
			printf("\n\rDetected Change of frequency: Changed Threshold to=%d\n\r", timeThresh);
			unstableTimeCounter=0;
			bufffi=0;
		}
#endif
		bufffi++;
		if(bufffi>=BUFFFSIZE){bufffi=0;}//Prevent out of bounds
#if EDGETIMERMODE
		insertEdgeCapture(data);
#else
		sample_signal_edge(data);
#endif
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
static HMP3Decoder hMP3Decoder;

void audioOutThread(void){

	OS_Kill();
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

//Global for controlling emitter transmitting frequency;
uint8_t freqBuffPtr=0;
static const unsigned long freqBuff[] = {1000,5000,10000,20000,50000,100000};

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
#if AUDIOOUT
			if(strcmp(token,"AUDIOTEST")==0){
				SPACEFORMATTING
				NumCreated += OS_AddThread(&audioOutThread,128,2);

				continue;
			}
#endif
		
//**************************************** RECEIVER ************************************
#ifdef RECEIVER
    if(strcmp(token,"ADCIN")==0){
      SPACEFORMATTING
      resetProducerTask();
      ADC_Collect(RECINPUT,SYMBOLRATE*oversampling,&producerTask); //Symboltime== Based on system Clock 80000
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
      ADC_Collect(RECINPUT,SYMBOLRATE,&producerTask); //Symboltime== Based on system Clock 80000
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
		
#if EDGETIMERMODE
		if(strcmp(token, "THRESHUP")==0){
			SPACEFORMATTING
			timeThresh+=timeThresh*10/100;
			printf("timeThresh: %d\n\r",timeThresh);
			continue;
		}
		if(strcmp(token, "THRESHDOWN")==0){
			SPACEFORMATTING
			timeThresh-=timeThresh*10/100;
			printf("timeThresh: %d\n\r",timeThresh);
			continue;
		}
		if(strcmp(token, "THRESHDEBUG")==0){
			SPACEFORMATTING
			timeThresh=1000;
			printf("timeThresh: %d\n\r",timeThresh);
			continue;
		}
		
		if(strcmp(token, "THRESHSET")==0){
			SPACEFORMATTING
			token=strtok(NULL,"-");
			if(strcmp(token, "1000")==0){
				timeThresh=100000;
				printf("Frequency: 1kHz,Thres: %d",timeThresh);
			}
			else if(strcmp(token,"10000")==0){
				timeThresh=10000;
				printf("Frequency: 10kHz,Thres: %d",timeThresh);
			}else if(strcmp(token,"20000")==0){
				timeThresh=6000;
				printf("Frequency: 20kHz,Thres: %d",timeThresh);
			}
			printf("timeThresh: %d\n\r",timeThresh);
			continue;
		}
#if EDGETIMERDEBUG
		if(strcmp(token,"SIMULATEEDGE")==0){
			SPACEFORMATTING
			NumCreated += OS_AddThread(&pulsePD0,128,2);
			continue;
		}
#endif
#endif
		
		
#endif
		
		
//**************************************** TRANSMITTER ************************************
void emit_half_bit(void);

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
			continue;
    }
    if(strcmp(token,"SENDCONT")==0){ 
			SPACEFORMATTING
      token=strtok(NULL,"-");
      strcpy(comBuffer,token);
      SPACEFORMATTING
      NumCreated += OS_AddThread(&sendPeriodicDataTask,128,4);
			continue;
    }
	if(strcmp(token,"CHANGEFREQ")==0){
			SPACEFORMATTING
			OS_StartCritical();
			OS_DisablePeriodicThread();
			resetEmitterState();
			OS_AddPeriodicThread(&emit_half_bit,TIME_1S/freqBuff[freqBuffPtr],3);
			OS_EndCritical();
			printf("Transmitting Frequency: %d\n\r",freqBuff[freqBuffPtr]);
			freqBuffPtr++;
			if(freqBuffPtr>=6){
				freqBuffPtr=0;
			}
			continue;
		}
#endif

    UART_OutChar(CR);
    UART_OutChar(LF);
  }
}


void SW1Push1(void){
  
}

//******************************************* MAIN FUNCTION ******************************

int main(void){
  
  OS_Init();        // OS Initialization
  GPIO_PortF_Init();
#ifdef RECEIVER
	//ST7735_InitR(INITR_REDTAB);
#endif
  Running    = 0;        // Log not running
  DataLost   = 0;        // lost data between producer and consumer
  NumSamples = 0;
  Idlecount  = 0;
  count1=0;
  NumCreated = 0;
#if SDDEBUG
	OS_AddPeriodicThread(&disk_timerproc,10*TIME_1MS,0);   // time out routines for disk
#endif
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
#if !EDGETIMERMODE
	NumCreated += OS_AddThread(&startSampling,128,2);
	//ADC_Collect(RECINPUT,SYMBOLRATE*oversampling,&producerTaskFifo); //Symboltime== Based on system Clock 80000
#endif
#endif

#if EDGETIMERMODE
	EdgeTimer_Init(&edgeTask);
#if EDGETIMERDEBUG	
	//Activate Port D for simulating squarewaves to test edge capture to PB6
	SYSCTL_RCGCGPIO_R |= 0x08; // activate Port D
  while((SYSCTL_PRGPIO_R&0x08) == 0){};// allow time to finish activating
  GPIO_PORTD_DIR_R |= 0x01;  // make PD0 out
  GPIO_PORTD_AFSEL_R &= ~0x01;// disable alt funct on PD0
  GPIO_PORTD_DEN_R |= 0x01;  // enable digital I/O on PD0
                             // configure PD0 as GPIO
  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0xFFFFFFF0)+0x00000000;
  GPIO_PORTD_AMSEL_R &= ~0x01;// disable analog functionality on PD0
  GPIO_PORTD_DATA_R = 0;
#endif
#endif
#endif
  OS_Launch(TIME_SLICE); // doesn't return, interrupts enabled in here

  return -1;             // this never executes
}
