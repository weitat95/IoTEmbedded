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
// PF1/IDX1 is user input select switch
// PE1/PWM5 is user input down switch
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "os.h"
#include "adct0atrigger.h"
#include "uart.h"
#include "pll.h"
#include "gpio.h"
#include "inc/tm4c123gh6pm.h"

#include "esp8266.h"
#include "hc05.h"

#define DEFAULT_SSID "leeleelee"
#define DEFAULT_PW "0340501087"

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

#define WHITE PF1=0x02; PF2=0x04; PF3=0x08;
#define RED PF1=0x02; PF2=0; PF3=0;
#define GREEN PF1=0; PF2=0x00; PF3=0x08;
#define BLUE PF1=0; PF2=0x04; PF3=0;
#define YELLOW PF1=0x02; PF2=0x00; PF3=0x08;
#define PURPLE  PF1=0x02; PF2=0x04; PF3=0;
#define CYAN PF1=0; PF2=0x04; PF3=0x08;

uint32_t NumCreated;   // number of foreground threads created
uint32_t NumSamples;   // incremented every sample
uint32_t DataLost;     // data sent by Producer, but not received by Consumer
uint32_t Idlecount;
uint8_t  Running;                // true while robot is running

uint32_t espSendcount;
uint32_t wifiStatus;
char ssid[20];
char pw[20];
//******** IdleTask  ***************
// foreground thread, runs when no other work needed
// never blocks, never sleeps, never dies
// inputs:  none
// outputs: none

void IdleTask(void){
  RED
  while(TRUE){
    if(wifiStatus==0){
      GREEN
    }else{
      BLUE
    }
    Idlecount++;        // debugging
    //WaitForInterrupt();
  }
}

void WifiTask(void){
  if(wifiStatus==1){ return; }
  wifiStatus=1;
  printf("\n\rwifi task starting...ssid=%s pw=%s\n\r",ssid,pw);
  if(ESP8266_ConnectWifi(ssid,pw)){
    ESP8266_GetVersionNumber();
    ESP8266_GetStatus();
  }else{
    printf("wifi not connected");
    wifiStatus=0;
    OS_Kill();
  }
  while(TRUE){
    printf("\n\rwifi logging to server...\n\r");
    if(wifiStatus==0 || !logOwnServer(111,222,333)){
        wifiStatus=0;
        printf("\n\rWifi Logging Failed\n\r");
        OS_Kill();
    }
    printf("\n\rData Logged to server\n\r");
    espSendcount++;
    OS_Sleep(5000);
  }
}
#define STR_SIZE 50
char received[STR_SIZE];
void SW1_Task(void);

void BluetoothTask(void){
  //uint8_t i=0;
  char str[STR_SIZE];
  const char s[2] = "=";
  char *token;
   
  while(TRUE){
    
    HC05_InString(received,STR_SIZE);
    strcpy(str,received);
    /* get the first token */
    token = strtok(str, s);
   
    /* walk through other tokens */
    if(strcmp(token,"AT+JWP")==0){
      printf("\n\rreceived AT+JWP\n\r");
      token=strtok(NULL,",");
      printf("SSID=%s ",token);
      strcpy(ssid,token);
      token=strtok(NULL,",");
      strcpy(pw,token);
      printf("PW=%s\n\r",token);
      wifiStatus=0;
    }else if(strcmp(token,"AT+CONNECT")==0){
      SW1_Task();
    }
  
    for(int i=0;i<STR_SIZE;i++){
      received[i]=0;
    }
  }
}

void SW1_Task(void){
  if(wifiStatus==1) return;
  NumCreated += OS_AddThread(&WifiTask,128,5);
}

  

int main(void){
  OS_Init();        // OS Initialization
  Output_Init();
  HC05_InitUART();
  HC05_EnableRXInterrupt();
  ESP8266_Init(115200);
  GPIO_PortF_Init();
  Running    = 0;        // Log not running
  DataLost   = 0;        // lost data between producer and consumer
  NumSamples = 0;
  Idlecount  = 0;
  espSendcount=0;
  NumCreated = 0;
  strcpy(ssid,DEFAULT_SSID);
  strcpy(pw,DEFAULT_PW);
  
  OS_AddSW1Task(&SW1_Task,1);
  
  // create initial foreground threads
  NumCreated += OS_AddThread(&IdleTask,128,5);  // runs when nothing useful to do
 // NumCreated += OS_AddThread(&WifiTask,128,5);
  NumCreated += OS_AddThread(&BluetoothTask,128,5);
  OS_Launch(TIME_SLICE); // doesn't return, interrupts enabled in here
  
  return -1;             // this never executes
}
