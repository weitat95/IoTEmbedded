// Name: Lab4.c
// Modified: Corentin Dugue & Wei Tat Lee
// 
// Lab 4
// TA: Kishore Punniyamurthy
// Last modified: 03/29/2017
// Hardware connections: none specific

//*****************************************************************************
//
// Lab4.c - user programs, File system, stream data onto disk
//*****************************************************************************

// Jonathan W. Valvano 3/7/17, valvano@mail.utexas.edu
// EE445M/EE380L.6
// You may use, edit, run or distribute this file
// You are free to change the syntax/organization of this file to do Lab 4
// as long as the basic functionality is simular
// 1) runs on your Lab 2 or Lab 3
// 2) implements your own eFile.c system with no code pasted in from other sources
// 3) streams real-time data from robot onto disk
// 4) supports multiple file reads/writes
// 5) has an interpreter that demonstrates features
// 6) interactive with UART input, and switch input

// LED outputs to logic analyzer for OS profile
// PF1 is preemptive thread switch
// PF2 is periodic task
// PF3 is SW1 task (touch PF4 button)

// Button inputs
// PF0 is SW2 task
// PF4 is SW1 button input

// Analog inputs
// PE3 sequencer 3, channel 3, J8/PE0, sampling in DAS(), software start
// PE0 timer-triggered sampling, channel 0, J5/PE3, 50 Hz, processed by Producer
//******Sensor Board I/O*******************
// **********ST7735 TFT and SDC*******************
// ST7735
// Backlight (pin 10) connected to +3.3 V
// MISO (pin 9) unconnected
// SCK (pin 8) connected to PA2 (SSI0Clk)
// MOSI (pin 7) connected to PA5 (SSI0Tx)
// TFT_CS (pin 6) connected to PA3 (SSI0Fss)
// CARD_CS (pin 5) connected to PB0
// Data/Command (pin 4) connected to PA6 (GPIO), high for data, low for command
// RESET (pin 3) connected to PA7 (GPIO)
// VCC (pin 2) connected to +3.3 V
// Gnd (pin 1) connected to ground

// HC-SR04 Ultrasonic Range Finder
// J9X  Trigger0 to PB7 output (10us pulse)
// J9X  Echo0    to PB6 T0CCP0
// J10X Trigger1 to PB5 output (10us pulse)
// J10X Echo1    to PB4 T1CCP0
// J11X Trigger2 to PB3 output (10us pulse)
// J11X Echo2    to PB2 T3CCP0
// J12X Trigger3 to PC5 output (10us pulse)
// J12X Echo3    to PF4 T2CCP0

// Ping))) Ultrasonic Range Finder
// J9Y  Trigger/Echo0 to PB6 T0CCP0
// J10Y Trigger/Echo1 to PB4 T1CCP0
// J11Y Trigger/Echo2 to PB2 T3CCP0
// J12Y Trigger/Echo3 to PF4 T2CCP0

// IR distance sensors
// J5/A0/PE3
// J6/A1/PE2
// J7/A2/PE1
// J8/A3/PE0

// ESP8266
// PB1 Reset
// PD6 Uart Rx <- Tx ESP8266
// PD7 Uart Tx -> Rx ESP8266

// Free pins (debugging)
// PF3, PF2, PF1 (color LED)
// PD3, PD2, PD1, PD0, PC4
char str[50];
#include "OS.h"
#include "inc/tm4c123gh6pm.h"
#include "ST7735.h"
#include "ADC.h"
#include "UART.h"
#include "eDisk.h"
#include "eFile.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "PLL.h"

#include "ff.h"
#define var0 NumSamples
#define var0s "NumSamples"
#define var1 NumCreated
#define var1s "NumCreated"
#define var2 MaxJitter
#define var2s "MaxJitter"
#define var3 DataLost
#define var3s "DataLost"
#define var4 FilterWork
#define var4s "FilterWork"
#define var5 PIDWork
#define var5s "PIDWork"

//*********Prototype for FFT in cr4_fft_64_stm32.s, STMicroelectronics
void cr4_fft_64_stm32(void *pssOUT, void *pssIN, unsigned short Nbin);

#define PF0  (*((volatile unsigned long *)0x40025004))
#define PF1  (*((volatile unsigned long *)0x40025008))
#define PF2  (*((volatile unsigned long *)0x40025010))
#define PF3  (*((volatile unsigned long *)0x40025020))
#define PF4  (*((volatile unsigned long *)0x40025040))

#define PD0  (*((volatile unsigned long *)0x40007004))
#define PD1  (*((volatile unsigned long *)0x40007008))
#define PD2  (*((volatile unsigned long *)0x40007010))
#define PD3  (*((volatile unsigned long *)0x40007020))

#define PB2  (*((volatile uint32_t *)0x40005010))
#define PB3  (*((volatile uint32_t *)0x40005020))
#define PB4  (*((volatile uint32_t *)0x40005040))
#define PB5  (*((volatile uint32_t *)0x40005080))
#define PB6  (*((volatile uint32_t *)0x40005100))
#define PB7  (*((volatile uint32_t *)0x40005200))  
void PortB_Init(void){
  SYSCTL_RCGCGPIO_R |= 0x00000002;     // 1) activate clock for Port F
  while((SYSCTL_PRGPIO_R&0x02) == 0){};// ready?  
  GPIO_PORTB_DIR_R |= 0xFC;             // make PB2-7 out 
  GPIO_PORTB_AFSEL_R &= ~0xFC;          // disable alt funct on PB2-7
  GPIO_PORTB_DEN_R |= 0xFC;             // enable digital I/O on PB2-7
                                        // configure PB2-7 as GPIO
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTB_AMSEL_R = 0;               // disable analog functionality on PB
}

void PortD_Init(void){
  SYSCTL_RCGCGPIO_R |= 0x08;       // activate port D
  while((SYSCTL_PRGPIO_R&0x08)==0){};
  GPIO_PORTD_DIR_R |= 0x0F;    // make PE3-0 output heartbeats
  GPIO_PORTD_AFSEL_R &= ~0x0F;   // disable alt funct on PD3-0
  GPIO_PORTD_DEN_R |= 0x0F;     // enable digital I/O on PD3-0
  GPIO_PORTD_PCTL_R = ~0x0000FFFF;
  GPIO_PORTD_AMSEL_R &= ~0x0F;;      // disable analog functionality on PD
}
unsigned long NumCreated;   // number of foreground threads created
unsigned long NumSamples;   // incremented every sample
unsigned long DataLost;     // data sent by Producer, but not received by Consumer
unsigned long PIDWork;      // current number of PID calculations finished
unsigned long FilterWork;   // number of digital filter calculations finished

int Running;                // true while robot is running

#define TIMESLICE 2*TIME_1MS  // thread switch time in system time units
long x[64],y[64];           // input and output arrays for FFT
Sema4Type doFFT;            // set every 64 samples by DAS

long median(long u1,long u2,long u3){
long result;
  if(u1>u2)
    if(u2>u3)   result=u2;     // u1>u2,u2>u3       u1>u2>u3
      else
        if(u1>u3) result=u3;   // u1>u2,u3>u2,u1>u3 u1>u3>u2
        else      result=u1;   // u1>u2,u3>u2,u3>u1 u3>u1>u2
  else
    if(u3>u2)   result=u2;     // u2>u1,u3>u2       u3>u2>u1
      else
        if(u1>u3) result=u1;   // u2>u1,u2>u3,u1>u3 u2>u1>u3
        else      result=u3;   // u2>u1,u2>u3,u3>u1 u2>u3>u1
  return(result);
}
//------------ADC2millimeter------------
// convert 12-bit ADC to distance in 1mm
// it is known the expected range is 100 to 800 mm
// Input:  adcSample 0 to 4095
// Output: distance in 1mm
long ADC2millimeter(long adcSample){
  if(adcSample<494) return 799; // maximum distance 80cm
  return (268130/(adcSample-159));
}
long Distance3;     // distance in mm on IR3
uint32_t Index3;    // counts to 64 samples
long x1,x2,x3;
void DAS(void){
long output;
  PD0 ^= 0x01;
  x3=x2; x2= x1;  // MACQ
  x1 = ADC_In();  // channel set when calling ADC_Init
  PD0 ^= 0x01;
  if(Index3<64){
    output = median(x1,x2,x3); // 3-wide median filter
    Distance3 = ADC2millimeter(output);
    FilterWork++;        // calculation finished
    x[Index3] = Distance3;
    Index3++;
    if(Index3==64){
      OS_Signal(&doFFT);
    }
  }
  PD0 ^= 0x01;
}
void DSP(void){
unsigned long DCcomponent;   // 12-bit raw ADC sample, 0 to 4095
  OS_InitSemaphore(&doFFT,0);
  while(1) {
    OS_Wait(&doFFT); // wait for 64 samples
    PD2 = 0x04;
    cr4_fft_64_stm32(y,x,64);  // complex FFT of last 64 ADC values
    PD2 = 0x00;
    Index3 = 0; // take another buffer
    DCcomponent = y[0]&0xFFFF; // Real part at frequency 0, imaginary part should be zero
    ST7735_Message(1,0,"IR3 (mm) =",DCcomponent);
  }
}
char Name[8]="robot0";
//******** Robot ***************
// foreground thread, accepts data from producer
// inputs:  none
// outputs: none
void Robot(void){
unsigned long data;      // ADC sample, 0 to 1023
unsigned long voltage;   // in mV,      0 to 3300
unsigned long distance;  // in mm,      100 to 800
unsigned long time;      // in 10msec,  0 to 1000
  OS_ClearMsTime();
  DataLost = 0;          // new run with no lost data
  OS_Fifo_Init(256);
  UART_OutString("Robot running...\n\r");
  eFile_RedirectToFile(Name); // robot0, robot1,...,robot7
  UART_OutString("time(sec)\tdata(volts)\tdistance(mm)\n\r");
  printf("time(sec)\tdata(volts)\tdistance(mm)\n\r");
  do{
    PIDWork++;    // performance measurement
    time=OS_MsTime();            // 10ms resolution in this OS
    data = OS_Fifo_Get();        // 1000 Hz sampling get from producer
    voltage = (300*data)/1024;   // in mV
    distance = ADC2millimeter(data);
    
    sprintf(str,"%0u.%02u\t%0u.%03u \t%5u\n\r",time/100,time%100,voltage/1000,voltage%1000,distance);
    
    printf(str);
    UART_OutString(str);
  }
  while(time < 200);       // change this to mean 2 seconds
  eFile_EndRedirectToFile();
  ST7735_Message(0,1,"IR0 (mm) =",distance);
  UART_OutString("done.\n\r-->");
  Name[5] = (Name[5]+1)&0xF7; // 0 to 7
  Running = 0;             // robot no longer running
  OS_Kill();
}

//************SW1Push*************
// Called when SW1 Button pushed
// background threads execute once and return
void SW1Push(void){
  if(Running==0){
    Running = 1;  // prevents you from starting two robot threads
    NumCreated += OS_AddThread(&Robot,128,0);  // start a 2 second run
  }
}
//************SW2Push*************
// Called when SW2 Button pushed
// background threads execute once and return
void SW2Push(void){
}



//******** Producer ***************
// The Producer in this lab will be called from your ADC ISR
// A timer runs at 1 kHz, started by your ADC_Collect
// The timer triggers the ADC, creating the 1 kHz sampling
// Your ADC ISR runs when ADC data is ready
// Your ADC ISR calls this function with a 10-bit sample
// sends data to the Robot, runs periodically at 1 kHz
// inputs:  none
// outputs: none
void Producer(unsigned long data){
  if(Running){
    if(OS_Fifo_Put(data)){     // send to Robot
      NumSamples++;
    } else{
      DataLost++;
    }
  }
}

//******** IdleTask  ***************
// foreground thread, runs when no other work needed
// never blocks, never sleeps, never dies
// inputs:  none
// outputs: none
unsigned long Idlecount=0;
void IdleTask(void){
  while(1) {
    Idlecount++;        // debugging
  }
}

char string[30];
char s[30];
//******** Interpreter **************
// your intepreter from Lab 4
// foreground thread, accepts input from UART port, outputs to UART port
// inputs:  none
// outputs: none
void Interpreter(void) {
// add the following commands, remove commands that do not make sense anymore
// 1) format
// 2) directory
// 3) print file
// 4) delete file
// execute   eFile_Init();  after periodic interrupts have started

//eFile_Format();  
  UART_Init();
  UART_OutChar(CR);
  UART_OutChar(LF);
  if(eFile_Init()){UART_OutString("Fail to initialize FS");}
  UART_OutChar(CR);
  UART_OutChar(LF);
  UART_OutString("***** FILE SYSTEM COMMANDS *****");
  UART_OutChar(CR);
  UART_OutChar(LF);
  UART_OutString("dir     list directory");
  UART_OutChar(CR);
  UART_OutChar(LF);
  UART_OutString("pf      print file");
  UART_OutChar(CR);
  UART_OutChar(LF);
  UART_OutString("df      delete file");
  UART_OutChar(CR);
  UART_OutChar(LF);
  UART_OutString("format  format disk");
  UART_OutChar(CR);
  UART_OutChar(LF);
  UART_OutString("help    this list of commands");
while(1){
  UART_OutChar('-');
  UART_OutChar('-');
  UART_OutChar('>');
  UART_InString(string,29);
 
  if (*string=='f'&&*(string+1)=='o'&&*(string+2)=='r'&&*(string+3)=='m'&&*(string+4)=='a'&&*(string+5)=='t') {
    UART_OutChar(CR);
    UART_OutChar(LF);
    if (eFile_Format()) UART_OutString("Formatting unsuccessful");
    else UART_OutString("Disk Formatted");
    
  }
  else if (*string=='d'&&*(string+1)=='i'&&*(string+2)=='r') {
    UART_OutChar(CR);
    UART_OutChar(LF);
    // list directory
    if (eFile_Directory(&UART_OutChar)) UART_OutString("Error reading directory");
  }
  else if (*string=='p'&&*(string+1)=='f') {
    UART_OutChar(CR);
    UART_OutChar(LF);
    // print file
    strncpy(s, string,29);
    char* token = strtok(s, " ");
    token = strtok(NULL," ");
    char data;
    int err=eFile_ROpen(token);
    eFile_Call(err);
    if(err==0){// file or disk does not exist
      do{
        if(eFile_ReadNext(&data)) {
          eFile_RClose();
          break; // done
        }
        UART_OutChar(data);
      }
      while(1);
      
    }else{
      //UART_OutString("Error 404: File does not exist.");
    }
    

    
  }
  else if (*string=='d'&&*(string+1)=='f') {
    UART_OutChar(CR);
    UART_OutChar(LF);
    // delete file
    strncpy(s, string,29);
    char* token = strtok(s, " ");
    token = strtok(NULL," ");
    if(eFile_Delete(token)){UART_OutString("Error 404: File does not exist.");}
    
  }
  
  else if (*string=='h'&&*(string+1)=='e'&&*(string+2)=='l'&&*(string+3)=='p') {
  UART_OutChar(CR);
  UART_OutChar(LF);
  UART_OutString("***** FILE SYSTEM COMMANDS *****");
  UART_OutChar(CR);
  UART_OutChar(LF);
  UART_OutString("dir     list directory");
  UART_OutChar(CR);
  UART_OutChar(LF);
  UART_OutString("pf      print file");
  UART_OutChar(CR);
  UART_OutChar(LF);
  UART_OutString("df      delete file");
  UART_OutChar(CR);
  UART_OutChar(LF);
  UART_OutString("format  format disk");
  UART_OutChar(CR);
  UART_OutChar(LF);
  UART_OutString("help    this list of commands");
  }


  UART_OutChar(CR);
  UART_OutChar(LF);

  for(int i=0;i<30;i++){
      string[i]=0;
    }
  }
}

//*******************lab 4 main **********
int mainF(void){        // lab 4 real main
  OS_Init();           // initialize, disable interrupts
  Running = 0;         // robot not running
  DataLost = 0;        // lost data between producer and consumer
  NumSamples = 0;
  PortD_Init();  // user debugging profile
//********initialize communication channels
  OS_Fifo_Init(256);
  ADC_Collect(0, 50, &Producer,50*200); // start ADC sampling, channel 0, PE3, 50 Hz
  ADC_Init(3);  // sequencer 3, channel 3, PE0, sampling in DAS()
  OS_AddPeriodicThread(&DAS,10*TIME_1MS,1); // 100Hz real time sampling of PE0

//*******attach background tasks***********
  OS_AddSW1Task(&SW1Push,2);    // PF4, SW1
  OS_AddSW2Task(&SW2Push,3);   // PF0
  OS_AddPeriodicThread(&disk_timerproc,10*TIME_1MS,5);

  NumCreated = 0 ;
  
// create initial foreground threads
  NumCreated += OS_AddThread(&Interpreter,128,2);
  NumCreated += OS_AddThread(&DSP,128,1);
  NumCreated += OS_AddThread(&IdleTask,128,5);  // runs when nothing useful to do

  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;             // this never executes
}

//+++++++++++++++++++++++++DEBUGGING CODE++++++++++++++++++++++++
// ONCE YOUR RTOS WORKS YOU CAN COMMENT OUT THE REMAINING CODE
//
//*****************test project 0*************************
// This is the simplest configuration,
// Just see if you can import your OS
// no UART interrupts
// no SYSTICK interrupts
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores
unsigned long Count1;   // number of times thread1 loops
unsigned long Count2;   // number of times thread2 loops
unsigned long Count3;   // number of times thread3 loops
unsigned long Count4;   // number of times thread4 loops
unsigned long Count5;   // number of times thread5 loops
void Thread1(void){
  Count1 = 0;
  for(;;){
    PD0 ^= 0x01;       // heartbeat
    Count1++;
  }
}
void Thread2(void){
  Count2 = 0;
  for(;;){
    PD1 ^= 0x02;       // heartbeat
    Count2++;
  }
}
void Thread3(void){
  Count3 = 0;
  for(;;){
    PD2 ^= 0x04;       // heartbeat
    Count3++;
  }
}

int main0(void){  // Testmain0
  OS_Init();          // initialize, disable interrupts
  PortD_Init();       // profile user threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1,128,1);
  NumCreated += OS_AddThread(&Thread2,128,2);
  NumCreated += OS_AddThread(&Thread3,128,3);
  // Count1 Count2 Count3 should be equal or off by one at all times
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}
//*****************test project 1*************************
unsigned char buffer[512];
unsigned char buffer2[512];
#define MAXBLOCKS 100
void diskError(char* errtype, unsigned long n){
  UART_OutString(errtype);
  sprintf(str," disk error %u",n);
  UART_OutString(str);
  OS_Kill();
}
void TestDisk(void){  DSTATUS result;  unsigned short block;  int i; unsigned long n;
  // simple test of eDisk
  ST7735_OutString(0, 1, "eDisk test      ", ST7735_WHITE);
  UART_OutString("\n\rEE445M/EE380L, Lab 4 eDisk test\n\r");
  
  result = eDisk_Init(0);  // initialize disk
  if(result) diskError("eDisk_Init",result);
  UART_OutString("Writing blocks\n\r");
  n = 1;    // seed
  for(block = 0; block < MAXBLOCKS; block++){
    for(i=0;i<512;i++){
      n = (16807*n)%2147483647; // pseudo random sequence
      buffer[i] = 0xFF&n;
    }
    PB3 = 0x08;     // PD3 high for 100 block writes
    if(eDisk_WriteBlock(buffer,block))diskError("eDisk_WriteBlock",block); // save to disk
    PB3 = 0x00;
  }
  UART_OutString("Reading blocks\n\r");
  n = 1;  // reseed, start over to get the same sequence
  for(block = 0; block < MAXBLOCKS; block++){
    PB2 = 0x04;     // PF2 high for one block read
    if(eDisk_ReadBlock(buffer2,block))diskError("eDisk_ReadBlock",block); // read from disk
    PB2 = 0x00;
    for(i=0;i<512;i++){
      n = (16807*n)%2147483647; // pseudo random sequence
      if(buffer2[i] != (0xFF&n)){
        sprintf(str,"Read data not correct, block=%u, i=%u, expected %u, read %u\n\r",block,i,(0xFF&n),buffer2[i]);
        UART_OutString(str);
        OS_Kill();
      }
    }
  }
  
  sprintf(str,"Successful test of %u blocks\n\r",MAXBLOCKS);
  UART_OutString(str);
  ST7735_OutString(0, 1, "eDisk successful", ST7735_YELLOW);
  Running=0; // launch again
  OS_Kill();
}
void RunTest(void){
  NumCreated += OS_AddThread(&TestDisk,128,1);
}
//************SW1Push*************
// Called when SW1 Button pushed
// background threads execute once and return
void CreateTest(void);
void ReadTest(void);
void SW1Push1(void){
  if(Running==0){
    Running = 1;  // prevents you from starting two test threads
    //NumCreated += OS_AddThread(&TestDisk,128,1);  // test eDisk
		NumCreated += OS_AddThread(&CreateTest,128,1);
  }
}
void SW2Push2(void){
	if(Running==0){
		Running = 1;
		NumCreated += OS_AddThread(&ReadTest,128,1);
	}
}
//******************* test main1 **********
// SYSTICK interrupts, period established by OS_Launch
// Timer interrupts, period established by first call to OS_AddPeriodicThread
int main(void){   // testmain1
  OS_Init();           // initialize, disable interrupts
  //PortD_Init();
  PortB_Init();
  //*******attach background tasks***********
  OS_AddPeriodicThread(&disk_timerproc,10*TIME_1MS,0);   // time out routines for disk
  
  NumCreated = 0 ;
  Running = 0;
// create initial foreground threads
  NumCreated += OS_AddThread(&IdleTask,128,3);
  //NumCreated += OS_AddThread(&TestDisk,128,1);
  OS_AddSW1Task(&SW1Push1,2);    // PF4, SW1 , Creates file5.txt
	OS_AddSW2Task(&SW2Push2,2);   //	SW2, Reads file5.txt
  OS_Launch(10*TIME_1MS); // doesn't return, interrupts enabled in here
  return 0;               // this never executes
}

//*****************test project 2*************************

void TestFile(void){   int i; char data;
  UART_OutString("\n\rEE445M/EE380L, Lab 4 eFile test\n\r");
  ST7735_OutString(0, 1, "eFile test      ", ST7735_WHITE);
  // simple test of eFile
  if(eFile_Init())              diskError("eFile_Init",0);
  if(eFile_Format())            diskError("eFile_Format",0);
  eFile_Directory(&UART_OutChar);
  if(eFile_Create("file1"))     diskError("eFile_Create",0);
  if(eFile_WOpen("file1"))      diskError("eFile_WOpen",0);
  for(i=0;i<1000;i++){
    if(eFile_Write('a'+i%26))   diskError("eFile_Write",i);
    if(i%52==51){
      if(eFile_Write('\n'))     diskError("eFile_Write",i);
      if(eFile_Write('\r'))     diskError("eFile_Write",i);
    }
  }
  if(eFile_WClose())            diskError("eFile_WClose",0);
  eFile_Directory(&UART_OutChar);
  if(eFile_ROpen("file1"))      diskError("eFile_ROpen",0);
  for(i=0;i<1000;i++){
    if(eFile_ReadNext(&data))   diskError("eFile_ReadNext",i);
    UART_OutChar(data);
  }
  if(eFile_Delete("file1"))     diskError("eFile_Delete",0);
  eFile_Directory(&UART_OutChar);
  if(eFile_Close())             diskError("eFile_Close",0);
  eFile_Directory(&UART_OutChar);
  UART_OutString("Successful test of creating a file\n\r");
  ST7735_OutString(0, 1, "eFile successful", ST7735_YELLOW);
  Running=0; // launch again
  OS_Kill();
}


void myTestFile(void){
  int i; 
  char data;
  UART_OutString("\n\rEE445M/EE380L, Lab 4 eFile test\n\r");
  ST7735_OutString(0, 1, "eFile test      ", ST7735_WHITE);
  // simple test of eFile
  UART_OutString("Init\n\r");
  if(eFile_Init())              diskError("eFile_Init",0);
  //UART_OutString("Formating\n\r");
  //if(eFile_Format())            diskError("eFile_Format",0);
  UART_OutString("Directory\n\r");
  eFile_Directory(&UART_OutChar);
  UART_OutString("Create\n\r");
  if(eFile_Create("file1"))     diskError("eFile_Create",0);
  if(eFile_Create("file2"))     diskError("eFile_Create",0);

  UART_OutString("Directory\n\r");
  eFile_Directory(&UART_OutChar);
  if(eFile_WOpen("file1"))      diskError("eFile_WOpen",0);
  for(i=0;i<1000;i++){
    if(eFile_Write('a'+i%26))   diskError("eFile_Write",i);
    if(i%52==51){
      if(eFile_Write('\n'))     diskError("eFile_Write",i);
      if(eFile_Write('\r'))     diskError("eFile_Write",i);
    }
  }
  if(eFile_WClose())            diskError("eFile_WClose",0);
  eFile_Directory(&UART_OutChar);
  if(eFile_ROpen("file1"))      diskError("eFile_ROpen",0);
  for(i=0;i<1000;i++){
    if(eFile_ReadNext(&data))   diskError("eFile_ReadNext",i);
    UART_OutChar((data));
  }
  
  OS_Kill();
}

void myTestFile2(void){
  int i; 
  char data;
  UART_OutString("\n\rmyTestFile2\n\r");
  ST7735_OutString(0, 1, "eFile test      ", ST7735_WHITE);
  // simple test of eFile
  UART_OutString("Init\n\r");
  eFile_Call(eFile_Init());
  //UART_OutString("Formating\n\r");
  //eFile_Call(eFile_Format());
  UART_OutString("Directory\n\r");
  eFile_Directory(&UART_OutChar);
  UART_OutString("Create\n\r");
  eFile_Call(eFile_Create("file1"));
  eFile_Call(eFile_Create("file2"));
  eFile_Call(eFile_Create("gile3"));
  eFile_Call(eFile_Create("gile4"));
  eFile_Call(eFile_Create("gile5"));
  eFile_Call(eFile_Create("gile6"));
  eFile_Call(eFile_Create("gile7"));
  eFile_Call(eFile_Create("gile8"));
  eFile_Call(eFile_Create("gile9"));
  eFile_Call(eFile_Create("gile10"));
  eFile_Call(eFile_Create("gile11"));
  eFile_Call(eFile_Create("gile12"));

  // Create a file that already exists
  eFile_Call(eFile_Create("file2"));
  eFile_Call(eFile_Create("file3"));
  UART_OutString("Directory\n\r");
  eFile_Directory(&UART_OutChar);
  // Close a file that is not open
  UART_OutString("Trying to close a file that is not open\n\r");
  eFile_Call(eFile_WClose());
  // Try to open a file that does not exist
  UART_OutString("Try to open file5\n\r");
  eFile_Call(eFile_WOpen("file5"));
  UART_OutString("Try to open file1\n\r");
  eFile_Call(eFile_WOpen("file1"));
  // Open a file while one is already open
  UART_OutString("Try to open file2\n\r");
  eFile_Call(eFile_WOpen("file2"));
  UART_OutString("Writing to file1n\r");
  for(i=0;i<1000;i++){
    eFile_Call(eFile_Write('a'+i%26));
    if(i%52==51){
      eFile_Call(eFile_Write('\n'));
      eFile_Call(eFile_Write('\r'));
    }
  }
  eFile_Call(eFile_WClose());
  
  eFile_Call(eFile_WOpen("file2"));
  UART_OutString("Writing to file2");
  UART_OutChar(CR);
  UART_OutChar(LF);
  for(i=0;i<1000;i++){
    eFile_Call(eFile_Write('z'-i%26));
    if(i%52==51){
      eFile_Call(eFile_Write('\n'));
      eFile_Call(eFile_Write('\r'));
    }
  }
  eFile_Call(eFile_WClose());
  
  eFile_Call(eFile_WOpen("file3"));
  UART_OutString("Writing to file3\n\r");
  for(i=0;i<1000;i++){
    eFile_Call(eFile_Write('a'+(2*i)%26));
    if(i%52==51){
      eFile_Call(eFile_Write('\n'));
      eFile_Call(eFile_Write('\r'));
    }
  }
  eFile_Call(eFile_WClose());
  
  
  eFile_Directory(&UART_OutChar);
  UART_OutString("Opening file1\n\r");
  eFile_Call(eFile_ROpen("file1"));
  for(i=0;i<1000;i++){
    eFile_Call(eFile_ReadNext(&data));
    UART_OutChar((data));
  }
  eFile_Call(eFile_RClose());
  
  UART_OutString("Opening file2\n\r");
  eFile_Call(eFile_ROpen("file2"));
  for(i=0;i<1000;i++){
    eFile_Call(eFile_ReadNext(&data));
    UART_OutChar((data));
  }
  eFile_Call(eFile_RClose());
  
  UART_OutString("Opening file3\n\r");
  eFile_Call(eFile_ROpen("file3"));
  for(i=0;i<1000;i++){
    eFile_Call(eFile_ReadNext(&data));
    UART_OutChar((data));
  }
  eFile_Call(eFile_RClose());

  
  OS_Kill();
}

//************FAT32****************
// A test file for FAT32 ff.c and ff.h
//*********************************

static FATFS g_sFatFs;
FIL Handle,Handle2;
void ReadTest(void){   int i;  UINT n;
  FRESULT fresult;
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
    fresult = f_read(&Handle,&buffer[i],1,&n);
    if(fresult) diskError("f_read",0);
    i++;
  } while((n==1)&&(i<511));
  buffer[i] = 0; // null termination
	UART_OutString("Done Reading into Buffer\n\r");
  fresult = f_close(&Handle);
  if(fresult) diskError("f_close",0);
	UART_OutString("File Closed\n\r");
	fresult = f_mount(0,0); //Unmount
	if(fresult) diskError("f_unmount",0);
	UART_OutString("Unmount Fs\n\r");
	Running=0; // launch again
	UART_OutString("Done ReadTest Task\n\r");
  OS_Kill();
}


void CreateTest(void){ int i;  UINT n; char letter;
  FRESULT fresult;
	
	fresult = f_mount(0, &g_sFatFs);
	UART_OutString("Mounted FS\n\r");

	if(fresult) diskError("f_mount",0);
//	fresult = f_mkfs(0,0,4096);
//if(fresult) diskError("f_mkfs",0);
// Creates a new file named file5.txt makes it empty
  fresult = f_open(&Handle2,"file5.txt", FA_CREATE_ALWAYS|FA_WRITE);
  if(fresult) diskError("f_open",0);
	UART_OutString("Opened file5.txt\n\r");
	UART_OutString("Writing to file5.txt..\n\r");
  i=0;
  do{
    if((i%10) == 9) letter = 13;
    else if ((i%10) == 10) letter = 10;
    else letter = (i%10)+'a';
    fresult = f_write(&Handle2,&letter,1,&n);

    if(fresult) diskError("f_write",0);
    i++;
  } while((n==1)&&(i<511));
	UART_OutString("Done Writing to file5.txt\n\r");
  fresult = f_close(&Handle2);
  if(fresult) diskError("f_close",0);
	UART_OutString("Closed file5.txt\n\r");
	fresult = f_mount(0,0); //Unmount
	if(fresult) diskError("f_unmount",0);
	UART_OutString("Unmount Fs\n\r");
	Running=0; // launch again
	UART_OutString("Done CreateTest Task\n\r");
  OS_Kill();
}


//************SW1Push2*************
// Called when SW1 Button pushed
// background threads execute once and return
void SW1Push2(void){
  if(Running==0){
    Running = 1;  // prevents you from starting two test threads
    NumCreated += OS_AddThread(&TestFile,128,1);  // test eFile
  }
}
//******************* test main2 **********
// SYSTICK interrupts, period established by OS_Launch
// Timer interrupts, period established by first call to OS_AddPeriodicThread
  int main2(void){
  OS_Init();           // initialize, disable interrupts
  PortD_Init();
  Running = 1;
  printf("PRINTF PRINTF");
//*******attach background tasks***********
  OS_AddPeriodicThread(&disk_timerproc,10*TIME_1MS,0);   // time out routines for disk
  OS_AddSW1Task(&SW1Push1,2);    // PF4, SW1
  OS_AddSW2Task(&SW1Push2,2);    // PF0, SW2
  NumCreated = 0 ;
// create initial foreground threads
  NumCreated += OS_AddThread(&IdleTask,128,3);
  NumCreated += OS_AddThread(&myTestFile2,128,1);
  NumCreated += OS_AddThread(&Interpreter,128,2);
  OS_Launch(10*TIME_1MS); // doesn't return, interrupts enabled in here
  return 0;               // this never executes
}

