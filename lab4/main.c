// Jonathan Valvano
// SDC test programs
// May 2, 2011
// TestDisk tests low level eDisk (will destroy format)
// to test the FAT file syste,
//   1) format a SDC on another system
//   2) run CreateTest to create a new file
//   3) run ReadTest to read that file back
#include <hidef.h>      /* common defines and macros */
#include "derivative.h" /* derivative-specific definitions */

#include "pll.h"
#include "edisk.h"
#include "ff.h"
unsigned char buffer[512];
void error(void){unsigned short startTime; 
  for(;;){
    PTP ^= 0x80;         // TCNT = 5.333us
    startTime = TCNT;    // 4 Hz
    while((TCNT-startTime) <= 46875){}  
  }
}
#define MAXBLOCKS 1000

void TestDisk(void){  DSTATUS result;  unsigned short block;  int i; unsigned long n;
  // simple test of eDisk
  DDRP |= 0x80;
  DDRT = 0xFF;
  PTT = 0;
  result = eDisk_Init(0);  // initialize disk
  if(result) { 
    PTT = 1;
    error();
    for(;;){
    };
  }
  n = 1;    // seed
  for(block = 0; block < MAXBLOCKS; block++){
    for(i=0;i<512;i++){
      n = (16807*n)%2147483647; // pseudo random sequence
      buffer[i] = 0xFF&n;        
    }
    PTT = 0x08;     // PT3 high for 100 block writes
    if(eDisk_WriteBlock(buffer,block)){ 
      PTT = 2;
      error();
      for(;;){
      };
    }
    PTT = 0x00;     
  }  
  n = 1;  // reseed, start over to get the same sequence
  for(block = 0; block < MAXBLOCKS; block++){
    PTT = 0x04;     // PT2 high for one block read
    if(eDisk_ReadBlock(buffer,block)){ 
      PTT = 3;
      error();
      for(;;){
      };
    }
    PTT = 0x00;
    for(i=0;i<512;i++){
      n = (16807*n)%2147483647; // pseudo random sequence
      if(buffer[i] != (0xFF&n)){ 
        PTT = 0x10;
        error();
        for(;;){
        };
      }     
    }
  }  
}

static FATFS g_sFatFs;
FIL Handle,Handle2;
void ReadTest(void){   int i;  UINT n;
  FRESULT fresult;
// assumes there is a short text file named file5.txt
  fresult = f_open(&Handle,"file5.txt", FA_READ);
  if(fresult) error();
  i=0;
  do{
    fresult = f_read(&Handle,&buffer[i],1,&n);
    if(fresult) error();
    i++;
  } while((n==1)&&(i<511));
  buffer[i] = 0; // null termination
  fresult = f_close(&Handle);
  if(fresult) error();
}
void CreateTest(void){ int i;  UINT n; char letter;
  FRESULT fresult;
// Creates a new file named file5.txt makes it empty
  fresult = f_open(&Handle2,"file5.txt", FA_CREATE_ALWAYS|FA_WRITE);
  if(fresult) error();
  i=0;
  do{
    if((i%10) == 9) letter = 13;
    else if ((i%10) == 10) letter = 10;
    else letter = (i%10)+'a';
    fresult = f_write(&Handle2,&letter,1,&n);
    if(fresult) error();
    i++;
  } while((n==1)&&(i<511));
  fresult = f_close(&Handle2);
  if(fresult) error();
}

void main(void) { 
  FRESULT fresult;
  PLL_Init();
	EnableInterrupts;
	DDRP |= 0x80;   // PP7 flashes on error
  PTP &= ~0x80;   // PP7 on if ok
  fresult = f_mount(0, &g_sFatFs);
//  TestDisk();   // tests edisk functions, will destroy formating
  CreateTest();   // tests creating a new file for writing
//  ReadTest();   // tests reading a file
  PTP |= 0x80;    // PP7 on means ok
  for(;;) {
  } 
}
