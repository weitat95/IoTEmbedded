// Name: OS.c
// Authors: Corentin Dugue & Wei Tat Lee
// Creation: 1/23/2017
// Description: Provide functions for the OS
// Lab 5
// TA: Kishore Punniyamurthy
// Last modified: 4/3/2017
// Hardware connections: none specific

#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "os.h"
#include "PLL.h"
#include "UART.h"
#include "heap.h"
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void StartOS(void);           // Start OS
//uint32_t DEBUG,DEBUG2,DEBUG3;

void   OS_NewData(void *data);           // Set R9 to data address
void*  OS_GetR9(void);
void InitTimer3A(uint32_t period);
//Process Implementation
//#define PROCESS

//Round Robin (RR) Scheduler or Priority Scheduler
//#define RR
#define PRIORITY

//Use Blocking semaphore
#define BLOCKING
//#define DEBUG 
#define INTCTRL         (*((volatile uint32_t *)0xE000ED04)) // Interrupt Control and State register
#define NVIC_EN1_INT35  0x00000008
#define PF3             (*((volatile uint32_t *)0x40025020))
#define PF0             (*((volatile uint32_t *)0x40025004))  
#define PF4             (*((volatile uint32_t *)0x40025040))
#define PF2             (*((volatile uint32_t *)0x40025010))
#define PB5             (*((volatile uint32_t *)0x40005080))
#define PB7             (*((volatile uint32_t *)0x40005200))  


#define NUMTHREADS  10        // maximum number of threads
#define NUMPROCESS  5
#define STACKSIZE   256      // number of 32-bit words in stack
long MaxJitter1,MaxJitter2;
long sr;
unsigned long t1,t2,tTotal,tMax,dt1,dt2;


#define JITTERSIZE 64

#define MDisableInterrupts(){DisableInterrupts();}
#define MEnableInterrupts(){EnableInterrupts();}
#define MEnterCritical(){t2=OS_Time(); sr=StartCritical();}
#define MExitCritical(){EndCritical(sr);}
unsigned long JitterSize=JITTERSIZE;
unsigned long JitterHistogram1[JITTERSIZE]={0,};
unsigned long JitterHistogram2[JITTERSIZE]={0,};

struct tcb{
  int32_t *sp;       // pointer to stack (valid for threads not running)
  struct tcb *next;  // linked-list pointer
  struct tcb *prev;
  int32_t id;
  uint32_t sleep;
  uint16_t priority;
  Sema4Type * blocked;
  uint16_t killed;
  int16_t pid;
};

typedef struct tcb tcbType;
static tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
tcbType *NextPt; // ADDED
int32_t Stacks[NUMTHREADS][STACKSIZE];
uint32_t counter_os,counter2_os;
unsigned long OSNumCreated=0;
uint32_t threadActive=0;

#ifdef PROCESS
struct pcb{
  struct pcb *nextProcess;
  int16_t pid;
  void *data;
  void *code;
  void(*entry)(void);
  uint32_t priority;
  uint32_t threadAlive;
  uint32_t threadSleeping;
};

typedef struct pcb pcbType;
static pcbType pcbs[NUMPROCESS];
pcbType *RunningProc;
uint32_t processActive=0;
uint32_t totalProcess=0;
#endif

int32_t StartCritical(void);
void EndCritical(int32_t primask);
void (*PeriodicTask)(void);   // user function
void (*PeriodicTask2)(void);   // user function
void (*SW1Task)(void);
void (*SW2Task)(void);



void SetInitialStack(int i){
  tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // thread stack pointer
  Stacks[i][STACKSIZE-1] = 0x01000000;   // thumb bit
  Stacks[i][STACKSIZE-3] = 0x14141414;   // R14
  Stacks[i][STACKSIZE-4] = 0x12121212;   // R12
  Stacks[i][STACKSIZE-5] = 0x03030303;   // R3
  Stacks[i][STACKSIZE-6] = 0x02020202;   // R2
  Stacks[i][STACKSIZE-7] = 0x01010101;   // R1
  Stacks[i][STACKSIZE-8] = 0x00000000;   // R0
  Stacks[i][STACKSIZE-9] = 0x11111111;   // R11
  Stacks[i][STACKSIZE-10] = 0x10101010;  // R10
  Stacks[i][STACKSIZE-11] = 0x09090909;  // R9
  Stacks[i][STACKSIZE-12] = 0x08080808;  // R8
  Stacks[i][STACKSIZE-13] = 0x07070707;  // R7
  Stacks[i][STACKSIZE-14] = 0x06060606;  // R6
  Stacks[i][STACKSIZE-15] = 0x05050505;  // R5
  Stacks[i][STACKSIZE-16] = 0x04040404;  // R4
}

int findFreeTCB(void){
  for(int i=0;i<NUMTHREADS;i++){
    if(tcbs[i].id==0){
      return i;
    }
  }
  return -1;
}
//******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
int OS_AddThread(void(*task)(void),unsigned long stackSize,unsigned long priority){

  MEnterCritical();
  if(threadActive>=NUMTHREADS){
    MExitCritical();
    return 0;
  }
  int freeTCBIndex=findFreeTCB();
  if(freeTCBIndex==-1) {    
    MExitCritical();
    return 0;
  }
  //shouldn't kill thread 0;
  // Link previous to new tcb
  if(OSNumCreated>0){
    tcbs[freeTCBIndex].prev= &tcbs[freeTCBIndex-1];
    tcbs[freeTCBIndex-1].next = &tcbs[freeTCBIndex];
  }
  // Link next to new tcb
  if(freeTCBIndex==threadActive){
    tcbs[freeTCBIndex].next = &tcbs[0]; // The latest thread pointing to thread first thread created (thread 0)
    tcbs[0].prev = &tcbs[freeTCBIndex];
  }else{
      tcbType *validTCB=tcbs[freeTCBIndex].next;
      while(validTCB->id==0){
        validTCB=(validTCB->next);
      }
      tcbs[freeTCBIndex].next = validTCB; // The latest thread pointing to thread first thread created (thread 0)
      validTCB->prev = &tcbs[freeTCBIndex];
  }
  SetInitialStack(freeTCBIndex); Stacks[freeTCBIndex][STACKSIZE-2] = (int32_t)(task); // PC
  if(OSNumCreated==0){
    RunPt = &tcbs[0];
  }
  OSNumCreated++;
  threadActive++;
  tcbs[freeTCBIndex].id=OSNumCreated;
  tcbs[freeTCBIndex].priority=priority;
  tcbs[freeTCBIndex].blocked=0;
  tcbs[freeTCBIndex].sleep=0;
  #ifdef PROCESS
  tcbs[freeTCBIndex].pid=RunningProc->pid;
  RunningProc->threadAlive++;
  #endif
  MExitCritical();
  return 1;               // successful
}

// ***************** OS_AddPeriodicThread ****************
// Activate TIMER4 interrupts to run user task periodically, increments counter periodically 
// Inputs:  task is a pointer to a user function
//          period given in system time units (12.5ns)
//          priority (0-7) 0:highest
// Outputs: none

uint8_t one=0;
uint32_t period1,period2;
int OS_AddPeriodicThread(void(*task)(void),
                           uint32_t period, uint32_t priority){
  counter_os=0;
  counter2_os=0;
  if (one==0){
  SYSCTL_RCGCTIMER_R |= 0x10;   // 0) activate TIMER4
  PeriodicTask = task;          // user function
  TIMER4_CTL_R = 0x00000000;    // 1) disable TIMER1A during setup
  TIMER4_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER4_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER4_TAILR_R = period-1;    // 4) reload value
  period1=period;
  TIMER4_TAPR_R = 0;            // 5) bus clock resolution
  TIMER4_ICR_R = 0x00000001;    // 6) clear TIMER1A timeout flag
  TIMER4_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  NVIC_PRI17_R = (NVIC_PRI17_R&0xFF00FFFF)+(priority<<21); // 8) priority set 
// interrupts enabled in the main program after all devices initialized
// vector number 70
  NVIC_EN2_R = 1<<6;           // 9) enable IRQ 70-64 in NVIC
  TIMER4_CTL_R = 0x00000001;    // 10) enable TIMER4A
  one++;
  return 1;
  }
  
  else if (one==1) {
    SYSCTL_RCGCTIMER_R |= 0x20;   // 0) activate TIMER5
    PeriodicTask2 = task;          // user function
    TIMER5_CTL_R = 0x00000000;    // 1) disable TIMER1A during setup
    TIMER5_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
    TIMER5_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
    TIMER5_TAILR_R = period-1;    // 4) reload value
    period2=period;
    TIMER5_TAPR_R = 0;            // 5) bus clock resolution
    TIMER5_ICR_R = 0x00000001;    // 6) clear TIMER1A timeout flag
    TIMER5_IMR_R = 0x00000001;    // 7) arm timeout interrupt
    NVIC_PRI17_R = (NVIC_PRI23_R&0xFFFFFF00)+(priority<<5); // 8) priority set 
  // interrupts enabled in the main program after all devices initialized
  // vector number 92
    NVIC_EN2_R = 1<<28;           // 9) enable IRQ 70-64 in NVIC
    TIMER5_CTL_R = 0x00000001;    // 10) enable TIMER4A
    one++;
    return 1;
  }
  
  else {
    return 0;
  }
}
// ***************** OS_ClearPeriodicTime ****************
// Reset 32-bit global Counter to zero;(needs to be used with OS_AddPeriodicThread) 
// Inputs:  none
// Outputs: none                        
void OS_ClearPeriodicTime(void){
  counter_os=0;
}
// ***************** OS_ReadPeriodicTime ****************
// returns the global counter(needs to be used with OS_AddPeriodicThread) 
// Inputs:  none
// Outputs: uint32_t counter_os:the current counter 
uint32_t OS_ReadPeriodicTime(void){
  return counter_os*TIMER4_TAILR_R;
}

// ******** OS_ClearMsTime ************
// sets the system time to zero (from Lab 1)
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void){
  counter2_os=0;
}

// ******** OS_MsTime ************
// reads the current time in msec (from Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// It is ok to make the resolution to match the first call to OS_AddPeriodicThread
unsigned long OS_MsTime(void){
  //DEBUG=TIMER4_TAILR_R;
  //DEBUG2=(counter2_os*TIMER4_TAILR_R)/TIME_1MS;
  return counter2_os;
}


// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 

unsigned long OS_Time(void){
  //  DEBUG=TIMER4_TAILR_R;
  //  DEBUG3=(counter_os*TIMER4_TAILR_R);
  //return (counter_os*TIMER4_TAILR_R);
  return WTIMER5_TAR_R;
}


// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
unsigned long OS_TimeDifference(unsigned long start, unsigned long stop){
  return start-stop;
}
unsigned long LastTime1,LastTime2;
unsigned long thisTime1,thisTime2;
unsigned long flag1=0;
unsigned long flag2=0;
long jitter1,jitter2;
void Timer4A_Handler(void){
  //PF2=0x04; 
  TIMER4_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER4A timeout
  (*PeriodicTask)();                // execute user task

//  #ifdef JITTER
  thisTime1=OS_Time();
  if(flag1>1){
      unsigned long diff = OS_TimeDifference(LastTime1,thisTime1);
      if(diff>period1){
        jitter1 = (diff-period1+4)/8;  // in 0.1 usec
      }else{
        jitter1 = (period1-diff+4)/8;  // in 0.1 usec
      }
      if(jitter1>100){
        jitter1=0;
      }
      if(jitter1 > MaxJitter1){
        MaxJitter1 = jitter1; // in usec
      }       // jitter1 should be 0
      if(jitter1 >= JitterSize){
        jitter1 = JitterSize-1;
      }
      JitterHistogram1[jitter1]++; 
  }
  LastTime1=thisTime1;
  flag1++;
//  #endif
  counter_os++;
  
  //PF2=0;
}


tcbType * FirstValid;
tcbType * Next2Run;
uint16_t Prior;
void Scheduler(void){
  
  // Round-Robin Scheduler
  #ifdef RR
  
    #ifdef PROCESS
    NextPt=RunPt->next;
    //There is other process other than the main process
    if(processActive>1){
      
      RunningProc=RunningProc->nextProcess;
      //If all the threads for the process is sleeping,skip it
      while(RunningProc->threadSleeping==RunningProc->threadAlive){
        RunningProc=RunningProc->nextProcess;
      }
      //Get one thread with the selected process
      while(NextPt->sleep || NextPt->blocked ||  NextPt->pid!=RunningProc->pid ){
        NextPt=NextPt->next;
      }
    }
    //Original Main process running
    else{
      while(NextPt->sleep || NextPt->blocked ){
        NextPt=NextPt->next;
      }
    }
    #else
    NextPt=RunPt->next;
    while(NextPt->sleep || NextPt->blocked ){   //if sleep>0 ,skip
      NextPt=NextPt->next;
    }
    #endif
  
  #endif
  
  #ifdef PRIORITY
  
  FirstValid=RunPt;
  Prior=9;
  uint16_t LastPrior=RunPt->priority;
  
  if(!RunPt->id){       //If runPt is Killed
    Next2Run=RunPt->next;
    do{
    if(!Next2Run->sleep && !Next2Run->blocked && Next2Run->id ){
      if(Next2Run->priority < Prior){
        Prior=Next2Run->priority;
        NextPt=Next2Run;
      }
    }
    Next2Run=Next2Run->next;
    }while(Next2Run!=RunPt->next);
    
  //RunPt is not Killed  
  }else{
    Next2Run=RunPt->next;
    while(Next2Run!=RunPt){
    if(!Next2Run->sleep && !Next2Run->blocked && Next2Run->id){
      if(Next2Run->priority < Prior){
        Prior=Next2Run->priority;
        NextPt=Next2Run;
      }
    }
    Next2Run=Next2Run->next;
    }
  }
  
  if(LastPrior<Prior && !RunPt->blocked && !RunPt->sleep && RunPt->id){
    NextPt=RunPt;
  }
  #endif
  
  
  
}



void SysTick_Handler(void) {
  MDisableInterrupts();
  struct tcb *firsttcbptr=RunPt->next;
  struct tcb *tcbptr;
  tcbptr=firsttcbptr;
  do{
    
    if(tcbptr->sleep){    //decrement sleep if its > 0
      tcbptr->sleep--;
    }
    tcbptr=tcbptr->next;
  }while(tcbptr!=firsttcbptr);
  
  Scheduler();
  INTCTRL = 0x10000000; // trigger PendSV

  MEnableInterrupts();

}


// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void) {
  NVIC_ST_CURRENT_R=0;
  INTCTRL = 0x04000000; // trigger SysTick
}
void wTimer5(void);

// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: systick, 50 MHz PLL
// input:  none
// output: none
void OS_Init(void){
  //MDisableInterrupts();
  DisableInterrupts();
  PLL_Init(Bus80MHz);         // set processor clock to 50 MHz
  wTimer5();
  InitTimer3A(TIME_1MS);
  UART_Init();
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  
  NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0x00FFFFFF)|0xC0000000; // priority 6 SYSTICK
  NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0xFF00FFFF)|0x00E00000; // priority 7 PENDSV
  
  NVIC_SYS_PRI2_R =(NVIC_SYS_PRI2_R&0x00FFFFFF)|0x00000000; // priority 6 SYSTICK

  for(int i=0;i<NUMTHREADS;i++){
    tcbs[i].id=0;
    tcbs[i].killed=0;
    tcbs[i].blocked=0;
    tcbs[i].priority=9;
  }
}

//******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
// It is ok to limit the range of theTimeSlice to match the 24-bit SysTick
///******** OS_Launch ***************
// start the scheduler, enable interrupts
// Inputs: number of 20ns clock cycles for each time slice
//         (maximum of 24 bits)
// Outputs: none (does not return)
void OS_Launch(uint32_t theTimeSlice){
  //Find the highest priority thread;
  #ifdef PRIORITY
  uint32_t prior=10;
  for(int i=0;i<NUMTHREADS;i++){
    if(prior<tcbs[i].priority){
      prior=tcbs[i].priority;
      RunPt=&tcbs[i];
    }
  }
  #endif
  NVIC_ST_RELOAD_R = theTimeSlice - 1; // reload value
  NVIC_ST_CTRL_R = 0x00000007; // enable, core clock and interrupt arm
  StartOS();                   // start on the first task
}


// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void){
  MDisableInterrupts();
  (RunPt->prev)->next = RunPt->next;
  (RunPt->next)->prev = RunPt->prev;
  RunPt->id=0;
  RunPt->killed++;
  #ifdef PROCESS
  RunningProc->threadAlive--;
  if(RunningProc->threadAlive==0){
    //PB5^=0x20;
    //PB5^=0x20;
    RunningProc->pid=-1;
    //Free Heap
    if(Heap_Free(RunningProc->data)!=HEAP_OK){UART_OutString("Failed Heap");while(1){;}}
    if(Heap_Free(RunningProc->code)!=HEAP_OK){UART_OutString("Failed Heap");while(1){;}}
    RunningProc->pid=-1;
    RunningProc=RunningProc->nextProcess;
    processActive--;
    //PB5^=0x20;
  }
  
  #endif 
  threadActive--;
  MEnableInterrupts();
  OS_Suspend();
  
}

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, long value){
  semaPt->Value=value;
}

		// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(unsigned long sleepTime){
  RunPt->sleep=(sleepTime*TIME_1MS)/NVIC_ST_RELOAD_R;
  OS_Suspend();
}

// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt){
  MDisableInterrupts();
  while(semaPt->Value==0){
    MEnableInterrupts();
    MDisableInterrupts();
  }
  semaPt->Value=0;
  MEnableInterrupts();  
}  

// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt){
  MEnterCritical();
  semaPt->Value=1;
  MExitCritical();
}

// ******** OS_Wait ************ 
// BLOCKING SEMAPHORE
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt) {
  #ifdef BLOCKING
  MDisableInterrupts();
  (semaPt->Value--);
  if ((semaPt->Value)<0) {
    RunPt->blocked=semaPt;
    MEnableInterrupts();
    OS_Suspend();
  }
  MEnableInterrupts();
  #endif
  
  #ifdef SPINLOCK
  MDisableInterrupts();
  while(semaPt->Value<=0){
    MEnableInterrupts();
    MDisableInterrupts();
  }
  semaPt->Value--;
  MEnableInterrupts();
  #endif
}
// ******** OS_Signal ************
// BLOCKING SEMAPHORE
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt){
  #ifdef BLOCKING
  tcbType *pt;
  MDisableInterrupts();
  (semaPt->Value++);
  if((semaPt->Value<=0)){
    pt = RunPt->next;
    while(pt->blocked != semaPt){
      pt=pt->next;
    }
    pt->blocked=0;
  }
  MEnableInterrupts();
  #endif
  
  #ifdef SPINLOCK
  MEnterCritical();
  
  semaPt->Value++;
  MExitCritical();
  #endif
}



//******** OS_AddSW1Task *************** 
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
uint16_t prio_PortF=5;
int OS_AddSW1Task(void(*task)(void), unsigned long priority){
  
  if(priority>5){
    return 0;
  }
  if(priority<prio_PortF){
    prio_PortF=priority;
  }
  SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
  SW1Task=task;
  GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x10;     //     enable digital I/O on PF4 
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFF0FFFF)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x10;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)+(prio_PortF<<21); // (g) priority 1 to 5
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
  return 1;
 
}
//******** OS_DisSw1Task *************** 
// Disable Aperiodic Task for SW1 PF4(activated by OS_AddSW1Task)
// Inputs: none
// Outputs: none
void OS_DisSW1Task(void){
  GPIO_PORTF_IM_R &= ~0x10;      // Disarm Interrupt PF4
}

//******** OS_AddSW2Task *************** 
// add a background task to run whenever the SW2 (PF0) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
// In lab 2, this function can be ignored
// In lab 3, this command will be called will be called 0 or 1 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW2Task(void(*task)(void), unsigned long priority){
    if(priority>5){
    return 0;
  }
  if(priority<prio_PortF){
    prio_PortF=priority;
  }
  SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
                                   // 2a) unlock GPIO Port F Commit Register
  GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
  GPIO_PORTF_CR_R |= (0x01);    // 2b) enable commit for PF4 and PF0
  SW2Task=task;
  GPIO_PORTF_DIR_R &= ~0x01;    // (c) make PF0 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x01;  //     disable alt funct on PF0
  GPIO_PORTF_DEN_R |= 0x01;     //     enable digital I/O on PF0
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFFFF0)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x01;     //     enable weak pull-up on PF0
  GPIO_PORTF_IS_R &= ~0x01;     // (d) PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x01;    //     PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x01;    //     PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x01;      // (e) clear flag0
  GPIO_PORTF_IM_R |= 0x01;      // (f) arm interrupt on PF0 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)+(prio_PortF<<21); // (g) priority 1 to 5
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
  return 1;
}
//******** OS_DisSw2Task *************** 
// Disable Aperiodic Task for SW2 PF0(activated by OS_AddSW2Task)
// Inputs: none
// Outputs: none
void OS_DisSW2Task(void){
  GPIO_PORTF_IM_R &= ~0x01;      // Disarm Interrupt PF0
}

void GPIOPortF_Handler(void){
  #ifdef DEBUG
  PF3^=0x08;
  PF3^=0x08;
  #endif
  if(GPIO_PORTF_RIS_R&0x10){  // poll PF4
    GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag4
    (*SW1Task)();                // execute user task
  }
  if(GPIO_PORTF_RIS_R&0x01){  // poll PF0
    GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag0
    (*SW2Task)();                // execute user task
  }
  #ifdef DEBUG
  PF3^=0x08;
  #endif
}
#define FIFOSIZE 2048
#define FIFOFAIL 0
#define FIFOSUCCESS 1
unsigned long fifoPt[FIFOSIZE];
unsigned long Fifosize;
uint32_t volatile PutI;// put next
uint32_t volatile GetI;// get next
Sema4Type DataAvailable,fifoMutex;
// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
void OS_Fifo_Init(unsigned long size){
  MEnterCritical();
  Fifosize=size;
  PutI=GetI=0;
  DataAvailable.Value=0;
  fifoMutex.Value=1;
  MExitCritical();
}
// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(unsigned long data){
  if((PutI-GetI) & ~(Fifosize-1)){
    return(FIFOFAIL); // Failed, fifo full
  }
  fifoPt[PutI&(Fifosize-1)] = data; // put
  PutI++;  // Success, update
  OS_Signal(&DataAvailable);
  return(FIFOSUCCESS);
}

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
unsigned long OS_Fifo_Get(void){
 
  OS_Wait(&DataAvailable);
  OS_Wait(&fifoMutex);
  unsigned long data=fifoPt[GetI&(Fifosize-1)];
  GetI++;  // Success, update
  OS_Signal(&fifoMutex);  
  return(data);
}

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
long OS_Fifo_Size(void){
   return ((uint32_t)(PutI-GetI));
}

unsigned long mailBoxData;
Sema4Type BoxFree,DataValid;
// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
void OS_MailBox_Init(void){
  OS_InitSemaphore(&BoxFree,1);
  OS_InitSemaphore(&DataValid,0);
  mailBoxData=0;
}

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(unsigned long data){
  OS_Wait(&BoxFree);
  mailBoxData=data;
  OS_Signal(&DataValid);
}

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
unsigned long OS_MailBox_Recv(void){
  unsigned long buffer;
  OS_Wait(&DataValid);
  buffer=mailBoxData;
  OS_Signal(&BoxFree);
  return buffer;
}
//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
unsigned long OS_Id(void){
  return RunPt->id;
}

void wTimer5(void){
  SYSCTL_RCGCWTIMER_R |= 0x20;   // 0) activate TIMER5
  WTIMER5_CTL_R = 0x00000000;    // 1) disable TIMER1A during setup
  WTIMER5_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  WTIMER5_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  WTIMER5_TAILR_R = 0xFFFFFFFF;    // 4) reload value
  WTIMER5_TBILR_R = 0xFFFFFFFF;    // 4) reload value
  WTIMER5_TAPR_R = 0;            // 5) bus clock resolution
  WTIMER5_CTL_R = 0x00000001;    // 10) enable TIMER4A
}

void Jitter(void){
  unsigned long tTotal_=tTotal;
  UART_OutString("Jitter 1= ");
  UART_OutUDec(MaxJitter1);
  UART_OutString("\n\rJitter 2= ");
  UART_OutUDec(MaxJitter2);
  UART_OutString("\n\rTotal Time in Interrupt(12.5ns)= ");
  UART_OutUDec(tTotal_);
  UART_OutString("\n\rTotal Time= 10s");
  UART_OutString("\n\rMaximum Time in Interrupt(12.5ns)= ");
  UART_OutUDec(tMax);
  UART_OutString("\n\rPercentage in Interrupt(0.01%)= ");
  UART_OutUDec(tTotal_*10000/800000000);
}

void InitTimer3A(uint32_t period) 
{
	long sr;
	volatile uint32_t delay;
	
	sr = StartCritical();
  SYSCTL_RCGCTIMER_R |= 0x08;
	
  delay = SYSCTL_RCGCTIMER_R;
	delay = SYSCTL_RCGCTIMER_R;
  delay = SYSCTL_RCGCTIMER_R;
	delay = SYSCTL_RCGCTIMER_R;
	
  TIMER3_CTL_R &= ~TIMER_CTL_TAEN; // 1) disable timer3A during setup
                                   // 2) configure for 32-bit timer mode
  TIMER3_CFG_R = TIMER_CFG_32_BIT_TIMER;
                                   // 3) configure for periodic mode, default down-count settings
  TIMER3_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER3_TAILR_R = period - 1;     // 4) reload value
                                   // 5) clear timer1A timeout flag
  TIMER3_ICR_R = TIMER_ICR_TATOCINT;
  TIMER3_IMR_R |= TIMER_IMR_TATOIM;// 6) arm timeout interrupt
								   // 7) priority shifted to bits 31-29 for timer2A
  NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|(1 << 29);	//3
  NVIC_EN1_R = NVIC_EN1_INT35;     // 8) enable interrupt 23 in NVIC
  TIMER3_TAPR_R = 0;
  TIMER3_CTL_R |= TIMER_CTL_TAEN;  // 9) enable timer3A
	
  EndCritical(sr);
}

void Timer3A_Handler(void)
{ 

	TIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer3A timeout
  counter2_os++;
}

