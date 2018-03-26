// Name: efile.c
// Modified: Corentin Dugue & Wei Tat Lee
// Lab 4
// TA: Kishore Punniyamurthy
// Last modified: 03/29/2017
// Hardware connections: none specific

// filename ************** eFile.c *****************************
// High-level routines to implement a solid-state disk
// Jonathan W. Valvano 3/9/17

#include <string.h>
#include "edisk.h"
#include "UART.h"
#include <stdio.h>
#include "efile.h"
#include <stdlib.h>

#define SUCCESS 0
#define FAIL 1
#define EXIST 2
#define OPEN 3
#define UNKNOWN 4
#define NOPEN 5
#define EMPTY 6


BYTE DEBUG[BLOCKSIZE];
dir_entry DIRDEBUG[MAXDIRENTRY];
int DEBUGG;

void eFile_Call(uint32_t err) {
  switch (err) {
    case 1:
      printf("The function failed\n\r");
      break;
    case 2:
      printf("This file already exists\n\r");
      break;
    case 3:
      printf("A file is alread opened\n\r");
      break;
    case 4:
      printf("File couldn't be opened, it doesn't exist\n\r");
      break;
    case 5:
      printf("No file is open\n\r");
      break;
    case 6:
      printf("The file is empty\n\r");
      break;
  }
}


// ****************** PRIVATE FUNCTIONS **************************

void dirIntoByte(dir_entry* dirs, BYTE* buffer){

  for(int i=0;i<MAXDIRENTRY;i++){
    for(int j=0;j<BYTEPERENTRY;j++){
      if(j<8){
        buffer[i*BYTEPERENTRY+j]=dirs[i].name[j];
      }else{
        buffer[i*BYTEPERENTRY+j]=dirs[i].secNum;
        buffer[i*BYTEPERENTRY+j+1]=dirs[i].count>>8;
        buffer[i*BYTEPERENTRY+j+2]=(dirs[i].count<<8)>>8;
        break;
      }
    }
  }

}

void byteIntoDir(BYTE *buffer,dir_entry * dirs){
  WORD buf=0;
  for(int i=0;i<MAXDIRENTRY;i++){
    for(int j=0;j<10;j++){
      if(j<8){
        dirs[i].name[j]=buffer[i*BYTEPERENTRY+j];
      }else{
        dirs[i].secNum=buffer[i*BYTEPERENTRY+j];
        buf=buffer[i*BYTEPERENTRY+j+1];
        buf<<=8;
        buf+=buffer[i*BYTEPERENTRY+j+2];
        dirs[i].count=buf;
        break;
      }
    }
  }

}

void clearDir(dir_entry * dirs){
  for(int i=0;i<MAXDIRENTRY;i++){
    for(int j=0;j<8;j++){
      dirs[i].name[j]=0;
    }  
    dirs[i].count=0;
    dirs[i].secNum=0xFF;
  }
}

void initFAT(BYTE * fat){
  fat[0]=255;
  fat[1]=255;
  for(int i=2;i<MAXBLOCK;i++){
    if(i==MAXBLOCK-1) {fat[i]=0;break;}
    fat[i]=i+1;
    
  }
}

int getFreeDirIndex(dir_entry *dirs){
  for(int i=0;i<MAXDIRENTRY;i++){
    if(dirs[i].secNum==0xFF){
      return i;
    }
  }
  return -1;
}

int findDirEntry(dir_entry *dirs,char name[]){
  for(int i=0;i<MAXDIRENTRY;i++){
    for(int j=0;j<7;j++){
      if(name[j]!=dirs[i].name[j]){break;}
      if(j==6){return i;}
    }
  }
  return -1;
}

void clearBuffer(BYTE * buffer){
  for(int i=0;i<BLOCKSIZE;i++){
    buffer[i]=0;
  }
}
// ****************** PUBLIC FUNCTIONS **************************

//---------- eFile_Init-----------------
// Activate the file system, without formating
// Input: none
// Output: 0 if successful and 1 on failure (already initialized)
int eFile_Init(void){ // initialize file system
  if (eDisk_Init(0)) {return FAIL;}
  return SUCCESS;
}

//---------- eFile_Format-----------------
// Erase all files, create blank directory, initialize free space manager
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Format(void){ // erase disk, add format
  //if (eDisk_Init(0)) {return FAIL;}
  
  // Clear directory
  BYTE buffer[BLOCKSIZE];
  dir_entry dirs[MAXDIRENTRY];
  clearDir(dirs);
  //Dir into byte
  for(int i=0;i<MAXDIRENTRY;i++){
    for(int j=0;j<BYTEPERENTRY;j++){
      if(j<8){
        buffer[i*BYTEPERENTRY+j]=dirs[i].name[j];
      }else{
        buffer[i*BYTEPERENTRY+j]=dirs[i].secNum;
        buffer[i*BYTEPERENTRY+j+1]=dirs[i].count>>8;
        buffer[i*BYTEPERENTRY+j+2]=(dirs[i].count<<8)>>8;
        break;
      }
    }
  }
  
  //Adding pointers to Head and Tail of free block 
  buffer[HEAD]=2;
  buffer[TAIL]=MAXBLOCK-1;
  
  // Write cleared directory and FAT to SD card
  if(eDisk_WriteBlock(buffer,0)){return FAIL;}
  initFAT(buffer);
  if(eDisk_WriteBlock(buffer,1)){return FAIL;}
  clearBuffer(buffer);
  for(int i=2;i<MAXBLOCK;i++){
    if(eDisk_WriteBlock(buffer,i)){return FAIL;}
  }
  return SUCCESS;
}


//---------- eFile_Create-----------------
// Create a new, empty file with one allocated block
// Input: file name is an ASCII string up to seven characters
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Create( char name[]){  // create new file, make it empty
  // Read directory from SD card
  BYTE buffer[BLOCKSIZE];
  eDisk_ReadBlock(buffer, 0);
  
  // Convert directory into dictionary
  dir_entry dirs[MAXDIRENTRY];
  WORD buf=0;
  for(int i=0;i<MAXDIRENTRY;i++){
    for(int j=0;j<10;j++){
      if(j<8){
        dirs[i].name[j]=buffer[i*BYTEPERENTRY+j];
      }else{
        dirs[i].secNum=buffer[i*BYTEPERENTRY+j];
        buf=buffer[i*BYTEPERENTRY+j+1];
        buf<<=8;
        buf+=buffer[i*BYTEPERENTRY+j+2];
        dirs[i].count=buf;
        break;
      }
    }
  }
  
  if (findDirEntry(dirs, name)==-1) {

    // Find head and tail from buffer
    uint16_t head=buffer[HEAD];
    uint16_t tail=buffer[TAIL];
    
    // Read FAT from SD card
    BYTE FAT[BLOCKSIZE];
    eDisk_ReadBlock(FAT, 1);

    // Find free entry in directory
    int i=getFreeDirIndex(dirs);
    if(i==-1){return FAIL;}
    
    // Set name and sector number in free entry
    for (int j=0; j<7; j++) {
      dirs[i].name[j]=name[j];
    }
    dirs[i].secNum=head;
    dirs[i].count=0;
    // Set new head
    head=FAT[head];    
    FAT[dirs[i].secNum]=0;

    // Convert dirs into buffer
    for(int i=0;i<MAXDIRENTRY;i++){
      for(int j=0;j<BYTEPERENTRY;j++){
        if(j<8){
          buffer[i*BYTEPERENTRY+j]=dirs[i].name[j];
        }else{
          buffer[i*BYTEPERENTRY+j]=dirs[i].secNum;
          buffer[i*BYTEPERENTRY+j+1]=dirs[i].count>>8;
          buffer[i*BYTEPERENTRY+j+2]=(dirs[i].count<<8)>>8;
          break;
        }
      }
    }
    buffer[HEAD]=head;
    // Write new directory and new FAT
    if(eDisk_WriteBlock(FAT, 1)){return FAIL;}
    if(eDisk_WriteBlock(buffer, 0)){return FAIL;}

    return SUCCESS;
  }
  else return EXIST;
}

// Global variables to save data when read/write to file
BYTE openBlock[BLOCKSIZE]; // sector currently being open
uint32_t block; // sector number open
uint32_t count; // number of bytes stored for the file
BYTE directory[BLOCKSIZE];
BYTE FAT[BLOCKSIZE];
uint32_t dirIndex;
uint32_t head,tail;
uint32_t nextBlock;
uint16_t position;
uint16_t readON=0;

//---------- eFile_WOpen-----------------
// Open the file, read into RAM last block
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_WOpen(char name[]){      // open a file for writing
  if (readON==0) {
    eDisk_ReadBlock(directory, 0);
    // Convert directory into dictionary
    dir_entry dirs[MAXDIRENTRY];
    //Byte into Dir
    WORD buf=0;
    for(int i=0;i<MAXDIRENTRY;i++){
      for(int j=0;j<10;j++){
        if(j<8){
          dirs[i].name[j]=directory[i*BYTEPERENTRY+j];
        }else{
          dirs[i].secNum=directory[i*BYTEPERENTRY+j];
          buf=directory[i*BYTEPERENTRY+j+1];
          buf<<=8;
          buf+=directory[i*BYTEPERENTRY+j+2];
          dirs[i].count=buf;
          break;
        }
      }
    }

    // Find free entry in directory
    dirIndex=findDirEntry(dirs,name);
    if(dirIndex==-1){return UNKNOWN;} // If file does not exist: UNKNOWN

    
    readON=1;

    count=dirs[dirIndex].count;
    block=dirs[dirIndex].secNum;
    head=directory[HEAD];
    tail=directory[TAIL];
    // Read FAT from SD card
    eDisk_ReadBlock(FAT, 1);
    
    while (FAT[block]!=0) {
      block=FAT[block];
    }

    eDisk_ReadBlock(openBlock, block);
    return SUCCESS;
  }
  else { return OPEN; }
}
//---------- eFile_Write-----------------
// save at end of the open file
// Input: data to be saved
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Write(char data){
  if (readON==1) {
    openBlock[count%BLOCKSIZE]=data;
    count++;
    if(count%BLOCKSIZE==0){ //Reaches the end of the old block
      if(eDisk_WriteBlock(openBlock, block)){return FAIL;} //Save old block to SDCard
      FAT[block]=head; //Link old block to new block in FAT
      block=head;  // Save new block index
      head=FAT[head]; // Saving new Head to directory sector
      FAT[block]=0;  ////Break the link
      
      //i think we dont need this for loop
      for(int k=0; k<BLOCKSIZE; k++){
        openBlock[k]=0;
      }
    }
    return SUCCESS;
  }
  else return NOPEN;
}


//---------- eFile_Close-----------------
// Deactivate the file system
// Input: none
// Output: 0 if successful and 1 on failure (not currently open)
int eFile_Close(void){
  if (readON==1) { readON=0; return SUCCESS; }
  else {return NOPEN;}
}

//---------- eFile_WClose-----------------
// close the file, left disk in a state power can be removed
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_WClose(void){ // close the file for writing
  if (readON==1) {  
    readON=0;
    BYTE buffer[BLOCKSIZE];
    dir_entry dirs[MAXDIRENTRY];
    if(eDisk_ReadBlock(buffer,0)){return FAIL;}
    
    //Byte into Dir
    WORD buf=0;
    for(int i=0;i<MAXDIRENTRY;i++){
      for(int j=0;j<10;j++){
        if(j<8){
          dirs[i].name[j]=buffer[i*BYTEPERENTRY+j];
        }else{
          dirs[i].secNum=buffer[i*BYTEPERENTRY+j];
          buf=buffer[i*BYTEPERENTRY+j+1];
          buf<<=8;
          buf+=buffer[i*BYTEPERENTRY+j+2];
          dirs[i].count=buf;
          break;
        }
      }
    }
    
    
    dirs[dirIndex].count=count;
    
    //Dir into byte
    for(int i=0;i<MAXDIRENTRY;i++){
      for(int j=0;j<BYTEPERENTRY;j++){
        if(j<8){
          buffer[i*BYTEPERENTRY+j]=dirs[i].name[j];
        }else{
          buffer[i*BYTEPERENTRY+j]=dirs[i].secNum;
          buffer[i*BYTEPERENTRY+j+1]=dirs[i].count>>8;
          buffer[i*BYTEPERENTRY+j+2]=(dirs[i].count<<8)>>8;
          break;
        }
      }
    }
    
    buffer[HEAD]=head;
    buffer[TAIL]=tail;
    if(eDisk_WriteBlock(buffer,0)){return FAIL;}
    if(eDisk_WriteBlock(FAT,1)){return FAIL;}
    if(eDisk_WriteBlock(openBlock,block)){return FAIL;}
    return SUCCESS;
  }
  else {return NOPEN;}
}

//---------- eFile_ROpen-----------------
// Open the file, read first block into RAM
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble read to flash)
int eFile_ROpen( char name[]){      // open a file for reading
  if (readON==0) {
  // Read directory from SD card
    eDisk_ReadBlock(directory, 0);
    // Convert directory into dictionary
    dir_entry dirs[MAXDIRENTRY];
    
    //Byte into Dir
    WORD buf=0;
    for(int i=0;i<MAXDIRENTRY;i++){
      for(int j=0;j<10;j++){
        if(j<8){
          dirs[i].name[j]=directory[i*BYTEPERENTRY+j];
        }else{
          dirs[i].secNum=directory[i*BYTEPERENTRY+j];
          buf=directory[i*BYTEPERENTRY+j+1];
          buf<<=8;
          buf+=directory[i*BYTEPERENTRY+j+2];
          dirs[i].count=buf;
          break;
        }
      }
    }
    

    // Find entry directory corresponding to given name
    dirIndex=findDirEntry(dirs,name);
    if(dirIndex==-1){return UNKNOWN;}
    
    readON=1;


    count=dirs[dirIndex].count;
    
    block=dirs[dirIndex].secNum;

    // Read FAT from SD card
    eDisk_ReadBlock(FAT, 1);

    
    if (FAT[block]!=0) {
      nextBlock=FAT[block];
    }

    eDisk_ReadBlock(openBlock, block);
    position=0;
    return SUCCESS;
  }
  else return OPEN;
}

char * pointerRead;
//---------- eFile_ReadNext-----------------
// retreive data from open file
// Input: none
// Output: return by reference data
//         0 if successful and 1 on failure (e.g., end of file)
int eFile_ReadNext( char *pt){       // get next byte
  if (readON==0) { return NOPEN; }
  if (count==0) {return EMPTY;}
  *pt=openBlock[position];
  position++;
  if (FAT[block]==0&& ((position-1)>(count)%512)) {
    return FAIL;
  }
  if (position==512) {
    position=0;
    block=nextBlock;
    if(eDisk_ReadBlock(openBlock, nextBlock)) diskError("eFile_ReadBlock",0);
    if (FAT[nextBlock]!=0) {
      nextBlock=FAT[nextBlock];
    }
  }
  return SUCCESS;
}


//---------- eFile_RClose-----------------
// close the reading file
// Input: none
// Output: 0 if successful and 1 on failure (e.g., wasn't open)
int eFile_RClose(void){ // close the file for writing
  if (readON==0) {
    return NOPEN;
  }
  readON=0;
  return SUCCESS;
}




//---------- eFile_Directory-----------------
// Display the directory with filenames and sizes
// Input: pointer to a function that outputs ASCII characters to display
// Output: none
//         0 if successful and 1 on failure (e.g., trouble reading from flash)
int eFile_Directory(void(*fp)(char)){
  dir_entry dirs[MAXDIRENTRY];
  BYTE buffer[BLOCKSIZE];
  if(eDisk_ReadBlock(buffer,0)){return FAIL;}
  
  //Convert Byte block to directory struct
  WORD buf=0;
  for(int i=0;i<MAXDIRENTRY;i++){
    for(int j=0;j<10;j++){
      if(j<8){
        dirs[i].name[j]=buffer[i*BYTEPERENTRY+j];
      }else{
        dirs[i].secNum=buffer[i*BYTEPERENTRY+j];
        buf=buffer[i*BYTEPERENTRY+j+1];
        buf<<=8;
        buf+=buffer[i*BYTEPERENTRY+j+2];
        dirs[i].count=buf;
        break;
      }
    }
  }
  
  //Start Printing
  
  (*fp)('\n');(*fp)('\r');
  (*fp)('#');(*fp)(':');(*fp)(' ');
  (*fp)('n');(*fp)('a');(*fp)('m');(*fp)('e');
  (*fp)(',');(*fp)(' ');
  (*fp)('c');(*fp)('o');(*fp)('u');(*fp)('n');(*fp)('t');
  (*fp)('\n');(*fp)('\r');
  for(int i=0;i<MAXDIRENTRY;i++){
    if(dirs[i].secNum!=0xFF){
      (*fp)('0'+i);(*fp)(':');(*fp)(' ');
      int j=0;
      while(dirs[i].name[j]!=0){
        (*fp)(dirs[i].name[j]);
        j++;
      }
      (*fp)(',');(*fp)(' ');
    
    
      char buff[10];  
      sprintf(buff,"%d",dirs[i].count);
      int k=0;
      while(buff[k]!=0){
        (*fp)(buff[k]);
        k++;
      }
      (*fp)('\n');(*fp)('\r');
    }
  }
  return SUCCESS;
}

//---------- eFile_Delete-----------------
// delete this file
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Delete( char name[]){  // remove this file
  // Read directory from SD card
  eDisk_ReadBlock(directory, 0);
  // Convert directory into dictionary
  dir_entry dirs[MAXDIRENTRY];

  //Byte into Dir
    WORD buf=0;
    for(int i=0;i<MAXDIRENTRY;i++){
      for(int j=0;j<10;j++){
        if(j<8){
          dirs[i].name[j]=directory[i*BYTEPERENTRY+j];
        }else{
          dirs[i].secNum=directory[i*BYTEPERENTRY+j];
          buf=directory[i*BYTEPERENTRY+j+1];
          buf<<=8;
          buf+=directory[i*BYTEPERENTRY+j+2];
          dirs[i].count=buf;
          break;
        }
      }
    }
  
  
  // Find entry directory corresponding to given name
  dirIndex=findDirEntry(dirs,name);
  if(dirIndex==-1){return UNKNOWN;}

  dirs[dirIndex].count=0;

  block=dirs[dirIndex].secNum;
  tail = directory[TAIL];  // prev tail
  head = directory[HEAD];  // head

  
  dirs[dirIndex].secNum=0xFF;
  for(int j=0;j<8;j++){
    dirs[dirIndex].name[j]=0;
  }  

  // Read FAT from SD card
  eDisk_ReadBlock(FAT, 1);

  FAT[tail]=block;
  
  while (FAT[block]!=0) {
    block=FAT[block];
  }
  //Dir into byte
  for(int i=0;i<MAXDIRENTRY;i++){
    for(int j=0;j<BYTEPERENTRY;j++){
      if(j<8){
        directory[i*BYTEPERENTRY+j]=dirs[i].name[j];
      }else{
        directory[i*BYTEPERENTRY+j]=dirs[i].secNum;
        directory[i*BYTEPERENTRY+j+1]=dirs[i].count>>8;
        directory[i*BYTEPERENTRY+j+2]=(dirs[i].count<<8)>>8;
        break;
      }
    }
  }
  
  directory[TAIL]=block;
  directory[HEAD]=head;

  eDisk_WriteBlock(directory, 0);
  eDisk_WriteBlock(FAT, 1);

  return SUCCESS;    // restore directory back to flash
}

int StreamToFile=0;                // 0=UART, 1=stream to file

int eFile_RedirectToFile(char *name){
  eFile_Create(name);              // ignore error if file already exists
  if(eFile_WOpen(name)) return 1;  // cannot open file
  StreamToFile = 1;
  return 0;
}

int eFile_EndRedirectToFile(void){
  StreamToFile = 0;
  if(eFile_WClose()) return 1;    // cannot close file
  return 0;
}
/*
int fputc (int ch, FILE *f) {
  if(StreamToFile){
    if(eFile_Write(ch)){          // close file on error
       eFile_EndRedirectToFile(); // cannot write to file
       return 1;                  // failure
    }
    return 0; // success writing
  }

   // regular UART output
  UART_OutChar(ch);
  return 0;
}

int fgetc (FILE *f){
  char ch = UART_InChar();  // receive from keyboard
  UART_OutChar(ch);            // echo
  return ch;
}
*/