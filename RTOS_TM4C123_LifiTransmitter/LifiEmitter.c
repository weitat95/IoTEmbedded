#include <stdint.h>
#include <string.h>

#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))
#define BLUE PF1=0; PF2=0x04; PF3=0;
#define CLEARCOLOR PF1=0; PF2=0; PF3=0;
#define WORD_LENGTH 10 /* Each byte is encoded on 10-bit with start, byte, stop */

#define SYNC_SYMBOL 0xD5
#define ETX 0x03
#define STX 0x02
#define DEBUG
unsigned char frame_buffer [38] ; //buffer for frame
char frame_index = -1; // index in frame
char frame_size = -1  ; // size of the frame to be sent


//state variables of the manchester encoder
unsigned char bit_counter = 0 ;
unsigned short data_word = 0 ;  //8bit data + start + stop
unsigned char half_bit = 0 ;
unsigned long int manchester_data ;

#ifdef DEBUG
int state=-1;
unsigned short debugBuff[100];
int i=0;
unsigned short dataStart=0;
void printInManchester(int data){
 if(dataStart==1){
  if(data==0){
    printf("0");
  }else{
    printf("1");
  }
  i++;
  if(i%4==0) printf(" ");
  if(i%20==0) printf("\n\r");
 }
}
#endif
void init_frame(unsigned char * frame){
  memset(frame, 0xAA, 3);
  frame[3] = SYNC_SYMBOL ;
  frame[4] = STX;
  frame_index = -1 ;
  frame_size = -1 ;
}

int create_frame(char * data, int data_size, unsigned char * frame){
  memcpy(&(frame[5]), data, data_size);
  frame[5+data_size] = ETX;
  return 1 ;
}




void to_manchester(unsigned char data, unsigned long int * data_manchester){
  unsigned int i ;
 (*data_manchester) = 0x02 ; // STOP symbol
 (*data_manchester) = (*data_manchester) << 2 ;
  for(i = 0 ; i < 8; i ++){
    if(data & 0x80) (*data_manchester) |=  0x02  ; // data LSB first
    else (*data_manchester) |= 0x01 ;
    (*data_manchester) = (*data_manchester) << 2 ;
    data = data << 1 ; // to next bit
  }
  (*data_manchester) |= 0x01 ; //START symbol
}

void initEmitter(void){
  init_frame(frame_buffer);
}

int write(char * data, unsigned long data_size){
  if(frame_index ==0) return -1;
  if(data_size >32) return -1;
  create_frame(data, data_size,frame_buffer);
  frame_index=0;
  frame_size =data_size+6;
  return 0;
}

//emitter interrupt
void emit_half_bit(){
     if(manchester_data & 0x01){
       #ifdef DEBUG
       printInManchester(1);
       #endif
       BLUE;
     }else{
       //CLR_LED();
       #ifdef DEBUG
       printInManchester(0);
       #endif
       CLEARCOLOR
     }
     bit_counter -- ;
     manchester_data = (manchester_data >> 1);
     if(bit_counter == 0){   
        //is there still bytes to send in the frame ?
        manchester_data = 0 ; // keep sending ones if nothing to send
        if(frame_index >= 0 ){
          if(frame_index < frame_size){
            /*Serial.println(frame_index, DEC);
            Serial.println(frame_buffer[frame_index], HEX);*/
            #ifdef DEBUG
            dataStart=1;
            i=0;
            printf("\n\r");
            #endif 
            to_manchester(frame_buffer[frame_index], &manchester_data);
            frame_index ++ ;
          }else{
            frame_index = -1 ;
            frame_size = -1 ;
            #ifdef DEBUG
            dataStart=0;
            #endif 
          }
        }

        bit_counter = WORD_LENGTH * 2 ;
        //Serial.println(manchester_data, BIN);
      }
}