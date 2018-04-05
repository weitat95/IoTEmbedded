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
//
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
unsigned short debugBuff[30];
int i=0;
unsigned long dFrameCounter=0;
unsigned short dataStart=0;
void resetCounter(void){
	dFrameCounter=0;
}

void printInManchester(int data){
 if(dataStart==1){
  if(data==0){
    printf("0");
		
  }else{
    printf("1");
  }
	if(i>=30){i=0;}
	debugBuff[i]=data;
  i++;
  if((i-2)%4==0) printf(" ");
	if(i==2) printf(" ");
	if(i==18) printf(" ");
  if(i%20==0) {
		printf("\n\r");
		for(int j=0;j<20;j+=2){
			printf(" ");
			if(debugBuff[j]==0 && debugBuff[j+1]==1){
				printf("1");
			}else if(debugBuff[j]==1 &&debugBuff[j+1]==0){
				printf("0");
			}
			if(j==0) {
				printf("  ");
			}
			else if(j%4==0) {
				printf(" ");
			}
		}
		i=0;
	}
	
	
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

void manToAnalogue(unsigned long int *data_manchester, unsigned long *buffer, unsigned long logicHigh, unsigned long logicLow, unsigned short oversampling){
	if(oversampling<1){
		oversampling=1;
	}
	if(oversampling>4){
		oversampling=4;
	}
	for(int i=0 ; i<20*oversampling; i++){
			if(*data_manchester & 0x01) {
				buffer[i]=logicHigh;
			}else { 
				buffer[i]=logicLow;
			}
			if(oversampling==1 || ((i!=0)&& ((i+1)%oversampling==0) )){ //duplicate the data points to imitate oversampling 1700 1700 1700 1700 (logicHigh, oversampling=4)
				(*data_manchester = (*data_manchester) >> 1);
			}
	}
}

void initEmitter(void){
  init_frame(frame_buffer);
}

void resetEmitterState(void){
	init_frame(frame_buffer);
	frame_index = -1; // index in frame
	frame_size = -1  ; // size of the frame to be sent
	//state variables of the manchester encoder
	bit_counter = 0 ;
	data_word = 0 ;  //8bit data + start + stop
	half_bit = 0 ;
	#ifdef DEBUG
	state=-1;
	i=0;
	dFrameCounter=0;
	dataStart=0;
	#endif
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
        manchester_data = 0xAAAAAAAA ; // keep sending ones if nothing to send
        if(frame_index >= 0 ){
          if(frame_index < frame_size){
            /*Serial.println(frame_index, DEC);
            Serial.println(frame_buffer[frame_index], HEX);*/
            #ifdef DEBUG
            dataStart=1;
            i=0;
            printf("\n\r");
						printf("Data Frame:%u\n\r",dFrameCounter);
						dFrameCounter++;
            #endif 
            to_manchester(frame_buffer[frame_index], &manchester_data);
            frame_index ++ ;
          }else{
            frame_index = -1 ;
            frame_size = -1 ;
            #ifdef DEBUG
            dataStart=0;
						printf("\n\r-->");
            #endif 
          }
        }

        bit_counter = WORD_LENGTH * 2 ;
        //Serial.println(manchester_data, BIN);
      }
}