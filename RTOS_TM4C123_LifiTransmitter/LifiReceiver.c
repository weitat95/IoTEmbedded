/*
LiFi Emitter and Receiver

A byte is sent as follow :
Start(0) 8bit data Stop(1), LSB first : 0 b0 b1 b2 b3 b4 b5 b6 b7 1
 
Each bit is coded in manchester with 
time is from left to right 
0 -> 10
1 -> 01
A data frame is formatted as follow :
0xAA : sent a number of time to help the receiver compute a signal average for the thresholding of analog values
0xD5 : synchronization byte to indicate start of a frame, breaks the regularity of the 0x55 pattern to be easily 
0x02 : STX start of frame
N times Effective data excluding command symbols, max length 32 bytes
0x03 : ETX end of frame
*/

#include "os.h"
#include "LifiReceiver.h"
#include "ST7735.h"
#define DEBUG
#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))
#define WHITE PF1=0x02; PF2=0x04; PF3=0x08;
#define RED PF1=0x02; PF2=0; PF3=0;
#define GREEN PF1=0; PF2=0x00; PF3=0x08;
#define BLUE PF1=0; PF2=0x04; PF3=0;
#define YELLOW PF1=0x02; PF2=0x00; PF3=0x08;
#define PURPLE  PF1=0x02; PF2=0x04; PF3=0;
#define CYAN PF1=0; PF2=0x04; PF3=0x08;
#define CLEARCOLOR PF1=0; PF2=0; PF3=0;
enum receiver_state frame_state = IDLE ;
Sema4Type semaWordDetected;

#define WORD_LENGTH 10 // a byte is encoded as a 10-bit value with start and stop bits
#define SYNC_SYMBOL 0xD5 // this symbol breaks the premanble of the frame
#define ETX 0x03 // End of frame symbol
#define STX 0x02 //Start or frame symbol 

//manechester decoder state variable
long shift_reg = 0;

// global variables for frame decoding
char frameRec_buffer[38] ;
int frameRec_index  = -1 ;
int frameRec_size = -1 ;


#define START_SYMBOL 0x02
#define STOP_SYMBOL 0x01
#define START_STOP_MASK  ((STOP_SYMBOL << 20) | (START_SYMBOL << 18) | STOP_SYMBOL) //STOP/START/16bits/STOP
#define SYNC_SYMBOL_MANCHESTER  (0x6665) /* Sync symbol, encoded as a 16-bit Manchester value to help the decoding */

int is_a_word(long  * manchester_word, int time_from_last_sync, unsigned int * detected_word){
        if(time_from_last_sync >= 20  || frame_state == IDLE){ // we received enough bits to test the sync      
            if(((*manchester_word) & START_STOP_MASK) == (START_STOP_MASK)){ // testing first position 
                  (*detected_word) = ((*manchester_word) >> 2) & 0xFFFF;
                  if(frame_state == IDLE){
                     if((*detected_word) == SYNC_SYMBOL_MANCHESTER) return 2 ;
                  }
                  return 1 ;
                  // byte with correct framing
            }else if(frame_state != IDLE && time_from_last_sync == 20){
               (*detected_word)= ((*manchester_word) >> 2) & 0xFFFF;
               return 1 ;
            }
          }
          return 0 ;
}


int insert_edge( long  * manchester_word, char edge, int edge_period, int * time_from_last_sync, unsigned int * detected_word){
   int new_word = 0 ;
   int is_a_word_value = 0 ;
   int sync_word_detect = 0 ;
   if( ((*manchester_word) & 0x01) != edge ){ //mak sure we don't have same edge ...
             if(edge_period > (OVERSAMPLING+1)){
                unsigned char last_bit = (*manchester_word) & 0x01 ;
                (*manchester_word) = ((*manchester_word) << 1) | last_bit ; // signal was steady for longer than a single symbol, 
                (*time_from_last_sync) += 1 ;
                is_a_word_value = is_a_word(manchester_word, (*time_from_last_sync), detected_word);
                if(is_a_word_value > 0){ //found start stop framing
                  new_word = 1 ;
									OS_Signal(&semaWordDetected);
                  (*time_from_last_sync) =  0 ;
                  if(is_a_word_value > 1) sync_word_detect = 1 ; //we detected framing and sync word in manchester format
                }
             }
             //storing edge value in word
             if(edge < 0){
              (*manchester_word) = ( (*manchester_word) << 1) | 0x00 ; // signal goes down
             }else{
              (*manchester_word) = ( (*manchester_word) << 1) | 0x01 ; // signal goes up
             }
             (*time_from_last_sync) += 1 ;
             is_a_word_value = is_a_word(manchester_word, (*time_from_last_sync), detected_word);
             if(sync_word_detect == 0 && is_a_word_value > 0){ //if sync word was detected at previous position, don't take word detection into account
               new_word = 1 ;	
							 OS_Signal(&semaWordDetected);
               (*time_from_last_sync) =  0 ;
             }
          }else{
            new_word = -1 ;
          }
          return new_word ;
}

int add_byte_to_frame(char * frame_buffer, int * frame_index, int * frame_size, enum receiver_state * frame_state ,unsigned char data){
  if(data == SYNC_SYMBOL/* && (*frame_index) < 0*/){
    (*frame_index) = 0 ;
    (*frame_size) = 0 ;
    (*frame_state) = SYNC ;
    //Serial.println("SYNC");
		#ifdef DEBUG
		printf("SYNCED\n\r");
		RED
		#endif
    return 0 ;
  }
  if((*frame_state) != IDLE){ // we are synced
  frame_buffer[*frame_index] = data ;
  (*frame_index) ++ ;
    if(data == STX){
      //Serial.println("START");
			#ifdef DEBUG
			printf("START\n\r");
			GREEN
			#endif
      (*frame_state) = START ;
       return 0 ;
    }else if(data == ETX){
      //Serial.println("END");
			ST7735_OutString(1,1,frame_buffer,ST7735_WHITE);
      (*frame_size) = (*frame_index) ;
      (*frame_index) = -1 ;
      (*frame_state) = IDLE ;
      //Serial.println("END");
			#ifdef DEBUG
			printf("END\n\r");
			CLEARCOLOR
			#endif
       return 1 ;
    }else if(data == 0xFF){
			ST7735_OutString(1,1,frame_buffer,ST7735_WHITE);
			(*frame_size) = (*frame_index);
			(*frame_index) = -1;
			(*frame_state) = IDLE;
			#ifdef DEBUG
			printf("END\n\r");
			CLEARCOLOR
			#endif
		}
		else if((*frame_index) >= 38){ //frame is larger than max size of frame ...
      (*frame_index) = -1 ;
      (*frame_size) = -1 ;
      (*frame_state) = IDLE ;
      return -1 ;
    }else{
      (*frame_state) = DATA ;
    }
    return 0 ;
  }
  return -1 ;
}


#define EDGE_THRESHOLD (4096-2300) /* Defines the voltage difference between two samples to detect a rising/falling edge. Can be increased depensing on the environment */
int oldValue = 0 ;
int steady_count = 0 ;
int dist_last_sync = 0 ;
unsigned int detected_word = 0;
int new_word = 0;
char old_edge_val = 0 ;
void sample_signal_edge(int readValue){
  char edge_val ;
  //int sensorValue = analogRead(SENSOR_PIN); // this is too slow and should be replaced with interrupt-driven ADC
  //int sensorValue  = ADC_read_conversion(); // read result of previously triggered conversion
  //ADC_start_conversion(SENSOR_PIN); // start a conversion for next loop
#ifdef DEBUG
  //#ifdef DEBUG_ANALOG
  //Serial.println(sensorValue, DEC);
//printf("%d\n\r",readValue);
  //#endif
#endif
  if((readValue- oldValue) > EDGE_THRESHOLD) edge_val = 1 ;
  else if((oldValue - readValue) > EDGE_THRESHOLD) edge_val = -1;
  else edge_val = 0 ;
  oldValue = readValue ;
  if(edge_val == 0 || edge_val == old_edge_val || (edge_val != old_edge_val && steady_count < 2)){
    if( steady_count < (4 * OVERSAMPLING)){
      steady_count ++ ;
    }
  }else{  
          new_word = insert_edge(&shift_reg, edge_val, steady_count, &(dist_last_sync), &detected_word); 
          if(dist_last_sync > (8*OVERSAMPLING)){ // limit dist_last_sync to avoid overflow problems
            dist_last_sync = 32 ;
          }
					if(new_word==1){
						OS_Signal(&semaWordDetected);
					}
          //if(new_word >= 0){
            steady_count = 0 ;
          //}
        }
        old_edge_val = edge_val ;
}

//get the received data frame and decode
int counter=0;
unsigned char lastChar=0;
int flag=0;
char characterBuff[30];
void printToST7735(unsigned char * received_data){
	
	char character= *received_data;
	if(lastChar==0x02){
		flag=1;
	}else if(lastChar==0x03 ||lastChar==0xFF){
		flag=0;
	}
	if(flag==1){
		characterBuff[counter]=character;
		counter++;
	}else if(flag==0){
		ST7735_OutString(1,1,characterBuff,ST7735_WHITE);
		for(int i=0;i<30;i++){
			characterBuff[i]=0;
		}
		counter=0;
	}
	lastChar=*received_data;
}	
void getDataFrame(void){
	unsigned char received_data;
  char received_data_print ;
  int nb_shift ;
  int byte_added = 0 ;
	OS_Wait(&semaWordDetected);
	if(new_word == 1){
			received_data = 0 ;
			for(int i = 0 ; i < 16 ; i = i + 2){ //decoding Manchester
				received_data = received_data << 1 ;
				if(((detected_word >> i) & 0x03) == 0x01){
					received_data |= 0x01 ;
        }else{
					received_data &= ~0x01 ;
        }
			}
			received_data = received_data & 0xFF ;
#ifdef DEBUG
      printf("received_data: %#04X: %c\n\r",received_data & 0xFF,received_data);
#endif
    new_word = 0 ;
    if((byte_added = add_byte_to_frame(frameRec_buffer, &frameRec_index, &frameRec_size, &frame_state,received_data)) > 0){
      frameRec_buffer[frameRec_size-1] = '\0';
      //printf(&(frameRec_buffer[1]));
    }
    //if(frame_state != IDLE) Serial.println(received_data, HEX);
  }
}

void initLifiReceiver(void){
	OS_InitSemaphore(&semaWordDetected,0);
}

