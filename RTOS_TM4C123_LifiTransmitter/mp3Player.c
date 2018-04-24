#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "mp3/fixpt/mp3dec.h"

static unsigned char allocated = 0;
static HMP3Decoder hMP3Decoder;
static MP3FrameInfo mp3FrameInfo;
static unsigned char *read_ptr;
static int bytes_left=0;
	
void mp3_alloc()
{
	if (!allocated) hMP3Decoder = MP3InitDecoder();
	if(hMP3Decoder){
		printf("mp3_alloc Error");
		while(1){;}
	}
	allocated = 1;
}

void mp3_free()
{
	if (allocated) MP3FreeDecoder(hMP3Decoder);
	allocated = 0;
}

void mp3_process(){
	//	err = MP3Decode(hMP3Decoder, &read_ptr, &bytes_left, dac_buffer[writeable_buffer], 0);
}