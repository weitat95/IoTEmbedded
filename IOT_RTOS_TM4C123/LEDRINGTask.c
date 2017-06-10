#include <stdint.h>
#include "WS2812.h"
#include "os.h"
//------------global variables for patterns------------
// These could be made private when done debugging.
uint8_t phase = 0;          // phase value of the LED pattern
uint8_t red, green, blue;   // value of each color
int32_t ired, igreen, iblue;// amount to increment each color

//------------global variables for random number generator------------
// These could be made private when done debugging.
// Linear congruential generator from Numerical Recipes
// by Press et al.  To use this module, call Random_Init()
// once with a seed and (Random()>>24)%60 over and over to
// get a new random number from 0 to 59.
uint32_t M;

//------------Random_Init------------
// Initialize the random number generator with the given seed.
// Input: seed  new seed value for random number generation
// Output: none
void Random_Init(uint32_t seed){
  M = seed;
}

//------------Pattern_Reset------------
// Reset global variables associated with all test pattern
// functions.  This should be called when transitioning from one
// pattern to the next to prevent brief errors.
// Input: none
// Output: none
void Pattern_Reset(void){
  phase = 0;
}

//------------Pattern_RainbowWaves------------
// Draw a rainbow pattern on the string of LEDs.  Each call
// draws the pattern once by updating the RAM buffer and sending
// it once.  An additional external delay is advised.
// Input: scale  LED outputs are right shifted by this amount (0<=scale<=7) to reduce current/brightness
//        cols   number of LEDs per rainbow (3<=cols)
//        row    index of LED row (0<=row<NUMROWS)
// Output: none
void Pattern_RainbowWaves(uint8_t scale, uint8_t cols, uint8_t row){
  uint32_t i;
  int32_t increment, maximum;
  if(row >= NUMROWS){
    return;                        // row parameter is invalid
  }
  if(cols < 3){
    return;                        // LEDs per rainbow parameter is invalid
  }
  WS2812_SetCursor(0, row);
  cols = 3*(cols/3);               // round down to nearest multiple of 3
  increment = 255/(cols/3);        // constant amount of color added or subtracted from one LED to the next
  maximum = increment*cols/3;      // largest amount of color any LED will reach (i.e. (maximum, 0, 0) is the "reddest" LED)
  if(phase < (cols/3)){
    red = maximum - increment*phase;
    green = increment*phase;
    blue = 0;
    ired = -1*increment;
    igreen = increment;
    iblue = 0;
  } else if(phase < (2*cols/3)){
    red = 0;
    green = maximum - increment*(phase - (cols/3));
    blue = increment*(phase - (cols/3));
    ired = 0;
    igreen = -1*increment;
    iblue = increment;
  } else{
    red = increment*(phase - (2*cols/3));
    green = 0;
    blue = maximum - increment*(phase - (2*cols/3));
    ired = increment;
    igreen = 0;
    iblue = -1*increment;
  }
  phase = phase + 1;
  if(phase >= cols){
    phase = 0;
  }
  for(i=0; i<NUMCOLS; i=i+1){
    WS2812_AddColor(red>>scale, green>>scale, blue>>scale, row);
    red = red + ired;
    green = green + igreen;
    blue = blue + iblue;
    if((red == maximum) && (ired > 0)){
      igreen = ired;
      ired = -1*ired;
    }
    if((red == 0) && (ired < 0)){
      ired = 0;
    }
    if((green == maximum) && (igreen > 0)){
      iblue = igreen;
      igreen = -1*igreen;
    }
    if((green == 0) && (igreen < 0)){
      igreen = 0;
    }
    if((blue == maximum) && (iblue > 0)){
      ired = iblue;
      iblue = -1*iblue;
    }
    if((blue == 0) && (iblue < 0)){
      iblue = 0;
    }
  }
  WS2812_SendReset();
  WS2812_PrintBuffer();
}

//------------Pattern_WhiteWaves------------
// Draw a white wave pattern on the string of LEDs.  Each call
// draws the pattern once by updating the RAM buffer and sending
// it once.  An additional external delay is advised.
// Input: scale  LED outputs are right shifted by this amount (0<=scale<=7) to reduce current/brightness
//        row    index of LED row (0<=row<NUMROWS)
// Output: none
void Pattern_WhiteWaves(uint8_t scale, uint8_t row){
  uint32_t i;
  if(row >= NUMROWS){
    return;                        // row parameter is invalid
  }
  WS2812_SetCursor(0, row);
  if((phase%25) < 13){
    red = green = blue = 21*(phase%25);
  } else if((phase%25) == 13){
    red = green = blue = 252;
  } else{
    red = green = blue = 252 - 21*((phase - 13)%25);
  }
  if((phase%25) < 12){
    ired = igreen = iblue = 21;
  } else if((phase%25) == 12){
    ired = igreen = iblue = 0;
  } else{
    ired = igreen = iblue = -21;
  }
  phase = phase + 1;
  if(phase > 24){
    phase = 0;
  }
  for(i=0; i<NUMCOLS; i=i+1){
    WS2812_AddColor(red>>scale, green>>scale, blue>>scale, row);
    red = red + ired;
    green = green + igreen;
    blue = blue + iblue;
    if((red == 252) && (ired > 0)){
      ired = 0;
    } else if((red == 252) && (ired == 0)){
      ired = -21;
    } else if(red == 0){
      ired = -1*ired;
    }
    if((green == 252) && (igreen > 0)){
      igreen = 0;
    } else if((green == 252) && (igreen == 0)){
      igreen = -21;
    } else if(green == 0){
      igreen = -1*igreen;
    }
    if((blue == 252) && (iblue > 0)){
      iblue = 0;
    } else if((blue == 252) && (iblue == 0)){
      iblue = -21;
    } else if(blue == 0){
      iblue = -1*iblue;
    }
  }
  WS2812_SendReset();
  WS2812_PrintBuffer();
}

//------------Pattern_RedWhiteBlue------------
// Draw a red, white, and blue wave pattern on the string of
// LEDs.  Each call draws the pattern once by updating the RAM
// buffer and sending it once.  An additional external delay is
// advised.
// Input: scale  LED outputs are right shifted by this amount (0<=scale<=7) to reduce current/brightness
//        row    index of LED row (0<=row<NUMROWS)
// Output: none
void Pattern_RedWhiteBlue(uint8_t scale, uint8_t row){
  uint32_t i;
  if(row >= NUMROWS){
    return;                        // row parameter is invalid
  }
  WS2812_SetCursor(0, row);
  if((phase%21) < 3){
    red = 63*(phase%21 + 1);
    green = 0;
    blue = 0;
  } else if((phase%21) < 7){
    red = 252 - 63*(phase%21 - 3);
    green = 0;
    blue = 0;
  } else if((phase%21) < 11){
    red = 63*(phase%21 - 6);
    green = 63*(phase%21 - 6);
    blue = 63*(phase%21 - 6);
  } else if((phase%21) < 14){
    red = 252 - 63*(phase%21 - 10);
    green = 252 - 63*(phase%21 - 10);
    blue = 252 - 63*(phase%21 - 10);
  } else if((phase%21) < 18){
    red = 0;
    green = 0;
    blue = 63*(phase%21 - 13);
  } else{
    red = 0;
    green = 0;
    blue = 252 - 63*(phase%21 - 17);
  }
  if((phase%21) < 3){
    ired = 63;
    igreen = 0;
    iblue = 0;
  } else if((phase%21) < 6){
    ired = -63;
    igreen = 0;
    iblue = 0;
  } else if((phase%21) == 6){
    ired = 0;
    igreen = 63;
    iblue = 63;
  } else if((phase%21) < 10){
    ired = 63;
    igreen = 63;
    iblue = 63;
  } else if((phase%21) < 13){
    ired = -63;
    igreen = -63;
    iblue = -63;
  } else if((phase%21) == 13){
    ired = -63;
    igreen = -63;
    iblue = 0;
  } else if((phase%21) < 17){
    ired = 0;
    igreen = 0;
    iblue = 63;
  } else if((phase%21) < 20){
    ired = 0;
    igreen = 0;
    iblue = -63;
  } else{
    ired = 63;
    igreen = 0;
    iblue = -63;
  }
  phase = phase + 1;
  if(phase > 20){
    phase = 0;
  }
  for(i=phase; i<(NUMCOLS+phase); i=i+1){
    WS2812_AddColor(red>>scale, green>>scale, blue>>scale, row);
    red = red + ired;
    green = green + igreen;
    blue = blue + iblue;
    if((i%21) < 3){
      ired = 63;
      igreen = 0;
      iblue = 0;
    } else if((i%21) < 6){
      ired = -63;
      igreen = 0;
      iblue = 0;
    } else if((i%21) == 6){
      ired = 0;
      igreen = 63;
      iblue = 63;
    } else if((i%21) < 10){
      ired = 63;
      igreen = 63;
      iblue = 63;
    } else if((i%21) < 13){
      ired = -63;
      igreen = -63;
      iblue = -63;
    } else if((i%21) == 13){
      ired = -63;
      igreen = -63;
      iblue = 0;
    } else if((i%21) < 17){
      ired = 0;
      igreen = 0;
      iblue = 63;
    } else if((i%21) < 20){
      ired = 0;
      igreen = 0;
      iblue = -63;
    } else{
      ired = 63;
      igreen = 0;
      iblue = -63;
    }
  }
  WS2812_SendReset();
  WS2812_PrintBuffer();
}

//------------Pattern_Stacker------------
// Stack LEDs of the given color from end to beginning of the
// LED strand.  The color starts at the end of the strand at
// full brightness and shifts to the beginning.  It stops at the
// first unilluminated LED nearest to the beginning, leaving the
// LED illuminated with the given color divided by 8.  The
// divide-by-eight limits the total current after this function
// has been called repeatedly and helps to emphasize the next
// moving LED.  The color delays at least 3*delay cycles before
// moving one step toward the beginning.  This means that
// subsequent calls to this function take less time as the LED
// strand fills up and later colors travel fewer steps.
// Input: red   8-bit red color value
//        green 8-bit green color value
//        blue  8-bit blue color value
//        delay wait 3 times this number of clock cycles between animations
//        row   index of LED row (0<=row<NUMROWS)
// Output: none
void Pattern_Stacker(uint8_t red, uint8_t green, uint8_t blue, uint32_t delay, uint8_t row){
  uint32_t i;
  if(row >= NUMROWS){
    return;                        // row parameter is invalid
  }
  for(i=(NUMCOLS-1); i>phase; i=i-1){
    WS2812_SetCursor(i, row);
    WS2812_AddColor(red, green, blue, row);
    WS2812_SendReset();
    WS2812_PrintBuffer();
    OS_Sleep(delay);
    WS2812_SetCursor(i, row);
    WS2812_AddColor(0, 0, 0, row);
    WS2812_SendReset();
    WS2812_PrintBuffer();
  }
  WS2812_SetCursor(phase, row);
  WS2812_AddColor(red>>3, green>>3, blue>>3, row);
  WS2812_SendReset();
  WS2812_PrintBuffer();
  OS_Sleep(delay);
  phase = phase + 1;
}

//------------Pattern_ThreeRowSquare------------
// Draw a square wave across the bottom three rows of LEDs.
// Each call draws the pattern once by updating the RAM buffer
// and sending it once.  An additional external delay is
// advised.  The pattern is eight columns long and repeating.
// [*] [*] [*] [*] [*] [ ] [ ] [ ]
// [*] [ ] [ ] [ ] [*] [ ] [ ] [ ]
// [*] [ ] [ ] [ ] [*] [*] [*] [*]
// Input: waveRed   8-bit red color value for the wave
//        waveGreen 8-bit green color value for the wave
//        waveBlue  8-bit blue color value for the wave
//        bgRed     8-bit red color value for the background
//        bgGreen   8-bit green color value for the background
//        bgBlue    8-bit blue color value for the background
// Output: none
void Pattern_ThreeRowSquare(uint8_t waveRed, uint8_t waveGreen, uint8_t waveBlue, uint8_t bgRed, uint8_t bgGreen, uint8_t bgBlue){
  uint32_t total, divisor, i;
  // This pattern will assign a color to every LED, and there
  // are 3 individual colored LEDs per LED unit, NUMCOLS (see
  // definition in WS2812.h) LEDs per row/strand, 3
  // rows/strands, and each colored LED uses a maximum of 20
  // mA.  All LEDs at full brightness will draw 3*3*20*NUMCOLS
  // mA, which can easily exceed your system's specifications.
  // Instead, automatically scale down the foreground and
  // background brightness to keep current below about 750 mA.
  // On average, each column has 1.5 LEDs on and 1.5 LEDs off.
  // The total amount of color in all LEDs is:
  total = (waveRed + waveGreen + waveBlue + bgRed + bgGreen + bgBlue)*NUMCOLS;
  // An LED with color of 255 draws about 20 mA.  (But the
  // previous line really should have been multiplied by 1.5.
  // See the equivalent lines in Pattern_ThreeRowTriangle()
  // and Pattern_ThreeRowSine() for clearer numbers.)
  // 1.5 LEDs with color of 255 draw about 30 mA.
  // The total amount of current in all LEDs is approximately:
  total = total*30/255; // units of mA
  // Calculate the value to divide all colors by to decrease
  // the total current below 750 mA.
  divisor = total/750 + 1;
  // Update the colors by dividing by the divisor.
  waveRed = waveRed/divisor;
  waveGreen = waveGreen/divisor;
  waveBlue = waveBlue/divisor;
  bgRed = bgRed/divisor;
  bgGreen = bgGreen/divisor;
  bgBlue = bgBlue/divisor;
  // Draw the pattern, starting at the first column.
  WS2812_SetCursor(0, 0);
  WS2812_SetCursor(0, 1);
  WS2812_SetCursor(0, 2);
  for(i=0; i<NUMCOLS; i=i+1){
    if(((i + phase + 4)%8) < 5){
      WS2812_AddColor(waveRed, waveGreen, waveBlue, 0);
    } else{
      WS2812_AddColor(bgRed, bgGreen, bgBlue, 0);
    }
    if(((i + phase)%4) == 0){
      WS2812_AddColor(waveRed, waveGreen, waveBlue, 1);
    } else{
      WS2812_AddColor(bgRed, bgGreen, bgBlue, 1);
    }
    if(((i + phase)%8) < 5){
      WS2812_AddColor(waveRed, waveGreen, waveBlue, 2);
    } else{
      WS2812_AddColor(bgRed, bgGreen, bgBlue, 2);
    }
  }
  phase = phase + 1;
  WS2812_SendReset();
  WS2812_PrintBuffer();
}

//------------Pattern_ThreeRowTriangle------------
// Draw a triangle wave across the bottom three rows of LEDs.
// Each call draws the pattern once by updating the RAM buffer
// and sending it once.  An additional external delay is
// advised.  The pattern is four columns long and repeating.
// [ ] [ ] [*] [ ]
// [ ] [*] [ ] [*]
// [*] [ ] [ ] [ ]
// Input: waveRed   8-bit red color value for the wave
//        waveGreen 8-bit green color value for the wave
//        waveBlue  8-bit blue color value for the wave
//        bgRed     8-bit red color value for the background
//        bgGreen   8-bit green color value for the background
//        bgBlue    8-bit blue color value for the background
// Output: none
void Pattern_ThreeRowTriangle(uint8_t waveRed, uint8_t waveGreen, uint8_t waveBlue, uint8_t bgRed, uint8_t bgGreen, uint8_t bgBlue){
  uint32_t total, divisor, i;
  // This pattern will assign a color to every LED, and there
  // are 3 individual colored LEDs per LED unit, NUMCOLS (see
  // definition in WS2812.h) LEDs per row/strand, 3
  // rows/strands, and each colored LED uses a maximum of 20
  // mA.  All LEDs at full brightness will draw 3*3*20*NUMCOLS
  // mA, which can easily exceed your system's specifications.
  // Instead, automatically scale down the foreground and
  // background brightness to keep current below about 750 mA.
  // Each column has one LED on and two LEDs off.
  // The total amount of color in all LEDs is:
  total = (waveRed + waveGreen + waveBlue + 2*bgRed + 2*bgGreen + 2*bgBlue)*NUMCOLS;
  // An LED with color of 255 draws about 20 mA.
  // The total amount of current in all LEDs is approximately:
  total = total*20/255; // units of mA
  // Calculate the value to divide all colors by to decrease
  // the total current below 750 mA.
  divisor = total/750 + 1;
  // Update the colors by dividing by the divisor.
  waveRed = waveRed/divisor;
  waveGreen = waveGreen/divisor;
  waveBlue = waveBlue/divisor;
  bgRed = bgRed/divisor;
  bgGreen = bgGreen/divisor;
  bgBlue = bgBlue/divisor;
  // Draw the pattern, starting at the first column.
  WS2812_SetCursor(0, 0);
  WS2812_SetCursor(0, 1);
  WS2812_SetCursor(0, 2);
  for(i=0; i<NUMCOLS; i=i+1){
    if(((i + phase)%4) == 0){
      WS2812_AddColor(waveRed, waveGreen, waveBlue, 0);
    } else{
      WS2812_AddColor(bgRed, bgGreen, bgBlue, 0);
    }
    if(((i + phase)%2) == 1){
      WS2812_AddColor(waveRed, waveGreen, waveBlue, 1);
    } else{
      WS2812_AddColor(bgRed, bgGreen, bgBlue, 1);
    }
    if(((i + phase)%4) == 2){
      WS2812_AddColor(waveRed, waveGreen, waveBlue, 2);
    } else{
      WS2812_AddColor(bgRed, bgGreen, bgBlue, 2);
    }
  }
  phase = phase + 1;
  WS2812_SendReset();
  WS2812_PrintBuffer();
}

//------------Pattern_ThreeRowSine------------
// Draw a sine wave across the bottom three rows of LEDs.
// Each call draws the pattern once by updating the RAM buffer
// and sending it once.  An additional external delay is
// advised.  The pattern is six columns long and repeating.
// [ ] [ ] [ ] [*] [*] [ ]
// [ ] [ ] [*] [ ] [ ] [*]
// [*] [*] [ ] [ ] [ ] [ ]
// Input: waveRed   8-bit red color value for the wave
//        waveGreen 8-bit green color value for the wave
//        waveBlue  8-bit blue color value for the wave
//        bgRed     8-bit red color value for the background
//        bgGreen   8-bit green color value for the background
//        bgBlue    8-bit blue color value for the background
// Output: none
void Pattern_ThreeRowSine(uint8_t waveRed, uint8_t waveGreen, uint8_t waveBlue, uint8_t bgRed, uint8_t bgGreen, uint8_t bgBlue){
  uint32_t total, divisor, i;
  // This pattern will assign a color to every LED, and there
  // are 3 individual colored LEDs per LED unit, NUMCOLS (see
  // definition in WS2812.h) LEDs per row/strand, 3
  // rows/strands, and each colored LED uses a maximum of 20
  // mA.  All LEDs at full brightness will draw 3*3*20*NUMCOLS
  // mA, which can easily exceed your system's specifications.
  // Instead, automatically scale down the foreground and
  // background brightness to keep current below about 750 mA.
  // Each column has one LED on and two LEDs off.
  // The total amount of color in all LEDs is:
  total = (waveRed + waveGreen + waveBlue + 2*bgRed + 2*bgGreen + 2*bgBlue)*NUMCOLS;
  // An LED with color of 255 draws about 20 mA.
  // The total amount of current in all LEDs is approximately:
  total = total*20/255; // units of mA
  // Calculate the value to divide all colors by to decrease
  // the total current below 750 mA.
  divisor = total/750 + 1;
  // Update the colors by dividing by the divisor.
  waveRed = waveRed/divisor;
  waveGreen = waveGreen/divisor;
  waveBlue = waveBlue/divisor;
  bgRed = bgRed/divisor;
  bgGreen = bgGreen/divisor;
  bgBlue = bgBlue/divisor;
  // Draw the pattern, starting at the first column.
  WS2812_SetCursor(0, 0);
  WS2812_SetCursor(0, 1);
  WS2812_SetCursor(0, 2);
  for(i=0; i<NUMCOLS; i=i+1){
    if(((i + phase)%6) < 2){
      WS2812_AddColor(waveRed, waveGreen, waveBlue, 0);
    } else{
      WS2812_AddColor(bgRed, bgGreen, bgBlue, 0);
    }
    if(((i + phase)%3) == 2){
      WS2812_AddColor(waveRed, waveGreen, waveBlue, 1);
    } else{
      WS2812_AddColor(bgRed, bgGreen, bgBlue, 1);
    }
    if(((i + phase + 3)%6) < 2){
      WS2812_AddColor(waveRed, waveGreen, waveBlue, 2);
    } else{
      WS2812_AddColor(bgRed, bgGreen, bgBlue, 2);
    }
  }
  phase = phase + 1;
  WS2812_SendReset();
  WS2812_PrintBuffer();
}

//------------Pattern_PwrMeterBasic------------
// Implement a power meter by illuminating a given number of
// LEDs, starting at the beginning of the strand.  The whole
// meter will always be the same constant color.
// Input: litLEDs number of LEDs to illuminate (litLEDs<=NUMCOLS)
//        row     index of LED row (0<=row<NUMROWS)
// Output: none
#define PWR_BASIC_RED 0
#define PWR_BASIC_GRN 0
#define PWR_BASIC_BLU 255
void Pattern_PwrMeterBasic(uint8_t litLEDs, uint8_t row){
  uint32_t i;
  if(row >= NUMROWS){
    return;                        // row parameter is invalid
  }
  WS2812_SetCursor(0, row);
  for(i=1; i<=NUMCOLS; i=i+1){
    if(litLEDs == 0){              // nothing to display; no LEDs on
      WS2812_AddColor(0, 0, 0, row);
    } else if(i < (litLEDs - 1)){  // LEDs near beginning of strand slightly on
      WS2812_AddColor(PWR_BASIC_RED>>3, PWR_BASIC_GRN>>3, PWR_BASIC_BLU>>3, row);
    } else if(i == (litLEDs - 1)){ // LED near end of bar slightly brighter
      WS2812_AddColor(PWR_BASIC_RED>>1, PWR_BASIC_GRN>>1, PWR_BASIC_BLU>>1, row);
    } else if(i == litLEDs){       // LED at end of bar full brightness
      WS2812_AddColor(PWR_BASIC_RED, PWR_BASIC_GRN, PWR_BASIC_BLU, row);
    } else{                        // LEDs beyond end of bar fully off
      WS2812_AddColor(0, 0, 0, row);
    }
  }
  WS2812_SendReset();
  WS2812_PrintBuffer();
}

//------------Pattern_PwrMeterStriped------------
// Implement a power meter by illuminating a given number of
// LEDs, starting at the beginning of the strand.  The meter
// will be striped, starting at the beginning of the strand.
// A fully illuminated striped power meter will be made of green
// LEDs followed by yellow, followed by orange, followed by red.
// A partially illuminated striped power meter will be made of
// green LEDs followed by yellow, possibly followed by orange.
// Input: litLEDs number of LEDs to illuminate (litLEDs<=NUMCOLS)
//        row     index of LED row (0<=row<NUMROWS)
// Output: none
#define PWR_STRIPE0     10   // number of LEDs in first stripe
#define PWR_STRIPE0_RED 0
#define PWR_STRIPE0_GRN 255
#define PWR_STRIPE0_BLU 0
#define PWR_STRIPE1     10   // number of LEDs in second stripe
#define PWR_STRIPE1_RED 255
#define PWR_STRIPE1_GRN 255
#define PWR_STRIPE1_BLU 0
#define PWR_STRIPE2     10   // number of LEDs in third stripe
#define PWR_STRIPE2_RED 255
#define PWR_STRIPE2_GRN 128
#define PWR_STRIPE2_BLU 0
#define PWR_STRIPE3     225  // number of LEDs in fourth stripe (all remaining LEDs)
#define PWR_STRIPE3_RED 255
#define PWR_STRIPE3_GRN 0
#define PWR_STRIPE3_BLU 0
void Pattern_PwrMeterStriped(uint8_t litLEDs, uint8_t row){
  uint32_t i;
  if(row >= NUMROWS){
    return;                        // row parameter is invalid
  }
  WS2812_SetCursor(0, row);
  for(i=1; i<=NUMCOLS; i=i+1){
    if(litLEDs == 0){              // nothing to display; no LEDs on
      WS2812_AddColor(0, 0, 0, row);
    } else if(i < (litLEDs - 1)){  // LEDs near beginning of strand slightly on
      if(i <= PWR_STRIPE0){
        WS2812_AddColor(PWR_STRIPE0_RED>>3, PWR_STRIPE0_GRN>>3, PWR_STRIPE0_BLU>>3, row);
      } else if(i <= (PWR_STRIPE0 + PWR_STRIPE1)){
        WS2812_AddColor(PWR_STRIPE1_RED>>3, PWR_STRIPE1_GRN>>3, PWR_STRIPE1_BLU>>3, row);
      } else if(i <= (PWR_STRIPE0 + PWR_STRIPE1 + PWR_STRIPE2)){
        WS2812_AddColor(PWR_STRIPE2_RED>>3, PWR_STRIPE2_GRN>>3, PWR_STRIPE2_BLU>>3, row);
      } else{
        WS2812_AddColor(PWR_STRIPE3_RED>>3, PWR_STRIPE3_GRN>>3, PWR_STRIPE3_BLU>>3, row);
      }
    } else if(i == (litLEDs - 1)){ // LED near end of bar slightly brighter
      if(i <= PWR_STRIPE0){
        WS2812_AddColor(PWR_STRIPE0_RED>>1, PWR_STRIPE0_GRN>>1, PWR_STRIPE0_BLU>>1, row);
      } else if(i <= (PWR_STRIPE0 + PWR_STRIPE1)){
        WS2812_AddColor(PWR_STRIPE1_RED>>1, PWR_STRIPE1_GRN>>1, PWR_STRIPE1_BLU>>1, row);
      } else if(i <= (PWR_STRIPE0 + PWR_STRIPE1 + PWR_STRIPE2)){
        WS2812_AddColor(PWR_STRIPE2_RED>>1, PWR_STRIPE2_GRN>>1, PWR_STRIPE2_BLU>>1, row);
      } else{
        WS2812_AddColor(PWR_STRIPE3_RED>>1, PWR_STRIPE3_GRN>>1, PWR_STRIPE3_BLU>>1, row);
      }
    } else if(i == litLEDs){       // LED at end of bar full brightness
      if(i <= PWR_STRIPE0){
        WS2812_AddColor(PWR_STRIPE0_RED, PWR_STRIPE0_GRN, PWR_STRIPE0_BLU, row);
      } else if(i <= (PWR_STRIPE0 + PWR_STRIPE1)){
        WS2812_AddColor(PWR_STRIPE1_RED, PWR_STRIPE1_GRN, PWR_STRIPE1_BLU, row);
      } else if(i <= (PWR_STRIPE0 + PWR_STRIPE1 + PWR_STRIPE2)){
        WS2812_AddColor(PWR_STRIPE2_RED, PWR_STRIPE2_GRN, PWR_STRIPE2_BLU, row);
      } else{
        WS2812_AddColor(PWR_STRIPE3_RED, PWR_STRIPE3_GRN, PWR_STRIPE3_BLU, row);
      }
    } else{                        // LEDs beyond end of bar fully off
      WS2812_AddColor(0, 0, 0, row);
    }
  }
  WS2812_SendReset();
  WS2812_PrintBuffer();
}

//------------Pattern_PwrMeterColored------------
// Implement a power meter by illuminating a given number of
// LEDs, starting at the beginning of the strand.  The meter
// will be all one color, according to the number of LEDs
// illuminated.  A fully illuminated colored power meter will be
// fully red.  A partially illuminated colored power meter will
// be fully orange or fully yellow.
// Input: litLEDs number of LEDs to illuminate (litLEDs<=NUMCOLS)
//        row     index of LED row (0<=row<NUMROWS)
// Output: none
void Pattern_PwrMeterColored(uint8_t litLEDs, uint8_t row){
  uint32_t i;
  if(row >= NUMROWS){
    return;                        // row parameter is invalid
  }
  WS2812_SetCursor(0, row);
  for(i=1; i<=NUMCOLS; i=i+1){
    if(litLEDs == 0){              // nothing to display; no LEDs on
      WS2812_AddColor(0, 0, 0, row);
    } else if(litLEDs <= PWR_STRIPE0){
      if(i < (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE0_RED>>3, PWR_STRIPE0_GRN>>3, PWR_STRIPE0_BLU>>3, row);
      } else if(i == (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE0_RED>>1, PWR_STRIPE0_GRN>>1, PWR_STRIPE0_BLU>>1, row);
      } else if(i == litLEDs){
        WS2812_AddColor(PWR_STRIPE0_RED, PWR_STRIPE0_GRN, PWR_STRIPE0_BLU, row);
      } else{
        WS2812_AddColor(0, 0, 0, row);
      }
    } else if(litLEDs <= (PWR_STRIPE0 + PWR_STRIPE1)){
      if(i < (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE1_RED>>3, PWR_STRIPE1_GRN>>3, PWR_STRIPE1_BLU>>3, row);
      } else if(i == (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE1_RED>>1, PWR_STRIPE1_GRN>>1, PWR_STRIPE1_BLU>>1, row);
      } else if(i == litLEDs){
        WS2812_AddColor(PWR_STRIPE1_RED, PWR_STRIPE1_GRN, PWR_STRIPE1_BLU, row);
      } else{
        WS2812_AddColor(0, 0, 0, row);
      }
    } else if(litLEDs <= (PWR_STRIPE0 + PWR_STRIPE1 + PWR_STRIPE2)){
      if(i < (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE2_RED>>3, PWR_STRIPE2_GRN>>3, PWR_STRIPE2_BLU>>3, row);
      } else if(i == (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE2_RED>>1, PWR_STRIPE2_GRN>>1, PWR_STRIPE2_BLU>>1, row);
      } else if(i == litLEDs){
        WS2812_AddColor(PWR_STRIPE2_RED, PWR_STRIPE2_GRN, PWR_STRIPE2_BLU, row);
      } else{
        WS2812_AddColor(0, 0, 0, row);
      }
    } else{
      if(i < (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE3_RED>>3, PWR_STRIPE3_GRN>>3, PWR_STRIPE3_BLU>>3, row);
      } else if(i == (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE3_RED>>1, PWR_STRIPE3_GRN>>1, PWR_STRIPE3_BLU>>1, row);
      } else if(i == litLEDs){
        WS2812_AddColor(PWR_STRIPE3_RED, PWR_STRIPE3_GRN, PWR_STRIPE3_BLU, row);
      } else{
        WS2812_AddColor(0, 0, 0, row);
      }
    }
  }
  WS2812_SendReset();
  WS2812_PrintBuffer();
}

//------------Pattern_SplitMeterBasic------------
// Implement a power meter by illuminating a given number of
// LEDs, starting in the middle of the strand.  The whole meter
// will always be the same constant color.
// Input: litLEDs     number of LEDs to illuminate (litLEDs<=LEDsPerSide<=(NUMCOLS/2))
//        LEDsPerSide number of LEDs on each side of the meter
//        row         index of LED row (0<=row<NUMROWS)
// Output: none
void Pattern_SplitMeterBasic(uint8_t litLEDs, uint8_t LEDsPerSide, uint8_t row){
  uint32_t i;
  if(row >= NUMROWS){
    return;                        // row parameter is invalid
  }
  if(LEDsPerSide > (NUMCOLS/2)){   // check that the whole meter fits in the LEDs
    LEDsPerSide = NUMCOLS/2;       // balance the number of LEDs per side if needed
  }
  if(litLEDs > LEDsPerSide){       // check that each half of the meter fits
    litLEDs = LEDsPerSide;         // light all of them
  }
  WS2812_SetCursor(0, row);
  for(i=LEDsPerSide; i>=1; i=i-1){ // draw first half of meter
    if(litLEDs == 0){              // nothing to display; no LEDs on
      WS2812_AddColor(0, 0, 0, row);
    } else if(i < (litLEDs - 1)){  // LEDs near middle of strand slightly on
      WS2812_AddColor(PWR_BASIC_RED>>3, PWR_BASIC_GRN>>3, PWR_BASIC_BLU>>3, row);
    } else if(i == (litLEDs - 1)){ // LED near end of bar slightly brighter
      WS2812_AddColor(PWR_BASIC_RED>>1, PWR_BASIC_GRN>>1, PWR_BASIC_BLU>>1, row);
    } else if(i == litLEDs){       // LED at end of bar full brightness
      WS2812_AddColor(PWR_BASIC_RED, PWR_BASIC_GRN, PWR_BASIC_BLU, row);
    } else{                        // LEDs beyond end of bar fully off
      WS2812_AddColor(0, 0, 0, row);
    }
  }
  for(i=1; i<=LEDsPerSide; i=i+1){ // draw second half of meter
    if(litLEDs == 0){              // nothing to display; no LEDs on
      WS2812_AddColor(0, 0, 0, row);
    } else if(i < (litLEDs - 1)){  // LEDs near middle of strand slightly on
      WS2812_AddColor(PWR_BASIC_RED>>3, PWR_BASIC_GRN>>3, PWR_BASIC_BLU>>3, row);
    } else if(i == (litLEDs - 1)){ // LED near end of bar slightly brighter
      WS2812_AddColor(PWR_BASIC_RED>>1, PWR_BASIC_GRN>>1, PWR_BASIC_BLU>>1, row);
    } else if(i == litLEDs){       // LED at end of bar full brightness
      WS2812_AddColor(PWR_BASIC_RED, PWR_BASIC_GRN, PWR_BASIC_BLU, row);
    } else{                        // LEDs beyond end of bar fully off
      WS2812_AddColor(0, 0, 0, row);
    }
  }
                                   // shut off any remaining LEDs
  for(i=(2*LEDsPerSide); i<NUMCOLS; i=i+1){
    WS2812_AddColor(0, 0, 0, row);
  }
  WS2812_SendReset();
  WS2812_PrintBuffer();
}

//------------Pattern_SplitMeterStriped------------
// Implement a power meter by illuminating a given number of
// LEDs, starting in the middle of the strand.  The meter will
// be striped, starting in the middle of the strand.
// A fully illuminated striped power meter will be made of red
// LEDs followed by orange, followed by yellow, followed by
// two bands of green, followed by yellow, followed by orange,
// followed by red.
// A partially illuminated striped power meter will be made of
// green LEDs surrounded by yellow, possibly also surrounded by
// orange.
// Input: litLEDs number of LEDs to illuminate (litLEDs<=LEDsPerSide<=(NUMCOLS/2))
//        LEDsPerSide number of LEDs on each side of the meter
//        row     index of LED row (0<=row<NUMROWS)
// Output: none
void Pattern_SplitMeterStriped(uint8_t litLEDs, uint8_t LEDsPerSide, uint8_t row){
  uint32_t i;
  if(row >= NUMROWS){
    return;                        // row parameter is invalid
  }
  if(LEDsPerSide > (NUMCOLS/2)){   // check that the whole meter fits in the LEDs
    LEDsPerSide = NUMCOLS/2;       // balance the number of LEDs per side if needed
  }
  if(litLEDs > LEDsPerSide){       // check that each half of the meter fits
    litLEDs = LEDsPerSide;         // light all of them
  }
  WS2812_SetCursor(0, row);
  for(i=LEDsPerSide; i>=1; i=i-1){ // draw first half of meter
    if(litLEDs == 0){              // nothing to display; no LEDs on
      WS2812_AddColor(0, 0, 0, row);
    } else if(i < (litLEDs - 1)){  // LEDs near middle of strand slightly on
      if(i <= (PWR_STRIPE0/2)){
        WS2812_AddColor(PWR_STRIPE0_RED>>3, PWR_STRIPE0_GRN>>3, PWR_STRIPE0_BLU>>3, row);
      } else if(i <= ((PWR_STRIPE0/2) + (PWR_STRIPE1/2))){
        WS2812_AddColor(PWR_STRIPE1_RED>>3, PWR_STRIPE1_GRN>>3, PWR_STRIPE1_BLU>>3, row);
      } else if(i <= ((PWR_STRIPE0/2) + (PWR_STRIPE1/2) + (PWR_STRIPE2/2))){
        WS2812_AddColor(PWR_STRIPE2_RED>>3, PWR_STRIPE2_GRN>>3, PWR_STRIPE2_BLU>>3, row);
      } else{
        WS2812_AddColor(PWR_STRIPE3_RED>>3, PWR_STRIPE3_GRN>>3, PWR_STRIPE3_BLU>>3, row);
      }
    } else if(i == (litLEDs - 1)){ // LED near end of bar slightly brighter
      if(i <= (PWR_STRIPE0/2)){
        WS2812_AddColor(PWR_STRIPE0_RED>>1, PWR_STRIPE0_GRN>>1, PWR_STRIPE0_BLU>>1, row);
      } else if(i <= ((PWR_STRIPE0/2) + (PWR_STRIPE1/2))){
        WS2812_AddColor(PWR_STRIPE1_RED>>1, PWR_STRIPE1_GRN>>1, PWR_STRIPE1_BLU>>1, row);
      } else if(i <= ((PWR_STRIPE0/2) + (PWR_STRIPE1/2) + (PWR_STRIPE2/2))){
        WS2812_AddColor(PWR_STRIPE2_RED>>1, PWR_STRIPE2_GRN>>1, PWR_STRIPE2_BLU>>1, row);
      } else{
        WS2812_AddColor(PWR_STRIPE3_RED>>1, PWR_STRIPE3_GRN>>1, PWR_STRIPE3_BLU>>1, row);
      }
    } else if(i == litLEDs){       // LED at end of bar full brightness
      if(i <= (PWR_STRIPE0/2)){
        WS2812_AddColor(PWR_STRIPE0_RED, PWR_STRIPE0_GRN, PWR_STRIPE0_BLU, row);
      } else if(i <= ((PWR_STRIPE0/2) + (PWR_STRIPE1/2))){
        WS2812_AddColor(PWR_STRIPE1_RED, PWR_STRIPE1_GRN, PWR_STRIPE1_BLU, row);
      } else if(i <= ((PWR_STRIPE0/2) + (PWR_STRIPE1/2) + (PWR_STRIPE2/2))){
        WS2812_AddColor(PWR_STRIPE2_RED, PWR_STRIPE2_GRN, PWR_STRIPE2_BLU, row);
      } else{
        WS2812_AddColor(PWR_STRIPE3_RED, PWR_STRIPE3_GRN, PWR_STRIPE3_BLU, row);
      }
    } else{                        // LEDs beyond end of bar fully off
      WS2812_AddColor(0, 0, 0, row);
    }
  }
  for(i=1; i<=LEDsPerSide; i=i+1){ // draw second half of meter
    if(litLEDs == 0){              // nothing to display; no LEDs on
      WS2812_AddColor(0, 0, 0, row);
    } else if(i < (litLEDs - 1)){  // LEDs near middle of strand slightly on
      if(i <= (PWR_STRIPE0/2)){
        WS2812_AddColor(PWR_STRIPE0_RED>>3, PWR_STRIPE0_GRN>>3, PWR_STRIPE0_BLU>>3, row);
      } else if(i <= ((PWR_STRIPE0/2) + (PWR_STRIPE1/2))){
        WS2812_AddColor(PWR_STRIPE1_RED>>3, PWR_STRIPE1_GRN>>3, PWR_STRIPE1_BLU>>3, row);
      } else if(i <= ((PWR_STRIPE0/2) + (PWR_STRIPE1/2) + (PWR_STRIPE2/2))){
        WS2812_AddColor(PWR_STRIPE2_RED>>3, PWR_STRIPE2_GRN>>3, PWR_STRIPE2_BLU>>3, row);
      } else{
        WS2812_AddColor(PWR_STRIPE3_RED>>3, PWR_STRIPE3_GRN>>3, PWR_STRIPE3_BLU>>3, row);
      }
    } else if(i == (litLEDs - 1)){ // LED near end of bar slightly brighter
      if(i <= (PWR_STRIPE0/2)){
        WS2812_AddColor(PWR_STRIPE0_RED>>1, PWR_STRIPE0_GRN>>1, PWR_STRIPE0_BLU>>1, row);
      } else if(i <= ((PWR_STRIPE0/2) + (PWR_STRIPE1/2))){
        WS2812_AddColor(PWR_STRIPE1_RED>>1, PWR_STRIPE1_GRN>>1, PWR_STRIPE1_BLU>>1, row);
      } else if(i <= ((PWR_STRIPE0/2) + (PWR_STRIPE1/2) + (PWR_STRIPE2/2))){
        WS2812_AddColor(PWR_STRIPE2_RED>>1, PWR_STRIPE2_GRN>>1, PWR_STRIPE2_BLU>>1, row);
      } else{
        WS2812_AddColor(PWR_STRIPE3_RED>>1, PWR_STRIPE3_GRN>>1, PWR_STRIPE3_BLU>>1, row);
      }
    } else if(i == litLEDs){       // LED at end of bar full brightness
      if(i <= (PWR_STRIPE0/2)){
        WS2812_AddColor(PWR_STRIPE0_RED, PWR_STRIPE0_GRN, PWR_STRIPE0_BLU, row);
      } else if(i <= ((PWR_STRIPE0/2) + (PWR_STRIPE1/2))){
        WS2812_AddColor(PWR_STRIPE1_RED, PWR_STRIPE1_GRN, PWR_STRIPE1_BLU, row);
      } else if(i <= ((PWR_STRIPE0/2) + (PWR_STRIPE1/2) + (PWR_STRIPE2/2))){
        WS2812_AddColor(PWR_STRIPE2_RED, PWR_STRIPE2_GRN, PWR_STRIPE2_BLU, row);
      } else{
        WS2812_AddColor(PWR_STRIPE3_RED, PWR_STRIPE3_GRN, PWR_STRIPE3_BLU, row);
      }
    } else{                        // LEDs beyond end of bar fully off
      WS2812_AddColor(0, 0, 0, row);
    }
  }
                                   // shut off any remaining LEDs
  for(i=(2*LEDsPerSide); i<NUMCOLS; i=i+1){
    WS2812_AddColor(0, 0, 0, row);
  }
  WS2812_SendReset();
  WS2812_PrintBuffer();
}

//------------Pattern_SplitMeterColored------------
// Implement a power meter by illuminating a given number of
// LEDs, starting in the middle of the strand.  The meter will
// be all one color, according to the number of LEDs
// illuminated.  A fully illuminated colored power meter will be
// fully red.  A partially illuminated colored power meter will
// be fully orange or fully yellow.
// Input: litLEDs number of LEDs to illuminate (litLEDs<=LEDsPerSide<=(NUMCOLS/2))
//        LEDsPerSide number of LEDs on each side of the meter
//        row     index of LED row (0<=row<NUMROWS)
// Output: none
void Pattern_SplitMeterColored(uint8_t litLEDs, uint8_t LEDsPerSide, uint8_t row){
  uint32_t i;
  if(row >= NUMROWS){
    return;                        // row parameter is invalid
  }
  if(LEDsPerSide > (NUMCOLS/2)){   // check that the whole meter fits in the LEDs
    LEDsPerSide = NUMCOLS/2;       // balance the number of LEDs per side if needed
  }
  if(litLEDs > LEDsPerSide){       // check that each half of the meter fits
    litLEDs = LEDsPerSide;         // light all of them
  }
  WS2812_SetCursor(0, row);
  for(i=LEDsPerSide; i>=1; i=i-1){ // draw first half of meter
    if(litLEDs == 0){              // nothing to display; no LEDs on
      WS2812_AddColor(0, 0, 0, row);
    } else if(litLEDs <= (PWR_STRIPE0/2)){
      if(i < (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE0_RED>>3, PWR_STRIPE0_GRN>>3, PWR_STRIPE0_BLU>>3, row);
      } else if(i == (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE0_RED>>1, PWR_STRIPE0_GRN>>1, PWR_STRIPE0_BLU>>1, row);
      } else if(i == litLEDs){
        WS2812_AddColor(PWR_STRIPE0_RED, PWR_STRIPE0_GRN, PWR_STRIPE0_BLU, row);
      } else{
        WS2812_AddColor(0, 0, 0, row);
      }
    } else if(litLEDs <= ((PWR_STRIPE0/2) + (PWR_STRIPE1/2))){
      if(i < (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE1_RED>>3, PWR_STRIPE1_GRN>>3, PWR_STRIPE1_BLU>>3, row);
      } else if(i == (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE1_RED>>1, PWR_STRIPE1_GRN>>1, PWR_STRIPE1_BLU>>1, row);
      } else if(i == litLEDs){
        WS2812_AddColor(PWR_STRIPE1_RED, PWR_STRIPE1_GRN, PWR_STRIPE1_BLU, row);
      } else{
        WS2812_AddColor(0, 0, 0, row);
      }
    } else if(litLEDs <= ((PWR_STRIPE0/2) + (PWR_STRIPE1/2) + (PWR_STRIPE2/2))){
      if(i < (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE2_RED>>3, PWR_STRIPE2_GRN>>3, PWR_STRIPE2_BLU>>3, row);
      } else if(i == (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE2_RED>>1, PWR_STRIPE2_GRN>>1, PWR_STRIPE2_BLU>>1, row);
      } else if(i == litLEDs){
        WS2812_AddColor(PWR_STRIPE2_RED, PWR_STRIPE2_GRN, PWR_STRIPE2_BLU, row);
      } else{
        WS2812_AddColor(0, 0, 0, row);
      }
    } else{
      if(i < (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE3_RED>>3, PWR_STRIPE3_GRN>>3, PWR_STRIPE3_BLU>>3, row);
      } else if(i == (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE3_RED>>1, PWR_STRIPE3_GRN>>1, PWR_STRIPE3_BLU>>1, row);
      } else if(i == litLEDs){
        WS2812_AddColor(PWR_STRIPE3_RED, PWR_STRIPE3_GRN, PWR_STRIPE3_BLU, row);
      } else{
        WS2812_AddColor(0, 0, 0, row);
      }
    }
  }
  for(i=1; i<=LEDsPerSide; i=i+1){ // draw second half of meter
    if(litLEDs == 0){              // nothing to display; no LEDs on
      WS2812_AddColor(0, 0, 0, row);
    } else if(litLEDs <= (PWR_STRIPE0/2)){
      if(i < (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE0_RED>>3, PWR_STRIPE0_GRN>>3, PWR_STRIPE0_BLU>>3, row);
      } else if(i == (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE0_RED>>1, PWR_STRIPE0_GRN>>1, PWR_STRIPE0_BLU>>1, row);
      } else if(i == litLEDs){
        WS2812_AddColor(PWR_STRIPE0_RED, PWR_STRIPE0_GRN, PWR_STRIPE0_BLU, row);
      } else{
        WS2812_AddColor(0, 0, 0, row);
      }
    } else if(litLEDs <= ((PWR_STRIPE0/2) + (PWR_STRIPE1/2))){
      if(i < (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE1_RED>>3, PWR_STRIPE1_GRN>>3, PWR_STRIPE1_BLU>>3, row);
      } else if(i == (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE1_RED>>1, PWR_STRIPE1_GRN>>1, PWR_STRIPE1_BLU>>1, row);
      } else if(i == litLEDs){
        WS2812_AddColor(PWR_STRIPE1_RED, PWR_STRIPE1_GRN, PWR_STRIPE1_BLU, row);
      } else{
        WS2812_AddColor(0, 0, 0, row);
      }
    } else if(litLEDs <= ((PWR_STRIPE0/2) + (PWR_STRIPE1/2) + (PWR_STRIPE2/2))){
      if(i < (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE2_RED>>3, PWR_STRIPE2_GRN>>3, PWR_STRIPE2_BLU>>3, row);
      } else if(i == (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE2_RED>>1, PWR_STRIPE2_GRN>>1, PWR_STRIPE2_BLU>>1, row);
      } else if(i == litLEDs){
        WS2812_AddColor(PWR_STRIPE2_RED, PWR_STRIPE2_GRN, PWR_STRIPE2_BLU, row);
      } else{
        WS2812_AddColor(0, 0, 0, row);
      }
    } else{
      if(i < (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE3_RED>>3, PWR_STRIPE3_GRN>>3, PWR_STRIPE3_BLU>>3, row);
      } else if(i == (litLEDs - 1)){
        WS2812_AddColor(PWR_STRIPE3_RED>>1, PWR_STRIPE3_GRN>>1, PWR_STRIPE3_BLU>>1, row);
      } else if(i == litLEDs){
        WS2812_AddColor(PWR_STRIPE3_RED, PWR_STRIPE3_GRN, PWR_STRIPE3_BLU, row);
      } else{
        WS2812_AddColor(0, 0, 0, row);
      }
    }
  }
                                   // shut off any remaining LEDs
  for(i=(2*LEDsPerSide); i<NUMCOLS; i=i+1){
    WS2812_AddColor(0, 0, 0, row);
  }
  WS2812_SendReset();
  WS2812_PrintBuffer();
}


uint32_t i,run,maxrun;  
void LEDRingTask(){
  WS2812_ClearBuffer();
  WS2812_SendReset();
  WS2812_AddColor(0xAA, 0xAA, 0xAA, 0);
  WS2812_AddColor(64, 0, 0, 0);
  WS2812_AddColor(128, 0, 0, 0);
  WS2812_AddColor(64, 64, 0, 0);
  WS2812_AddColor(0, 64, 0, 0);
  WS2812_AddColor(0, 128, 0, 0);
  WS2812_AddColor(0, 64, 64, 0);
  WS2812_AddColor(0, 0, 64, 0);
  WS2812_AddColor(0, 0, 128, 0);
  WS2812_AddColor(64, 0, 64, 0);
  WS2812_AddColor(32, 0, 32, 0);
  WS2812_AddColor(64, 0, 64, 0);
  WS2812_AddColor(96, 0, 96, 0);
  WS2812_AddColor(128, 0, 128, 0);
  WS2812_AddColor(0, 64, 64, 0);
  WS2812_AddColor(0, 32, 32, 0);   // second LED of row 1 green
  
  WS2812_PrintBuffer();
  OS_Sleep(250);
  while(1){
  
  
//    run = 0;
//    maxrun = 80;
//    Pattern_Reset();
//    WS2812_ClearBuffer();
//    while((SWITCHES == (SW1|SW2) && (run < maxrun))){ // repeat while switches released or for about 20 seconds
//      Pattern_RainbowWaves(1, NUMCOLS, 0);
//      OS_Sleep(250);              // delay about 0.25 sec
//      run = run + 1;
//    }
//    // wait for switch released
    
/*temp long test*/
//Test the dynamic Pattern_RainbowWaves() function with rainbows of
//increasing length.  Every time the button is pressed, the length
//of the rainbow on row 0 increases by 1, and the number of red
//LEDs on row 1 increases by 1.  The button may need to be pressed
//three times before a rainbow actually appears on row 0, since a
//rainbow cannot be formed with fewer than three colors.  A color
//is not necessarily added to the rainbow each time the length of
//the rainbow is incremented.
/*
    i = 0;
    WS2812_SetCursor(0, 1);
    while(i <= 24){
      Pattern_Reset();
      while(SWITCHES == (SW1|SW2)){// repeat while switches released
        Pattern_RainbowWaves(1, i, 0);
        OS_Sleep(250);            // delay about 0.25 sec
      }
      // wait for switch released
      i = i + 1;
      WS2812_AddColor(64, 0, 0, 1);// light another LED on row 1
      WS2812_SendReset();
      WS2812_PrintBuffer();
    }
    */
/*end of temp long test*/

//    run = 0;
//    maxrun = 80;
//    Pattern_Reset();
//    while((run < maxrun)){ // repeat while switches released or for about 20 seconds
//      Pattern_WhiteWaves(2, 0);
//      OS_Sleep(250);              // delay about 0.25 sec
//      run = run + 1;
//    }
//    // wait for switch released
    
    
//    run = 0;
//    maxrun = 200;
//    Pattern_Reset();
//    while((run < maxrun)){ // repeat while switches released or for about 20 seconds
//      Pattern_RedWhiteBlue(1, 0);
//      Delay(2666666);              // delay about 0.1 sec
//      run = run + 1;
//    }
//    // wait for switch released
    
    
//    run = 0;
//    maxrun = 3;
//    while((run < maxrun)){ // repeat while switches released or until sequence completes 3 times
//      Pattern_Reset();
//      WS2812_ClearBuffer();
//      i = 0;
//      while((i < NUMCOLS)){
//        i = i + 1;
//        Pattern_Stacker(255, 0, 0, 80000, 0);
//      }
//      Pattern_Reset();
//      WS2812_ClearBuffer();
//      Random_Init(NVIC_ST_CURRENT_R);
//      i = 0;
//      while((i < NUMCOLS)){
//        i = i + 1;
//        Pattern_Stacker(Random()>>24, Random()>>24, Random()>>24, 266666, 0);
//      }
//      if(i >= NUMCOLS){            // skip the delay if the above loops broke due to button pressed
//        Delay(133333333);          // delay about 5 sec to observe random strand
//      }
//      run = run + 1;
//    }
//    // wait for switch released
    
// squarewave    
    run = 0;
    maxrun = 80;
    Pattern_Reset();
    while(run < maxrun){ // repeat while switches released or for about 20 seconds
      //Pattern_ThreeRowSquare(0, 128, 255, 12, 0, 0);
      Pattern_RainbowWaves(1,24,0);
      OS_Sleep(250);              // delay about 0.25 sec
      run = run + 1;
    }
    run = 0;
    maxrun = 80;
    Pattern_Reset();
    while(run < maxrun){ // repeat while switches released or for about 20 second
      i = 0;
      while(i < NUMCOLS){

        Pattern_PwrMeterBasic(i, 0);
        OS_Sleep(250);              // delay about 0.25 sec
        run = run + 1;
        i=i+1;
      }
    }
    run = 0;
    maxrun = 80;
    Pattern_Reset();
    while(run < maxrun){ // repeat while switches released or for about 20 second
      i = 0;
      while(i < NUMCOLS){
        
        Pattern_PwrMeterStriped(i, 0);
        OS_Sleep(250);              // delay about 0.25 sec
        run = run + 1;
        i=i+1;
      }
    }
    run = 0;
    maxrun = 80;
    Pattern_Reset();
    while(run < maxrun){ // repeat while switches released or for about 20 second
      i = 0;
      while(i < NUMCOLS){
        
        Pattern_PwrMeterColored(i, 0);
        OS_Sleep(250);              // delay about 0.25 sec
        run = run + 1;
        i=i+1;
      }
    }
    
    run = 0;
    maxrun = 80;
    Pattern_Reset();
    while(run < maxrun){ // repeat while switches released or for about 20 seconds
      Pattern_ThreeRowTriangle(255, 255, 0, 0, 0, 12);
      OS_Sleep(250);              // delay about 0.25 sec
      run = run + 1;
    }
   
   
// sine wave    
    i = 0;
    run = 0;
    maxrun = 80;
    Pattern_Reset();
    while(run < maxrun){ // repeat while switches released or for about 20 seconds
      if(i < 25){
        red = 250 - 10*i;
        green = 10*i;
        blue = 0;
        ired = -10;
        igreen = 10;
        iblue = 0;
      } else if(i < 50){
        red = 0;
        green = 250 - 10*(i - 25);
        blue = 10*(i - 25);
        ired = 0;
        igreen = -10;
        iblue = 10;
      } else{
        red = 10*(i - 50);
        green = 0;
        blue = 250 - 10*(i - 50);
        ired = 10;
        igreen = 0;
        iblue = -10;
      }
      i = i + 1;
      if(i > 74){
        i = 0;
      }
      Pattern_ThreeRowSine(red, green, blue, 0, 0, 0);
      red = red + ired;
      green = green + igreen;
      blue = blue + iblue;
      if((red == 250) && (ired > 0)){
        igreen = ired;
        ired = -1*ired;
      }
      if((red == 0) && (ired < 0)){
        ired = 0;
      }
      if((green == 250) && (igreen > 0)){
        iblue = igreen;
        igreen = -1*igreen;
      }
      if((green == 0) && (igreen < 0)){
        igreen = 0;
      }
      if((blue == 250) && (iblue > 0)){
        ired = iblue;
        iblue = -1*iblue;
      }
      if((blue == 0) && (iblue < 0)){
        iblue = 0;
      }
      OS_Sleep(250);              // delay about 0.25 sec
      run = run + 1;
    }
    
    
    run = 0;
    maxrun = 2;
    while((run < maxrun)){ // repeat while switches released or for about 18 seconds
      i = 0;
      while((i < NUMCOLS)){
        Pattern_PwrMeterBasic(i, 0);
        //Pattern_PwrMeterStriped(i, 1);
        //Pattern_PwrMeterColored(i, 2);
        OS_Sleep(125);            // delay about 0.125 sec
        i = i + 1;
      }
      while((i < NUMCOLS)){
        //Pattern_PwrMeterBasic(i, 0);
        Pattern_PwrMeterStriped(i, 0);
        //Pattern_PwrMeterColored(i, 2);
        OS_Sleep(125);            // delay about 0.125 sec
        i = i + 1;
      }
      while((i < NUMCOLS)){
        //Pattern_PwrMeterBasic(i, 0);
        //Pattern_PwrMeterStriped(i, 1);
        Pattern_PwrMeterColored(i, 0);
        OS_Sleep(125);            // delay about 0.125 sec
        i = i + 1;
      }
      i = 36;
      while((i > 0)){
        Pattern_PwrMeterBasic(i, 0);
        Pattern_PwrMeterStriped(i, 1);
        Pattern_PwrMeterColored(i, 2);
        OS_Sleep(125);            // delay about 0.125 sec
        i = i - 1;
      }
      run = run + 1;
    }
    // wait for switch released
    run = 0;
    maxrun = 4;
    while((run < maxrun)){ // repeat while switches released or for about 18 seconds
      i = 0;
      while((i < NUMCOLS/2)){
        Pattern_SplitMeterBasic(i, 18, 0);
        Pattern_SplitMeterStriped(i, 18, 1);
        Pattern_SplitMeterColored(i, 18, 2);
        OS_Sleep(125);            // delay about 0.125 sec
        i = i + 1;
      }
      i = 18;
      while((i > 0)){
        Pattern_SplitMeterBasic(i, 18, 0);
        Pattern_SplitMeterStriped(i, 18, 1);
        Pattern_SplitMeterColored(i, 18, 2);
        OS_Sleep(125);            // delay about 0.125 sec
        i = i - 1;
      }
      run = run + 1;
    }
    // wait for switch released
    
    /*
    run = 0;
    maxrun = 80;
    TIMER1_CTL_R |= 0x0001;        // enable Timer1A 16b, periodic, interrupts
    while((run < maxrun)){ // repeat while switches released or for about 20 seconds
      if(AccX > LastX){            // simple absolute value implementation
        Pattern_SplitMeterColored(((AccX - LastX)>>1), 18, 0);
      } else{
        Pattern_SplitMeterColored(((LastX - AccX)>>1), 18, 0);
      }
      if(AccY > LastY){            // simple absolute value implementation
        Pattern_SplitMeterColored(((AccY - LastY)>>1), 18, 1);
      } else{
        Pattern_SplitMeterColored(((LastY - AccY)>>1), 18, 1);
      }
      if(AccZ > LastZ){            // simple absolute value implementation
        Pattern_SplitMeterColored(((AccZ - LastZ)>>1), 18, 2);
      } else{
        Pattern_SplitMeterColored(((LastZ - AccZ)>>1), 18, 2);
      }
      OS_Sleep(250);              // delay about 0.25 sec
      run = run + 1;
    }
    TIMER1_CTL_R &= ~0x0001;       // disable Timer1A
    // wait for switch released
  */
    }
  }
  
