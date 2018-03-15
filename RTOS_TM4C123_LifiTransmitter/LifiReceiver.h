#include <stdint.h>
#define OVERSAMPLING 2 //over sampling factor for each manchester symbol
//PD0 As logic input
enum receiver_state {
  IDLE, //waiting for sync
  SYNC, //synced, waiting for STX
  START, //STX received
  DATA //receiving DATA
};

void sample_signal_edge(int readValue);
void getDataFrame(void);
void initLifiReceiver(void);
