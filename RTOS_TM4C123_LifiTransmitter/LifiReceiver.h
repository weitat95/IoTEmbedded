#include <stdint.h>

//PD0 As logic input
enum receiver_state {
  IDLE, //waiting for sync
  SYNC, //synced, waiting for STX
  START, //STX received
  DATA //receiving DATA
};

void sample_signal_edge(int readValue);
void getDataFrame(void);
void initLifiReceiver(int oversamplingFactor);
void insertEdgeCapture(unsigned long data);
