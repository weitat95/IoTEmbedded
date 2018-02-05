#include <stdint.h>
#define OVERSAMPLING 4 //over sampling factor for each manchester symbol

enum receiver_state {
  IDLE, //waiting for sync
  SYNC, //synced, waiting for STX
  START, //STX received
  DATA //receiving DATA
};

void sample_signal_edge(int readValue);
void getDataFrame(void);
void initLifiReceiver(void);
