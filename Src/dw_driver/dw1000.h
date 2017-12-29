#ifndef __DW1000_H__
#define __DW1000_H__

#include "stm32f1xx.h"

#define DW1000_DEVICE_ID        ((unsigned int)0xDECA0130)
#define MAX_ANCHORS             6

typedef struct uwbConfig_s {
  unsigned char mode;
  unsigned char address[8];
  unsigned char anchorListSize;
  unsigned char anchors[MAX_ANCHORS];
  float position[3];
  float positionEnabled;
} uwbConfig_t;

// enum to determine RX or TX mode of device
#define IDLE_MODE 0x00
#define RX_MODE 0x01
#define TX_MODE 0x02

HAL_StatusTypeDef DW_Init(void);

#endif
