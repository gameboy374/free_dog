#ifndef __DW1000_H__
#define __DW1000_H__

#include "stm32f1xx.h"
#include "dw1000_type.h"

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

typedef enum uwbEvent_e {
  eventTimeout,
  eventPacketReceived,
  eventPacketSent,
  eventReceiveTimeout,
  eventReceiveFailed,
} uwbEvent_t;

// Callback for one uwb algorithm
typedef struct uwbAlgorithm_s {
  void (*init)(uwbConfig_t * config, DwDevice_st *dev);
  unsigned int (*onEvent)(DwDevice_st *dev, uwbEvent_t event);
} uwbAlgorithm_t;

// enum to determine RX or TX mode of device
#define IDLE_MODE 0x00
#define RX_MODE 0x01
#define TX_MODE 0x02

/* frame length settings. */
#define FRAME_LENGTH_NORMAL 0x00
#define FRAME_LENGTH_EXTENDED 0x03

HAL_StatusTypeDef DW_Init(void);

#endif
