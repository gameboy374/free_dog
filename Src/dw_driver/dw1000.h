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

/* Settings */
// transmission/reception bit rate
#define TRX_RATE_110KBPS 0x00
#define TRX_RATE_850KBPS 0x01
#define TRX_RATE_6800KBPS 0x02

// transmission pulse frequency
// 0x00 is 4MHZ, but receiver in DW1000 does not support it (!??)
#define TX_PULSE_FREQ_16MHZ 0x01
#define TX_PULSE_FREQ_64MHZ 0x02

// preamble length (PE + TXPSR bits)
#define TX_PREAMBLE_LEN_64 0x01
#define TX_PREAMBLE_LEN_128 0x05
#define TX_PREAMBLE_LEN_256 0x09
#define TX_PREAMBLE_LEN_512 0x0D
#define TX_PREAMBLE_LEN_1024 0x02
#define TX_PREAMBLE_LEN_1536 0x06
#define TX_PREAMBLE_LEN_2048 0x0A
#define TX_PREAMBLE_LEN_4096 0x03

// PAC size. */
#define PAC_SIZE_8 8
#define PAC_SIZE_16 16
#define PAC_SIZE_32 32
#define PAC_SIZE_64 64

/* channel of operation. */
#define CHANNEL_1 1
#define CHANNEL_2 2
#define CHANNEL_3 3
#define CHANNEL_4 4
#define CHANNEL_5 5
#define CHANNEL_7 7

/* preamble codes. */
#define PREAMBLE_CODE_16MHZ_1 1
#define PREAMBLE_CODE_16MHZ_2 2
#define PREAMBLE_CODE_16MHZ_3 3
#define PREAMBLE_CODE_16MHZ_4 4
#define PREAMBLE_CODE_16MHZ_5 5
#define PREAMBLE_CODE_16MHZ_6 6
#define PREAMBLE_CODE_16MHZ_7 7
#define PREAMBLE_CODE_16MHZ_8 8
#define PREAMBLE_CODE_64MHZ_9 9
#define PREAMBLE_CODE_64MHZ_10 10
#define PREAMBLE_CODE_64MHZ_11 11
#define PREAMBLE_CODE_64MHZ_12 12
#define PREAMBLE_CODE_64MHZ_17 17
#define PREAMBLE_CODE_64MHZ_18 18
#define PREAMBLE_CODE_64MHZ_19 19
#define PREAMBLE_CODE_64MHZ_20 20

// Speed of radio waves [m/s] * timestamp resolution [~15.65ps] of DW1000
#define DISTANCE_OF_RADIO       0.0046917639786159f
#define DISTANCE_OF_RADIO_INV   213.139451293f

#define MAX_TIMEOUT             2000            //unit:ms

HAL_StatusTypeDef DW_Init(void);
void DW_Idle(DwDevice_st* dev);
void DW_GetTransmitTimestamp(DwDevice_st* dev, dwTime_t* time);
void DW_GetReceiveTimestamp(DwDevice_st* dev, dwTime_t* time);
unsigned int DW_GetDataLength(DwDevice_st* dev);
void DW_GetData(DwDevice_st* dev, uint8_t data[], unsigned int n);
void DW_SetData(DwDevice_st* dev, uint8_t data[], unsigned int n);
void DW_NewReceive(DwDevice_st* dev);
void DW_StartReceive(DwDevice_st* dev);
void DW_NewTransmit(DwDevice_st* dev);
void DW_StartTransmit(DwDevice_st* dev);
void DW_WaitForResponse(DwDevice_st* dev, bool val);
void DW_SetDefaults(DwDevice_st* dev);

#endif
