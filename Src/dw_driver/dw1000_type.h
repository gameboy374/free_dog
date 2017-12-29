#ifndef __DW1000_TYPE_H__
#define __DW1000_TYPE_H__
#include "stm32f1xx.h"
#include "dw1000_regs.h"

typedef enum{
    dwClockAuto = 0x00,
    dwClockXti = 0x01,
    dwClockPll = 0x02
}dwClock_t;

typedef union dwTime_u {
  uint8_t raw[5];
  uint64_t full;
  struct {
    uint32_t low32;
    uint8_t high8;
  } __attribute__((packed));
  struct {
    uint8_t low8;
    uint32_t high32;
  } __attribute__((packed));
} dwTime_t;

struct dwOps_s;
typedef struct {
  struct dwOps_s *ops;
  SPI_HandleTypeDef *handle;
  //state
  unsigned char spiSpeed;
  dwClock_t    dwclock;
  unsigned char *deviceMode;
  //reg
  unsigned int networkAndAddress;   //4byte
  unsigned int syscfg;              //4byte
  unsigned int sysctrl;             //4byte
  unsigned int sysmask;             //4byte

  dwTime_t antennaDelay;
  //RTX
  unsigned short txBufLens;
  unsigned short rxBufLens;
  unsigned char *pTxBuf;
  unsigned char *pRxBuf;
} __attribute__((aligned)) DwDevice_st;

typedef struct dwOps_s{
  /**
   * Function that activates the chip-select, sends header, read data and
   * disable the chip-select.
   */
  void (*spiRead)(DwDevice_st *dev, unsigned short address, unsigned short subIndex,
                                   void *readBuf, unsigned int lens );

  /**
   * Function that activates the chip-select, sends header, sends data and
   * disable the chip-select.
   */
  void (*spiWrite)(DwDevice_st *dev, unsigned short regAddr, unsigned short subIndex,
                                    const void *writeBuf, unsigned int lens );

  /**
   * Sets the SPI bus speed. Take as argument:
   *   - dwSpiSpeedLow: <= 4MHz
   *   - dwSpiSpeedHigh: <= 20MHz
   */
  void (*spiSetSpeed)(DwDevice_st* dev, unsigned char speed);

  /**
   * Waits at least 'delay' miliseconds.
   */
  void (*delayms)(unsigned int delay);

  /**
   * Resets the DW1000 by pulling the reset pin low and then releasing it.
   * This function is optional, if not set softreset via SPI will be used.
   */
   void (*reset)(DwDevice_st *dev);
} dwOps_t;

#endif

