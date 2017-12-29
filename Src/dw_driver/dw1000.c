#include "dw1000.h"
#include "dw1000_type.h"
#include "..\board_config.h"
#include <string.h>
#include <stdbool.h>

#define DW_MAX_TXBUF              16
#define DW_MAX_RXBUF              16

static unsigned char TX_BUFFER[DW_MAX_TXBUF] = {0};
static unsigned char RX_BUFFER[DW_MAX_RXBUF] = {0};
SPI_HandleTypeDef DW_SPI_HANDLE;
dwOps_t     Ops_t;
DwDevice_st dw_devcie;
static uwbConfig_t config_t = {
  .address = {0,0,0,0,0,0,0xcf,0xbc},
};

static void DW_ReadData(DwDevice_st *dev, unsigned short regAddr, unsigned short subIndex, void *readBuf, unsigned int lens )
{
  unsigned char header[3];
  unsigned int count = 0;

  if (subIndex == 0) {
    header[count++] = (unsigned char)regAddr;
  }
  else {
    header[count++] = 0x40 | (unsigned char)regAddr;
    if (subIndex < 0xF0) {   /* 7-bit, subIndex <= 0x7F */
      header[count++] = (unsigned char)subIndex;
    }
    else {                  /* 15-bit, subIndex <= 0x7FFF, extended address */
      header[count++] = 0x80 | (unsigned char)subIndex;
      header[count++] = (unsigned char)(subIndex >> 7);
    }
  }

  DW_SPI_CS_L();
  HAL_SPI_Transmit(dev->handle, header, count, HAL_MAX_DELAY);
  HAL_SPI_Receive(dev->handle, readBuf, lens, HAL_MAX_DELAY);
  DW_SPI_CS_H();
}

static void DW_WriteData(DwDevice_st *dev, unsigned short regAddr, unsigned short subIndex, const void *writeBuf, unsigned int lens )
{
  unsigned char header[3] = {0};
  unsigned int count = 0;

  if (subIndex == 0) {
    header[count++] = 0x80 | (unsigned char)regAddr;
  }
  else {
    header[count++] = 0xC0 | (unsigned char)regAddr;
    if (subIndex < 0xF0) {   /* 7-bit, subIndex <= 0x7F */
      header[count++] = (unsigned char)subIndex;
    }
    else {                  /* 15-bit, subIndex <= 0x7FFF, extended address */
      header[count++] = 0x80 | (unsigned char)subIndex;
      header[count++] = (unsigned char)(subIndex >> 7);
    }
  }

  DW_SPI_CS_L();
  HAL_SPI_Transmit(dev->handle, header, count, HAL_MAX_DELAY);
  HAL_SPI_Transmit(dev->handle, (unsigned char*)writeBuf, lens, HAL_MAX_DELAY);
  DW_SPI_CS_H();
}

static void DW_Set_SpiSpeed(DwDevice_st *dev, unsigned char speed)
{
    __HAL_SPI_DISABLE(dev->handle);
    dev->handle->Init.BaudRatePrescaler = speed;
    HAL_SPI_Init(dev->handle);
    __HAL_SPI_ENABLE(dev->handle);
}

static void DW_Hardware_Reset(DwDevice_st *dev)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_InitStruct.Pin   = DW_RESET_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DW_RESET_Port, &GPIO_InitStruct);
    DW_RST_L();
    dev->ops->delayms(1);
    
    GPIO_InitStruct.Pin   = DW_RESET_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DW_RESET_Port, &GPIO_InitStruct);
    dev->ops->delayms(5);
}

static unsigned int DW_ReadDeviceID()
{
    unsigned int ID = 0;
    
    dw_devcie.ops->spiRead(&dw_devcie, DEV_ID_ID, 0, (unsigned char*)&ID, sizeof(ID));

    return ID;
}

static void DW_SetClock(dwClock_t clock)
{
    unsigned char pmscctrl0[PMSC_CTRL0_LEN];
    memset(pmscctrl0, 0, PMSC_CTRL0_LEN);
    dw_devcie.ops->spiRead(&dw_devcie, PMSC_ID, PMSC_CTRL0_OFFSET, pmscctrl0, PMSC_CTRL0_LEN);
    if(clock == dwClockAuto) {
    dw_devcie.ops->spiSetSpeed(&dw_devcie, SPI_BAUDRATEPRESCALER_64);
        pmscctrl0[0] = dwClockAuto;
        pmscctrl0[1] &= 0xFE;
    } else if(clock == dwClockXti) {
    dw_devcie.ops->spiSetSpeed(&dw_devcie, SPI_BAUDRATEPRESCALER_64);
        pmscctrl0[0] &= 0xFC;
        pmscctrl0[0] |= dwClockXti;
    } else if(clock == dwClockPll) {
    dw_devcie.ops->spiSetSpeed(&dw_devcie, SPI_BAUDRATEPRESCALER_4);
        pmscctrl0[0] &= 0xFC;
        pmscctrl0[0] |= dwClockPll;
    } else {
      // TODO deliver proper warning
    }
    //refer to demo code
    dw_devcie.ops->spiWrite(&dw_devcie, PMSC_ID, PMSC_CTRL0_OFFSET, pmscctrl0, 1);
    dw_devcie.ops->spiWrite(&dw_devcie, PMSC_ID, PMSC_CTRL0_OFFSET + 1, pmscctrl0, 1);

}

static void DW_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, DW_GPIO5_SPIPOL_Pin|DW_GPIO6_SPIPHA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DW_SPI_CS_Port, DW_SPI_CS_Pin, GPIO_PIN_SET);
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    
    /*Configure GPIO pins : DW_GPIO5_SPIPOL_Pin DW_GPIO6_SPIPHA_Pin DW_CS_Pin*/
    GPIO_InitStruct.Pin = DW_GPIO5_SPIPOL_Pin|DW_GPIO6_SPIPHA_Pin|DW_SPI_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /*Configure GPIO pins : PC6 PC7 */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

static void DW_SPI_Config(void)
{
    /* SPI1 init function */

    /* SPI1 parameter configuration*/
    dw_devcie.handle->Instance = SPI1;
    dw_devcie.handle->Init.Mode = SPI_MODE_MASTER;
    dw_devcie.handle->Init.Direction = SPI_DIRECTION_2LINES;
    dw_devcie.handle->Init.DataSize = SPI_DATASIZE_8BIT;
    dw_devcie.handle->Init.CLKPolarity = SPI_POLARITY_LOW;
    dw_devcie.handle->Init.CLKPhase = SPI_PHASE_1EDGE;
    dw_devcie.handle->Init.NSS = SPI_NSS_SOFT;
    dw_devcie.handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;//SPI_BAUDRATEPRESCALER_2;
    dw_devcie.handle->Init.FirstBit = SPI_FIRSTBIT_MSB;
    dw_devcie.handle->Init.TIMode = SPI_TIMODE_DISABLE;
    dw_devcie.handle->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    dw_devcie.handle->Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(dw_devcie.handle) != HAL_OK)
    {
    _Error_Handler(__FILE__, __LINE__);
    }
}

/************************Register Configurate Start**************************/
static void DW_SetDoubleBuffering(DwDevice_st* dev, bool val)
{
    if(val)
    {
        dev->syscfg &= ~SYS_CFG_DIS_DRXB;
    }
    else
    {
        dev->syscfg |= SYS_CFG_DIS_DRXB;
    }
}

static void DW_SetInterruptPolarity(DwDevice_st* dev, bool val)
{
    if(val)
    {
        dev->syscfg |= SYS_CFG_HIRQ_POL;
    }
    else
    {
        dev->syscfg &= ~SYS_CFG_HIRQ_POL;
    }
}
/************************Register Configurate End**************************/


/************************Register Read/Write Start**************************/
static void DW_WriteSystemConfigurationRegister(DwDevice_st* dev) {
	dev->ops->spiWrite(dev, SYS_CFG_ID, SYS_CFG_OFFSET, dev->syscfg, SYS_CFG_LEN);
}

static void DW_WriteSystemEventMaskRegister(DwDevice_st* dev) {
	dev->ops->spiWrite(dev, SYS_MASK_ID, SYS_MASK_OFFSET, dev->sysmask, SYS_MASK_LEN);
}

/************************Register Read/Write End**************************/
static void DW_Idle(DwDevice_st* dev)
{
   memset(dev->sysctrl, 0, SYS_CTRL_LEN);
   dev->sysctrl[0] |= SYS_CTRL_TRXOFF;
   dev->deviceMode = IDLE_MODE;
   dev->ops->spiWrite(dev, SYS_CTRL_ID, SYS_CTRL_OFFSET, dev->sysctrl, SYS_CTRL_LEN);
}

static void DW_ManageLDE(DwDevice_st* dev) {
	// transfer any ldo tune values
	// uint8_t ldoTune[LEN_OTP_RDAT];
	// readBytesOTP(0x04, ldoTune); // TODO #define
	// if(ldoTune[0] != 0) {
	// 	// TODO tuning available, copy over to RAM: use OTP_LDO bit
	// }
	// tell the chip to load the LDE microcode
	// TODO remove clock-related code (PMSC_CTRL) as handled separately
	unsigned char pmscctrl0[PMSC_CTRL0_LEN];
	unsigned char otpctrl[OTP_CTRL_LEN];
	memset(pmscctrl0, 0, PMSC_CTRL0_LEN);
	memset(otpctrl, 0, OTP_CTRL_LEN);
	dev->ops->spiRead(dev, PMSC_ID, PMSC_CTRL0_OFFSET, pmscctrl0, PMSC_CTRL0_LEN);
	dev->ops->spiRead(dev, OTP_IF_ID, OTP_CTRL, otpctrl, OTP_CTRL_LEN);
	pmscctrl0[0] = 0x01;
	pmscctrl0[1] = 0x03;
	otpctrl[0] = 0x00;
	otpctrl[1] = 0x80;
	dev->ops->spiWrite(dev, PMSC_ID, PMSC_CTRL0_OFFSET, pmscctrl0, PMSC_CTRL0_LEN);
	dev->ops->spiWrite(dev, OTP_IF_ID, OTP_CTRL, otpctrl, OTP_CTRL_LEN);
	dev->ops->delayms(5);
	pmscctrl0[0] = 0x00;
	pmscctrl0[1] = 0x02;
	dev->ops->spiWrite(dev, PMSC_ID, PMSC_CTRL0_OFFSET, pmscctrl0, PMSC_CTRL0_LEN);
}

static void DW_EnableAllLeds(DwDevice_st* dev)
{
  unsigned int reg;

  // Set all 4 GPIO in LED mode
  dev->ops->spiRead(dev, GPIO_CTRL_ID, GPIO_MODE_OFFSET, (unsigned char*)&reg, GPIO_MODE_LEN);
  reg &= ~0x00003FC0ul;
  reg |= 0x00001400ul;      //GPIO2,GPIO3 conf as led
  dev->ops->spiWrite(dev, GPIO_CTRL_ID, GPIO_MODE_OFFSET, &reg, GPIO_MODE_LEN);

  // Enable debounce clock (used to clock the LED blinking)
  dev->ops->spiRead(dev, PMSC_ID, PMSC_CTRL0_OFFSET, (unsigned char*)&reg, PMSC_CTRL0_LEN);
  reg |= 0x00840000ul;
  dev->ops->spiWrite(dev, PMSC_ID, PMSC_CTRL0_OFFSET, &reg, PMSC_CTRL0_LEN);

  // Enable LED blinking and set the rate
  reg = 0x00000110ul;
  dev->ops->spiWrite(dev, PMSC_ID, PMSC_LEDC_OFFSET, &reg, PMSC_LEDC_LEN);

  // Trigger a manual blink of the LEDs for test
  reg |= 0x000c0000ul;
  dev->ops->spiWrite(dev, PMSC_ID, PMSC_LEDC_OFFSET, &reg, PMSC_LEDC_LEN);

  reg &= ~0x000c0000ul;
  dev->ops->spiWrite(dev, PMSC_ID, PMSC_LEDC_OFFSET, &reg, PMSC_LEDC_LEN);
}

static void DW_SoftReset(DwDevice_st* dev)
{
  unsigned char pmscctrl0[PMSC_CTRL0_LEN];

  dev->ops->spiRead(dev, PMSC_ID, PMSC_CTRL0_OFFSET, pmscctrl0, PMSC_CTRL0_LEN);
  pmscctrl0[0] = 0x01;
  dev->ops->spiWrite(dev, PMSC_ID, PMSC_CTRL0_OFFSET, pmscctrl0, PMSC_CTRL0_LEN);
  pmscctrl0[3] = 0x00;
  dev->ops->spiWrite(dev, PMSC_ID, PMSC_CTRL0_OFFSET, pmscctrl0, PMSC_CTRL0_LEN);
  dev->ops->delayms(10);
  pmscctrl0[0] = 0x00;
  pmscctrl0[3] = 0xF0;
  dev->ops->spiWrite(dev, PMSC_ID, PMSC_CTRL0_OFFSET, pmscctrl0, PMSC_CTRL0_LEN);
  // force into idle mode
  DW_Idle(dev);
}
static void DW_Devcie_Init( DwDevice_st *dev )
{
    dev->handle    = &DW_SPI_HANDLE;
    dev->spiSpeed  = SPI_BAUDRATEPRESCALER_64;
    dev->dwclock   = dwClockAuto;
    dev->deviceMode = IDLE_MODE;
    
    memset(&dev->networkAndAddress, 0xff, PANADR_LEN);   //Be the broadcast PAN and SHORT_ADDR ID (0xFFFF)
    memset(&dev->syscfg, 0, SYS_CFG_LEN);
    memset(&dev->sysctrl, 0, SYS_CTRL_LEN);
    dev->ops                = &Ops_t;
    dev->ops->spiRead       = &DW_ReadData;
    dev->ops->spiWrite      = &DW_WriteData;
    dev->ops->delayms       = &HAL_Delay;
    dev->ops->reset         = &DW_Hardware_Reset;
    dev->ops->spiSetSpeed   = &DW_Set_SpiSpeed;

    dev->txBufLens = DW_MAX_TXBUF;
    dev->rxBufLens = DW_MAX_RXBUF;
    dev->pTxBuf    = TX_BUFFER;
    dev->pRxBuf    = RX_BUFFER;
}


static HAL_StatusTypeDef DW_Config( DwDevice_st *dev)
{
    DW_GPIO_Config();
    DW_SPI_Config();

    DW_SetClock(dwClockAuto);
    dev->ops->delayms(5);
    if(dev->ops->reset)
    {
        dev->ops->reset(dev);
    }
    else
    {
        DW_SoftReset(dev);
    }
    
    if(DW1000_DEVICE_ID != DW_ReadDeviceID())
    {
        return HAL_ERROR;
    }
    DW_SetDoubleBuffering(dev, false);              
    DW_SetInterruptPolarity(dev, true);
    DW_WriteSystemConfigurationRegister(dev);
    //clear interrupt flag
    memset(dev->sysmask, 0, SYS_MASK_LEN);
    DW_WriteSystemEventMaskRegister(dev);
    //LDE
    DW_SetClock(dwClockXti);
    dev->ops->delayms(5);
    DW_ManageLDE(dev);
    dev->ops->delayms(5);
    DW_SetClock(dwClockPll);
    dev->ops->delayms(5);

    return HAL_OK;
}

HAL_StatusTypeDef DW_Init(void)
{
    HAL_StatusTypeDef hal_status = HAL_ERROR;

    DW_Devcie_Init(&dw_devcie);
    hal_status = DW_Config(&dw_devcie);
    if(HAL_OK != hal_status)
    {
        return hal_status;
    }
    DW_EnableAllLeds(&dw_devcie);

    
    return hal_status;
}


