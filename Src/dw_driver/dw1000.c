#include "dw1000.h"
#include "..\board_config.h"
#include <string.h>

/**************/
#define     TWR_ANCHOR              0
#define     TWR_TAG                 1
#define     CURRENT_TAG             TWR_TAG
/**************/

#define DW_MAX_TXBUF              16
#define DW_MAX_RXBUF              16
// Default Mode of operation
const unsigned char MODE_LONGDATA_RANGE_LOWPOWER[] = {TRX_RATE_110KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_2048};
const unsigned char MODE_SHORTDATA_FAST_LOWPOWER[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_128};
const unsigned char MODE_LONGDATA_FAST_LOWPOWER[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_1024};
const unsigned char MODE_SHORTDATA_FAST_ACCURACY[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_128};
const unsigned char MODE_LONGDATA_FAST_ACCURACY[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_1024};
const unsigned char MODE_LONGDATA_RANGE_ACCURACY[] = {TRX_RATE_110KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_2048};

static unsigned int timeout;
static unsigned char TX_BUFFER[DW_MAX_TXBUF] = {0};
static unsigned char RX_BUFFER[DW_MAX_RXBUF] = {0};
SPI_HandleTypeDef DW_SPI_HANDLE;
dwOps_t     Ops_t;
DwDevice_st dw_devcie;
static uwbConfig_t config_t = {
    .mdoe = 0,
    .address = {0,0,0,0,0,0,0xcf,0xbc},
    .anchorListSize = 0,
    .anchors = {0},
    .position = {0.0},
    .positionEnabled = 0,
};
struct {
  uwbAlgorithm_t *algorithm;
  char *name;
} availableAlgorithms[] = {
  {.algorithm = &uwbTwrAnchorAlgorithm, .name = "TWR Anchor"},
  {.algorithm = &uwbTwrTagAlgorithm,    .name = "TWR Tag"},
  //{.algorithm = &uwbSnifferAlgorithm,   .name = "Sniffer"},
  //{.algorithm = &uwbTdoa2Algorithm,     .name = "TDoA Anchor V2"},
  {NULL, NULL},
};
static uwbAlgorithm_t *algorithm = NULL;

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

static void txcallback(DwDevice_st *dev)
{
  timeout = algorithm->onEvent(dev, eventPacketSent);
}

static void rxcallback(DwDevice_st *dev)
{
  timeout = algorithm->onEvent(dev, eventPacketReceived);
}

static void rxTimeoutCallback(DwDevice_st * dev) {
  timeout = algorithm->onEvent(dev, eventReceiveTimeout);
}

static void rxfailedcallback(DwDevice_st *dev) {
  timeout = algorithm->onEvent(dev, eventReceiveFailed);
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

static void DW_UseExtendedFrameLength(DwDevice_st* dev, bool val) {
    if(val)
    {
        dev->extendedFrameLength = FRAME_LENGTH_EXTENDED;
        dev->syscfg |= SYS_CFG_PHR_MODE_11;
    }
    else
    {
        dev->extendedFrameLength = FRAME_LENGTH_NORMAL;
        dev->syscfg &= ~SYS_CFG_PHR_MODE_11;
    }
}

static void DW_UseSmartPower(DwDevice_st* dev, bool smartPower) {
    dev->smartPower = smartPower;
    if(!smartPower)
    {
        dev->syscfg |= SYS_CFG_DIS_STXP;
    }
    else
    {
        dev->syscfg &= ~SYS_CFG_DIS_STXP;
    }
}

static void DW_SuppressFrameCheck(DwDevice_st* dev, bool val) {
	dev->frameCheck = val;
}

static void DW_SetFrameFilter(DwDevice_st* dev, bool val) {
    if(val)
    {
        dev->syscfg |= SYS_CFG_FFE;
    }
    else
    {
        dev->syscfg &= ~SYS_CFG_FFE;
    }
}

static void DW_SetFrameFilterAllowData(DwDevice_st* dev, bool val) {
    if(val)
    {
        dev->syscfg |= SYS_CFG_FFAD;
    }
    else
    {
        dev->syscfg &= ~SYS_CFG_FFAD;
    }
}

static void DW_SetFrameFilterAllowReserved(DwDevice_st* dev, bool val) {
    if(val)
    {
        dev->syscfg |= SYS_CFG_FFAR;
    }
    else
    {
        dev->syscfg &= ~SYS_CFG_FFAR;
    }
}

static void DW_InterruptOnSent(DwDevice_st* dev, bool val) {
    if(val)
    {
        dev->sysmask |= SYS_MASK_MTXFRS;
    }
    else
    {
        dev->sysmask &= ~SYS_MASK_MTXFRS;
    }
}

static void DW_InterruptOnReceived(DwDevice_st* dev, bool val) {
    if(val)
    {
        dev->sysmask |= (SYS_MASK_MRXDFR + SYS_MASK_MRXFCG);
    }
    else
    {
        dev->sysmask &= ~(SYS_MASK_MRXDFR + SYS_MASK_MRXFCG);
    }
}

static void DW_InterruptOnReceiveTimeout(DwDevice_st* dev, bool val) {
    if(val)
    {
        dev->sysmask |= SYS_MASK_MRXRFTO;
    }
    else
    {
        dev->sysmask &= ~SYS_MASK_MRXRFTO;
    }
}

static void DW_InterruptOnReceiveFailed(DwDevice_st* dev, bool val) {
    if(val)
    {
        dev->sysmask |= (SYS_MASK_MRXPHE+SYS_MASK_MRXFCE+SYS_MASK_MRXRFSL+SYS_MASK_MLDEERR);
    }
    else
    {
        dev->sysmask &= ~(SYS_MASK_MRXPHE+SYS_MASK_MRXFCE+SYS_MASK_MRXRFSL+SYS_MASK_MLDEERR);
    }
}

static void DW_InterruptOnReceiveTimestampAvailable(DwDevice_st* dev, bool val) {
    if(val)
    {
        dev->sysmask |= SYS_MASK_MLDEDONE;
    }
    else
    {
        dev->sysmask &= ~SYS_MASK_MLDEDONE;
    }
}

static void DW_InterruptOnAutomaticAcknowledgeTrigger(DwDevice_st* dev, bool val) {
    if(val)
    {
        dev->sysmask |= SYS_MASK_MAAT;
    }
    else
    {
        dev->sysmask &= ~SYS_MASK_MAAT;
    }
}

static void DW_SetReceiverAutoReenable(DwDevice_st* dev, bool val) {
    if(val)
    {
        dev->syscfg |= SYS_CFG_RXAUTR;
    }
    else
    {
        dev->syscfg &= ~SYS_CFG_RXAUTR;
    }
}

static void DW_SetDataRate(DwDevice_st* dev, unsigned char rate) {
	rate &= 0x03;
	dev->txfctrl[1] &= 0x83;
	dev->txfctrl[1] |= (unsigned char)((rate << 5) & 0xFF);
	// special 110kbps flag
	if(rate == TRX_RATE_110KBPS) {
	    dev->syscfg |= SYS_CFG_RXM110K;
	} else {
	    dev->syscfg &= ~SYS_CFG_RXM110K;
	}
	// SFD mode and type (non-configurable, as in Table )
	if(rate == TRX_RATE_6800KBPS) {
	    dev->chanctrl &= ~(CHAN_CTRL_DWSFD+CHAN_CTRL_TNSSFD+CHAN_CTRL_RNSSFD);
	} else {
        dev->chanctrl |= (CHAN_CTRL_DWSFD+CHAN_CTRL_TNSSFD+CHAN_CTRL_RNSSFD);
	}
	unsigned char sfdLength;
	if(rate == TRX_RATE_6800KBPS) {
		sfdLength = 0x08;
	} else if(rate == TRX_RATE_850KBPS) {
		sfdLength = 0x10;
	} else {
		sfdLength = 0x40;
	}
	dev->ops->spiWrite(dev, USR_SFD_ID, USR_SFD_OFFSET, &sfdLength, 1);
	dev->dataRate = rate;
}

static void DW_SetPulseFrequency(DwDevice_st* dev, unsigned char freq) {
    unsigned int temp_freq = freq;
    
	freq &= 0x03;
	dev->txfctrl[2] &= 0xFC;
	dev->txfctrl[2] |= (unsigned char)(freq & 0xFF);
	dev->chanctrl &= ~CHAN_CTRL_RXFPRF_MASK;
	dev->chanctrl |= (temp_freq<<CHAN_CTRL_RXFPRF_SHIFT);
	dev->pulseFrequency = freq;
}

static void DW_SetPreambleLength(DwDevice_st* dev, unsigned char prealen) {
	prealen &= 0x0F;
	dev->txfctrl[2] &= 0xC3;
	dev->txfctrl[2] |= (unsigned char)((prealen << 2) & 0xFF);
	if(prealen == TX_PREAMBLE_LEN_64 || prealen == TX_PREAMBLE_LEN_128) {
		dev->pacSize = PAC_SIZE_8;
	} else if(prealen == TX_PREAMBLE_LEN_256 || prealen == TX_PREAMBLE_LEN_512) {
		dev->pacSize = PAC_SIZE_16;
	} else if(prealen == TX_PREAMBLE_LEN_1024) {
		dev->pacSize = PAC_SIZE_32;
	} else {
		dev->pacSize = PAC_SIZE_64;
	}
	dev->preambleLength = prealen;
}

static void DW_SetChannel(DwDevice_st* dev, unsigned char channel) {
	channel &= 0xF;
	dev->chanctrl = (unsigned int)((channel | (channel << 4)) & 0xFF);
	dev->channel = channel;
}

static void DW_SetPreambleCode(DwDevice_st* dev, unsigned char preacode) {
	preacode &= 0x1F;
	dev->chanctrl[2] &= 0x3F;
	dev->chanctrl[2] |= ((preacode << 6) & 0xFF);
	dev->chanctrl[3] = 0x00;
	dev->chanctrl[3] = ((((preacode >> 2) & 0x07) | (preacode << 3)) & 0xFF);
	dev->preambleCode = preacode;
}

static void DW_Idle(DwDevice_st* dev)
{
   memset(dev->sysctrl, 0, SYS_CTRL_LEN);
   dev->sysctrl[0] |= SYS_CTRL_TRXOFF;
   dev->deviceMode = IDLE_MODE;
   dev->ops->spiWrite(dev, SYS_CTRL_ID, SYS_CTRL_OFFSET, dev->sysctrl, SYS_CTRL_LEN);
}
/************************Register Configurate End**************************/


/************************Register Read/Write Start**************************/
static void DW_ReadNetworkIdAndDeviceAddress(DwDevice_st* dev) {
	dev->ops->spiRead(dev, PANADR_ID, PANADR_ID_OFFSET, &dev->networkAndAddress, PANADR_LEN);
}

static void DW_WriteNetworkIdAndDeviceAddress(DwDevice_st* dev) {
	dev->ops->spiWrite(dev, PANADR_ID, PANADR_ID_OFFSET, &dev->networkAndAddress, PANADR_LEN);
}

static void DW_ReadSystemConfigurationRegister(DwDevice_st* dev) {
	dev->ops->spiRead(dev, SYS_CFG_ID, SYS_CFG_OFFSET, &dev->syscfg, SYS_CFG_LEN);
}

static void DW_WriteSystemConfigurationRegister(DwDevice_st* dev) {
    dev->ops->spiWrite(dev, SYS_CFG_ID, SYS_CFG_OFFSET, &dev->syscfg, SYS_CFG_LEN);
}

static void DW_ReadChannelControlRegister(DwDevice_st* dev) {
	dev->ops->spiRead(dev, CHAN_CTRL_ID, CHAN_CTRL_OFFSET, &dev->chanctrl, CHAN_CTRL_LEN);
}

static void DW_WriteChannelControlRegister(DwDevice_st* dev) {
	dev->ops->spiWrite(dev, CHAN_CTRL_ID, CHAN_CTRL_OFFSET, &dev->chanctrl, CHAN_CTRL_LEN);
}

static void DW_ReadTransmitFrameControlRegister(DwDevice_st* dev) {
	dev->ops->spiRead(dev, TX_FCTRL_ID, TX_FCTRL_OFFSET, dev->txfctrl, TX_FCTRL_LEN);
}

static void DW_WriteTransmitFrameControlRegister(DwDevice_st* dev) {
	dev->ops->spiWrite(dev, TX_FCTRL_ID, TX_FCTRL_OFFSET, dev->txfctrl, TX_FCTRL_LEN);
}

static void DW_ReadSystemEventMaskRegister(DwDevice_st* dev) {
	dev->ops->spiRead(dev, SYS_MASK_ID, SYS_MASK_OFFSET, &dev->sysmask, SYS_MASK_LEN);
}

static void DW_WriteSystemEventMaskRegister(DwDevice_st* dev) {
	dev->ops->spiWrite(dev, SYS_MASK_ID, SYS_MASK_OFFSET, &dev->sysmask, SYS_MASK_LEN);
}

/************************Register Read/Write End**************************/
static void DW_AttachSentHandler(DwDevice_st *dev, dwHandler_t handler)
{
    dev->handleSent = handler;
}

static void DW_AttachReceivedHandler(DwDevice_st *dev, dwHandler_t handler)
{
    dev->handleReceived = handler;
}

static void DW_AttachReceiveTimeoutHandler(DwDevice_st *dev, dwHandler_t handler)
{
    dev->handleReceiveTimeout = handler;
}

static void DW_AttachReceiveFailedHandler(DwDevice_st *dev, dwHandler_t handler)
{
    dev->handleReceiveFailed = handler;
}

static void DW_EnableMode(DwDevice_st *dev, const unsigned char mode[]) {
	DW_SetDataRate(dev, mode[0]);
	DW_SetPulseFrequency(dev, mode[1]);
	DW_SetPreambleLength(dev, mode[2]);
	// TODO add channel and code to mode tuples
	// TODO add channel and code settings with checks (see Table 58)
	DW_SetChannel(dev, CHANNEL_5);
	if(mode[1] == TX_PULSE_FREQ_16MHZ) {
		DW_SetPreambleCode(dev, PREAMBLE_CODE_16MHZ_4);
	} else {
		DW_SetPreambleCode(dev, PREAMBLE_CODE_64MHZ_10);
	}
}

static void DW_GetConfiguration(DwDevice_st* dev) {
	DW_Idle(dev);
	DW_ReadNetworkIdAndDeviceAddress(dev);
	DW_ReadSystemConfigurationRegister(dev);
	DW_ReadChannelControlRegister(dev);
	DW_ReadTransmitFrameControlRegister(dev);
	DW_ReadSystemEventMaskRegister(dev);
}

static void DW_Tune(DwDevice_st *dev) {
	// these registers are going to be tuned/configured
	uint8_t agctune1[LEN_AGC_TUNE1];
	uint8_t agctune2[LEN_AGC_TUNE2];
	uint8_t agctune3[LEN_AGC_TUNE3];
	uint8_t drxtune0b[LEN_DRX_TUNE0b];
	uint8_t drxtune1a[LEN_DRX_TUNE1a];
	uint8_t drxtune1b[LEN_DRX_TUNE1b];
	uint8_t drxtune2[LEN_DRX_TUNE2];
	uint8_t drxtune4H[LEN_DRX_TUNE4H];
	uint8_t ldecfg1[LEN_LDE_CFG1];
	uint8_t ldecfg2[LEN_LDE_CFG2];
	uint8_t lderepc[LEN_LDE_REPC];
	uint8_t txpower[LEN_TX_POWER];
	uint8_t rfrxctrlh[LEN_RF_RXCTRLH];
	uint8_t rftxctrl[LEN_RF_TXCTRL];
	uint8_t tcpgdelay[LEN_TC_PGDELAY];
	uint8_t fspllcfg[LEN_FS_PLLCFG];
	uint8_t fsplltune[LEN_FS_PLLTUNE];
	// uint8_t fsxtalt[LEN_FS_XTALT];
	// AGC_TUNE1
	if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		writeValueToBytes(agctune1, 0x8870, LEN_AGC_TUNE1);
	} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
		writeValueToBytes(agctune1, 0x889B, LEN_AGC_TUNE1);
	} else {
		// TODO proper error/warning handling
	}
	// AGC_TUNE2
	writeValueToBytes(agctune2, 0x2502A907L, LEN_AGC_TUNE2);
	// AGC_TUNE3
	writeValueToBytes(agctune3, 0x0035, LEN_AGC_TUNE3);
	// DRX_TUNE0b (already optimized according to Table 20 of user manual)
	if(dev->dataRate == TRX_RATE_110KBPS) {
		writeValueToBytes(drxtune0b, 0x0016, LEN_DRX_TUNE0b);
	} else if(dev->dataRate == TRX_RATE_850KBPS) {
		writeValueToBytes(drxtune0b, 0x0006, LEN_DRX_TUNE0b);
	} else if(dev->dataRate == TRX_RATE_6800KBPS) {
		writeValueToBytes(drxtune0b, 0x0001, LEN_DRX_TUNE0b);
	} else {
		// TODO proper error/warning handling
	}
	// DRX_TUNE1a
	if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		writeValueToBytes(drxtune1a, 0x0087, LEN_DRX_TUNE1a);
	} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
		writeValueToBytes(drxtune1a, 0x008D, LEN_DRX_TUNE1a);
	} else {
		// TODO proper error/warning handling
	}
	// DRX_TUNE1b
	if(dev->preambleLength ==  TX_PREAMBLE_LEN_1536 || dev->preambleLength ==  TX_PREAMBLE_LEN_2048 ||
			dev->preambleLength ==  TX_PREAMBLE_LEN_4096) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(drxtune1b, 0x0064, LEN_DRX_TUNE1b);
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->preambleLength != TX_PREAMBLE_LEN_64) {
		if(dev->dataRate == TRX_RATE_850KBPS || dev->dataRate == TRX_RATE_6800KBPS) {
			writeValueToBytes(drxtune1b, 0x0020, LEN_DRX_TUNE1b);
		} else {
			// TODO proper error/warning handling
		}
	} else {
		if(dev->dataRate == TRX_RATE_6800KBPS) {
			writeValueToBytes(drxtune1b, 0x0010, LEN_DRX_TUNE1b);
		} else {
			// TODO proper error/warning handling
		}
	}
	// DRX_TUNE2
	if(dev->pacSize == PAC_SIZE_8) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			writeValueToBytes(drxtune2, 0x311A002DL, LEN_DRX_TUNE2);
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			writeValueToBytes(drxtune2, 0x313B006BL, LEN_DRX_TUNE2);
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->pacSize == PAC_SIZE_16) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			writeValueToBytes(drxtune2, 0x331A0052L, LEN_DRX_TUNE2);
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			writeValueToBytes(drxtune2, 0x333B00BEL, LEN_DRX_TUNE2);
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->pacSize == PAC_SIZE_32) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			writeValueToBytes(drxtune2, 0x351A009AL, LEN_DRX_TUNE2);
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			writeValueToBytes(drxtune2, 0x353B015EL, LEN_DRX_TUNE2);
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->pacSize == PAC_SIZE_64) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			writeValueToBytes(drxtune2, 0x371A011DL, LEN_DRX_TUNE2);
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			writeValueToBytes(drxtune2, 0x373B0296L, LEN_DRX_TUNE2);
		} else {
			// TODO proper error/warning handling
		}
	} else {
		// TODO proper error/warning handling
	}
	// DRX_TUNE4H
	if(dev->preambleLength == TX_PREAMBLE_LEN_64) {
		writeValueToBytes(drxtune4H, 0x0010, LEN_DRX_TUNE4H);
	} else {
		writeValueToBytes(drxtune4H, 0x0028, LEN_DRX_TUNE4H);
	}
	// RF_RXCTRLH
	if(dev->channel != CHANNEL_4 && dev->channel != CHANNEL_7) {
		writeValueToBytes(rfrxctrlh, 0xD8, LEN_RF_RXCTRLH);
	} else {
		writeValueToBytes(rfrxctrlh, 0xBC, LEN_RF_RXCTRLH);
	}
	// RX_TXCTRL
	if(dev->channel == CHANNEL_1) {
		writeValueToBytes(rftxctrl, 0x00005C40L, LEN_RF_TXCTRL);
	} else if(dev->channel == CHANNEL_2) {
		writeValueToBytes(rftxctrl, 0x00045CA0L, LEN_RF_TXCTRL);
	} else if(dev->channel == CHANNEL_3) {
		writeValueToBytes(rftxctrl, 0x00086CC0L, LEN_RF_TXCTRL);
	} else if(dev->channel == CHANNEL_4) {
		writeValueToBytes(rftxctrl, 0x00045C80L, LEN_RF_TXCTRL);
	} else if(dev->channel == CHANNEL_5) {
		writeValueToBytes(rftxctrl, 0x001E3FE0L, LEN_RF_TXCTRL);
	} else if(dev->channel == CHANNEL_7) {
		writeValueToBytes(rftxctrl, 0x001E7DE0L, LEN_RF_TXCTRL);
	} else {
		// TODO proper error/warning handling
	}
	// TC_PGDELAY
	if(dev->channel == CHANNEL_1) {
		writeValueToBytes(tcpgdelay, 0xC9, LEN_TC_PGDELAY);
	} else if(dev->channel == CHANNEL_2) {
		writeValueToBytes(tcpgdelay, 0xC2, LEN_TC_PGDELAY);
	} else if(dev->channel == CHANNEL_3) {
		writeValueToBytes(tcpgdelay, 0xC5, LEN_TC_PGDELAY);
	} else if(dev->channel == CHANNEL_4) {
		writeValueToBytes(tcpgdelay, 0x95, LEN_TC_PGDELAY);
	} else if(dev->channel == CHANNEL_5) {
		writeValueToBytes(tcpgdelay, 0xC0, LEN_TC_PGDELAY);
	} else if(dev->channel == CHANNEL_7) {
		writeValueToBytes(tcpgdelay, 0x93, LEN_TC_PGDELAY);
	} else {
		// TODO proper error/warning handling
	}
	// FS_PLLCFG and FS_PLLTUNE
	if(dev->channel == CHANNEL_1) {
		writeValueToBytes(fspllcfg, 0x09000407L, LEN_FS_PLLCFG);
		writeValueToBytes(fsplltune, 0x1E, LEN_FS_PLLTUNE);
	} else if(dev->channel == CHANNEL_2 || dev->channel == CHANNEL_4) {
		writeValueToBytes(fspllcfg, 0x08400508L, LEN_FS_PLLCFG);
		writeValueToBytes(fsplltune, 0x26, LEN_FS_PLLTUNE);
	} else if(dev->channel == CHANNEL_3) {
		writeValueToBytes(fspllcfg, 0x08401009L, LEN_FS_PLLCFG);
		writeValueToBytes(fsplltune, 0x5E, LEN_FS_PLLTUNE);
	} else if(dev->channel == CHANNEL_5 || dev->channel == CHANNEL_7) {
		writeValueToBytes(fspllcfg, 0x0800041DL, LEN_FS_PLLCFG);
		writeValueToBytes(fsplltune, 0xA6, LEN_FS_PLLTUNE);
	} else {
		// TODO proper error/warning handling
	}
	// LDE_CFG1
	writeValueToBytes(ldecfg1, 0xD, LEN_LDE_CFG1);
	// LDE_CFG2
	if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		writeValueToBytes(ldecfg2, 0x1607, LEN_LDE_CFG2);
	} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
		writeValueToBytes(ldecfg2, 0x0607, LEN_LDE_CFG2);
	} else {
		// TODO proper error/warning handling
	}
	// LDE_REPC
	if(dev->preambleCode == PREAMBLE_CODE_16MHZ_1 || dev->preambleCode == PREAMBLE_CODE_16MHZ_2) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x5998 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x5998, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_16MHZ_3 || dev->preambleCode == PREAMBLE_CODE_16MHZ_8) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x51EA >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x51EA, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_16MHZ_4) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x428E >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x428E, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_16MHZ_5) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x451E >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x451E, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_16MHZ_6) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x2E14 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x2E14, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_16MHZ_7) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x8000 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x8000, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_9) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x28F4 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x28F4, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_10 || dev->preambleCode == PREAMBLE_CODE_64MHZ_17) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x3332 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x3332, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_11) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x3AE0 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x3AE0, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_12) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x3D70 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x3D70, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_18 || dev->preambleCode == PREAMBLE_CODE_64MHZ_19) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x35C2 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x35C2, LEN_LDE_REPC);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_20) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x47AE >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x47AE, LEN_LDE_REPC);
		}
	} else {
		// TODO proper error/warning handling
	}
	// TX_POWER (enabled smart transmit power control)
	if(dev->channel == CHANNEL_1 || dev->channel == CHANNEL_2) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x15355575L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x75757575L, LEN_TX_POWER);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x07274767L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x67676767L, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->channel == CHANNEL_3) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x0F2F4F6FL, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x6F6F6F6FL, LEN_TX_POWER);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x2B4B6B8BL, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x8B8B8B8BL, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->channel == CHANNEL_4) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x1F1F3F5FL, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x5F5F5F5FL, LEN_TX_POWER);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x3A5A7A9AL, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x9A9A9A9AL, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->channel == CHANNEL_5) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x0E082848L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x48484848L, LEN_TX_POWER);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x25456585L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x85858585L, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->channel == CHANNEL_7) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x32527292L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x92929292L, LEN_TX_POWER);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x5171B1D1L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0xD1D1D1D1L, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else {
		// TODO proper error/warning handling
	}
	// mid range XTAL trim (TODO here we assume no calibration data available in OTP)
	//writeValueToBytes(fsxtalt, 0x60, LEN_FS_XTALT);
	// write configuration back to chip
	dwSpiWrite(dev, AGC_TUNE, AGC_TUNE1_SUB, agctune1, LEN_AGC_TUNE1);
	dwSpiWrite(dev, AGC_TUNE, AGC_TUNE2_SUB, agctune2, LEN_AGC_TUNE2);
	dwSpiWrite(dev, AGC_TUNE, AGC_TUNE3_SUB, agctune3, LEN_AGC_TUNE3);
	dwSpiWrite(dev, DRX_TUNE, DRX_TUNE0b_SUB, drxtune0b, LEN_DRX_TUNE0b);
	dwSpiWrite(dev, DRX_TUNE, DRX_TUNE1a_SUB, drxtune1a, LEN_DRX_TUNE1a);
	dwSpiWrite(dev, DRX_TUNE, DRX_TUNE1b_SUB, drxtune1b, LEN_DRX_TUNE1b);
	dwSpiWrite(dev, DRX_TUNE, DRX_TUNE2_SUB, drxtune2, LEN_DRX_TUNE2);
	dwSpiWrite(dev, DRX_TUNE, DRX_TUNE4H_SUB, drxtune4H, LEN_DRX_TUNE4H);
	dwSpiWrite(dev, LDE_IF, LDE_CFG1_SUB, ldecfg1, LEN_LDE_CFG1);
	dwSpiWrite(dev, LDE_IF, LDE_CFG2_SUB, ldecfg2, LEN_LDE_CFG2);
	dwSpiWrite(dev, LDE_IF, LDE_REPC_SUB, lderepc, LEN_LDE_REPC);
	dwSpiWrite(dev, TX_POWER, NO_SUB, txpower, LEN_TX_POWER);
	dwSpiWrite(dev, RF_CONF, RF_RXCTRLH_SUB, rfrxctrlh, LEN_RF_RXCTRLH);
	dwSpiWrite(dev, RF_CONF, RF_TXCTRL_SUB, rftxctrl, LEN_RF_TXCTRL);
	dwSpiWrite(dev, TX_CAL, TC_PGDELAY_SUB, tcpgdelay, LEN_TC_PGDELAY);
	dwSpiWrite(dev, FS_CTRL, FS_PLLTUNE_SUB, fsplltune, LEN_FS_PLLTUNE);
	dwSpiWrite(dev, FS_CTRL, FS_PLLCFG_SUB, fspllcfg, LEN_FS_PLLCFG);
	//dwSpiWrite(dev, FS_CTRL, FS_XTALT_SUB, fsxtalt, LEN_FS_XTALT);
}
static void DW_CommitConfiguration(DwDevice_st* dev) {
	// write all configurations back to device
	DW_WriteNetworkIdAndDeviceAddress(dev);
	DW_WriteSystemConfigurationRegister(dev);
	DW_WriteChannelControlRegister(dev);
	DW_WriteTransmitFrameControlRegister(dev);
	DW_WriteSystemEventMaskRegister(dev);
	// tune according to configuration
	DW_Tune(dev);
	// TODO clean up code + antenna delay/calibration API
	// TODO setter + check not larger two bytes integer
	// uint8_t antennaDelayBytes[LEN_STAMP];
	// writeValueToBytes(antennaDelayBytes, 16384, LEN_STAMP);
	// dev->antennaDelay.setTimestamp(antennaDelayBytes);
	// dwSpiRead(dev, TX_ANTD, NO_SUB, antennaDelayBytes, LEN_TX_ANTD);
  // dwSpiRead(dev, LDE_IF, LDE_RXANTD_SUB, antennaDelayBytes, LEN_LDE_RXANTD);
  dwSpiWrite(dev, TX_ANTD, NO_SUB, dev->antennaDelay.raw, LEN_TX_ANTD);
  dwSpiWrite(dev, LDE_IF, LDE_RXANTD_SUB, dev->antennaDelay.raw, LEN_LDE_RXANTD);
}

static void DW_SetDefaults(DwDevice_st* dev) {
	if(dev->deviceMode == TX_MODE) {

	} else if(dev->deviceMode == RX_MODE) {

	} else if(dev->deviceMode == IDLE_MODE) {
        DW_UseExtendedFrameLength(dev, false);
        DW_UseSmartPower(dev, false);
        DW_SuppressFrameCheck(dev, false);
        //for global frame filtering
        DW_SetFrameFilter(dev, false);
        //for data frame (poll, poll_ack, range, range report, range failed) filtering
        DW_SetFrameFilterAllowData(dev, false);
        //for reserved (blink) frame filtering
        DW_SetFrameFilterAllowReserved(dev, false);
        
        DW_InterruptOnSent(dev, true);
        DW_InterruptOnReceived(dev, true);
        DW_InterruptOnReceiveTimeout(dev, true);
        DW_InterruptOnReceiveFailed(dev, false);
        DW_InterruptOnReceiveTimestampAvailable(dev, false);
        DW_InterruptOnAutomaticAcknowledgeTrigger(dev, false);
        DW_SetReceiverAutoReenable(dev, true);
        // default mode when powering up the chip
        // still explicitly selected for later tuning
        DW_EnableMode(dev, MODE_LONGDATA_RANGE_LOWPOWER);
	}
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
    dev->extendedFrameLength = FRAME_LENGTH_NORMAL;
    dev->smartPower = false;
    dev->frameCheck = true;
    dev->dataRate = TRX_RATE_6800KBPS;
    dev->pulseFrequency = TX_PULSE_FREQ_16MHZ;
    dev->preambleLength = TX_PREAMBLE_LEN_128;
    dev->pacSize = PAC_SIZE_8;

    memset(&dev->networkAndAddress, 0xff, PANADR_LEN);   //Be the broadcast PAN and SHORT_ADDR ID (0xFFFF)
    memset(&dev->syscfg, 0, SYS_CFG_LEN);
    memset(&dev->sysctrl, 0, SYS_CTRL_LEN);
    memset(&dev->sysmask, 0, SYS_MASK_LEN);              //clear interrupt flag
    memset(&dev->chanctrl, 0, CHAN_CTRL_LEN);
    memset(dev->txfctrl, 0, TX_FCTRL_LEN);

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

    if(CURRENT_TAG == TWR_TAG)
    {
        //TODO:configurate config_t
    }
    else if(CURRENT_TAG == TWR_ANCHOR)
    {
        //TODO:configurate config_ts
    }
    algorithm = availableAlgorithms[CURRENT_TAG].algorithm;

    DW_AttachSentHandler(&dw_devcie, txcallback);
    DW_AttachReceivedHandler(&dw_devcie, rxcallback);
    DW_AttachReceiveTimeoutHandler(&dw_devcie, rxTimeoutCallback);
    DW_AttachReceiveFailedHandler(&dw_devcie, rxfailedcallback);

    DW_GetConfiguration(&dw_devcie);
    DW_SetDefaults(&dw_devcie);
    DW_EnableMode(&dw_devcie, MODE_SHORTDATA_FAST_ACCURACY);
    DW_SetChannel(&dw_devcie, CHANNEL_2);
    DW_UseSmartPower(&dw_devcie, true);
    DW_SetPreambleCode(&dw_devcie, PREAMBLE_CODE_64MHZ_9);

    

    return hal_status;
}


