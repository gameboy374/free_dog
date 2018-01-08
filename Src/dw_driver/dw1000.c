#include "dw1000.h"
#include "..\board_config.h"
#include <string.h>
#include <math.h>

/**************/
#define     TWR_ANCHOR              0
#define     TWR_TAG                 1
#define     CURRENT_TAG             TWR_TAG
/**************/
/**** DW1000 interrupt handling *****/
#define DW_IRQn                    EXTI3_IRQn
#define DW_IRQ_PORT                GPIOA
#define DW_IRQ_PIN                 GPIO_PIN_3
/**** DW1000 TX RX BUF Length *****/
#define DW_MAX_TXBUF                16
#define DW_MAX_RXBUF                16

static const uint8_t BIAS_500_16_ZERO = 10;
static const uint8_t BIAS_500_64_ZERO = 8;
static const uint8_t BIAS_900_16_ZERO = 7;
static const uint8_t BIAS_900_64_ZERO = 7;
// range bias tables (500 MHz in [mm] and 900 MHz in [2mm] - to fit into bytes)
static const uint8_t BIAS_500_16[] = {198, 187, 179, 163, 143, 127, 109, 84, 59, 31,   0,  36,  65,  84,  97, 106, 110, 112};
static const uint8_t BIAS_500_64[] = {110, 105, 100,  93,  82,  69,  51, 27,  0, 21,  35,  42,  49,  62,  71,  76,  81,  86};
static const uint8_t BIAS_900_16[] = {137, 122, 105, 88, 69,  47,  25,  0, 21, 48, 79, 105, 127, 147, 160, 169, 178, 197};
static const uint8_t BIAS_900_64[] = {147, 133, 117, 99, 75, 50, 29,  0, 24, 45, 63, 76, 87, 98, 116, 122, 132, 142};

// Default Mode of operation
const unsigned char MODE_LONGDATA_RANGE_LOWPOWER[] = {TRX_RATE_110KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_2048};
const unsigned char MODE_SHORTDATA_FAST_LOWPOWER[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_128};
const unsigned char MODE_LONGDATA_FAST_LOWPOWER[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_1024};
const unsigned char MODE_SHORTDATA_FAST_ACCURACY[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_128};
const unsigned char MODE_LONGDATA_FAST_ACCURACY[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_1024};
const unsigned char MODE_LONGDATA_RANGE_ACCURACY[] = {TRX_RATE_110KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_2048};

static uint32_t dw_event_timeout = 0;
static bool irq_flag = false;
static unsigned char TX_BUFFER[DW_MAX_TXBUF] = {0};
static unsigned char RX_BUFFER[DW_MAX_RXBUF] = {0};
SPI_HandleTypeDef DW_SPI_HANDLE;
dwOps_t     Ops_t;
DwDevice_st dw_devcie;
static uwbConfig_t config_t = {
    .mode = 0,
    .address = {0,0,0,0,0,0,0xcf,0xbc},
    .anchorListSize = 0,
    .anchors = {0},
    .position = {0.0},
    .positionEnabled = 0,
};

extern uwbAlgorithm_t uwbTwrAnchorAlgorithm;
extern uwbAlgorithm_t uwbTwrTagAlgorithm;

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

static void writeValueToBytes(unsigned char data[], long val, unsigned int n) {
	unsigned int i;
	for(i = 0; i < n; i++) {
		data[i] = ((val >> (i * 8)) & 0xFF);
	}
}

struct uwbConfig_s * DW_GetConfig(void)
{
  return &config_t;
}

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

    /*Configure GPIO pin : PA3 */
    GPIO_InitStruct.Pin = DW_IRQ_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(DW_IRQ_PORT, &GPIO_InitStruct);
}

static void DW_NVIC_Config(void)
{
    HAL_NVIC_SetPriority(DW_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(DW_IRQn);
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
  dw_event_timeout = algorithm->onEvent(dev, eventPacketSent);
}

static void rxcallback(DwDevice_st *dev)
{
  dw_event_timeout = algorithm->onEvent(dev, eventPacketReceived);
}

static void rxTimeoutCallback(DwDevice_st * dev) {
  dw_event_timeout = algorithm->onEvent(dev, eventReceiveTimeout);
}

static void rxfailedcallback(DwDevice_st *dev) {
  dw_event_timeout = algorithm->onEvent(dev, eventReceiveFailed);
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

static void DW_SetChannel(DwDevice_st* dev, unsigned char ch) {
	ch &= 0xF;
	dev->chanctrl = (unsigned int)((ch | (ch << 4)) & 0xFF);
	dev->channel = ch;
}

static void DW_SetPreambleCode(DwDevice_st* dev, unsigned char preacode) {
	uint32_t temp_preacode = 0;
	temp_preacode = (preacode & 0x1F);
    dev->chanctrl &= ~(CHAN_CTRL_TX_PCOD_MASK + CHAN_CTRL_RX_PCOD_MASK);
	dev->chanctrl |= ((temp_preacode<<CHAN_CTRL_TX_PCOD_SHIFT)+(temp_preacode<<CHAN_CTRL_RX_PCOD_SHIFT));
	dev->preambleCode = preacode;
}

void DW_Idle(DwDevice_st* dev)
{
   memset(&dev->sysctrl, 0, SYS_CTRL_LEN);
   dev->sysctrl |= SYS_CTRL_TRXOFF;
   dev->deviceMode = IDLE_MODE;
   dev->ops->spiWrite(dev, SYS_CTRL_ID, SYS_CTRL_OFFSET, &dev->sysctrl, SYS_CTRL_LEN);
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

static void DW_ReadSystemEventStatusRegister(DwDevice_st* dev) {
	dev->ops->spiRead(dev, SYS_STATUS_ID, SYS_STATUS_OFFSET, dev->sysstatus, SYS_STATUS_LEN);
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

/*************************Operate IC state Start**************************/
static bool DW_IsClockProblem(DwDevice_st* dev) {
	bool clkllErr, rfllErr;

	clkllErr = dev->sysstatus[3]&(SYS_STATUS_CLKPLL_LL>>24);
	rfllErr = dev->sysstatus[3]&(SYS_STATUS_RFPLL_LL>>24);
	if(clkllErr || rfllErr) {
		return true;
	}
	return false;
}

static bool DW_IsTransmitDone(DwDevice_st* dev) {
	return dev->sysstatus[0]&SYS_STATUS_TXFRS;
}

static void DW_ClearTransmitStatus(DwDevice_st* dev) {
	// clear latched TX bits
    uint8_t reg[SYS_STATUS_LEN] = {0};
    reg[0] |= (SYS_STATUS_TXFRB+SYS_STATUS_TXPRS+SYS_STATUS_TXPHS+SYS_STATUS_TXFRS);
	dev->ops->spiWrite(dev, SYS_STATUS_ID, SYS_STATUS_OFFSET, reg, SYS_STATUS_LEN);
}

static bool DW_IsReceiveTimestampAvailable(DwDevice_st* dev) {
	return dev->sysstatus[1]&(SYS_STATUS_LDEDONE>>8);
}

static void DW_ClearReceiveTimestampAvailableStatus(DwDevice_st* dev) {
    uint8_t reg[SYS_STATUS_LEN] = {0};
    reg[1] |= (SYS_STATUS_LDEDONE>>8);
	dev->ops->spiWrite(dev, SYS_STATUS_ID, SYS_STATUS_OFFSET, reg, SYS_STATUS_LEN);
}

static bool DW_IsReceiveFailed(DwDevice_st *dev) {
	bool ldeErr, rxCRCErr, rxHeaderErr, rxDecodeErr;

    ldeErr = dev->sysstatus[1]&(SYS_STATUS_RXPHE>>8);
    rxCRCErr = dev->sysstatus[1]&(SYS_STATUS_RXFCE>>8);
    rxHeaderErr = dev->sysstatus[2]&(SYS_STATUS_RXRFSL>>16);
    rxDecodeErr = dev->sysstatus[2]&(SYS_STATUS_LDEERR>>16);
	if(ldeErr || rxCRCErr || rxHeaderErr || rxDecodeErr) {
		return true;
	}
	return false;
}

static void DW_ClearReceiveStatus(DwDevice_st* dev) {
	// clear latched RX bits (i.e. write 1 to clear)
    uint8_t reg[SYS_STATUS_LEN] = {0};
    reg[1] |= ((SYS_STATUS_LDEDONE+SYS_STATUS_RXPHE+SYS_STATUS_RXDFR+SYS_STATUS_RXFCG+SYS_STATUS_RXFCE)>>8);
    reg[2] |= ((SYS_STATUS_RXRFSL+SYS_STATUS_RXRFTO+SYS_STATUS_LDEERR)>>16);
	dev->ops->spiWrite(dev, SYS_STATUS_ID, SYS_STATUS_OFFSET, reg, SYS_STATUS_LEN);
}

static bool DW_IsReceiveTimeout(DwDevice_st* dev) {
	return dev->sysstatus[2]&(SYS_STATUS_RXRFTO>>16);
}

static bool DW_IsReceiveDone(DwDevice_st* dev) {
	if(dev->frameCheck) {
		return dev->sysstatus[1]&(SYS_STATUS_RXFCG>>8);
	}
	return dev->sysstatus[1]&(SYS_STATUS_RXDFR>>8);
}
/*************************Operate IC state Start**************************/
static float calculatePower(float base, float N, uint8_t pulseFrequency) {
  float A, corrFac;

	if(TX_PULSE_FREQ_16MHZ == pulseFrequency) {
		A = 115.72f;
		corrFac = 2.3334f;
	} else {
		A = 121.74f;
		corrFac = 1.1667f;
	}

	float estFpPwr = 10.0f * log10f(base / (N * N)) - A;

	if(estFpPwr <= -88) {
		return estFpPwr;
	} else {
		// approximation of Fig. 22 in user manual for dbm correction
		estFpPwr += (estFpPwr + 88) * corrFac;
	}

	return estFpPwr;
}

float DW_GetReceivePower(DwDevice_st* dev) {
    uint16_t C = 0;
	uint8_t rxFrameInfo[RX_FINFO_LEN];

    dev->ops->spiRead(dev, RX_FQUAL_ID, CIR_PWR_SUB, &C, LEN_CIR_PWR);
    dev->ops->spiRead(dev, RX_FINFO_ID, RX_FINFO_OFFSET, rxFrameInfo, RX_FINFO_LEN);
    uint32_t N = (((unsigned int)rxFrameInfo[2] >> 4) & 0xFF) | ((unsigned int)rxFrameInfo[3] << 4);

    float twoPower17 = 131072.0f;

    return calculatePower(((float)C )* twoPower17, (float)N, dev->pulseFrequency);
}

void DW_CorrectTimestamp(DwDevice_st* dev, dwTime_t* timestamp) {
	// base line dBm, which is -61, 2 dBm steps, total 18 data points (down to -95 dBm)
	float rxPowerBase = -(DW_GetReceivePower(dev) + 61.0f) * 0.5f;
	if (!isfinite(rxPowerBase)) {
	  return;
	}
	int rxPowerBaseLow = (int)rxPowerBase;
	int rxPowerBaseHigh = rxPowerBaseLow + 1;
	if(rxPowerBaseLow < 0) {
		rxPowerBaseLow = 0;
		rxPowerBaseHigh = 0;
	} else if(rxPowerBaseHigh > 17) {
		rxPowerBaseLow = 17;
		rxPowerBaseHigh = 17;
	}
	// select range low/high values from corresponding table
	int rangeBiasHigh = 0;
	int rangeBiasLow = 0;
	if(dev->channel == CHANNEL_4 || dev->channel == CHANNEL_7) {
		// 900 MHz receiver bandwidth
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_900_16_ZERO ? -BIAS_900_16[rxPowerBaseHigh] : BIAS_900_16[rxPowerBaseHigh]);
			rangeBiasHigh <<= 1;
			rangeBiasLow = (rxPowerBaseLow < BIAS_900_16_ZERO ? -BIAS_900_16[rxPowerBaseLow] : BIAS_900_16[rxPowerBaseLow]);
			rangeBiasLow <<= 1;
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_900_64_ZERO ? -BIAS_900_64[rxPowerBaseHigh] : BIAS_900_64[rxPowerBaseHigh]);
			rangeBiasHigh <<= 1;
			rangeBiasLow = (rxPowerBaseLow < BIAS_900_64_ZERO ? -BIAS_900_64[rxPowerBaseLow] : BIAS_900_64[rxPowerBaseLow]);
			rangeBiasLow <<= 1;
		} else {
			// TODO proper error handling
		}
	} else {
		// 500 MHz receiver bandwidth
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_500_16_ZERO ? -BIAS_500_16[rxPowerBaseHigh] : BIAS_500_16[rxPowerBaseHigh]);
			rangeBiasLow = (rxPowerBaseLow < BIAS_500_16_ZERO ? -BIAS_500_16[rxPowerBaseLow] : BIAS_500_16[rxPowerBaseLow]);
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_500_64_ZERO ? -BIAS_500_64[rxPowerBaseHigh] : BIAS_500_64[rxPowerBaseHigh]);
			rangeBiasLow = (rxPowerBaseLow < BIAS_500_64_ZERO ? -BIAS_500_64[rxPowerBaseLow] : BIAS_500_64[rxPowerBaseLow]);
		} else {
			// TODO proper error handling
		}
	}
	// linear interpolation of bias values
	float rangeBias = rangeBiasLow + (rxPowerBase - rxPowerBaseLow) * (rangeBiasHigh - rangeBiasLow);
	// range bias [mm] to timestamp modification value conversion
	dwTime_t adjustmentTime;
    adjustmentTime.full = (int)(rangeBias * DISTANCE_OF_RADIO_INV * 0.001f);
	// apply correction
	timestamp->full += adjustmentTime.full;
}


void DW_GetTransmitTimestamp(DwDevice_st* dev, dwTime_t* time) {
	dev->ops->spiRead(dev, TX_TIME_ID, TX_TIME_TX_STAMP_OFFSET, time->raw, TX_TIME_TX_STAMP_LEN);
}

void DW_GetReceiveTimestamp(DwDevice_st* dev, dwTime_t* time) {
    time->full = 0;
	dev->ops->spiRead(dev, RX_TIME_ID, RX_TIME_RX_STAMP_OFFSET, time->raw, RX_TIME_RX_STAMP_LEN);
	// correct timestamp (i.e. consider range bias)
	DW_CorrectTimestamp(dev, time);
}

unsigned int DW_GetDataLength(DwDevice_st* dev) {
	unsigned int len = 0;
	if(dev->deviceMode == TX_MODE) {
		// 10 bits of TX frame control register
		len = ((((unsigned int)dev->txfctrl[1] << 8) | (unsigned int)dev->txfctrl[0]) & 0x03FF);
	} else if(dev->deviceMode == RX_MODE) {
		// 10 bits of RX frame control register
		uint8_t rxFrameInfo[RX_FINFO_LEN];
		dev->ops->spiRead(dev, RX_FINFO_ID, RX_FINFO_OFFSET, rxFrameInfo, RX_FINFO_LEN);
		len = ((((unsigned int)rxFrameInfo[1] << 8) | (unsigned int)rxFrameInfo[0]) & 0x03FF);
	}
	if(dev->frameCheck && len > 2) {
		return len-2;
	}
	return len;
}

void DW_GetData(DwDevice_st* dev, uint8_t data[], unsigned int n) {
	if(n <= 0) {
		return;
	}
	dev->ops->spiRead(dev, RX_BUFFER_ID, RX_BUFFER_OFFSET, data, n);
}

void DW_SetData(DwDevice_st* dev, uint8_t data[], unsigned int n) {
	if(dev->frameCheck) {
		n+=2; // two bytes CRC-16
	}
	if(n > LEN_EXT_UWB_FRAMES) {
		return; // TODO proper error handling: frame/buffer size
	}
	if(n > LEN_UWB_FRAMES && !dev->extendedFrameLength) {
		return; // TODO proper error handling: frame/buffer size
	}
	// transmit data and length
	dev->ops->spiWrite(dev, TX_BUFFER_ID, TX_BUFFER_OFFSE, data, n);
	dev->txfctrl[0] = (uint8_t)(n & 0xFF); // 1 byte (regular length + 1 bit)
	dev->txfctrl[1] &= 0xE0;
	dev->txfctrl[1] |= (uint8_t)((n >> 8) & 0x03);	// 2 added bits if extended length
}

void DW_NewReceive(DwDevice_st* dev) {
	DW_Idle(dev);
	memset(&dev->sysctrl, 0, SYS_CTRL_LEN);
	DW_ClearReceiveStatus(dev);
	dev->deviceMode = RX_MODE;
}

void DW_StartReceive(DwDevice_st* dev) {
    if(dev->frameCheck)
    {
        dev->sysctrl &= ~SYS_CTRL_SFCST;
    }
    else
    {
        dev->sysctrl |= SYS_CTRL_SFCST;
    }
    dev->sysctrl |= SYS_CTRL_RXENAB;
	dev->ops->spiWrite(dev, SYS_CTRL_ID, SYS_CTRL_OFFSET, &dev->sysctrl, SYS_CTRL_LEN);
}

void DW_NewTransmit(DwDevice_st* dev) {
	DW_Idle(dev);
	memset(&dev->sysctrl, 0, SYS_CTRL_LEN);
	DW_ClearTransmitStatus(dev);
	dev->deviceMode = TX_MODE;
}

void DW_StartTransmit(DwDevice_st* dev) {
	DW_WriteTransmitFrameControlRegister(dev);
    if(dev->frameCheck)
    {
        dev->sysctrl &= ~SYS_CTRL_SFCST;
    }
    else
    {
        dev->sysctrl |= SYS_CTRL_SFCST;
    }
    dev->sysctrl |= SYS_CTRL_TXSTRT;
	dev->ops->spiWrite(dev, SYS_CTRL_ID, SYS_CTRL_OFFSET, &dev->sysctrl, SYS_CTRL_LEN);
	if(dev->permanentReceive) {
		memset(&dev->sysctrl, 0, SYS_CTRL_LEN);
		dev->deviceMode = RX_MODE;
		DW_StartReceive(dev);
	} else if (dev->wait4resp) {
        dev->deviceMode = RX_MODE;
    } else {
		dev->deviceMode = IDLE_MODE;
	}
}

void DW_WaitForResponse(DwDevice_st* dev, bool val) {
    dev->wait4resp = val;
    if(val)
    {
        dev->sysctrl |= SYS_CTRL_WAIT4RESP;
    }
    else
    {
        dev->sysctrl &= ~SYS_CTRL_WAIT4RESP;
    }
}

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
	unsigned char agctune1[AGC_TUNE1_LEN];
	unsigned char agctune2[AGC_TUNE2_LEN];
	unsigned char agctune3[AGC_TUNE3_LEN];
	unsigned char drxtune0b[DRX_TUNE0b_LEN];
	unsigned char drxtune1a[DRX_TUNE1a_LEN];
	unsigned char drxtune1b[DRX_TUNE1b_LEN];
	unsigned char drxtune2[DRX_TUNE2_LEN];
	unsigned char drxtune4H[DRX_TUNE4H_LEN];
	unsigned char ldecfg1[LDE_CFG1_LEN];
	unsigned char ldecfg2[LDE_CFG2_LEN];
	unsigned char lderepc[LDE_REPC_LEN];
	unsigned char txpower[TX_POWER_LEN];
	unsigned char rfrxctrlh[RF_RXCTRLH_LEN];
	unsigned char rftxctrl[RF_TXCTRL_LEN];
	unsigned char tcpgdelay[TC_PGDELAY_LEN];
	unsigned char fspllcfg[FS_PLLCFG_LEN];
	unsigned char fsplltune[FS_PLLTUNE_LEN];

	// AGC_TUNE1
	if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		writeValueToBytes(agctune1, 0x8870, AGC_TUNE1_LEN);
	} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
		writeValueToBytes(agctune1, 0x889B, AGC_TUNE1_LEN);
	} else {
		// TODO proper error/warning handling
	}
	// AGC_TUNE2
	writeValueToBytes(agctune2, 0x2502A907L, AGC_TUNE2_LEN);
	// AGC_TUNE3
	writeValueToBytes(agctune3, 0x0035, AGC_TUNE3_LEN);
	// DRX_TUNE0b (already optimized according to Table 20 of user manual)
	if(dev->dataRate == TRX_RATE_110KBPS) {
		writeValueToBytes(drxtune0b, 0x0016, DRX_TUNE0b_LEN);
	} else if(dev->dataRate == TRX_RATE_850KBPS) {
		writeValueToBytes(drxtune0b, 0x0006, DRX_TUNE0b_LEN);
	} else if(dev->dataRate == TRX_RATE_6800KBPS) {
		writeValueToBytes(drxtune0b, 0x0001, DRX_TUNE0b_LEN);
	} else {
		// TODO proper error/warning handling
	}
	// DRX_TUNE1a
	if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		writeValueToBytes(drxtune1a, 0x0087, DRX_TUNE1a_LEN);
	} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
		writeValueToBytes(drxtune1a, 0x008D, DRX_TUNE1a_LEN);
	} else {
		// TODO proper error/warning handling
	}
	// DRX_TUNE1b
	if(dev->preambleLength ==  TX_PREAMBLE_LEN_1536 || dev->preambleLength ==  TX_PREAMBLE_LEN_2048 ||
			dev->preambleLength ==  TX_PREAMBLE_LEN_4096) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(drxtune1b, 0x0064, DRX_TUNE1b_LEN);
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->preambleLength != TX_PREAMBLE_LEN_64) {
		if(dev->dataRate == TRX_RATE_850KBPS || dev->dataRate == TRX_RATE_6800KBPS) {
			writeValueToBytes(drxtune1b, 0x0020, DRX_TUNE1b_LEN);
		} else {
			// TODO proper error/warning handling
		}
	} else {
		if(dev->dataRate == TRX_RATE_6800KBPS) {
			writeValueToBytes(drxtune1b, 0x0010, DRX_TUNE1b_LEN);
		} else {
			// TODO proper error/warning handling
		}
	}
	// DRX_TUNE2
	if(dev->pacSize == PAC_SIZE_8) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			writeValueToBytes(drxtune2, 0x311A002DL, DRX_TUNE2_LEN);
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			writeValueToBytes(drxtune2, 0x313B006BL, DRX_TUNE2_LEN);
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->pacSize == PAC_SIZE_16) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			writeValueToBytes(drxtune2, 0x331A0052L, DRX_TUNE2_LEN);
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			writeValueToBytes(drxtune2, 0x333B00BEL, DRX_TUNE2_LEN);
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->pacSize == PAC_SIZE_32) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			writeValueToBytes(drxtune2, 0x351A009AL, DRX_TUNE2_LEN);
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			writeValueToBytes(drxtune2, 0x353B015EL, DRX_TUNE2_LEN);
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->pacSize == PAC_SIZE_64) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			writeValueToBytes(drxtune2, 0x371A011DL, DRX_TUNE2_LEN);
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			writeValueToBytes(drxtune2, 0x373B0296L, DRX_TUNE2_LEN);
		} else {
			// TODO proper error/warning handling
		}
	} else {
		// TODO proper error/warning handling
	}
	// DRX_TUNE4H
	if(dev->preambleLength == TX_PREAMBLE_LEN_64) {
		writeValueToBytes(drxtune4H, 0x0010, DRX_TUNE4H_LEN);
	} else {
		writeValueToBytes(drxtune4H, 0x0028, DRX_TUNE4H_LEN);
	}
	// RF_RXCTRLH
	if(dev->channel != CHANNEL_4 && dev->channel != CHANNEL_7) {
		writeValueToBytes(rfrxctrlh, 0xD8, RF_RXCTRLH_LEN);
	} else {
		writeValueToBytes(rfrxctrlh, 0xBC, RF_RXCTRLH_LEN);
	}
	// RF_TXCTRL
	if(dev->channel == CHANNEL_1) {
		writeValueToBytes(rftxctrl, 0x00005C40L, RF_TXCTRL_LEN);
	} else if(dev->channel == CHANNEL_2) {
		writeValueToBytes(rftxctrl, 0x00045CA0L, RF_TXCTRL_LEN);
	} else if(dev->channel == CHANNEL_3) {
		writeValueToBytes(rftxctrl, 0x00086CC0L, RF_TXCTRL_LEN);
	} else if(dev->channel == CHANNEL_4) {
		writeValueToBytes(rftxctrl, 0x00045C80L, RF_TXCTRL_LEN);
	} else if(dev->channel == CHANNEL_5) {
		writeValueToBytes(rftxctrl, 0x001E3FE0L, RF_TXCTRL_LEN);
	} else if(dev->channel == CHANNEL_7) {
		writeValueToBytes(rftxctrl, 0x001E7DE0L, RF_TXCTRL_LEN);
	} else {
		// TODO proper error/warning handling
	}
	// TC_PGDELAY
	if(dev->channel == CHANNEL_1) {
		writeValueToBytes(tcpgdelay, 0xC9, TC_PGDELAY_LEN);
	} else if(dev->channel == CHANNEL_2) {
		writeValueToBytes(tcpgdelay, 0xC2, TC_PGDELAY_LEN);
	} else if(dev->channel == CHANNEL_3) {
		writeValueToBytes(tcpgdelay, 0xC5, TC_PGDELAY_LEN);
	} else if(dev->channel == CHANNEL_4) {
		writeValueToBytes(tcpgdelay, 0x95, TC_PGDELAY_LEN);
	} else if(dev->channel == CHANNEL_5) {
		writeValueToBytes(tcpgdelay, 0xC0, TC_PGDELAY_LEN);
	} else if(dev->channel == CHANNEL_7) {
		writeValueToBytes(tcpgdelay, 0x93, TC_PGDELAY_LEN);
	} else {
		// TODO proper error/warning handling
	}
	// FS_PLLCFG and FS_PLLTUNE
	if(dev->channel == CHANNEL_1) {
		writeValueToBytes(fspllcfg, 0x09000407L, FS_PLLCFG_LEN);
		writeValueToBytes(fsplltune, 0x1E, FS_PLLTUNE_LEN);
	} else if(dev->channel == CHANNEL_2 || dev->channel == CHANNEL_4) {
		writeValueToBytes(fspllcfg, 0x08400508L, FS_PLLCFG_LEN);
		writeValueToBytes(fsplltune, 0x26, FS_PLLTUNE_LEN);
	} else if(dev->channel == CHANNEL_3) {
		writeValueToBytes(fspllcfg, 0x08401009L, FS_PLLCFG_LEN);
		writeValueToBytes(fsplltune, 0x56, FS_PLLTUNE_LEN);
	} else if(dev->channel == CHANNEL_5 || dev->channel == CHANNEL_7) {
		writeValueToBytes(fspllcfg, 0x0800041DL, FS_PLLCFG_LEN);
		writeValueToBytes(fsplltune, 0xBE, FS_PLLTUNE_LEN);
	} else {
		// TODO proper error/warning handling
	}
	// LDE_CFG1
	writeValueToBytes(ldecfg1, 0xD, LDE_CFG1_LEN);
	// LDE_CFG2
	if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		writeValueToBytes(ldecfg2, 0x1607, LDE_CFG2_LEN);
	} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
		writeValueToBytes(ldecfg2, 0x0607, LDE_CFG2_LEN);
	} else {
		// TODO proper error/warning handling
	}
	// LDE_REPC
	if(dev->preambleCode == PREAMBLE_CODE_16MHZ_1 || dev->preambleCode == PREAMBLE_CODE_16MHZ_2) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x5998 >> 3) & 0xFFFF), LDE_REPC_LEN);
		} else {
			writeValueToBytes(lderepc, 0x5998, LDE_REPC_LEN);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_16MHZ_3 || dev->preambleCode == PREAMBLE_CODE_16MHZ_8) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x51EA >> 3) & 0xFFFF), LDE_REPC_LEN);
		} else {
			writeValueToBytes(lderepc, 0x51EA, LDE_REPC_LEN);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_16MHZ_4) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x428E >> 3) & 0xFFFF), LDE_REPC_LEN);
		} else {
			writeValueToBytes(lderepc, 0x428E, LDE_REPC_LEN);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_16MHZ_5) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x451E >> 3) & 0xFFFF), LDE_REPC_LEN);
		} else {
			writeValueToBytes(lderepc, 0x451E, LDE_REPC_LEN);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_16MHZ_6) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x2E14 >> 3) & 0xFFFF), LDE_REPC_LEN);
		} else {
			writeValueToBytes(lderepc, 0x2E14, LDE_REPC_LEN);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_16MHZ_7) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x8000 >> 3) & 0xFFFF), LDE_REPC_LEN);
		} else {
			writeValueToBytes(lderepc, 0x8000, LDE_REPC_LEN);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_9) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x28F4 >> 3) & 0xFFFF), LDE_REPC_LEN);
		} else {
			writeValueToBytes(lderepc, 0x28F4, LDE_REPC_LEN);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_10 || dev->preambleCode == PREAMBLE_CODE_64MHZ_17) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x3332 >> 3) & 0xFFFF), LDE_REPC_LEN);
		} else {
			writeValueToBytes(lderepc, 0x3332, LDE_REPC_LEN);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_11) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x3AE0 >> 3) & 0xFFFF), LDE_REPC_LEN);
		} else {
			writeValueToBytes(lderepc, 0x3AE0, LDE_REPC_LEN);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_12) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x3D70 >> 3) & 0xFFFF), LDE_REPC_LEN);
		} else {
			writeValueToBytes(lderepc, 0x3D70, LDE_REPC_LEN);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_18 || dev->preambleCode == PREAMBLE_CODE_64MHZ_19) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x35C2 >> 3) & 0xFFFF), LDE_REPC_LEN);
		} else {
			writeValueToBytes(lderepc, 0x35C2, LDE_REPC_LEN);
		}
	} else if(dev->preambleCode == PREAMBLE_CODE_64MHZ_20) {
		if(dev->dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x47AE >> 3) & 0xFFFF), LDE_REPC_LEN);
		} else {
			writeValueToBytes(lderepc, 0x47AE, LDE_REPC_LEN);
		}
	} else {
		// TODO proper error/warning handling
	}
	// TX_POWER (enabled smart transmit power control)
	if(dev->channel == CHANNEL_1 || dev->channel == CHANNEL_2) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x15355575L, TX_POWER_LEN);
			} else {
				writeValueToBytes(txpower, 0x75757575L, TX_POWER_LEN);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x07274767L, TX_POWER_LEN);
			} else {
				writeValueToBytes(txpower, 0x67676767L, TX_POWER_LEN);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->channel == CHANNEL_3) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x0F2F4F6FL, TX_POWER_LEN);
			} else {
				writeValueToBytes(txpower, 0x6F6F6F6FL, TX_POWER_LEN);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x2B4B6B8BL, TX_POWER_LEN);
			} else {
				writeValueToBytes(txpower, 0x8B8B8B8BL, TX_POWER_LEN);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->channel == CHANNEL_4) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x1F1F3F5FL, TX_POWER_LEN);
			} else {
				writeValueToBytes(txpower, 0x5F5F5F5FL, TX_POWER_LEN);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x3A5A7A9AL, TX_POWER_LEN);
			} else {
				writeValueToBytes(txpower, 0x9A9A9A9AL, TX_POWER_LEN);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->channel == CHANNEL_5) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x0E082848L, TX_POWER_LEN);
			} else {
				writeValueToBytes(txpower, 0x48484848L, TX_POWER_LEN);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x25456585L, TX_POWER_LEN);
			} else {
				writeValueToBytes(txpower, 0x85858585L, TX_POWER_LEN);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->channel == CHANNEL_7) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x32527292L, TX_POWER_LEN);
			} else {
				writeValueToBytes(txpower, 0x92929292L, TX_POWER_LEN);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x5171B1D1L, TX_POWER_LEN);
			} else {
				writeValueToBytes(txpower, 0xD1D1D1D1L, TX_POWER_LEN);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else {
		// TODO proper error/warning handling
	}

	// write configuration back to chip
	dev->ops->spiWrite(dev, AGC_CTRL_ID, AGC_TUNE1_OFFSET, agctune1, AGC_TUNE1_LEN);
	dev->ops->spiWrite(dev, AGC_CTRL_ID, AGC_TUNE2_OFFSET, agctune2, AGC_TUNE2_LEN);
	dev->ops->spiWrite(dev, AGC_CTRL_ID, AGC_TUNE3_OFFSET, agctune3, AGC_TUNE3_LEN);
	dev->ops->spiWrite(dev, DRX_CONF_ID, DRX_TUNE0b_OFFSET, drxtune0b, DRX_TUNE0b_LEN);
	dev->ops->spiWrite(dev, DRX_CONF_ID, DRX_TUNE1a_OFFSET, drxtune1a, DRX_TUNE1a_LEN);
	dev->ops->spiWrite(dev, DRX_CONF_ID, DRX_TUNE1b_OFFSET, drxtune1b, DRX_TUNE1b_LEN);
	dev->ops->spiWrite(dev, DRX_CONF_ID, DRX_TUNE2_OFFSET, drxtune2, DRX_TUNE2_LEN);
	dev->ops->spiWrite(dev, DRX_CONF_ID, DRX_TUNE4H_OFFSET, drxtune4H, DRX_TUNE4H_LEN);
	dev->ops->spiWrite(dev, LDE_IF_ID, LDE_CFG1_OFFSET, ldecfg1, LDE_CFG1_LEN);
	dev->ops->spiWrite(dev, LDE_IF_ID, LDE_CFG2_OFFSET, ldecfg2, LDE_CFG2_LEN);
	dev->ops->spiWrite(dev, LDE_IF_ID, LDE_REPC_OFFSET, lderepc, LDE_REPC_LEN);
	dev->ops->spiWrite(dev, TX_POWER_ID, TX_POWER_OFFSET, txpower, TX_POWER_LEN);
	dev->ops->spiWrite(dev, RF_CONF_ID, RF_RXCTRLH_OFFSET, rfrxctrlh, RF_RXCTRLH_LEN);
	dev->ops->spiWrite(dev, RF_CONF_ID, RF_TXCTRL_OFFSET, rftxctrl, RF_TXCTRL_LEN);
	dev->ops->spiWrite(dev, TX_CAL_ID, TC_PGDELAY_OFFSET, tcpgdelay, TC_PGDELAY_LEN);
	dev->ops->spiWrite(dev, FS_CTRL_ID, FS_PLLTUNE_OFFSET, fsplltune, FS_PLLTUNE_LEN);
	dev->ops->spiWrite(dev, FS_CTRL_ID, FS_PLLCFG_OFFSET, fspllcfg, FS_PLLCFG_LEN);
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
    dev->ops->spiWrite(dev, TX_ANTD_ID, TX_ANTD_OFFSET, dev->antennaDelay.raw, TX_ANTD_LEN);
    dev->ops->spiWrite(dev, LDE_IF_ID, LDE_RXANTD_OFFSET, dev->antennaDelay.raw, LDE_RXANTD_LEN);
}

void DW_SetDefaults(DwDevice_st* dev) {
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
    dev->permanentReceive = false;
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
    DW_NVIC_Config();
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
    HAL_StatusTypeDef hal_status = HAL_OK;

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
    DW_CommitConfiguration(&dw_devcie);

    algorithm->init(&config_t, &dw_devcie);
    return hal_status;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
        case DW_IRQ_PIN:
            irq_flag = true;
            HAL_NVIC_ClearPendingIRQ(DW_IRQn);
        break;
            default:
        break;
    }
}

void HAL_SYSTICK_Callback(void)
{
    if(dw_event_timeout)
    {
        dw_event_timeout--;
    }
}

static int checkIrq()
{
    return HAL_GPIO_ReadPin(DW_IRQ_PORT, DW_IRQ_PIN);
}

static void DW_HandleInterrupt(DwDevice_st *dev) {
	// read current status and handle via callbacks
	DW_ReadSystemEventStatusRegister(dev);
	if(DW_IsClockProblem(dev)) {
		 /* TODO handle error */ 
	}
	if(DW_IsTransmitDone(dev) && dev->handleSent != 0) {
        DW_ClearTransmitStatus(dev);
		(*dev->handleSent)(dev);
	}
	if(DW_IsReceiveTimestampAvailable(dev)) {
        DW_ClearReceiveTimestampAvailableStatus(dev);
	}
	if(DW_IsReceiveFailed(dev) && dev->handleReceiveFailed != 0) {
        DW_ClearReceiveStatus(dev);
		dev->handleReceiveFailed(dev);
	} else if(DW_IsReceiveTimeout(dev) && dev->handleReceiveTimeout != 0) {
        DW_ClearReceiveStatus(dev);
		(*dev->handleReceiveTimeout)(dev);
	} else if(DW_IsReceiveDone(dev) && dev->handleReceived != 0) {
        DW_ClearReceiveStatus(dev);
		(*dev->handleReceived)(dev);
	}
}

void DW_Task(void)
{
    if(!dw_event_timeout)
    {
        dw_event_timeout = algorithm->onEvent(&dw_devcie, eventTimeout);
    }
    if(irq_flag)
    {
        do{
            DW_HandleInterrupt(&dw_devcie);
        } while(checkIrq() != 0);
    }   
}
