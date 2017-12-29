#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#define __GPIO_SET(_PORT, _PIN)         ((_PORT)->BSRR = (_PIN))
#define __GPIO_RST(_PORT, _PIN)         ((_PORT)->BSRR = ((uint32_t)_PIN << 16U))
#define __GPIO_TOG(_PORT, _PIN)         ((_PORT)->ODR ^= (_PIN))
#define __GPIO_READ(_PORT, _PIN)        ((_PORT)->IDR & (_PIN))

//#define delay_ms(__ms)    HAL_Delay(__ms)

//SPI PHA POL
#define DW_GPIO5_SPIPOL_Pin             GPIO_PIN_1
#define DW_GPIO5_SPIPOL_GPIO_Port       GPIOA
#define DW_GPIO6_SPIPHA_Pin             GPIO_PIN_2
#define DW_GPIO6_SPIPHA_GPIO_Port       GPIOA
//Reset 
#define DW_RESET_Pin                    GPIO_PIN_0
#define DW_RESET_Port                   GPIOA
#define DW_RST_H()                      __GPIO_SET(DW_RESET_Port, DW_RESET_Pin)
#define DW_RST_L()                      __GPIO_RST(DW_RESET_Port, DW_RESET_Pin)
//SPI CS
#define DW_SPI_CS_Pin                       GPIO_PIN_4
#define DW_SPI_CS_Port                      GPIOA
#define DW_SPI_CS_H()                       __GPIO_SET(DW_SPI_CS_Port, DW_SPI_CS_Pin)
#define DW_SPI_CS_L()                       __GPIO_RST(DW_SPI_CS_Port, DW_SPI_CS_Pin)
#endif
