#ifndef	_MODE_SPI_H_
#define	_MODE_SPI_H_

#include <ch32v00x.h>

#define SPI_SCK_MIN_DELAY   20
#define SPI_CS_DELAY        10
#define DELAY_CLK(Delay) Delay_Us(Delay)
#define DELAY_CS()       Delay_Us(SPI_CS_DELAY)

#define SPI_SCK_GPIO    GPIOD
#define SPI_SCK_PIN     GPIO_Pin_4
#define SPI_SCK_HIGH()  (GPIO_SetBits (SPI_SCK_GPIO, SPI_SCK_PIN))
#define SPI_SCK_LOW()   (GPIO_ResetBits (SPI_SCK_GPIO, SPI_SCK_PIN))
#define SPI_SCK_RCC_APB2Periph  RCC_APB2Periph_GPIOD

#define SPI_SCK_EDGE(Edge)  GPIO_WriteBit (SPI_SCK_GPIO, SPI_SCK_PIN, Edge)

#define SPI_MISO_GPIO   GPIOA
#define SPI_MISO_PIN    GPIO_Pin_2
#define SPI_MISO_HIGH() (GPIO_SetBits (SPI_MISO_GPIO, SPI_MISO_PIN))
#define SPI_MISO_LOW()  (GPIO_ResetBits (SPI_MISO_GPIO, SPI_MISO_PIN))
#define SPI_MISO_LEVEL() GPIO_ReadInputDataBit (GPIOA, GPIO_Pin_2)
#define SPI_MISO_RCC_APB2Periph  RCC_APB2Periph_GPIOA

#define SPI_MOSI_GPIO   GPIOA
#define SPI_MOSI_PIN    GPIO_Pin_1
#define SPI_MOSI_HIGH() (GPIO_SetBits (SPI_MOSI_GPIO, SPI_MOSI_PIN))
#define SPI_MOSI_LOW()  (GPIO_ResetBits (SPI_MOSI_GPIO, SPI_MOSI_PIN))
#define SPI_MOSI_RCC_APB2Periph  RCC_APB2Periph_GPIOA

#define SPI_SS_GPIO     GPIOD
#define SPI_SS_PIN      GPIO_Pin_3
#define SPI_SS_HIGH()   (GPIO_SetBits (SPI_SS_GPIO, SPI_SS_PIN))
#define SPI_SS_LOW()    (GPIO_ResetBits (SPI_SS_GPIO, SPI_SS_PIN))
#define SPI_SS_RCC_APB2Periph  RCC_APB2Periph_GPIOD

void Soft_SPI_Init (uint8_t Mode, uint32_t Clock);
uint32_t Soft_SPI_Transfer (uint32_t SendData, uint8_t Size);
uint8_t Soft_SPI_Transfer8 (uint8_t SendData);
uint16_t Soft_SPI_Transfer16 (uint16_t SendData);
uint32_t Soft_SPI_Transfer32 (uint32_t SendData);

#endif
