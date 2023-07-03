#ifndef _PROTOCOL_DECODE_H_
#define _PROTOCOL_DECODE_H_

#include <ch32v00x.h>

/* Global define */
#define CH32_SPI    SPI1
#define SPI_ERR_TIMEOUT 1
#define SPI_TIMEOUT_MS  100

#define MAX_BUFFER_SIZE 0xFF

#define SYNC_MAGIC    0xAA
#define SYNC_RESPONSE 0x55

// simple dummy constants for 8 and 16 bits SPI transfer
#define DUMMY_DATA    0xFA
#define DUMMY_DATA16  0xFAFA

// Extender commands macros
#define MODE_GPIO       0x00
#define MODE_I2C        0x01
#define MODE_SPI        0x02
#define MODE_UART       0x03

#define DIRECTION_OUT   0x00
#define DIRECTION_IN    0x01

#define CMD_PORT_INIT   0x01
#define CMD_PORT_SET    0x02
#define CMD_PORT_GET    0x03

#define CMD_I2C_INIT    0x01
#define CMD_I2C_WRITE   0x02
#define CMD_I2C_READ    0x03
#define CMD_I2C_READREG 0x04

#define CMD_SPI_INIT       0x01
#define CMD_SPI_TRANSFER8  0x02
#define CMD_SPI_TRANSFER16 0x03



// UART configuration
void USARTx_CFG(uint32_t baudrate);

// ----------------------------------------------------------- SPI ---------------------------------------------------------------------------
void SPI_FullDuplex_Init(SPI_TypeDef *SPIx);

//
void Wait_For_Command (SPI_TypeDef *SPIx);

#endif
