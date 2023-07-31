#ifndef _PROTOCOL_DECODE_H_
#define _PROTOCOL_DECODE_H_

#include <ch32v00x.h>

// Version 1.0
#define VERSION_MAJOR       ((uint8_t)1)
#define VERSION_MINOR       ((uint8_t)0)

/* Global define */
#define CH32_SPI            SPI1
#define SPI_ERR_TIMEOUT     1
#define SPI_TIMEOUT_MS      100

#define MAX_BUFFER_SIZE     256

#define SYNC_MAGIC          0xAA
#define SYNC_RESPONSE       0x55
#define CMD_VERSION         0xBD

// simple dummy constants for 8 and 16 bits SPI transfer
#define DUMMY_DATA          0xFA
#define DUMMY_DATA16        0xFAFA

// Extender commands macros
#define MODE_GPIO           0x00
#define MODE_I2C            0x01
#define MODE_SPI            0x02
#define MODE_UART           0x03

#define DIRECTION_OUT       0x00
#define DIRECTION_IN        0x01

#define CMD_PORT_INIT       0x01
#define CMD_PORT_SET        0x02
#define CMD_PORT_GET        0x03

#define CMD_PWR_SENSE       0x09
#define CMD_BAT_SENSE       0x0A

#define CMD_INT_ACTIVE      0x11
#define CMD_INT_ENABLE      0x12
#define CMD_INT_DISABLE     0x13
#define CMD_INT_FLAGS       0x14
#define CMD_INT_CAPTURE     0x15

#define CMD_I2C_INIT        0x01
#define CMD_I2C_WRITE       0x02
#define CMD_I2C_READ        0x03
#define CMD_I2C_READREG     0x04

#define CMD_SPI_INIT        0x01
#define CMD_SPI_TRANSFER8   0x02
#define CMD_SPI_TRANSFER16  0x03

#define CMD_UART_CONFIGURE  0x01
#define CMD_UART_WRITE      0x02
#define CMD_UART_READ       0x03


// ----------------------------------------------------------- SPI ---------------------------------------------------------------------------
void SPI_FullDuplex_Init();

//
void Wait_For_Command ();

#endif
