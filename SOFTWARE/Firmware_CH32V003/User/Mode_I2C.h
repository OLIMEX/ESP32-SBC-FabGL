#ifndef _MODE_I2C_H_
#define _MODE_I2C_H_

#include <ch32v00x.h>

#define CH32_I2C                    I2C1
#define CH32_RCC_APB1Periph_I2Cx    RCC_APB1Periph_I2C1

#define I2C_SLAVE_NO_ACTIVITY   0
#define I2C_SLAVE_SEND          1
#define I2C_SLAVE_RECEIVE       2

typedef enum
{
    I2C_SUCCESS = 0,
    I2C_TIMEOUT = 1,
    I2C_ERROR = 2
} I2C_Error_Type;

void I2C_Initialize(uint32_t ClockSpeed);

// master
I2C_Error_Type I2C_SendDataToSlave (uint16_t SlaveAddress, uint8_t DataBuffer[], uint16_t NumberOfBytes);
I2C_Error_Type I2C_ReadDataFromSlave (uint16_t SlaveAddress, uint8_t DataBuffer[], uint16_t NumberOfBytes);
I2C_Error_Type I2C_ReadRegFromSlave (uint16_t SlaveAddress, uint8_t reg, uint8_t *data);

#endif
