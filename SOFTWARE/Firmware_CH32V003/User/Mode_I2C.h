#ifndef _MODE_I2C_H_
#define _MODE_I2C_H_

#include <ch32v00x.h>

#define USED_I2C                    I2C1
#define USED_RCC_APB1Periph_I2Cx    RCC_APB1Periph_I2C1

#define I2C_DELAY   200 // microseconds
#define I2C_SLAVE_NO_ACTIVITY   0
#define I2C_SLAVE_SEND          1
#define I2C_SLAVE_RECEIVE       2

void I2C_Initialize(uint32_t ClockSpeed);

// master
void I2C_SendDataToSlave (uint16_t SlaveAddress, uint8_t DataBuffer[], uint16_t NumberOfBytes);
void I2C_ReadDataFromSlave (uint16_t SlaveAddress, uint8_t DataBuffer[], uint16_t NumberOfBytes);
uint8_t I2C_ReadRegFromSlave (uint16_t SlaveAddress, uint8_t reg);

#endif
