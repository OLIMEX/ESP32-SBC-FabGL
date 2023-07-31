#include "Mode_I2C.h"
#include "Uptime.h"

#define I2C_TIMEOUT_MS    100

#undef _DEBUG_
#ifdef DEBUG_MODE_I2C
#define _DEBUG_ 1
#else
#define _DEBUG_ 0
#endif

static uint32_t I2C_Delay = 0;

/*********************************************************************
 * @fn      I2C_Initialize
 *
 * @brief   Initializes the I2C peripheral.
 *
 * @return  none
 */
void I2C_Initialize (uint32_t ClockSpeed)
{
    I2C_Delay = 10000000 / ClockSpeed - 17;
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitTSturcture;

    RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd (CH32_RCC_APB1Periph_I2Cx, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init (GPIOC, &GPIO_InitStructure);

    I2C_InitTSturcture.I2C_ClockSpeed = ClockSpeed;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    //I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init (CH32_I2C, &I2C_InitTSturcture);

    I2C_Cmd (CH32_I2C, ENABLE);

    I2C_AcknowledgeConfig (CH32_I2C, ENABLE);
}

I2C_Error_Type I2C_WaitFlag(uint32_t flag, uint32_t timeoutMS)
{
    uint32_t t = Uptime_Ms();
    while (1) {
        if (I2C_GetFlagStatus (CH32_I2C, flag) == RESET) {
            return I2C_SUCCESS;
        }
        if ((uint32_t)(Uptime_Ms() - t) >= timeoutMS) {
            return I2C_TIMEOUT;
        }
    }
}

I2C_Error_Type I2C_WaitEvent(uint32_t event, uint32_t timeoutMS)
{
    uint32_t t = Uptime_Ms();
    while (1) {
        if (I2C_CheckEvent (CH32_I2C, event)) {
            return I2C_SUCCESS;
        }
        if ((uint32_t)(Uptime_Ms() - t) >= timeoutMS) {
            return I2C_TIMEOUT;
        }
    }
}

I2C_Error_Type I2C_SendDataToSlave (uint16_t SlaveAddress, uint8_t DataBuffer[], uint16_t NumberOfBytes)
{
    I2C_Error_Type error = I2C_SUCCESS;

    if (I2C_WaitFlag(I2C_FLAG_BUSY, I2C_TIMEOUT_MS) == I2C_TIMEOUT) {
        LOG_DEBUG("I2C_READY Timeout" EOL);
        return I2C_ERROR;
    }
    I2C_GenerateSTART (CH32_I2C, ENABLE);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT, I2C_TIMEOUT_MS) == I2C_TIMEOUT) {
        LOG_DEBUG("I2C_MASTER_MODE Timeout" EOL);
        error = I2C_ERROR;
        goto stop;
    }

    I2C_Send7bitAddress (CH32_I2C, SlaveAddress << 1, I2C_Direction_Transmitter);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, I2C_TIMEOUT_MS) == I2C_TIMEOUT) {
        LOG_DEBUG("I2C_MASTER_TRANSMITTER Timeout" EOL);
        error = I2C_ERROR;
        goto stop;
    }

    Delay_Us(I2C_Delay);
    for (uint16_t i=0; i<NumberOfBytes; i++)
    {
        I2C_SendData (CH32_I2C, DataBuffer[i]);
        if (I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, I2C_TIMEOUT_MS) == I2C_TIMEOUT) {
            LOG_DEBUG("I2C_MASTER_BYTE_TX Timeout" EOL);
            error = I2C_ERROR;
            goto stop;
        }
    }

    stop:
    I2C_GenerateSTOP (CH32_I2C, ENABLE);
    return error;
}

I2C_Error_Type I2C_ReadDataFromSlave (uint16_t SlaveAddress, uint8_t DataBuffer[], uint16_t NumberOfBytes)
{
    I2C_Error_Type error = I2C_SUCCESS;

    if (I2C_WaitFlag(I2C_FLAG_BUSY, I2C_TIMEOUT_MS) == I2C_TIMEOUT) {
        LOG_DEBUG("I2C_READY Timeout" EOL);
        return I2C_ERROR;
    }
    I2C_GenerateSTART (CH32_I2C, ENABLE);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT, I2C_TIMEOUT_MS) == I2C_TIMEOUT) {
        LOG_DEBUG("I2C_MASTER_MODE Timeout" EOL);
        error = I2C_ERROR;
        goto stop;
    }

    I2C_Send7bitAddress (CH32_I2C, SlaveAddress << 1, I2C_Direction_Receiver);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, I2C_TIMEOUT_MS) == I2C_TIMEOUT) {
        LOG_DEBUG("I2C_MASTER_RECEIVER Timeout" EOL);
        error = I2C_ERROR;
        goto stop;
    }

    Delay_Us(I2C_Delay);
    for (uint16_t i=0; i<NumberOfBytes; i++)
    {
        DataBuffer[i] = I2C_ReceiveData (CH32_I2C);
        if (I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED, I2C_TIMEOUT_MS) == I2C_TIMEOUT) {
            LOG_DEBUG("I2C_MASTER_BYTE_RX Timeout" EOL);
            error = I2C_ERROR;
            goto stop;
        }
    }

    stop:
    I2C_GenerateSTOP (CH32_I2C, ENABLE);
    return error;
}

I2C_Error_Type I2C_ReadRegFromSlave (uint16_t SlaveAddress, uint8_t reg, uint8_t *data)
{
    *data = 0;

    if (I2C_SendDataToSlave (SlaveAddress, &reg, 1) != I2C_SUCCESS) {
        return I2C_ERROR;
    }

    Delay_Us(I2C_Delay);
    if (I2C_ReadDataFromSlave (SlaveAddress, data, 1) != I2C_SUCCESS) {
        return I2C_ERROR;
    }

    return I2C_SUCCESS;
}

