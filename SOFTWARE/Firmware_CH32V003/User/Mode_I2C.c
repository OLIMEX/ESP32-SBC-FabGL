#include "Mode_I2C.h"

#define I2C_TIMEOUT_MS    100
#define I2C_ERR_TIMEOUT     1

static uint32_t p_ms = 0;

/*********************************************************************
 * @fn      I2C_Initialize
 *
 * @brief   Initializes the I2C peripheral.
 *
 * @return  none
 */
void I2C_Initialize (uint32_t ClockSpeed)
{
    p_ms = (uint32_t)(SystemCoreClock / 8000);

    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitTSturcture;

    RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd (USED_RCC_APB1Periph_I2Cx, ENABLE);

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
    I2C_Init (USED_I2C, &I2C_InitTSturcture);

    I2C_Cmd (USED_I2C, ENABLE);

    I2C_AcknowledgeConfig (USED_I2C, ENABLE);
}

uint8_t I2C_WaitFlag(uint32_t flag, uint32_t timeoutMS)
{
    uint32_t t;

    SysTick->SR &= ~(1 << 0);
    t = (uint32_t)timeoutMS * p_ms;

    SysTick->CMP = t;
    SysTick->CNT = 0;
    SysTick->CTLR |=(1 << 0);

    uint8_t timeout = 0;
    while (1) {
        if (I2C_GetFlagStatus (USED_I2C, flag) == RESET) {
            break;
        }
        if ((SysTick->SR & (1 << 0)) == (1 << 0)) {
            timeout = 1;
            break;
        }
    }

    SysTick->CTLR &= ~(1 << 0);

    return (timeout ? I2C_ERR_TIMEOUT : 0);
}

uint8_t I2C_WaitEvent(uint32_t event, uint32_t timeoutMS)
{
    uint32_t t;

    SysTick->SR &= ~(1 << 0);
    t = (uint32_t)timeoutMS * p_ms;

    SysTick->CMP = t;
    SysTick->CNT = 0;
    SysTick->CTLR |=(1 << 0);

    uint8_t timeout = 0;
    while (1) {
        if (I2C_CheckEvent (USED_I2C, event)) {
            break;
        }
        if ((SysTick->SR & (1 << 0)) == (1 << 0)) {
            timeout = 1;
            break;
        }
    }

    SysTick->CTLR &= ~(1 << 0);

    return (timeout ? I2C_ERR_TIMEOUT : 0);
}

void I2C_SendDataToSlave (uint16_t SlaveAddress, uint8_t DataBuffer[], uint16_t NumberOfBytes)
{
    if (I2C_WaitFlag(I2C_FLAG_BUSY, I2C_TIMEOUT_MS) == I2C_ERR_TIMEOUT) {
        LOG_DEBUG("I2C_READY Timeout\r\n");
        return;
    }
    I2C_GenerateSTART (USED_I2C, ENABLE);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT, I2C_TIMEOUT_MS) == I2C_ERR_TIMEOUT) {
        LOG_DEBUG("I2C_EVENT_MASTER_MODE_SELECT Timeout\r\n");
        goto stop;
    }

    I2C_Send7bitAddress (USED_I2C, SlaveAddress << 1, I2C_Direction_Transmitter);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, I2C_TIMEOUT_MS) == I2C_ERR_TIMEOUT) {
        LOG_DEBUG("I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED Timeout\r\n");
        goto stop;
    }

    for (uint16_t i=0; i<NumberOfBytes; i++)
    {
        Delay_Us (I2C_DELAY);
        I2C_SendData (USED_I2C, DataBuffer[i]);
        if (I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, I2C_TIMEOUT_MS) == I2C_ERR_TIMEOUT) {
            LOG_DEBUG("I2C_EVENT_MASTER_BYTE_TRANSMITTED Timeout\r\n");
            goto stop;
        }
    }

    stop:
    I2C_GenerateSTOP (USED_I2C, ENABLE);
    Delay_Us (I2C_DELAY);
}

void I2C_ReadDataFromSlave (uint16_t SlaveAddress, uint8_t DataBuffer[], uint16_t NumberOfBytes)
{
    if (I2C_WaitFlag(I2C_FLAG_BUSY, I2C_TIMEOUT_MS) == I2C_ERR_TIMEOUT) {
        LOG_DEBUG("I2C_READY Timeout\r\n");
        return;
    }
    I2C_GenerateSTART (USED_I2C, ENABLE);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT, I2C_TIMEOUT_MS) == I2C_ERR_TIMEOUT) {
        LOG_DEBUG("I2C_EVENT_MASTER_MODE_SELECT Timeout\r\n");
        goto stop;
    }

    I2C_Send7bitAddress (USED_I2C, SlaveAddress << 1, I2C_Direction_Receiver);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, I2C_TIMEOUT_MS) == I2C_ERR_TIMEOUT) {
        LOG_DEBUG("I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED Timeout\r\n");
        goto stop;
    }

    for (uint16_t i=0; i<NumberOfBytes; i++)
    {
        Delay_Us (I2C_DELAY);
        DataBuffer[i] = I2C_ReceiveData (USED_I2C);
        if (I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED, I2C_TIMEOUT_MS) == I2C_ERR_TIMEOUT) {
            LOG_DEBUG("I2C_EVENT_MASTER_BYTE_RECEIVED Timeout\r\n");
            goto stop;
        }
    }

    stop:
    I2C_GenerateSTOP (USED_I2C, ENABLE);
    Delay_Us (I2C_DELAY);
}

uint8_t I2C_ReadRegFromSlave (uint16_t SlaveAddress, uint8_t reg)
{
    uint8_t data = 0;

    // SEND Register index to slave
    if (I2C_WaitFlag(I2C_FLAG_BUSY, I2C_TIMEOUT_MS) == I2C_ERR_TIMEOUT) {
        LOG_DEBUG("I2C_READY Timeout\r\n");
        return data;
    }
    I2C_GenerateSTART (USED_I2C, ENABLE);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT, I2C_TIMEOUT_MS) == I2C_ERR_TIMEOUT) {
        LOG_DEBUG("I2C_EVENT_MASTER_MODE_SELECT Timeout\r\n");
        goto stop;
    }

    I2C_Send7bitAddress (USED_I2C, SlaveAddress << 1, I2C_Direction_Transmitter);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, I2C_TIMEOUT_MS) == I2C_ERR_TIMEOUT) {
        LOG_DEBUG("I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED Timeout\r\n");
        goto stop;
    }

    Delay_Us (I2C_DELAY);
    I2C_SendData (USED_I2C, reg);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, I2C_TIMEOUT_MS) == I2C_ERR_TIMEOUT) {
        LOG_DEBUG("I2C_EVENT_MASTER_BYTE_TRANSMITTED Timeout\r\n");
        goto stop;
    }

    I2C_GenerateSTOP (USED_I2C, ENABLE);

    Delay_Us (I2C_DELAY);
    // RECEIVE Register data from

    I2C_GenerateSTART (USED_I2C, ENABLE);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT, I2C_TIMEOUT_MS) == I2C_ERR_TIMEOUT) {
        LOG_DEBUG("I2C_EVENT_MASTER_MODE_SELECT Timeout\r\n");
        goto stop;
    }

    I2C_Send7bitAddress (USED_I2C, SlaveAddress << 1, I2C_Direction_Receiver);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, I2C_TIMEOUT_MS) == I2C_ERR_TIMEOUT) {
        LOG_DEBUG("I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED Timeout\r\n");
        goto stop;
    }

    Delay_Us (I2C_DELAY);
    data = I2C_ReceiveData (USED_I2C);
    if (I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED, I2C_TIMEOUT_MS) == I2C_ERR_TIMEOUT) {
        LOG_DEBUG("I2C_EVENT_MASTER_BYTE_RECEIVED Timeout\r\n");
        goto stop;
    }

    stop:
    I2C_GenerateSTOP (USED_I2C, ENABLE);
    Delay_Us (I2C_DELAY);

    return data;
}

