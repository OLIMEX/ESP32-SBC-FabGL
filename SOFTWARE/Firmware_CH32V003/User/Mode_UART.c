#include "Uptime.h"
#include "Mode_UART.h"
#include "Mode_GPIO.h"

#define UART_BUFFER_SIZE 256

static uint8_t uart_buffer[UART_BUFFER_SIZE];
static uint8_t p_in=0, p_out=0;

/*********************************************************************
 * @fn      USARTx_CFG
 *
 * @brief   Initializes the USART2 & USART3 peripheral.
 *
 * @return  none
 */
void UART_Configure(uint32_t baudrate, UART_StopBits stop_bits, UART_Parity parity)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

    /* CH32_UART TX-->D.5   RX-->D.6 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    switch (stop_bits) {
        case UART_StopBits_1 :
            USART_InitStructure.USART_StopBits = USART_StopBits_1;
        break;
        case UART_StopBits_0_5 :
            USART_InitStructure.USART_StopBits = USART_StopBits_0_5;
        break;
        case UART_StopBits_2 :
            USART_InitStructure.USART_StopBits = USART_StopBits_2;
        break;
        case UART_StopBits_1_5 :
            USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
        break;
    }
    switch (parity) {
        case UART_Parity_No :
            USART_InitStructure.USART_Parity = USART_Parity_No;
        break;
        case UART_Parity_Odd :
            USART_InitStructure.USART_Parity = USART_Parity_Odd;
        break;
        case UART_Parity_Even :
            USART_InitStructure.USART_Parity = USART_Parity_Even;
        break;
    }
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(CH32_UART, &USART_InitStructure);
    USART_Cmd(CH32_UART, ENABLE);

    // Enable USART interrupt
    NVIC_InitTypeDef  NVIC_InitStructure = {0};
    USART_ITConfig(CH32_UART, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);;
}

UART_Error_Type UART_WaitFlag(uint32_t flag, uint32_t timeoutMS)
{
    uint32_t t = Uptime_Ms();
    while (1) {
        if (USART_GetFlagStatus(CH32_UART, flag) != RESET) {
            return UART_SUCCESS;
        }
        if ((uint32_t)(Uptime_Ms() - t) >= timeoutMS) {
            return UART_TIMEOUT;
        }
    }
}


UART_Error_Type UART_Send(uint8_t tx_buff[], uint8_t size)
{
    for (uint8_t i=0; i<size; i++) {
        if (UART_WaitFlag(USART_FLAG_TXE, UART_TIMEOUT_MS) == UART_TIMEOUT) {
            return UART_ERROR;
        }
        USART_SendData(CH32_UART, tx_buff[i]);
    }
    return UART_SUCCESS;
}

UART_Error_Type UART_Receive(uint8_t rx_buff[], uint8_t size, uint8_t *received)
{
    *received = 0;
    if (p_in == p_out) {
        return UART_SUCCESS;
    }

    while ((p_in != p_out) && (*received < size)) {
        rx_buff[*received] = uart_buffer[p_out];
        p_out++;
        (*received)++;
    }

    if (p_in == p_out) {
        UART_Interrupt_Clear();
    }

    return UART_SUCCESS;
}

void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(CH32_UART, USART_IT_RXNE) != RESET) {
        uint8_t d = USART_ReceiveData(CH32_UART);
        uint8_t p = p_in + 1;
        if (p != p_out) {
            uart_buffer[p_in] = d;
            p_in = p;

            UART_Interrupt_Set();
        }
    }
}
