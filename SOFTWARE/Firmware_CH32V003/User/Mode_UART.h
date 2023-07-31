#include <ch32v00x.h>

#ifndef USER_MODE_UART_H_
#define USER_MODE_UART_H_

#define CH32_UART          USART1
#define UART_TIMEOUT_MS    100


typedef enum {
    UART_StopBits_1    = 0x01,
    UART_StopBits_0_5  = 0x02,
    UART_StopBits_2    = 0x03,
    UART_StopBits_1_5  = 0x04
} UART_StopBits;

typedef enum {
    UART_Parity_No   = 0x00,
    UART_Parity_Odd  = 0x01,
    UART_Parity_Even = 0x02
} UART_Parity;

typedef enum
{
    UART_SUCCESS = 0,
    UART_TIMEOUT = 1,
    UART_ERROR = 2
} UART_Error_Type;

void UART_Configure(uint32_t baudrate, UART_StopBits stop_bits, UART_Parity parity);

UART_Error_Type UART_Send(uint8_t tx_buff[], uint8_t size);
UART_Error_Type UART_Receive(uint8_t rx_buff[], uint8_t size, uint8_t *received);

void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void);

#endif /* USER_MODE_UART_H_ */
