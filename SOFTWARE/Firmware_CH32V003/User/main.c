/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/08
 * Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include <ch32v00x.h>

#include "Protocol_Decode.h"
#include "Mode_GPIO.h"
#include "Mode_UART.h"
#include "Uptime.h"

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Uptime_Init();
    Delay_Init();
#ifdef _DEBUG_
    UART_Configure(115200, UART_StopBits_1, UART_Parity_No);
#endif
    LOG_DEBUG("SystemClk: %d" EOL, SystemCoreClock);

    // IO_EXP_IRQ configure - default active high
    Port_Configuration (IO_EXP_IRQ_PORT, IO_EXP_IRQ_PIN, 0x00, 0x00);
    Port_Set_Output (IO_EXP_IRQ_PORT, IO_EXP_IRQ_PIN, 0x00);
    LOG_DEBUG("IO_EXP_IRQ configured" EOL);

    // PWR_SENSE configure
    Port_Configuration (PWR_SENSE_PORT, PWR_SENSE_PIN, PWR_SENSE_PIN, 0x8000);
    Interrupt_Enable(PWR_SENSE_PORT, PWR_SENSE_PINSRC, FRONT_CHANGE);
    LOG_DEBUG("PWR_SENSE configured" EOL);

    // BAT_SENSE configure
    if (ADC_Channel_Init (ADC_TIMEOUT_MS) == GPIO_SUCCESS) {
        LOG_DEBUG("BAT_SENSE configured" EOL);
    } else {
        LOG_DEBUG("BAT_SENSE configuration FAILED" EOL);
    }

    // UEXT Power enable by default
    Port_Configuration (GPIO_PORTC, GPIO_Pin_3, 0x00, 0x00);
    Port_Set_Output (GPIO_PORTC, GPIO_Pin_3, 0x00);
    LOG_DEBUG("UEXT Power ENABLED" EOL);

    SPI_FullDuplex_Init ();

    while (1)
    {
        Wait_For_Command ();
    }
}
