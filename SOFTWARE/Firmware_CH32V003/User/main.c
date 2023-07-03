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
#include "Mode_GPIO.h"  // TODO Remove




// ----------------------------------------------------------- SPI ---------------------------------------------------------------------------

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
    Delay_Init();
    USARTx_CFG(115200);
    LOG_DEBUG("SystemClk:%d\r\n",SystemCoreClock);

    // Power enable
    Port_Configuration (GPIO_PORTC, GPIO_Pin_3, 0x00, 0x00);
    Port_Set_Output (GPIO_PORTC, GPIO_Pin_3, 0x00);
    LOG_DEBUG("UEXT Power ENABLED\r\n");

    SPI_FullDuplex_Init (CH32_SPI);

    while (1)
    {
        Wait_For_Command (CH32_SPI);
    }
}
