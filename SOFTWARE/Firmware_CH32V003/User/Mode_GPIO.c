#include "Mode_GPIO.h"

//                                       PA    PB    PC    PD
uint8_t GPIO_Available_Pins[5] = {0x00, 0x06, 0x00, 0x1E, 0x7E};

void IndexToPort (GPIO_PORT_Index GPIO_Port, GPIO_TypeDef* *GPIO)
{
    switch (GPIO_Port)
    {
        case GPIO_PORTA:
            *GPIO = GPIOA;
        break;
        case GPIO_PORTC:
            *GPIO = GPIOC;
        break;
        case GPIO_PORTD:
            *GPIO = GPIOD;
        break;
        default:
            *GPIO = NULL;
            LOG_DEBUG ("GPIO port doesn't exist!\n\r");
            return;
        break;
    }
}

// output 0, input 1
void Port_Configuration (GPIO_PORT_Index GPIO_Port, uint16_t GPIO_Pins, uint16_t GPIO_Directions, uint16_t GPIO_PullUp)
{
    GPIO_TypeDef* GPIO;
    uint32_t RCC_APB2Periph_GPIO;

    IndexToPort (GPIO_Port, &GPIO);
    GPIO_Pins = GPIO_Pins & GPIO_Available_Pins[GPIO_Port]; // removing from the mask all of the forbidden pins on the selected Port


    switch (GPIO_Port)
    {
        case GPIO_PORTA:
            LOG_DEBUG("PortA \r\n");
            RCC_APB2Periph_GPIO = RCC_APB2Periph_GPIOA;
        break;
        case GPIO_PORTC:
            LOG_DEBUG("PortC \r\n");
            RCC_APB2Periph_GPIO = RCC_APB2Periph_GPIOC;
        break;
        case GPIO_PORTD:
            LOG_DEBUG("PortD \r\n");
            RCC_APB2Periph_GPIO = RCC_APB2Periph_GPIOD;
        break;
        default:
            RCC_APB2Periph_GPIO = 0;
            LOG_DEBUG ("GPIO port doesn't exist!\n\r");
            return;
        break;
    }

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO, ENABLE);    // enable GPIO clock

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pins & GPIO_Directions & GPIO_PullUp;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIO, &GPIO_InitStructure);
    // LOG_DEBUG("IPU 0x%02x \r\n", GPIO_InitStructure.GPIO_Pin);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pins & GPIO_Directions & (~GPIO_PullUp);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIO, &GPIO_InitStructure);
    // LOG_DEBUG("IPD 0x%02x \r\n", GPIO_InitStructure.GPIO_Pin);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pins & (~GPIO_Directions);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIO, &GPIO_InitStructure);
    // LOG_DEBUG("OUT 0x%02x \r\n", GPIO_InitStructure.GPIO_Pin);
}

void Port_Set_Output (GPIO_PORT_Index GPIO_Port, uint16_t GPIO_Pins, uint16_t GPIO_Value)
{
    GPIO_TypeDef* GPIO;

    IndexToPort (GPIO_Port, &GPIO);
    GPIO_Pins = GPIO_Pins & GPIO_Available_Pins[GPIO_Port]; // removing from the mask all of the forbidden pins on the selected Port

    GPIO_SetBits (GPIO, GPIO_Pins & GPIO_Value);
    GPIO_ResetBits (GPIO, GPIO_Pins & (~GPIO_Value));
}

uint16_t Port_Get_Input (GPIO_PORT_Index GPIO_Port, uint16_t GPIO_Pins)
{
    GPIO_TypeDef* GPIO;

    IndexToPort (GPIO_Port, &GPIO);
    GPIO_Pins = GPIO_Pins & GPIO_Available_Pins[GPIO_Port]; // removing from the mask all of the forbidden pins on the selected Port

    return (GPIO_ReadInputData (GPIO) & GPIO_Pins);
}

uint16_t Port_Get_Output (GPIO_PORT_Index GPIO_Port, uint16_t GPIO_Pins)
{
    GPIO_TypeDef* GPIO;

    IndexToPort (GPIO_Port, &GPIO);
    GPIO_Pins = GPIO_Pins & GPIO_Available_Pins[GPIO_Port]; // removing from the mask all of the forbidden pins on the selected Port

    return (GPIO_ReadOutputData (GPIO) & GPIO_Pins);
}

