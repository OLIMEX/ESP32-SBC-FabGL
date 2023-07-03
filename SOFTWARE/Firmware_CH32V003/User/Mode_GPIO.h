#ifndef	_MODE_GPIO_H_
#define	_MODE_GPIO_H_

#include <ch32v00x.h>

extern uint8_t GPIO_Available_Pins[5];

typedef enum
{
    GPIO_PORTA = 1,
    GPIO_PORTB = 2,
    GPIO_PORTC = 3,
    GPIO_PORTD = 4
}GPIO_PORT_Index;

void IndexToPort (GPIO_PORT_Index GPIO_Port, GPIO_TypeDef* *GPIO);

// output 0, input 1
void Port_Configuration (GPIO_PORT_Index GPIO_Port, uint16_t GPIO_Pins, uint16_t GPIO_Directions, uint16_t GPIO_PullUp);
void Port_Set_Output (GPIO_PORT_Index GPIO_Port, uint16_t GPIO_Pins, uint16_t GPIO_Value);
uint16_t Port_Get_Input (GPIO_PORT_Index GPIO_Port, uint16_t GPIO_Pins);
uint16_t Port_Get_Output (GPIO_PORT_Index GPIO_Port, uint16_t GPIO_Pins);

#endif
