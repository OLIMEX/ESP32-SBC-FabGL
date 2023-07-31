#ifndef	_MODE_GPIO_H_
#define	_MODE_GPIO_H_

#include <ch32v00x.h>


typedef enum
{
    GPIO_SUCCESS = 0,
    GPIO_ERROR = 1
}GPIO_Error_Type;

typedef enum
{
    GPIO_PORTA = 1,
    GPIO_PORTB = 2,
    GPIO_PORTC = 3,
    GPIO_PORTD = 4
}GPIO_PORT_Index;

typedef enum {
	FRONT_RISING  = 0x01,
	FRONT_FALLING = 0x02,
	FRONT_CHANGE  = 0x03
}GPIO_INT_Trigger;

#define IO_EXP_IRQ_PORT   GPIO_PORTC
#define IO_EXP_IRQ_PIN    GPIO_Pin_4

#define PWR_SENSE_PORT    GPIO_PORTD
#define PWR_SENSE_PIN     GPIO_Pin_0
#define PWR_SENSE_PINSRC  GPIO_PinSource0

#define BAT_SENSE_PORT    GPIO_PORTD
#define BAT_SENSE_PIN     GPIO_Pin_2

#define ADC_TIMEOUT_MS    100

GPIO_Error_Type IndexToPort (GPIO_PORT_Index GPIO_Port, GPIO_TypeDef* *GPIO);

// output 0, input 1
GPIO_Error_Type Port_Configuration (GPIO_PORT_Index GPIO_Port, uint16_t GPIO_Pins, uint16_t GPIO_Directions, uint16_t GPIO_PullUp);
GPIO_Error_Type Port_Set_Output (GPIO_PORT_Index GPIO_Port, uint16_t GPIO_Pins, uint16_t GPIO_Value);
GPIO_Error_Type Port_Get_Input (GPIO_PORT_Index GPIO_Port, uint16_t GPIO_Pins, uint16_t *GPIO_Value);
GPIO_Error_Type Port_Get_Output (GPIO_PORT_Index GPIO_Port, uint16_t GPIO_Pins, uint16_t *GPIO_Value);

GPIO_Error_Type ADC_Channel_Init(uint32_t timeoutMS);
GPIO_Error_Type Get_ADC_Val(uint8_t channel, uint32_t timeoutMS, uint16_t *ADC_Value);
GPIO_Error_Type Get_ADC_Average( uint8_t channel, uint8_t times, uint16_t *ADC_Value);
uint16_t ADC_Map (uint16_t Value, uint16_t MinValue, uint16_t MaxValue, float MinVoltage, float MaxVoltage);

GPIO_Error_Type Interrupt_Level_Set(GPIO_PORT_Index int_active);
void Interrupt_Enable(GPIO_PORT_Index GPIO_PortSourceX, uint8_t GPIO_PinSourceX, GPIO_INT_Trigger trigger);
void Interrupt_Disable(GPIO_PORT_Index GPIO_PortSourceX, uint8_t GPIO_PinSourceX);
uint8_t Interrupt_Flags(GPIO_PORT_Index GPIO_PortSourceX);
uint8_t Interrupt_Capture(GPIO_PORT_Index GPIO_PortSourceX);

void UART_Interrupt_Set();
void UART_Interrupt_Clear();

void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler(void);

#endif
