#include "Mode_GPIO.h"
#include "Uptime.h"

#undef _DEBUG_
#ifdef DEBUG_MODE_GPIO
#define _DEBUG_ 1
#else
#define _DEBUG_ 0
#endif

//                                              PA    PB    PC    PD
static uint8_t GPIO_Available_Pins[5] = {0x00, 0x06, 0x00, 0x1E, 0x7D};

//                                      PA    PB    PC    PD
static uint8_t INT_Flags[5]   = {0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t INT_Capture[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t INT_Level = 1;
static uint8_t in_interrupt = 0;

GPIO_Error_Type IndexToPort (GPIO_PORT_Index GPIO_Port, GPIO_TypeDef* *GPIO)
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
            return GPIO_ERROR;
        break;
    }
    return GPIO_SUCCESS;
}

static uint32_t EXTI_Get_Line_Port(uint32_t EXTI_LineX)
{
    uint32_t mask = ((uint32_t)(3 << (EXTI_LineX << 1)));
    return ((AFIO->EXTICR & mask) >> (EXTI_LineX << 1)) + 1;
}

static void Interrupt_Set_Output()
{
    for (uint8_t i=1; i<5; i++)
        if (INT_Flags[i]) {
            Port_Set_Output (IO_EXP_IRQ_PORT, IO_EXP_IRQ_PIN, INT_Level ? IO_EXP_IRQ_PIN : 0x00);
            return;
        }
    Port_Set_Output (IO_EXP_IRQ_PORT, IO_EXP_IRQ_PIN, INT_Level ? 0x00 : IO_EXP_IRQ_PIN);
}

// ----------------------------------------------------------- GPIO ---------------------------------------------------------------------------

GPIO_Error_Type Port_Configuration (GPIO_PORT_Index GPIO_Port, uint16_t GPIO_Pins, uint16_t GPIO_Directions, uint16_t GPIO_PullUp)
{
    GPIO_TypeDef* GPIO;
    uint32_t RCC_APB2Periph_GPIO;

    if (IndexToPort (GPIO_Port, &GPIO) == GPIO_ERROR)
        return GPIO_ERROR;
    GPIO_Pins = GPIO_Pins & GPIO_Available_Pins[GPIO_Port]; // removing from the mask all of the forbidden pins on the selected Port


    switch (GPIO_Port)
    {
        case GPIO_PORTA:
            RCC_APB2Periph_GPIO = RCC_APB2Periph_GPIOA;
        break;
        case GPIO_PORTC:
            RCC_APB2Periph_GPIO = RCC_APB2Periph_GPIOC;
        break;
        case GPIO_PORTD:
            RCC_APB2Periph_GPIO = RCC_APB2Periph_GPIOD;
        break;
        default:
            RCC_APB2Periph_GPIO = 0;
            return GPIO_ERROR;
        break;
    }

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO, ENABLE);    // enable GPIO clock

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    if ((GPIO_PullUp & 0x8000) && (GPIO_Pins & GPIO_Directions)) {
        GPIO_InitStructure.GPIO_Pin = GPIO_Pins & GPIO_Directions;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIO, &GPIO_InitStructure);
        // LOG_DEBUG("IF 0x%02x " EOL, GPIO_InitStructure.GPIO_Pin);
        return GPIO_SUCCESS;
    }

    GPIO_InitStructure.GPIO_Pin = GPIO_Pins & GPIO_Directions & GPIO_PullUp;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIO, &GPIO_InitStructure);
    // LOG_DEBUG("IPU 0x%02x " EOL, GPIO_InitStructure.GPIO_Pin);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pins & GPIO_Directions & (~GPIO_PullUp);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIO, &GPIO_InitStructure);
    // LOG_DEBUG("IPD 0x%02x " EOL, GPIO_InitStructure.GPIO_Pin);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pins & (~GPIO_Directions);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIO, &GPIO_InitStructure);
    // LOG_DEBUG("OUT 0x%02x " EOL, GPIO_InitStructure.GPIO_Pin);
    return GPIO_SUCCESS;
}

GPIO_Error_Type Port_Set_Output (GPIO_PORT_Index GPIO_Port, uint16_t GPIO_Pins, uint16_t GPIO_Value)
{
    GPIO_TypeDef* GPIO;

    if (IndexToPort (GPIO_Port, &GPIO) == GPIO_ERROR)
        return GPIO_ERROR;
    GPIO_Pins = GPIO_Pins & GPIO_Available_Pins[GPIO_Port]; // removing from the mask all of the forbidden pins on the selected Port

    GPIO_SetBits (GPIO, GPIO_Pins & GPIO_Value);
    GPIO_ResetBits (GPIO, GPIO_Pins & (~GPIO_Value));
    return GPIO_SUCCESS;
}

GPIO_Error_Type Port_Get_Input (GPIO_PORT_Index GPIO_Port, uint16_t GPIO_Pins, uint16_t *GPIO_Value)
{
    GPIO_TypeDef* GPIO;

    if (IndexToPort (GPIO_Port, &GPIO) == GPIO_ERROR)
        return GPIO_ERROR;
    GPIO_Pins = GPIO_Pins & GPIO_Available_Pins[GPIO_Port]; // removing from the mask all of the forbidden pins on the selected Port

    *GPIO_Value = (GPIO_ReadInputData (GPIO) & GPIO_Pins);

    if ((INT_Flags[(uint8_t)GPIO_Port] & GPIO_Pins) != 0x00) {
        // Clear IO_EXP_IRQ line
        uint8_t flags = INT_Flags[(uint8_t)GPIO_Port];
        INT_Flags[(uint8_t)GPIO_Port] &= ~GPIO_Pins;
        if (INT_Flags[(uint8_t)GPIO_Port] == 0x00)
            Interrupt_Set_Output();

        // Enable interrupts
        EXTI->INTENR |= (flags & GPIO_Pins);
    }
    return GPIO_SUCCESS;
}

GPIO_Error_Type Port_Get_Output (GPIO_PORT_Index GPIO_Port, uint16_t GPIO_Pins, uint16_t *GPIO_Value)
{
    GPIO_TypeDef* GPIO;

    if (IndexToPort (GPIO_Port, &GPIO) == GPIO_ERROR)
        return GPIO_ERROR;
    GPIO_Pins = GPIO_Pins & GPIO_Available_Pins[GPIO_Port]; // removing from the mask all of the forbidden pins on the selected Port

    *GPIO_Value = (GPIO_ReadOutputData (GPIO) & GPIO_Pins);
    return GPIO_SUCCESS;
}

GPIO_Error_Type Interrupt_Level_Set(GPIO_PORT_Index int_active)
{
    GPIO_Error_Type flag = int_active ?
        Port_Set_Output (IO_EXP_IRQ_PORT, IO_EXP_IRQ_PIN, 0x00)
        :
        Port_Set_Output (IO_EXP_IRQ_PORT, IO_EXP_IRQ_PIN, IO_EXP_IRQ_PIN)
    ;

    if (flag == GPIO_ERROR) return GPIO_ERROR;

    INT_Level = int_active;
    return GPIO_SUCCESS;
}

// ----------------------------------------------------------- ADC ---------------------------------------------------------------------------

static uint8_t ADC_Ready = 0;

GPIO_Error_Type ADC_Channel_Init(uint32_t timeoutMS)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    ADC_InitTypeDef ADC_InitStructure = {0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD, ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1, ENABLE );
    RCC_ADCCLKConfig( RCC_PCLK2_Div8 );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOD, &GPIO_InitStructure );

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init( ADC1, &ADC_InitStructure );

    ADC_Calibration_Vol(ADC1, ADC_CALVOL_50PERCENT);
    ADC_Cmd( ADC1, ENABLE );

    ADC_ResetCalibration(ADC1);

    uint32_t t = Uptime_Ms();
    uint8_t timeout = 0;
    while(ADC_GetResetCalibrationStatus(ADC1)) {
        if ((uint32_t)(Uptime_Ms() - t) >= timeoutMS) {
            timeout = 1;
            break;
        }
    }
    if (timeout) return GPIO_ERROR;

    ADC_StartCalibration(ADC1);

    t = Uptime_Ms();
    timeout = 0;
    while(ADC_GetCalibrationStatus(ADC1)){
        if ((uint32_t)(Uptime_Ms() - t) >= timeoutMS) {
            timeout = 1;
            break;
        }
    }
    if (timeout) return GPIO_ERROR;

    ADC_Ready = 1;
    return GPIO_SUCCESS;
}

GPIO_Error_Type Get_ADC_Val(uint8_t channel, uint32_t timeoutMS, uint16_t *ADC_Value)
{
    if (!ADC_Ready) return GPIO_ERROR;
    uint32_t t = Uptime_Ms();

    ADC_RegularChannelConfig( ADC1, channel, 1, ADC_SampleTime_241Cycles );
    ADC_SoftwareStartConvCmd( ADC1, ENABLE );

    uint8_t timeout = 0;
    while( !ADC_GetFlagStatus( ADC1, ADC_FLAG_EOC )) {
        if ((uint32_t)(Uptime_Ms() - t) >= timeoutMS) {
            timeout = 1;
            break;
        }
    }
    if (timeout) return GPIO_ERROR;

    *ADC_Value = ADC_GetConversionValue( ADC1 );

    return GPIO_SUCCESS;
}

GPIO_Error_Type Get_ADC_Average( uint8_t channel, uint8_t times, uint16_t *ADC_Value)
{
    if (!ADC_Ready) return GPIO_ERROR;
    uint32_t tmp_val = 0;
    uint16_t val;

    for(uint8_t t = 0; t < times; t++ ){
        if (Get_ADC_Val(channel, ADC_TIMEOUT_MS, &val) == GPIO_ERROR) {
            return GPIO_ERROR;
        }
        tmp_val += val;
        Delay_Ms( 5 );
    }

    *ADC_Value = tmp_val / times;

    return GPIO_SUCCESS;
}

uint16_t ADC_Map (uint16_t Value, uint16_t MinValue, uint16_t MaxValue, float MinVoltage, float MaxVoltage)
{
    float tmp;
    tmp = (MaxVoltage - MinVoltage) / (MaxValue - MinValue);
    tmp = tmp * (Value-MinValue) + MinVoltage;
    tmp = tmp * 1000;
    return (uint16_t) tmp;
}

// ----------------------------------------------------------- Interrupts ---------------------------------------------------------------------------

void Interrupt_Enable(GPIO_PORT_Index GPIO_PortSourceX, uint8_t GPIO_PinSourceX, GPIO_INT_Trigger trigger)
{
    const EXTITrigger_TypeDef trigger_map[3] = {EXTI_Trigger_Rising, EXTI_Trigger_Falling, EXTI_Trigger_Rising_Falling};

    EXTI_InitTypeDef EXTI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    /* GPIOx ----> EXTI_LineX */
    GPIO_EXTILineConfig(GPIO_PortSourceX - 1, GPIO_PinSourceX);
    EXTI_InitStructure.EXTI_Line = 1 << GPIO_PinSourceX;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = trigger_map[trigger - 1];
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void Interrupt_Disable(GPIO_PORT_Index GPIO_PortSourceX, uint8_t GPIO_PinSourceX)
{
    uint32_t EXTI_LineX = 1 << GPIO_PinSourceX;
    uint32_t EXTI_PortX = EXTI_Get_Line_Port(GPIO_PinSourceX);

    if (GPIO_PortSourceX == EXTI_PortX) {
        if (INT_Flags[GPIO_PortSourceX] & GPIO_PinSourceX) {
            // Clear IO_EXP_IRQ line
            INT_Flags[GPIO_PortSourceX] &= ~GPIO_PinSourceX;
            if (INT_Flags[GPIO_PortSourceX] == 0x00) {
                Interrupt_Set_Output();
            }
        }
        EXTI->INTENR &= ~EXTI_LineX;
    }
}

uint8_t Interrupt_Flags(GPIO_PORT_Index GPIO_PortSourceX)
{
    return INT_Flags[GPIO_PortSourceX];
}

uint8_t Interrupt_Capture(GPIO_PORT_Index GPIO_PortSourceX)
{
    uint8_t flags = INT_Flags[GPIO_PortSourceX];

    // Clear IO_EXP_IRQ line
    INT_Flags[GPIO_PortSourceX] = 0x00;
    Interrupt_Set_Output();

    // Enable interrupt
    EXTI->INTENR |= flags;

    return INT_Capture[GPIO_PortSourceX];
}

void UART_Interrupt_Set()
{
    // Port - D Pin - 6 = UART_Rx
    INT_Flags[GPIO_PORTD] |= (1 << 6);
    Interrupt_Set_Output();
}

void UART_Interrupt_Clear()
{
    // Port - D Pin - 6 = UART_Rx
    INT_Flags[GPIO_PORTD] &= ~((uint8_t)(1 << 6));
    Interrupt_Set_Output();
}

void EXTI7_0_IRQHandler(void)
{
    // LOG_DEBUG(">");
    in_interrupt = 1;
    uint32_t EXTI_LineX = 0x01;
    for (uint8_t GPIO_PinSourceX=0; GPIO_PinSourceX<8; GPIO_PinSourceX++) {
        if(EXTI_GetITStatus(EXTI_LineX) != RESET) {
            // LOG_DEBUG(" %x ", EXTI_LineX);
            // Disable interrupt
            EXTI->INTENR &= ~EXTI_LineX;

            uint32_t EXTI_PortX = EXTI_Get_Line_Port(GPIO_PinSourceX);
            if ((INT_Flags[EXTI_PortX] & EXTI_LineX) == 0) {
                // LOG_DEBUG("^");
                // Set flags
                INT_Flags[EXTI_PortX] |= EXTI_LineX;

                // Clear capture bit
                INT_Capture[EXTI_PortX] &= ~EXTI_LineX;
                // Read GPIO
                GPIO_TypeDef* GPIO;
                IndexToPort (EXTI_PortX, &GPIO);
                // Set capture bit
                INT_Capture[EXTI_PortX] |= (GPIO_ReadInputData (GPIO) & EXTI_LineX);
            }

            // Clear Flag
            EXTI_ClearITPendingBit(EXTI_LineX);
        }
        EXTI_LineX <<= 1;
    }

    // Set IO_EXP_IRQ_PIN
    Interrupt_Set_Output();
    in_interrupt = 0;
    // LOG_DEBUG("<");
}
