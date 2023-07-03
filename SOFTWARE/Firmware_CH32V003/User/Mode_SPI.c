#include "Mode_SPI.h"

static uint8_t Phase=0, Polarity=0, Leading=1, Trailing=0, ClockDelay=20;

void Soft_SPI_Init (uint8_t Mode, uint32_t Clock)
{
	GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(SPI_SCK_RCC_APB2Periph | SPI_MISO_RCC_APB2Periph | SPI_MOSI_RCC_APB2Periph | SPI_SS_RCC_APB2Periph, ENABLE);    // enable GPIO clock

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(SPI_SCK_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SPI_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(SPI_MISO_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SPI_MOSI_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(SPI_MOSI_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SPI_SS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(SPI_SS_GPIO, &GPIO_InitStructure);

    Polarity = Mode >> 1;
    Phase = Mode & 1;

    Leading = !Polarity;
    Trailing = Polarity;

    LOG_DEBUG("Polarity: %d, Phase: %d\n\r", Polarity, Phase);

    ClockDelay = 1000000 / Clock / 2;
    if (ClockDelay < SPI_SCK_MIN_DELAY) {
        ClockDelay = SPI_SCK_MIN_DELAY;
    }

    SPI_SCK_EDGE (Trailing);
    SPI_SS_HIGH ();
}

uint32_t Soft_SPI_Transfer (uint32_t SendData, uint8_t Size)
{
    uint32_t ReceivedData = 0, Mask=1<<(Size-1);
    if (!((Size == 8) || (Size == 16) || (Size == 32)))
    {
        LOG_DEBUG("Invalid size!\n\r");
        return 0;
    }

    SPI_SS_LOW ();
    DELAY_CS();
    for (uint16_t i=0; i<Size; i++)
    {
        if (Phase)  // PHA = 1
        {
            SPI_SCK_EDGE(Leading);
            if (SendData & Mask)
                SPI_MOSI_HIGH();
            else
                SPI_MOSI_LOW();
            SendData = SendData<<1;
            DELAY_CLK(ClockDelay);

            SPI_SCK_EDGE(Trailing);
            ReceivedData = (ReceivedData << 1) | SPI_MISO_LEVEL();
            DELAY_CLK(ClockDelay);
        }
        else    // PHA = 0
        {
            if (SendData & Mask)
                SPI_MOSI_HIGH();
            else
                SPI_MOSI_LOW();
            SendData = SendData<<1;
            DELAY_CLK(ClockDelay);
            SPI_SCK_EDGE(Leading);

            ReceivedData = (ReceivedData << 1) | SPI_MISO_LEVEL();
            DELAY_CLK(ClockDelay);
            SPI_SCK_EDGE(Trailing);
        }
    }
    DELAY_CS();
    SPI_SS_HIGH ();
    return ReceivedData;
}


uint8_t Soft_SPI_Transfer8 (uint8_t SendData)
{
    return Soft_SPI_Transfer (SendData, 8);
}

uint16_t Soft_SPI_Transfer16 (uint16_t SendData)
{
    return Soft_SPI_Transfer (SendData, 16);
}

uint32_t Soft_SPI_Transfer32 (uint32_t SendData)
{
    return Soft_SPI_Transfer (SendData, 32);
}
