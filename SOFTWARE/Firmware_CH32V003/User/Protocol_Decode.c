#include "Protocol_Decode.h"
#include "Mode_GPIO.h"
#include "Mode_I2C.h"
#include "Mode_SPI.h"


/* Global Variable */

static uint8_t dir=0, cmd=0, mode=0, sync_mode=1;
static uint8_t start=0, size=0;
static uint8_t port=0, port_mask=0, port_dir=0, port_pullup=0, port_data=0, invalid_protocol_data=0, i;
static uint8_t transfer_buffer[MAX_BUFFER_SIZE], i2c_address, i2c_reg, spi_mode;
static uint32_t i2c_clock, spi_clock;

static uint32_t p_ms = 0;

//                                        PA    PB    PC    PD
uint8_t PROTO_Available_Pins[5] = {0x00, 0x06, 0x00, 0x0E, 0x78};

/*********************************************************************
 * @fn      USARTx_CFG
 *
 * @brief   Initializes the USART2 & USART3 peripheral.
 *
 * @return  none
 */
void USARTx_CFG(uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

    /* USART1 TX-->D.5   RX-->D.6 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}



// ----------------------------------------------------------- Hardware SPI functions -----------------------------------------------------------

uint8_t SPI_WaitFlag(SPI_TypeDef *SPIx, uint32_t flag, int32_t timeoutMS)
{
    if (timeoutMS < 0) {
        while (SPI_I2S_GetFlagStatus (SPIx, flag) == RESET);
        return 0;
    }

    uint32_t t;

    SysTick->SR &= ~(1 << 0);
    t = (uint32_t)timeoutMS * p_ms;

    SysTick->CMP = t;
    SysTick->CNT = 0;
    SysTick->CTLR |=(1 << 0);

    uint8_t timeout = 0;
    while (1) {
        if (SPI_I2S_GetFlagStatus (SPIx, flag) != RESET) {
            break;
        }
        if ((SysTick->SR & (1 << 0)) == (1 << 0)) {
            timeout = 1;
            break;
        }
    }

    SysTick->CTLR &= ~(1 << 0);

    return (timeout ? SPI_ERR_TIMEOUT : 0);
}

void SPI_FullDuplex_Init(SPI_TypeDef *SPIx)
{
    p_ms = (uint32_t)(SystemCoreClock / 8000);

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    SPI_InitTypeDef  SPI_InitStructure = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_PinRemapConfig (GPIO_Remap_SPI1, ENABLE);  // remapping NSS to PC0 (instead of PC1 which is default)

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPIx, &SPI_InitStructure);

    SPI_SSOutputCmd(SPIx, DISABLE);

    SPI_Cmd(SPIx, ENABLE);
}

static uint8_t SPI_Transfer(SPI_TypeDef *SPIx, uint8_t response, int16_t timeoutMS)
{
    if (SPI_WaitFlag(SPIx, SPI_I2S_FLAG_TXE, timeoutMS) == SPI_ERR_TIMEOUT) {
        LOG_DEBUG("SPI TX Ready Timeout\r\n");
        sync_mode = 1;
        return 0;
    }
    SPI_I2S_SendData(SPIx, response);

    if (SPI_WaitFlag(SPIx, SPI_I2S_FLAG_RXNE, timeoutMS) == SPI_ERR_TIMEOUT) {
        LOG_DEBUG("SPI RX Ready Timeout\r\n");
        sync_mode = 1;
        return 0;
    }
    uint8_t data = SPI_I2S_ReceiveData(SPIx);
    return data;
}

// ----------------------------------------------------------- Error function (fetching bytes) -----------------------------------------------------------
static void Error_Fetching (SPI_TypeDef *SPIx, uint8_t Interface[], uint8_t Command, uint8_t Size)
{
    if (Command)
        LOG_DEBUG ("Unknown %s command [%d]\n\r", Interface, Command);
    else
        LOG_DEBUG ("Invalid %s frame\n\r", Interface);

    sync_mode = 1;
}

// ----------------------------------------------------------- GPIO functions -----------------------------------------------------------
static uint8_t Command_Port_Mask (uint8_t port, uint8_t port_mask) 
{
    uint8_t mask = port_mask & PROTO_Available_Pins[port];
    if (port_mask != mask) {
        LOG_DEBUG("IGNORED   0x%02x\n\r",  port_mask & ~PROTO_Available_Pins[port]);
    }
    return mask;
}

static void Command_Port_Init (SPI_TypeDef *SPIx)
{
    port = SPI_Transfer(SPIx, DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return;
    LOG_DEBUG ("PORT %d\n\r",  port);

    port_mask = SPI_Transfer(SPIx, DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return;
    port_mask = Command_Port_Mask(port, port_mask);
    LOG_DEBUG("MASK      0x%02x\n\r",  port_mask);

    port_dir = SPI_Transfer(SPIx, DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return;
    LOG_DEBUG("PORT_DIR  0x%02x\n\r",  port_dir);

    port_pullup = SPI_Transfer(SPIx, DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return;
    LOG_DEBUG("PORT_PULL 0x%02x\n\r",  port_pullup);

    Port_Configuration (port, port_mask, port_dir, port_pullup); // Configure Port
}

static void Command_Port_Set (SPI_TypeDef *SPIx)
{
    port = SPI_Transfer(SPIx, DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return;
    LOG_DEBUG ("PORT %d\n\r",  port);

    port_mask = SPI_Transfer(SPIx, DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return;
    port_mask = Command_Port_Mask(port, port_mask);
    LOG_DEBUG("MASK      0x%02x\n\r",  port_mask);

    port_data = SPI_Transfer(SPIx, DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return;
    LOG_DEBUG("PORT_SET  0x%02x\n\r",  port_data);

    Port_Set_Output (port, port_mask, port_data); // Set Port
}

static void Command_Port_Get (SPI_TypeDef *SPIx)
{
    port = SPI_Transfer(SPIx, DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return;
    LOG_DEBUG ("PORT %d\n\r",  port);

    port_mask = SPI_Transfer(SPIx, DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return;
    port_mask = Command_Port_Mask(port, port_mask);
    LOG_DEBUG("MASK      0x%02x\n\r",  port_mask);

    port_data = Port_Get_Input (port, port_mask); // Get Port
    if (sync_mode) return;
    LOG_DEBUG("PORT_GET  0x%02x\n\r",  port_data);

    SPI_Transfer(SPIx, port_data, SPI_TIMEOUT_MS);
}

static void Mode_GPIO_Selected (SPI_TypeDef *SPIx)
{
    LOG_DEBUG ("GPIO FRAME BEGIN\n\r");
    switch (cmd)
    {
        case CMD_PORT_INIT:
            if (size == 4 && dir == DIRECTION_OUT)
                Command_Port_Init (SPIx);
            else
                invalid_protocol_data = 1;
        break;

        case CMD_PORT_SET:
            if (size == 3 && dir == DIRECTION_OUT)
                Command_Port_Set (SPIx);
            else
                invalid_protocol_data = 1;
        break;

        case CMD_PORT_GET:
            if (size == 3 && dir == DIRECTION_IN)
                Command_Port_Get (SPIx);
            else
                invalid_protocol_data = 1;
        break;

        default:
            Error_Fetching (SPIx, "GPIO", cmd, size);
        break;
    }
    if (invalid_protocol_data)
        Error_Fetching (SPIx, "GPIO", 0, size);
    LOG_DEBUG ("GPIO FRAME END\n\r");
}

// ----------------------------------------------------------- I2C functions -----------------------------------------------------------
static void Command_I2C_Init (SPI_TypeDef *SPIx)
{
    i2c_clock = 0;
    for (i=0; i<4; i++) {
        uint8_t spi_data = SPI_Transfer(SPIx, DUMMY_DATA, SPI_TIMEOUT_MS);
        if (sync_mode) return;
        i2c_clock = ((uint32_t)(i2c_clock << 8)) | spi_data;
    }

    I2C_Initialize(i2c_clock);
    LOG_DEBUG ("i2c_clock: %d\n\r", i2c_clock);
}

static void Command_I2C_Write (SPI_TypeDef *SPIx)
{
    LOG_DEBUG ("i2c_write start\n\r");
    i2c_address = SPI_Transfer(SPIx, DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return;

    LOG_DEBUG ("i2c_address: 0x%02x\n\r", i2c_address);
    for (i=0; i<size-1; i++)
    {
        transfer_buffer[i] = SPI_Transfer(SPIx, DUMMY_DATA, SPI_TIMEOUT_MS);
        if (sync_mode) return;
        LOG_DEBUG ("i2c_data: 0x%02x\n\r", transfer_buffer[i]);
    }
    I2C_SendDataToSlave (i2c_address, transfer_buffer, size-1);
    LOG_DEBUG ("i2c_write done\n\r");
}

static void Command_I2C_Read (SPI_TypeDef *SPIx)
{
    LOG_DEBUG ("i2c_read start\n\r");
    i2c_address = SPI_Transfer(SPIx, DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return;
    LOG_DEBUG ("i2c_address: 0x%02x\n\r", i2c_address);
    I2C_ReadDataFromSlave (i2c_address, transfer_buffer, size-1);
    for (i=0; i<size-1; i++) {
        SPI_Transfer(SPIx, transfer_buffer[i], SPI_TIMEOUT_MS);
        if (sync_mode) return;
    }

    for (i=0; i<size-1; i++)
        LOG_DEBUG("i2c_data: 0x%02x\r\n", transfer_buffer[i]);

    LOG_DEBUG ("i2c_read done\n\r");
}

static void Command_I2C_ReadReg (SPI_TypeDef *SPIx)
{
    LOG_DEBUG ("i2c_readreg start\n\r");
    i2c_address = SPI_Transfer(SPIx, DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return;
    LOG_DEBUG ("i2c_address: 0x%02x\n\r", i2c_address);
    i2c_reg = SPI_Transfer(SPIx, DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return;
    LOG_DEBUG ("i2c_reg: 0x%02x\n\r", i2c_reg);

    uint8_t data = I2C_ReadRegFromSlave (i2c_address, i2c_reg);

    SPI_Transfer (SPIx, data, SPI_TIMEOUT_MS);
    if (sync_mode) return;
    LOG_DEBUG("i2c_data: 0x%02x\r\n", data);

    LOG_DEBUG ("i2c_readreg done\n\r");
}

static void Mode_I2C_Selected (SPI_TypeDef *SPIx)
{
    LOG_DEBUG ("I2C FRAME BEGIN\n\r");
    switch (cmd)
    {
        case CMD_I2C_INIT:
            if (size == 4 && dir == DIRECTION_OUT)
                Command_I2C_Init (SPIx);
            else
                invalid_protocol_data = 1;
        break;

        case CMD_I2C_WRITE:
            if (size >= 2 && dir == DIRECTION_OUT)
                Command_I2C_Write (SPIx);
            else
               invalid_protocol_data = 1;
        break;

        case CMD_I2C_READ:
            if (size >= 2 && dir == DIRECTION_IN)
                Command_I2C_Read (SPIx);
            else
               invalid_protocol_data = 1;
        break;

        case CMD_I2C_READREG:
            if (size == 3 && dir == DIRECTION_IN)
                Command_I2C_ReadReg (SPIx);
            else
               invalid_protocol_data = 1;
        break;

        default:
            Error_Fetching (SPIx, "I2C", cmd, size);
        break;
    }
    if (invalid_protocol_data)
        Error_Fetching (SPIx, "I2C", 0, size);
    LOG_DEBUG ("I2C FRAME END\n\r");
}

// ----------------------------------------------------------- Software SPI functions -----------------------------------------------------------

static void Command_SoftSPI_Init (SPI_TypeDef *SPIx)
{
    spi_mode = SPI_Transfer(SPIx, DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return;

    spi_clock = 0;
    for (i=0; i<4; i++) {
        uint8_t spi_data = SPI_Transfer(SPIx, DUMMY_DATA, SPI_TIMEOUT_MS);
        if (sync_mode) return;
        spi_clock = ((uint32_t)(spi_clock << 8)) | spi_data;
    }

    LOG_DEBUG ("spi_mode: %d\n\r", spi_mode);
    LOG_DEBUG ("spi_clock: %d\n\r", spi_clock);

    if (spi_mode <= 3)
        Soft_SPI_Init(spi_mode, spi_clock);
    else
        LOG_DEBUG ("Invalid spi_mode. SPI initialization failed!\n\r");
}

static void Command_SoftSPI_Transfer8 (SPI_TypeDef *SPIx)
{
    uint8_t request = 0, response = DUMMY_DATA;
    for (i=0; i<size; i++)
    {
        request = SPI_Transfer(SPIx, response, SPI_TIMEOUT_MS);
        if (sync_mode) return;
        LOG_DEBUG("spi_req8: 0x%02x\r\n", request);
        LOG_DEBUG("spi_res8: 0x%02x\r\n", response);
        response = Soft_SPI_Transfer8(request);
    }
}

static void Command_SoftSPI_Transfer16 (SPI_TypeDef *SPIx)
{
    uint16_t request = 0, response = DUMMY_DATA16;
    for (i=0; i<(size/2); i++)
    {
        uint8_t msb = SPI_Transfer(SPIx, response >> 8, SPI_TIMEOUT_MS);
        if (sync_mode) return;

        uint8_t lsb = SPI_Transfer(SPIx, response & 0xFF, SPI_TIMEOUT_MS);
        if (sync_mode) return;

        request = (msb << 8) | lsb;
        LOG_DEBUG("spi_req16: 0x%04x\r\n", request);
        LOG_DEBUG("spi_res16: 0x%04x\r\n", response);
        response = Soft_SPI_Transfer16(request);
    }
}

static void Mode_SPI_Selected (SPI_TypeDef *SPIx)
{
    LOG_DEBUG ("SPI FRAME BEGIN\n\r");
    switch (cmd)
    {
        case CMD_SPI_INIT:
            if (size == 5 && dir == DIRECTION_OUT)
                Command_SoftSPI_Init (SPIx);
            else
                invalid_protocol_data = 1;
        break;

        case CMD_SPI_TRANSFER8:
            if (size > 1 && dir == DIRECTION_OUT)
                Command_SoftSPI_Transfer8 (SPIx);
            else
                invalid_protocol_data = 1;
        break;

        case CMD_SPI_TRANSFER16:
            if (size >= 4 && dir == DIRECTION_OUT)
                Command_SoftSPI_Transfer16 (SPIx);
            else
                invalid_protocol_data = 1;
        break;

        default:
            Error_Fetching (SPIx, "SPI", cmd, size);
        break;
    }
    if (invalid_protocol_data)
        Error_Fetching (SPIx, "SPI", 0, size);
    LOG_DEBUG ("SPI FRAME END\n\r");
}

// ----------------------------------------------------------- UART functions -----------------------------------------------------------

// ----------------------------------------------------------- Wait for Command function -----------------------------------------------------------
void Wait_For_Command (SPI_TypeDef *SPIx)
{
    if (sync_mode) {
        LOG_DEBUG ("Sync WAITING... ");
    } else {
        LOG_DEBUG ("READY\n\r");
    }

    start = SPI_Transfer(SPIx, sync_mode ? SYNC_RESPONSE : DUMMY_DATA, -1);
    if (start == SYNC_MAGIC) {
        sync_mode ^= 1;
        if (sync_mode) {
            LOG_DEBUG("Sync DETECTED\n\r");
        } else {
            LOG_DEBUG("Sync DONE\r\n");
        }
        return;
    }
    if (sync_mode) {
        LOG_DEBUG ("[0x%02X]\r\n", start);
        return;
    }

    LOG_DEBUG ("START 0x%02x\n\r", start);

    dir  = (start & 0x01);
    cmd  = (start >> 1) & 0x1F;
    mode = (start >> 6) & 0x03;

    LOG_DEBUG ("dir   %d\n\r",  dir);
    LOG_DEBUG ("cmd   %d\n\r",  cmd);
    LOG_DEBUG ("mode  %d\n\r", mode);

    size = SPI_Transfer(SPIx, DUMMY_DATA, SPI_TIMEOUT_MS);
    LOG_DEBUG ("SIZE  %d\n\r", size);

    invalid_protocol_data = 0;
    switch (mode)
    {
        case MODE_GPIO:
            Mode_GPIO_Selected (SPIx);
        break;

        case MODE_I2C:
            Mode_I2C_Selected (SPIx);
        break;

        case MODE_SPI:
            Mode_SPI_Selected (SPIx);
        break;

        case MODE_UART:
        break;
    }
}
