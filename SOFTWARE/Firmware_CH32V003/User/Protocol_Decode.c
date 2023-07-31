#include "Protocol_Decode.h"
#include "Mode_GPIO.h"
#include "Mode_I2C.h"
#include "Mode_SPI.h"
#include "Mode_UART.h"
#include "Uptime.h"

typedef enum
{
    CMD_SUCCESS = 0,
    CMD_FAILURE = 1,
    CMD_INVALID_COMMAND = 2,
    CMD_INVALID_FRAME = 3,
    CMD_SPI_TIMEOUT = 4
}Command_Result_Type;


/* Global Variable */

static uint8_t protocol_start=0, payload_size=0;
static uint8_t protocol_dir=0, protocol_cmd=0, protocol_mode=0, sync_mode=1;

static uint8_t  data8=0;
static uint16_t data16=0;
static uint32_t clock;

static uint8_t port=0, port_mask=0, port_dir=0, port_pullup=0, i;
static uint8_t i2c_address, i2c_reg, spi_mode, int_active = 1;

UART_StopBits stop_bits;
UART_Parity parity;

static uint8_t tx_buffer[MAX_BUFFER_SIZE];

static Command_Result_Type cmd_result = CMD_SUCCESS;

//                                        PA    PB    PC    PD
uint8_t PROTO_Available_Pins[5] = {0x00, 0x06, 0x00, 0x0E, 0x78};


// ----------------------------------------------------------- Hardware SPI functions -----------------------------------------------------------

uint8_t SPI_WaitFlag(uint32_t flag, int32_t timeoutMS)
{
    if (timeoutMS < 0) {
        while (SPI_I2S_GetFlagStatus (CH32_SPI, flag) == RESET);
        return 0;
    }

    uint32_t t = Uptime_Ms();
    while (1) {
        if (SPI_I2S_GetFlagStatus (CH32_SPI, flag) != RESET) {
            return 0;
        }
        if ((uint32_t)(Uptime_Ms() - t) >= timeoutMS) {
            return SPI_ERR_TIMEOUT;
        }
    }
}

void SPI_FullDuplex_Init()
{
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
    SPI_Init(CH32_SPI, &SPI_InitStructure);

    SPI_SSOutputCmd(CH32_SPI, DISABLE);

    SPI_Cmd(CH32_SPI, ENABLE);
}

static uint8_t SPI_Transfer(uint8_t response, int16_t timeoutMS)
{
    if (SPI_WaitFlag(SPI_I2S_FLAG_TXE, timeoutMS) == SPI_ERR_TIMEOUT) {
        LOG_DEBUG("SPI TX Ready Timeout" EOL);
        sync_mode = 1;
        return 0;
    }
    SPI_I2S_SendData(CH32_SPI, response);

    if (SPI_WaitFlag(SPI_I2S_FLAG_RXNE, timeoutMS) == SPI_ERR_TIMEOUT) {
        LOG_DEBUG("SPI RX Ready Timeout" EOL);
        sync_mode = 1;
        return 0;
    }
    uint8_t data = SPI_I2S_ReceiveData(CH32_SPI);
    return data;
}

// ----------------------------------------------------------- Error function (fetching bytes) -----------------------------------------------------------
static void Error_Fetching ()
{
    switch (cmd_result) {
        case CMD_INVALID_COMMAND:
            LOG_DEBUG("INVALID [%d]" EOL, protocol_cmd);
        break;
        
        case CMD_INVALID_FRAME:
            LOG_DEBUG("Invalid frame" EOL);
        break;
    }
    sync_mode = 1;
}

// ----------------------------------------------------------- GPIO functions -----------------------------------------------------------
#undef _DEBUG_
#ifdef DEBUG_MODE_GPIO
#define _DEBUG_ 1
#else
#define _DEBUG_ 0
#endif

static uint8_t Command_Port_Mask (uint8_t port, uint8_t port_mask) 
{
    uint8_t mask = port_mask & PROTO_Available_Pins[port];
    if (port_mask != mask) {
        LOG_DEBUG("IGNORED 0x%02x" EOL,  port_mask & ~PROTO_Available_Pins[port]);
    }
    return mask;
}

#ifdef LOG_DEBUG_ENABLED
static const char * port_names[] = {
    "A[1]",
    "B[2]*",
    "C[3]",
    "D[4]"
};
static const char INVALID_PORT_STR[] = "INVALID";
static const uint8_t port_count = (sizeof (port_names) / sizeof (const char *));

static const char * Command_Port_Name (GPIO_PORT_Index port)
{
    if (port > port_count) return INVALID_PORT_STR;
    return port_names[port-1];
}
#endif

static Command_Result_Type Command_Port_Init ()
{
    LOG_DEBUG("PORT_INIT" EOL);
    if (payload_size != 4 || protocol_dir != DIRECTION_OUT) return CMD_INVALID_FRAME;

    port = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;
    LOG_DEBUG("PORT %s" EOL, Command_Port_Name(port));

    port_mask = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;
    port_mask = Command_Port_Mask(port, port_mask);
    LOG_DEBUG("MASK 0x%02x" EOL,  port_mask);

    port_dir = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;
    LOG_DEBUG("DIR 0x%02x" EOL,  port_dir);

    port_pullup = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;
    LOG_DEBUG("PULLUP 0x%02x" EOL,  port_pullup);

    if (Port_Configuration (port, port_mask, port_dir, port_pullup) == GPIO_ERROR) {
        return CMD_FAILURE;
    }
    
    return CMD_SUCCESS;
}

static Command_Result_Type Command_Port_Set ()
{
    LOG_DEBUG("PORT_SET" EOL);
    if (payload_size != 3 || protocol_dir != DIRECTION_OUT) return CMD_INVALID_FRAME;

    port = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;
    LOG_DEBUG("PORT %s" EOL, Command_Port_Name(port));

    port_mask = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;
    port_mask = Command_Port_Mask(port, port_mask);
    LOG_DEBUG("MASK 0x%02x" EOL,  port_mask);

    data8 = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;
    LOG_DEBUG("SET 0x%02x" EOL,  data8);

    if (Port_Set_Output (port, port_mask, data8) == GPIO_ERROR) {
        return CMD_FAILURE;
    }

    return CMD_SUCCESS;
}

static Command_Result_Type Command_Port_Get ()
{
    LOG_DEBUG("PORT_GET" EOL);
    if (payload_size != 3 || protocol_dir != DIRECTION_IN) return CMD_INVALID_FRAME;

    port = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;
    LOG_DEBUG("PORT %s" EOL, Command_Port_Name(port));

    port_mask = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;
    port_mask = Command_Port_Mask(port, port_mask);
    LOG_DEBUG("MASK 0x%02x" EOL,  port_mask);

    if (Port_Get_Input (port, port_mask, &data16) == GPIO_ERROR) {
        return CMD_FAILURE;
    }
    LOG_DEBUG("GET 0x%02x" EOL,  data16);

    SPI_Transfer(data16, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;

    return CMD_SUCCESS;
}

#undef _DEBUG_
#ifdef DEBUG_MODE_PWR
#define _DEBUG_ 1
#else
#define _DEBUG_ 0
#endif

static Command_Result_Type Command_Pwr_Sense ()
{
    LOG_DEBUG("PWR_SENSE" EOL);
    if (payload_size != 1 || protocol_dir != DIRECTION_IN) return CMD_INVALID_FRAME;

    if (Port_Get_Input (PWR_SENSE_PORT, PWR_SENSE_PIN, &data16) == GPIO_ERROR) {
        return CMD_FAILURE;
    }
    data16 = (data16 != 0);
    LOG_DEBUG("PWR 0x%02x" EOL,  data16);

    SPI_Transfer(data16, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;

    return CMD_SUCCESS;
}

static Command_Result_Type Command_Bat_Sense ()
{
    LOG_DEBUG("BAT_SENSE" EOL);
    if (payload_size != 2 || protocol_dir != DIRECTION_IN) return CMD_INVALID_FRAME;

    if (Get_ADC_Val(ADC_Channel_3, ADC_TIMEOUT_MS, &data16) == GPIO_ERROR) {
        return CMD_FAILURE;
    }
    data16 = ADC_Map(data16, 0, 1023, 0.0, 3.3);
    // compensation of the hardware divider on the battery input
    data16 = data16*62/15;

    // Transfer MSB first
    SPI_Transfer(data16 >> 8,   SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;

    SPI_Transfer(data16 & 0xFF, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;

    LOG_DEBUG("BAT %d mV" EOL,  data16);

    return CMD_SUCCESS;
}

#undef _DEBUG_
#ifdef DEBUG_MODE_INT
#define _DEBUG_ 1
#else
#define _DEBUG_ 0
#endif

static Command_Result_Type Command_Set_Int_Active ()
{
    LOG_DEBUG("INT_ACTIVE" EOL);
    if (payload_size != 1 || protocol_dir != DIRECTION_OUT) return CMD_INVALID_FRAME;

    int_active = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;

    if (Interrupt_Level_Set(int_active) == GPIO_ERROR) {
        return CMD_FAILURE;
    }

    LOG_DEBUG("ACTIVE %d" EOL, int_active);
    return CMD_SUCCESS;
}

static Command_Result_Type Command_Enable_Interrupt ()
{
    LOG_DEBUG("INT_ENABLE" EOL);
    if (payload_size != 3 || protocol_dir != DIRECTION_OUT) return CMD_INVALID_FRAME;

    port = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;
    LOG_DEBUG("PORT %s" EOL,  Command_Port_Name(port));

    uint8_t pin = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;
    port_mask = Command_Port_Mask(port, 1 << pin);
    LOG_DEBUG("PIN 0x%02x" EOL,  pin);

    uint8_t int_trigger = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;
    LOG_DEBUG("TRIGGER 0x%02x" EOL,  int_trigger);

    Interrupt_Enable(port, pin, int_trigger);
    return CMD_SUCCESS;
}

static Command_Result_Type Command_Disable_Interrupt ()
{
    LOG_DEBUG("INT_DISABLE" EOL);
    if (payload_size != 2 || protocol_dir != DIRECTION_OUT) return CMD_INVALID_FRAME;

    port = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;
    LOG_DEBUG("PORT %s" EOL, Command_Port_Name(port));

    uint8_t pin = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;
    port_mask = Command_Port_Mask(port, 1 << pin);
    LOG_DEBUG("PIN 0x%02x" EOL,  pin);

    Interrupt_Disable(port, pin);
    return CMD_SUCCESS;
}

static Command_Result_Type Command_Interrupt_Flags ()
{
    LOG_DEBUG("INT_FLAGS" EOL);
    if (payload_size != 2 || protocol_dir != DIRECTION_IN) return CMD_INVALID_FRAME;

    port = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;
    LOG_DEBUG("PORT %s" EOL, Command_Port_Name(port));

    uint8_t flags = Interrupt_Flags(port);
    SPI_Transfer(flags, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;

    LOG_DEBUG("FLAGS 0x%02X" EOL,  flags);
    return CMD_SUCCESS;
}

static Command_Result_Type Command_Interrupt_Capture ()
{
    LOG_DEBUG("INT_CAPTURE" EOL);
    if (payload_size != 2 || protocol_dir != DIRECTION_IN) return CMD_INVALID_FRAME;

    port = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;
    LOG_DEBUG("PORT %s" EOL, Command_Port_Name(port));

    uint8_t capture = Interrupt_Capture(port);
    SPI_Transfer(capture, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;

    LOG_DEBUG("CAPTURE 0x%02X" EOL,  capture);
    return CMD_SUCCESS;
}

#undef _DEBUG_
#ifdef DEBUG_MODE_GPIO
#define _DEBUG_ 1
#else
#define _DEBUG_ 0
#endif

static void Mode_GPIO_Selected ()
{
    LOG_DEBUG("GPIO CMD_");
    switch (protocol_cmd)
    {
        case CMD_PORT_INIT:
            cmd_result = Command_Port_Init ();
        break;

        case CMD_PORT_SET:
            cmd_result = Command_Port_Set ();
        break;

        case CMD_PORT_GET:
            cmd_result = Command_Port_Get ();
        break;

        case CMD_PWR_SENSE:
            cmd_result = Command_Pwr_Sense ();
        break;

        case CMD_BAT_SENSE:
            cmd_result = Command_Bat_Sense ();
        break;

        case CMD_INT_ACTIVE:
            cmd_result = Command_Set_Int_Active ();
        break;

        case CMD_INT_ENABLE:
            cmd_result = Command_Enable_Interrupt ();
        break;

        case CMD_INT_DISABLE:
            cmd_result = Command_Disable_Interrupt ();
        break;

        case CMD_INT_FLAGS:
            cmd_result = Command_Interrupt_Flags ();
        break;

        case CMD_INT_CAPTURE:
            cmd_result = Command_Interrupt_Capture ();
        break;

        default:
            cmd_result = CMD_INVALID_COMMAND;
        break;
    }
    if (cmd_result != CMD_SUCCESS) Error_Fetching ();
    LOG_DEBUG("GPIO END" EOL);
}

// ----------------------------------------------------------- I2C functions -----------------------------------------------------------
#undef _DEBUG_
#ifdef DEBUG_MODE_I2C
#define _DEBUG_ 1
#else
#define _DEBUG_ 0
#endif

static Command_Result_Type Command_I2C_Init ()
{
    LOG_DEBUG("INIT" EOL);
    if (payload_size != 4 || protocol_dir != DIRECTION_OUT) return CMD_INVALID_FRAME;

    clock = 0;
    for (i=0; i<4; i++) {
        data8 = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
        if (sync_mode) return CMD_SPI_TIMEOUT;
        clock = ((uint32_t)(clock << 8)) | data8;
    }

    I2C_Initialize(clock);
    LOG_DEBUG("CLOCK %d" EOL, clock);
    return CMD_SUCCESS;
}

static Command_Result_Type Command_I2C_Write ()
{
    LOG_DEBUG("WRITE" EOL);
    if (payload_size < 2 || protocol_dir != DIRECTION_OUT) return CMD_INVALID_FRAME;

    i2c_address = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;

    LOG_DEBUG("ADDRESS 0x%02x" EOL, i2c_address);
    for (i=0; i<payload_size-1; i++)
    {
        tx_buffer[i] = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
        if (sync_mode) return CMD_SPI_TIMEOUT;
        LOG_DEBUG("DATA 0x%02x" EOL, tx_buffer[i]);
    }

    if (I2C_SendDataToSlave (i2c_address, tx_buffer, payload_size-1) != I2C_SUCCESS) {
        return CMD_FAILURE;
    }

    return CMD_SUCCESS;
}

static Command_Result_Type Command_I2C_Read ()
{
    LOG_DEBUG("READ" EOL);
    if (payload_size < 2 || protocol_dir != DIRECTION_IN) return CMD_INVALID_FRAME;

    i2c_address = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;
    LOG_DEBUG("ADDRESS 0x%02x" EOL, i2c_address);

    if (I2C_ReadDataFromSlave (i2c_address, tx_buffer, payload_size-1) != I2C_SUCCESS) {
        return CMD_FAILURE;
    }

    for (i=0; i<payload_size-1; i++) {
        SPI_Transfer(tx_buffer[i], SPI_TIMEOUT_MS);
        if (sync_mode) return CMD_SPI_TIMEOUT;
    }

    for (i=0; i<payload_size-1; i++) {
        LOG_DEBUG("DATA 0x%02x" EOL, tx_buffer[i]);
    }
    return CMD_SUCCESS;
}

static Command_Result_Type Command_I2C_ReadReg ()
{
    LOG_DEBUG("READREG" EOL);
    if (payload_size != 3 || protocol_dir != DIRECTION_IN) return CMD_INVALID_FRAME;

    i2c_address = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;
    LOG_DEBUG("ADDRESS 0x%02x" EOL, i2c_address);

    i2c_reg = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;
    LOG_DEBUG("REG 0x%02x" EOL, i2c_reg);

    if (I2C_ReadRegFromSlave (i2c_address, i2c_reg, &data8) != I2C_SUCCESS) {
        return CMD_FAILURE;
    }

    SPI_Transfer (data8, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;
    LOG_DEBUG("DATA 0x%02x" EOL, data8);

    return CMD_SUCCESS;
}

static void Mode_I2C_Selected ()
{
    LOG_DEBUG("I2C CMD_");
    switch (protocol_cmd)
    {
        case CMD_I2C_INIT:
            cmd_result = Command_I2C_Init ();
        break;

        case CMD_I2C_WRITE:
            cmd_result = Command_I2C_Write ();
        break;

        case CMD_I2C_READ:
            cmd_result = Command_I2C_Read ();
        break;

        case CMD_I2C_READREG:
            cmd_result = Command_I2C_ReadReg ();
        break;

        default:
            cmd_result = CMD_INVALID_COMMAND;
        break;
    }
    if (cmd_result != CMD_SUCCESS) Error_Fetching ();
    LOG_DEBUG("I2C END" EOL);
}

// ----------------------------------------------------------- Software SPI functions -----------------------------------------------------------
#undef _DEBUG_
#ifdef DEBUG_MODE_SPI
#define _DEBUG_ 1
#else
#define _DEBUG_ 0
#endif

static Command_Result_Type Command_SoftSPI_Init ()
{
    LOG_DEBUG("INIT" EOL);
    if (payload_size != 5 || protocol_dir != DIRECTION_OUT) return CMD_INVALID_FRAME;

    spi_mode = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;

    clock = 0;
    for (i=0; i<4; i++) {
        data8 = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
        if (sync_mode) return CMD_SPI_TIMEOUT;
        clock = ((uint32_t)(clock << 8)) | data8;
    }

    LOG_DEBUG("MODE %d" EOL, spi_mode);
    LOG_DEBUG("CLOCK %d" EOL, clock);

    if (spi_mode > 3) {
        LOG_DEBUG("Invalid SPI mode" EOL);
        return CMD_FAILURE;
    }

    Soft_SPI_Init(spi_mode, clock);
    return CMD_SUCCESS;
}

static Command_Result_Type Command_SoftSPI_Transfer8 ()
{
    LOG_DEBUG("TRANSFER8" EOL);
    if (payload_size < 2 || protocol_dir != DIRECTION_OUT) return CMD_INVALID_FRAME;

    uint8_t request = 0, response = DUMMY_DATA;
    for (i=0; i<payload_size; i++)
    {
        request = SPI_Transfer(response, SPI_TIMEOUT_MS);
        if (sync_mode) return CMD_SPI_TIMEOUT;

        LOG_DEBUG("REQ8 0x%02x - 0x%02x" EOL, request, response);
        response = Soft_SPI_Transfer8(request);
    }
    return CMD_SUCCESS;
}

static Command_Result_Type Command_SoftSPI_Transfer16 ()
{
    LOG_DEBUG("TRANSFER16" EOL);
    if (payload_size < 4 || protocol_dir != DIRECTION_OUT) return CMD_INVALID_FRAME;

    uint16_t request = 0, response = DUMMY_DATA16;
    for (i=0; i<(payload_size/2); i++)
    {
        uint8_t msb = SPI_Transfer(response >> 8, SPI_TIMEOUT_MS);
        if (sync_mode) return CMD_SPI_TIMEOUT;

        uint8_t lsb = SPI_Transfer(response & 0xFF, SPI_TIMEOUT_MS);
        if (sync_mode) return CMD_SPI_TIMEOUT;

        request = (msb << 8) | lsb;

        LOG_DEBUG("REQ16 0x%04x - 0x%04x" EOL, request, response);
        response = Soft_SPI_Transfer16(request);
    }
    return CMD_SUCCESS;
}

static void Mode_SPI_Selected ()
{
    LOG_DEBUG("SPI CMD_");
    switch (protocol_cmd)
    {
        case CMD_SPI_INIT:
            cmd_result = Command_SoftSPI_Init ();
        break;

        case CMD_SPI_TRANSFER8:
            cmd_result = Command_SoftSPI_Transfer8 ();
        break;

        case CMD_SPI_TRANSFER16:
            cmd_result = Command_SoftSPI_Transfer16 ();
        break;

        default:
            cmd_result = CMD_INVALID_COMMAND;
        break;
    }
    if (cmd_result != CMD_SUCCESS) Error_Fetching ();
    LOG_DEBUG("SPI END" EOL);
}

// ----------------------------------------------------------- UART functions -----------------------------------------------------------
#undef _DEBUG_
#ifdef DEBUG_MODE_UART
#define _DEBUG_ 1
#else
#define _DEBUG_ 0
#endif

static Command_Result_Type Command_UART_Configure ()
{
    LOG_DEBUG("UART_CONFIGURE" EOL);
    if (payload_size != 6 || protocol_dir != DIRECTION_OUT) return CMD_INVALID_FRAME;

    clock = 0;
    for (i=0; i<4; i++) {
        data8 = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
        if (sync_mode) return CMD_SPI_TIMEOUT;
        clock = ((uint32_t)(clock << 8)) | data8;
    }

    stop_bits = (UART_StopBits)SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;

    parity = (UART_Parity)SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;

    UART_Configure(clock, stop_bits, parity);
    LOG_DEBUG("BAUDRATE %d" EOL, clock);

    return CMD_SUCCESS;
}

static Command_Result_Type Command_UART_Write ()
{
    LOG_DEBUG("UART_WRITE" EOL);
    if (payload_size < 1 || protocol_dir != DIRECTION_OUT) return CMD_INVALID_FRAME;

    for (i=0; i<payload_size; i++) {
        tx_buffer[i] = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
        if (sync_mode) return CMD_SPI_TIMEOUT;
        LOG_DEBUG("DATA 0x%02x" EOL, tx_buffer[i]);
    }

    if (UART_Send(tx_buffer, payload_size) != UART_SUCCESS) {
        return CMD_FAILURE;
    }

    return CMD_SUCCESS;
}

static Command_Result_Type Command_UART_Read ()
{
    LOG_DEBUG("UART_READ" EOL);
    if (payload_size < 2 || protocol_dir != DIRECTION_IN) return CMD_INVALID_FRAME;

    if (UART_Receive(tx_buffer, payload_size-1, &data8) != UART_SUCCESS) {
        return CMD_FAILURE;
    }
    LOG_DEBUG("RECEIVED %d" EOL, data8);

    // Send actual received count
    SPI_Transfer(data8, SPI_TIMEOUT_MS);
    if (sync_mode) return CMD_SPI_TIMEOUT;

    if (data8) {
        // Send received data
        for (i=0; i<data8; i++) {
            SPI_Transfer(tx_buffer[i], SPI_TIMEOUT_MS);
            if (sync_mode) return CMD_SPI_TIMEOUT;
        }

        for (i=0; i<data8; i++) {
            LOG_DEBUG("DATA 0x%02x" EOL, tx_buffer[i]);
        }
    }
    return CMD_SUCCESS;
}

static void Mode_UART_Selected ()
{
    LOG_DEBUG("UART CMD_");
    switch (protocol_cmd)
    {
        case CMD_UART_CONFIGURE:
            cmd_result = Command_UART_Configure ();
        break;

        case CMD_UART_WRITE:
            cmd_result = Command_UART_Write ();
        break;

        case CMD_UART_READ:
            cmd_result = Command_UART_Read ();
        break;

        default:
            cmd_result = CMD_INVALID_COMMAND;
        break;
    }
    if (cmd_result != CMD_SUCCESS) Error_Fetching ();
    LOG_DEBUG("UART END" EOL);
}

// ----------------------------------------------------------- Wait for Command function -----------------------------------------------------------
#undef _DEBUG_
#ifdef LOG_DEBUG_ENABLED
#define _DEBUG_ 1
#else
#define _DEBUG_ 0
#endif

void Wait_For_Command ()
{
    if (sync_mode) {
        LOG_DEBUG("Sync WAITING ");
    } else {
        LOG_DEBUG("READY" EOL);
    }

    protocol_start = SPI_Transfer(sync_mode ? SYNC_RESPONSE : DUMMY_DATA, -1);
    if (protocol_start == SYNC_MAGIC) {
        sync_mode ^= 1;
        if (sync_mode) {
            LOG_DEBUG("Sync DETECTED" EOL);
        } else {
            LOG_DEBUG("Sync DONE" EOL);
        }
        return;
    }
    if (sync_mode) {
        LOG_DEBUG("[0x%02X]" EOL, protocol_start);
        return;
    }

    LOG_DEBUG("START 0x%02x" EOL, protocol_start);

    protocol_dir  = (protocol_start & 0x01);
    protocol_cmd  = (protocol_start >> 1) & 0x1F;
    protocol_mode = (protocol_start >> 6) & 0x03;

    LOG_DEBUG("M 0x%02X C 0x%02X D 0x%02X" EOL, protocol_mode,  protocol_cmd,  protocol_dir);

    payload_size = SPI_Transfer(DUMMY_DATA, SPI_TIMEOUT_MS);
    if (sync_mode) return;
    LOG_DEBUG("SIZE %d" EOL, payload_size);

    if (protocol_start == CMD_VERSION) {
        LOG_DEBUG("VERSION ");
        if (payload_size == 2) {
            SPI_Transfer(VERSION_MAJOR, SPI_TIMEOUT_MS);
            if (sync_mode) return;
            LOG_DEBUG("%d.", VERSION_MAJOR);

            SPI_Transfer(VERSION_MINOR, SPI_TIMEOUT_MS);
            if (sync_mode) return;
            LOG_DEBUG("%d" EOL, VERSION_MINOR);
        } else {
            sync_mode = 1;
        }
        return;
    }

    switch (protocol_mode)
    {
        case MODE_GPIO:
            Mode_GPIO_Selected ();
        break;

        case MODE_I2C:
            Mode_I2C_Selected ();
        break;

        case MODE_SPI:
            Mode_SPI_Selected ();
        break;

        case MODE_UART:
            Mode_UART_Selected ();
        break;
    }
    LOG_DEBUG(EOL);
}
