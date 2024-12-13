/**
 * @file    flash_s25flxxxl.c
 * @author  Thierry Zinkeng (tz@dc.systems)
 * @brief   This file provides the S25FLXXXL QSPI drivers.
 * @version  1.0.0
 * @date    2024-11-21
 *
 * @copyright Copyright DC Systems BV (c) 2024
 *
 */

#include "flash_s25flxxxl.h"
#include <stdio.h>
#include <string.h> // memcpy

/** @addtogroup Flash_Driver
 * @{
 */

#ifndef S25FLXXXL_DUMMY_CYCLES_READ_DUAL_INOUT
#define S25FLXXXL_DUMMY_CYCLES_READ_DUAL_INOUT 4U
#endif

#ifndef S25FLXXXL_DUMMY_CYCLES_READ_QUAD_INOUT
#define S25FLXXXL_DUMMY_CYCLES_READ_QUAD_INOUT 6U
#endif

// Timeouts (in ms)
#define S25FL_SPI_TIMEOUT   1000   // Typical Page Program Timeout
#define S25FL_ERASE_TIMEOUT 200000 // Typical Erase Timeout

#define CR2V_ADDR 0x000003 // Address of CR2V register

#define LOG_ERRORN(format, ...) printf("[ERROR] fn=%s(), " format "\r\n", __FUNCTION__, ##__VA_ARGS__)
#define LOG_ERROR(format, ...)  printf("[ERROR] fn=%s(), " format, __FUNCTION__, ##__VA_ARGS__)
#define LOG_INFON(format, ...)  printf("[INFO ] fn=%s(), " format "\r\n", __FUNCTION__, ##__VA_ARGS__)
#define LOG_INFO(format, ...)   printf("[INFO ] fn=%s(), " format, __FUNCTION__, ##__VA_ARGS__)

static flash_command_cb command_cb;
__IO uint8_t            CmdCplt, RxCplt, TxCplt;
volatile uint8_t        cmd_complete = 0, data_received = 0;

/* Hardware backend functions */
static void    start_transmission(FLASH_S25FL_t* _ctx);
static void    end_transmission(FLASH_S25FL_t* _ctx);
static int     flash_write(FLASH_S25FL_t* _ctx, const uint8_t* _data, const uint16_t _size);
static int     flash_read(FLASH_S25FL_t* _ctx, uint8_t* _data, uint16_t _size);
static uint8_t get_command_for_24bit_addr(FLASH_S25FL_t* _ctx, uint8_t _data[4], uint8_t _cmd, uint32_t _address);
static uint8_t get_command_for_32bit_addr(FLASH_S25FL_t* _ctx, uint8_t* _data, uint8_t _cmd, uint32_t _address);
static int     flash_xspi_prepare_cmd(FLASH_S25FL_t* _ctx, const uint32_t _instruction, const uint32_t _address,
                                      const uint32_t _data_length, const uint32_t _dummy_cycles);
static int     flash_backend_init(FLASH_S25FL_t* _ctx, S25FLXXXL_backend_t _type);

/* Flash driver, Command Protocol functions */
static int flash_write_enable_set(FLASH_S25FL_t* _ctx);

/**
 * @brief Assert CS GPIO
 *
 * @param _ctx Pointe to Flash context driver
 *
 */
static void start_transmission(FLASH_S25FL_t* _ctx)
{
    HAL_GPIO_WritePin(_ctx->chp_select.port, _ctx->chp_select.pin, GPIO_PIN_RESET); // Pull CS low
}

/**
 * @brief Release CS GPIO
 *
 * @param _ctx Pointe to Flash context driver
 *
 */
static void end_transmission(FLASH_S25FL_t* _ctx)
{
    HAL_GPIO_WritePin(_ctx->chp_select.port, _ctx->chp_select.pin, GPIO_PIN_SET); // Pull CS high
}

/**
 * @brief Send SPI command
 *
 * @param _ctx  Pointer to Flash context
 * @param _data Pointer to data to be sent
 * @param _size Size of data to be sent
 *
 * @return int 0 if success
 */
static int flash_write(FLASH_S25FL_t* _ctx, const uint8_t* _data, const uint16_t _size)
{
    if (_ctx->spi != NULL)
    {
        return ((HAL_SPI_Transmit(_ctx->spi, _data, _size, S25FL_SPI_TIMEOUT) == HAL_OK) ? S25FLXXXL_OK
                                                                                         : S25FLXXXL_ERROR);
    }
    else if (_ctx->xspi != NULL)
    {
        return ((HAL_XSPI_Command(_ctx->xspi, &_ctx->backend.xspi_cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK)
                    ? S25FLXXXL_OK
                    : S25FLXXXL_ERROR);
    }
    else
    {
        Error_Handler();
        return S25FLXXXL_ERROR;
    }

    return S25FLXXXL_ERROR;
}

/**
 * @brief Read SPI command
 *
 * @param _ctx  Pointer to Flash context
 * @param _data Pointer to data to store the values
 * @param _size Size of data to be stored the values
 *
 * @return int 0 if success
 */
static int flash_read(FLASH_S25FL_t* _ctx, uint8_t* _data, uint16_t _size)
{
    int ret;

    if (_ctx->spi != NULL)
    {
        return ((HAL_SPI_Receive(_ctx->spi, _data, _size, S25FL_SPI_TIMEOUT) == HAL_OK) ? S25FLXXXL_OK
                                                                                        : S25FLXXXL_ERROR);
    }
    else if (_ctx->xspi != NULL)
    {
        return ((HAL_XSPI_Receive(_ctx->xspi, _data, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK) ? S25FLXXXL_OK
                                                                                                : S25FLXXXL_ERROR);
        // ret = HAL_XSPI_Receive_DMA(_ctx->xspi, _data);
        // if (ret != HAL_OK)
        // {
        //     LOG_ERROR("error_code=0x%lx\r\n", HAL_XSPI_GetError(_ctx->xspi));
        // }
        // else
        // {
        //     // Wait for data reception to complete
        //     while (!data_received)
        //         ;
        //     return S25FLXXXL_OK;
        // }
    }
    else
    {
        Error_Handler();
        return S25FLXXXL_ERROR;
    }

    return S25FLXXXL_ERROR;
}

/**
 * @brief Get the command for 24bit addr object
 *
 * @param _data Pointer to data
 * @param _cmd Which CMD should be executed
 * @param _address
 *
 * @return uint8_t return how many bytes where written
 */
static uint8_t get_command_for_24bit_addr(FLASH_S25FL_t* _ctx, uint8_t _data[4], uint8_t _cmd, uint32_t _address)
{
    uint8_t pos = 0;

    UNUSED(_ctx);

    _data[pos++] = _cmd;                    // Operation command
    _data[pos++] = (_address >> 16) & 0xFF; // Address MSB for 24 bit address | Address Middle for 32 bit address
    _data[pos++] = (_address >> 8) & 0xFF;  // Address Middle
    _data[pos++] = _address & 0xFF;         // Address LSB

    return pos;
}

/**
 * @brief Get the command for 32bit addr object
 *
 * @param _data Pointer to data
 * @param _cmd Which CMD should be executed
 * @param _address Address to be written
 *
 * @return uint8_t return how many bytes where written
 */
static uint8_t get_command_for_32bit_addr(FLASH_S25FL_t* _ctx, uint8_t* _data, uint8_t _cmd, uint32_t _address)
{
    uint8_t pos = 0;

    UNUSED(_ctx);

    _data[pos++] = _cmd;                    // Operation command
    _data[pos++] = (_address >> 24) & 0xFF; // Address MSB for 32 bit address
    _data[pos++] = (_address >> 16) & 0xFF; // Address Middle for 32 bit address
    _data[pos++] = (_address >> 8) & 0xFF;  // Address Middle
    _data[pos++] = _address & 0xFF;         // Address LSB

    return pos;
}

static int flash_xspi_prepare_cmd(FLASH_S25FL_t* _ctx, const uint32_t _instruction, const uint32_t _address,
                                  const uint32_t _data_length, const uint32_t _dummy_cycles)
{
    XSPI_RegularCmdTypeDef* s_command = &_ctx->backend.xspi_cmd;

    memset(s_command, 0, sizeof(XSPI_RegularCmdTypeDef));

    /* Configure the command */
    s_command->OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
    s_command->InstructionMode    = HAL_XSPI_INSTRUCTION_1_LINE;
    s_command->Instruction        = _instruction; // Command instruction
    s_command->InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
    s_command->InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;

    // Address phase
    s_command->AddressMode = HAL_XSPI_ADDRESS_1_LINE; // 1-line address phase
    s_command->Address     = _address;                // Start of bytes read from
    s_command->AddressWidth =
        (_ctx->address_mode == S25FLXXXL_ADDR_MODE_24BIT) ? HAL_XSPI_ADDRESS_24_BITS : HAL_XSPI_ADDRESS_32_BITS;

    // Data phase
    s_command->DataMode           = HAL_XSPI_DATA_1_LINE; // 1-line data phase
    s_command->DataLength         = _data_length;         // Read  bytes
    s_command->DummyCycles        = _dummy_cycles;        // dummy cycles as per datasheet
    s_command->DQSMode            = HAL_XSPI_DQS_DISABLE;
    s_command->SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
    s_command->AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;

    return S25FLXXXL_OK;
}

/*==============================================*/
/* Callbacks implementation */
/*==============================================*/
void xspi_error_cb(XSPI_HandleTypeDef* hxspi) {}
/*==============================================*/
/* End of Callbacks implementation */
/*==============================================*/

static int flash_backend_init(FLASH_S25FL_t* _ctx, S25FLXXXL_backend_t _type)
{
    flash_backend_t* backend = &_ctx->backend;

    /* Initialize hardware interfaces */
    switch (_type)
    {
    case S25FLXXXL_BACKEND_SPI:
        backend->cmd_cb      = get_command_for_24bit_addr;
        backend->start_tx_cb = start_transmission;
        backend->stop_tx_cb  = end_transmission;
        break;

    case S25FLXXXL_BACKEND_XSPI:
    default:
        // HAL_XSPI_RegisterCallback(_ctx->xspi, HAL_XSPI_ERROR_CB_ID, NULL);

        _ctx->xspi            = &S25FLXXXL_XSPI_INSTANCE;
        _ctx->spi             = NULL;
        _ctx->chp_select.port = NULL;
        _ctx->chp_select.pin  = 0;
        backend->cmd_cb       = NULL;
        backend->start_tx_cb  = NULL;
        backend->stop_tx_cb   = NULL;
        backend->prep_cmd_cb  = flash_xspi_prepare_cmd;
        break;
    }

    backend->read_cb  = flash_read;
    backend->write_cb = flash_write;

    return S25FLXXXL_OK;
}

static int flash_write_enable_set(FLASH_S25FL_t* _ctx)
{
    uint8_t          instruction = S25FLXXXL_CMD_WREN;
    int              ret         = S25FLXXXL_OK;
    flash_backend_t* backend     = &_ctx->backend;

    if (backend->prep_cmd_cb != NULL)
    {
        backend->prep_cmd_cb(_ctx, instruction, 0, 1, 8);
    }

    // Pull CS low to start communication
    if (backend->start_tx_cb != NULL)
        backend->start_tx_cb(_ctx);

    if (backend->write_cb(_ctx, &instruction, 1) != S25FLXXXL_OK)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

end:
    // Pull CS high to end communication
    if (backend->stop_tx_cb != NULL)
        backend->stop_tx_cb(_ctx);

    return ret;
}

static int flash_write_disable_set(FLASH_S25FL_t* _ctx)
{
    uint8_t          instruction = S25FLXXXL_CMD_WRDI;
    int              ret         = S25FLXXXL_OK;
    flash_backend_t* backend     = &_ctx->backend;

    if (backend->prep_cmd_cb != NULL)
    {
        backend->prep_cmd_cb(_ctx, instruction, 0, 1, 8);
    }

    // Pull CS low to start communication
    if (backend->start_tx_cb != NULL)
        backend->start_tx_cb(_ctx);

    if (backend->write_cb(_ctx, &instruction, 1) != S25FLXXXL_OK)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

end:
    // Pull CS high to end communication
    if (backend->stop_tx_cb != NULL)
        backend->stop_tx_cb(_ctx);

    return ret;
}

static int flash_read_single_register(FLASH_S25FL_t* _ctx, uint16_t _reg, uint8_t* _out)
{
    uint8_t          instruction = _reg;
    uint8_t          response    = 0;
    int              ret         = S25FLXXXL_OK;
    flash_backend_t* backend     = &_ctx->backend;

    if (!_ctx->initialized)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

    if (backend->prep_cmd_cb != NULL)
    {
        backend->prep_cmd_cb(_ctx, instruction, 0, 1, 8);
    }

    // Pull CS low to start communication
    if (backend->start_tx_cb != NULL)
        backend->start_tx_cb(_ctx);

    if (backend->write_cb(_ctx, &instruction, 1) != S25FLXXXL_OK)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

    if (backend->read_cb(_ctx, &response, 1) != S25FLXXXL_OK)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

    *_out = response;

end:
    // Pull CS high to end communication
    if (backend->stop_tx_cb != NULL)
        backend->stop_tx_cb(_ctx);

    return ret;
}

/**
 * @brief Reads an amount of SFDP data from the QSPI memory.
 *         SFDP : Serial Flash Discoverable Parameter
 *
 * @param _ctx Pointer to Flash context
 * @param _start_address Start address to read from
 * @param _n_bytes Number of bytes to read
 * @param _p_out Pointer to output data
 *
 * @return int 0 if success, otherwise failed
 */
static int flash_read_sfdp(FLASH_S25FL_t* _ctx, const uint32_t _start_address, const uint16_t _n_bytes, uint8_t* _p_out)
{
    uint8_t          instruction = S25FLXXXL_CMD_RSFDP;
    int              ret         = S25FLXXXL_OK;
    flash_backend_t* backend     = &_ctx->backend;

    if (!_ctx->initialized)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

    if (backend->prep_cmd_cb != NULL)
    {
        backend->prep_cmd_cb(_ctx, instruction, _start_address, _n_bytes, CONF_QSPI_DUMMY_CLOCK);
    }

    // Pull CS low to start communication
    if (backend->start_tx_cb != NULL)
        backend->start_tx_cb(_ctx);

    if (backend->write_cb(_ctx, &instruction, 1) != S25FLXXXL_OK)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

    if (backend->read_cb(_ctx, _p_out, _n_bytes) != S25FLXXXL_OK)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

end:
    // Pull CS high to end communication
    if (backend->stop_tx_cb != NULL)
        backend->stop_tx_cb(_ctx);

    return ret;
}

static int flash_get_addr_mode(FLASH_S25FL_t* _ctx, S25FLXXXL_address_mode_t* _mode)
{
    uint8_t          cmd[5];
    const uint8_t    instruction = S25FLXXXL_CMD_RDAR;
    const uint8_t    address     = CR2V_ADDR;
    const uint8_t    cmd_length  = 4;
    int              status      = S25FLXXXL_OK;
    Reg_CR2V_t       reg         = {0};
    flash_backend_t* backend     = &_ctx->backend;

    // flash_xspi_prepare_cmd(_ctx, instruction, address, 1, 8);

    if (backend->prep_cmd_cb != NULL)
        backend->prep_cmd_cb(_ctx, instruction, address, 1, 8);

    if (backend->cmd_cb != NULL)
        backend->cmd_cb(_ctx, cmd, S25FLXXXL_CMD_RDAR, CR2V_ADDR);

    if (backend->start_tx_cb != NULL)
        backend->start_tx_cb(_ctx); // Pull CS low

    if (backend->write_cb(_ctx, cmd, cmd_length) != S25FLXXXL_OK)
    {
        status = S25FLXXXL_ERROR;
        goto end;
    }

    if (backend->read_cb(_ctx, &reg.value, 1) != S25FLXXXL_OK)
    {
        status = S25FLXXXL_ERROR;
        goto end;
    }

    *_mode = reg.bits.ADS;

end:
    if (backend->stop_tx_cb != NULL)
        backend->stop_tx_cb(_ctx); // Pull CS high

    return status;
}

/**
 * @brief Read status register
 *
 * @param _ctx
 * @param _reg
 *
 * @return int
 */
static int flash_read_status_register(FLASH_S25FL_t* _ctx, Reg_SR1V_t* _reg)
{
    uint8_t instruction = S25FLXXXL_REG_RDSR1;

    return flash_read_single_register(_ctx, instruction, &_reg->value);
}

static int flash_write_in_progress(FLASH_S25FL_t* _ctx, uint8_t* _out)
{
    Reg_SR1V_t sr1v = {0xFF};

    if (flash_read_status_register(_ctx, &sr1v) != S25FLXXXL_OK)
    {
        return S25FLXXXL_ERROR;
    }

    *_out = sr1v.bits.WIP;

    return S25FLXXXL_OK;
}

/** @defgroup S25FLxxxl_Exported_Functions S25FLxxxl Exported Functions
 * @{
 */

int Flash_S25FLXXXL_get_flash_info(Flash_S25FL_info_t* pInfo)
{
    /* Configure the structure with the memory configuration */
    pInfo->flash_size           = EXT_FLASH_SIZE;
    pInfo->erase_sector_size    = EXT_FLASH_SECTOR_SIZE;
    pInfo->erase_sectors_number = EXT_FLASH_SECTOR_NUM;
    pInfo->program_page_size    = EXT_FLASH_PAGE_SIZE;
    pInfo->prog_pages_number    = EXT_FLASH_PAGE_NUM;

    return S25FLXXXL_OK;
}

int Flash_S25FLXXXL_print_flash_info(FLASH_S25FL_t* _ctx)
{
    printf("Flash information:\r\n"
           "    HARDWARE_CONFIG         = %s\r\n"
           "    ADDRESS_MODE            = %d byte address\r\n"
           "    FLASH_SIZE              = %lu MB\r\n"
           "    FLASH_SECTOR_SIZE       = %lu kB\r\n"
           "    FLASH_SECTOR_NUMBERS    = %lu \r\n"
           "    FLASH_PAGE_SIZE         = %lu B\r\n"
           "    FLASH_PAGE_NUMBERS      = %lu \r\n",
           (_ctx->xspi == NULL) ? "SPI" : "OCTOSPI Single-Mode",
           ((_ctx->address_mode == S25FLXXXL_ADDR_MODE_24BIT) ? 3 : 4), (_ctx->info.flash_size / 1024 / 1024),
           (_ctx->info.erase_sector_size / 1024), (_ctx->info.erase_sectors_number), (_ctx->info.program_page_size),
           (_ctx->info.prog_pages_number));

    return S25FLXXXL_OK;
}

bool Flash_S25FLXXXL_is_available(FLASH_S25FL_t* _ctx)
{
    uint8_t        sfdp[0x100]   = {0};
    const uint32_t start_address = 0x000000;
    uint32_t       n_bytes       = 4;

    int status = flash_read_sfdp(_ctx, start_address, n_bytes, sfdp);

    return ((status == S25FLXXXL_OK) && ((sfdp[0] == 'S') && (sfdp[1] == 'F') && (sfdp[2] == 'D') && (sfdp[3] == 'P')));
}

int Flash_S25FLXXXL_write_enable(FLASH_S25FL_t* _ctx)
{
    int        ret  = S25FLXXXL_OK;
    Reg_SR1V_t sr1v = {0};

    if (flash_write_enable_set(_ctx) != S25FLXXXL_OK)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

    while (sr1v.bits.WEL == 0)
    {
        flash_read_status_register(_ctx, &sr1v);
    }

end:
    return ret;
}

int Flash_S25FLXXXL_write_disable(FLASH_S25FL_t* _ctx)
{
    int        ret  = S25FLXXXL_OK;
    Reg_SR1V_t sr1v = {0xFF};

    if (flash_write_disable_set(_ctx) != S25FLXXXL_OK)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

    while (sr1v.bits.WEL)
    {
        flash_read_status_register(_ctx, &sr1v);
    }

end:
    return ret;
}

int Flash_S25FLXXXL_helper_sector_number_get(uint32_t start_address)
{
    if (start_address < FLASH_BASE_ADDR)
    {
        return S25FLXXXL_ERROR; // Error: Address is below FLASH base
    }

    uint32_t offset = start_address - FLASH_BASE_ADDR;

    return offset / EXT_FLASH_SECTOR_SIZE;
}

int Flash_S25FLXXXL_init(FLASH_S25FL_t* _ctx)
{
    int status = S25FLXXXL_OK;

    /* According to datasheet (300uS) */
    // HAL_Delay(1);

    // Initialize hardware functions used to read/write to external flash driver
    flash_backend_init(_ctx, S25FLXXXL_BACKEND_XSPI);

    /* Get FLASH information */
    if (Flash_S25FLXXXL_get_flash_info(&_ctx->info) != S25FLXXXL_OK)
    {
        status = S25FLXXXL_ERROR;
        goto end;
    }
    Flash_S25FLXXXL_print_flash_info(_ctx);

    // IMPORTANT: Should called here, otherwise initialization cannot be possible
    _ctx->initialized = true;

    // QSPI_ReadID(_ctx);
    // QSPI_Read(_ctx);

    if (Flash_S25FLXXXL_is_available(_ctx) != true)
    {
        _ctx->initialized = false;
        status            = S25FLXXXL_ERROR;
        goto end;
    }

    /* Get Flash address mode*/
    // if (flash_get_addr_mode(_ctx, &_ctx->address_mode) != S25FLXXXL_OK)
    // {
    //     status = S25FLXXXL_ERROR;
    //     goto end;
    // }
    // // Change bit address just in case chip is configured on wrong way
    // if (_ctx->address_mode == S25FLXXXL_ADDR_MODE_32BIT)
    // {
    //     _ctx->cmd_cb = get_command_for_32bit_addr;
    // }

    // if (!Flash_S25FLXXXL_reset_memory(_ctx))
    // {
    //     return status;
    // }

end:
    return status;
}

// TODO: Monitor WIP bit
int Flash_S25FLXXXL_auto_polling(FLASH_S25FL_t* _ctx)
{
    int status = S25FLXXXL_OK;

end:
    return status;
}

int Flash_S25FLXXXL_read_data(FLASH_S25FL_t* _ctx, uint32_t _address, uint32_t _n_bytes, uint8_t* _p_data)
{
    uint8_t cmd_length;
    uint8_t cmd[5]; // Command + Address
    uint8_t instruction =
        (_ctx->address_mode == S25FLXXXL_ADDR_MODE_24BIT) ? S25FLXXXL_CMD_FAST_READ : S25FLXXXL_CMD_4FAST_READ;
    int              ret     = S25FLXXXL_OK;
    flash_backend_t* backend = &_ctx->backend;

    if (!_ctx->initialized)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

    // Build XSPI package
    if (backend->prep_cmd_cb != NULL)
    {
        backend->prep_cmd_cb(_ctx, instruction, _address, _n_bytes, 0);
    }

    // Build SPI packet based on address mode
    if (backend->cmd_cb != NULL)
        cmd_length = backend->cmd_cb(_ctx, cmd, instruction, _address);

    if (backend->start_tx_cb != NULL)
        backend->start_tx_cb(_ctx); // Pull CS low

    if (backend->write_cb(_ctx, cmd, cmd_length) != S25FLXXXL_OK)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

    if (backend->read_cb(_ctx, _p_data, _n_bytes) != S25FLXXXL_OK)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

end:
    // Pull CS high to end communication
    if (backend->stop_tx_cb != NULL)
        backend->stop_tx_cb(_ctx);

    return ret;
}

int Flash_S25FLXXXL_block_erase(FLASH_S25FL_t* _ctx, uint32_t _block_address)
{
    uint8_t cmd[5];
    uint8_t instruction = (_ctx->address_mode == S25FLXXXL_ADDR_MODE_24BIT) ? S25FLXXXL_CMD_BE : S25FLXXXL_CMD_4BE;
    uint8_t wip         = 1;
    int     ret         = S25FLXXXL_OK;

    if (!_ctx->initialized)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

    /* Before the Sector Erase (SE) command can be accepted by the device,
       a Write Enable (WREN) command must be issued.
     */
    if (Flash_S25FLXXXL_write_enable(_ctx) != S25FLXXXL_OK)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

    // Build SPI packet based on address mode
    const uint8_t cmd_length = _ctx->cmd_cb(_ctx, cmd, instruction, _block_address);

    // Pull CS low to start communication
    if (_ctx->start_tx_cb != NULL)
        _ctx->start_tx_cb(_ctx);

    if (_ctx->write_cb(_ctx, cmd, cmd_length) != S25FLXXXL_OK)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

    // Pull CS high to end communication
    if (_ctx->stop_tx_cb != NULL)
        _ctx->stop_tx_cb(_ctx);

    // Wait for complete last operation
    while (wip)
    {
        flash_write_in_progress(_ctx, &wip);
    }

    /* Write disable */
    if (Flash_S25FLXXXL_write_disable(_ctx) != S25FLXXXL_OK)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

end:
    // Pull CS high to end communication
    if (_ctx->stop_tx_cb != NULL)
        _ctx->stop_tx_cb(_ctx);

    return ret;
}

int Flash_S25FLXXXL_erase_sector(FLASH_S25FL_t* _ctx, uint32_t _sector_address)
{
    uint8_t cmd[5];
    uint8_t instruction = (_ctx->address_mode == S25FLXXXL_ADDR_MODE_24BIT) ? S25FLXXXL_CMD_SE : S25FLXXXL_CMD_4SE;
    uint8_t wip         = 1;
    int     ret         = S25FLXXXL_OK;

    if (!_ctx->initialized)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

    /* Before the Sector Erase (SE) command can be accepted by the device,
       a Write Enable (WREN) command must be issued.
     */
    if (Flash_S25FLXXXL_write_enable(_ctx) != S25FLXXXL_OK)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

    // Build SPI packet based on address mode
    const uint8_t cmd_length = _ctx->cmd_cb(_ctx, cmd, instruction, _sector_address);

    // Pull CS low to start communication
    if (_ctx->start_tx_cb != NULL)
        _ctx->start_tx_cb(_ctx);

    if (_ctx->write_cb(_ctx, cmd, cmd_length) != S25FLXXXL_OK)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

    // Pull CS high to end communication
    if (_ctx->stop_tx_cb != NULL)
        _ctx->stop_tx_cb(_ctx);

    // Wait for complete last operation
    while (wip)
    {
        flash_write_in_progress(_ctx, &wip);
    }

    // flash_read_status_register(_ctx, &sr1v);
    // while (sr1v.bits.WIP)
    // {
    //     flash_read_status_register(_ctx, &sr1v);
    // }

    /* Write disable */
    if (Flash_S25FLXXXL_write_disable(_ctx) != S25FLXXXL_OK)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

end:
    // Pull CS high to end communication
    if (_ctx->stop_tx_cb != NULL)
        _ctx->stop_tx_cb(_ctx);

    return ret;
}

int Flash_S25FLXXXL_chip_erase(FLASH_S25FL_t* _ctx)
{
    uint8_t instruction = (_ctx->address_mode == S25FLXXXL_ADDR_MODE_24BIT) ? S25FLXXXL_CMD_CE : S25FLXXXL_CMD_4CE;
    uint8_t wip         = 1;
    int     ret         = S25FLXXXL_OK;

    if (!_ctx->initialized)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

    /* Before the Chip Erase (CE) command can be accepted by the device,
       a Write Enable (WREN) command must be issued.
     */
    if (Flash_S25FLXXXL_write_enable(_ctx) != S25FLXXXL_OK)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

    // Pull CS low to start communication
    if (_ctx->start_tx_cb != NULL)
        _ctx->start_tx_cb(_ctx);

    if (_ctx->write_cb(_ctx, &instruction, 1) != S25FLXXXL_OK)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }
    // Pull CS high to end communication
    if (_ctx->stop_tx_cb != NULL)
        _ctx->stop_tx_cb(_ctx);

    // Wait for complete last operation
    while (wip)
    {
        flash_write_in_progress(_ctx, &wip);
    }

    /* Write disable */
    if (Flash_S25FLXXXL_write_disable(_ctx) != S25FLXXXL_OK)
    {
        ret = S25FLXXXL_ERROR;
        goto end;
    }

end:
    // Pull CS high to end communication
    if (_ctx->stop_tx_cb != NULL)
        _ctx->stop_tx_cb(_ctx);

    return ret;
}

// SPI Page Program
bool Flash_S25FLXXXL_page_program1(FLASH_S25FL_t* _ctx, uint8_t* _data, uint32_t _write_addr, uint16_t _size)
{
    uint8_t cmd[4 + 256]; // Command (4 bytes) + Max page size (256 bytes)
    bool    status = false;

    if (command_cb == NULL)
    {
        return false;
    }
    uint8_t cmd_length = command_cb(_ctx, cmd, S25FLXXXL_CMD_PP, _write_addr);

    memcpy(&cmd[cmd_length++], _data, _size);

    Flash_S25FLXXXL_write_enable(_ctx);
    // Pull CS low to start communication
    start_transmission(_ctx);

    // Send the Page Program command
    status = flash_write(_ctx, cmd, _size + cmd_length);

    // Pull CS high to end communication
    end_transmission(_ctx);

    Flash_S25FLXXXL_write_disable(_ctx);

    return status;
}

bool Flash_S25FLXXXL_init1(FLASH_S25FL_t* _ctx)
{
    uint32_t jedec_id = 0;
    bool     status   = false;

    HAL_Delay(100);
    // delay(100);

    if (!Flash_S25FLXXXL_reset_memory(_ctx))
    {
        return status;
    }

    // status = Flash_S25FXXXL_set_addr_mode(_ctx, S25FLXXXL_ADDR_MODE_24BIT);
    command_cb = &get_command_for_24bit_addr;
    status     = true;

    if (status)
    {
        status = Flash_S25FLXXXL_is_available(_ctx);

        if (status)
        {
            jedec_id = Flash_S25FXXXL_read_jedec_id(_ctx);

            if (jedec_id > 0)
            {
                uint32_t mfg_id = (jedec_id >> 16) & 0XFF;
                return (mfg_id == MANUFACTURER_ID_INFINEON); // if ManufacturerID is not Infineon S25FL256L flash (0x9F)
            }
        }
    }
    return status; // return memSize as per table in Flash_ReadJedecID() definition
}

/**
 * @brief Reads the JEDEC ID of the flash memory.
 * @param  _ctx Component object pointer
 * @return The JEDEC ID (3 bytes) as a uint32_t value.
 */
uint32_t Flash_S25FXXXL_read_jedec_id(FLASH_S25FL_t* _ctx)
{
    uint8_t id[3] = {0};

    if (Flash_S25FLXXXL_read_id(_ctx, id))
    {
        // Combine the ID bytes into a single uint32_t value
        return (id[0] << 16) | (id[1] << 8) | id[2];
    }

    return 0;
}

/**
 * @brief Reads the JEDEC ID of the flash memory.
 * @param  _ctx Component object pointer
 * @return The JEDEC ID (3 bytes) as a uint32_t value.
 */
bool Flash_S25FXXXL_set_addr_mode(FLASH_S25FL_t* _ctx, S25FLXXXL_address_mode_t _mode)
{
    uint8_t cmd[5] = {0};
    bool    status = false;

    if ((_mode != S25FLXXXL_ADDR_MODE_24BIT) && (_mode != S25FLXXXL_ADDR_MODE_32BIT))
    {
        return status;
    }

    if (!Flash_S25FLXXXL_write_enable(_ctx))
    {
        return status;
    }

    uint8_t length = get_command_for_24bit_addr(_ctx, cmd, S25FLXXXL_CMD_WRAR, CR2V_ADDR);

    cmd[length++] = (uint8_t)_mode; // Address mode (24-bit or 32-bit)

    start_transmission(_ctx); // Pull CS low

    status = flash_write(_ctx, cmd, length);

    end_transmission(_ctx); // Pull CS high

    if (status)
    {
        if (_mode == S25FLXXXL_ADDR_MODE_24BIT)
        {
            command_cb = &get_command_for_24bit_addr;
        }
        else if (_mode == S25FLXXXL_ADDR_MODE_32BIT)
        {
            command_cb = &get_command_for_32bit_addr;
        }
    }

    return status;
}

/**
 * @brief  Get Flash information
 * @param  _info pointer to information structure
 * @retval QSPI memory status
 */
bool Flash_S25FLXXXL_get_info(Flash_S25FL_info_t* _info)
{
    /* Configure the structure with the memory configuration */
    _info->flash_size           = S25FL256L_FLASH_SIZE;
    _info->erase_sector_size    = S25FL256L_SECTOR_SIZE;
    _info->erase_sectors_number = (S25FL256L_FLASH_SIZE / S25FL256L_SECTOR_SIZE);
    _info->program_page_size    = S25FL256L_PAGE_SIZE;
    _info->prog_pages_number    = (S25FL256L_FLASH_SIZE / S25FL256L_PAGE_SIZE);

    return true;
}

/**
 * @brief  This reads the manufactuerer id.
 * @param  _ctx Component object pointer
 * @param  _manufacturer_id Interface mode
 * @param _device_id the device id
 * @retval QSPI memory status
 */
bool Flash_S25FLXXXL_read_id_ext(FLASH_S25FL_t* _ctx, uint8_t* _manufacturer_id, uint8_t* _device_id)
{
    uint8_t cmd        = S25FLXXXL_CMD_RDID;
    uint8_t address[3] = {0, 0, 0}; // 3 dummy bytes for the command
    uint8_t id[2]      = {0};

    start_transmission(_ctx); // Pull CS low

    // Send the command and address
    if (flash_write(_ctx, &cmd, 1))
    {
        end_transmission(_ctx);
        return -1;
    }

    if (flash_write(_ctx, address, 3))
    {
        end_transmission(_ctx);
        return -1;
    }

    // Receive the ID bytes
    if (flash_read(_ctx, id, 2))
    {
        end_transmission(_ctx);
        return -1;
    }

    end_transmission(_ctx); // Pull CS high

    // Parse the results
    *_manufacturer_id = id[0];
    *_device_id       = id[1];

    return 0; // Success
}

// SPI Page Program
bool Flash_S25FLXXXL_page_program(FLASH_S25FL_t* _ctx, uint8_t* _data, uint32_t _write_addr, uint16_t _size)
{
    uint8_t cmd[4 + 256]; // Command (4 bytes) + Max page size (256 bytes)
    bool    status = false;

    if (command_cb == NULL)
    {
        return false;
    }
    uint8_t cmd_length = command_cb(_ctx, cmd, S25FLXXXL_CMD_PP, _write_addr);

    memcpy(&cmd[cmd_length++], _data, _size);

    Flash_S25FLXXXL_write_enable(_ctx);
    // Pull CS low to start communication
    start_transmission(_ctx);

    // Send the Page Program command
    status = flash_write(_ctx, cmd, _size + cmd_length);

    // Pull CS high to end communication
    end_transmission(_ctx);

    Flash_S25FLXXXL_write_disable(_ctx);

    return status;
}

// SPI Read
bool Flash_S25FLXXXL_read(FLASH_S25FL_t* _ctx, uint8_t* _data, uint32_t _read_addr, uint16_t _size)
{
    uint8_t cmd[5]; // Command + Address
    bool    status = false;

    if (command_cb == NULL)
    {
        return false;
    }
    const uint8_t cmd_length = command_cb(_ctx, cmd, S25FLXXXL_CMD_READ, _read_addr);

    // Pull CS low to start communication
    start_transmission(_ctx);

    // Send the Read command
    if (!flash_write(_ctx, cmd, cmd_length))
    {
        // Pull CS high to end communication
        end_transmission(_ctx);
        return status;
    }

    // Receive the data
    status = flash_read(_ctx, _data, _size);

    // Pull CS high to end communication
    end_transmission(_ctx);

    return status;
}

/**
 * @brief  Polling WIP(Write In Progress) bit become to 0
 * @param  _ctx Component object pointer
 * @param  _mode Interface mode
 * @retval QSPI memory status
 */
bool Flash_S25FLXXXL_auto_polling_mem_ready(FLASH_S25FL_t* _ctx)
{
    uint8_t cmd = S25FLXXXL_REG_RDSR1;
    uint8_t status;
    bool    result = false;

    do
    {
        // Pull CS low
        start_transmission(_ctx);

        // Send the "Read Status Register" command
        if (!flash_write(_ctx, &cmd, 1))
        {
            // Pull CS high
            end_transmission(_ctx);
            return result;
        }

        // Receive the status byte
        result = flash_read(_ctx, &status, 1);

        // Pull CS high
        end_transmission(_ctx);

    } while (status & S25FL256L_WIP_BIT); // Poll until WIP bit is cleared

    return result;
}

/**
 * @brief  This function set the QSPI memory in 4-byte address mode
 * @param  _ctx Component object pointer
 * @param  _mode Interface mode
 * @retval QSPI memory status
 */
bool Flash_S25FLXXXL_enter_4_bytes_address_mode(FLASH_S25FL_t* _ctx)
{
    uint8_t cmd    = S25FLXXXL_CMD_4BEN;
    bool    status = false;

    if (!Flash_S25FLXXXL_write_enable(_ctx))
    {
        return status;
    }

    // Pull CS low
    start_transmission(_ctx);

    status = flash_write(_ctx, &cmd, 1);
    // Pull CS high
    end_transmission(_ctx);

    return status;
}

/** @brief  Reads an amount of data from the QSPI memory in STR mode.
 *         SPI/DPI/QPI; 1-1-1/1-1-2/1-2-2/1-1-4/1-4-4
 * @param  _ctx Flash handle
 * @param  _mode Flash mode
 * @param  _data Pointer to data to be read
 * @param  _read_addr Read start address
 * @param  _size _size of data to read
 * @retval QSPI memory status
 */
bool Flash_S25FLXXXL_read_str(FLASH_S25FL_t* _ctx, S25FLXXXL_Interface_t _mode, uint8_t* _data, uint32_t _read_addr,
                              uint16_t _size)
{
    uint8_t cmd[5];
    uint8_t cmd_option   = 0;
    uint8_t dummy_cycles = 0;
    bool    status       = false;

    // Configure the read command based on the mode
    switch (_mode)
    {
    case S25FLXXXL_SPI_1I2O_MODE:
        cmd_option   = S25FLXXXL_CMD_4DOR;
        dummy_cycles = S25FLXXXL_DUMMY_CYCLES_READ;
        break;

    case S25FLXXXL_SPI_2IO_MODE:
        cmd_option   = S25FLXXXL_CMD_QIOR;
        dummy_cycles = S25FLXXXL_DUMMY_CYCLES_READ_DUAL_INOUT;
        break;

    case S25FLXXXL_SPI_1I4O_MODE:
        cmd_option   = S25FLXXXL_CMD_4QOR;
        dummy_cycles = S25FLXXXL_DUMMY_CYCLES_READ;
        break;

    case S25FLXXXL_SPI_4IO_MODE:
        cmd_option   = S25FLXXXL_CMD_4QIOR;
        dummy_cycles = S25FLXXXL_DUMMY_CYCLES_READ_QUAD_INOUT;
        break;

    case S25FLXXXL_SPI_MODE:
    default:
        // cmd_option       = S25FLXXXL_CMD_4FAST_READ;
        cmd_option   = S25FLXXXL_CMD_4FAST_READ;
        dummy_cycles = S25FLXXXL_DUMMY_CYCLES_READ;
        break;
    }

    // Populate the address bytes (32-bit address)

    if (command_cb == NULL)
    {
        return false;
    }
    const uint8_t cmd_length = command_cb(_ctx, cmd, cmd_option, _read_addr);

    // Pull CS low
    start_transmission(_ctx);

    // Send the command and address
    if (!flash_write(_ctx, cmd, cmd_length))
    {
        end_transmission(_ctx);
        return status;
    }

    // Send dummy bytes for dummy cycles (if any)
    if (dummy_cycles > 0)
    {
        uint8_t dummy = 0xFF;
        for (uint8_t i = 0; i < dummy_cycles; i++)
        {
            if (!flash_write(_ctx, &dummy, 1))
            {
                end_transmission(_ctx);
                return status;
            }
        }
    }

    // uint8_t dummy[dummy_cycles];
    // if (!flash_write(_ctx, dummy, dummy_cycles))
    // {
    //     end_transmission(_ctx);
    //     return status;
    // }

    // Receive the data
    status = flash_read(_ctx, _data, _size);

    // Pull CS high
    end_transmission(_ctx);

    return status;
}

/**
 * @brief  Erases the specified block of the QSPI memory.
 *         S25FLXXXL support 4K, 64K size block erase commands.
 *         SPI; 1-0-0/1-1-0
 * @param  _ctx Flash handle
 * @param  _mode Flash mode
 * @param  _block_address Block address to erase
 * @param  _block_size 4K or 64K
 * @retval QSPI memory status
 */
bool Flash_S25FLXXXL_block_erase1(FLASH_S25FL_t* _ctx, uint32_t _block_address, S25FLXXXL_Erase_t _block_size)
{
    uint8_t cmd[5];
    uint8_t cmd_option = 0;
    bool    status     = false;

    switch (_block_size)
    {
    case S25FLXXXL_ERASE_64K:
        cmd_option = S25FLXXXL_CMD_4BE;
        break;
    case S25FLXXXL_ERASE_4K:
    default:
        cmd_option = S25FLXXXL_CMD_4SE;
        break;
    }

    if (command_cb == NULL)
    {
        return false;
    }
    const uint8_t cmd_length = command_cb(_ctx, cmd, cmd_option, _block_address);

    start_transmission(_ctx); // Assert CS

    if (flash_write(_ctx, cmd, cmd_length) != HAL_OK)
    {
        status = true;
    }

    end_transmission(_ctx); // Deassert CS

    return status;
}

/**
 * @brief  Reads an amount of data from the QSPI memory on STR mode.
 *         SPI/DPI/QPI; 1-1-1/1-1-2/1-2-2/1-1-4/1-4-4
 * @param  _ctx Flash handle
 * @param  _mode Flash mode
 * @retval QSPI memory status
 */
bool Flash_S25FLXXXL_enable_memory_mapped_mode_str(FLASH_S25FL_t* _ctx, S25FLXXXL_Interface_t _mode, uint32_t _address,
                                                   uint8_t* _buffer, uint32_t _length)
{
    uint8_t cmd[6];
    uint8_t cmd_option     = 0;
    uint8_t dummy_bytes[4] = {0xFF, 0xFF, 0xFF, 0xFF}; // Adjust dummy bytes based on _mode
    bool    status         = false;

    // Prepare command based on _mode
    switch (_mode)
    {
    case S25FLXXXL_SPI_1I2O_MODE: // 1-1-2 read commands
        cmd_option = S25FLXXXL_CMD_4DOR;
        break;

    case S25FLXXXL_SPI_2IO_MODE: // 1-2-2 read commands
        cmd_option = S25FLXXXL_CMD_QIOR;
        break;

    case S25FLXXXL_SPI_1I4O_MODE: // 1-1-4 read commands
        cmd_option = S25FLXXXL_CMD_4QOR;
        break;

    case S25FLXXXL_SPI_4IO_MODE: // 1-4-4 read commands
        cmd_option = S25FLXXXL_CMD_4QIOR;
        break;

    case S25FLXXXL_SPI_MODE: // 1-1-1 default mode
    default:
        cmd_option = S25FLXXXL_CMD_4FAST_READ;
        break;
    }

    if (command_cb == NULL)
    {
        return false;
    }
    const uint8_t cmd_length = command_cb(_ctx, cmd, cmd_option, _address);

    // Perform SPI transaction
    start_transmission(_ctx);

    // Send command and address
    if (!flash_write(_ctx, cmd, cmd_length) != HAL_OK)
    {
        end_transmission(_ctx); // Deassert CS
        return status;
    }

    // Send dummy cycles (if needed based on _mode)
    if (_mode != S25FLXXXL_SPI_MODE)
    { // Default SPI _mode has no dummy cycles
        if (!flash_write(_ctx, dummy_bytes, S25FLXXXL_DUMMY_CYCLES_READ) != HAL_OK)
        {
            end_transmission(_ctx); // Deassert CS
            return status;
        }
    }

    // Receive data
    status = flash_read(_ctx, _buffer, _length);

    end_transmission(_ctx); // Deassert CS

    return status;
}

/**
 * @brief  Flash reset enable command
 *         SPI; 1-0-0
 * @param  _ctx Flash handle
 * @param  _mode Flash mode
 * @retval QSPI memory status
 */
bool Flash_S25FLXXXL_reset_enable(FLASH_S25FL_t* _ctx)
{
    uint8_t cmd    = S25FLXXXL_CMD_RSTEN; // Replace with actual reset enable command
    bool    status = false;

    // Assert chip select (CS) to start communication
    start_transmission(_ctx);

    // Transmit the command
    status = flash_write(_ctx, &cmd, 1);

    // Deassert chip select (CS) to end communication
    end_transmission(_ctx);

    return status;
}

/**
 * @brief  Flash reset memory command
 *         SPI; 1-0-0
 * @param  _ctx Flash handle
 * @param  _mode Flash mode
 * @retval QSPI memory status
 */
bool Flash_S25FLXXXL_reset_memory(FLASH_S25FL_t* _ctx)
{
    uint8_t cmd    = S25FLXXXL_CMD_RSTEN;
    bool    status = false;

    // Assert chip select (CS) to start communication
    start_transmission(_ctx);

    // Transmit the command
    if (!flash_write(_ctx, &cmd, 1) != HAL_OK)
    {
        end_transmission(_ctx); // Deassert CS on error
        return status;
    }

    // Deassert chip select (CS) to end communication
    end_transmission(_ctx);

    HAL_Delay(100); // Small delay between commands
    // delay(100);

    cmd = S25FLXXXL_CMD_RST;

    // Assert chip select (CS) to start communication
    start_transmission(_ctx);

    // Transmit the command
    status = flash_write(_ctx, &cmd, 1);

    // Deassert chip select (CS) to end communication
    end_transmission(_ctx);

    return status;
}

/**
 * @brief  Read Flash 3 Byte IDs.
 *         Manufacturer ID, Memory type, Memory density
 *         SPI; 1-0-1
 * @param  _ctx Flash handle
 * @param  _mode Flash mode
 * @param  ID  Flash ID
 * @retval QSPI memory status
 */
bool Flash_S25FLXXXL_read_id(FLASH_S25FL_t* _ctx, uint8_t* _id)
{
    uint8_t cmd    = S25FLXXXL_CMD_RDID; // Replace with actual read ID command
    bool    status = false;

    // Assert chip select (CS) to start communication
    start_transmission(_ctx);

    // Transmit the command
    if (!flash_write(_ctx, &cmd, 1) != HAL_OK)
    {
        end_transmission(_ctx); // Deassert CS on error
        return status;
    }

    // Receive the 3-byte ID
    status = flash_read(_ctx, _id, 3);

    // Deassert chip select (CS) to end communication
    end_transmission(_ctx);

    return status;
}

/**
 * @brief  Read Flash 3 Byte IDs.
 *         Manufacturer ID, Memory type, Memory density
 *         SPI; 1-0-1
 * @param  _ctx Flash handle
 * @param  _mode Flash mode
 * @param  ID  Flash ID
 * @retval QSPI memory status
 */
bool Flash_S25FLXXXL_read_id1(FLASH_S25FL_t* _ctx, uint8_t _id[3])
{
    uint8_t cmd    = S25FLXXXL_CMD_RDID; // Replace with actual read ID command
    bool    status = false;

    // Assert chip select (CS) to start communication
    start_transmission(_ctx);

    // Transmit the command
    if (!flash_write(_ctx, &cmd, 1) != HAL_OK)
    {
        end_transmission(_ctx); // Deassert CS on error
        return status;
    }

    // Receive the 3-byte ID
    status = flash_read(_ctx, _id, 3);

    // Deassert chip select (CS) to end communication
    end_transmission(_ctx);

    return status;
}

/**
 * @brief  Program/Erases suspend. Interruption Program/Erase operations.
 *         After the device has entered Erase-Suspended mode,
 *         system can read any address except the block/sector being Program/Erased.
 *         SPI; 1-0-0
 * @param  _ctx Flash handle
 * @param  _mode Flash moder
 * @retval QSPI memory status
 */
bool Flash_S25FLXXXL_program_erase_suspend(FLASH_S25FL_t* _ctx)
{
    uint8_t cmd    = S25FLXXXL_CMD_EPS; // Replace with actual suspend command
    bool    status = false;

    // Assert chip select (CS) to start communication
    start_transmission(_ctx);

    // Transmit the command
    status = flash_write(_ctx, &cmd, 1);

    // Deassert chip select (CS) to end communication
    end_transmission(_ctx);

    return status;
}

/**
 * @brief  Program/Erases resume.
 *         SPI; 1-0-0
 * @param  _ctx Flash handle
 * @param  _mode Flash mode
 * @retval QSPI memory status
 */
bool Flash_S25FLXXXL_program_erase_resume(FLASH_S25FL_t* _ctx)
{
    uint8_t cmd    = S25FLXXXL_CMD_EPR; // Replace with actual resume command
    bool    status = false;

    // Assert chip select (CS) to start communication
    start_transmission(_ctx);

    // Transmit the command
    status = flash_write(_ctx, &cmd, 1);

    // Deassert chip select (CS) to end communication
    end_transmission(_ctx);

    return status;
}

/**
 * @brief  Enter deep sleep
 * @param  _ctx Flash handle
 * @param  _mode Flash mode
 * @retval QSPI memory status
 */
bool Flash_S25FLXXXL_enter_deep_power_down(FLASH_S25FL_t* _ctx, S25FLXXXL_Interface_t _mode)
{
    UNUSED(_ctx);
    UNUSED(_mode);

    /* No Deep Power Down command for this memory */
    return false;
}

/**
 * @brief  This function reads the ID of the QSPI Memory and fills the info struct
 * @param  pqspi_info: pointer to the Info Typedef strcture
 *
 *          Manufacturer ID, Memory type, Memory density
 *          SPI/OPI; 1-0-1/1-0-8
 */
int QSPI_ReadID1(FLASH_S25FL_t* _ctx)
{

    XSPI_RegularCmdTypeDef s_command = {0};
    uint8_t                reg[21]   = {0};
    HAL_StatusTypeDef      status;

    /* Configure the command */
    s_command.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
    s_command.InstructionMode    = HAL_XSPI_INSTRUCTION_1_LINE;
    s_command.Instruction        = S25FLXXXL_CMD_RSFDP; // 0x5A
    s_command.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
    s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;

    // Address phase
    s_command.AddressMode  = HAL_XSPI_ADDRESS_1_LINE; // 1-line address phase
    s_command.Address      = 0x000000;                // Start of SFDP table
    s_command.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;

    // Data phase
    s_command.DataMode           = HAL_XSPI_DATA_1_LINE;  // 1-line data phase
    s_command.DataLength         = 10U;                   // Read 5 bytes
    s_command.DummyCycles        = CONF_QSPI_DUMMY_CLOCK; // 8 dummy cycles as per datasheet
    s_command.DQSMode            = HAL_XSPI_DQS_DISABLE;
    s_command.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
    s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;

    if ((status = HAL_XSPI_Command(_ctx->xspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK)
    {
        return S25FLXXXL_ERROR;
    }

    /* Reception of the data */
    if ((status = HAL_XSPI_Receive(_ctx->xspi, reg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK)
    {
        printf("Errror reading XSPI %lu\r\n", HAL_XSPI_GetError(_ctx->xspi));
        return S25FLXXXL_ERROR;
    }

    /* Debug: Check the received data */
    printf("SFDP: ");
    for (int i = 0; i < 4; i++)
    {
        printf("%c ", reg[i]);
    }
    printf("\r\n");

    return S25FLXXXL_OK;
}

/**
 * @brief  This function reads the ID of the QSPI Memory and fills the info struct
 * @param  pqspi_info: pointer to the Info Typedef strcture
 *
 *          Manufacturer ID, Memory type, Memory density
 *          SPI/OPI; 1-0-1/1-0-8
 */
int QSPI_ReadID2(FLASH_S25FL_t* _ctx)
{
    uint8_t                reg[100]  = {0xFF};
    XSPI_RegularCmdTypeDef s_command = {0};
    HAL_StatusTypeDef      status;

    /* Configure the command */
    s_command.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
    s_command.InstructionMode    = HAL_XSPI_INSTRUCTION_1_LINE;
    s_command.Instruction        = S25FLXXXL_CMD_RSFDP; // 0x5A
    s_command.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
    s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;

    // Address phase
    s_command.AddressMode  = HAL_XSPI_ADDRESS_1_LINE; // 1-line address phase
    s_command.Address      = 0x000000;                // Start of SFDP table
    s_command.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;

    // Data phase
    s_command.DataMode = HAL_XSPI_DATA_1_LINE; // 1-line data phase
    // s_command.DataMode           = HAL_XSPI_DATA_NONE;    // 1-line data phase
    s_command.DataLength = 32U; // Read 32 bytes
    // s_command.DataLength         = 40U;                   // Read 32 bytes
    s_command.DummyCycles        = CONF_QSPI_DUMMY_CLOCK; // 8 dummy cycles as per datasheet
    s_command.DQSMode            = HAL_XSPI_DQS_DISABLE;
    s_command.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
    s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;

    // Send command
    cmd_complete = 0; // Reset flag
    if ((status = HAL_XSPI_Command(_ctx->xspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE)) != HAL_OK)
    // if ((status = HAL_XSPI_Command_IT(_ctx->xspi, &s_command)) != HAL_OK)
    {
        LOG_ERROR("HAL_XSPI_Command_IT() error_code=%lx\r\n", HAL_XSPI_GetError(_ctx->xspi));
        return S25FLXXXL_ERROR;
    }

    // Wait for command to complete
    // while (!cmd_complete)
    //     ;

    // Receive data
    data_received = 0; // Reset flag
    /* Reception of the data */
    if ((status = HAL_XSPI_Receive_DMA(_ctx->xspi, reg)) != HAL_OK)
    {
        LOG_ERROR("HAL_XSPI_Receive_DMA() error_code=%lx\r\n", HAL_XSPI_GetError(_ctx->xspi));
        return S25FLXXXL_ERROR;
    }
    // Wait for data reception to complete
    while (!data_received)
        ;

    /* Debug: Check the received data */
    printf("SFDP: ");
    for (int i = 0; i < (int)(sizeof(reg)); i++)
    {
        printf("0x%02x ", reg[i]);
    }
    printf("\r\n");

    return S25FLXXXL_OK;
}

#define CHUNK_SIZE 64U // Chunk size for each transfer

/**
 * @brief  This function reads the ID of the QSPI Memory and fills the info struct
 * @param  pqspi_info: pointer to the Info Typedef strcture
 *
 *          Manufacturer ID, Memory type, Memory density
 *          SPI/OPI; 1-0-1/1-0-8
 */
int QSPI_ReadID(FLASH_S25FL_t* _ctx)
{

    uint8_t  buffer[64] = {0xFF};         // Example buffer
    uint32_t totalBytes = sizeof(buffer); // Total bytes to transfer
    uint32_t offset     = 0;              // Offset for each chunk

    while (totalBytes > 0)
    {
        uint32_t chunkLength = (totalBytes >= CHUNK_SIZE) ? CHUNK_SIZE : totalBytes;
        chunkLength -= (chunkLength % 4); // Ensure multiple of 4

        XSPI_RegularCmdTypeDef s_command = {0};
        s_command.OperationType          = HAL_XSPI_OPTYPE_COMMON_CFG;
        s_command.InstructionMode        = HAL_XSPI_INSTRUCTION_1_LINE;
        s_command.Instruction            = S25FLXXXL_CMD_RSFDP; // Adjust with your read command
        s_command.AddressMode            = HAL_XSPI_ADDRESS_1_LINE;
        s_command.Address                = 0 + offset;
        s_command.AddressWidth           = HAL_XSPI_ADDRESS_24_BITS;
        s_command.DataMode               = HAL_XSPI_DATA_1_LINE;
        s_command.DataLength             = chunkLength;
        s_command.DummyCycles            = CONF_QSPI_DUMMY_CLOCK;
        s_command.DQSMode                = HAL_XSPI_DQS_DISABLE;

        if (HAL_XSPI_Command(_ctx->xspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        {
            printf("Error in command transmission.\n");
            break;
        }

        if (HAL_XSPI_Receive_DMA(_ctx->xspi, &buffer[offset]) != HAL_OK)
        // if (HAL_XSPI_Receive_IT(_ctx->xspi, &buffer[offset]) != HAL_OK)
        {
            printf("Error in data reception.\n");
            break;
        }

        while (!data_received)
            ; // Wait for the DMA transfer to complete

        // HAL_XSPI_Abort_IT(_ctx->xspi);

        offset += chunkLength;
        totalBytes -= chunkLength;
    }

    /* Debug: Check the received data */
    printf("SFDP: ");
    for (int i = 0; i < (int)(sizeof(buffer)); i++)
    {
        printf("0x%02x ", buffer[i]);
    }
    printf("\r\n");

    return S25FLXXXL_OK;
}

int QSPI_Read(FLASH_S25FL_t* _ctx)
{

    uint8_t  buffer[256] = {0xFF};         // Example buffer
    uint32_t totalBytes  = sizeof(buffer); // Total bytes to transfer
    uint32_t offset      = 0;              // Offset for each chunk

    while (totalBytes > 0)
    {
        uint32_t chunkLength = (totalBytes >= CHUNK_SIZE) ? CHUNK_SIZE : totalBytes;
        chunkLength -= (chunkLength % 4); // Ensure multiple of 4

        XSPI_RegularCmdTypeDef s_command = {0};
        s_command.OperationType          = HAL_XSPI_OPTYPE_COMMON_CFG;
        s_command.InstructionMode        = HAL_XSPI_INSTRUCTION_1_LINE;
        s_command.Instruction            = S25FLXXXL_CMD_FAST_READ; // Adjust with your read command
        s_command.AddressMode            = HAL_XSPI_ADDRESS_1_LINE;
        s_command.Address                = 0 + offset;
        s_command.AddressWidth           = HAL_XSPI_ADDRESS_24_BITS;
        s_command.DataMode               = HAL_XSPI_DATA_1_LINE;
        s_command.DataLength             = chunkLength;
        s_command.DummyCycles            = 0;
        s_command.DQSMode                = HAL_XSPI_DQS_DISABLE;

        if (HAL_XSPI_Command(_ctx->xspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        {
            printf("Error in command transmission.\n");
            break;
        }

        if (HAL_XSPI_Receive_DMA(_ctx->xspi, &buffer[offset]) != HAL_OK)
        {
            printf("Error in data reception.\n");
            break;
        }

        while (!data_received)
            ; // Wait for the DMA transfer to complete

        offset += chunkLength;
        totalBytes -= chunkLength;
    }

    /* Debug: Check the received data */
    LOG_INFO("FLASH_READ: ");
    for (int i = 0; i < (int)(sizeof(buffer)); i++)
    {
        LOG_INFO("0x%02x ", buffer[i]);
    }
    LOG_INFON();

    return S25FLXXXL_OK;
}

/**
 * @brief  Command completed callback.
 * @param  hospi: OSPI handle
 * @retval None
 */
void HAL_XSPI_CmdCpltCallback(XSPI_HandleTypeDef* hxspi)
{
    // Set the state to CMD_CFG for Receive phase
    hxspi->State = HAL_XSPI_STATE_CMD_CFG;

    cmd_complete = 1; // Set flag when command phase completes
}

/**
 * @brief  Rx Transfer completed callback.
 * @param  hospi: OSPI handle
 * @retval None
 */
void HAL_XSPI_RxCpltCallback(XSPI_HandleTypeDef* hxspi)
{
    data_received = 1; // Set flag when data reception completes
}

/**
 * @brief  Tx Transfer completed callback.
 * @param  hospi: OSPI handle
 * @retval None
 */
void HAL_XSPI_TxCpltCallback(XSPI_HandleTypeDef* hospi) { TxCplt++; }

/**
 * @brief  Transfer Error callback.
 * @param  hospi: OSPI handle
 * @retval None
 */
void HAL_XSPI_ErrorCallback(XSPI_HandleTypeDef* hospi) { Error_Handler(); }

/**
 * @}
 */

/**
 * @}
 */