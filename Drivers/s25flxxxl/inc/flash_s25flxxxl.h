/**
 * @file    Dev_Inf.c
 * @author  Thierry Zinkeng (tz@dc.systems)
 * @brief   This file contains all the description of the S25FLXXXL SPI memory.
 * @version  1.0.0
 * @date    2024-11-21
 * 
 * @copyright Copyright DC Systems BV (c) 2024
 * 
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef S25FLXXXL_H
#define S25FLXXXL_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "flash_s25flxxxl_config.h"
#include <stdbool.h>
#include <stdint.h>

/** @addtogroup BSP
 * @{
 */

/** @addtogroup Components
 * @{
 */

// Device Identification
#define MANUFACTURER_ID_INFINEON 0x01 // Manufacturer ID for Cypress (Infineon)
/* S25FLXXXLAGNFI010 Flash Memory Commands */

#define BASE_ADDR_MASK    0xC0000000

/**
 * @defgroup S25FLXXXL_Definitions S25FLXXXL Flash Definitions
 * @brief Definitions for S25FLXXXL flash memory.
 * @{
 */

/** 
 * @defgroup S25FLXXXL_ErrorCodes Error Codes
 * @brief Error codes returned by S25FLXXXL functions.
 * @{
 */
#define S25FLXXXL_OK                (0)  /**< Operation succeeded. */
#define S25FLXXXL_ERROR             (-1) /**< Generic error occurred. */
/** @} */

/** 
 * @defgroup S25FLXXXL_Commands Commands
 * @brief Flash memory command codes for S25FLXXXL.
 * @{
 */

/** @defgroup Identification_Commands Identification Commands 
 * @brief Commands used to identify the flash memory device.
 * @{
 */
#define S25FLXXXL_CMD_RDID  0x9F /**< Read ID (JEDEC Manufacturer ID). */
#define S25FLXXXL_CMD_RSFDP 0x5A /**< Read Serial Flash Discoverable Parameters. */
#define S25FLXXXL_CMD_RDQID 0xAF /**< Read Quad ID. */
#define S25FLXXXL_CMD_RUID  0x4B /**< Read Unique ID. */
/** @} */

// Read Commands
#define S25FLXXXL_CMD_READ       0x03 // Read Data
#define S25FLXXXL_CMD_4READ      0x13 // Read Data
#define S25FLXXXL_CMD_FAST_READ  0x0B // Fast Read
#define S25FLXXXL_CMD_4FAST_READ 0x0C // Fast Read
#define S25FLXXXL_CMD_DOR        0x3B // Dual Output Read
#define S25FLXXXL_CMD_4DOR       0x3C // Dual Output Read
#define S25FLXXXL_CMD_QOR        0x6B // Quad Output Read
#define S25FLXXXL_CMD_4QOR       0x6C // Quad Output Read
#define S25FLXXXL_CMD_DIOR       0xBB // Dual I/O Read
#define S25FLXXXL_CMD_4DIOR      0xBC // Dual I/O Read
#define S25FLXXXL_CMD_QIOR       0xEB // Quad I/O Read (CR1V[1] = 1) or CR2V[3] = 1
#define S25FLXXXL_CMD_4QIOR      0xEC // Quad I/O Read (CR1V[1] = 1) or CR2V[3] = 1
#define S25FLXXXL_CMD_DDRQIOR    0xED // DDR Quad I/O Read (CR1V[1] = 1 or CR2V[3] = 1)
#define S25FLXXXL_CMD_4DDRQIOR   0xEE // DDR Quad I/O Read (CR1V[1] = 1 or CR2V[3] = 1)
#define S25FLXXXL_CMD_RDAR       0x65 // Read Any Register

// Program Commands
#define S25FLXXXL_CMD_WREN 0x06 // Write Enable
#define S25FLXXXL_CMD_WRDI 0x04 // Write Disable
#define S25FLXXXL_CMD_WRAR 0x71 // Write Any Register
#define S25FLXXXL_CMD_PP   0x02 // Page Program
#define S25FLXXXL_CMD_4PP  0x12 // Page Program (4 Byte Address)
#define S25FLXXXL_CMD_QPP  0x32 // Quad Page Program
#define S25FLXXXL_CMD_4QPP 0x34 // Quad Page Program

/// Erase Commands
#define S25FLXXXL_CMD_SE   0x20 // Sector Erase (4 KB)
#define S25FLXXXL_CMD_4SE  0x21 // Sector Erase (4 KB)
#define S25FLXXXL_CMD_HBE  0x52 // Half Block Erase (32 KB)
#define S25FLXXXL_CMD_4HBE 0x53 // Half Block Erase (32 KB)
#define S25FLXXXL_CMD_BE   0xD8 // Block Erase (64 KB)
#define S25FLXXXL_CMD_4BE  0xDC // Block Erase (64 KB)
#define S25FLXXXL_CMD_CE   0x60 // Chip Erase (3-byte address)
#define S25FLXXXL_CMD_4CE  0xC7 // Chip Erase

// Erase/Program Suspend/Resume
#define S25FLXXXL_CMD_EPS 0x75 // Erase / Program Suspend
#define S25FLXXXL_CMD_EPR 0x7A // Erase / Program Resume

// Status/Configuration Register Commands
#define S25FLXXXL_CMD_RDSR1 0x05 // Read Status Register 1
#define S25FLXXXL_CMD_RDSR2 0x35 // Read Status Register 2
#define S25FLXXXL_CMD_RDCR  0x15 // Read Configuration Register
#define S25FLXXXL_CMD_WRSR  0x01 // Write Status and Configuration Register

// Security and Protection Commands
#define S25FLXXXL_CMD_RDSCUR    0x2B // Read Security Register
#define S25FLXXXL_CMD_WRSCUR    0x2F // Write Security Register
#define S25FLXXXL_CMD_RDBLOCK   0x3C // Read Block Protection
#define S25FLXXXL_CMD_WDBLOCK   0x3D // Write Block Protection
#define S25FLXXXL_CMD_WRPROTSEL 0x68 // Write Protection Selection

// Reset Commands
#define S25FLXXXL_CMD_RSTEN 0x66 // Software Reset Enable
#define S25FLXXXL_CMD_RST   0x99 // Software Reset
#define S25FLXXXL_CMD_MBR   0xFF // Mode Bit Reset

// Power Management Commands
#define S25FLXXXL_CMD_DP  0xB9 // Deep Power-Down
#define S25FLXXXL_CMD_RDI 0xAB // Release Deep Power-Down / Device ID

// One-Time Programmable Commands
#define S25FLXXXL_CMD_OTPR 0x4B // Read OTP Array
#define S25FLXXXL_CMD_OTPP 0x42 // Program OTP Array

// Status Register Bits
#define S25FLXXXL_REG_SR1_WIP 0x01 // Write-In-Progress
#define S25FLXXXL_REG_SR1_WEL 0x02 // Write Enable Latch
#define S25FLXXXL_REG_SR1_BP0 0x04 // Block Protect Bit 0
#define S25FLXXXL_REG_SR1_BP1 0x08 // Block Protect Bit 1
#define S25FLXXXL_REG_SR1_BP2 0x10 // Block Protect Bit 2
#define S25FLXXXL_REG_SR1_QE  0x40 // Quad Enable

#define S25FLXXXL_REG_RDSR1  0x05 // Read Status Register 1
#define S25FLXXXL_REG_RDSR2  0x07 // Read Status Register 2
#define S25FLXXXL_REG_RDCR1  0x35 // Read Configuration Register 1
#define S25FLXXXL_REG_WRR    0x01 // Write Register (Status-1 and Configuration-1,2,3)
#define S25FLXXXL_REG_WRDI   0x04 // Write Disable
#define S25FLXXXL_REG_WREN   0x06 // Write Enable for Non-volatile data change
#define S25FLXXXL_REG_CLSR   0x30 // Clear Status Register
#define S25FLXXXL_CMD_4BEN   0xB7 // Enter 4 Byte Address Mode
#define S25FLXXXL_CMD_4BEX   0xE9 // Exit 4 Byte Address Mode
#define S25FLXXXL_REG_DLPRD  0x41 // Data Learning Pattern Read
#define S25FLXXXL_REG_PDLRNV 0x43 // Program NV Data Learning Register
#define S25FLXXXL_REG_WDLRV  0x4A // Write Volatile Data Learning Register

#define S25FL256L_WIP_BIT (1 << 0) /* WIP bit is in position 0 */

// Configuration Register Bits
#define S25FLXXXL_REG_CR_TB       0x08 // Top/Bottom Protect
#define S25FLXXXL_REG_CR_ODS_MASK 0x07 // Output Driver Strength Mask

// Other Macros
/**
 *  @defgroup S25FLXXXL_Exported_Functions
* @{ 
* @brief Flash memory capacity (256 Mbits = 32 MBytes) 
 *1.  Total flash memory size:
 *    From the datasheet, the S25FL256L flash memory has 256 megabits of storage capacity.
 *    256Mb = (256 / 8) MB = 32 MB = 33,554,432  bytes
 *2. Page size: 
 *  According to the datasheet, the page size for this flash memory is 256 bytes.
 * Calculating the number of pages:
 * Number of pages = (Total size in bytes / Page size in bytes) = (33,554,432 / 256) = 131,072 pages
 */
// #define S25FL256L_FLASH_SIZE     (uint32_t)(256 * 1024 * 1024 / 8) /* 256 Mbits to bytes (32 MB) */
// #define S25FL256L_PAGE_SIZE      256                               // Page size in bytes
// #define S25FL256L_SECTOR_SIZE    4096                              // Sector size in bytes (4 KB)
// #define S25FL256L_BLOCK_32K_SIZE 32768                             // Block size in bytes (32 KB)
// #define S25FL256L_BLOCK_64K_SIZE 65536                             // Block size in bytes (64 KB)

// #define EXT_FLASH_PAGE_SIZE   0x0100     // 256 bytes per page (S25FL256L specification)
// #define EXT_FLASH_SECTOR_SIZE 0x1000     // 4 KB per sector (S25FL256L specification)
// #define EXT_FLASH_BLOCK_SIZE  0x010000   // 64 KB per block (S25FL256L specification)
// #define EXT_FLASH_SIZE        0x01000000 // 256 Mbits = 32 MB total size (S25FL256L specification)
// #define EXT_FLASH_PAGE_NUM    (S25FL256L_FLASH_SIZE / S25FL256L_PAGE_SIZE) //  131,072 pages
// #define EXT_FLASH_SECTOR_NUM  0x2000     // 8192 sectors (32 MB / 4 KB per sector)
// #define EXT_FLASH_BLOCK_NUM   0x800      // 512 blocks (32 MB / 64 KB per block)

// #define S25FL256L_FLASH_SIZE     (uint32_t)(256 * 1024 * 1024) // Flash Size = 32 MB (256 Mbits)
#define S25FL256L_FLASH_SIZE    (uint32_t)(32 * 1024 * 1024)                   // Flash Size in MB = 32 MB (256 Mbits)
#define S25FL256L_PAGE_SIZE     0x0100                                         // Page Size = 256 bytes
#define S25FL256L_SECTOR_SIZE   0x1000                                         // Sector Size = 4 KB (4096 bytes)
#define S25FL256L_BLOCK_SIZE    0x010000                                       // Block Size = 64 KB
#define S25FL256L_NUM_SECTORS   (S25FL256L_FLASH_SIZE / S25FL256L_SECTOR_SIZE) // 8192 sectors
#define S25FL256L_NUM_PAGES     (S25FL256L_FLASH_SIZE / S25FL256L_PAGE_SIZE) // 8192 sectors

#define EXT_FLASH_SIZE        S25FL256L_FLASH_SIZE
#define EXT_FLASH_PAGE_SIZE   S25FL256L_PAGE_SIZE
#define EXT_FLASH_SECTOR_SIZE S25FL256L_SECTOR_SIZE
#define EXT_FLASH_SECTOR_NUM  S25FL256L_NUM_SECTORS
#define EXT_FLASH_PAGE_NUM    S25FL256L_NUM_PAGES
#define EXT_FLASH_BLOCK_SIZE  S25FL256L_BLOCK_SIZE

/**
* @}
*/
// Timing and Parameters (example values, check datasheet for actual numbers)
#define S25FL256L_WRITE_CYCLE_TIME_MS  1  // Typical write cycle time (in ms)
#define S25FL256L_ERASE_SECTOR_TIME_MS 50 // Typical sector erase time (in ms)

#define S25FL256L_BLOCK_64K (uint32_t)(64 * 1024) /* 256 blocks of 64KBytes  */
#define S25FL256L_SECTOR_4K (uint32_t)(4 * 1024)  /* 4096 sectors of 4KBytes */

/* Memory organization definitions for S25FL256L */

/* Erase and write timing (maximum times in milliseconds) */
#define S25FL256L_BULK_ERASE_MAX_TIME   600000 /* Bulk erase max time: 600 seconds */
#define S25FL256L_BLOCK_ERASE_MAX_TIME  2000   /* Block erase (64 KB) max time: 2 seconds */
#define S25FL256L_SECTOR_ERASE_MAX_TIME 450    /* Sector erase (4 KB) max time: 450 ms */

/* Write operation timing */
#define S25FL256L_PAGE_PROGRAM_MAX_TIME 3 /* Page program max time: 3 ms */

/** @} */ // End of S25FLXXXL_Definitions

// Forward declaration of the structure
struct FLASH_S25FL_s;

typedef uint8_t (*flash_command_cb)(struct FLASH_S25FL_s* _ctx, uint8_t* _data, uint8_t _cmd, uint32_t _address);
typedef int (*flash_prepare_command_cb)(struct FLASH_S25FL_s* _ctx, const uint32_t _instruction, const uint32_t _address,
                                            const uint32_t _data_length, const uint32_t _dummy_cycles);
typedef void (*flash_start_transmission_cb)(struct FLASH_S25FL_s* _ctx);
typedef void (*flash_stop_transmission_cb)(struct FLASH_S25FL_s* _ctx); 
typedef int (*flash_write_cb)(struct FLASH_S25FL_s* _ctx, const uint8_t* _data, const uint16_t _size);
typedef int (*flash_read_cb)(struct FLASH_S25FL_s* _ctx, uint8_t* _data, const uint16_t _size);

/** @addtogroup Registers
 * @{
 */
typedef union {
    uint8_t value; // Access the entire register as a byte
    struct {
        uint8_t ADS:1; // Bit 0: Address Length Status
        uint8_t ADP:1; // Bit 1: Address Length at Power-up
        uint8_t WPS:1; // Bit 2: Write Protect Selection
        uint8_t QPI:1; // Bit 3: QPI
        uint8_t RFU:1; // Bit 4: Reserved
        uint8_t OI :1; // Bit 5: Output Impedance
        uint8_t IO3R:1; // Bit 6: IO3_Reset
        uint8_t _reserved:1; // Bit 7: Unused or Reserved
    } bits; // Access individual bitfields
} Reg_CR2V_t;

/**
 * @brief Status Register 1 Volatile (SR1V) S25FL256L
 */
typedef union {
    uint8_t value; // Access the entire register as a byte
    struct {
        uint8_t WIP   : 1; /// Bit 0: Write in Progress
        uint8_t WEL   : 1; /// Bit 1: Write Enable Latch
        uint8_t BP0   : 1; /// Bit 2: Block Protection 0
        uint8_t BP1   : 1; /// Bit 3: Block Protection 1
        uint8_t BP2   : 1; /// Bit 4: Block Protection 2
        uint8_t BP3   : 1; /// Bit 5: Block Protection 3
        uint8_t TBPROT: 1; /// Bit 6: Top/Bottom Relative Protection
        uint8_t SRP0  : 1; /// Bit 7: Status Register Protect 0
    } bits; // Access individual bitfields
} Reg_SR1V_t;
/**
* @}
*/

typedef struct
{
    uint32_t flash_size;            /*!< Size of the flash                         */
    uint32_t erase_sector_size;     /*!< Size of sectors for the erase operation   */
    uint32_t erase_sectors_number;  /*!< Number of sectors for the erase operation */
    uint32_t program_page_size;     /*!< Size of pages for the program operation   */
    uint32_t prog_pages_number;     /*!< Number of pages for the program operation */
} Flash_S25FL_info_t;

typedef enum
{
    S25FLXXXL_SPI_MODE = 0,  /*!< 1-1-1 commands, Power on H/W default setting */
    S25FLXXXL_SPI_1I2O_MODE, /*!< 1-1-2 read commands                          */
    S25FLXXXL_SPI_2IO_MODE,  /*!< 1-2-2 read commands                          */
    S25FLXXXL_SPI_1I4O_MODE, /*!< 1-1-4 read commands                          */
    S25FLXXXL_SPI_4IO_MODE,  /*!< 1-4-4 read commands                          */
    S25FLXXXL_DPI_MODE,      /*!< 2-2-2 commands                               */
    S25FLXXXL_QPI_MODE       /*!< 4-4-4 commands                               */
} S25FLXXXL_Interface_t;

typedef enum
{
    S25FLXXXL_STR_TRANSFER = 0, /*!< Single Transfer Rate */
} S25FLXXXL_Transfer_t;

typedef enum
{
    S25FLXXXL_DUALFLASH_DISABLE = 0, /*!< Single flash mode    */
} S25FLXXXL_DualFlash_t;

typedef enum
{
    S25FLXXXL_ERASE_4K = 0, /*!< 4K size Sector erase */
    S25FLXXXL_ERASE_64K,    /*!< 64K size Block erase */
    S25FLXXXL_ERASE_CHIP    /*!< Whole chip erase     */
} S25FLXXXL_Erase_t;

typedef enum
{
    S25FLXXXL_ADDR_MODE_24BIT = 0x00, /*!< 24-bit address operation */
    S25FLXXXL_ADDR_MODE_32BIT = 0x01,    /*!< 32-bit address operation */
} S25FLXXXL_address_mode_t;

typedef enum
{
    S25FLXXXL_BACKEND_SPI = 0x00, /*!< Flash connected to SPI */
    S25FLXXXL_BACKEND_XSPI = 0x01, /*!< Flash connected to OCTOSPI */
} S25FLXXXL_backend_t;

typedef struct FLASH_S25FL_GPIO_s
{
    GPIO_TypeDef* port;
    uint16_t      pin;
} FLASH_S25FL_GPIO_t;

typedef struct flash_backend
{
    flash_read_cb read_cb;
    flash_write_cb write_cb;

    XSPI_RegularCmdTypeDef xspi_cmd;
    flash_prepare_command_cb prep_cmd_cb;

    flash_command_cb cmd_cb;
    flash_start_transmission_cb start_tx_cb;
    flash_stop_transmission_cb stop_tx_cb;

} flash_backend_t;
/**
 * @brief Flash context
 * 
 */
typedef struct FLASH_S25FL_s
{
    bool initialized;
    SPI_HandleTypeDef*  spi;
    XSPI_HandleTypeDef*  xspi;
    Flash_S25FL_info_t info;
    FLASH_S25FL_GPIO_t  chp_select;
    S25FLXXXL_address_mode_t address_mode;
    flash_backend_t backend;
    flash_command_cb cmd_cb;
    flash_write_cb write_cb;
    flash_read_cb read_cb;
    flash_start_transmission_cb start_tx_cb;
    flash_stop_transmission_cb stop_tx_cb;
} FLASH_S25FL_t;
/**
 * @}
 */

/** 
 * @defgroup S25FLXXXL_Exported_Functions Functions
 * @brief Flash memory exported functions.
 * @{
 */

/**
 * @brief  Get Flash information
 * @param  pInfo pointer to information structure
 * @retval 0 if success
 */
int Flash_S25FLXXXL_get_flash_info(Flash_S25FL_info_t* pInfo);

/**
 * @brief Print out Flash chip info
 * 
 * @param _ctx Pointer to Flash context
 * 
 * @return int 0 if success, otherwise failed
 */
int Flash_S25FLXXXL_print_flash_info(FLASH_S25FL_t* _ctx);

/**
 * @brief Get if flash chip is ready
 *        
 * @details Read SFDP (0x5A) Header 1st DWORD.
 *          Must return a string "SFDP"
 * 
 * @param _ctx Pointer to Flash context
 * 
 * @return true Available
 * @return false Not available
 */
bool Flash_S25FLXXXL_is_available(FLASH_S25FL_t* _ctx);

/**
 * @brief This function send a Write Enable and wait it is effective.
 * 
 * @param _ctx Pointer to Flash context
 * 
 * @return int 0 if success, otherwise failed 
 */
int Flash_S25FLXXXL_write_enable(FLASH_S25FL_t* _ctx);

/**
 * @brief This function reset the (WEL) Write Enable Latch bit.
 * 
 * @param _ctx Pointer to Flash context
 * 
 * @return int 0 if success, otherwise failed 
 */
int Flash_S25FLXXXL_write_disable(FLASH_S25FL_t* _ctx);

/**
 * @brief Get sector number for given start address
 * 
 * @param start_address Start address
 * 
 * @return int Which sector, otherwise -1
 */
int Flash_S25FLXXXL_helper_sector_number_get(uint32_t start_address);

/**
 * @brief Initialize Flash drivers
 * 
 * @param _ctx Pointer to Flash context
 * 
 * @return int 0 if success, otherwise failed
 */
int Flash_S25FLXXXL_init(FLASH_S25FL_t* _ctx);


int Flash_S25FLXXXL_auto_polling(FLASH_S25FL_t* _ctx);

/**
 * @brief Erases the specified block of the Flash memory.
 *
 * This function erases a block of memory (64KB) on the S25FLXXXL Flash device.
 * It sets all bits in the specified block to `1` (all bytes become `0xFF`).
 * 
 * @note 
 * - Before the Block Erase (BE) command can be accepted, a Write Enable (WREN) command 
 *   must be issued to set the Write Enable Latch (WEL) in the Status Register.
 * - The erase operation must monitor the Write-In-Progress (WIP) bit to confirm its completion.
 * - The block address and size must be aligned to the Flash's memory layout.
 *
 * @details
 * The function executes the following sequence according to the datasheet:
 *  1. **Write Enable (WREN):** 
 *     - Send the WREN command to set the Write Enable Latch (WEL) bit in the Status Register.
 *     - @ref Flash_S25FLXXXL_write_enable().
 *
 *  2. **Verify Write Enable Latch (WEL):**
 *     - Read the Status Register 1 (RDSR1) and confirm the WEL bit is set.
 *
 *  3. **Send Block Erase (BE) Command:**
 *     - Issue the BE command (`D8h` or `DCh`) along with the block address.
 *     - The block size is determined by the command and the addressing mode (3-byte or 4-byte).
 *
 *  4. **Wait for Completion (WIP):**
 *     - Continuously poll the Write-In-Progress (WIP) bit in the Status Register.
 *     - Wait until the WIP bit is cleared, indicating the erase operation is complete.
 *
 *  5. **Write Disable (WRDI):**
 *     - Optionally, send the Write Disable (WRDI) command to clear the WEL bit.
 *     - @ref Flash_S25FLXXXL_write_disable().
 *
 * @param[in]  _ctx          Pointer to the Flash context structure.
 * @param[in]  _block_address The block's starting address to erase (aligned to block size).
 *
 * @return int 
 * - `0` on success.
 * - Non-zero value on failure.
 */
int Flash_S25FLXXXL_block_erase(FLASH_S25FL_t* _ctx, uint32_t _block_address);

/**
 * @brief 
 * 
 * @param _ctx 
 * @param _sector_address 
 * 
 * @return int 
 */
int Flash_S25FLXXXL_erase_sector(FLASH_S25FL_t* _ctx, uint32_t _sector_address);

int Flash_S25FLXXXL_chip_erase(FLASH_S25FL_t* _ctx);


int Flash_S25FLXXXL_read_data(FLASH_S25FL_t* _ctx, uint32_t _address, uint32_t _n_bytes, uint8_t* _p_data);

int Flash_S25FLXXXL_write_data(FLASH_S25FL_t* _ctx, uint32_t _address, const uint8_t * _p_data, uint32_t _n_bytes);


/** @} */ // End of S25FLXXXL_Exported_Functions

int Flash_S25FLXXXL_mass_erase(FLASH_S25FL_t* _ctx);

int Flash_S25FLXXXL_print_flash_info(FLASH_S25FL_t* _ctx);

bool     Flash_S25FLXXXL_init1(FLASH_S25FL_t* _ctx);

uint32_t Flash_S25FXXXL_read_jedec_id(FLASH_S25FL_t* _ctx);
bool     Flash_S25FXXXL_set_addr_mode(FLASH_S25FL_t* _ctx, S25FLXXXL_address_mode_t mode);
bool     Flash_S25FXXXL_get_addr_mode1(FLASH_S25FL_t* _ctx, uint8_t* _mode);
bool     Flash_S25FLXXXL_get_info(Flash_S25FL_info_t* _info);
bool     Flash_S25FLXXXL_read_id_ext(FLASH_S25FL_t* _ctx, uint8_t* _manufacturer_id, uint8_t* _device_id);


bool     Flash_S25FLXXXL_page_program(FLASH_S25FL_t* _ctx, uint8_t* _data, uint32_t _write_addr, uint16_t _size);
bool     Flash_S25FLXXXL_read(FLASH_S25FL_t* _ctx, uint8_t* _data, uint32_t _read_addr, uint16_t _size);
bool     Flash_S25FLXXXL_auto_polling_mem_ready(FLASH_S25FL_t* _ctx);
bool     Flash_S25FLXXXL_enter_4_bytes_address_mode(FLASH_S25FL_t* _ctx);
bool Flash_S25FLXXXL_read_str(FLASH_S25FL_t* _ctx, S25FLXXXL_Interface_t _mode, uint8_t* _data, uint32_t _read_addr,
                                uint16_t _size);
bool Flash_S25FLXXXL_block_erase1(FLASH_S25FL_t* _ctx, uint32_t _block_address, S25FLXXXL_Erase_t _block_size);

bool Flash_S25FLXXXL_enable_memory_mapped_mode_str(FLASH_S25FL_t* _ctx, S25FLXXXL_Interface_t _mode,
                                                    uint32_t _address, uint8_t* _buffer, uint32_t _length);
bool Flash_S25FLXXXL_reset_enable(FLASH_S25FL_t* _ctx);
bool Flash_S25FLXXXL_reset_memory(FLASH_S25FL_t* _ctx);
bool Flash_S25FLXXXL_read_id(FLASH_S25FL_t* _ctx, uint8_t* _id);
bool Flash_S25FLXXXL_read_id1(FLASH_S25FL_t* _ctx, uint8_t _id[3]);
bool Flash_S25FLXXXL_program_erase_suspend(FLASH_S25FL_t* _ctx);
bool Flash_S25FLXXXL_program_erase_resume(FLASH_S25FL_t* _ctx);
bool Flash_S25FLXXXL_enter_deep_power_down(FLASH_S25FL_t* _ctx, S25FLXXXL_Interface_t _mode);

#ifdef __cplusplus
}
#endif

#endif /* S25FLXXXL_H */
