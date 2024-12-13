/**
 * @file    Loader_src.c
 * @author  Thierry Zinkeng (tz@dc.systems)
 * @brief   This file provides the Loader implementations.
 * @version  1.0.0
 * @date    2024-11-21
 *
 * @copyright Copyright DC Systems BV (c) 2024
 *
 */

#include "Loader_Src.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

#if defined(EXT_LOADER_GENERATE) && EXT_LOADER_GENERATE
#define EXT_FLASH_ADDR_MASK BASE_ADDR_MASK

#define LOADER_OK   0x1
#define LOADER_FAIL 0x0

#ifdef IS_LED
#define LED_PIN_ON GPIO_PIN_RESET // this is the GPIO level turning on led
#endif                            // IS_LED

extern void SystemClock_Config(void);
extern XSPI_HandleTypeDef hospi1;

static FLASH_S25FL_t flash_ctx;

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) { return HAL_OK; }

uint32_t HAL_GetTick(void) { return 1; }

// RAM_FUNC
void LOC_SPI_Init()
{
    /* No clue why is needed */
    // __HAL_RCC_SPI2_FORCE_RESET();
    // __HAL_RCC_SPI2_RELEASE_RESET();

    // MX_SPI2_Init(); // !!! this line needs to be aligned to the SPI port used!  Check this function: it is defined in
    //                 // spi.c !!!

    __HAL_RCC_OSPI1_FORCE_RESET();
    __HAL_RCC_OSPI1_RELEASE_RESET();
    // MX_GPDMA1_Init();
    // MX_ICACHE_Init();
    MX_OCTOSPI1_Init();
}

// RAM_FUNC
void LOC_LedOn()
{
#ifdef IS_LED
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, LED_PIN_ON);
#endif // IS_LED
}

// RAM_FUNC
void LOC_LedOff()
{
#ifdef IS_LED
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, !LED_PIN_ON);
#endif // IS_LED
}

// RAM_FUNC
int Init(void)
{
    uint8_t res = LOADER_OK;

    *(uint32_t*)0xE000EDF0 = 0xA05F0000; // enable interrupts in debug

    SystemInit();

    /* ADAPTATION TO THE DEVICE
     *
     * change VTOR setting for H7 device
     * SCB->VTOR = 0x24000000 | 0x200;
     *
     * change VTOR setting for H5 devices
     * SCB->VTOR = 0x20003000 | 0x200;
     *
     * change VTOR setting for other devices
     * SCB->VTOR = 0x20000000 | 0x200;
     *
     * */
    SCB->VTOR = 0x20003000 | 0x200;

    __set_PRIMASK(0); // enable interrupts

    HAL_Init();

    SystemClock_Config();

    // HAL_XSPI_DeInit(&hospi1);

    MX_GPIO_Init();
    LOC_SPI_Init();
    // MX_UART7_Init();
    MX_USART3_UART_Init();

    //printf("[LOADER] %s(), Initialization completed\r\n", __func__);

    if (Flash_S25FLXXXL_init(&flash_ctx) != 0)
        res = LOADER_FAIL;

    printf("[LOADER] %s(), Flash_S25FLXXXL_init() result=0x%x\r\n", __func__, res);
    __set_PRIMASK(1); // disable interrupts

    // __enable_irq();
    return res;
}

/**
 * Description :
 * Read data from the device
 * Inputs    :
 *      Address       : Write location
 *      Size          : Length in bytes
 *      buffer        : Address where to get the data to write
 * outputs   :
 *      R0             : "1" 			: Operation succeeded
 * 			 		  "0" 			: Operation failure
 * Note: Mandatory for all types except SRAM and PSRAM
 */
// RAM_FUNC
int Read(uint32_t Address, uint32_t Size, uint8_t* buffer)
{
    int ret = LOADER_OK;

    printf("[LOADER] %s(), Reading %lu from address %08x\r\n", __func__, Size, (unsigned int)Address);

    __set_PRIMASK(0); // enable interrupts

    // if (Flash_S25FLXXXL_read_data(&flash_ctx, Address, Size, buffer) != 0)
    //     ret = LOADER_FAIL;

    __set_PRIMASK(1); // disable interrupts

    return ret;
}

/**
 * Description :
 * Write data to the device
 * Inputs    :
 *      Address       : Write location
 *      Size          : Length in bytes
 *      buffer        : Address where to get the data to write
 * outputs   :
 *      R0           : "1" 			: Operation succeeded
 *                     "0" 			: Operation failure
 * Note: Mandatory for all types except SRAM and PSRAM
 */
// RAM_FUNC
int Write(uint32_t Address, uint32_t Size, uint8_t* buffer)
{
    printf("[LOADER] %s(), Writing %lu bytes on address 0x%08x\r\n", __func__, Size, Address);

    __set_PRIMASK(0); // enable interrupts

    // #if defined(EXT_FLASH_ADDR_MASK)
    //     Address = Address & EXT_FLASH_ADDR_MASK;
    // #endif /* defined(EXT_FLASH_ADDR_MASK) */

    //     LOC_LedOn();
    //     Flash_S25FLXXXL_page_program(&flash_ctx, buffer, Address, Size);
    //     LOC_LedOff();
    __set_PRIMASK(1); // disable interrupts

    return LOADER_OK;
}

/**
 * Description :
 * Erase the full chip
 * Inputs    :
 *     None
 * outputs   :
 *     R0             : "1" : Operation succeeded
 * 					 "0" : Operation failure
 * Note: Not Mandatory for SRAM PSRAM and NOR_FLASH
 */
// RAM_FUNC
int MassErase(uint32_t Parallelism)
{
    UNUSED(Parallelism);

    printf("[LOADER] %s(), Full chip erase\r\n", __func__);

    __set_PRIMASK(0); // enable interrupts
    // LOC_LedOn();
    // Flash_S25FLXXXL_chip_erase(&flash_ctx);
    // LOC_LedOff();
    __set_PRIMASK(1); // disable interrupts

    return LOADER_OK;
}

/**
 * Description :
 * Erase a full sector in the device
 * Inputs    :
 *      SectrorAddress	: Start of sector
 *      Size          : Size (in WORD)
 *      InitVal       : Initial CRC value
 * outputs   :
 *     R0             : "1" : Operation succeeded
 * 			 		 "0" : Operation failure
 * Note: Not Mandatory for SRAM PSRAM and NOR_FLASH
 */
// RAM_FUNC
int SectorErase(uint32_t EraseStartAddress, uint32_t EraseEndAddress)
{
    uint32_t BlockAddr;

    __set_PRIMASK(0); // enable interrupts

    printf("[LOADER] %s(), Sector erasing from 0x%08x to address 0x%08x\r\n", __func__, (unsigned int)EraseStartAddress,
           (unsigned int)EraseEndAddress);

    // /* Map memory-mapped addresses to Flash internal addressing */
    // EraseStartAddress &= 0x0FFFFFFF; // Remove the mapping offset (0x90000000 -> 0x00000000)
    // EraseEndAddress &= 0x0FFFFFFF;   // Same adjustment for end address
    // printf("[LOADER] %s() Map memory-mapped address from 0x%08x to address 0x%08x\r\n", __func__,
    //        (unsigned int)EraseStartAddress, (unsigned int)EraseEndAddress);

    // /* Align start address to the nearest 4 KB sector boundary */
    // EraseStartAddress = EraseStartAddress - (EraseStartAddress % EXT_FLASH_SECTOR_SIZE);

    /* Erase all sectors between start and end addresses */
    while (EraseEndAddress >= EraseStartAddress)
    {
        BlockAddr = Flash_S25FLXXXL_helper_sector_number_get(EraseStartAddress);
        // BlockAddr = EraseStartAddress;
        printf("[LOADER] Erasing sector number %lu\r\n", BlockAddr);

        /* Erase the specified block */
        // if (Flash_S25FLXXXL_erase_sector(&flash_ctx, BlockAddr) != 0)
        // {
        //     /* Erase failed, re-enable interrupts and return failure */
        //     __enable_irq();
        //     return LOADER_FAIL;
        // }

        /* Move to the next sector */
        EraseStartAddress += EXT_FLASH_SECTOR_SIZE; // Increment by 4 KB sector size
    }

    __set_PRIMASK(1); // disable interrupts

    return 1;
}

/**
 * Description :
 * Calculates checksum value of the memory zone
 * Inputs    :
 *      StartAddress  : Flash start address
 *      Size          : Size (in WORD)
 *      InitVal       : Initial CRC value
 * outputs   :
 *     R0             : Checksum value
 * Note - Optional for all types of device
 * NOTE - keeping original ST algorithm: not verified and optimized
 */
// RAM_FUNC
uint32_t CheckSum(uint32_t StartAddress, uint32_t Size, uint32_t InitVal)
{
    printf("[LOADER] %s(), CheckSum data\r\n", __func__);
    uint8_t  missalignementAddress = StartAddress % 4;
    uint8_t  missalignementSize    = Size;
    int      cnt;
    uint32_t Val;
    // uint8_t value;

    StartAddress -= StartAddress % 4;
    Size += (Size % 4 == 0) ? 0 : 4 - (Size % 4);

    for (cnt = 0; cnt < Size; cnt += 4)
    {
        LOC_LedOn();
        Flash_S25FLXXXL_read(&flash_ctx, (uint8_t*)&Val, StartAddress + 1, 4);
        LOC_LedOff();

        if (missalignementAddress)
        {
            for (uint8_t k = missalignementAddress; k <= 3; k++)
            {
                InitVal += (uint8_t)(Val >> (8 * k) & 0xff);
            }
            missalignementAddress = 0;
        }
        else if ((Size - missalignementSize) % 4 && (Size - cnt) <= 4)
        {
            for (uint8_t k = (Size - missalignementSize); k <= 3; k++)
            {
                InitVal += (uint8_t)(Val >> (8 * (k - 1)) & 0xff);
            }
            missalignementSize = 2 * missalignementSize - Size;
        }
        else
        {
            for (uint8_t k = 0; k <= 3; k++)
            {
                InitVal += (uint8_t)(Val >> (8 * k) & 0xff);
            }
        }
        StartAddress += 4;
    }

    return (InitVal);
}

/**
 * Description :
 * Verify flash memory with RAM buffer and calculates checksum value of
 * the programmed memory
 * Inputs    :
 *      FlashAddr     : Flash address
 *      RAMBufferAddr : RAM buffer address
 *      Size          : Size (in WORD)
 *      InitVal       : Initial CRC value
 * outputs   :
 *     R0             : Operation failed (address of failure)
 *     R1             : Checksum value
 * Note: Optional for all types of device
 * NOTE - keeping original ST algorithm: not verified and optimized
 */
// RAM_FUNC
uint64_t Verify(uint32_t MemoryAddr, uint32_t RAMBufferAddr, uint32_t Size, uint32_t missalignement)
{
    printf("[LOADER] %s(), Verifying data\r\n", __func__);
#define BUF_SIZE 2
    uint32_t InitVal      = 0;
    uint32_t VerifiedData = 0;
    //  uint8_t TmpBuffer = 0x00;
    uint64_t checksum;
    Size *= 4;
    uint8_t  Buffer[BUF_SIZE];
    uint32_t LocAddr = 0;
    uint32_t posBuf;

#if defined(EXT_FLASH_ADDR_MASK)
    LocAddr = MemoryAddr & EXT_FLASH_ADDR_MASK;
#else
    LocAddr = MemoryAddr;
#endif /* defined(EXT_FLASH_ADDR_MASK) */

    checksum = CheckSum((uint32_t)LocAddr + (missalignement & 0xf), Size - ((missalignement >> 16) & 0xF), InitVal);

    while (Size > VerifiedData)
    {
        LOC_LedOn();

        Flash_S25FLXXXL_read_data(&flash_ctx, (MemoryAddr + VerifiedData), BUF_SIZE, Buffer);
        // Flash_S25FLXXXL_read(&flash_ctx, Buffer, MemoryAddr + VerifiedData, BUF_SIZE);
        LOC_LedOff();

        posBuf = 0;
        while ((Size > VerifiedData) && (posBuf < 1024))
        {
            if (Buffer[posBuf] != *((uint8_t*)RAMBufferAddr + VerifiedData))
                return ((checksum << 32) + MemoryAddr + VerifiedData);
            posBuf++;
            VerifiedData++;
        }
    }

    return (checksum << 32);
}

#endif /*  defined(EXT_LOADER_GENERATE) && EXT_LOADER_GENERATE */
