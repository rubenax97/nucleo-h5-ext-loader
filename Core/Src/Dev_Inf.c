/**
 * @file    Dev_Inf.c
 * @author  Thierry Zinkeng (tz@dc.systems)
 * @brief   This file provides the S25FLXXXL storage info.
 * @version  1.0.0
 * @date    2024-11-21
 *
 * @copyright Copyright DC Systems BV (c) 2024
 *
 */
#include "Dev_Inf.h"
#include "flash_s25flxxxl.h"
#include "main.h"

/* This structure containes information used by ST-LINK Utility to program and erase the device */
// #if defined(__ICCARM__)
// __root struct StorageInfo const StorageInfo = {
// #else
// struct StorageInfo const StorageInfo = {
// #endif
//     "S25FL256L_SENTRY-H563", // Device Name + version number
//     SPI_FLASH,               // Device Type  (that's from Dev_Inf.h)
// 	 0x90000000,              // Start Address (mapped address for STM32 OctoSPI)
// 	    0x02000000,              // Device Size (32 MB)
// 	    0x100,                   // Page Size (256 bytes)
// 	    0xFF,                    // Erase Value (flash default erase value)
// 	    {
// 	        {8192, 0x1000},      // Sector Size: 8192 sectors, 4 KB each
// 	        {0x00000000, 0x00000000}                // Terminator
// 	    }

//     		// Specify Size and Address of Sectors (view example below)
// 		// { { (MEMORY_FLASH_SIZE / MEMORY_SECTOR_SIZE),  // Sector Numbers,
// 		// 		(uint32_t) MEMORY_SECTOR_SIZE },       //Sector Size

// 		// 		{ 0x00000000, 0x00000000 } } };
// };

#if defined(__ICCARM__)
__root struct StorageInfo const StorageInfo = {
#else
struct StorageInfo const StorageInfo = {
#endif
    "S25FL256L_NUCLEO-H563ZI", // Device Name + version number
    SPI_FLASH,             // Device Type  (that's from Dev_Inf.h)
    // NOR_FLASH,                  					// Device Type  (that's from Dev_Inf.h)
    0x90000000,                                   // Device Start Address
    EXT_FLASH_SIZE,                               // Device Size in Bytes (that's from Flash interface package)
    EXT_FLASH_PAGE_SIZE,                          // Programming Page Size (that's from Flash interface package)
    0xFF,                                         // Initial Content of Erased Memory
                                                  // Specify Size and Address of Sectors (view example below)
    {{EXT_FLASH_PAGE_NUM, EXT_FLASH_SECTOR_SIZE}, // (that's from Flash interface package)
     {0x00000000, 0x00000000}}};

// /* This structure containes information used by ST-LINK Utility to program and erase the device */
// #if defined (__ICCARM__)
// __root struct StorageInfo const StorageInfo  =  {
// #else
// struct StorageInfo const StorageInfo  =  {
// #endif
//    "INTERNAL_G03", 	      					// Device Name
//    SRAM,                   	             				        // Device Type
//    0xC0000000,                						        // Device Start Address
//    0x1000000,              						        // Device Size in 16 MBytes
//    0x1000000,                    						// Programming Page Size 4096 Bytes
//    0xFF,                       						        // Initial Content of Erased Memory
// // Specify Size and Address of Sectors (view example below)
//    0x00000001, 0x01000000,     				 		        // Sector Num : 1 ,Sector Size: 16 MBytes
//    0x00000000, 0x00000000,
// };
