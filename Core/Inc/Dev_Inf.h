/**
 * @file    Dev_Inf.h
 * @author  Thierry Zinkeng (tz@dc.systems)
 * @brief   This file provides the S25FLXXXL storage info.
 * @version  1.0.0
 * @date    2024-11-21
 * 
 * @copyright Copyright DC Systems BV (c) 2024
 * 
 */

#define MCU_FLASH  1
#define NAND_FLASH 2
#define NOR_FLASH  3
#define SRAM       4
#define PSRAM      5
#define PC_CARD    6
#define SPI_FLASH  7
#define I2C_FLASH  8
#define SDRAM      9
#define I2C_EEPROM 10

#define SECTOR_NUM 10 // Max Number of Sector types
// #define SECTOR_NUM 2 // Max Number of Sector types
// #define SECTOR_NUM 0x2000 // 8192 sectors

struct DeviceSectors
{
    unsigned long SectorNum;  // Number of Sectors
    unsigned long SectorSize; // Sector Size in Bytes
};

struct StorageInfo
{
    char                 DeviceName[100];    // Device Name and Description
    unsigned short       DeviceType;         // Device Type: ONCHIP, EXT8BIT, EXT16BIT, ...
    unsigned long        DeviceStartAddress; // Default Device Start Address
    unsigned long        DeviceSize;         // Total Size of Device
    unsigned long        PageSize;           // Programming Page Size
    unsigned char        EraseValue;         // Content of Erased Memory
    struct DeviceSectors sectors[SECTOR_NUM];
};

// typedef struct DeviceSectors
// {
//   unsigned long		SectorNum;     // Number of Sectors
//   unsigned long		SectorSize;    // Sector Size in Bytes
// } device_sector_t;

// struct StorageInfo
// {
//    char				DeviceName[100];		// Device Name and Description
//    unsigned short	DeviceType;				// Device Type: ONCHIP, EXT8BIT, EXT16BIT, ...
//    unsigned long	DeviceStartAddress;		// Default Device Start Address
//    unsigned long	DeviceSize;				// Total Size of Device
//    unsigned long	PageSize;				// Programming Page Size
//    unsigned char	EraseValue;				// Content of Erased Memory
//    device_sector_t	sectors[SECTOR_NUM];	// Flash sector types
//    unsigned long	padding[16];			// Total size of struct StorageInfo must be 200 bytes
// };