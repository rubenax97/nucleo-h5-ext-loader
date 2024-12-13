 /**
 * @file    flash_s25flxxxl_config.h
 * @author  Thierry Zinkeng (tz@dc.systems)
 * @brief   This file contains the configurations of the S25FLXXXL SPI memory.
 * @version  1.0.0
 * @date    2024-11-21
 * 
 * @copyright Copyright DC Systems BV (c) 2024
 * 
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef S25FLXXXL_CONF_H
#define S25FLXXXL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx.h"
#include "usart.h"
#if defined(HAL_SPI_MODULE_ENABLED)
  #include "spi.h"
#endif
#if defined(HAL_XSPI_MODULE_ENABLED)
  #include "octospi.h"
#else
  #error "Please, enable SPI/OCTOSPI driver"acos
#endif


/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */

/** @addtogroup S25FLXXXL
  * @brief     This file provides a set of definitions for the Spansion
  *            S25FLXXXL memory configuration.
  * @{
  */

/** @addtogroup S25FLXXXL_Exported_Constants
  * @{
  */

#define CONF_S25FLXXXL_READ_ENHANCE            0  /* MMP performance enhance read enable/disable */
#define CONF_QSPI_DUMMY_CLOCK                  8U

/* Dummy cycles for STR read mode */
#define S25FLXXXL_DUMMY_CYCLES_READ_QUAD       8U
#define S25FLXXXL_DUMMY_CYCLES_READ            8U
#define S25FLXXXL_DUMMY_CYCLES_READ_DUAL_INOUT 4U
#define S25FLXXXL_DUMMY_CYCLES_READ_QUAD_INOUT 6U

/* Hardware configuration 
   @note Please, change these information just in case you want to 
   port everything to another platform*/
#define S25FLXXXL_SPI_INSTANCE          hspi2
#define S25FLXXXL_SPI_CS_PIN            GPIO_PIN_4
#define S25FLXXXL_SPI_CS_PORT           GPIOA
#define S25FLXXXL_UART_DEBUG_INSTANCE   huart7

#define S25FLXXXL_XSPI_INSTANCE         hospi1
#define FLASH_BASE_ADDR      0x90000000

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* S25FLXXXL_CONF_H */