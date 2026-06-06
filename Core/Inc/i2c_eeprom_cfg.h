/**
 * Copyright (C) 2026 HexaMix
 * SPDX-License-Identifier: GPL-3.0-only
 * Dual-licensed — see LICENSE for commercial use.
 *
 * @file    i2c_eeprom_cfg.h
 * @brief   I2C EEPROM driver — user configuration.
 *
 * Adapt this file to your hardware setup:
 *   - Select the HAL I2C handle
 *   - Set the device variant constants
 *   - Tune timeout and retry behaviour
 *
 * Do NOT include this file directly in application code —
 * include i2c_eeprom.h instead.
 */

#ifndef I2C_EEPROM_CFG_H
#define I2C_EEPROM_CFG_H

#include "stm32g0xx_hal.h"   /* adjust to your STM32 family */

/* =========================================================================
 * I2C channel abstraction
 * =========================================================================
 * Declare the HAL handle that the driver should use.
 * The handle itself is defined in main.c (or i2c.c if using CubeMX).
 * Example: hi2c1, hi2c2, hi2c3
 * ======================================================================= */
extern I2C_HandleTypeDef hi2c2;
#define I2C_EEPROM_HANDLE       hi2c2


/* =========================================================================
 * Device variant — 24LC08
 * =========================================================================
 * 24LC08:  1024 bytes total, 4 x 256-byte blocks, 16-byte page write buffer
 * Change these if you swap to a different 24-series device:
 *   24LC01:  128  bytes, 1 block,  8-byte  page
 *   24LC02:  256  bytes, 1 block,  8-byte  page
 *   24LC04:  512  bytes, 2 blocks, 16-byte page
 *   24LC16:  2048 bytes, 8 blocks, 16-byte page
 *   24LC32:  4096 bytes, 1 block,  32-byte page  (16-bit addressing)
 * ======================================================================= */
#define I2C_EEPROM_BASE_ADDR        (0x50u)     /**< 7-bit base: 0b1010_000x        */
#define I2C_EEPROM_TOTAL_BYTES      (1024u)     /**< Total storage in bytes          */
#define I2C_EEPROM_BLOCK_SIZE       (256u)      /**< Bytes per address block         */
#define I2C_EEPROM_NUM_BLOCKS       (I2C_EEPROM_TOTAL_BYTES / I2C_EEPROM_BLOCK_SIZE)
#define I2C_EEPROM_PAGE_SIZE        (16u)       /**< Page write buffer in bytes      */


/* =========================================================================
 * Timing
 * =========================================================================
 * I2C_EEPROM_WRITE_TIMEOUT_MS  — HAL transmit/receive call timeout
 * I2C_EEPROM_WRITE_CYCLE_MS    — tWC: internal write cycle (5ms for 24LC08)
 * I2C_EEPROM_ACK_POLL_RETRIES  — max ACK poll attempts after a write
 *                                 Set to 0 to use fixed HAL_Delay instead
 * ======================================================================= */
#define I2C_EEPROM_WRITE_TIMEOUT_MS     (10u)
#define I2C_EEPROM_READ_TIMEOUT_MS      (10u)
#define I2C_EEPROM_WRITE_CYCLE_MS       (5u)
#define I2C_EEPROM_ACK_POLL_RETRIES     (10u)   /**< 0 = use HAL_Delay fallback     */


#endif /* I2C_EEPROM_CFG_H */
