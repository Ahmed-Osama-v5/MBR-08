/**
 * Copyright (C) 2026 HexaMix
 * SPDX-License-Identifier: GPL-3.0-only
 * Dual-licensed — see LICENSE for commercial use.
 *
 * @file    i2c_eeprom.h
 * @brief   I2C EEPROM driver — public API.
 *
 * Supports byte, page, and multi-page operations on 24-series EEPROMs.
 * Configured via i2c_eeprom_cfg.h — no direct HAL references here.
 *
 * Usage example:
 * @code
 *   EEPROM_vidInit();
 *
 *   uint8_t u8Tx = 0xAB;
 *   EEPROM_enuWrite(0x0010u, &u8Tx, 1u);
 *
 *   uint8_t u8Rx = 0u;
 *   EEPROM_enuRead(0x0010u, &u8Rx, 1u);
 * @endcode
 */

#ifndef I2C_EEPROM_H
#define I2C_EEPROM_H

#include <stdint.h>
#include <stddef.h>

/* =========================================================================
 * Return codes
 * ======================================================================= */
typedef enum
{
    EEPROM_OK           = 0x00u,   /**< Operation succeeded                 */
    EEPROM_ERR_PARAM    = 0x01u,   /**< NULL pointer or out-of-range arg    */
    EEPROM_ERR_BOUNDS   = 0x02u,   /**< Address + length exceeds device     */
    EEPROM_ERR_I2C      = 0x03u,   /**< HAL I2C error                       */
    EEPROM_ERR_TIMEOUT  = 0x04u,   /**< ACK poll or HAL timeout             */
    EEPROM_ERR_BUSY     = 0x05u,   /**< Device busy after max retries       */
} EEPROM_Status_t;


/* =========================================================================
 * Init
 * ======================================================================= */

/**
 * @brief  Initialise the EEPROM driver.
 *         Verifies the I2C handle is ready; does not perform any bus
 *         transaction. Call once after HAL_I2C_Init().
 */
void EEPROM_vidInit(void);


/* =========================================================================
 * Core read / write
 * ======================================================================= */

/**
 * @brief  Read an arbitrary number of bytes starting at a flat address.
 *
 * The flat address space (0x0000..total-1) is transparently mapped to the
 * correct block address and 8-bit byte offset for the 24LC08.
 *
 * Sequential reads wrap within a 256-byte block on the 24LC08; this driver
 * automatically splits cross-block reads into multiple transactions.
 *
 * @param  u16Addr   Flat byte address (0x0000 .. EEPROM_TOTAL_BYTES-1).
 * @param  pu8Buf    Destination buffer.
 * @param  u16Len    Number of bytes to read.
 * @return EEPROM_OK on success, error code otherwise.
 */
EEPROM_Status_t EEPROM_enuRead(uint16_t u16Addr,
                                uint8_t *pu8Buf,
                                uint16_t u16Len);

/**
 * @brief  Write an arbitrary number of bytes starting at a flat address.
 *
 * Automatically splits the payload across 16-byte page boundaries to
 * respect the 24LC08 page write buffer. Each page write is followed by
 * an ACK-poll (or fixed delay if polling is disabled in cfg) to wait for
 * the internal write cycle to complete.
 *
 * @param  u16Addr   Flat byte address (0x0000 .. EEPROM_TOTAL_BYTES-1).
 * @param  pu8Buf    Source buffer.
 * @param  u16Len    Number of bytes to write.
 * @return EEPROM_OK on success, error code otherwise.
 */
EEPROM_Status_t EEPROM_enuWrite(uint16_t       u16Addr,
                                 const uint8_t *pu8Buf,
                                 uint16_t       u16Len);


/* =========================================================================
 * Convenience wrappers
 * ======================================================================= */

/**
 * @brief  Write a single byte.
 * @param  u16Addr   Target flat address.
 * @param  u8Data    Byte value to write.
 * @return EEPROM_OK on success, error code otherwise.
 */
EEPROM_Status_t EEPROM_enuWriteByte(uint16_t u16Addr, uint8_t u8Data);

/**
 * @brief  Read a single byte.
 * @param  u16Addr   Source flat address.
 * @param  pu8Data   Pointer to destination byte.
 * @return EEPROM_OK on success, error code otherwise.
 */
EEPROM_Status_t EEPROM_enuReadByte(uint16_t u16Addr, uint8_t *pu8Data);

/**
 * @brief  Write a uint16_t value (big-endian, high byte first).
 * @param  u16Addr   Target flat address (must have 2 bytes available).
 * @param  u16Data   Value to write.
 * @return EEPROM_OK on success, error code otherwise.
 */
EEPROM_Status_t EEPROM_enuWriteU16(uint16_t u16Addr, uint16_t u16Data);

/**
 * @brief  Read a uint16_t value (big-endian, high byte first).
 * @param  u16Addr   Source flat address (must have 2 bytes available).
 * @param  pu16Data  Pointer to destination uint16_t.
 * @return EEPROM_OK on success, error code otherwise.
 */
EEPROM_Status_t EEPROM_enuReadU16(uint16_t u16Addr, uint16_t *pu16Data);


/* =========================================================================
 * Diagnostics
 * ======================================================================= */

/**
 * @brief  Check whether the EEPROM acknowledges on the I2C bus.
 *         Useful as a startup self-test.
 * @return EEPROM_OK if device responds, EEPROM_ERR_I2C otherwise.
 */
EEPROM_Status_t EEPROM_enuIsReady(void);


#endif /* I2C_EEPROM_H */
