/**
 * Copyright (C) 2026 HexaMix
 * SPDX-License-Identifier: GPL-3.0-only
 * Dual-licensed — see LICENSE for commercial use.
 *
 * @file    i2c_eeprom.c
 * @brief   I2C EEPROM driver — implementation.
 *
 * 24LC08 addressing model:
 *
 *   Flat address space: 0x0000 .. 0x03FF  (1024 bytes)
 *
 *   The 24LC08 uses the lower 2 bits of the I2C device address (A1:A0)
 *   to select one of 4 x 256-byte blocks. The 8-bit byte address then
 *   selects the offset within that block.
 *
 *   7-bit I2C address sent on bus:
 *     bits [6:3] = 0b1010  (fixed)
 *     bits [2:1] = block   (0..3)
 *     bit  [0]   = R/W     (handled by HAL)
 *
 *   Flat → block + offset mapping:
 *     block  = flat_addr / 256  (bits [9:8] of flat address)
 *     offset = flat_addr % 256  (bits [7:0] of flat address)
 */

#include "i2c_eeprom.h"
#include "i2c_eeprom_cfg.h"

/* =========================================================================
 * Private macros
 * ======================================================================= */

/** Build the 7-bit I2C address for a given block (shifted left 1 by HAL) */
#define EEPROM_I2C_ADDR(block)  \
    (uint16_t)((I2C_EEPROM_BASE_ADDR | ((block) & 0x03u)) << 1u)

/** Extract block index from a flat address */
#define EEPROM_BLOCK(addr)      ((uint8_t)((addr) / I2C_EEPROM_BLOCK_SIZE))

/** Extract byte offset within a block from a flat address */
#define EEPROM_OFFSET(addr)     ((uint8_t)((addr) % I2C_EEPROM_BLOCK_SIZE))

/** Bytes remaining until the next page boundary from a given flat address */
#define EEPROM_BYTES_TO_PAGE_END(addr) \
    (uint16_t)(I2C_EEPROM_PAGE_SIZE - ((addr) % I2C_EEPROM_PAGE_SIZE))

/** Bytes remaining in the current block from a given flat address */
#define EEPROM_BYTES_TO_BLOCK_END(addr) \
    (uint16_t)(I2C_EEPROM_BLOCK_SIZE - EEPROM_OFFSET(addr))


/* =========================================================================
 * Private types
 * ======================================================================= */

/** Internal state */
typedef struct
{
    uint8_t u8Initialised;
} EEPROM_State_t;


/* =========================================================================
 * Private variables
 * ======================================================================= */

static EEPROM_State_t s_state = { .u8Initialised = 0u };


/* =========================================================================
 * Private function prototypes
 * ======================================================================= */

static EEPROM_Status_t enuAckPoll(uint8_t u8Block);
static EEPROM_Status_t enuWritePage(uint16_t       u16Addr,
                                    const uint8_t *pu8Buf,
                                    uint8_t        u8Len);
static EEPROM_Status_t enuReadBlock(uint16_t  u16Addr,
                                    uint8_t  *pu8Buf,
                                    uint16_t  u16Len);


/* =========================================================================
 * Public — Init
 * ======================================================================= */

void EEPROM_vidInit(void)
{
    s_state.u8Initialised = 1u;
}


/* =========================================================================
 * Public — Diagnostics
 * ======================================================================= */

EEPROM_Status_t EEPROM_enuIsReady(void)
{
    HAL_StatusTypeDef halStatus;

    halStatus = HAL_I2C_IsDeviceReady(
                    &I2C_EEPROM_HANDLE,
                    EEPROM_I2C_ADDR(0u),
                    I2C_EEPROM_ACK_POLL_RETRIES,
                    I2C_EEPROM_WRITE_TIMEOUT_MS);

    return (halStatus == HAL_OK) ? EEPROM_OK : EEPROM_ERR_I2C;
}


/* =========================================================================
 * Public — Read
 * ======================================================================= */

EEPROM_Status_t EEPROM_enuRead(uint16_t u16Addr,
                                uint8_t *pu8Buf,
                                uint16_t u16Len)
{
    EEPROM_Status_t enuStatus;
    uint16_t        u16Remaining;
    uint16_t        u16ChunkLen;
    uint16_t        u16CurAddr;
    uint8_t        *pu8Cur;

    /* ---- Parameter checks ---- */
    if (pu8Buf == NULL)
        return EEPROM_ERR_PARAM;

    if (u16Len == 0u)
        return EEPROM_ERR_PARAM;

    if ((u16Addr + u16Len) > I2C_EEPROM_TOTAL_BYTES)
        return EEPROM_ERR_BOUNDS;

    if (s_state.u8Initialised == 0u)
        return EEPROM_ERR_PARAM;

    /* ---- Split reads across block boundaries ---- */
    u16Remaining = u16Len;
    u16CurAddr   = u16Addr;
    pu8Cur       = pu8Buf;

    while (u16Remaining > 0u)
    {
        /* How many bytes until the end of the current 256-byte block */
        u16ChunkLen = EEPROM_BYTES_TO_BLOCK_END(u16CurAddr);

        if (u16ChunkLen > u16Remaining)
            u16ChunkLen = u16Remaining;

        enuStatus = enuReadBlock(u16CurAddr, pu8Cur, u16ChunkLen);
        if (enuStatus != EEPROM_OK)
            return enuStatus;

        u16CurAddr   += u16ChunkLen;
        pu8Cur       += u16ChunkLen;
        u16Remaining -= u16ChunkLen;
    }

    return EEPROM_OK;
}


/* =========================================================================
 * Public — Write
 * ======================================================================= */

EEPROM_Status_t EEPROM_enuWrite(uint16_t       u16Addr,
                                 const uint8_t *pu8Buf,
                                 uint16_t       u16Len)
{
    EEPROM_Status_t enuStatus;
    uint16_t        u16Remaining;
    uint16_t        u16ChunkLen;
    uint16_t        u16CurAddr;
    const uint8_t  *pu8Cur;

    /* ---- Parameter checks ---- */
    if (pu8Buf == NULL)
        return EEPROM_ERR_PARAM;

    if (u16Len == 0u)
        return EEPROM_ERR_PARAM;

    if ((u16Addr + u16Len) > I2C_EEPROM_TOTAL_BYTES)
        return EEPROM_ERR_BOUNDS;

    if (s_state.u8Initialised == 0u)
        return EEPROM_ERR_PARAM;

    /* ---- Split writes across page boundaries ---- */
    u16Remaining = u16Len;
    u16CurAddr   = u16Addr;
    pu8Cur       = pu8Buf;

    while (u16Remaining > 0u)
    {
        /* How many bytes until the end of the current 16-byte page */
        u16ChunkLen = EEPROM_BYTES_TO_PAGE_END(u16CurAddr);

        if (u16ChunkLen > u16Remaining)
            u16ChunkLen = u16Remaining;

        /* Page write is max 16 bytes — cast is safe */
        enuStatus = enuWritePage(u16CurAddr, pu8Cur, (uint8_t)u16ChunkLen);
        if (enuStatus != EEPROM_OK)
            return enuStatus;

        /* Wait for internal write cycle to complete */
        enuStatus = enuAckPoll(EEPROM_BLOCK(u16CurAddr));
        if (enuStatus != EEPROM_OK)
            return enuStatus;

        u16CurAddr   += u16ChunkLen;
        pu8Cur       += u16ChunkLen;
        u16Remaining -= u16ChunkLen;
    }

    return EEPROM_OK;
}


/* =========================================================================
 * Public — Convenience wrappers
 * ======================================================================= */

EEPROM_Status_t EEPROM_enuWriteByte(uint16_t u16Addr, uint8_t u8Data)
{
    return EEPROM_enuWrite(u16Addr, &u8Data, 1u);
}

EEPROM_Status_t EEPROM_enuReadByte(uint16_t u16Addr, uint8_t *pu8Data)
{
    return EEPROM_enuRead(u16Addr, pu8Data, 1u);
}

EEPROM_Status_t EEPROM_enuWriteU16(uint16_t u16Addr, uint16_t u16Data)
{
    uint8_t au8Buf[2u];
    au8Buf[0u] = (uint8_t)(u16Data >> 8u);    /* high byte first */
    au8Buf[1u] = (uint8_t)(u16Data & 0x00FFu);
    return EEPROM_enuWrite(u16Addr, au8Buf, 2u);
}

EEPROM_Status_t EEPROM_enuReadU16(uint16_t u16Addr, uint16_t *pu16Data)
{
    EEPROM_Status_t enuStatus;
    uint8_t         au8Buf[2u];

    if (pu16Data == NULL)
        return EEPROM_ERR_PARAM;

    enuStatus = EEPROM_enuRead(u16Addr, au8Buf, 2u);
    if (enuStatus == EEPROM_OK)
        *pu16Data = ((uint16_t)au8Buf[0u] << 8u) | (uint16_t)au8Buf[1u];

    return enuStatus;
}


/* =========================================================================
 * Private — single page write (max 16 bytes, must not cross page boundary)
 * ======================================================================= */

/**
 * @brief  Write up to 16 bytes within a single page of a single block.
 *
 * I2C transaction:
 *   [START][DEV_ADDR+W][BYTE_ADDR][DATA_0]..[DATA_N][STOP]
 *
 * The write buffer is constructed as:
 *   byte 0     = 8-bit byte address (offset within block)
 *   bytes 1..N = payload
 *
 * @param  u16Addr   Flat address of first byte (page-aligned start assumed
 *                   by caller — EEPROM_enuWrite splits on page boundaries).
 * @param  pu8Buf    Source data.
 * @param  u8Len     Number of bytes (1..16).
 */
static EEPROM_Status_t enuWritePage(uint16_t       u16Addr,
                                    const uint8_t *pu8Buf,
                                    uint8_t        u8Len)
{
    /* Transmit buffer: 1 byte address + up to 16 bytes data */
    uint8_t          au8TxBuf[1u + I2C_EEPROM_PAGE_SIZE];
    HAL_StatusTypeDef halStatus;
    uint8_t           u8Block;
    uint8_t           u8Offset;

    u8Block  = EEPROM_BLOCK(u16Addr);
    u8Offset = EEPROM_OFFSET(u16Addr);

    au8TxBuf[0u] = u8Offset;   /* byte address within block */

    for (uint8_t i = 0u; i < u8Len; i++)
        au8TxBuf[1u + i] = pu8Buf[i];

    halStatus = HAL_I2C_Master_Transmit(
                    &I2C_EEPROM_HANDLE,
                    EEPROM_I2C_ADDR(u8Block),
                    au8TxBuf,
                    (uint16_t)(1u + u8Len),
                    I2C_EEPROM_WRITE_TIMEOUT_MS);

    return (halStatus == HAL_OK) ? EEPROM_OK : EEPROM_ERR_I2C;
}


/* =========================================================================
 * Private — block read (must not cross 256-byte block boundary)
 * ======================================================================= */

/**
 * @brief  Read up to 256 bytes from within a single block.
 *
 * Uses the I2C random-read sequence:
 *   [START][DEV_ADDR+W][BYTE_ADDR][RESTART][DEV_ADDR+R][DATA...][STOP]
 *
 * HAL_I2C_Mem_Read handles the dummy write + repeated-start internally.
 *
 * @param  u16Addr   Flat start address.
 * @param  pu8Buf    Destination buffer.
 * @param  u16Len    Bytes to read (must not cross block boundary).
 */
static EEPROM_Status_t enuReadBlock(uint16_t  u16Addr,
                                    uint8_t  *pu8Buf,
                                    uint16_t  u16Len)
{
    HAL_StatusTypeDef halStatus;
    uint8_t           u8Block;
    uint8_t           u8Offset;

    u8Block  = EEPROM_BLOCK(u16Addr);
    u8Offset = EEPROM_OFFSET(u16Addr);

    halStatus = HAL_I2C_Mem_Read(
                    &I2C_EEPROM_HANDLE,
                    EEPROM_I2C_ADDR(u8Block),
                    (uint16_t)u8Offset,       /* 8-bit memory address */
                    I2C_MEMADD_SIZE_8BIT,
                    pu8Buf,
                    u16Len,
                    I2C_EEPROM_READ_TIMEOUT_MS);

    return (halStatus == HAL_OK) ? EEPROM_OK : EEPROM_ERR_I2C;
}


/* =========================================================================
 * Private — ACK polling after write
 * ======================================================================= */

/**
 * @brief  Poll the device with HAL_I2C_IsDeviceReady until it ACKs,
 *         indicating the internal write cycle is complete.
 *
 *         If I2C_EEPROM_ACK_POLL_RETRIES == 0, falls back to a fixed
 *         HAL_Delay of I2C_EEPROM_WRITE_CYCLE_MS instead.
 *
 * @param  u8Block   Block index (0..3) to poll — each block has its own
 *                   device address on the 24LC08.
 */
static EEPROM_Status_t enuAckPoll(uint8_t u8Block)
{
#if (I2C_EEPROM_ACK_POLL_RETRIES == 0u)

    /* Fixed delay fallback — simpler but wastes time */
    HAL_Delay(I2C_EEPROM_WRITE_CYCLE_MS);
    return EEPROM_OK;

#else

    HAL_StatusTypeDef halStatus;
    uint8_t           u8Retries = I2C_EEPROM_ACK_POLL_RETRIES;

    do
    {
        halStatus = HAL_I2C_IsDeviceReady(
                        &I2C_EEPROM_HANDLE,
                        EEPROM_I2C_ADDR(u8Block),
                        1u,
                        I2C_EEPROM_WRITE_TIMEOUT_MS);
        u8Retries--;
    }
    while ((halStatus != HAL_OK) && (u8Retries > 0u));

    if (halStatus != HAL_OK)
        return EEPROM_ERR_BUSY;

    return EEPROM_OK;

#endif /* I2C_EEPROM_ACK_POLL_RETRIES */
}
