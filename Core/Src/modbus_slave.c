/**
 *
 * Copyright (C) 2026 HexaMix
 * SPDX-License-Identifier: GPL-3.0-only
 * Dual-licensed — see LICENSE for commercial use.
 *
 * @file    modbus_slave.c
 * @brief   Minimal Modbus RTU Slave — STM32G0B1CB
 *
 * Supported Function Codes:
 *   0x01 — Read Coils            (coils 0..7 )
 *   0x05 — Write Single Coil
 *   0x0F — Write Multiple Coils
 *   0x03 — Read Holding Registers (regs 0..7, shadow of coils)
 *   0x06 — Write Single Register
 *   0x10 — Write Multiple Registers
 *
 * Frame flow:
 *   UART RX byte → Modbus_RxByteCallback() → buffer + restart timer
 *   timer fires  → Modbus_TimerCallback()  → process frame → send response
 */

#include "modbus_slave.h"
#include "modbus_timer.h"
#include <string.h>
#include <stdbool.h>

/* ── Coils status ───────────────────────────────────────────────────────── */
Coil_Status_t str_CoilStates;
extern bool bNewData;

/* ── Modbus exception codes ─────────────────────────────────────────────── */
#define MB_EX_ILLEGAL_FUNCTION      0x01u
#define MB_EX_ILLEGAL_DATA_ADDRESS  0x02u
#define MB_EX_ILLEGAL_DATA_VALUE    0x03u


/* ── Holding registers ───────────────────────────────────────────────────── */
/* Relay channels */
/* 0x0000 - 0x0007 channels 0..7
 * Permission: R/W
 * Functions: 0x01, 0x05, 0x0F
 * Values:
 * 0xFF00: relay on
 * 0x0000: relay off
 * 0xAA00: relay toggle
 * */
Reg_t regRelayCon0;
Reg_t regRelayCon1;
Reg_t regRelayCon2;
Reg_t regRelayCon3;
Reg_t regRelayCon4;
Reg_t regRelayCon5;
Reg_t regRelayCon6;
Reg_t regRelayCon7;

/* struct that holds all relay control registers */
Regs_t regsRelCon;

/* 0x00FF controls all relays
 * Permission: W
 * Functions: 0x05
 * Values:
 * 0xFF00: all relays on
 * 0x0000: all relays off
 * 0xAA00: all relays toggle
 * */
Reg_t regRelayConAll;

/* 0x0200 - 0x0207 relay on delay
 * Permission: W
 * Functions: 0x05
 * Values:
 * 0x0005: 5 * 100ms
 * */
Reg_t regRelayDelOn0;
Reg_t regRelayDelOn1;
Reg_t regRelayDelOn2;
Reg_t regRelayDelOn3;
Reg_t regRelayDelOn4;
Reg_t regRelayDelOn5;
Reg_t regRelayDelOn6;
Reg_t regRelayDelOn7;

/* struct that holds all relay on delay registers */
Regs_t regsRelDelOn;

/* 0x0400 - 0x407 relay off delay
 * Permission: W
 * Functions: 0x05
 * Values:
 * 0x0005: 5 * 100ms
 * */
Reg_t regRelayDelOff0;
Reg_t regRelayDelOff1;
Reg_t regRelayDelOff2;
Reg_t regRelayDelOff3;
Reg_t regRelayDelOff4;
Reg_t regRelayDelOff5;
Reg_t regRelayDelOff6;
Reg_t regRelayDelOff7;

/* struct that holds all relay off delay registers */
Regs_t regsRelDelOff;

/* 0x2000 UART parameters
 * Permission: R/W
 * Functions: 0x03, 0x06
 * Values:
 * The high eight bits indicate the parity mode: 0x00~0x02
 * The low eight bits indicate the baud rate mode: 0x00~0x07
 * */
Reg_t regUART_Param;

/* 0x3000 Device address
 * Permission: R/W
 * Functions: 0x03, 0x06
 * Values:
 * Directly store Modbus address
 * Device address: 0x0001-0x00F7
 * Based on DIP switch config:
 * 0000: defaults to memory stored address (default: 0x01)
 * 0001 - 1111 : address: 0x01 - 0x0F
 * */
Reg_t regDevAdd;

/* 0x4000 Software version
 * Permission: R
 * Functions: 0x03
 * Values:
 * Converting to decimal and then shifting the decimal point
 * two places to the left will represent the software version
 * 0x0064 = 100 = V1.00
 * refer to cu16SOFTWARE_VERSION in main.h
 * */
Reg_t regSWVer;



/* ── internal state ─────────────────────────────────────────────────────── */
static uint8_t  s_rxBuf[MB_RX_BUF_SIZE];
static uint16_t s_rxLen;
       uint8_t  g_lastRxByte;   /* HAL writes here; exposed to main.c       */
static uint8_t  s_txBuf[MB_TX_BUF_SIZE];

static uint8_t  s_coils[MB_NUM_COILS];         /* 0 = OFF, 1 = ON          */
static uint16_t s_regs[MB_NUM_REGS];           /* holding registers         */

/* ── CRC-16 (Modbus polynomial 0xA001) ─────────────────────────────────── */
static uint16_t crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0u; i < len; i++)
    {
        crc ^= (uint16_t)buf[i];
        for (uint8_t b = 0u; b < 8u; b++)
        {
            if (crc & 0x0001u)
                crc = (crc >> 1u) ^ 0xA001u;
            else
                crc >>= 1u;
        }
    }
    return crc;
}

/* ── response helpers ───────────────────────────────────────────────────── */
static void sendResponse(uint16_t len)
{
	/* Turn TX led on */
	HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_SET);

    uint16_t crc = crc16(s_txBuf, len);
    s_txBuf[len]     = (uint8_t)(crc & 0xFFu);         /* CRC low           */
    s_txBuf[len + 1u] = (uint8_t)((crc >> 8u) & 0xFFu); /* CRC high         */
    RS485_DE_HIGH();   // ← assert DE/RE before TX
    HAL_UART_Transmit_IT(MODBUS_UART_HANDLE, s_txBuf, len + 2u);

	/* Turn TX led off */
	HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_RESET);
}

static void sendException(uint8_t fc, uint8_t exCode)
{
    s_txBuf[0] = MB_SLAVE_ADDR;
    s_txBuf[1] = fc | 0x80u;
    s_txBuf[2] = exCode;
    sendResponse(3u);
}

/* ── FC 0x01 — Read Coils ───────────────────────────────────────────────── */
static void fc01_readCoils(void)
{
    uint16_t startAddr = ((uint16_t)s_rxBuf[2] << 8u) | s_rxBuf[3];
    uint16_t quantity  = ((uint16_t)s_rxBuf[4] << 8u) | s_rxBuf[5];

    if (quantity == 0u || quantity > MB_NUM_COILS)
    {
        sendException(0x01u, MB_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    if ((startAddr + quantity) > MB_NUM_COILS)
    {
        sendException(0x01u, MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }

    uint8_t byteCount = (uint8_t)((quantity + 7u) / 8u);
    s_txBuf[0] = MB_SLAVE_ADDR;
    s_txBuf[1] = 0x01u;
    s_txBuf[2] = byteCount;

    for (uint8_t i = 0u; i < byteCount; i++)
        s_txBuf[3u + i] = 0x00u;

    for (uint16_t i = 0u; i < quantity; i++)
    {
        if (s_coils[startAddr + i])
            s_txBuf[3u + (i / 8u)] |= (uint8_t)(1u << (i % 8u));
    }

    sendResponse(3u + byteCount);
}

/* ── FC 0x05 — Write Single Coil ────────────────────────────────────────── */
static void fc05_writeSingleCoil(void)
{
    uint16_t addr  = ((uint16_t)s_rxBuf[2] << 8u) | s_rxBuf[3];
    uint16_t value = ((uint16_t)s_rxBuf[4] << 8u) | s_rxBuf[5];

    if (addr >= MB_NUM_COILS)
    {
        sendException(0x05u, MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }
    if (value != cu16_RELAY_OFF && value != cu16_RELAY_ON && value !=cu16_RELAY_TOGGLE)
    {
        sendException(0x05u, MB_EX_ILLEGAL_DATA_VALUE);
        return;
    }

    s_coils[addr] = (value == cu16_RELAY_ON) ? 1u : 0u;
    s_regs[addr]  = s_coils[addr];

    // sync value with app layer
    str_CoilStates.coil[addr] = s_coils[addr];

    /* echo request as response */
    memcpy(s_txBuf, s_rxBuf, 6u);
    sendResponse(6u);
}

/* ── FC 0x0F — Write Multiple Coils ─────────────────────────────────────── */
static void fc0F_writeMultipleCoils(void)
{
    uint16_t startAddr = ((uint16_t)s_rxBuf[2] << 8u) | s_rxBuf[3];
    uint16_t quantity  = ((uint16_t)s_rxBuf[4] << 8u) | s_rxBuf[5];

    if (quantity == 0u || quantity > MB_NUM_COILS)
    {
        sendException(0x0Fu, MB_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    if ((startAddr + quantity) > MB_NUM_COILS)
    {
        sendException(0x0Fu, MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }

    for (uint16_t i = 0u; i < quantity; i++)
    {
        uint8_t byteIdx = (uint8_t)(i / 8u);
        uint8_t bitIdx  = (uint8_t)(i % 8u);
        s_coils[startAddr + i] = (s_rxBuf[7u + byteIdx] >> bitIdx) & 0x01u;
        s_regs[startAddr + i]  = s_coils[startAddr + i];

        // sync value with app layer
        str_CoilStates.coil[startAddr + i] = s_coils[startAddr + i];
    }

    s_txBuf[0] = MB_SLAVE_ADDR;
    s_txBuf[1] = 0x0Fu;
    s_txBuf[2] = s_rxBuf[2];
    s_txBuf[3] = s_rxBuf[3];
    s_txBuf[4] = s_rxBuf[4];
    s_txBuf[5] = s_rxBuf[5];
    sendResponse(6u);
}

/* ── FC 0x03 — Read Holding Registers ───────────────────────────────────── */
static void fc03_readHoldingRegs(void)
{
    uint16_t startAddr = ((uint16_t)s_rxBuf[2] << 8u) | s_rxBuf[3];
    uint16_t quantity  = ((uint16_t)s_rxBuf[4] << 8u) | s_rxBuf[5];

    if (quantity == 0u || quantity > MB_NUM_REGS)
    {
        sendException(0x03u, MB_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    if ((startAddr + quantity) > MB_NUM_REGS)
    {
        sendException(0x03u, MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }

    uint8_t byteCount = (uint8_t)(quantity * 2u);
    s_txBuf[0] = MB_SLAVE_ADDR;
    s_txBuf[1] = 0x03u;
    s_txBuf[2] = byteCount;

    for (uint16_t i = 0u; i < quantity; i++)
    {
        s_txBuf[3u + (i * 2u)]      = (uint8_t)(s_regs[startAddr + i] >> 8u);
        s_txBuf[3u + (i * 2u) + 1u] = (uint8_t)(s_regs[startAddr + i] & 0xFFu);
    }

    sendResponse(3u + byteCount);
}

/* ── FC 0x06 — Write Single Register ────────────────────────────────────── */
static void fc06_writeSingleReg(void)
{
    uint16_t addr  = ((uint16_t)s_rxBuf[2] << 8u) | s_rxBuf[3];
    uint16_t value = ((uint16_t)s_rxBuf[4] << 8u) | s_rxBuf[5];

    if (addr >= MB_NUM_REGS)
    {
        sendException(0x06u, MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }

    s_regs[addr]  = value;
    s_coils[addr] = (value != 0u) ? 1u : 0u;

    // sync value with app layer
    str_CoilStates.coil[addr] = s_coils[addr];

    memcpy(s_txBuf, s_rxBuf, 6u);
    sendResponse(6u);
}

/* ── FC 0x10 — Write Multiple Registers ─────────────────────────────────── */
static void fc10_writeMultipleRegs(void)
{
    uint16_t startAddr = ((uint16_t)s_rxBuf[2] << 8u) | s_rxBuf[3];
    uint16_t quantity  = ((uint16_t)s_rxBuf[4] << 8u) | s_rxBuf[5];

    if (quantity == 0u || quantity > MB_NUM_REGS)
    {
        sendException(0x10u, MB_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    if ((startAddr + quantity) > MB_NUM_REGS)
    {
        sendException(0x10u, MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }

    for (uint16_t i = 0u; i < quantity; i++)
    {
        uint16_t val = ((uint16_t)s_rxBuf[7u + (i * 2u)] << 8u)
                     | s_rxBuf[7u + (i * 2u) + 1u];
        s_regs[startAddr + i]  = val;
        s_coils[startAddr + i] = (val != 0u) ? 1u : 0u;

        // sync value with app layer
        str_CoilStates.coil[startAddr + i] = s_coils[startAddr + i];
    }

    s_txBuf[0] = MB_SLAVE_ADDR;
    s_txBuf[1] = 0x10u;
    s_txBuf[2] = s_rxBuf[2];
    s_txBuf[3] = s_rxBuf[3];
    s_txBuf[4] = s_rxBuf[4];
    s_txBuf[5] = s_rxBuf[5];
    sendResponse(6u);
}

/* ── frame processor ────────────────────────────────────────────────────── */
static void processFrame(void)
{
    /* minimum frame: addr(1) + fc(1) + data(min 4) + crc(2) = 8 bytes      */
    if (s_rxLen < 8u)
        return;

    /* address filter */
    if (s_rxBuf[0] != MB_SLAVE_ADDR)
        return;

    /* CRC check */
    uint16_t rxCrc   = ((uint16_t)s_rxBuf[s_rxLen - 1u] << 8u)
                      | s_rxBuf[s_rxLen - 2u];
    uint16_t calcCrc = crc16(s_rxBuf, s_rxLen - 2u);
    if (rxCrc != calcCrc)
        return;

    uint8_t fc = s_rxBuf[1];
    switch (fc)
    {
        case 0x01u: fc01_readCoils();          break;
        case 0x05u: fc05_writeSingleCoil();    break;
        case 0x0Fu: fc0F_writeMultipleCoils(); break;
        case 0x03u: fc03_readHoldingRegs();    break;
        case 0x06u: fc06_writeSingleReg();     break;
        case 0x10u: fc10_writeMultipleRegs();  break;
        default:    sendException(fc, MB_EX_ILLEGAL_FUNCTION); break;
    }
}

/* ══════════════════════════════════════════════════════════════════════════
   Public API
   ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialise Modbus slave.
 *        Call after timer_Init() and USART_Init().
 */
void Modbus_Init(void)
{
	/* Init holding registers */
	/* Relay channels */
	regRelayCon0.u16Add = cu16RELAY_CON_CH_0_ADD;
	regRelayCon0.u16Data = RELAY_OFF;
	regRelayCon1.u16Add = cu16RELAY_CON_CH_1_ADD;
	regRelayCon1.u16Data = RELAY_OFF;
	regRelayCon2.u16Add = cu16RELAY_CON_CH_2_ADD;
	regRelayCon2.u16Data = RELAY_OFF;
	regRelayCon3.u16Add = cu16RELAY_CON_CH_3_ADD;
	regRelayCon3.u16Data = RELAY_OFF;
	regRelayCon4.u16Add = cu16RELAY_CON_CH_4_ADD;
	regRelayCon4.u16Data = RELAY_OFF;
	regRelayCon5.u16Add = cu16RELAY_CON_CH_5_ADD;
	regRelayCon5.u16Data = RELAY_OFF;
	regRelayCon6.u16Add = cu16RELAY_CON_CH_6_ADD;
	regRelayCon6.u16Data = RELAY_OFF;
	regRelayCon7.u16Add = cu16RELAY_CON_CH_7_ADD;
	regRelayCon7.u16Data = RELAY_OFF;

	regsRelCon.regs[0] = regRelayCon0;
	regsRelCon.regs[1] = regRelayCon1;
	regsRelCon.regs[2] = regRelayCon2;
	regsRelCon.regs[3] = regRelayCon3;
	regsRelCon.regs[4] = regRelayCon4;
	regsRelCon.regs[5] = regRelayCon5;
	regsRelCon.regs[6] = regRelayCon6;
	regsRelCon.regs[7] = regRelayCon7;

	regRelayConAll.u16Add = cu16RELAY_CON_ALL_ADD;
	regRelayConAll.u16Data = RELAY_OFF;

	/* Relay on delay */
	regRelayDelOn0.u16Add = cu16RELAY_ON_DELAY_CH_0_ADD;
	regRelayDelOn0.u16Data = cu16RELAY_DEFAULT_ON_DELAY;
	regRelayDelOn1.u16Add = cu16RELAY_ON_DELAY_CH_1_ADD;
	regRelayDelOn1.u16Data = cu16RELAY_DEFAULT_ON_DELAY;
	regRelayDelOn2.u16Add = cu16RELAY_ON_DELAY_CH_2_ADD;
	regRelayDelOn2.u16Data = cu16RELAY_DEFAULT_ON_DELAY;
	regRelayDelOn3.u16Add = cu16RELAY_ON_DELAY_CH_3_ADD;
	regRelayDelOn3.u16Data = cu16RELAY_DEFAULT_ON_DELAY;
	regRelayDelOn4.u16Add = cu16RELAY_ON_DELAY_CH_4_ADD;
	regRelayDelOn4.u16Data = cu16RELAY_DEFAULT_ON_DELAY;
	regRelayDelOn5.u16Add = cu16RELAY_ON_DELAY_CH_5_ADD;
	regRelayDelOn5.u16Data = cu16RELAY_DEFAULT_ON_DELAY;
	regRelayDelOn6.u16Add = cu16RELAY_ON_DELAY_CH_6_ADD;
	regRelayDelOn6.u16Data = cu16RELAY_DEFAULT_ON_DELAY;
	regRelayDelOn7.u16Add = cu16RELAY_ON_DELAY_CH_7_ADD;
	regRelayDelOn7.u16Data = cu16RELAY_DEFAULT_ON_DELAY;


	regsRelDelOn.regs[0] = regRelayDelOn0;
	regsRelDelOn.regs[1] = regRelayDelOn1;
	regsRelDelOn.regs[2] = regRelayDelOn2;
	regsRelDelOn.regs[3] = regRelayDelOn3;
	regsRelDelOn.regs[4] = regRelayDelOn4;
	regsRelDelOn.regs[5] = regRelayDelOn5;
	regsRelDelOn.regs[6] = regRelayDelOn6;
	regsRelDelOn.regs[7] = regRelayDelOn7;

	/* Relay off delay */
	regRelayDelOff0.u16Add = cu16RELAY_OFF_DELAY_CH_0_ADD;
	regRelayDelOff0.u16Data = cu16RELAY_DEFAULT_OFF_DELAY;
	regRelayDelOff1.u16Add = cu16RELAY_OFF_DELAY_CH_1_ADD;
	regRelayDelOff1.u16Data = cu16RELAY_DEFAULT_OFF_DELAY;
	regRelayDelOff2.u16Add = cu16RELAY_OFF_DELAY_CH_2_ADD;
	regRelayDelOff2.u16Data = cu16RELAY_DEFAULT_OFF_DELAY;
	regRelayDelOff3.u16Add = cu16RELAY_OFF_DELAY_CH_3_ADD;
	regRelayDelOff3.u16Data = cu16RELAY_DEFAULT_OFF_DELAY;
	regRelayDelOff4.u16Add = cu16RELAY_OFF_DELAY_CH_4_ADD;
	regRelayDelOff4.u16Data = cu16RELAY_DEFAULT_OFF_DELAY;
	regRelayDelOff5.u16Add = cu16RELAY_OFF_DELAY_CH_5_ADD;
	regRelayDelOff5.u16Data = cu16RELAY_DEFAULT_OFF_DELAY;
	regRelayDelOff6.u16Add = cu16RELAY_OFF_DELAY_CH_6_ADD;
	regRelayDelOff6.u16Data = cu16RELAY_DEFAULT_OFF_DELAY;
	regRelayDelOff7.u16Add = cu16RELAY_OFF_DELAY_CH_7_ADD;
	regRelayDelOff7.u16Data = cu16RELAY_DEFAULT_OFF_DELAY;


	regsRelDelOff.regs[0] = regRelayDelOff0;
	regsRelDelOff.regs[1] = regRelayDelOff1;
	regsRelDelOff.regs[2] = regRelayDelOff2;
	regsRelDelOff.regs[3] = regRelayDelOff3;
	regsRelDelOff.regs[4] = regRelayDelOff4;
	regsRelDelOff.regs[5] = regRelayDelOff5;
	regsRelDelOff.regs[6] = regRelayDelOff6;
	regsRelDelOff.regs[7] = regRelayDelOff7;


	/* UART parameters */
	regUART_Param.u16Add = cu16UART_PARAM_ADD;
	/* TODO: create default value */

	/* Device address */
	regDevAdd.u16Add = cu16DEVICE_ADDRESS_ADD;
	regDevAdd.u16Data = 0x01;

	/* Software version */
	regSWVer.u16Add = cu16SOFTWARE_VERSION_ADD;
	regSWVer.u16Data = cu16SOFTWARE_VERSION;

/* ************************************************************** */


    memset(s_coils, 0, sizeof(s_coils));
    memset(s_regs,  0, sizeof(s_regs));
    s_rxLen = 0u;

    for (uint16_t i = 0u; i < MB_NUM_COILS; i++)
    {
    	if (s_coils[i] == cu16_RELAY_ON)
    		str_CoilStates.coil[i] = 1;
    	else
    		str_CoilStates.coil[i] = 0;
    }

    /* enable UART receive interrupt — 1 byte at a time */
    HAL_UART_Receive_IT(MODBUS_UART_HANDLE, &g_lastRxByte, 1u);
}

/**
 * @brief Feed a received byte into the Modbus state machine.
 *        Call from HAL_UART_RxCpltCallback when huart->Instance == USART2.
 */
void Modbus_RxByteCallback(uint8_t byte)
{
	/* Turn RX led on */
	HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, GPIO_PIN_SET);

    /* restart 3.5T timer on every byte */
	/* On every received byte — restart 3.5T window */
	ModbusTimer_Restart();

    if (s_rxLen < MB_RX_BUF_SIZE)
        s_rxBuf[s_rxLen++] = byte;

    /* re-arm for next byte */
    HAL_UART_Receive_IT(MODBUS_UART_HANDLE, &g_lastRxByte, 1u);

	/* Turn RX led off */
	HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, GPIO_PIN_RESET);

	bNewData = true;
}

/**
 * @brief Call from HAL_TIM_PeriodElapsedCallback when htim->Instance == TIM10.
 *        Signals 3.5T silence — frame is complete.
 */
void ModbusTimer_FrameDoneCallback(void)
{
	/* Explicit stop (e.g. during TX turnaround) */
	ModbusTimer_Stop();
    processFrame();
    s_rxLen = 0u;

    /* re-arm UART from start of buffer */
    HAL_UART_Receive_IT(MODBUS_UART_HANDLE, &g_lastRxByte, 1u);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
    	// Wait for shift register to fully empty
    	while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET);
        RS485_DE_LOW();   // ← release bus after last byte is fully shifted out
    }
}
