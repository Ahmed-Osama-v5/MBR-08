/**
 *
 * Copyright (C) 2026 HexaMix
 * SPDX-License-Identifier: GPL-3.0-only
 * Dual-licensed — see LICENSE for commercial use.
 *
 * @file    modbus_slave.h
 * @brief   Minimal Modbus RTU Slave
 *
 */

#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include "stm32g0xx_hal.h"
#include "main.h"
#include <stdint.h>

/* ── public handles (set before calling Modbus_Init) ─────────────────────── */
extern Modbus_UART_t MODBUS_UART_CH;

#define MODBUS_UART_HANDLE	(&MODBUS_UART_CH)

/* RS-485 direction pin — PA1 */
#define RS485_DE_PORT   RX_EN_GPIO_Port
#define RS485_DE_PIN    RX_EN_Pin
#define RS485_DE_HIGH() HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET)
#define RS485_DE_LOW()  HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET)

/* ── configuration ───────────────────────────────────────────────────────── */
#define MB_SLAVE_ADDR       0x01u
#define MB_RX_BUF_SIZE      256u
#define MB_TX_BUF_SIZE      256u
#define MB_NUM_COILS        8u      /* REL0 .. REL7					         */
#define MB_NUM_REGS         28u     /* holding registers, shadow coil state  */

#define cu16RELAY_DEFAULT_ON_DELAY	((uint16_t) 0x0005)
#define cu16RELAY_DEFAULT_OFF_DELAY	((uint16_t) 0x0005)

/* ── Register struct and values ──────────────────────────────────────────── */
typedef struct
{
	uint16_t u16Data;
	uint16_t u16Add;
}Reg_t;

typedef struct
{
	Reg_t* pRegs[MB_NUM_COILS];
}Regs_t;

typedef enum
{
	RELAY_OFF = 0x0000,
	RELAY_ON = 0xFF00,
	RELAY_TOGGLE = 0xAA00
}Relay_Value_t;

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
#define	cu16RELAY_CON_CH_0_ADD	((uint16_t) 0x0000)
#define	cu16RELAY_CON_CH_1_ADD	((uint16_t) 0x0001)
#define	cu16RELAY_CON_CH_2_ADD	((uint16_t) 0x0002)
#define	cu16RELAY_CON_CH_3_ADD	((uint16_t) 0x0003)
#define	cu16RELAY_CON_CH_4_ADD	((uint16_t) 0x0004)
#define	cu16RELAY_CON_CH_5_ADD	((uint16_t) 0x0005)
#define	cu16RELAY_CON_CH_6_ADD	((uint16_t) 0x0006)
#define	cu16RELAY_CON_CH_7_ADD	((uint16_t) 0x0007)

/* 0x00FF controls all relays
 * Permission: W
 * Functions: 0x05
 * Values:
 * 0xFF00: all relays on
 * 0x0000: all relays off
 * 0xAA00: all relays toggle
 * */
#define	cu16RELAY_CON_ALL_ADD	((uint16_t) 0x00FF)

/* 0x0200 - 0x0207 relay on delay
 * Permission: W
 * Functions: 0x05
 * Values:
 * 0x0005: 5 * 100ms
 * */
#define	cu16RELAY_ON_DELAY_CH_0_ADD	((uint16_t) 0x0200)
#define	cu16RELAY_ON_DELAY_CH_1_ADD	((uint16_t) 0x0201)
#define	cu16RELAY_ON_DELAY_CH_2_ADD	((uint16_t) 0x0202)
#define	cu16RELAY_ON_DELAY_CH_3_ADD	((uint16_t) 0x0203)
#define	cu16RELAY_ON_DELAY_CH_4_ADD	((uint16_t) 0x0204)
#define	cu16RELAY_ON_DELAY_CH_5_ADD	((uint16_t) 0x0205)
#define	cu16RELAY_ON_DELAY_CH_6_ADD	((uint16_t) 0x0206)
#define	cu16RELAY_ON_DELAY_CH_7_ADD	((uint16_t) 0x0207)

/* 0x0400 - 0x407 relay off delay
 * Permission: W
 * Functions: 0x05
 * Values:
 * 0x0005: 5 * 100ms
 * */
#define	cu16RELAY_OFF_DELAY_CH_0_ADD	((uint16_t) 0x0400)
#define	cu16RELAY_OFF_DELAY_CH_1_ADD	((uint16_t) 0x0401)
#define	cu16RELAY_OFF_DELAY_CH_2_ADD	((uint16_t) 0x0402)
#define	cu16RELAY_OFF_DELAY_CH_3_ADD	((uint16_t) 0x0403)
#define	cu16RELAY_OFF_DELAY_CH_4_ADD	((uint16_t) 0x0404)
#define	cu16RELAY_OFF_DELAY_CH_5_ADD	((uint16_t) 0x0405)
#define	cu16RELAY_OFF_DELAY_CH_6_ADD	((uint16_t) 0x0406)
#define	cu16RELAY_OFF_DELAY_CH_7_ADD	((uint16_t) 0x0407)

/* 0x2000 UART parameters
 * Permission: R/W
 * Functions: 0x03, 0x06
 * Values:
 * The high eight bits indicate the parity mode: 0x00~0x02
 * The low eight bits indicate the baud rate mode: 0x00~0x07
 * */
#define	cu16UART_PARAM_ADD				((uint16_t) 0x2000)

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
#define	cu16DEVICE_ADDRESS_ADD			((uint16_t) 0x3000)

/* 0x4000 Software version
 * Permission: R
 * Functions: 0x03
 * Values:
 * Converting to decimal and then shifting the decimal point
 * two places to the left will represent the software version
 * 0x0064 = 100 = V1.00
 * refer to cu16SOFTWARE_VERSION in main.h
 * */
#define	cu16SOFTWARE_VERSION_ADD		((uint16_t) 0x4000)

/* Coil values */
#define cu16_RELAY_OFF		((uint16_t) 0x0000u)
#define cu16_RELAY_ON		((uint16_t) 0xFF00u)
#define cu16_RELAY_TOGGLE	((uint16_t) 0x5500u)


typedef struct
{
	uint8_t coil[MB_NUM_COILS];
}Coil_Status_t;

/* ── API ─────────────────────────────────────────────────────────────────── */
/** HAL writes each received byte here. Arm UART with: HAL_UART_Receive_IT(&huart2, &g_lastRxByte, 1) */
extern uint8_t g_lastRxByte;

void Modbus_Init(void);

/**
 * @brief Call from uart IRQ handler  (HAL_UART_RxCpltCallback or raw IRQ).
 *        Each received byte is passed here.
 */
void Modbus_RxByteCallback(uint8_t byte);

/**
 * @brief Call from timer period-elapsed callback.
 *        Signals end of Modbus frame (3.5T silence detected).
 */
void ModbusTimer_FrameDoneCallback(void);

#endif /* MODBUS_SLAVE_H */
