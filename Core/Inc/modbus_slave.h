/**
 * @file    modbus_slave.h
 * @brief   Minimal Modbus RTU Slave — STM32F407-DISC1 test build
 *
 * Slave Address : 0x01
 * Baud          : 115200
 * UART          : USART2 (PA2=TX, PA3=RX)
 * Timer         : TIM10  (3.5T interframe = ~334 µs @ 115200)
 * Coils         : PD12..PD15 (onboard LEDs, FC01/FC05/FC0F)
 * Registers     : 4 holding regs mapped to coil shadow (FC03/FC06/FC10)
 */

#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include "stm32g0xx_hal.h"
#include "main.h"
#include <stdint.h>

/* ── public handles (set before calling Modbus_Init) ─────────────────────── */
extern UART_HandleTypeDef huart2;
extern LPTIM_HandleTypeDef hlptim1;

/* RS-485 direction pin — PA1 */
#define RS485_DE_PORT   RX_EN_GPIO_Port
#define RS485_DE_PIN    RX_EN_Pin
#define RS485_DE_HIGH() HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET)
#define RS485_DE_LOW()  HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET)

/* Coil values */
#define cu16_RELAY_OFF		((uint16_t) 0x0000u)
#define cu16_RELAY_ON		((uint16_t) 0xFF00u)
#define cu16_RELAY_TOGGLE	((uint16_t) 0x5500u)

/* ── configuration ───────────────────────────────────────────────────────── */
#define MB_SLAVE_ADDR       0x01u
#define MB_RX_BUF_SIZE      256u
#define MB_TX_BUF_SIZE      256u
#define MB_NUM_COILS        8u      /* PB12, PB2..PB0, PA7..PA4              */
#define MB_NUM_REGS         8u      /* holding registers, shadow coil state  */

typedef struct
{
	uint8_t coil[MB_NUM_COILS];
}Coil_Status_t;

/* ── API ─────────────────────────────────────────────────────────────────── */
/** HAL writes each received byte here. Arm UART with: HAL_UART_Receive_IT(&huart2, &g_lastRxByte, 1) */
extern uint8_t g_lastRxByte;

void Modbus_Init(void);

/**
 * @brief Call from USART2 IRQ handler  (HAL_UART_RxCpltCallback or raw IRQ).
 *        Each received byte is passed here.
 */
void Modbus_RxByteCallback(uint8_t byte);

/**
 * @brief Call from TIM10 period-elapsed callback.
 *        Signals end of Modbus frame (3.5T silence detected).
 */
void Modbus_TimerCallback(void);

#endif /* MODBUS_SLAVE_H */
