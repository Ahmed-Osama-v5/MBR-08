/**
 * @file    modbus_slave.c
 * @brief   Minimal Modbus RTU Slave — STM32F407-DISC1 test build
 *
 * Supported Function Codes:
 *   0x01 — Read Coils            (coils 0..3 → PD12..PD15)
 *   0x05 — Write Single Coil
 *   0x0F — Write Multiple Coils
 *   0x03 — Read Holding Registers (regs 0..3, shadow of coils)
 *   0x06 — Write Single Register
 *   0x10 — Write Multiple Registers
 *
 * Frame flow:
 *   UART RX byte → Modbus_RxByteCallback() → buffer + restart TIM10
 *   TIM10 fires  → Modbus_TimerCallback()  → process frame → send response
 */

#include "modbus_slave.h"
#include "modbus_timer.h"
#include <string.h>

/* ── configuration ───────────────────────────────────────────────────────── */
#define MB_SLAVE_ADDR       0x01u
#define MB_RX_BUF_SIZE      256u
#define MB_TX_BUF_SIZE      256u
#define MB_NUM_COILS        4u      /* PD12..PD15                            */
#define MB_NUM_REGS         4u      /* holding registers, shadow coil state  */

/* ── LED GPIO ────────────────────────────────────────────────────────────── */
/* STM32F407-DISC1: LD3=PD13(orange), LD4=PD12(green),
                    LD5=PD14(red),    LD6=PD15(blue)                        */
static const uint16_t LED_PINS[MB_NUM_COILS] =
{
    GPIO_PIN_12,    /* coil 0 → green  */
    GPIO_PIN_13,    /* coil 1 → orange */
    GPIO_PIN_14,    /* coil 2 → red    */
    GPIO_PIN_15     /* coil 3 → blue   */
};
#define LED_PORT    GPIOD

/* ── Modbus exception codes ─────────────────────────────────────────────── */
#define MB_EX_ILLEGAL_FUNCTION      0x01u
#define MB_EX_ILLEGAL_DATA_ADDRESS  0x02u
#define MB_EX_ILLEGAL_DATA_VALUE    0x03u

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

/* ── LED helpers ────────────────────────────────────────────────────────── */
static void led_apply(uint8_t coilIdx)
{
    if (s_coils[coilIdx])
        HAL_GPIO_WritePin(LED_PORT, LED_PINS[coilIdx], GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(LED_PORT, LED_PINS[coilIdx], GPIO_PIN_RESET);
}

static void led_applyAll(void)
{
    for (uint8_t i = 0u; i < MB_NUM_COILS; i++)
        led_apply(i);
}

/* ── response helpers ───────────────────────────────────────────────────── */
static void sendResponse(uint16_t len)
{
    uint16_t crc = crc16(s_txBuf, len);
    s_txBuf[len]     = (uint8_t)(crc & 0xFFu);         /* CRC lo           */
    s_txBuf[len + 1u] = (uint8_t)((crc >> 8u) & 0xFFu); /* CRC hi          */
    RS485_DE_HIGH();   // ← assert DE/RE before TX
    HAL_UART_Transmit_IT(&huart2, s_txBuf, len + 2u);
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
    if (value != 0x0000u && value != 0xFF00u)
    {
        sendException(0x05u, MB_EX_ILLEGAL_DATA_VALUE);
        return;
    }

    s_coils[addr] = (value == 0xFF00u) ? 1u : 0u;
    s_regs[addr]  = s_coils[addr];
    led_apply((uint8_t)addr);

    /* echo request as response */
    memcpy(s_txBuf, s_rxBuf, 6u);
    sendResponse(6u);
}

/* ── FC 0x0F — Write Multiple Coils ─────────────────────────────────────── */
static void fc0F_writeMultipleCoils(void)
{
    uint16_t startAddr = ((uint16_t)s_rxBuf[2] << 8u) | s_rxBuf[3];
    uint16_t quantity  = ((uint16_t)s_rxBuf[4] << 8u) | s_rxBuf[5];
    uint8_t  byteCount = s_rxBuf[6];

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
    }
    led_applyAll();

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
    led_apply((uint8_t)addr);

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
    }
    led_applyAll();

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
 *        Call after CubeMX MX_TIM10_Init() and MX_USART2_UART_Init().
 */
void Modbus_Init(void)
{
    memset(s_coils, 0, sizeof(s_coils));
    memset(s_regs,  0, sizeof(s_regs));
    s_rxLen = 0u;
    led_applyAll();

    /* enable UART receive interrupt — 1 byte at a time */
    HAL_UART_Receive_IT(&huart2, &g_lastRxByte, 1u);
}

/**
 * @brief Feed a received byte into the Modbus state machine.
 *        Call from HAL_UART_RxCpltCallback when huart->Instance == USART2.
 */
void Modbus_RxByteCallback(uint8_t byte)
{
    /* restart 3.5T timer on every byte */
	/* On every received byte — restart 3.5T window */
	HAL_LPTIM_OnePulse_Stop_IT(&hlptim1);                        /* stop + reset counter */
	uint32_t s_arrvalue = Modbus_GetArrValue();
	HAL_LPTIM_OnePulse_Start_IT(&hlptim1, s_arrvalue, 0U);       /* fresh one-shot */

    if (s_rxLen < MB_RX_BUF_SIZE)
        s_rxBuf[s_rxLen++] = byte;

    /* re-arm for next byte */
    HAL_UART_Receive_IT(&huart2, &g_lastRxByte, 1u);
}

/**
 * @brief Call from HAL_TIM_PeriodElapsedCallback when htim->Instance == TIM10.
 *        Signals 3.5T silence — frame is complete.
 */
void Modbus_TimerCallback(void)
{
	/* Explicit stop (e.g. during TX turnaround) */
	HAL_LPTIM_OnePulse_Stop_IT(&hlptim1);
    processFrame();
    s_rxLen = 0u;

    /* re-arm UART from start of buffer */
    HAL_UART_Receive_IT(&huart2, &g_lastRxByte, 1u);
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
