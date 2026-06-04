/*
 * modbus_timer.c
 *
 *  Created on: 3 Jun 2026
 *      Author: Osama
 */

/* modbus_timer.c
 *
 * LPTIM1 clocked from PCLK (16 MHz HSI, no PLL, APB prescaler /1)
 * Prescaler /16 → 1 tick = 1 µs, ARR_MAX = 65535 → ~65.5 ms max
 *
 * Modbus RTU 3.5T inter-frame gap:
 *   baud ≤ 19200 → T35_us = (3.5 × 11 × 1e6) / baud
 *   baud > 19200 → T35_us = 1750 (fixed per Modbus spec §2.5.1)
 *
 * GPIO baud-select pins (example: PB4, PB5 — adjust to your schematic):
 *   PB4=0, PB5=0 →  1200 baud
 *   PB4=1, PB5=0 →  9600 baud
 *   PB4=0, PB5=1 → 19200 baud
 *   PB4=1, PB5=1 → 115200 baud
 */

#include "modbus_timer.h"
#include "stm32g0xx.h"   /* CMSIS header for G0 family */

/* ------------------------------------------------------------------ */
/* Configure                                                          */
/* ------------------------------------------------------------------ */

#define LPTIM_PRESCALER_DIV 16UL         /* /16 → 1 tick = 1 µs       */
#define LPTIM_PRESC_BITS    (0x4UL << LPTIM_CFGR_PRESC_Pos) /* b100 = /16 */

/* Baud select GPIO — PB4 = bit0, PB5 = bit1 */
#define BAUD_SEL_PORT   GPIOB
#define BAUD_SEL_PIN0   4U   /* LSB */
#define BAUD_SEL_PIN1   5U   /* MSB */

/* ------------------------------------------------------------------ */
/* Private helpers                                                      */
/* ------------------------------------------------------------------ */

/**
 * @brief  Compute 3.5T inter-frame gap in microseconds.
 *         Fixed minimum of 1750 µs for baud > 19200 (Modbus RTU spec §2.5.1).
 */
static uint32_t prvCalc_T35_us(uint32_t baud)
{
    if (baud <= 19200U)
    {
        /* 3.5 × 11 bits × 1e6 µs/s ÷ baud  →  numerator = 38500000 */
        return 38500000UL / baud;   /* integer division, acceptable rounding */
    }
    return 1750U;
}

/**
 * @brief  Convert microseconds to LPTIM ticks.
 *         tick_period = LPTIM_PRESCALER_DIV / SystemCoreClock seconds
 *         ticks = us × CLK_HZ / (PRESCALER_DIV × 1e6)
 */
static uint32_t prvUs_to_Ticks(uint32_t us)
{
    /* HAL_RCC_GetPCLK1Freq() reads the actual APB1 clock at call time.
     * LPTIM1 is on APB1 on G0B1, so this is always correct regardless
     * of what SYSCLK or APB prescaler you've configured.              */
    uint32_t lptimClk = HAL_RCC_GetPCLK1Freq() / LPTIM_PRESCALER_DIV;

    return (us * lptimClk) / 1000000UL;
}

/**
 * @brief  Read 2-pin GPIO encoding and return the selected baud rate.
 *         Pins must already be configured as inputs with pull-downs.
 */
static ModbusBaud_t prvReadBaudPins(void)
{
    uint32_t idr = BAUD_SEL_PORT->IDR;
    uint8_t  sel = (uint8_t)(((idr >> BAUD_SEL_PIN0) & 0x1U) |
                             (((idr >> BAUD_SEL_PIN1) & 0x1U) << 1U));

    switch (sel)
    {
        case 0x0U: return MODBUS_BAUD_1200;
        case 0x1U: return MODBUS_BAUD_9600;
        case 0x2U: return MODBUS_BAUD_19200;
        case 0x3U: return MODBUS_BAUD_115200;
        default:   return MODBUS_BAUD_9600;   /* unreachable, satisfy compiler */
    }
}

/* ------------------------------------------------------------------ */
/* Module state                                                         */
/* ------------------------------------------------------------------ */

static uint32_t s_arrValue = 0U;   /* cached ARR for restart */

/* ------------------------------------------------------------------ */
/* Private: low-level LPTIM write helpers                               */
/* ------------------------------------------------------------------ */

/**
 * @brief  Wait for ARR write to propagate to LPTIM domain.
 *         Required because LPTIM register writes cross a clock domain boundary.
 *         Timeout guards against a broken clock source.
 */
static void prvWaitARROK(void)
{
    uint32_t timeout = 1000U;
    while (((LPTIM1->ISR & LPTIM_ISR_ARROK) == 0U) && (timeout-- > 0U))
    {
        __NOP();
    }
    LPTIM1->ICR = LPTIM_ICR_ARROKCF;   /* clear the flag */
}

/**
 * @brief  Fully stop LPTIM1 (disable, clear counter).
 *         Disabling resets the internal counter on G0.
 */
static void prvLPTIM_Stop(void)
{
    LPTIM1->CR = 0U;   /* clear ENABLE — counter resets */
}

/**
 * @brief  Load ARR and fire a single-shot start.
 *         LPTIM must be ENABLED before ARR is written (G0 RM §24.4.6).
 */
static void prvLPTIM_SingleShot(uint32_t arr)
{
    /* 1. Enable (counter reset to 0) */
    LPTIM1->CR = LPTIM_CR_ENABLE;

    /* 2. Write ARR — must happen while ENABLE=1 */
    LPTIM1->ARR = arr;
    prvWaitARROK();

    /* 3. Clear any stale match flags before starting */
    LPTIM1->ICR = LPTIM_ICR_ARRMCF;

    /* 4. Single-shot start */
    LPTIM1->CR |= LPTIM_CR_SNGSTRT;
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

/**
 * @brief  Initialise GPIO baud-select pins, compute 3.5T ARR, configure LPTIM1.
 *         Call once after system clock init, before USART init.
 *
 * Clock assumptions:
 *   - HSI = 16 MHz, selected as SYSCLK
 *   - AHB prescaler /1, APB prescaler /1  → PCLK = 16 MHz
 *   - LPTIM1 kernel clock = PCLK (default after reset on G0B1)
 *
 * If you select a different LPTIM kernel clock (LSI, LSE, HSI16 via
 * RCC_CCIPR_LPTIM1SEL), update SystemCoreClock accordingly.
 */
void ModbusTimer_Init(ModbusBaud_t baud)
{
    /* ---- 1. GPIO baud-select pins: input, pull-down ---- */
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    /* Clear MODER bits → input (00) for PIN0 and PIN1 */
    GPIOB->MODER &= ~((0x3UL << (BAUD_SEL_PIN0 * 2U)) |
                      (0x3UL << (BAUD_SEL_PIN1 * 2U)));

    /* Pull-down (10) */
    GPIOB->PUPDR &= ~((0x3UL << (BAUD_SEL_PIN0 * 2U)) |
                      (0x3UL << (BAUD_SEL_PIN1 * 2U)));
    GPIOB->PUPDR |=  ((0x2UL << (BAUD_SEL_PIN0 * 2U)) |
                      (0x2UL << (BAUD_SEL_PIN1 * 2U)));

    /* Small settling delay for pull-down to stabilise */
    for (volatile uint32_t i = 0U; i < 100U; i++) { __NOP(); }

    /* ---- 2. Read baud pins (override argument if you always read GPIO) ---- */
    (void)baud;                          /* caller can pass 0 to force pin-read */
    ModbusBaud_t selectedBaud = prvReadBaudPins();
    uint32_t     t35_us       = prvCalc_T35_us((uint32_t)selectedBaud);
    s_arrValue                = prvUs_to_Ticks(t35_us);

    /* ARR must be ≥ 1 and fit in 16 bits */
    if (s_arrValue < 1U)    { s_arrValue = 1U;      }
    if (s_arrValue > 0xFFFFU) { s_arrValue = 0xFFFFU; }

    /* ---- 3. Enable LPTIM1 peripheral clock ---- */
    RCC->APBENR1 |= RCC_APBENR1_LPTIM1EN;

    /* ---- 4. LPTIM kernel clock: select PCLK (bits = 00, reset default) ---- */
    RCC->CCIPR &= ~RCC_CCIPR_LPTIM1SEL_Msk;   /* 00 = PCLK */

    /* ---- 5. Configure LPTIM1 (must be done while ENABLE=0) ---- */
    LPTIM1->CR   = 0U;   /* disable first */
    LPTIM1->CFGR = LPTIM_PRESC_BITS;   /* /16, all other fields = 0 (internal clk, no filter) */
    LPTIM1->IER  = LPTIM_IER_ARRMIE;   /* interrupt on ARR match only */

    /* Do NOT start yet — wait for first RXNE byte */
}

/**
 * @brief  Restart the 3.5T one-shot timer.
 *         Call this from the USART RXNE interrupt handler on every received byte.
 *         Stopping then restarting resets the counter, extending the window.
 */
void ModbusTimer_Restart(void)
{
    prvLPTIM_Stop();
    prvLPTIM_SingleShot(s_arrValue);
}

/**
 * @brief  Stop the timer explicitly (e.g. during TX turnaround).
 */
void ModbusTimer_Stop(void)
{
    prvLPTIM_Stop();
}

uint32_t Modbus_GetArrValue(void)
{
	return s_arrValue;
}

/**
 * @brief  LPTIM1 global IRQ handler.
 */
void LPTIM1_IRQHandler(void)
{
    if ((LPTIM1->ISR & LPTIM_ISR_ARRM) != 0U)
    {
        LPTIM1->ICR = LPTIM_ICR_ARRMCF;   /* clear flag */
        prvLPTIM_Stop();                   /* stop — frame window closed */

        ModbusTimer_FrameDoneCallback();
    }
}

/**
 * @brief  Weak default callback. Override in your Modbus layer.
 */
__attribute__((weak)) void ModbusTimer_FrameDoneCallback(void)
{
    /* User implements: copy rx_buffer → parse queue, set frame_ready flag, etc. */
}
