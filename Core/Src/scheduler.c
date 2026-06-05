/**
 *
 * Copyright (C) 2026 HexaMix
 * SPDX-License-Identifier: GPL-3.0-only
 * Dual-licensed — see LICENSE for commercial use.
 *
 * scheduler.c
 *
 *  Created on: 4 Jun 2026
 *      Author: Osama
 */
#include "scheduler.h"
#include "stm32g0xx_hal.h"
#include "stm32g0xx_hal_iwdg.h"           /* CubeMX-generated: extern IWDG_HandleTypeDef hiwdg */

/* ------------------------------------------------------------------ */
/*  Private state                                                       */
/* ------------------------------------------------------------------ */
static SchedulerTask_t s_tasks[SCHEDULER_MAX_TASKS];
static uint8_t         s_task_count = 0U;

extern IWDG_HandleTypeDef hiwdg;

/* ------------------------------------------------------------------ */
void Scheduler_Init(void)
{
    s_task_count = 0U;
    for (uint8_t i = 0U; i < SCHEDULER_MAX_TASKS; i++) {
        s_tasks[i].func             = NULL;
        s_tasks[i].period_ms        = 0U;
        s_tasks[i].wdog_deadline_ms = 0U;
        s_tasks[i].last_tick        = 0U;
        s_tasks[i].last_run_tick    = 0U;
        s_tasks[i].enabled          = false;
    }
}

/* ------------------------------------------------------------------ */
bool Scheduler_AddTask(TaskFunc_t func, uint32_t period_ms, uint32_t wdog_deadline_ms)
{
    if (func == NULL || s_task_count >= SCHEDULER_MAX_TASKS) {
        return false;
    }

    uint32_t now = HAL_GetTick();
    SchedulerTask_t *t    = &s_tasks[s_task_count++];
    t->func               = func;
    t->period_ms          = period_ms;
    t->wdog_deadline_ms   = wdog_deadline_ms;
    t->last_tick          = now;
    t->last_run_tick      = now;   /* don't flag as stuck immediately */
    t->enabled            = true;

    return true;
}

/* ------------------------------------------------------------------ */
/*  Returns true if ALL wdog-guarded tasks have run within deadline    */
/* ------------------------------------------------------------------ */
bool Scheduler_IsHealthy(void)
{
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0U; i < s_task_count; i++) {
        const SchedulerTask_t *t = &s_tasks[i];

        if (!t->enabled || t->wdog_deadline_ms == 0U) {
            continue;   /* not guarded */
        }

        if ((now - t->last_run_tick) >= t->wdog_deadline_ms) {
            return false;   /* this task is stuck */
        }
    }
    return true;
}

/* ------------------------------------------------------------------ */
/*  Main scheduler loop — call this from while(1)                      */
/* ------------------------------------------------------------------ */
void Scheduler_Run(void)
{
    uint32_t now         = HAL_GetTick();
    bool     task_ran    = false;
    uint32_t min_wait_ms = UINT32_MAX;

    for (uint8_t i = 0U; i < s_task_count; i++) {
        SchedulerTask_t *t = &s_tasks[i];

        if (!t->enabled || t->func == NULL) {
            continue;
        }

        uint32_t elapsed = now - t->last_tick;

        if (elapsed >= t->period_ms) {
            t->last_tick     = now;
            t->last_run_tick = now;
            t->func();
            task_ran = true;
        } else {
            uint32_t remaining = t->period_ms - elapsed;
            if (remaining < min_wait_ms) {
                min_wait_ms = remaining;
            }
        }
    }

    /* ---- Watchdog refresh (only when all guarded tasks are healthy) ---- */
    if (Scheduler_IsHealthy()) {
        HAL_IWDG_Refresh(&hiwdg);
    }
    /* If unhealthy: IWDG will not be kicked → MCU resets → relays go to   */
    /* safe state via hardware default (pull-down on relay driver inputs).  */

    /* ---- Sleep if nothing is due ---- */
    /*  WFI: CPU halts, peripherals keep running.                           */
    /*  Woken by: SysTick (next 1 ms tick) OR any enabled UART/GPIO IRQ.   */
    if (!task_ran) {
        (void)min_wait_ms;   /* SysTick at 1 ms granularity handles wakeup */
        __WFI();
    }
}

/* ------------------------------------------------------------------ */
/*  Runtime helpers                                                     */
/* ------------------------------------------------------------------ */
static SchedulerTask_t* find_task(TaskFunc_t func)
{
    for (uint8_t i = 0U; i < s_task_count; i++) {
        if (s_tasks[i].func == func) return &s_tasks[i];
    }
    return NULL;
}

bool Scheduler_EnableTask(TaskFunc_t func)
{
    SchedulerTask_t *t = find_task(func);
    if (t == NULL) return false;
    t->last_tick     = HAL_GetTick();
    t->last_run_tick = HAL_GetTick();
    t->enabled       = true;
    return true;
}

bool Scheduler_DisableTask(TaskFunc_t func)
{
    SchedulerTask_t *t = find_task(func);
    if (t == NULL) return false;
    t->enabled = false;
    return true;
}

bool Scheduler_SetPeriod(TaskFunc_t func, uint32_t new_period_ms)
{
    SchedulerTask_t *t = find_task(func);
    if (t == NULL) return false;
    t->period_ms = new_period_ms;
    return true;
}
