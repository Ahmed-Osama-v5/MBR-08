/*
 * scheduler.h
 *
 *  Created on: 4 Jun 2026
 *      Author: Osama
 */

#ifndef INC_SCHEDULER_H_
#define INC_SCHEDULER_H_

#include <stdint.h>
#include <stdbool.h>

#define SCHEDULER_MAX_TASKS  10U

typedef void (*TaskFunc_t)(void);

typedef struct {
    TaskFunc_t  func;               /* Task function pointer                */
    uint32_t    period_ms;          /* Nominal execution period (ms)        */
    uint32_t    wdog_deadline_ms;   /* Max allowed gap before IWDG not fed  */
    uint32_t    last_tick;          /* When task was last DUE               */
    uint32_t    last_run_tick;      /* When task last actually EXECUTED     */
    bool        enabled;
} SchedulerTask_t;

/* Core API */
void Scheduler_Init(void);

/**
 * @param func             Task function
 * @param period_ms        How often to call it
 * @param wdog_deadline_ms Max tolerated silence before considered stuck
 *                         (0 = exclude this task from watchdog guard)
 */
bool Scheduler_AddTask(TaskFunc_t func, uint32_t period_ms, uint32_t wdog_deadline_ms);

void Scheduler_Run(void);          /* Call in main while(1) loop           */
bool Scheduler_IsHealthy(void);    /* All guarded tasks running on time?   */

/* Runtime control */
bool Scheduler_EnableTask(TaskFunc_t func);
bool Scheduler_DisableTask(TaskFunc_t func);
bool Scheduler_SetPeriod(TaskFunc_t func, uint32_t new_period_ms);


#endif /* INC_SCHEDULER_H_ */
