#ifndef BSP_TASK_TIMING_H
#define BSP_TASK_TIMING_H

#include "main.h"
#include "cmsis_os2.h"

/* DWT is enabled once in main, before starting the scheduler.
 * Durations include interrupt and higher-priority task preemption.
 */
typedef struct
{
    uint32_t loop_count;
    uint32_t period_us;
    uint32_t max_period_us;
    uint32_t execution_us;
    uint32_t max_execution_us;
    uint32_t deadline_misses;
    float loop_hz;              /* Updated about once per second. */
    uint32_t start_cycle;
    uint32_t rate_cycle;
    uint32_t rate_count;
} TaskTiming_s;

static inline void TaskTimingBegin(volatile TaskTiming_s *timing)
{
    uint32_t now = DWT->CYCCNT;
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    if (timing->loop_count != 0U)
    {
        timing->period_us = (uint32_t)(now - timing->start_cycle) / cycles_per_us;
        if (timing->period_us > timing->max_period_us)
            timing->max_period_us = timing->period_us;
    }
    else
    {
        timing->rate_cycle = now;
        timing->rate_count = 1U;
    }
    timing->start_cycle = now;
    timing->loop_count++;
    if ((uint32_t)(now - timing->rate_cycle) >= SystemCoreClock)
    {
        timing->loop_hz = (float)(timing->loop_count - timing->rate_count) *
                         (float)SystemCoreClock / (float)(uint32_t)(now - timing->rate_cycle);
        timing->rate_cycle = now;
        timing->rate_count = timing->loop_count;
    }
}

static inline void TaskTimingEnd(volatile TaskTiming_s *timing)
{
    timing->execution_us = (uint32_t)(DWT->CYCCNT - timing->start_cycle) /
                           (SystemCoreClock / 1000000U);
    if (timing->execution_us > timing->max_execution_us)
        timing->max_execution_us = timing->execution_us;
}

/* One tick is 1 ms with the project's configTICK_RATE_HZ=1000.
 * On overrun, yield and resynchronize instead of starving lower tasks.
 */
static inline void TaskWaitNext1ms(uint32_t *next_tick, volatile TaskTiming_s *timing)
{
    *next_tick += 1U;
    if (osDelayUntil(*next_tick) != osOK)
    {
        timing->deadline_misses++;
        osDelay(1U);
        *next_tick = osKernelGetTickCount();
    }
}

#endif
