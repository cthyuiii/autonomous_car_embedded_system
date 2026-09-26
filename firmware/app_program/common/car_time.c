/** @file car_time.c
 *
 * @brief The kernel's clock, and the tick every control loop runs on.
 *
 * One cyclic handler serves every task that asks for ticks, because the
 * kernel has only CNF_MAX_CYCID of them.
 *
 * Owner: the team.
 */

#include "car_time.h"

#include <tk/tkernel.h>

#if CNF_TIMER_PERIOD != CAR_TIME_TICK_MSEC
#error "CAR_TIME_TICK_MSEC must equal the kernel tick, CNF_TIMER_PERIOD"
#endif

#define CAR_TIME_MAX_TASKS  6u

static ID                g_tasks[CAR_TIME_MAX_TASKS];
static volatile uint32_t g_task_count = 0u;
static ID                gh_cyclic    = 0;

static void tick_handler (void * p_exinf);

uint32_t car_time_msec (void)
{
    SYSTIM now = { 0, 0u };

    (void)tk_get_otm(&now);

    return (uint32_t)now.lo;   /* The low 32 bits, wrapping every 49 days */
}

car_status_t car_time_start_ticks (void)
{
    car_status_t status     = CAR_ERR_RANGE;
    bool         b_first    = false;
    ID           h_self     = tk_get_tid();
    UINT         imask      = 0u;

    DI(imask);

    if (g_task_count < CAR_TIME_MAX_TASKS)
    {
        b_first               = (0u == g_task_count);
        g_tasks[g_task_count] = h_self;
        g_task_count++;
        status                = CAR_OK;
    }

    EI(imask);

    if (b_first)
    {
        /* Casts: the kernel takes a handler as FP whatever its arguments,
         * and RELTIM and TMO hold a tick of 10 ms. */
        T_CCYC const ccyc =
        {
            .exinf  = NULL,
            .cycatr = TA_HLNG | TA_STA,
            .cychdr = (FP)tick_handler,
            .cyctim = (RELTIM)CAR_TIME_TICK_MSEC,
            .cycphs = (RELTIM)CAR_TIME_TICK_MSEC,
        };

        gh_cyclic = tk_cre_cyc(&ccyc);
        status    = (gh_cyclic > 0) ? CAR_OK : CAR_ERR_HARDWARE;
    }

    return status;
}

void car_time_wait_tick (void)
{
    (void)tk_slp_tsk((TMO)(2u * CAR_TIME_TICK_MSEC));   /* 20 ms */
    (void)tk_can_wup(TSK_SELF);
}

/**
 * @brief Cyclic handler: wake every registered task, once per tick.
 *
 * @param[in] p_exinf Unused.
 */
static void tick_handler (void * p_exinf)
{
    uint32_t index = 0u;

    (void)p_exinf;

    for (index = 0u; index < g_task_count; index++)
    {
        (void)tk_wup_tsk(g_tasks[index]);
    }
}

/*** end of file ***/
