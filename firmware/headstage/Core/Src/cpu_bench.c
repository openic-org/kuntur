/**
 * @file  cpu_bench.c
 * @brief CPU burn benchmark — measures BLE throughput vs. task CPU load.
 *
 * A VTIMER fires at the same period as StreamSendTask (~1300 Hz).  Each fire
 * schedules BurnTask at PRIO_1.  BurnTask spins for g_burn_loops NOPs, then
 * checks two HAL_GetTick() gates:
 *   - every 1 s: print "BENCH t=N loops=N notifs/s=N"
 *   - every 5 s: advance g_burn_loops to the next sweep value
 *
 * Sweep: [0, 100, 300, 1000, 3000, 10000, 30000, 100000] iterations
 * At 64 MHz, 1 NOP ≈ 15.6 ns, so 30000 loops ≈ 468 µs per invocation.
 * At 1300 Hz that is ~61 % CPU — should produce visible BLE degradation.
 *
 * Capture output:   cat /dev/ttyACM0 | tee bench.log
 * Plot:             python3 software/tunki/plot_bench.py bench.log
 */

#include "cpu_bench.h"
#include "app_conf.h"
#include "app_common.h"
#include "stream_app.h"
#include "stm32_seq.h"
#include "stm32wb0x_hal_radio_timer.h"

/* ── Sweep configuration ─────────────────────────────────────────────────── */

static const uint32_t SWEEP[] = {0, 100, 300, 1000, 3000, 10000, 30000, 100000};
#define SWEEP_LEN       (sizeof(SWEEP) / sizeof(SWEEP[0]))
#define BENCH_PERIOD_MS  2U    /* VTIMER period — same as StreamSendTask    */
#define PRINT_INTERVAL   1000U /* ms between BENCH prints                   */
#define STEP_INTERVAL    5000U /* ms per sweep step                         */

/* ── State ───────────────────────────────────────────────────────────────── */

volatile uint32_t g_burn_loops = 0U;

static VTIMER_HandleType s_burn_timer;
static uint32_t          s_step_idx        = 0U;
static uint32_t          s_last_print_tick = 0U;
static uint32_t          s_last_step_tick  = 0U;
static uint32_t          s_notifs_prev     = 0U;
static uint32_t          s_bench_t         = 0U;

/* ── VTIMER callback — fires at ~1300 Hz ─────────────────────────────────── */

static void BurnTimerCb(void *p)
{
    (void)p;
    UTIL_SEQ_SetTask(1UL << CFG_TASK_BURN_ID, CFG_SEQ_PRIO_1);
    HAL_RADIO_TIMER_StartVirtualTimer(&s_burn_timer, BENCH_PERIOD_MS);
}

/* ── Burn task — runs in UTIL_SEQ task context ───────────────────────────── */

static void BurnTask(void)
{
    /* Spin for g_burn_loops iterations — simulates FPGA SPI work on AHB GPIO */
    volatile uint32_t n = g_burn_loops;
    while (n--) { __NOP(); }

    uint32_t now = HAL_GetTick();

    /* Print every second */
    if ((now - s_last_print_tick) >= PRINT_INTERVAL)
    {
        uint32_t total  = STREAM_APP_GetNotifyTotal();
        uint32_t delta  = total - s_notifs_prev;
        s_notifs_prev   = total;
        s_bench_t++;
        APP_DBG_MSG("BENCH t=%lu loops=%lu notifs/s=%lu\r\n",
                    (unsigned long)s_bench_t,
                    (unsigned long)g_burn_loops,
                    (unsigned long)delta);
        s_last_print_tick = now;
    }

    /* Advance sweep every 5 seconds */
    if ((now - s_last_step_tick) >= STEP_INTERVAL)
    {
        s_step_idx++;
        if (s_step_idx < SWEEP_LEN)
        {
            g_burn_loops = SWEEP[s_step_idx];
            APP_DBG_MSG("BENCH step=%lu loops=%lu\r\n",
                        (unsigned long)s_step_idx,
                        (unsigned long)g_burn_loops);
        }
        s_last_step_tick = now;
    }
}

/* ── Public init — call from APP_BLE_Init after STREAM_APP_Init ──────────── */

void CPU_BENCH_Init(void)
{
    UTIL_SEQ_RegTask(1UL << CFG_TASK_BURN_ID, UTIL_SEQ_RFU, BurnTask);

    uint32_t now      = HAL_GetTick();
    s_last_print_tick = now;
    s_last_step_tick  = now;
    s_notifs_prev     = 0U;
    g_burn_loops      = SWEEP[0];

    s_burn_timer.callback = BurnTimerCb;
    HAL_RADIO_TIMER_StartVirtualTimer(&s_burn_timer, BENCH_PERIOD_MS);

    APP_DBG_MSG("CPU_BENCH: started — %u steps x %u s, period=%u\r\n",
                (unsigned)SWEEP_LEN, (unsigned)(STEP_INTERVAL / 1000U),
                (unsigned)BENCH_PERIOD_MS);
}
