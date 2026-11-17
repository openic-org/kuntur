/**
 * @file  gpio_bench.c
 * @brief GPIO toggle benchmark — see gpio_bench.h for overview.
 *
 * Sweep: {0, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096} toggle-pairs.
 * At 32 MHz one HIGH+LOW pair ≈ 4–6 cycles ≈ 125–187 ns.
 * 4096 pairs ≈ 0.5–0.8 ms per StreamSendTask call.
 * At ~500 pkt/s that is ~25–40 % of CPU — enough to observe BLE degradation.
 */

#include "gpio_bench.h"
#include "app_common.h"
#include "stm32wb0x_hal.h"

/* PB3 — same pin as FPGA SCK in fpga_spi.c */
#define SCK_PORT   GPIOB
#define SCK_PIN    GPIO_PIN_3
#define SCK_HIGH() (SCK_PORT->BSRR = (uint32_t)SCK_PIN)
#define SCK_LOW()  (SCK_PORT->BSRR = (uint32_t)SCK_PIN << 16U)

static const uint32_t SWEEP[] = {0, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096};
#define SWEEP_LEN  (sizeof(SWEEP) / sizeof(SWEEP[0]))
#define STEP_MS    5000U

volatile uint32_t g_gpio_bench_loops = 0U;
static uint32_t   s_step_idx         = 0U;
static uint32_t   s_last_step_tick   = 0U;

void GPIO_BENCH_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = SCK_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SCK_PORT, &gpio);
    SCK_LOW();

    s_step_idx         = 0U;
    g_gpio_bench_loops = SWEEP[0];
    s_last_step_tick   = HAL_GetTick();

    APP_DBG_MSG("GPIO_BENCH: init  pin=PB3  steps=%u  step_s=%u\r\n",
                (unsigned)SWEEP_LEN, (unsigned)(STEP_MS / 1000U));
    APP_DBG_MSG("GPIO_BENCH step=0 loops=0\r\n");
}

void GPIO_BENCH_Work(void)
{
    /* Toggle PB3 for g_gpio_bench_loops HIGH/LOW pairs */
    volatile uint32_t n = g_gpio_bench_loops;
    while (n--)
    {
        SCK_HIGH();
        SCK_LOW();
    }

    /* Advance sweep — only while steps remain to avoid repeated "done" prints */
    if (s_step_idx < SWEEP_LEN)
    {
        uint32_t now = HAL_GetTick();
        if ((now - s_last_step_tick) >= STEP_MS)
        {
            s_last_step_tick = now;
            s_step_idx++;
            if (s_step_idx < SWEEP_LEN)
            {
                g_gpio_bench_loops = SWEEP[s_step_idx];
                APP_DBG_MSG("GPIO_BENCH step=%lu loops=%lu\r\n",
                            (unsigned long)s_step_idx,
                            (unsigned long)g_gpio_bench_loops);
            }
            else
            {
                APP_DBG_MSG("GPIO_BENCH done  final_loops=%lu\r\n",
                            (unsigned long)g_gpio_bench_loops);
            }
        }
    }
}
