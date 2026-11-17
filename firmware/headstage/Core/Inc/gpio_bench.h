/**
 * @file  gpio_bench.h
 * @brief GPIO toggle benchmark — measures BLE throughput impact of in-loop GPIO work.
 *
 * GPIO_BENCH_Work() is called between every BLE notification in StreamSendTask.
 * It toggles PB3 (FPGA SCK) for g_gpio_bench_loops HIGH/LOW pairs, simulating
 * the real FPGA SPI bit-bang workload on AHB GPIO.
 *
 * The loop count auto-sweeps every 5 s through:
 *   {0, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096}
 * One hex covers the full sweep — no reflash needed.
 * UART prints "GPIO_BENCH step=N loops=N" at each transition.
 *
 * NOTE: PB3 is also used as FPGA SCK in fpga_spi.c.  Do not call both
 * GPIO_BENCH_Init() and FPGA_SPI_Init() in the same firmware image.
 */

#ifndef GPIO_BENCH_H
#define GPIO_BENCH_H

#include <stdint.h>

/** Number of SCK HIGH+LOW toggle pairs per StreamSendTask iteration.
 *  Volatile so it can be overridden live via SWD without reflashing. */
extern volatile uint32_t g_gpio_bench_loops;

/**
 * @brief  Init PB3 as output and start the auto-sweep timer.
 *         Call once from APP_BLE_Init after STREAM_APP_Init.
 */
void GPIO_BENCH_Init(void);

/**
 * @brief  Hot path: toggle PB3 g_gpio_bench_loops times, then check sweep timer.
 *         Call after every successful STREAM_NotifyData + BLE_STACK_Tick() in
 *         StreamSendTask's tight loop.
 */
void GPIO_BENCH_Work(void);

#endif /* GPIO_BENCH_H */
