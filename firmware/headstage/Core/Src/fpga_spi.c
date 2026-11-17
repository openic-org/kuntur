/**
 * @file  fpga_spi.c
 * @brief FPGA data acquisition — synchronous bit-bang SPI pull.
 *
 * Why bit-bang instead of SPI3 hardware:
 *   The STM32WB09 BLE radio gates APB1 clocks (SPI3, USART1) during radio
 *   windows.  A load/store to an APB1 register while gated stalls the AHB
 *   bus indefinitely on Cortex-M0+.  GPIOA/B are on AHB (0x48000000 /
 *   0x48100000, RCC_AHBENR_GPIOAEN/GPIOBEN) — not on any APB, and never
 *   gated.  (Earlier comments here said "APB0"; that is wrong. APB0 at
 *   0x40000000 carries RTC, SYSCFG, TIM2/16/17 and WDG, no GPIO.)
 *
 * Architecture: synchronous pull inside the BLE tight loop.
 *   StreamSendTask calls FPGA_SPI_ReadSamples(pkt.samples, 59) once per
 *   BLE notification.  59 pairs × 2 transfers × ~5 µs (O2) ≈ 590 µs per
 *   packet, well within the 1.97 ms data interval at 30 kSPS.
 *   The FPGA buffers samples in a FIFO; no timer or ISR needed on the MCU.
 */

#include "fpga_spi.h"
#include "app_common.h"
#include "stm32wb0x_hal.h"

/* ── Pin macros (AHB GPIO — never gated by BLE radio) ────────────────────── */

#define SCK_PORT   GPIOB
#define SCK_PIN    GPIO_PIN_3    /* PB3  */
#define MISO_PORT  GPIOA
#define MISO_PIN   GPIO_PIN_8    /* PA8  */
#define MOSI_PORT  GPIOA
#define MOSI_PIN   GPIO_PIN_11   /* PA11 */
#define NSS_PORT   GPIOA
#define NSS_PIN    GPIO_PIN_9    /* PA9  */

#define SCK_HIGH()   (SCK_PORT->BSRR  = (uint32_t)SCK_PIN)
#define SCK_LOW()    (SCK_PORT->BSRR  = (uint32_t)SCK_PIN  << 16U)
#define MOSI_HIGH()  (MOSI_PORT->BSRR = (uint32_t)MOSI_PIN)
#define MOSI_LOW()   (MOSI_PORT->BSRR = (uint32_t)MOSI_PIN << 16U)
#define NSS_LOW()    (NSS_PORT->BSRR  = (uint32_t)NSS_PIN  << 16U)
#define NSS_HIGH()   (NSS_PORT->BSRR  = (uint32_t)NSS_PIN)
#define MISO_READ()  ((MISO_PORT->IDR >> 8U) & 1U)   /* PA8 = bit 8 */

/* ── Bit-bang transfer — SPI mode 0 (CPOL=0, CPHA=0), MSB first ─────────── */

/* Two NOPs after SCK_HIGH give the FPGA MISO output ~31 ns propagation margin.
 * NSS_LOW setup NOPs: history below predates 2026-07-29's hardware fix —
 * FPGA clk now derives from HSE (32 MHz) via MCU MCO3/PB14 instead of its own
 * free-running crystal, so the FPGA clk (still 44.55 MHz, mypll retuned) and
 * SYSCLK (64 MHz) are both locked to the same reference. Phase between them
 * should now be fixed, not continuously drifting — so the worst-case-async
 * margin math below (~45-67 ns needed against a 22.45 ns FPGA clock period)
 * no longer directly applies; the real safe minimum has to come from data.
 * First post-fix recording at 12/48 (unchanged from before the hardware fix)
 * still showed SKP at ~1/s, same ~54-60 magnitude signature as before, just
 * less frequent (vega_20260729_030823.csv) — stepping back down to 6/24
 * (matches an already-tested async-clock config, SKP was 2.48/s there) to
 * see whether the residual SKP is still margin-sensitive post-fix, or is a
 * fixed, margin-independent floor now.
 * Progression before the hardware fix: 1 NOP (baseline, short by 3-4x) -> 6
 * (SKP -22%) -> 12 (combined with the other gap, SKP -56%) -> 24/96
 * regressed (added enough overhead to drop drain rate below FPGA fill rate,
 * 30303->27441 SPS, trading rare ~54-sample SKP for chronic small-magnitude
 * overflow SKP, median jump 3 — vega_20260728_122239.csv) -> reverted to
 * 12/48 as the best point pre-fix (SKP -80% vs baseline, underruns -73%).
 * Both delays are important after FPGA resynthesis that can change routing delays. */
#define SPI_BIT(n) \
    if (tx & (1U << (n))) MOSI_HIGH(); else MOSI_LOW(); \
    SCK_HIGH(); \
    __NOP(); __NOP(); \
    if (MISO_READ()) rx |= (uint16_t)(1U << (n)); \
    SCK_LOW()

__attribute__((optimize("O2")))
static uint16_t spi_bb_transfer(uint16_t tx)
{
    uint16_t rx = 0U;
    NSS_LOW();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    SPI_BIT(15); SPI_BIT(14); SPI_BIT(13); SPI_BIT(12);
    SPI_BIT(11); SPI_BIT(10); SPI_BIT(9);  SPI_BIT(8);
    SPI_BIT(7);  SPI_BIT(6);  SPI_BIT(5);  SPI_BIT(4);
    SPI_BIT(3);  SPI_BIT(2);  SPI_BIT(1);  SPI_BIT(0);
    NSS_HIGH();
    /* Post-2026-07-29 hardware fix (FPGA clk now shares a 32 MHz reference
     * with SYSCLK via MCU MCO3/PB14, mypll retuned to still output 44.55 MHz)
     * did NOT make SKP margin-independent: halving both gaps to 6/24 raised
     * SKP from 1.03/s to 1.85/s in a 60 s recording (vega_20260729_031755.csv)
     * — still clearly margin-sensitive, just less severe than pre-fix at the
     * same margin (was 2.48/s async). Likely explanation: the FPGA's mypll
     * and the MCU's RC64MPLL are separate PLL circuits independently locked
     * to the shared reference — frequency-locked now, but each still has its
     * own loop dynamics/phase jitter, so the race is reduced, not eliminated.
     * Reverted to 48 — still the best confirmed point, pre- and post-fix.
     * Pre-fix progression: 12 NOPs (baseline, ~1.6-2x margin) -> 24 (SKP
     * -56% combined with the other gap's fix) -> 48 (SKP -43% more,
     * underruns -73%) -> 96 regressed (dropped drain rate below FPGA fill
     * rate, 30303->27441 SPS, chronic small-magnitude overflow SKP instead
     * of the rare ~54-sample kind — vega_20260728_122239.csv). */
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    return rx;
}
#undef SPI_BIT

/* ── SPI0 command words ──────────────────────────────────────────────────────
 * Wire format: {opcode[15:14], tag[13:8], data[7:0]}.
 * See docs/interfaces/channel-selection-control-plane.md section 1.
 *
 * Tag 1 uniformly means "load addr_reg from the data byte", on both REG_WRITE
 * and REG_READ. The tag field never carries an address — the address travels
 * in the data byte.
 */

#define REG_WRITE_OPCODE   0x1U
#define REG_READ_OPCODE    0x2U
#define NOP_OPCODE         0x3U

#define WR_TAG_ADDR        0x01U   /* transfer 1: addr_reg  <= data          */
#define WR_TAG_HIGH        0x02U   /* transfer 2: staged_h  <= data          */
#define WR_TAG_COMMIT      0x03U   /* transfer 3: ram[addr_reg] <= {h, data} */
#define RD_TAG_ADDR        0x01U   /* REG_READ carries tag 1 + its address   */

/* Regbank RAM word addresses. Map defined in intan.vh
 * (RB_CONFIG_BASE 0 / RB_SAMPLING_BASE 48 / RB_CTRL_BASE 192). */
#define REG_ADDR_CH_A            196U
#define REG_ADDR_CH_B            197U
#define REG_ADDR_STREAM_ENABLE   228U   /* RB_CTRL_BASE + 36, bit 0 gates fifo_wen */
#define REG_ADDR_DATA_SOURCE     229U   /* RB_CTRL_BASE + 37, bits [1:0]: 0=real, 1=ramp */

/* RHD sampling-command table: one 16-bit RHD command word per slot.
 * Slot k's answer is observed at ch_a[5:0] = k (see fpga-diagnostic-access.md
 * section 1.2). Slot 32 is the alternate-command placeholder. */
#define REG_SAMPLING_BASE        48U    /* RB_SAMPLING_BASE */
#define REG_SAMPLING_MAX_SLOT    32U    /* RB_SAMPLING_MAX  */

static inline uint16_t write_word(uint8_t tag, uint8_t data)
{
    return (uint16_t)((REG_WRITE_OPCODE << 14) | ((uint16_t)(tag & 0x3FU) << 8) | data);
}

static inline uint16_t read_word(uint8_t addr)
{
    return (uint16_t)((REG_READ_OPCODE << 14) | ((uint16_t)RD_TAG_ADDR << 8) | addr);
}

static inline uint16_t nop_word(void)
{
    return (uint16_t)(NOP_OPCODE << 14);
}

/* One 16-bit register write = three tagged transfers. A mismatched tag at any
 * stage aborts the sequence and leaves the target word untouched, so the three
 * must be issued together and in order. Transfer 2 cannot be skipped even for
 * an 8-bit value — omitting it aborts at the tag check (verified in
 * kuntur_tb.sv T6 case 6a). */
static void reg_write16(uint8_t addr, uint16_t value)
{
    (void)spi_bb_transfer(write_word(WR_TAG_ADDR,   addr));
    (void)spi_bb_transfer(write_word(WR_TAG_HIGH,   (uint8_t)(value >> 8)));
    (void)spi_bb_transfer(write_word(WR_TAG_COMMIT, (uint8_t)(value & 0xFFU)));
}

/* One 16-bit register read = two transfers. REG_READ is self-addressing (it
 * carries tag 1 plus its own address), and the value lands on the FOLLOWING
 * transfer's MISO — so the NOP is a carrier, not padding, and cannot be
 * dropped. Reads do not auto-increment, so back-to-back reads of different
 * words are independent (kuntur_tb.sv T3). */
static uint16_t reg_read16(uint8_t addr)
{
    (void)spi_bb_transfer(read_word(addr));   /* stale — answers whatever preceded */
    return spi_bb_transfer(nop_word());       /* word `addr`'s value               */
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void FPGA_SPI_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    gpio.Pin   = SCK_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SCK_PORT, &gpio);
    SCK_LOW();

    gpio.Pin  = MISO_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(MISO_PORT, &gpio);

    gpio.Pin   = MOSI_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MOSI_PORT, &gpio);

    gpio.Pin   = NSS_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(NSS_PORT, &gpio);
    NSS_HIGH();

    /* No priming transfer. Removed 2026-08-11 with A.1.1g.
     *
     * It used to discard one transfer here so that ReadSamples()'s 2*n
     * transfers stayed aligned with the FPGA's one-request-deep output
     * pipeline. That left the transfer count permanently odd, i.e. every
     * ReadSamples() call ended with a FIFO_POP pair half-open, dangling into
     * whatever came next. Harmless while nothing checked; under the A.1.1g FSM
     * a POP whose partner is not a POP aborts AND CONSUMES the offending
     * transfer — so the first transfer of the next command (in practice
     * STOP_STREAMING's tag-1 write) would be silently eaten, and the whole
     * write sequence would then abort tag-by-tag.
     *
     * ReadSamples() now handles the pipeline itself and closes every pair it
     * opens, so there is nothing to prime and no alignment to maintain across
     * calls. */

    APP_DBG_MSG("FPGA_SPI: init done (sync bit-bang pull, O2)\r\n");
}

/* Read n raw 16-bit words directly from SPI and print them via UART.
 *   All 0xFFFF → MISO stuck high (no FPGA drive or wrong SPI mode)
 *   All 0x0000 → MISO stuck low
 *   All 0x8000 → FPGA returning empty-FIFO sentinel
 *   Varying    → FPGA is shifting out data */
void FPGA_SPI_DebugDump(uint8_t n)
{
    APP_DBG_MSG("FPGA_SPI raw (%u words):\r\n", n);
    for (uint8_t i = 0U; i < n; i++)
    {
        uint16_t raw = spi_bb_transfer(FPGA_STREAM_CMD);
        APP_DBG_MSG("  [%2u] 0x%04X  (%d)\r\n", i, raw, (int16_t)raw);
    }
    /* n raw POPs may leave a pair half-open (odd n). Close it with a NOP so
     * this diagnostic cannot poison the next command — under the A.1.1g FSM a
     * dangling POP consumes the following transfer via the pair-check abort. */
    (void)spi_bb_transfer(nop_word());
    APP_DBG_MSG("done\r\n");
}

/* Read n_pairs using the real pairing logic and auto-detect FIFO errors.
 *
 * ch1_offset: expected value of (ch1 - ch0) for every pair.
 *   FPGA counter mode: CH0 = N, CH1 = N + 1000  →  pass 1000
 *   Pass 0 to skip the offset check.
 *
 * Flags printed per row:
 *   DUP   ch0 same as previous pair   — FIFO read pointer did not advance
 *   SKP   ch0 did not increment by 1  — samples dropped or pointer jumped
 *   OFF   ch1 - ch0 ≠ ch1_offset      — channel swap or pairing misalignment
 *   OK    no errors
 *
 * Summary line at the end counts total errors of each type. */
void FPGA_SPI_DebugDumpPairs(uint8_t n_pairs, int16_t ch1_offset)
{
    int16_t samples[2 * 32];  /* max 32 pairs */
    if (n_pairs > 32U) n_pairs = 32U;

    FPGA_SPI_ReadSamples(samples, n_pairs);

    APP_DBG_MSG("FPGA_SPI pairs (%u)  [expected ch1 = ch0 + %d]\r\n",
                n_pairs, (int)ch1_offset);

    uint32_t n_dup = 0U, n_skp = 0U, n_off = 0U;
    int16_t  prev_c0 = samples[0];

    for (uint8_t i = 0U; i < n_pairs; i++)
    {
        int16_t c0   = samples[i * 2U];
        int16_t c1   = samples[i * 2U + 1U];
        int     diff = (int)c1 - (int)c0;
        int     step = (int)c0 - (int)prev_c0;

        /* Classify */
        uint8_t is_dup = (i > 0U) && (step == 0);
        uint8_t is_skp = (i > 0U) && (step != 0) && (step != 1);
        uint8_t is_off = (ch1_offset != 0) && (diff != (int)ch1_offset);
        if (is_dup) n_dup++;
        if (is_skp) n_skp++;
        if (is_off) n_off++;

        /* Build compact tag */
        char tag[12];
        uint8_t t = 0U;
        if (is_dup) { tag[t++]='D'; tag[t++]='U'; tag[t++]='P'; }
        else if (is_skp) { tag[t++]='S'; tag[t++]='K'; tag[t++]='P'; }
        if (is_off) {
            if (t > 0U) tag[t++] = '|';
            tag[t++]='O'; tag[t++]='F'; tag[t++]='F';
        }
        if (t == 0U) { tag[t++]='O'; tag[t++]='K'; }
        tag[t] = '\0';

        if (i == 0U)
            APP_DBG_MSG("  [%2u] ch0=%6d  ch1=%6d  diff=%5d  step=  --  %s\r\n",
                        i, c0, c1, diff, tag);
        else
            APP_DBG_MSG("  [%2u] ch0=%6d  ch1=%6d  diff=%5d  step=%+4d  %s\r\n",
                        i, c0, c1, diff, step, tag);

        prev_c0 = c0;
    }

    APP_DBG_MSG("summary: dup=%u  skp=%u  off=%u  / %u pairs\r\n",
                n_dup, n_skp, n_off, (uint32_t)n_pairs);
    APP_DBG_MSG("done\r\n");
}

/* ── Channel-selection control plane ────────────────────────────────────────
 * See docs/interfaces/channel-selection-control-plane.md sections 1 and 1a.
 */

/* raw_code = insert a zero bit between friendly_channel's bit 4 and bit 5.
 * Every value 0-127 maps to a real, distinct amplifier channel (spec 1a). */
uint8_t FPGA_SPI_ChannelToRaw(uint8_t friendly_channel)
{
    return (uint8_t)(((friendly_channel & 0x60U) << 1) | (friendly_channel & 0x1FU));
}

/* Inverse of FPGA_SPI_ChannelToRaw() — undoes the inserted zero bit (spec 4.2). */
uint8_t FPGA_SPI_RawToChannel(uint8_t raw_code)
{
    return (uint8_t)(((raw_code >> 1) & 0x60U) | (raw_code & 0x1FU));
}

/* Two 3-transfer tagged write sequences, 6 transfers. ch_a/ch_b are 8-bit
 * values in a 16-bit word, so the staged high byte is always 0. */
void FPGA_SPI_SetChannels(uint8_t ch_a_raw, uint8_t ch_b_raw)
{
    reg_write16(REG_ADDR_CH_A, (uint16_t)ch_a_raw);
    reg_write16(REG_ADDR_CH_B, (uint16_t)ch_b_raw);
}

/* Read back ch_a/ch_b — spec section 4.1. REG_READ is self-addressing, so no
 * preceding write sequence is needed; 3 transfers, down from 4 (the old 4th
 * was even-count padding for the retired pairing invariant).
 * One-transfer-deep pipeline: a REG_READ's value appears on the NEXT
 * transfer's MISO, not the same one. */
void FPGA_SPI_ReadChannels(uint8_t *ch_a_raw, uint8_t *ch_b_raw)
{
    (void)spi_bb_transfer(read_word(REG_ADDR_CH_A));            /* stale — discard  */
    uint16_t rx_a = spi_bb_transfer(read_word(REG_ADDR_CH_B));  /* word 196's value */
    uint16_t rx_b = spi_bb_transfer(nop_word());                /* word 197's value */

    *ch_a_raw = (uint8_t)(rx_a & 0xFFU);
    *ch_b_raw = (uint8_t)(rx_b & 0xFFU);
}

/* Gate FPGA sample ingestion at the source — see spec section 5 (revised)
 * and the docs comment in fpga_spi.h. One 3-transfer tagged write; the old
 * trailing NOP is gone with the even-count rule. */
void FPGA_SPI_SetStreamEnable(uint8_t enable)
{
    reg_write16(REG_ADDR_STREAM_ENABLE, enable ? 1U : 0U);
}

/* Read back RAM word 228 (stream_enable) — same one-transfer-deep pipeline as
 * FPGA_SPI_ReadChannels(). 2 transfers. */
uint8_t FPGA_SPI_ReadStreamEnable(void)
{
    return (uint8_t)(reg_read16(REG_ADDR_STREAM_ENABLE) & 0x1U);
}

/* ── Generic register console ────────────────────────────────────────────────
 * docs/interfaces/fpga-diagnostic-access.md section 3.1. Raw, unvalidated
 * access to all 256 regbank words — the diagnostic path that drives the A.1.1
 * verification ladder. Deliberately separate from the operator path
 * (FPGA_SPI_SetChannels and friends above), which keeps its friendly-index
 * mapping and range validation; a diagnostic path must not be able to loosen
 * those.
 *
 * No write protection, matching the RTL (PLAN.md A.1.1g: "partial protection
 * would give a false sense of safety"). These can corrupt the RHD config table
 * and leave the sampling cycle issuing nonsense until the FPGA is reset. The
 * gating is at Tunki (spec section 2.2), not here.
 *
 * Same calling-context rules as every other function in this section: never
 * from an ISR, never interleaved with an in-progress FPGA_SPI_ReadSamples().
 * Structurally guaranteed — these run only from StreamSendTask's stopped
 * branch, and 0xFFF1 rejects them unless streaming is already stopped.
 */

void FPGA_SPI_RegWrite16(uint8_t addr, uint16_t value)
{
    reg_write16(addr, value);
}

uint16_t FPGA_SPI_RegRead16(uint8_t addr)
{
    return reg_read16(addr);
}

/* Inject an RHD command word into sampling slot `slot`. Out-of-range slots
 * issue NO transfers at all — a partial write would abort at the tag check and
 * leave the FSM fine, but silently writing nothing is worse than visibly
 * writing nothing, so the caller gets a hard false. */
uint8_t FPGA_SPI_SetSamplingSlot(uint8_t slot, uint16_t cmd)
{
    if (slot > REG_SAMPLING_MAX_SLOT)
    {
        return 0U;
    }
    reg_write16((uint8_t)(REG_SAMPLING_BASE + slot), cmd);
    return 1U;
}

/* Select what the FPGA streams — real RHD data or the synthetic ramp.
 * RAM word 229 bits [1:0]; reset default 0 (real data), so the test pattern is
 * always an explicit opt-in. See fpga-diagnostic-access.md section 1.1 for why
 * that direction and not the other. */
void FPGA_SPI_SetDataSource(uint8_t source)
{
    reg_write16(REG_ADDR_DATA_SOURCE, (uint16_t)(source & 0x3U));
}

uint8_t FPGA_SPI_ReadDataSource(void)
{
    return (uint8_t)(reg_read16(REG_ADDR_DATA_SOURCE) & 0x3U);
}

/* Read n_pairs sample pairs from the FPGA FIFO into samples[].
 * Output layout: [ch0_0, ch1_0, ch0_1, ch1_1, ...] — matches BLE packet.
 * FPGA outputs 0x8000 on both channels when its FIFO is empty; the value
 * passes through to the BLE stream as an in-band underrun sentinel for the
 * PC app to detect.  Never blocks.
 *
 * Transfer structure (rewritten 2026-08-11 for A.1.1g — see fpga_spi.h):
 *
 *   T1        FIFO_POP   MISO = response to whatever preceded this call
 *                        (undefined after reset, previous call's last ChB
 *                        otherwise) — DISCARDED
 *   T2..T2n   FIFO_POP   MISO = samples[0 .. 2n-2]
 *   T2n+1     NOP        MISO = samples[2n-1], the final ChB
 *
 * 2n+1 transfers, n FIFO entries consumed (the NOP pops nothing). Pairs are
 * (T1,T2), (T3,T4) ... (T2n-1,T2n): every pair opened is closed inside this
 * call, so the FPGA FSM is back at its decode state on return and any command
 * may follow. This is what replaced FPGA_SPI_Init()'s priming transfer.
 *
 * Cost of the extra transfer: 1 in 119 at n_pairs=59, ~5 µs on ~590 µs. */
__attribute__((optimize("O2")))
void FPGA_SPI_ReadSamples(int16_t *samples, uint8_t n_pairs)
{
    if (n_pairs == 0U) return;

    const uint16_t n_words = (uint16_t)n_pairs * 2U;

    (void)spi_bb_transfer(FPGA_STREAM_CMD);         /* T1 — response discarded */

    for (uint16_t i = 1U; i < n_words; i++)
        samples[i - 1U] = (int16_t)spi_bb_transfer(FPGA_STREAM_CMD);

    samples[n_words - 1U] = (int16_t)spi_bb_transfer(nop_word());
}
