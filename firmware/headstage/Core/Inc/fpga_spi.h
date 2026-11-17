#ifndef FPGA_SPI_H
#define FPGA_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ── Hardware (bit-bang SPI, all pins on AHB GPIO — never gated by radio) ──
 *   SCK   PB3   GPIO output
 *   MISO  PA8   GPIO input
 *   MOSI  PA11  GPIO output
 *   NSS   PA9   GPIO output (active low)
 *
 * Protocol: synchronous pull.
 *   FPGA_SPI_ReadSamples() is called once per BLE packet inside the
 *   StreamSendTask tight loop.  The FPGA must buffer samples in an internal
 *   FIFO (≥256 pairs recommended at 30 kSPS / 7.5 ms CI).
 *   Words alternate ch0 / ch1; SPI mode 0 (CPOL=0, CPHA=0), MSB first.
 *
 * SPI0 command protocol: see
 * docs/interfaces/channel-selection-control-plane.md section 1.
 * Rewritten 2026-08-11 for A.1.1g — every function below changed transfer
 * count and/or addressing. Two properties the whole file now depends on:
 *
 *   1. RESPONSE PIPELINE, one transfer deep. A command's result appears on
 *      the NEXT transfer's MISO, never the same one. True for FIFO_POP and
 *      REG_READ alike.
 *   2. NO DANGLING SEQUENCES. Every function leaves the FPGA FSM at its
 *      decode state, so any function may follow any other. In particular
 *      FIFO_POP pairs are never left half-open across a call boundary — the
 *      A.1.1g FSM aborts on a POP whose partner is not a POP, and the abort
 *      CONSUMES the offending transfer, which would silently eat the first
 *      transfer of whatever command came next.
 */

/* TX word during a sample-pop transfer (FPGA ignores contents, but bits
 * [15:14] are also the SPI0 opcode once the RTL's 4-way decode lands —
 * 0x2525 has top bits 00 (FIFO_POP), unlike the old 0xA5A5 (top bits 10,
 * which collides with the new REG_READ opcode). Coordinated MCU+RTL change:
 * this MCU-side value is safe to flash alone under the OLD (pre-4-way) RTL
 * (still just "not opcode 01" = pop either way) — the unsafe direction is
 * flashing the NEW RTL without this fix, which would misread every
 * streaming pop as a register read. See
 * docs/interfaces/channel-selection-control-plane.md section 4.1. */
#define FPGA_STREAM_CMD   0x2525U

void FPGA_SPI_Init(void);

/* Read n_pairs sample pairs. Issues 2*n_pairs FIFO_POP transfers plus one
 * trailing NOP carrier = 2*n_pairs + 1 transfers, and consumes exactly
 * n_pairs FIFO entries. The leading transfer's MISO is discarded (it carries
 * the response to whatever preceded the call); the trailing NOP clocks out the
 * final ChB. Self-contained: no priming transfer at init, no pair left open.
 * Never blocks. */
void FPGA_SPI_ReadSamples(int16_t *samples, uint8_t n_pairs);
void FPGA_SPI_DebugDump(uint8_t n);            /* n raw words, printed via UART */
void FPGA_SPI_DebugDumpPairs(uint8_t n_pairs, int16_t ch1_offset); /* n pairs via ReadSamples; ch1_offset=expected(ch1-ch0), 0=skip check */

/* ── Channel-selection control plane ────────────────────────────────────────
 * See docs/interfaces/channel-selection-control-plane.md sections 1 and 1a.
 */

/* Friendly channel index (0-127) -> raw FPGA regbank code (ch_a/ch_b encoding).
 * Single source of truth for this mapping; callers above the FPGA boundary
 * (BLE command handler, etc.) must never construct raw codes themselves. */
uint8_t FPGA_SPI_ChannelToRaw(uint8_t friendly_channel);

/* Write ch_a/ch_b — RAM words 196/197 — as two 3-transfer tagged write
 * sequences (6 transfers total). Takes raw FPGA codes; call
 * FPGA_SPI_ChannelToRaw() first.
 *
 * Calling context: BLE event-handler context only, never an ISR, and never
 * interleaved with an in-progress FPGA_SPI_ReadSamples(). As built this is
 * structural — SET_CHANNELS runs only while streaming is explicitly stopped
 * (spec section 5).
 *
 * The old "must be an even-numbered batch" rule is GONE — the A.1.1g FSM
 * drives the ChA/ChB phase explicitly per state, so register traffic cannot
 * disturb it. Confirmed in simulation 2026-08-11 (kuntur_tb.sv T5: pairing
 * held across a deliberately-odd 3-transfer write). This function is 6
 * transfers, an even count, but no longer needs to be. */
void FPGA_SPI_SetChannels(uint8_t ch_a_raw, uint8_t ch_b_raw);

/* Raw FPGA regbank code -> friendly channel index (0-127). Inverse of
 * FPGA_SPI_ChannelToRaw() — undoes the inserted zero bit. See
 * docs/interfaces/channel-selection-control-plane.md section 4.2. */
uint8_t FPGA_SPI_RawToChannel(uint8_t raw_code);

/* Read back ch_a/ch_b — RAM words 196/197 — in 3 transfers:
 *   REG_READ(196), REG_READ(197), NOP
 * REG_READ is self-addressing (carries tag 1 plus its own address), so no
 * preceding write sequence is needed. One-transfer-deep pipeline: word 196's
 * value lands on transfer 2, word 197's on transfer 3.
 * Returns raw FPGA codes — convert with FPGA_SPI_RawToChannel() before
 * exposing to BLE/Tunki. Same calling-context rules as FPGA_SPI_SetChannels().
 * See docs/interfaces/channel-selection-control-plane.md section 4.1. */
void FPGA_SPI_ReadChannels(uint8_t *ch_a_raw, uint8_t *ch_b_raw);

/* Gate FPGA sample ingestion at the source — RAM word 228, bit 0. See
 * docs/interfaces/channel-selection-control-plane.md section 5 (revised).
 * When disabled, ch_sel's fifo_wen is forced low regardless of the
 * generator/AFE, so fifo0 accumulates nothing while off — this is what makes
 * the STOP_STREAMING flush bounded/fast instead of racing a continuous
 * 30 kSPS producer. One 3-transfer tagged write sequence. */
void FPGA_SPI_SetStreamEnable(uint8_t enable);

/* Read back RAM word 228 (stream_enable) — returns bit 0 (1=ingesting,
 * 0=gated off). 2 transfers: REG_READ(228), NOP.
 * Same calling-context rules as FPGA_SPI_ReadChannels(). */
uint8_t FPGA_SPI_ReadStreamEnable(void);

/* ── Generic register console ────────────────────────────────────────────────
 * docs/interfaces/fpga-diagnostic-access.md section 3.1. The diagnostic path
 * behind the A.1.1 verification ladder: raw, unvalidated access to any of the
 * 256 regbank words, reaching what the operator path deliberately cannot —
 * sampling slot 32 (unreachable through the friendly 0-127 channel mapping,
 * yet where every rung reads its answer), RHD command injection, and arbitrary
 * readback for diagnosis.
 *
 * NO WRITE PROTECTION, matching the RTL. These can corrupt the RHD config
 * table and leave the sampling cycle issuing nonsense until the FPGA is reset;
 * an FPGA reset is the documented recovery. Gating is Tunki's job
 * (spec 2.2).
 *
 * Calling context, all of them: never from an ISR, never interleaved with an
 * in-progress FPGA_SPI_ReadSamples(). Structural as built — 0xFFF1 rejects
 * these unless streaming is already stopped, and they execute from
 * StreamSendTask's stopped branch.
 */

/* 3-transfer tagged write sequence. */
void     FPGA_SPI_RegWrite16(uint8_t addr, uint16_t value);

/* 2 transfers: REG_READ(addr) then a NOP carrier. The NOP is not padding —
 * a REG_READ's value lands on the FOLLOWING transfer's MISO. */
uint16_t FPGA_SPI_RegRead16(uint8_t addr);

/* Inject an RHD command word into sampling slot `slot` (RAM word 48+slot).
 * Slot k's answer is observed at ch_a[5:0] = k once the two-counter offset
 * lands (spec 1.2); slot 32 is the alternate-command placeholder.
 * Returns 0 and issues no transfers if slot > 32. */
uint8_t  FPGA_SPI_SetSamplingSlot(uint8_t slot, uint16_t cmd);

/* Select what the FPGA streams — RAM word 229, bits [1:0]:
 *   0 = real RHD data (FPGA reset default), 1 = ramp test pattern.
 * The test pattern is always an explicit opt-in: a test-pattern default would
 * silently reproduce the A.1 bug (convincing synthetic data, no error) after
 * every FPGA reset. See spec 1.1.
 * Requires the A.1.1e RTL; against an older bitstream word 229 is an ordinary
 * unused word and these are harmless no-ops that read back 0. */
void     FPGA_SPI_SetDataSource(uint8_t source);
uint8_t  FPGA_SPI_ReadDataSource(void);

#ifdef __cplusplus
}
#endif

#endif /* FPGA_SPI_H */
