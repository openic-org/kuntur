/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stream_app.c
  * @author  Manuel's Team
  * @brief   STREAM application – ring buffer, simulation, BLE notification send.
  *
  *  Data flow (simulation mode):
  *    StreamSendTimerCb()  →  StreamSendTask()  →  STREAM_NotifyData()
  *
  *  Data flow (FPGA/SPI mode, steady state):
  *    StreamSendTask() → StreamAssemblePacket() → FPGA_SPI_ReadSamples() → STREAM_NotifyData()
  *
  *  Data flow (FPGA/SPI mode, during a BLE TX-pool flow-off stall):
  *    StreamSendTimerCb() [~2 ms fallback tick] → StreamIngestDuringStall()
  *      → FPGA_SPI_ReadSamples() → ring buffer (s_ringBuf, 2048 pairs / ~68 ms)
  *    On resume, StreamAssemblePacket() drains the ring buffer (oldest first)
  *    before falling back to live FPGA reads — see STREAM_APP_ResumeSending().
  *
  *  Target rate:  30 000 SPS per channel  (59 pairs/pkt × ~508 pkt/s)
  *  BLE strategy: tight notify→BLE_STACK_Tick loop (Phase 1 experiment).
  *                Direct BLE_STACK_Tick() call bypasses UTIL_SEQ __WFE() sleep.
  *                If batch ≥ 4 at flow-off, UTIL_SEQ was the bottleneck → 30k SPS possible.
  *                If batch = 1, BLE_STACK_Tick itself is slow → hardware pivot needed.
  *  On disconnect: sending stops immediately (ring buffer is NOT filled — see
  *                 the top-of-task IsConnected/IsCCCDEnabled check); it only
  *                 absorbs flow-off stalls within an active connection.
  *  On reconnect:  ring buffer indices reset; streaming restarts from scratch.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "app_ble.h"
#include "ble.h"
#include "stream_app.h"
#include "stream.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_seq.h"
#include "stm32wb0x_hal_radio_timer.h"
#include "stm32_lpm.h"
#include "fpga_spi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

typedef struct
{
  /* USER CODE BEGIN Service_APP_Context_t */
  uint16_t  ConnectionHandle;
  uint8_t   IsConnected;
  uint8_t   IsCCCDEnabled;  /**< 1 once Android has written CCCD to enable notifications */
  /* USER CODE END Service_APP_Context_t */
} STREAM_APP_Context_t;

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

/** VTIMER safety-fallback period (ms; actual ≈ period/2.59 due to radio clock).
 *  Real throughput is driven by StreamSendTask's tight BLE_STACK_Tick() loop. */
#define STREAM_SEND_PERIOD_MS   (2U)

/* USER CODE END PD */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

extern RTC_HandleTypeDef hrtc;  /**< Defined in main.c; LSE-clocked, BYPSHAD set. */

/* USER CODE END EV */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
static STREAM_APP_Context_t STREAM_APP_Context;

/* Legacy tx buffer (kept for STREAM_APP_SendData API compatibility) */
uint8_t a_STREAM_Data[STREAM_BUFFER_SIZE];

/* USER CODE BEGIN PV */

/* ---------- Timers -------------------------------------------------------- */
static VTIMER_HandleType send_timer;
/* sim_timer removed: simulation is now fed synchronously inside StreamSendTimerCb
 * so the generation rate is always exactly equal to the send rate, immune to
 * any radio-timer tick-rate inaccuracy. */

/* ---------- Packet state -------------------------------------------------- */
static uint8_t seq_num = 0;           /**< Rolling packet sequence number     */

/** TX flow control: set when aci_gatt_srv_notify returns 0x88 (pool full).
 *  Cleared by STREAM_APP_ResumeSending() via ACI_GATT_TX_POOL_AVAILABLE event. */
static uint8_t s_txFlowOff = 0U;

/* SET_CHANNELS / STOP_STREAMING / START_STREAMING state, polled from
 * StreamSendTask() — see the full explanation above STREAM_APP_OnCommandWrite()
 * / SetChannelsTask() further down this file, and
 * docs/interfaces/channel-selection-control-plane.md section 5. Declared
 * here (not lower, near where used) because StreamSendTask() references
 * them and is defined earlier in this file than the 0xFFF1 command handler
 * section.
 *
 * 2026-08-06 history: three earlier placements for SET_CHANNELS's SPI0 work
 * (inline in the BLE event-handler callback; a standalone UTIL_SEQ task
 * that mysteriously never ran; debounced-and-checked-inside-the-streaming-
 * loop, which survived one call but hung the MCU on repeated rapid calls)
 * were all abandoned. Current design: SET_CHANNELS now requires streaming
 * to already be stopped via STOP_STREAMING — no more searching for a safe
 * moment to interleave with live streaming, because there no longer needs
 * to be one. The debounce timer from the third attempt is gone; explicit
 * operator-paced STOP/SET/START sequencing (separate BLE round-trips)
 * provides natural spacing instead. */
static uint8_t  s_pending_ch_a;
static uint8_t  s_pending_ch_b;
/* Per-command debug output for the register console — OFF by default.
 *
 * Bench evidence 2026-08-11: a verification-ladder rung issues ~90 console
 * commands back to back, and each one was emitting ~3 APP_DBG_MSG lines. At
 * 115200 that is ~15 ms of BLOCKING UART per command, and USART1 sits on APB1,
 * which the BLE radio gates — the same hazard that forced the FPGA SPI onto
 * bit-banged AHB GPIO. The symptom was a steady trickle of Tunki command
 * timeouts (recovered by retry) and one rung aborting outright in its restore
 * phase, which leaves the FPGA half-reconfigured.
 *
 * One line per operator click is fine; ninety per rung is not. Mismatches and
 * malformed frames still print unconditionally — those are rare and are
 * exactly what you want to see. */
#ifndef STREAM_REG_CONSOLE_VERBOSE
#define STREAM_REG_CONSOLE_VERBOSE  0
#endif

#if STREAM_REG_CONSOLE_VERBOSE
#define REG_DBG_MSG(...)  APP_DBG_MSG(__VA_ARGS__)
#else
#define REG_DBG_MSG(...)  ((void)0)
#endif

/* Register console (0x04/0x05) — same lifetime and same protection as
 * s_pending_ch_a/b above: written by STREAM_APP_OnCommandWrite(), consumed by
 * StreamSendTask()'s stopped branch, guarded against a mid-use overwrite by
 * s_command_busy. */
static uint8_t  s_pending_reg_addr;
static uint16_t s_pending_reg_value;
static volatile uint8_t s_reg_write_pending;
static volatile uint8_t s_reg_read_pending;
static uint16_t s_pending_conn_handle;
static volatile uint8_t s_set_channels_pending;

/** Set by STOP_STREAMING, cleared by START_STREAMING. While set,
 *  StreamSendTask()'s tight per-packet loop is never entered — see the
 *  branch near the top of StreamSendTask(). */
static volatile uint8_t s_streaming_stopped;

/** Set by STOP_STREAMING; drained by StreamSendTask()'s stopped-branch,
 *  which disables FPGA sample ingestion at the source
 *  (FPGA_SPI_SetStreamEnable(0), regbank addr 36) and then flushes fifo0 —
 *  blocking SPI0 work, must not run inline in the BLE event-handler
 *  callback, same rule as SET_CHANNELS. Revised 2026-08-06: the flush now
 *  happens *here*, at stop time, not at START_STREAMING — because ingestion
 *  is disabled first, the flush drains a static backlog instead of racing
 *  a continuous 30 kSPS producer (which is what made the old
 *  discard-on-resume design never actually reach true empty — see
 *  PLAN.md A.2 and StreamFlushFpgaFifo()'s doc comment). */
static volatile uint8_t s_stop_pending;

/** Set by START_STREAMING; drained by StreamSendTask()'s stopped-branch,
 *  which now only needs to re-enable FPGA ingestion
 *  (FPGA_SPI_SetStreamEnable(1)) — no flush here anymore, fifo0 has been
 *  empty (and ingestion off) since STOP_STREAMING's flush completed. */
static volatile uint8_t s_start_streaming_pending;

/** Set for the duration of SetChannelsTask()/StreamFlushFpgaFifo() —
 *  both call BLE_STACK_Tick() (the flush internally, periodically; more
 *  importantly, StreamSendTask() calls it right after either returns, and
 *  a still-in-progress flush's OWN internal ticks already let a new GATT
 *  write get processed *while the outer call hasn't returned yet*). A new
 *  command arriving during that window would previously overwrite
 *  s_pending_ch_a/b or s_streaming_stopped mid-use by the in-progress
 *  operation — observed 2026-08-06 as a real bug (not hypothetical): the
 *  debug log showed a new STOP_STREAMING + SET_CHANNELS cycle's prints
 *  interleaved *inside* a still-running flush's own prints, and Tunki
 *  intermittently got "no response" for a SET_CHANNELS that had been
 *  silently orphaned this way (pending flag set, but streaming had already
 *  resumed by the time anything checked it again). STREAM_APP_OnCommandWrite()
 *  now rejects any new command while this is set, making one full
 *  STOP/SET/START cycle atomic with respect to the next. */
static volatile uint8_t s_command_busy;

/** RTC timestamp captured at the moment flow-off begins, used to measure how
 *  long the controller's TX pool takes to free up again — that duration is
 *  entirely outside application control (depends on the radio successfully
 *  draining its queue over the air), so a long one here is a direct signature
 *  of link-layer trouble (interference/retries), not a firmware stall. */
static uint32_t s_flowoff_ts_s   = 0U;
static uint16_t s_flowoff_ts_sub = 0U;
/** Longest flow-off->resume duration seen since connect, for a quick at-a-glance
 *  worst case without needing to catch a live print. */
static uint32_t s_longest_stall_us = 0U;

/* ---------- Burst diagnostics --------------------------------------------- */
/** Count of successful notifies since the last pool-available event (batch depth). */
static uint32_t s_notifs_in_batch   = 0U;
/** Total flow-off events since connect (0x88 returned by aci_gatt_srv_notify). */
static uint32_t s_flowoff_total     = 0U;
/** Total pool-available (resume) events since connect. */
static uint32_t s_resume_total      = 0U;
/** Total successful notifications since connect. */
static uint32_t s_notify_total      = 0U;

/* ---------- A.7 step 2 telemetry counters ---------------------------------
 * docs/interfaces/stream-packet-format.md section 6. Cumulative since
 * START_STREAMING / CCCD enable, never reset by a report — a lost telemetry
 * frame then costs resolution, not information (spec section 6.3).
 */

/** Aggregate samples (2 per pair) handed to the radio since streaming started.
 *  This is spec section 3.1's `sample_index` maintained early: under v0 it is
 *  not in the packet header, but the receiver can count the same quantity
 *  independently, and the difference between the two IS the whole-path sample
 *  loss (spec section 6.7). Incremented only after a notify succeeds, so it
 *  counts what was actually handed over, not what was assembled. */
static uint32_t s_sample_index = 0U;

/** Samples discarded because the ring had no room for a packet that had
 *  already been pulled out of the FPGA when the notify failed — the flow_off:
 *  label below. This is the only site in this file that genuinely discards:
 *  StreamIngestDuringStall() and StreamContinuousPollDuringStall() clamp their
 *  FPGA *read* rather than their push, so a short read leaves the remainder in
 *  fifo0 and loses nothing. (Spec section 6.5 names both as discard sites;
 *  only this one is.) */
static uint32_t s_ring_truncated_samples = 0U;

/** Cumulative milliseconds spent with s_txFlowOff set. With s_flowoff_total
 *  this gives both halves of the stall duty cycle that spec section 1.3's `m`
 *  has to clear, and that step 3b needs measured rather than assumed.
 *  Accumulated through a microsecond remainder rather than by dividing each
 *  stall: at ~133 flow-offs/s, most of them resolving in well under 1 ms,
 *  truncating each one to whole milliseconds would report a duty cycle of
 *  approximately zero — i.e. exactly the wrong answer, confidently. */
static uint32_t s_stall_time_ms_total = 0U;
static uint32_t s_stall_us_remainder  = 0U;

/* ---------- Telemetry emission ---------------------------------------------
 * Cadence is ~1 Hz and deliberately imprecise (spec section 6.3): it is a
 * report about the stream, never a participant in it.
 *
 * fifo0's overflow counter and high-water mark do not exist in the RTL yet
 * (A.7 step 1 / spec section 6.4). Until they do, no regbank read is issued at
 * all and the frame carries fpga_counters_valid = 0 — a receiver then knows
 * FPGA-side accounting is ABSENT, which is a different fact from "clean".
 * Turning it on is this switch plus the two word addresses; nothing else in
 * the telemetry chain changes, so the chain does not need re-testing when the
 * RTL lands. See spec section 6.6 for why reading the regbank mid-stream is
 * safe from this one call site and nowhere else. */
#define STREAM_TELEMETRY_FPGA_COUNTERS   0
#if STREAM_TELEMETRY_FPGA_COUNTERS
#define STREAM_REG_ADDR_FIFO0_OVF_LO     0U   /* TODO: set with the A.7 step 1 RTL */
#define STREAM_REG_ADDR_FIFO0_OVF_HI     0U
#define STREAM_REG_ADDR_FIFO0_HIGH_WATER 0U
#endif

#define STREAM_TELEMETRY_PERIOD_MS       1000U
static uint32_t s_telemetry_last_ms  = 0U;
static uint32_t s_telemetry_sent     = 0U;
static uint32_t s_telemetry_failed   = 0U;

/* ---------- MCU-side sample ring buffer -----------------------------------
 * Absorbs the gap between a BLE TX-pool flow-off and STREAM_APP_ResumeSending()
 * so samples that arrive during a stall aren't lost, without touching the
 * steady-state send loop's timing (see StreamAssemblePacket / StreamIngestDuringStall).
 * Sized as a power of 2 for cheap index masking.
 * 2048 pairs = ~68 ms at 30 kSPS, 8 KB RAM — stacked on top of the FPGA's own
 * ~137 ms hardware FIFO (fifo0, ADDR_WIDTH=12) for ~205 ms combined margin,
 * comfortably above the worst stall measured so far (~22 ms, scope_15).
 * (Was ~34 ms / ADDR_WIDTH=10 / ~100 ms combined until fifo0 was deepened
 * 1024->4096 pairs on 2026-07-31 — see the fifo_full watch below.) */
#define STREAM_RING_PAIRS   2048U
#define STREAM_RING_MASK    (STREAM_RING_PAIRS - 1U)

typedef struct { int16_t ch0; int16_t ch1; } StreamPair_t;

static StreamPair_t s_ringBuf[STREAM_RING_PAIRS];
/** Free-running write/read indices; count = head - tail, valid across wraparound. */
static uint32_t s_ringHead = 0U;
static uint32_t s_ringTail = 0U;

/* ---------- Category-B post-drain watch (diagnostic, default OFF) ---------
 * The +40.1ms residual-SKP offset (2026-07-31, correlate_stalls.py) is
 * relative to "ring drained to 0", not colocated with it, so a single
 * timestamped print at the drain itself can't show what actually fires
 * ~40ms later. This opens a bounded window after every drain-to-0 and
 * records what happens inside it — flow-off/resume transitions and any
 * assembled packet whose ch0 step is anomalous — flushed once the window
 * closes. See memory/project_skp_investigation.md Category B.
 *
 * v1 (2026-07-31 first capture) buffered and printed EVERY packet in the
 * window (~28 lines/drain). APP_DBG_MSG is a blocking, one-character-at-a-
 * time HAL_UART_Transmit at 115200 baud (~86.8us/char) — 28 lines was
 * ~144ms of blocking transmit sitting inside the tight notify loop, 86
 * times over an 83s recording (~12.4s total, ~15% of the run). That's
 * long enough to overrun the FPGA's hardware FIFO on its own at either
 * depth (~34ms before the 2026-07-31 deepening, ~137ms after) —
 * confirmed against bench data: pps mean dropped 500.2->428.8, std
 * 5.0->52.8 vs. a no-diagnostic baseline recorded the same day. The
 * "big gaps"/throughput drop the user saw live were this, not (only)
 * the phenomenon being measured. v2 (below) only records/prints
 * anomalous packets (~1/window in practice) instead of the whole
 * window, cutting output ~25x. */
#define STREAM_DIAG_POST_DRAIN_WATCH   1

#if STREAM_DIAG_POST_DRAIN_WATCH
/* Forward decl needed here: the real prototype further down (with the rest
 * of this file's private prototypes) comes after this block, so without
 * this, StreamGetTimestamp() below is an implicit (int-returning) extern
 * declaration that then conflicts with the real static one. */
static void StreamGetTimestamp(uint32_t *pSeconds, uint16_t *pSubSeconds);

#define DRAIN_WATCH_US        60000U  /* covers the +40-43ms band plus margin */
#define DRAIN_WATCH_MAX_EVT   12U     /* defensive cap only — normal case is ~1 */

typedef struct
{
  int32_t  since_us;
  int16_t  ch0;
  int32_t  step;
  uint32_t from_ring;
  uint32_t from_fpga;
  uint8_t  kind;         /* 0=pkt (anomalous only)  1=flow-off  2=resume */
} DrainWatchEvt_t;

static DrainWatchEvt_t s_drainWatch[DRAIN_WATCH_MAX_EVT];
static uint8_t  s_drainWatchCount    = 0U;
static uint8_t  s_drainWatchActive   = 0U;
static uint32_t s_drainWatch_s       = 0U;
static uint16_t s_drainWatch_sub     = 0U;
/* Previous packet's ch0, tracked every packet regardless of window state so
 * step is always valid the instant a window opens — cheap arithmetic, no
 * UART, safe to run unconditionally in the hot loop. */
static int32_t  s_prevPktCh0    = 0;
static uint8_t  s_havePrevPktCh0 = 0U;

static int32_t StreamDrainWatchElapsedUs(uint32_t now_s, uint16_t now_sub)
{
  int32_t ds   = (int32_t)(now_s - s_drainWatch_s);
  int32_t dsub = (int32_t)now_sub - (int32_t)s_drainWatch_sub;
  return (int32_t)((int64_t)ds * 1000000 + (int64_t)dsub * 1000 / 32);
}

static void StreamDrainWatchFlush(void)
{
  if (s_drainWatchCount == 0U)
  {
    s_drainWatchActive = 0U;
    return;
  }
  for (uint8_t i = 0U; i < s_drainWatchCount; i++)
  {
    DrainWatchEvt_t *e = &s_drainWatch[i];
    if (e->kind == 1U)
      APP_DBG_MSG("STREAM: post-drain +%6ld us  FLOW-OFF\n", (long)e->since_us);
    else if (e->kind == 2U)
      APP_DBG_MSG("STREAM: post-drain +%6ld us  RESUME\n", (long)e->since_us);
    else
      APP_DBG_MSG("STREAM: post-drain +%6ld us  SKP ch0=%6d step=%+5ld  (ring=%lu fpga=%lu)\n",
                  (long)e->since_us, (int)e->ch0, (long)e->step,
                  (unsigned long)e->from_ring, (unsigned long)e->from_fpga);
  }
  s_drainWatchActive = 0U;
  s_drainWatchCount  = 0U;
}

/** Record a flow-off/resume event into the current window (rare — a few per
 *  window at most, cheap to always print). Auto-closes+flushes if the window
 *  has elapsed. */
static void StreamDrainWatchRecordEvt(uint8_t kind)
{
  if (!s_drainWatchActive)
    return;

  uint32_t ts_s; uint16_t ts_sub;
  StreamGetTimestamp(&ts_s, &ts_sub);
  int32_t since_us = StreamDrainWatchElapsedUs(ts_s, ts_sub);

  if (since_us > (int32_t)DRAIN_WATCH_US || s_drainWatchCount >= DRAIN_WATCH_MAX_EVT)
  {
    StreamDrainWatchFlush();
    return;
  }

  DrainWatchEvt_t *e = &s_drainWatch[s_drainWatchCount++];
  e->since_us = since_us;
  e->kind     = kind;
}

/** Called once per assembled packet, unconditionally (cheap — no UART unless
 *  step is actually anomalous). Tracks ch0 continuity across packets
 *  (unwrapped modulo 65536, same fix as analyze_recording.py's compute_stats
 *  — otherwise a packet landing on the ramp's own wrap boundary would read
 *  as a bogus ~-65000 "step") and, only if a window is open and the step is
 *  anomalous, records+possibly-flushes it. */
static void StreamDrainWatchCheckPacket(int16_t ch0, uint32_t n_pairs,
                                         uint32_t from_ring, uint32_t from_fpga)
{
  int32_t step = 0;
  if (s_havePrevPktCh0)
  {
    int32_t raw = (int32_t)ch0 - s_prevPktCh0;
    step = ((raw + 32768) % 65536) - 32768;   /* unwrap the int16 ramp */
  }
  uint8_t have_prev = s_havePrevPktCh0;
  s_prevPktCh0     = ch0;
  s_havePrevPktCh0 = 1U;

  if (!s_drainWatchActive || !have_prev || step == (int32_t)n_pairs)
    return;   /* no window open, no reference yet, or steady-state (expected step) */

  uint32_t ts_s; uint16_t ts_sub;
  StreamGetTimestamp(&ts_s, &ts_sub);
  int32_t since_us = StreamDrainWatchElapsedUs(ts_s, ts_sub);

  if (since_us > (int32_t)DRAIN_WATCH_US || s_drainWatchCount >= DRAIN_WATCH_MAX_EVT)
  {
    StreamDrainWatchFlush();
    return;
  }

  DrainWatchEvt_t *e = &s_drainWatch[s_drainWatchCount++];
  e->since_us  = since_us;
  e->kind      = 0U;
  e->ch0       = ch0;
  e->step      = step;
  e->from_ring = from_ring;
  e->from_fpga = from_fpga;
}
#endif /* STREAM_DIAG_POST_DRAIN_WATCH */

/* ---------- fifo_full long-duration watch (diagnostic) --------------------
 * The FPGA's fifo0 was deepened 1024->4096 pairs (2026-07-31,
 * kuntur_fpga.v ADDR_WIDTH 10->12, ~137ms buffering) to absorb the
 * BLE_STACK_Tick() blackout, confirmed via post-synthesis report (33% BRAM,
 * matching 4096 entries). But fifo_full (routed to PA10, replacing the old
 * flow-off output marker — see STREAM_APP_Init) still asserts on the same
 * ~ms timescale as before the depth change (scope_20), which a single
 * ~10-22ms blackout shouldn't come close to filling. Leading hypothesis:
 * chronic backlog accumulated across many flow-off episodes, not a single
 * episode locally overflowing — needs a long (many-second/minute)
 * recording to distinguish from a one-off spike, not another scope shot.
 * See memory/project_skp_investigation.md Category B.
 *
 * v1 (2026-08-03 first long capture) used an EXTI both-edge interrupt on
 * PA10, buffered in the ISR, flushed from StreamSendTask — same
 * no-UART-in-ISR discipline as the post-drain watch above. Caused the same
 * class of throughput perturbation as that watch's own v1 anyway: the raw
 * fifo_full signal turned out to be chattering continuously at ~30-60us
 * intervals (3168 edges logged over a 100s recording, clustered in
 * microsecond-scale bursts — not clean asserts/clears, electrical/logic
 * noise right at the FIFO's full boundary, consistent with occupancy
 * genuinely riding at/near the threshold much of the time). Even bounded
 * per-flush buffering couldn't keep up with a genuine edge-interrupt storm
 * at that rate. v2 (below) drops the interrupt entirely and polls the pin
 * once per StreamSendTask invocation (~2ms cadence, the resolution we
 * actually care about), requiring the new level to hold for several
 * consecutive polls before it's logged as a real transition — a
 * classic debounce that's immune to sub-poll-period chatter by
 * construction, whatever the raw signal is doing underneath. */
/* 2026-08-03: disabled. This pin (PA10) is one of the STM32WB09xE's eight
 * boot-strap pins (IOBOOTVAL bit 2) — the ROM bootloader forces PA10 high at
 * POR to hard-select the UART bootloader instead of launching the
 * application (DS14210 3.22). Wiring it directly to the FPGA's fifo_full
 * output (NOPULL, actively driven) meant whatever level the FPGA happened
 * to drive at MCU power-on could hijack the boot path — confirmed via
 * PWRC_SR2 read showing IOBOOTVAL bit2=1 while the CPU was permanently
 * parked in the internal boot ROM. PA10 is now left unconfigured/unused;
 * see memory/project_skp_investigation.md. */
#define STREAM_DIAG_FIFO_FULL_WATCH   0

#if STREAM_DIAG_FIFO_FULL_WATCH
#define FIFO_FULL_DEBOUNCE_POLLS   8U        /* candidate level must hold for this
                                               * many consecutive StreamSendTask
                                               * invocations (~16ms @ ~2ms/poll)
                                               * before it's even considered */
#define FIFO_FULL_MIN_PRINT_GAP_US   100000U /* hard rate limit, independent of
                                               * the debounce above — 2026-08-03:
                                               * a connection dropped hard enough
                                               * to need a power cycle to recover
                                               * after re-enabling this watch,
                                               * suspected cause is real (not
                                               * chatter) toggling on a timescale
                                               * that survives the debounce but
                                               * still prints often enough to
                                               * starve BLE_STACK_Tick() of CPU
                                               * time past the connection's
                                               * supervision timeout. This caps
                                               * worst-case print rate to ~10/s
                                               * no matter what the signal does. */

static uint8_t  s_fifoFullConfirmed = 0U;   /* last logged/confirmed level */
static uint8_t  s_fifoFullCandidate = 0U;   /* level currently being confirmed */
static uint8_t  s_fifoFullRunLength = 0U;   /* consecutive polls at candidate level */
static uint32_t s_fifoFullLastPrint_s   = 0U;
static uint16_t s_fifoFullLastPrint_sub = 0U;
static uint8_t  s_fifoFullHavePrinted   = 0U;

/* Called once per StreamSendTask invocation — plain GPIO read, no
 * interrupts involved. Only prints (rare, debounced, additionally rate-
 * limited) when the level has genuinely changed, held for
 * FIFO_FULL_DEBOUNCE_POLLS in a row, AND enough time has passed since the
 * last print. A transition that's debounce-confirmed but rate-limited is
 * simply not logged (silently skipped) rather than queued/retried — under
 * sustained rapid toggling this means some transitions go unlogged, which
 * is the intended tradeoff: this diagnostic must never be able to disrupt
 * the connection it's trying to observe. */
static void StreamFifoFullWatchPoll(void)
{
  uint8_t level = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET) ? 1U : 0U;

  if (level == s_fifoFullCandidate)
  {
    if (s_fifoFullRunLength < 0xFFU)
      s_fifoFullRunLength++;
  }
  else
  {
    s_fifoFullCandidate = level;
    s_fifoFullRunLength = 1U;
  }

  if (s_fifoFullRunLength != FIFO_FULL_DEBOUNCE_POLLS || s_fifoFullCandidate == s_fifoFullConfirmed)
    return;

  uint32_t ts_s; uint16_t ts_sub;
  StreamGetTimestamp(&ts_s, &ts_sub);

  if (s_fifoFullHavePrinted)
  {
    int32_t ds   = (int32_t)(ts_s - s_fifoFullLastPrint_s);
    int32_t dsub = (int32_t)ts_sub - (int32_t)s_fifoFullLastPrint_sub;
    int32_t elapsed_us = (int32_t)((int64_t)ds * 1000000 + (int64_t)dsub * 1000 / 32);
    if (elapsed_us < (int32_t)FIFO_FULL_MIN_PRINT_GAP_US)
      return;   /* rate-limited: skip this transition, don't retry it later */
  }

  s_fifoFullConfirmed     = s_fifoFullCandidate;
  s_fifoFullLastPrint_s   = ts_s;
  s_fifoFullLastPrint_sub = ts_sub;
  s_fifoFullHavePrinted   = 1U;
  APP_DBG_MSG("STREAM: fifo_full %s at %lu.%05u\n",
              s_fifoFullConfirmed ? "SET" : "CLR", (unsigned long)ts_s, ts_sub);
}
#endif /* STREAM_DIAG_FIFO_FULL_WATCH */

/* ---------- MCU software ring buffer occupancy watch (diagnostic) --------
 * Added 2026-08-03: after fixing the chronic-backlog issue
 * (CFG_BLE_MBLOCK_COUNT_MARGIN 54->200), the user wants visibility into
 * whether STREAM_RING_PAIRS (2048, ~68ms) is actually being approached
 * during normal operation or is comfortably oversized — to make an
 * evidence-based call instead of guessing at buffer sizes. Tracks the
 * peak occupancy (s_ringHead - s_ringTail) since the last report and
 * prints a summary periodically — no edge-triggering, no interrupt, just
 * a peak-hold sampled every StreamSendTask invocation and flushed on a
 * timer, so it can't reproduce either of this session's earlier
 * perturbation bugs (chattering signal / high print volume). See
 * memory/project_skp_investigation.md and project_production_readiness.md. */
#define STREAM_DIAG_RING_WATCH   0

#if STREAM_DIAG_RING_WATCH
/* Forward decl: StreamRingCount()'s real prototype comes later with the
 * rest of this file's private prototypes — same reason StreamGetTimestamp
 * needed one above. */
static uint32_t StreamRingCount(void);

#define RING_WATCH_REPORT_US   2000000U   /* report cadence: ~2s */

static uint32_t s_ringWatchPeak     = 0U;
static uint32_t s_ringWatchLast_s   = 0U;
static uint16_t s_ringWatchLast_sub = 0U;
static uint8_t  s_ringWatchHaveLast = 0U;

/* Called once per StreamSendTask invocation. */
static void StreamRingWatchPoll(void)
{
  uint32_t count = StreamRingCount();
  if (count > s_ringWatchPeak)
    s_ringWatchPeak = count;

  uint32_t ts_s; uint16_t ts_sub;
  StreamGetTimestamp(&ts_s, &ts_sub);

  if (!s_ringWatchHaveLast)
  {
    s_ringWatchLast_s   = ts_s;
    s_ringWatchLast_sub = ts_sub;
    s_ringWatchHaveLast = 1U;
    return;
  }

  int32_t ds   = (int32_t)(ts_s - s_ringWatchLast_s);
  int32_t dsub = (int32_t)ts_sub - (int32_t)s_ringWatchLast_sub;
  int32_t elapsed_us = (int32_t)((int64_t)ds * 1000000 + (int64_t)dsub * 1000 / 32);
  if (elapsed_us < (int32_t)RING_WATCH_REPORT_US)
    return;

  unsigned long pct = (unsigned long)s_ringWatchPeak * 100UL / STREAM_RING_PAIRS;
  APP_DBG_MSG("STREAM: ring peak=%lu/%lu (%lu%%) at %lu.%05u\n",
              (unsigned long)s_ringWatchPeak, (unsigned long)STREAM_RING_PAIRS, pct,
              (unsigned long)ts_s, ts_sub);

  s_ringWatchPeak     = 0U;
  s_ringWatchLast_s   = ts_s;
  s_ringWatchLast_sub = ts_sub;
}
#endif /* STREAM_DIAG_RING_WATCH */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

static void StreamSendTimerCb(void *pData);
static void StreamSendTask(void);
static void SetChannelsTask(void);
static void StreamFlushFpgaFifo(void);
static void StreamGetTimestamp(uint32_t *pSeconds, uint16_t *pSubSeconds);
static uint32_t StreamRingCount(void);
static uint32_t StreamRingFree(void);
static void StreamRingPush(const int16_t *pairs, uint32_t n_pairs);
static void StreamRingPop(int16_t *pairs, uint32_t n_pairs);
static void StreamIngestDuringStall(void);
static void StreamAssemblePacket(int16_t *samples, uint32_t n_pairs);

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
/* STREAM_Notification() is defined in stream.c – do not duplicate here.    */

void STREAM_APP_EvtRx(STREAM_APP_ConnHandleNotEvt_t *p_Notification)
{
  switch(p_Notification->EvtOpcode)
  {
    case STREAM_CONN_HANDLE_EVT:
      STREAM_APP_Context.ConnectionHandle = p_Notification->ConnectionHandle;
      STREAM_APP_Context.IsConnected  = 1U;
      STREAM_APP_Context.IsCCCDEnabled = 0U;  /* wait for Android to write CCCD before streaming */
      s_txFlowOff = 0U;
      seq_num     = 0U;
      s_ringHead  = 0U;
      s_ringTail  = 0U;
      s_streaming_stopped       = 0U;
      s_stop_pending            = 0U;
      s_set_channels_pending    = 0U;
      s_start_streaming_pending = 0U;
      s_reg_write_pending       = 0U;
      s_reg_read_pending        = 0U;
      s_command_busy            = 0U;
      /* USER CODE BEGIN Service_APP_CENTR_CONN_HANDLE_EVT */
      APP_DBG_MSG("STREAM: connected (handle 0x%04X)\n",
                  p_Notification->ConnectionHandle);
      /* USER CODE END Service_APP_CENTR_CONN_HANDLE_EVT */
      break;

    case STREAM_DISCON_HANDLE_EVT:
      STREAM_APP_Context.ConnectionHandle = 0xFFFF;
      STREAM_APP_Context.IsConnected  = 0U;
      STREAM_APP_Context.IsCCCDEnabled = 0U;
      /* USER CODE BEGIN Service_APP_DISCON_HANDLE_EVT */
      /* Release the DEEPSTOP block unconditionally — disconnect can happen
       * while CCCD was still enabled, without a CCCD-disable write ever
       * arriving to clear it via STREAM_APP_OnCCCDWrite(). */
      UTIL_LPM_SetStopMode(1U << CFG_LPM_FPGA_SPI, UTIL_LPM_ENABLE);
      UTIL_LPM_SetOffMode(1U << CFG_LPM_FPGA_SPI, UTIL_LPM_ENABLE);
      APP_DBG_MSG("STREAM: disconnected, sending stopped.\n");
      /* USER CODE END Service_APP_DISCON_HANDLE_EVT */
      break;

    default:
      break;
  }
  return;
}

void STREAM_APP_Init(void)
{
  STREAM_APP_Context.ConnectionHandle = 0xFFFF;
  STREAM_APP_Context.IsConnected = 0U;
  STREAM_APP_Context.IsCCCDEnabled = 0U;
  STREAM_Init();

  /* USER CODE BEGIN Service_APP_Init */

  /* PA10 repurposed 2026-07-31: was the TX-flow-off output marker (used for
   * scope triggering in scope_16-20); now reads the FPGA's fifo_full signal
   * directly (wired by the user onto the same test point scope_16-20
   * probed). The flow-off/resume events it used to mark are still
   * independently logged via the existing "STREAM: flow-off #N" /
   * "STREAM: LONG STALL ... resumed_at=" prints on the same RTC clock, so
   * nothing is lost — this just needed the pin back, since driving it as an
   * output while the FPGA also drives it would short the two together.
   * Plain polled input, not EXTI (see STREAM_DIAG_FIFO_FULL_WATCH above for
   * why — the raw signal chatters too fast for edge interrupts to be safe).
   * See memory/project_skp_investigation.md Category B. */
  __HAL_RCC_GPIOA_CLK_ENABLE();
#if STREAM_DIAG_FIFO_FULL_WATCH
  {
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;   /* actively driven by the FPGA */
    HAL_GPIO_Init(GPIOA, &gpio);
  }
#endif /* STREAM_DIAG_FIFO_FULL_WATCH */

  /* PB1: StreamSendTask invocation heartbeat — pulses every time the task is
   * actually entered, regardless of what it does afterward (early-return on
   * not-connected/flow-off, or a real send). Lets us see directly whether
   * the task is being scheduled at all during the ~10ms gap after PA10 goes
   * low, rather than inferring it from out_ptr[0] alone. */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  {
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_1;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  }

  /* PA0: UTIL_LPM_EnterLowPower() entry/exit marker (toggled in app_entry.c's
   * UTIL_SEQ_Idle()). High for the exact duration the CPU is inside that
   * call — direct test of whether the ~10ms gap is spent in low-power sleep
   * entry itself, or happens before it's even reached (e.g. inside the
   * closed-source BLE_STACK_SleepCheck() gate). GPIOA clock already enabled
   * above for PA10. */
  {
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_0;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  }

  /* Register the streaming send task with the cooperative scheduler.
   * CFG_TASK_STREAM_SEND_ID is defined in stream_app.h (bit 6).
   * Verify it does not conflict with other tasks in app_conf.h.            */
  UTIL_SEQ_RegTask(1UL << CFG_TASK_STREAM_SEND_ID, UTIL_SEQ_RFU, StreamSendTask);
  /* SetChannelsTask() is polled from inside StreamSendTask(), not registered
   * as its own UTIL_SEQ task — see the comment above s_pending_ch_a in the
   * 0xFFF1 command handler section for why. */

  /* Start the BLE send timer (assembles and transmits packets).
   * Simulation is fed synchronously inside StreamSendTimerCb — no separate
   * sim timer needed. */
  send_timer.callback = StreamSendTimerCb;
  send_timer.userData = NULL;
  HAL_RADIO_TIMER_StartVirtualTimer(&send_timer, STREAM_SEND_PERIOD_MS);

  APP_DBG_MSG("STREAM: service initialised.  Send=%u ms  (sync SPI pull)\n",
              STREAM_SEND_PERIOD_MS);

  /* USER CODE END Service_APP_Init */
  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* -------------------------------------------------------------------------- */
/*  Timer callbacks (interrupt context – only schedule tasks, no BLE calls)  */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Send-timer callback: schedule the streaming send task.
 *         Re-arms itself for the next period.
 */
static void StreamSendTimerCb(void *pData)
{
  /* Safety fallback: if the event-driven loop stalled, re-prime it.
   * If s_txFlowOff is set, StreamSendTask returns immediately; the
   * ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE handler will restart it. */
  UTIL_SEQ_SetTask(1UL << CFG_TASK_STREAM_SEND_ID, CFG_SEQ_PRIO_1);
  HAL_RADIO_TIMER_StartVirtualTimer(&send_timer, STREAM_SEND_PERIOD_MS);
}

/* -------------------------------------------------------------------------- */
/*  MCU-side sample ring buffer                                              */
/* -------------------------------------------------------------------------- */

static uint32_t StreamRingCount(void) { return s_ringHead - s_ringTail; }
static uint32_t StreamRingFree(void)  { return STREAM_RING_PAIRS - StreamRingCount(); }

static void StreamRingPush(const int16_t *pairs, uint32_t n_pairs)
{
  for (uint32_t i = 0U; i < n_pairs; i++)
  {
    StreamPair_t *slot = &s_ringBuf[(s_ringHead + i) & STREAM_RING_MASK];
    slot->ch0 = pairs[2U * i];
    slot->ch1 = pairs[2U * i + 1U];
  }
  s_ringHead += n_pairs;
}

static void StreamRingPop(int16_t *pairs, uint32_t n_pairs)
{
  for (uint32_t i = 0U; i < n_pairs; i++)
  {
    const StreamPair_t *slot = &s_ringBuf[(s_ringTail + i) & STREAM_RING_MASK];
    pairs[2U * i]      = slot->ch0;
    pairs[2U * i + 1U] = slot->ch1;
  }
  s_ringTail += n_pairs;
}

/**
 * @brief  Opportunistically drain one packet's worth of FPGA samples into the
 *         ring buffer while flow-off holds the send loop back.
 *
 *  Called once per StreamSendTask invocation (every ~2 ms via the fallback
 *  VTIMER, same cadence measured on PB1 in scope_15) — pulling one
 *  STREAM_PAIRS_PER_PACKET-sized chunk per tick keeps pace with the ~2 ms of
 *  data the FPGA accumulates per tick, so the backlog grows only slowly even
 *  across a multi-ms stall. Deliberately bounded and always returns instead
 *  of spinning: STREAM_APP_ResumeSending() (and thus the clearing of
 *  s_txFlowOff) runs from the HCI async-event task on this same cooperative
 *  scheduler, so StreamSendTask must give UTIL_SEQ_Run a chance to reach it.
 *  If the ring buffer itself is full, this is a no-op — the FPGA's own
 *  ~137 ms hardware FIFO (fifo0, 4096 pairs) is the second line of defense. */
static void StreamIngestDuringStall(void)
{
  uint32_t room = StreamRingFree();
  if (room == 0U)
    return;

  uint32_t n = STREAM_PAIRS_PER_PACKET;
  if (n > room)
    n = room;

  int16_t tmp[STREAM_PAIRS_PER_PACKET * 2U];
  FPGA_SPI_ReadSamples(tmp, (uint8_t)n);
  StreamRingPush(tmp, n);
}

/** DIAGNOSTIC SWITCH.
 *  0: StreamIngestDuringStall() — bounded, one packet per ~2 ms fallback tick.
 *  1: StreamContinuousPollDuringStall() — gap-free spin for the whole stall.
 *  Result: both produced the SAME SKP rate/signature as no ring buffer at all
 *  (recordings 20260729_143741 vs 20260730_050038 vs 20260730_051445) — the
 *  real loss was the flow_off: transition discarding an already-assembled
 *  packet (fixed below), not polling cadence during the stall. Left at 0
 *  (bounded, scheduler-friendly) for production; kept for reference. */
#define STREAM_DIAG_CONTINUOUS_POLL_DURING_STALL   0

/**
 * @brief  DIAGNOSTIC: poll the FPGA into the ring buffer with zero gaps for
 *         the entire flow-off duration, instead of once per fallback tick.
 *
 *  Safe against deadlocking the cooperative scheduler even though this spins
 *  without returning: BLE_STACK_Tick() "executes the processing of all Host
 *  Stack layers... All stack callbacks are called by this function" (ble_stack.h),
 *  so ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE -> STREAM_APP_ResumeSending() (and
 *  thus s_txFlowOff clearing) is dispatched synchronously from inside this
 *  loop's own BLE_STACK_Tick() calls — it does not depend on UTIL_SEQ_Run()
 *  reaching a separate task the way STREAM_APP_ResumeSending()'s
 *  UTIL_SEQ_SetTask() re-trigger normally does.
 *  Naturally bounded to ~68 ms worst case: once the ring buffer fills, the
 *  loop exits and control returns to the scheduler like the non-diagnostic path. */
static void StreamContinuousPollDuringStall(void)
{
  int16_t tmp[STREAM_PAIRS_PER_PACKET * 2U];

  while (s_txFlowOff && STREAM_APP_Context.IsConnected)
  {
    uint32_t room = StreamRingFree();
    if (room == 0U)
      break;

    uint32_t n = STREAM_PAIRS_PER_PACKET;
    if (n > room)
      n = room;

    FPGA_SPI_ReadSamples(tmp, (uint8_t)n);
    StreamRingPush(tmp, n);

    BLE_STACK_Tick();
  }
}

/**
 * @brief  Fill one packet's sample payload, preferring buffered backlog
 *         (oldest data first) over a fresh FPGA read, so a stall's backlog
 *         drains before catching up to live data.
 */
static void StreamAssemblePacket(int16_t *samples, uint32_t n_pairs)
{
  uint32_t before = StreamRingCount();

  uint32_t from_ring = before;
  if (from_ring > n_pairs)
    from_ring = n_pairs;
  if (from_ring > 0U)
    StreamRingPop(samples, from_ring);

  uint32_t from_fpga = n_pairs - from_ring;
  if (from_fpga > 0U)
    FPGA_SPI_ReadSamples(&samples[from_ring * 2U], (uint8_t)from_fpga);

  /* Category-B SKP diagnostic: log the exact moment the ring backlog fully
   * drains and this packet's assembly falls through to a live FPGA read
   * within the same call (before > 0, after == 0). Refined 2026-07-31: the
   * residual SKPs sit at a tight +40-43ms offset AFTER this moment (varies
   * per session, stable within one), not colocated with it — see
   * STREAM_DIAG_POST_DRAIN_WATCH below for the follow-up trace.
   * See memory/project_skp_investigation.md Category B. */
  if (before > 0U && StreamRingCount() == 0U)
  {
    uint32_t ts_s; uint16_t ts_sub;
    StreamGetTimestamp(&ts_s, &ts_sub);
    APP_DBG_MSG("STREAM: ring drained to 0 at %lu.%05u (backlog was %lu pairs)\n",
                (unsigned long)ts_s, ts_sub, (unsigned long)before);
#if STREAM_DIAG_POST_DRAIN_WATCH
    s_drainWatch_s     = ts_s;
    s_drainWatch_sub   = ts_sub;
    s_drainWatchCount  = 0U;
    s_drainWatchActive = 1U;
#endif
  }

#if STREAM_DIAG_POST_DRAIN_WATCH
  StreamDrainWatchCheckPacket(samples[0], n_pairs, from_ring, from_fpga);
#endif
}

/* -------------------------------------------------------------------------- */
/*  A.7 step 2 telemetry — docs/interfaces/stream-packet-format.md section 6 */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Put every telemetry counter back to zero, so one frame's fields all
 *         describe the same epoch.
 *
 *  Spec section 6.3: counters are cumulative *since START_STREAMING*. The
 *  burst-diagnostic counters are reset here too even though only
 *  s_flowoff_total appears in the frame — a report that mixes "since connect"
 *  and "since start" quantities is the kind of quietly-wrong artefact this
 *  project keeps having to re-derive, and the debug prints that consume the
 *  others read alongside the frame.
 */
static void StreamResetTelemetryCounters(void)
{
  s_sample_index           = 0U;
  s_ring_truncated_samples = 0U;
  s_stall_time_ms_total    = 0U;
  s_stall_us_remainder     = 0U;
  s_flowoff_total          = 0U;
  s_resume_total           = 0U;
  s_notify_total           = 0U;
  s_notifs_in_batch        = 0U;
  s_longest_stall_us       = 0U;
  s_telemetry_last_ms      = HAL_GetTick();
}

/**
 * @brief  Emit one telemetry frame on 0xFFF4 if a second has passed.
 *
 *  CALLING CONTEXT IS THE WHOLE DESIGN — spec section 6.6. This must be
 *  called from exactly one place: the top of StreamSendTask, at a packet
 *  boundary, with no FPGA_SPI_ReadSamples() in progress. That is what makes
 *  the (future) regbank read safe mid-stream despite every other regbank
 *  access in this firmware being confined to the streaming-stopped branch.
 *  Never call it from the 0xFFF1 callback, from a timer, or from the flow-off
 *  path.
 *
 *  Skips rather than defers when the moment is wrong: a stall that keeps the
 *  send loop away for more than a second simply costs that second's frame.
 *  Counters are cumulative, so the next frame carries everything the lost one
 *  would have (spec section 6.3), and a telemetry frame is worth nothing next
 *  to the stream it measures.
 */
static void StreamTelemetryPoll(void)
{
  uint32_t now = HAL_GetTick();
  if ((now - s_telemetry_last_ms) < STREAM_TELEMETRY_PERIOD_MS)
    return;
  s_telemetry_last_ms = now;

  StreamTelemetry_t t;
  t.flags                    = 0U;
  t.anchor_sample_index      = s_sample_index;
  t.fifo0_overflow_samples   = 0U;
  t.fifo0_high_water         = 0U;
  t.ring_truncated_samples   = s_ring_truncated_samples;
  t.flow_off_count           = s_flowoff_total;
  t.stall_time_ms_total      = s_stall_time_ms_total;

  /* Same RTC encoding as the packet header, so the anchor and any v0
   * timestamp are directly comparable without a conversion step. */
  StreamGetTimestamp(&t.anchor_timestamp_s, &t.anchor_timestamp_sub_s);

#if STREAM_TELEMETRY_FPGA_COUNTERS
  /* Reads only, never writes — spec section 6.6 rule 1. Three 2-transfer
   * REG_READ16s, ~60 us total once per second. */
  {
    uint16_t ovf_lo = FPGA_SPI_RegRead16(STREAM_REG_ADDR_FIFO0_OVF_LO);
    uint16_t ovf_hi = FPGA_SPI_RegRead16(STREAM_REG_ADDR_FIFO0_OVF_HI);
    t.fifo0_overflow_samples = ((uint32_t)ovf_hi << 16) | (uint32_t)ovf_lo;
    t.fifo0_high_water       = FPGA_SPI_RegRead16(STREAM_REG_ADDR_FIFO0_HIGH_WATER);
    t.flags |= STREAM_TELEMETRY_FLAG_FPGA_COUNTERS_VALID;
  }
#endif

  tBleStatus ret = STREAM_NotifyTelemetry(&t, STREAM_APP_Context.ConnectionHandle);
  if (ret == BLE_STATUS_SUCCESS)
  {
    s_telemetry_sent++;
  }
  else
  {
    /* Not escalated and deliberately not retried: the most likely cause is a
     * momentarily full mblock pool, and stealing the data stream's recovery
     * budget to re-send a report about the data stream is the wrong trade.
     * Logged sparsely so a systematically failing telemetry path (e.g. the
     * host never enabled the 0xFFF4 CCCD) is still visible on the bench. */
    s_telemetry_failed++;
    if ((s_telemetry_failed % 30U) == 1U)
    {
      APP_DBG_MSG("STREAM: 0xFFF4 telemetry notify failed 0x%02X (sent=%lu failed=%lu, CCCD not enabled?)\n",
                  ret, (unsigned long)s_telemetry_sent, (unsigned long)s_telemetry_failed);
    }
  }
}

/* -------------------------------------------------------------------------- */
/*  Cooperative task: assemble + send one BLE notification packet            */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Streaming send task (WB09KE_HF — Phase 1 tight loop).
 *
 *  Tight loop: notify then call BLE_STACK_Tick() directly (no UTIL_SEQ
 *  round-trip) until the mblock pool is exhausted (0x88).
 *  Expected: batch ≥ 4 → UTIL_SEQ __WFE() was the bottleneck → 30k SPS possible.
 */
static void StreamSendTask(void)
{
  /* PB1 heartbeat: ~1us pulse (64 NOPs @ 64MHz) on every invocation, wide
   * enough to survive coarse sample rates on a long capture window — unlike
   * fifo_ren's single-cycle pulse, this needs to be visible at ~300ns/sample. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  for (volatile int i = 0; i < 64; i++) { __NOP(); }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

#if STREAM_DIAG_FIFO_FULL_WATCH
  StreamFifoFullWatchPoll();
#endif
#if STREAM_DIAG_RING_WATCH
  StreamRingWatchPoll();
#endif

  if (!STREAM_APP_Context.IsConnected || !STREAM_APP_Context.IsCCCDEnabled)
    return;

  /* Telemetry, ~1 Hz — spec section 6.6. This exact position is load-bearing:
   * a packet boundary, before the stopped and flow-off branches below, so the
   * regbank read it will eventually carry cannot land inside an
   * FPGA_SPI_ReadSamples(). Skipped entirely during a flow-off stall, which is
   * both the moment a notify would fail anyway and the moment fifo0 can least
   * afford SPI0 time spent on anything but popping it. */
  if (!s_txFlowOff)
    StreamTelemetryPoll();

  /* Streaming explicitly halted via STOP_STREAMING — see
   * docs/interfaces/channel-selection-control-plane.md section 5. All
   * SET_CHANNELS SPI0/notify work and START_STREAMING's FIFO flush run
   * from here, never from inside the tight per-packet loop below: this
   * branch is guaranteed to be revisited every ~2 ms via the VTIMER
   * fallback (same mechanism flow-off recovery already relies on) because
   * the tight loop is never entered while stopped — unlike the abandoned
   * "checked at function entry" attempt, which failed specifically because
   * the tight loop could run for a long time without returning here. */
  if (s_streaming_stopped)
  {
    /* Checked first — SET_CHANNELS/START_STREAMING shouldn't run until
     * ingestion is actually disabled and fifo0 is flushed, not just until
     * the STOP_STREAMING flag is set. s_command_busy already rejects any
     * new command that arrives before this completes, so this ordering is
     * enough on its own — no separate "confirmed stopped" flag needed. */
    if (s_stop_pending)
    {
      s_stop_pending = 0U;
      s_command_busy = 1U;
      FPGA_SPI_SetStreamEnable(0U);   /* disable ingestion FIRST ... */
      uint8_t enabled_after_stop = FPGA_SPI_ReadStreamEnable();
      if (enabled_after_stop != 0U)
      {
        /* Write didn't take — flushing now would race a still-active
         * producer again, the exact bug this whole redesign exists to
         * avoid. Flag loudly and skip the flush rather than silently
         * repeat it; ingestion (and therefore the flush) is deferred to
         * the next STOP_STREAMING attempt. */
        APP_DBG_MSG("STREAM: STOP_STREAMING — stream_enable readback=%u (expected 0), FIFO WRITE DID NOT TAKE — skipping flush\n",
                    enabled_after_stop);
      }
      else
      {
        StreamFlushFpgaFifo();          /* ... so this drains a static backlog, not a moving target */
      }
      s_command_busy = 0U;
      /* Real MCU confirmation, not a host-side timer — see
       * docs/interfaces/channel-selection-control-plane.md section 5.6. */
      STREAM_NotifyStreamingAck(STREAM_RESPONSE_TYPE_STOP_STREAMING,
                                 (enabled_after_stop == 0U) ? 1U : 0U,
                                 s_pending_conn_handle);
      BLE_STACK_Tick();
      return;
    }
    if (s_set_channels_pending)
    {
      s_set_channels_pending = 0U;
      s_command_busy = 1U;
      SetChannelsTask();
      s_command_busy = 0U;
      BLE_STACK_Tick();
      return;
    }
    /* Register console — docs/interfaces/fpga-diagnostic-access.md section 2.
     * Placed after SET_CHANNELS and before START_STREAMING for the same reason
     * SET_CHANNELS sits where it does: all regbank traffic must complete while
     * ingestion is still disabled and fifo0 still flushed. */
    if (s_reg_write_pending)
    {
      s_reg_write_pending = 0U;
      s_command_busy = 1U;
      FPGA_SPI_RegWrite16(s_pending_reg_addr, s_pending_reg_value);
      /* Automatic readback: the ack must carry what the regbank actually
       * holds, not what we sent. Also gives Tunki's rung scripts a free
       * per-write verification with no extra round trip, which is what makes a
       * 32-word sampling-table rewrite trustworthy. */
      uint16_t readback = FPGA_SPI_RegRead16(s_pending_reg_addr);
      s_command_busy = 0U;
      if (readback != s_pending_reg_value)
      {
        /* Always loud: a regbank that did not take the value is the one thing
         * on this path worth interrupting the radio for. */
        APP_DBG_MSG("STREAM: REG_WRITE16 addr=%u wrote=0x%04X readback=0x%04X  *** MISMATCH ***\n",
                    s_pending_reg_addr, s_pending_reg_value, readback);
      }
      else
      {
        REG_DBG_MSG("STREAM: REG_WRITE16 addr=%u = 0x%04X ok\n",
                    s_pending_reg_addr, readback);
      }
      STREAM_NotifyRegAccess(STREAM_RESPONSE_TYPE_REG_WRITE16,
                             s_pending_reg_addr, readback,
                             s_pending_conn_handle);
      BLE_STACK_Tick();
      return;
    }
    if (s_reg_read_pending)
    {
      s_reg_read_pending = 0U;
      s_command_busy = 1U;
      uint16_t value = FPGA_SPI_RegRead16(s_pending_reg_addr);
      s_command_busy = 0U;
      REG_DBG_MSG("STREAM: REG_READ16 addr=%u value=0x%04X\n", s_pending_reg_addr, value);
      STREAM_NotifyRegAccess(STREAM_RESPONSE_TYPE_REG_READ16,
                             s_pending_reg_addr, value,
                             s_pending_conn_handle);
      BLE_STACK_Tick();
      return;
    }
    if (s_start_streaming_pending)
    {
      s_start_streaming_pending = 0U;
      s_command_busy = 1U;
      FPGA_SPI_SetStreamEnable(1U);   /* re-enable ingestion — no flush needed, fifo0 has been empty since the stop-time flush */
      uint8_t enabled_after_start = FPGA_SPI_ReadStreamEnable();
      s_command_busy = 0U;
      s_streaming_stopped = 0U;
      /* Spec section 6.3/3.1: telemetry counters and sample_index are
       * cumulative *since START_STREAMING*. Reset here, after the FPGA is
       * actually ingesting again, so sample 0 of the new epoch is the first
       * sample the FPGA produced in it. */
      StreamResetTelemetryCounters();
      if (enabled_after_start != 1U)
      {
        /* Not fatal the way a failed disable is (worst case here: the tight
         * loop resumes and reads nothing but the empty-FIFO sentinel —
         * safe, already-handled, and immediately visible in Tunki's
         * underrun stats) — but still worth flagging loudly on the bench. */
        APP_DBG_MSG("STREAM: START_STREAMING — stream_enable readback=%u (expected 1), FIFO WRITE DID NOT TAKE\n",
                    enabled_after_start);
      }
      APP_DBG_MSG("STREAM: START_STREAMING — resuming\n");
      /* Real MCU confirmation, not a host-side timer — see
       * docs/interfaces/channel-selection-control-plane.md section 5.6. */
      STREAM_NotifyStreamingAck(STREAM_RESPONSE_TYPE_START_STREAMING,
                                 (enabled_after_start == 1U) ? 1U : 0U,
                                 s_pending_conn_handle);
      BLE_STACK_Tick();
      return;
    }
    /* Stopped, nothing pending — just idle. */
    BLE_STACK_Tick();
    return;
  }

  if (s_txFlowOff)
  {
#if STREAM_DIAG_CONTINUOUS_POLL_DURING_STALL
    StreamContinuousPollDuringStall();
#else
    StreamIngestDuringStall();
#endif
    return;
  }

  while (1)
  {
    /* Cheap check only (no SPI0/BLE work here, unlike the abandoned
     * SET_CHANNELS-in-loop attempt) — lets STOP_STREAMING take effect
     * within about one packet cycle instead of never, since this loop
     * otherwise only exits via flow_off. */
    if (s_streaming_stopped)
      return;

    StreamDataPacket_t pkt;
    uint32_t ts_s = 0U; uint16_t ts_sub = 0U;
    StreamGetTimestamp(&ts_s, &ts_sub);
    pkt.timestamp_s = ts_s; pkt.timestamp_sub_s = ts_sub;
    pkt.seq_num = seq_num; pkt.num_pairs = STREAM_PAIRS_PER_PACKET;

    StreamAssemblePacket(pkt.samples, STREAM_PAIRS_PER_PACKET);

    tBleStatus ret = STREAM_NotifyData(&pkt, STREAM_APP_Context.ConnectionHandle);
    if (ret == BLE_STATUS_INSUFFICIENT_RESOURCES)
      goto flow_off;
    if (ret != BLE_STATUS_SUCCESS)
      return;

    seq_num++;
    s_notify_total++;
    s_notifs_in_batch++;
    s_sample_index += STREAM_PAIRS_PER_PACKET * 2U;   /* aggregate samples, spec 3.1 */

    /* Flush HOST HCI buffer → LL TX queue without going through UTIL_SEQ. */
    BLE_STACK_Tick();
    continue;

flow_off:
    /* pkt.samples was already pulled out of the FPGA/ring buffer above, but
     * STREAM_NotifyData() just failed to send it — without this, those
     * STREAM_PAIRS_PER_PACKET samples would simply be discarded here, which
     * was the actual SKP root cause (confirmed by two negative experiments:
     * bounding vs. gap-free polling during the stall itself, neither of
     * which touches this transition). Save it into the ring buffer so
     * StreamAssemblePacket() sends it first on resume, same as backlog
     * ingested during the stall. */
    {
      uint32_t room = StreamRingFree();
      uint32_t n = STREAM_PAIRS_PER_PACKET;
      if (n > room)
        n = room;
      if (n > 0U)
        StreamRingPush(pkt.samples, n);
      /* The clamp above is the headstage's only silent MCU-side discard: these
       * pairs were already popped out of fifo0, so they exist nowhere else.
       * Reachable only with the ring already full, i.e. deep into a stall —
       * rare, but counting it is what lets spec section 7 attribute a gap to
       * the MCU ring instead of to the air. 2 samples per pair (aggregate,
       * spec section 3.1). */
      if (n < STREAM_PAIRS_PER_PACKET)
        s_ring_truncated_samples += (STREAM_PAIRS_PER_PACKET - n) * 2U;
    }
    s_txFlowOff = 1U;
    s_flowoff_total++;
    StreamGetTimestamp(&s_flowoff_ts_s, &s_flowoff_ts_sub);
    /* PA10 no longer toggled here — repurposed as the fifo_full EXTI input,
     * see STREAM_APP_Init. This event is still fully timestamped below. */
    APP_DBG_MSG("STREAM: flow-off #%u (batch=%u tot=%u)\n",
                s_flowoff_total, s_notifs_in_batch, s_notify_total);
    s_notifs_in_batch = 0U;
#if STREAM_DIAG_POST_DRAIN_WATCH
    StreamDrainWatchRecordEvt(1U);
#endif
    return;
  }
}

/* -------------------------------------------------------------------------- */
/*  RTC timestamp helper                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Return a high-resolution timestamp from the LSE-clocked RTC.
 *
 * @param  pSeconds    Output: elapsed seconds since reset (H×3600 + M×60 + S).
 * @param  pSubSeconds Output: sub-second value in 0..31 999 (32 000 ticks/s).
 *                     Each unit = 31.25 µs.  Android formula:
 *                       timestamp_us = seconds×1 000 000 + sub_s×1000/32
 *
 * @note   RTC is configured with AsynchPrediv=0, SynchPrediv=32767:
 *           f_ck_apre = 32 768 Hz,  f_ck_spre = 1 Hz (calendar)
 *         SSR counts DOWN from 32 767 to 0 within each second.
 *         We flip and rescale to 32 000 ticks/s for the Android parser:
 *           sub_s = (32767 - SSR) × 32000 / 32768    →  0..31 999
 *         Resolution: 1/32 768 s ≈ 30.5 µs.
 *
 * @note   BYPSHAD is set in MX_RTC_Init (RTC_CR_BYPSHAD): HAL_RTC_GetTime
 *         reads the counter registers directly — no RSF wait, non-blocking.
 *         HAL_RTC_GetDate must be called after GetTime to unlock the
 *         read sequence even though the date value is unused here.
 */
static void StreamGetTimestamp(uint32_t *pSeconds, uint16_t *pSubSeconds)
{
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  /* BYPSHAD set → direct counter reads, no scheduler stall. */
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);  /* mandatory to complete read sequence */

  *pSeconds = (uint32_t)sTime.Hours   * 3600U
            + (uint32_t)sTime.Minutes * 60U
            + (uint32_t)sTime.Seconds;

  /* SSR counts DOWN; flip to up-counting and rescale 32768→32000 ticks/s. */
  *pSubSeconds = (uint16_t)((32767U - sTime.SubSeconds) * 32000U / 32768U);
}

/* -------------------------------------------------------------------------- */
/*  CCCD state callback (called from app_ble.c)                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Called when Android writes the CCCD on the 0xFFF2 notify characteristic.
 *
 *  Bit 0 of cccd_value = notification enable flag (0x0001 = enabled).
 *
 *  On first enable:
 *    - seq_num is reset to 0 so Android always sees a clean sequence at the
 *      start of every connection (no artificial gaps caused by failed sends
 *      that happened before CCCD was enabled).
 *    - Sending starts immediately on the next VTIMER fire.
 *
 *  On disable / disconnect:
 *    - IsCCCDEnabled = 0, sending stops (ring buffer keeps accumulating).
 */
void STREAM_APP_OnCCCDWrite(uint16_t conn_handle, uint16_t cccd_value)
{
  if (cccd_value & 0x0001U)
  {
    STREAM_APP_Context.IsCCCDEnabled = 1U;
    seq_num          = 0U;
    s_txFlowOff      = 0U;   /* clear any stale flow-off from previous connection */
    /* Covers s_notifs_in_batch/s_flowoff_total/s_resume_total/s_notify_total
     * plus the A.7 telemetry counters, so the first telemetry frame of a
     * connection describes this connection and nothing before it. */
    StreamResetTelemetryCounters();
    APP_DBG_MSG("STREAM: CCCD enabled (conn=0x%04X) – seq+counter reset to 0\n", conn_handle);

    /* CFG_LPM_FPGA_SPI was defined (app_conf.h) but never actually wired to
     * the low-power manager — nothing was blocking DEEPSTOP during active
     * streaming despite the comment saying it should. Block STOP/OFF mode
     * for as long as streaming is active: entering DEEPSTOP here stalls
     * StreamSendTask (and the periodic safety-fallback timer) for several ms
     * on wake, which is the dominant cause of SKP events (see 2026-07-29/30
     * scope captures — out_ptr[0] frozen for ~10ms beyond the logged
     * flow-off/resume window, with zero StreamSendTask invocations in
     * between). */
    UTIL_LPM_SetStopMode(1U << CFG_LPM_FPGA_SPI, UTIL_LPM_DISABLE);
    UTIL_LPM_SetOffMode(1U << CFG_LPM_FPGA_SPI, UTIL_LPM_DISABLE);

    /* Prime the event-driven pipeline immediately. */
    UTIL_SEQ_SetTask(1UL << CFG_TASK_STREAM_SEND_ID, CFG_SEQ_PRIO_1);
  }
  else
  {
    STREAM_APP_Context.IsCCCDEnabled = 0U;
    APP_DBG_MSG("STREAM: CCCD disabled\n");

    /* Streaming stopped — safe to allow DEEPSTOP again. */
    UTIL_LPM_SetStopMode(1U << CFG_LPM_FPGA_SPI, UTIL_LPM_ENABLE);
    UTIL_LPM_SetOffMode(1U << CFG_LPM_FPGA_SPI, UTIL_LPM_ENABLE);
  }
}

/* -------------------------------------------------------------------------- */
/*  0xFFF1 command handler (called from app_ble.c / stream.c)                */
/* -------------------------------------------------------------------------- */

#define CMD_SET_CHANNELS      0x01U
#define CMD_STOP_STREAMING    0x02U
#define CMD_START_STREAMING   0x03U
/* Register console — docs/interfaces/fpga-diagnostic-access.md section 2.
 * Same validate-stash-defer shape and the same stopped-stream precondition as
 * SET_CHANNELS; no new hazard, same SPI0 primitives on the same deferred path. */
#define CMD_REG_WRITE16       0x04U
#define CMD_REG_READ16        0x05U


/* Safety bound for StreamFlushFpgaFifo() below — NOT the same as the FIFO's
 * raw depth (4096 pairs). The FPGA keeps ingesting real samples at 30 kSPS
 * throughout the flush, so draining a fully-saturated FIFO (worst case,
 * hardware-capped at 4096) to true empty needs more than 4096 pops: at the
 * measured ~10us/pair SPI0 pop rate (100k pairs/s) against a 30k pairs/s
 * fill rate, net drain is 70k pairs/s, so ~4096/(70000/100000) ~= 5850 pops
 * are actually needed from full. 2026-08-06: the cap was originally set to
 * 4096 (the FIFO's raw depth) without accounting for this — the flush hit
 * the cap and gave up every single time in bench testing, never actually
 * reaching empty. 16384 gives ~2.8x margin over the ~5850 estimate against
 * the pop-rate estimate being off; worst case is still only ~164 ms of
 * flush time, comfortably inside the ~1s tolerance for this operation. */
#define FPGA_FIFO_MAX_PAIRS   16384U
#define FIFO_EMPTY_SENTINEL   ((int16_t)0x8000)

/* s_pending_ch_a/ch_b/conn_handle/s_set_channels_pending/s_streaming_stopped/
 * s_start_streaming_pending are declared near the top of this file (with
 * s_txFlowOff) — StreamSendTask() references them and is defined earlier in
 * the file than this section. Full design history there and in
 * docs/interfaces/channel-selection-control-plane.md section 5. */

/**
 * @brief  Deferred SET_CHANNELS work — the actual SPI0 write+readback.
 *         Called from StreamSendTask()'s stopped-branch when
 *         s_set_channels_pending is set. Only reachable while
 *         s_streaming_stopped is set (see STREAM_APP_OnCommandWrite's
 *         precondition check below) — never runs concurrently with the
 *         tight per-packet loop, and never inline in the BLE GATT
 *         event-handler callback (that placement hung the MCU — history in
 *         the comment above s_pending_ch_a).
 */
static void SetChannelsTask(void)
{
  APP_DBG_MSG("STREAM: SetChannelsTask running, ch_a=%u ch_b=%u\n", s_pending_ch_a, s_pending_ch_b);
  FPGA_SPI_SetChannels(FPGA_SPI_ChannelToRaw(s_pending_ch_a), FPGA_SPI_ChannelToRaw(s_pending_ch_b));
  APP_DBG_MSG("STREAM: SET_CHANNELS ch_a=%u ch_b=%u\n", s_pending_ch_a, s_pending_ch_b);

  /* Readback verification — see
   * docs/interfaces/channel-selection-control-plane.md section 4.
   * The RTL's REG_READ decode landed 2026-08-06 and the read became
   * self-addressing with A.1.1g, so this returns real regbank contents;
   * the earlier "stale/undefined until the RTL lands" caveat is obsolete. */
  uint8_t rb_a_raw, rb_b_raw;
  FPGA_SPI_ReadChannels(&rb_a_raw, &rb_b_raw);
  uint8_t rb_a = FPGA_SPI_RawToChannel(rb_a_raw);
  uint8_t rb_b = FPGA_SPI_RawToChannel(rb_b_raw);
  STREAM_NotifyChannelsReadback(rb_a, rb_b, s_pending_conn_handle);
}

/**
 * @brief  Pop-and-discard FPGA FIFO contents until the empty-FIFO sentinel
 *         is observed, or FPGA_FIFO_MAX_PAIRS as a safety bound. Called
 *         from StreamSendTask()'s stopped-branch on START_STREAMING — see
 *         docs/interfaces/channel-selection-control-plane.md section 5.3
 *         for why this discards rather than preserves (pre-switch data,
 *         and avoids feeding the open FPGA-FIFO chronic-backlog question).
 *         Ticks the BLE stack periodically rather than once at the end, to
 *         avoid one long uninterrupted blocking stretch in the worst case.
 */
/* Batched: FPGA_SPI_ReadSamples() costs 2*n+1 transfers since A.1.1g (the
 * trailing NOP closes the POP pair so the call leaves the FPGA FSM clean).
 * Called one pair at a time that is 3 transfers per pair instead of 2 — a 50%
 * flush-time regression on a blocking operation. A batch of 32 amortises the
 * NOP to 65 transfers per 32 pairs, i.e. ~2.03 per pair.
 *
 * Over-popping past the sentinel is harmless: once the FIFO is empty the RTL
 * returns 0x8000/0x8000 without advancing out_ptr, so the extra pops in the
 * final batch consume nothing. */
static void StreamFlushFpgaFifo(void)
{
  enum { FLUSH_BATCH = 32U };
  int16_t  batch[2U * FLUSH_BATCH];
  uint32_t n = 0U;
  uint8_t  empty = 0U;

  while ((empty == 0U) && (n < FPGA_FIFO_MAX_PAIRS))
  {
    FPGA_SPI_ReadSamples(batch, (uint8_t)FLUSH_BATCH);

    for (uint32_t i = 0U; i < (uint32_t)FLUSH_BATCH; i++)
    {
      if ((batch[i * 2U] == FIFO_EMPTY_SENTINEL) &&
          (batch[i * 2U + 1U] == FIFO_EMPTY_SENTINEL))
      {
        empty = 1U;
        break;
      }
      n++;
    }

    /* Tick per batch rather than once at the end, to avoid one long
     * uninterrupted blocking stretch in the worst case. */
    BLE_STACK_Tick();
  }

  /* n counts real pairs only — the sentinel pair that ended the loop was
   * never a discarded sample. (The previous per-pair version reported n+1,
   * which counted it.) */
  APP_DBG_MSG("STREAM: FIFO flush discarded %lu pair(s)\n", (unsigned long)n);
}

/**
 * @brief  Called when the central writes the 0xFFF1 command characteristic.
 *         See docs/interfaces/channel-selection-control-plane.md sections 2
 *         and 5.
 *
 *  SET_CHANNELS (0x01) now requires streaming to already be stopped —
 *  rejected and logged otherwise. STOP_STREAMING (0x02) and
 *  START_STREAMING (0x03) are new. Unknown cmd, wrong length, or an
 *  out-of-range channel index is logged and ignored — never partially
 *  applied. The original reason given was the even-transfer-count pairing
 *  invariant, which was retired 2026-08-11 (A.1.1g; confirmed in simulation,
 *  spec section 1a). All-or-nothing still stands on its own merits, and more
 *  strongly than before: under the A.1.1g FSM a partially-applied
 *  SET_CHANNELS would abandon a 3-transfer tagged write sequence mid-way,
 *  and the FPGA would then abort the next unrelated command's first transfer
 *  on a tag mismatch. Validate fully, then act — or do nothing.
 *
 *  Deliberately does no SPI0 work itself for any of these — see
 *  SetChannelsTask() / StreamFlushFpgaFifo() above for why. This function
 *  must stay cheap: validate, stash a pending flag for StreamSendTask() to
 *  pick up (or, for STOP_STREAMING, just set a flag directly — no SPI0/BLE
 *  work involved in that one at all), return.
 */
void STREAM_APP_OnCommandWrite(uint16_t conn_handle, const uint8_t *p_data, uint8_t length)
{
  /* Was unconditional. Quiet for the two high-rate console opcodes only;
   * the operator-facing commands still announce themselves. */
  if (length > 0U && (p_data[0] == CMD_REG_WRITE16 || p_data[0] == CMD_REG_READ16))
  {
    REG_DBG_MSG("STREAM: OnCommandWrite entered, len=%u cmd=0x%02X\n",
                length, p_data[0]);
  }
  else
  {
    APP_DBG_MSG("STREAM: OnCommandWrite entered, len=%u cmd=0x%02X\n",
                length, length > 0U ? p_data[0] : 0U);
  }

  if (length == 0U)
  {
    APP_DBG_MSG("STREAM: empty 0xFFF1 command, ignored\n");
    return;
  }

  /* Reject any command while a previous one is still being processed —
   * see the comment above s_command_busy for the real bug this prevents
   * (a new command overwriting pending state mid-use by an in-progress
   * SetChannelsTask()/StreamFlushFpgaFifo() call). One full STOP/SET/START
   * cycle is atomic; the operator/Tunki is expected to wait for this one
   * to finish (readback, or Tunki's own timeout) before starting the
   * next. */
  if (s_command_busy)
  {
    APP_DBG_MSG("STREAM: 0xFFF1 command 0x%02X rejected — still processing a previous one\n",
                p_data[0]);
    return;
  }

  /* Captured unconditionally (not just for SET_CHANNELS) — STOP_STREAMING/
   * START_STREAMING's acks (section 5.6) need it too, and there's only ever
   * one active connection in this system. */
  s_pending_conn_handle = conn_handle;

  switch (p_data[0])
  {
  case CMD_SET_CHANNELS:
    if (length != 3U)
    {
      APP_DBG_MSG("STREAM: malformed SET_CHANNELS (len=%u), ignored\n", length);
      break;
    }
    {
      uint8_t ch_a = p_data[1];
      uint8_t ch_b = p_data[2];
      if (ch_a > 127U || ch_b > 127U)
      {
        APP_DBG_MSG("STREAM: SET_CHANNELS out of range (ch_a=%u ch_b=%u), ignored\n", ch_a, ch_b);
      }
      else if (!s_streaming_stopped)
      {
        APP_DBG_MSG("STREAM: SET_CHANNELS rejected — streaming not stopped (STOP_STREAMING first)\n");
      }
      else
      {
        s_pending_ch_a         = ch_a;
        s_pending_ch_b         = ch_b;
        s_set_channels_pending = 1U;
        APP_DBG_MSG("STREAM: SET_CHANNELS scheduled (ch_a=%u ch_b=%u)\n", ch_a, ch_b);
      }
    }
    break;

  case CMD_STOP_STREAMING:
    if (s_streaming_stopped)
    {
      APP_DBG_MSG("STREAM: STOP_STREAMING ignored — already stopped\n");
    }
    else
    {
      s_streaming_stopped = 1U;
      s_stop_pending = 1U;
      APP_DBG_MSG("STREAM: STOP_STREAMING received\n");
    }
    break;

  /* Register console — docs/interfaces/fpga-diagnostic-access.md section 2.
   * Raw and unvalidated by design: no address range check (all 256 words are
   * legitimately reachable, and the RTL has no write protection either), no
   * friendly-index mapping. The operator path above keeps its validation; a
   * diagnostic path must not be able to loosen it, which is why these are
   * separate commands rather than an extension of SET_CHANNELS. */
  case CMD_REG_WRITE16:
    if (length != 4U)
    {
      APP_DBG_MSG("STREAM: malformed REG_WRITE16 (len=%u), ignored\n", length);
    }
    else if (!s_streaming_stopped)
    {
      APP_DBG_MSG("STREAM: REG_WRITE16 rejected — streaming not stopped (STOP_STREAMING first)\n");
    }
    else
    {
      s_pending_reg_addr  = p_data[1];
      s_pending_reg_value = (uint16_t)(p_data[2] | ((uint16_t)p_data[3] << 8));
      s_reg_write_pending = 1U;
      REG_DBG_MSG("STREAM: REG_WRITE16 scheduled (addr=%u value=0x%04X)\n",
                  s_pending_reg_addr, s_pending_reg_value);
    }
    break;

  case CMD_REG_READ16:
    if (length != 2U)
    {
      APP_DBG_MSG("STREAM: malformed REG_READ16 (len=%u), ignored\n", length);
    }
    else if (!s_streaming_stopped)
    {
      APP_DBG_MSG("STREAM: REG_READ16 rejected — streaming not stopped (STOP_STREAMING first)\n");
    }
    else
    {
      s_pending_reg_addr = p_data[1];
      s_reg_read_pending = 1U;
      REG_DBG_MSG("STREAM: REG_READ16 scheduled (addr=%u)\n", s_pending_reg_addr);
    }
    break;

  case CMD_START_STREAMING:
    if (!s_streaming_stopped)
    {
      APP_DBG_MSG("STREAM: START_STREAMING ignored — already streaming\n");
    }
    else
    {
      s_start_streaming_pending = 1U;
      APP_DBG_MSG("STREAM: START_STREAMING scheduled\n");
    }
    break;

  default:
    APP_DBG_MSG("STREAM: unknown 0xFFF1 command 0x%02X (len=%u), ignored\n", p_data[0], length);
    break;
  }
}

/**
 * @brief  Called from the ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE handler in app_ble.c.
 *         Clears the flow-off flag and re-starts the streaming pipeline.
 */
uint32_t STREAM_APP_GetNotifyTotal(void)
{
  return s_notify_total;
}

void STREAM_APP_ResumeSending(void)
{
  s_resume_total++;

  /* Measure how long the TX pool took to free up — entirely a radio/link-layer
   * duration, not application timing. A large value here means the controller
   * needed retries to drain its queue (interference/congestion), which is
   * exactly the kind of stall that stops FPGA_SPI_ReadSamples() from being
   * called at all, independent of any FPGA-side or NOP-margin timing. */
  uint32_t now_s; uint16_t now_sub;
  StreamGetTimestamp(&now_s, &now_sub);
  /* PA10 no longer toggled here — repurposed as the fifo_full EXTI input,
   * see STREAM_APP_Init. This event is still fully timestamped below. */
  int32_t  ds       = (int32_t)(now_s - s_flowoff_ts_s);
  int32_t  dsub     = (int32_t)now_sub - (int32_t)s_flowoff_ts_sub;
  uint32_t stall_us = (uint32_t)((int64_t)ds * 1000000 + (int64_t)dsub * 1000 / 32);
  if (stall_us > s_longest_stall_us)
    s_longest_stall_us = stall_us;
  /* Cumulative stall time, carried in microseconds so sub-millisecond resumes
   * are not each truncated to zero — see s_stall_time_ms_total's declaration. */
  s_stall_us_remainder += stall_us;
  s_stall_time_ms_total += s_stall_us_remainder / 1000U;
  s_stall_us_remainder  %= 1000U;
  /* Only print outliers (>3ms, well above the normal sub-ms resume latency) —
   * printing every resume at ~133 Hz would flood the UART and hide the signal.
   * now_s/now_sub use the same RTC encoding as the packet header timestamp
   * (StreamGetTimestamp), so this can be matched directly against a specific
   * SKP event's timestamp_us in a simultaneous Tunki recording — not just
   * compared in aggregate. */
  if (stall_us > 3000U)
  {
    APP_DBG_MSG("STREAM: LONG STALL - flow-off #%u took %lu us to resume (longest=%lu us) resumed_at=%lu.%05u\n",
                s_flowoff_total, (unsigned long)stall_us, (unsigned long)s_longest_stall_us,
                (unsigned long)now_s, now_sub);
  }

  /* Log every 500 resumes (~every 3.75 s at 133 Hz) to avoid UART saturation. */
  if ((s_resume_total % 500U) == 1U)
  {
    APP_DBG_MSG("STREAM: pool-avail #%u  flowoffs=%u  notifs=%u  batch_now=%u\n",
                s_resume_total, s_flowoff_total, s_notify_total, s_notifs_in_batch);
  }
#if STREAM_DIAG_POST_DRAIN_WATCH
  StreamDrainWatchRecordEvt(2U);
#endif
  s_txFlowOff = 0U;
  UTIL_SEQ_SetTask(1UL << CFG_TASK_STREAM_SEND_ID, CFG_SEQ_PRIO_1);
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/
