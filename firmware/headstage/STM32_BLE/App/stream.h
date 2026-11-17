/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stream.h
  * @author  Manuel's Team
  * @brief   Header for stream.c - High-throughput data streaming service
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STREAM_H
#define STREAM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ble_status.h"

/* Exported defines ----------------------------------------------------------*/
/* Maximum payload size for streaming data */
#define STREAM_MAX_PAYLOAD_SIZE     247

/* Notify characteristic packet layout (244 bytes, fits ATT MTU 247):
 *   Header  :  8 bytes
 *   Payload : 59 pairs × (int16_t ch0 + int16_t ch1) = 59 × 4 = 236 bytes
 *   Total   : 244 bytes
 *
 *   Service  0xFFF0  |  Write char 0xFFF1 (phone→STM32)
 *                    |  Notify char 0xFFF2 (STM32→phone, data)
 *                    |  Notify char 0xFFF3 (STM32→phone, command responses)
 *                    |  Notify char 0xFFF4 (STM32→phone, telemetry — see
 *                    |    docs/interfaces/stream-packet-format.md section 6)
 */
#define STREAM_PAIRS_PER_PACKET     (59U)
#define STREAM_NOTIFY_PAYLOAD_SIZE  (8U + STREAM_PAIRS_PER_PACKET * 4U)  /* 8 + 236 = 244 bytes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  STREAM_DATA,

  /* USER CODE BEGIN Service_CharOpcode_t */
  STREAM_NOTIFY,        /**< 0xFFF2 – STM32→phone data notification */
  /* USER CODE END Service_CharOpcode_t */

  STREAM_CHAROPCODE_LAST
} STREAM_CharOpcode_t;

typedef enum
{
  STREAM_WRITE_ENABLED_EVT,

  /* USER CODE BEGIN Service_OpcodeEvt_t */

  /* USER CODE END Service_OpcodeEvt_t */

  STREAM_BOOT_REQUEST_EVT
} STREAM_OpcodeEvt_t;

typedef struct
{
  uint8_t *p_Payload;
  uint8_t Length;

  /* USER CODE BEGIN Service_Data_t */

  /* USER CODE END Service_Data_t */

} STREAM_Data_t;

typedef struct
{
  STREAM_OpcodeEvt_t       EvtOpcode;
  STREAM_Data_t            DataTransfered;
  uint16_t                 ConnectionHandle;
  uint16_t                 AttributeHandle;
  uint8_t                  ServiceInstance;

  /* USER CODE BEGIN Service_NotificationEvt_t */

  /* USER CODE END Service_NotificationEvt_t */

} STREAM_NotificationEvt_t;

/* USER CODE BEGIN ET */

/**
 * BLE notification packet sent on characteristic 0xFFF2.
 *
 * Header (8 bytes):
 *   timestamp_s     – RTC wall-clock seconds (HH*3600 + MM*60 + SS)
 *   timestamp_sub_s – RTC SSR sub-second count (0-32767 ≙ 0..~1 s at 32.768 kHz LSE)
 *   seq_num         – Rolling packet counter; receiver detects gaps = dropped packets
 *   num_pairs       – Valid sample pairs in this packet (normally STREAM_PAIRS_PER_PACKET)
 *
 * Payload (59 pairs × 4 bytes = 236 bytes, little-endian):
 *   ch0[0], ch1[0], ch0[1], ch1[1], ...   (interleaved signed int16_t, 30 kSps per channel)
 */
/* packed alone drops the struct's own alignment to 1 byte in GCC, which
 * lets a local StreamDataPacket_t land at an odd stack address — risky on
 * Cortex-M0/M0+, which has no hardware unaligned-access support (unaligned
 * access to e.g. &pkt.samples silently HardFaults into a while(1)).
 * aligned(4) forces the struct's own base address to always be 4-byte
 * aligned regardless of surrounding stack layout, without changing the
 * wire format (packed still removes inter-field padding).
 *
 * 2026-08-03: a first attempt at this exact fix was tried and reverted the
 * same day after bisection seemed to show the "MCU silent" symptom
 * persisted even with it applied — but that investigation was confounded:
 * the real cause of the MCU-silent symptom turned out to be unrelated
 * (PA10, an STM32WB09xE boot-strap pin, being hijacked into the ROM
 * bootloader by a chattering FPGA signal — see
 * memory/project_pa10_boot_strap_hijack.md, fixed same day). With that
 * confound removed and root-caused separately, re-applying this fix now
 * that the -Waddress-of-packed-member warnings are otherwise real and
 * still firing on every build (see memory/project_skp_investigation.md). */
typedef struct __attribute__((packed, aligned(4)))
{
  uint32_t  timestamp_s;                          /**< Seconds component of RTC time     */
  uint16_t  timestamp_sub_s;                      /**< Sub-second ticks (32.768 kHz LSE) */
  uint8_t   seq_num;                              /**< Rolling packet sequence number     */
  uint8_t   num_pairs;                            /**< Number of sample pairs             */
  int16_t   samples[STREAM_PAIRS_PER_PACKET * 2]; /**< 59 pairs: ch0[0],ch1[0],ch0[1],ch1[1],… signed int16_t */
} StreamDataPacket_t;

/* ── Telemetry frame, characteristic 0xFFF4 ────────────────────────────────
 * docs/interfaces/stream-packet-format.md section 6. Cumulative counters plus
 * the RTC time anchor, ~1 Hz, off the per-packet path.
 *
 * This struct is the MCU's IN-MEMORY view only. The wire layout is section
 * 6.2's byte table, serialised byte-wise by STREAM_NotifyTelemetry() — the
 * uint32 fields sit at offsets 2/6/18/22/26, none of them 4-byte aligned, so
 * this struct is deliberately NOT the wire format and must never be memcpy'd
 * onto it. (Same hazard class as the aligned(4) note above: an unaligned
 * uint32 access HardFaults on the M0+.) */
#define STREAM_TELEMETRY_VERSION      (1U)

/* MCU half of the section 6.2 frame — bytes 0..29. The bridge appends its own
 * 8 bytes (tx_ring_drop_bytes/frames) before emitting 0xDD 0x22 to the PC. */
#define STREAM_TELEMETRY_PAYLOAD_SIZE (30U)

/* flags, byte 1 — section 6.2. Bit 0 set means the two fifo0_* fields were
 * actually read from the regbank for this frame; clear means they were not,
 * and both are transmitted as 0 for a receiver that must not display them.
 * "Cannot be read" and "read as zero" are different facts, and section 7's
 * attribution table is unusable if they look alike. */
#define STREAM_TELEMETRY_FLAG_FPGA_COUNTERS_VALID  (0x01U)

typedef struct
{
  uint8_t   flags;                    /**< STREAM_TELEMETRY_FLAG_* bits        */
  uint32_t  anchor_sample_index;      /**< aggregate samples since START_STREAMING */
  uint32_t  anchor_timestamp_s;       /**< RTC seconds, same encoding as the packet header */
  uint16_t  anchor_timestamp_sub_s;   /**< RTC sub-seconds, 0..31999           */
  uint32_t  fifo0_overflow_samples;   /**< FPGA regbank; 0 unless flags bit 0  */
  uint16_t  fifo0_high_water;         /**< FPGA regbank; 0 unless flags bit 0  */
  uint32_t  ring_truncated_samples;   /**< MCU ring push clamped at flow-off   */
  uint32_t  flow_off_count;           /**< s_flowoff_total                     */
  uint32_t  stall_time_ms_total;      /**< cumulative time with s_txFlowOff set */
} StreamTelemetry_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ------------------------------------------------------- */
void STREAM_Init(void);
void STREAM_Notification(STREAM_NotificationEvt_t *p_Notification);
tBleStatus STREAM_SendData(STREAM_Data_t *pData, uint16_t ConnectionHandle);

/* USER CODE BEGIN EF */
/**
 * @brief  Send a data packet via BLE notification on characteristic 0xFFF2.
 * @param  pPacket          Pointer to a fully-populated StreamDataPacket_t.
 * @param  ConnectionHandle Active connection handle (0xFFFF = not connected).
 * @retval BLE status code.
 */
tBleStatus STREAM_NotifyData(StreamDataPacket_t *pPacket, uint16_t ConnectionHandle);

/**
 * @brief  Return the GATT declaration handle of the 0xFFF2 notify characteristic.
 *         The CCCD descriptor handle is this value + 2.
 */
uint16_t STREAM_GetNotifyCharHandle(void);

/**
 * @brief  Notify the SET_CHANNELS readback result on 0xFFF3 — see
 *         docs/interfaces/channel-selection-control-plane.md section 4.3.
 * @param  ch_a_friendly    Friendly index (0-127) read back from the FPGA regbank.
 * @param  ch_b_friendly    Friendly index (0-127) read back from the FPGA regbank.
 * @param  ConnectionHandle Active connection handle (0xFFFF = not connected).
 * @retval BLE status code.
 */
tBleStatus STREAM_NotifyChannelsReadback(uint8_t ch_a_friendly, uint8_t ch_b_friendly,
                                          uint16_t ConnectionHandle);

/* Response type byte values — see docs/interfaces/channel-selection-control-plane.md
 * section 5.6. Intentionally match the 0xFFF1 command opcodes. */
#define STREAM_RESPONSE_TYPE_STOP_STREAMING  0x02U
#define STREAM_RESPONSE_TYPE_START_STREAMING 0x03U

/**
 * @brief  Notify STOP_STREAMING/START_STREAMING completion on 0xFFF3 — see
 *         docs/interfaces/channel-selection-control-plane.md section 5.6.
 * @param  cmd              STREAM_RESPONSE_TYPE_STOP_STREAMING or
 *                           STREAM_RESPONSE_TYPE_START_STREAMING.
 * @param  success          1 if the stream_enable write was confirmed via
 *                           readback, 0 otherwise.
 * @param  ConnectionHandle Active connection handle (0xFFFF = not connected).
 * @retval BLE status code.
 */
tBleStatus STREAM_NotifyStreamingAck(uint8_t cmd, uint8_t success, uint16_t ConnectionHandle);

/* Register-console response types — docs/interfaces/fpga-diagnostic-access.md
 * section 2. Same convention as the two above: the type byte equals the 0xFFF1
 * command opcode it answers. */
#define STREAM_RESPONSE_TYPE_REG_WRITE16     0x04U
#define STREAM_RESPONSE_TYPE_REG_READ16      0x05U

/**
 * @brief  Notify a register-console result on 0xFFF3.
 * @param  type             STREAM_RESPONSE_TYPE_REG_WRITE16 or
 *                           STREAM_RESPONSE_TYPE_REG_READ16.
 * @param  addr             Regbank RAM word address, echoed back.
 * @param  value            Value READ BACK from that word — for REG_WRITE16
 *                           too, so an ack means "the regbank holds this",
 *                           not "the write call returned".
 * @param  ConnectionHandle Active connection handle (0xFFFF = not connected).
 * @retval BLE status code.
 */
tBleStatus STREAM_NotifyRegAccess(uint8_t type, uint8_t addr, uint16_t value,
                                  uint16_t ConnectionHandle);

/**
 * @brief  Notify one telemetry frame on 0xFFF4 — see
 *         docs/interfaces/stream-packet-format.md section 6.
 *         Serialises pTelemetry little-endian byte-wise into section 6.2's
 *         layout (bytes 0..29); telemetry_version is written by this
 *         function, the caller fills everything else.
 * @param  pTelemetry       Fully-populated counter snapshot.
 * @param  ConnectionHandle Active connection handle (0xFFFF = not connected).
 * @retval BLE status code. A failure here is not an error worth escalating:
 *         counters are cumulative, so a lost frame costs resolution, not
 *         information (section 6.3).
 */
tBleStatus STREAM_NotifyTelemetry(const StreamTelemetry_t *pTelemetry,
                                  uint16_t ConnectionHandle);
/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*STREAM_H */
