/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stream.c
  * @author  Manuel's Team
  * @brief   STREAM service definition - High-throughput data streaming
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

/* Includes ------------------------------------------------------------------*/
#include <app_common.h>
#include "ble.h"
#include "stream.h"

/* USER CODE BEGIN Includes */
#include "ble_evt.h"
#include "stream_app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

typedef struct{
  uint16_t  StreamSvcHdle;           /**< Stream Service Handle */
  uint16_t  StreamDataCharHdle;      /**< Stream Data Characteristic Handle (0xFFF1, write) */
/* USER CODE BEGIN Context */
  uint16_t  StreamNotifyCharHdle;    /**< Stream Notify Characteristic Handle (0xFFF2, notify) */
  uint16_t  StreamResponseCharHdle;  /**< Stream Response Characteristic Handle (0xFFF3, notify) */
  uint16_t  StreamTelemetryCharHdle; /**< Stream Telemetry Characteristic Handle (0xFFF4, notify) */
/* USER CODE END Context */
}STREAM_Context_t;

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN PD */
#define STREAM_DATA_SIZE        STREAM_MAX_PAYLOAD_SIZE  /* Stream Data Characteristic max size */
/* USER CODE END PD */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private macros ------------------------------------------------------------*/
#define CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET        2
#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET             1
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

static STREAM_Context_t STREAM_Context;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
static BLEEVT_EvtAckStatus_t STREAM_EventHandler(aci_blecore_event *p_evt);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
/* USER CODE BEGIN FD */

/* USER CODE END FD */

/* Private functions ----------------------------------------------------------*/

/*
 * UUIDs for Streaming service (vendor-specific 0xFFF0 range)
 *   0xFFF0  Service
 *   0xFFF1  Write-without-response  (phone → STM32, command frames)
 *   0xFFF2  Notify                  (STM32 → phone, 244-byte StreamDataPacket_t)
 *   0xFFF3  Notify                  (STM32 → phone, command-response — see
 *                                    docs/interfaces/channel-selection-control-plane.md
 *                                    section 4.3)
 *   0xFFF4  Notify                  (STM32 → phone, ~1 Hz telemetry frame — see
 *                                    docs/interfaces/stream-packet-format.md
 *                                    section 6)
 */
#define STREAM_UUID                 (0xFFF0)
#define STREAM_DATA_UUID            (0xFFF1)
#define STREAM_NOTIFY_UUID          (0xFFF2)
#define STREAM_RESPONSE_UUID        (0xFFF3)
/* Telemetry gets its own characteristic rather than reusing 0xFFF3 with an
 * unsolicited opcode: 0xFFF3's contract is request/response, and unsolicited
 * traffic there would be correct only by convention. A characteristic handle
 * is enforced by the GATT layer, so the bridge demultiplexes structurally.
 * Full reasoning: docs/interfaces/stream-packet-format.md section 6.1. */
#define STREAM_TELEMETRY_UUID       (0xFFF4)

/* 0xFFF3 payload, max size across all response types — see
 * docs/interfaces/channel-selection-control-plane.md section 5.6:
 *   SET_CHANNELS readback:        [0x01, ch_a_friendly, ch_b_friendly]  (3 bytes)
 *   STOP/START_STREAMING ack:     [cmd, success]                       (2 bytes)
 * First byte is always the type/cmd — same values as the 0xFFF1 command
 * opcodes (section 2, section 5.1), so a response always echoes which
 * command it's answering. */
/* Widest 0xFFF3 payload. 3 → 4 for the register console's
 * [type, addr, val_lo, val_hi] — docs/interfaces/fpga-diagnostic-access.md §2.
 * The shorter responses are simply shorter notifies on the same
 * characteristic. */
#define STREAM_RESPONSE_PAYLOAD_SIZE  (4U)

/* --- 0xFFF1 write characteristic value buffer --- */
#define STREAM_DATA_SIZE            STREAM_MAX_PAYLOAD_SIZE
uint8_t stream_data_buffer[STREAM_DATA_SIZE];

static ble_gatt_val_buffer_def_t stream_data_buffer_def = {
  /* MODIFIED_EVT_ENABLE_FLAG required so writes to 0xFFF1 raise
   * ACI_GATT_SRV_ATTRIBUTE_MODIFIED_VSEVT_CODE — see STREAM_EventHandler /
   * STREAM_APP_OnCommandWrite. Without it, command writes are silently
   * accepted into the buffer but no event fires. */
  .op_flags = BLE_GATT_SRV_OP_VALUE_VAR_LENGTH_FLAG | BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG,
  .val_len = STREAM_DATA_SIZE,
  .buffer_len = sizeof(stream_data_buffer),
  .buffer_p = stream_data_buffer
};

/* --- 0xFFF2 notify characteristic value buffer --- */
uint8_t stream_notify_buffer[STREAM_NOTIFY_PAYLOAD_SIZE];

static ble_gatt_val_buffer_def_t stream_notify_buffer_def = {
  .op_flags = BLE_GATT_SRV_OP_VALUE_VAR_LENGTH_FLAG,
  .val_len = STREAM_NOTIFY_PAYLOAD_SIZE,
  .buffer_len = sizeof(stream_notify_buffer),
  .buffer_p = stream_notify_buffer
};

/* --- 0xFFF3 notify characteristic value buffer --- */
uint8_t stream_response_buffer[STREAM_RESPONSE_PAYLOAD_SIZE];

static ble_gatt_val_buffer_def_t stream_response_buffer_def = {
  .op_flags = BLE_GATT_SRV_OP_VALUE_VAR_LENGTH_FLAG,
  .val_len = STREAM_RESPONSE_PAYLOAD_SIZE,
  .buffer_len = sizeof(stream_response_buffer),
  .buffer_p = stream_response_buffer
};

/* --- 0xFFF4 notify characteristic value buffer --- */
uint8_t stream_telemetry_buffer[STREAM_TELEMETRY_PAYLOAD_SIZE];

static ble_gatt_val_buffer_def_t stream_telemetry_buffer_def = {
  .op_flags = BLE_GATT_SRV_OP_VALUE_VAR_LENGTH_FLAG,
  .val_len = STREAM_TELEMETRY_PAYLOAD_SIZE,
  .buffer_len = sizeof(stream_telemetry_buffer),
  .buffer_p = stream_telemetry_buffer
};

/* CCCD storage for the notify characteristics.
 * Macro: BLE_GATT_SRV_CCCD_DECLARE(name, num_conn, perm, op_flags)          */
BLE_GATT_SRV_CCCD_DECLARE(stream_notify,
                           CFG_BLE_NUM_RADIO_TASKS,
                           BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                           BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);

BLE_GATT_SRV_CCCD_DECLARE(stream_response,
                           CFG_BLE_NUM_RADIO_TASKS,
                           BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                           BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);

BLE_GATT_SRV_CCCD_DECLARE(stream_telemetry,
                           CFG_BLE_NUM_RADIO_TASKS,
                           BLE_GATT_SRV_CCCD_PERM_DEFAULT,
                           BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);

/* STREAM service characteristics definition */
static const ble_gatt_chr_def_t stream_chars[] = {
    {   /* 0xFFF1: Write-without-response (phone → STM32) */
        .properties = BLE_GATT_SRV_CHAR_PROP_WRITE_NO_RESP,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = 0x10,
        .uuid = BLE_UUID_INIT_16(STREAM_DATA_UUID),
        .val_buffer_p = &stream_data_buffer_def
    },
    {   /* 0xFFF2: Notify (STM32 → phone, StreamDataPacket_t) */
        .properties = BLE_GATT_SRV_CHAR_PROP_NOTIFY,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = 0x10,
        .uuid = BLE_UUID_INIT_16(STREAM_NOTIFY_UUID),
        .val_buffer_p = &stream_notify_buffer_def,
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(stream_notify),
            .descr_count = 1U,
        },
    },
    {   /* 0xFFF3: Notify (STM32 → phone, SET_CHANNELS readback) */
        .properties = BLE_GATT_SRV_CHAR_PROP_NOTIFY,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = 0x10,
        .uuid = BLE_UUID_INIT_16(STREAM_RESPONSE_UUID),
        .val_buffer_p = &stream_response_buffer_def,
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(stream_response),
            .descr_count = 1U,
        },
    },
    {   /* 0xFFF4: Notify (STM32 -> host, ~1 Hz telemetry frame) */
        .properties = BLE_GATT_SRV_CHAR_PROP_NOTIFY,
        .permissions = BLE_GATT_SRV_PERM_NONE,
        .min_key_size = 0x10,
        .uuid = BLE_UUID_INIT_16(STREAM_TELEMETRY_UUID),
        .val_buffer_p = &stream_telemetry_buffer_def,
        .descrs = {
            .descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(stream_telemetry),
            .descr_count = 1U,
        },
    },
};

/* STREAM service definition */
static const ble_gatt_srv_def_t stream_service = {
   .type = BLE_GATT_SRV_PRIMARY_SRV_TYPE,
   .uuid = BLE_UUID_INIT_16(STREAM_UUID),
   .chrs = {
       .chrs_p = (ble_gatt_chr_def_t *)stream_chars,
       .chr_count = 4U,
   },
};

/* Functions Definition ------------------------------------------------------*/

/**
 * @brief  Event handler for STREAM service
 * @param  p_Notification: characteristics notification
 * @retval None
 */
void STREAM_Notification(STREAM_NotificationEvt_t *p_Notification)
{
  /* USER CODE BEGIN STREAM_Notification_1 */

  /* USER CODE END STREAM_Notification_1 */

  switch(p_Notification->EvtOpcode)
  {
    /* USER CODE BEGIN STREAM_Notification_EvtOpcode */

    /* USER CODE END STREAM_Notification_EvtOpcode */

    default:
      /* USER CODE BEGIN STREAM_Notification_default */

      /* USER CODE END STREAM_Notification_default */
      break;
  }

  /* USER CODE BEGIN STREAM_Notification_2 */

  /* USER CODE END STREAM_Notification_2 */

  return;
}

/**
 * @brief  Initialization of STREAM service
 * @param  None
 * @retval None
 */
void STREAM_Init(void)
{
  tBleStatus ret = BLE_STATUS_SUCCESS;

  /* USER CODE BEGIN STREAM_Init */

  /* Register GATT event handler so we receive CCCD writes. */
  BLEEVT_RegisterGattEvtHandler(STREAM_EventHandler);

  /**
   * Register STREAM service (0xFFF0) with four characteristics:
   *   0xFFF1  Write-without-response  → host commands to STM32
   *   0xFFF2  Notify                  → STM32 streams StreamDataPacket_t to host
   *   0xFFF3  Notify                  → command responses
   *   0xFFF4  Notify                  → ~1 Hz telemetry frame
   */
  ret = aci_gatt_srv_add_service((ble_gatt_srv_def_t *) &stream_service);

  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_srv_add_service command: Stream service, result: 0x%02X\n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_srv_add_service command: Stream service\n");

    /* Retrieve handles using the correct WB0x BLE stack API. */
    STREAM_Context.StreamSvcHdle        = aci_gatt_srv_get_service_handle(
                                              (ble_gatt_srv_def_t *)&stream_service);
    STREAM_Context.StreamDataCharHdle   = aci_gatt_srv_get_char_decl_handle(
                                              (ble_gatt_chr_def_t *)&stream_chars[0]);
    STREAM_Context.StreamNotifyCharHdle = aci_gatt_srv_get_char_decl_handle(
                                              (ble_gatt_chr_def_t *)&stream_chars[1]);
    STREAM_Context.StreamResponseCharHdle = aci_gatt_srv_get_char_decl_handle(
                                              (ble_gatt_chr_def_t *)&stream_chars[2]);
    STREAM_Context.StreamTelemetryCharHdle = aci_gatt_srv_get_char_decl_handle(
                                              (ble_gatt_chr_def_t *)&stream_chars[3]);

    APP_DBG_MSG("  Stream handles: Svc=0x%04X  Write=0x%04X  Notify=0x%04X  Response=0x%04X  Telemetry=0x%04X\n",
                STREAM_Context.StreamSvcHdle,
                STREAM_Context.StreamDataCharHdle,
                STREAM_Context.StreamNotifyCharHdle,
                STREAM_Context.StreamResponseCharHdle,
                STREAM_Context.StreamTelemetryCharHdle);
  }

  /* USER CODE END STREAM_Init */

  return;
}

/**
 * @brief  Send streaming data via write-without-response
 * @param  pData: pointer to data to send
 * @param  ConnectionHandle: connection handle
 * @retval Status
 */
tBleStatus STREAM_SendData(STREAM_Data_t *pData, uint16_t ConnectionHandle)
{
  tBleStatus ret = BLE_STATUS_SUCCESS;

  /* USER CODE BEGIN STREAM_SendData_1 */

  /* USER CODE END STREAM_SendData_1 */

  /* 0xFFF1 is a write-without-response characteristic (phone → STM32).
   * The STM32 does not write to it; outbound data goes via STREAM_NotifyData()
   * on 0xFFF2.  This function is retained for API compatibility only.       */
  UNUSED(pData);
  UNUSED(ConnectionHandle);
  ret = BLE_STATUS_SUCCESS;

  /* USER CODE BEGIN STREAM_SendData_2 */

  /* USER CODE END STREAM_SendData_2 */

  return ret;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/**
 * @brief  Send a StreamDataPacket_t via BLE notification on characteristic 0xFFF2.
 * @param  pPacket          Fully-populated packet (header + samples).
 * @param  ConnectionHandle Active connection handle; 0xFFFF means not connected.
 * @retval BLE status code.
 *
 * @note   CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET = 1: the notify call takes the
 *         characteristic *value* handle = declaration handle + 1.
 */
tBleStatus STREAM_NotifyData(StreamDataPacket_t *pPacket, uint16_t ConnectionHandle)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

  if (ConnectionHandle != 0xFFFF && pPacket != NULL)
  {
    ret = aci_gatt_srv_notify(ConnectionHandle,
                              BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                              STREAM_Context.StreamNotifyCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET,
                              GATT_NOTIFICATION,
                              (uint16_t)sizeof(StreamDataPacket_t),
                              (uint8_t *)pPacket);
    /* No per-packet logging here: APP_DBG_MSG → HAL_UART_Transmit at 115200
     * baud blocks ~4 ms per call.  With rapid OK/0x88 alternation that easily
     * exceeds one connection interval (7.5–15 ms), starving the BLE stack.
     * 0x88 (INSUFFICIENT_RESOURCES) is normal flow-control; the caller in
     * stream_app.c handles it.  Log unexpected errors there if needed. */
  }
  return ret;
}

/**
 * @brief  Return the GATT declaration handle of the 0xFFF2 notify characteristic.
 *         CCCD descriptor handle = this value + 2.
 */
uint16_t STREAM_GetNotifyCharHandle(void)
{
  return STREAM_Context.StreamNotifyCharHdle;
}

/* Response type byte — intentionally the same values as the 0xFFF1 command
 * opcodes in stream_app.c (CMD_SET_CHANNELS/CMD_STOP_STREAMING/
 * CMD_START_STREAMING), so a response always echoes which command it's
 * answering. Not #include-shared from stream_app.c (it's a local #define
 * there, not exposed via a header) — kept in sync by convention, both
 * documented in docs/interfaces/channel-selection-control-plane.md section 5.6. */
#define STREAM_RESPONSE_TYPE_SET_CHANNELS    0x01U
#define STREAM_RESPONSE_TYPE_STOP_STREAMING  0x02U
#define STREAM_RESPONSE_TYPE_START_STREAMING 0x03U
/* 0x04/0x05 — register console, docs/interfaces/fpga-diagnostic-access.md §2 */

/**
 * @brief  Notify the SET_CHANNELS readback result on 0xFFF3. Low-rate,
 *         user-triggered — unlike STREAM_NotifyData() this can afford to log
 *         on failure (e.g. peer hasn't enabled the 0xFFF3 CCCD yet).
 */
tBleStatus STREAM_NotifyChannelsReadback(uint8_t ch_a_friendly, uint8_t ch_b_friendly,
                                          uint16_t ConnectionHandle)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

  if (ConnectionHandle != 0xFFFF)
  {
    uint8_t payload[3] = { STREAM_RESPONSE_TYPE_SET_CHANNELS, ch_a_friendly, ch_b_friendly };
    ret = aci_gatt_srv_notify(ConnectionHandle,
                              BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                              STREAM_Context.StreamResponseCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET,
                              GATT_NOTIFICATION,
                              (uint16_t)sizeof(payload),
                              payload);
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("STREAM: 0xFFF3 readback notify failed 0x%02X (CCCD not enabled?)\n", ret);
    }
  }
  return ret;
}

/**
 * @brief  Notify STOP_STREAMING/START_STREAMING completion on 0xFFF3 — see
 *         docs/interfaces/channel-selection-control-plane.md section 5.6.
 *         Lets the pc-app wait for actual MCU confirmation instead of a
 *         fixed timer before proceeding to the next step of an Apply
 *         sequence.
 * @param  cmd              STREAM_RESPONSE_TYPE_STOP_STREAMING or
 *                           STREAM_RESPONSE_TYPE_START_STREAMING.
 * @param  success          1 if the stream_enable write was confirmed via
 *                           readback, 0 otherwise.
 */
tBleStatus STREAM_NotifyStreamingAck(uint8_t cmd, uint8_t success, uint16_t ConnectionHandle)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

  if (ConnectionHandle != 0xFFFF)
  {
    uint8_t payload[2] = { cmd, success };
    ret = aci_gatt_srv_notify(ConnectionHandle,
                              BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                              STREAM_Context.StreamResponseCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET,
                              GATT_NOTIFICATION,
                              (uint16_t)sizeof(payload),
                              payload);
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("STREAM: 0xFFF3 streaming-ack notify failed 0x%02X (cmd=0x%02X)\n", ret, cmd);
    }
  }
  return ret;
}

/**
 * @brief  Notify a register-console result on 0xFFF3 — see
 *         docs/interfaces/fpga-diagnostic-access.md section 2.
 *         `value` is always what was READ BACK from the FPGA regbank, for both
 *         REG_WRITE16 and REG_READ16: an ack must mean "the regbank holds
 *         this", never "the write call returned". Same principle as
 *         FPGA_SPI_SetStreamEnable()'s existing write-then-read.
 *         `addr` is echoed so the pc-app can match a response to its request
 *         without tracking an outstanding-command slot.
 * @param  type             STREAM_RESPONSE_TYPE_REG_WRITE16 or
 *                           STREAM_RESPONSE_TYPE_REG_READ16.
 */
tBleStatus STREAM_NotifyRegAccess(uint8_t type, uint8_t addr, uint16_t value,
                                  uint16_t ConnectionHandle)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

  if (ConnectionHandle != 0xFFFF)
  {
    uint8_t payload[4] = { type, addr,
                           (uint8_t)(value & 0xFFU),
                           (uint8_t)(value >> 8) };
    ret = aci_gatt_srv_notify(ConnectionHandle,
                              BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                              STREAM_Context.StreamResponseCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET,
                              GATT_NOTIFICATION,
                              (uint16_t)sizeof(payload),
                              payload);
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("STREAM: 0xFFF3 reg-access notify failed 0x%02X (type=0x%02X addr=%u)\n",
                  ret, type, addr);
    }
  }
  return ret;
}

/* ── 0xFFF4 telemetry ─────────────────────────────────────────────────────
 * docs/interfaces/stream-packet-format.md section 6.2 is the single source
 * of truth for this layout; the offsets below are its byte table transcribed.
 * Written byte-wise on purpose: the uint32 fields land at offsets 2, 6, 18,
 * 22 and 26, none 4-byte aligned, and an unaligned uint32 store HardFaults on
 * the M0+. A packed struct would also work but hides the wire format behind a
 * compiler attribute, where this makes it readable next to the spec. */

static uint8_t *TelemetryPutU16(uint8_t *p, uint16_t v)
{
  *p++ = (uint8_t)(v & 0xFFU);
  *p++ = (uint8_t)((v >> 8) & 0xFFU);
  return p;
}

static uint8_t *TelemetryPutU32(uint8_t *p, uint32_t v)
{
  *p++ = (uint8_t)(v & 0xFFU);
  *p++ = (uint8_t)((v >> 8) & 0xFFU);
  *p++ = (uint8_t)((v >> 16) & 0xFFU);
  *p++ = (uint8_t)((v >> 24) & 0xFFU);
  return p;
}

/**
 * @brief  Notify one telemetry frame on 0xFFF4 — see
 *         docs/interfaces/stream-packet-format.md section 6.
 */
tBleStatus STREAM_NotifyTelemetry(const StreamTelemetry_t *pTelemetry,
                                  uint16_t ConnectionHandle)
{
  tBleStatus ret = BLE_STATUS_FAILED;

  if (ConnectionHandle != 0xFFFF && pTelemetry != NULL)
  {
    uint8_t  frame[STREAM_TELEMETRY_PAYLOAD_SIZE];
    uint8_t *p = frame;

    *p++ = (uint8_t)STREAM_TELEMETRY_VERSION;             /* off  0 */
    *p++ = pTelemetry->flags;                             /* off  1 */
    p = TelemetryPutU32(p, pTelemetry->anchor_sample_index);     /* off  2 */
    p = TelemetryPutU32(p, pTelemetry->anchor_timestamp_s);      /* off  6 */
    p = TelemetryPutU16(p, pTelemetry->anchor_timestamp_sub_s);  /* off 10 */
    p = TelemetryPutU32(p, pTelemetry->fifo0_overflow_samples);  /* off 12 */
    p = TelemetryPutU16(p, pTelemetry->fifo0_high_water);        /* off 16 */
    p = TelemetryPutU32(p, pTelemetry->ring_truncated_samples);  /* off 18 */
    p = TelemetryPutU32(p, pTelemetry->flow_off_count);          /* off 22 */
    p = TelemetryPutU32(p, pTelemetry->stall_time_ms_total);     /* off 26 */

    /* Compile-time-checkable restatement of section 6.2's "30 bytes". */
    if ((uint32_t)(p - frame) != (uint32_t)STREAM_TELEMETRY_PAYLOAD_SIZE)
    {
      APP_DBG_MSG("STREAM: telemetry serialiser wrote %lu bytes, expected %u\n",
                  (unsigned long)(p - frame), (unsigned)STREAM_TELEMETRY_PAYLOAD_SIZE);
      return BLE_STATUS_FAILED;
    }

    ret = aci_gatt_srv_notify(ConnectionHandle,
                              BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
                              STREAM_Context.StreamTelemetryCharHdle
                                + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET,
                              GATT_NOTIFICATION,
                              (uint16_t)STREAM_TELEMETRY_PAYLOAD_SIZE,
                              frame);
  }

  return ret;
}

/**
 * @brief  GATT event handler registered with BLEEVT_RegisterGattEvtHandler().
 *
 *  Intercepts ACI_GATT_SRV_ATTRIBUTE_MODIFIED_VSEVT_CODE to detect when Android
 *  writes the CCCD on characteristic 0xFFF2 (enabling or disabling notifications).
 *  Returns BLEEVT_Ack when it consumes the event, BLEEVT_NoAck otherwise.
 */
static BLEEVT_EvtAckStatus_t STREAM_EventHandler(aci_blecore_event *p_evt)
{
  BLEEVT_EvtAckStatus_t ack = BLEEVT_NoAck;

  if (p_evt->ecode == ACI_GATT_SRV_ATTRIBUTE_MODIFIED_VSEVT_CODE)
  {
    aci_gatt_srv_attribute_modified_event_rp0 *p_attr =
        (aci_gatt_srv_attribute_modified_event_rp0 *)p_evt->data;

    /* CCCD handle = declaration handle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET (2) */
    if (p_attr->Attr_Handle ==
        (STREAM_Context.StreamNotifyCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET) &&
        p_attr->Attr_Data_Length >= 2U)
    {
      uint16_t cccd_val = (uint16_t)p_attr->Attr_Data[0] |
                          ((uint16_t)p_attr->Attr_Data[1] << 8U);
      STREAM_APP_OnCCCDWrite(p_attr->Connection_Handle, cccd_val);
      ack = BLEEVT_Ack;
    }
    /* 0xFFF1 command write (write-without-response value attribute). */
    else if (p_attr->Attr_Handle ==
        (STREAM_Context.StreamDataCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
    {
      STREAM_APP_OnCommandWrite(p_attr->Connection_Handle,
                                 p_attr->Attr_Data,
                                 (uint8_t)p_attr->Attr_Data_Length);
      ack = BLEEVT_Ack;
    }
  }

  return ack;
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/
