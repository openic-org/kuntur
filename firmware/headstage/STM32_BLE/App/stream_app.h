/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stream_app.h
  * @author  Manuel's Team
  * @brief   Header for stream_app.c - Streaming application layer
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
#ifndef STREAM_APP_H
#define STREAM_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  STREAM_CONN_HANDLE_EVT,
  STREAM_DISCON_HANDLE_EVT,

  /* USER CODE BEGIN Service_OpcodeNotificationEvt_t */

  /* USER CODE END Service_OpcodeNotificationEvt_t */

  STREAM_LAST_EVT,
} STREAM_APP_OpcodeNotificationEvt_t;

typedef struct
{
  STREAM_APP_OpcodeNotificationEvt_t     EvtOpcode;
  uint16_t                               ConnectionHandle;

  /* USER CODE BEGIN STREAM_APP_ConnHandleNotEvt_t */

  /* USER CODE END STREAM_APP_ConnHandleNotEvt_t */
} STREAM_APP_ConnHandleNotEvt_t;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* ── Stream mode ──────────────────────────────────────────────────────────────
 *   STREAM_MODE_WB09KE_HF: NUCLEO-WB09KE USB bridge → PC.
 *                           Tight BLE_STACK_Tick() burst loop; VTIMER 2 ms
 *                           safety fallback.  Target: 30 000 SPS per channel.
 *                           Device name: "Kuntur-Headstage".
 */
#define STREAM_MODE_WB09KE_HF      1
#define STREAM_ACTIVE_MODE         STREAM_MODE_WB09KE_HF

#define STREAM_BUFFER_SIZE           (STREAM_MAX_PAYLOAD_SIZE)  /* Legacy tx buffer size */

/* CFG_TASK_STREAM_SEND_ID is defined in app_conf.h (CFG_Task_Id_t enum). */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ------------------------------------------------------- */
void STREAM_APP_Init(void);
void STREAM_APP_EvtRx(STREAM_APP_ConnHandleNotEvt_t *p_Notification);

/* USER CODE BEGIN EF */

/**
 * @brief  Called by app_ble.c when the central writes the CCCD on 0xFFF2.
 *         Gates BLE notifications: seq_num is reset to 0 on first enable so
 *         Android always sees a clean sequence start at connection time.
 * @param  conn_handle  BLE connection handle.
 * @param  cccd_value   Raw CCCD value (0x0001 = notifications enabled, 0x0000 = disabled).
 */
void STREAM_APP_OnCCCDWrite(uint16_t conn_handle, uint16_t cccd_value);

/**
 * @brief  Called by app_ble.c from ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE.
 *         Clears the TX flow-off flag and restarts the streaming pipeline.
 */
void STREAM_APP_ResumeSending(void);

/**
 * @brief  Called when the central writes the 0xFFF1 command characteristic.
 *         See docs/interfaces/channel-selection-control-plane.md section 2.
 * @param  conn_handle  BLE connection handle.
 * @param  p_data       Raw write payload (cmd byte + command-specific bytes).
 * @param  length       Payload length in bytes.
 */
void STREAM_APP_OnCommandWrite(uint16_t conn_handle, const uint8_t *p_data, uint8_t length);

/**
 * @brief  Returns the cumulative number of GATT notifications sent since init.
 *         Used by cpu_bench.c to compute per-second throughput.
 */
uint32_t STREAM_APP_GetNotifyTotal(void);

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*STREAM_APP_H */
