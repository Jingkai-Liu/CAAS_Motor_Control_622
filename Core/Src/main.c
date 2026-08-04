/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "fatfs.h"
#include "sdio.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>            // 如果需要简单 printf 调试
#include <stdarg.h>           // 同上
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t node_id;
  uint8_t online;
  uint8_t enable;
  uint8_t drive_mode;
  uint8_t ebrake_released;
  uint8_t system_state;
  int16_t torque_dNm;
  int16_t speed_rpm;
  int16_t mech_rpm;
  int16_t motor_temp_c;
  int16_t controller_temp_c;
  uint8_t error_code;
  uint16_t udc_dV;
  int16_t idc_dA;
  uint16_t iphase_dA;
  uint16_t limit_flags;
  uint32_t last_status1_tick;
  uint32_t last_status2_tick;
} AppMotorStatus_t;

typedef struct
{
  uint8_t node_id;
  int8_t direction;       /* 1 = forward, -1 = reverse. Change here for field wiring. */
} AppMotorConfig_t;

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
} AppGpioPin_t;

extern DMA_HandleTypeDef hdma_usart1_tx;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_MOTOR_NODE_ID_1          1U
#define APP_MOTOR_NODE_ID_2          2U
#define APP_MOTOR_MAX_RPM            3000U
#define APP_SPEED_OFFSET_RPM         30000L
#define APP_TORQUE_OFFSET_RAW        10000U
#define APP_CAN_CMD_BASE_ID          0x200U
#define APP_CAN_STATUS1_BASE_ID      0x180U
#define APP_CAN_STATUS2_BASE_ID      0x280U
#define APP_CAN_CMD_PERIOD_MS        20U
#define APP_UART_PERIOD_MS           100U
#define APP_MOTOR_ONLINE_TIMEOUT_MS  500U
#define APP_CAN_TX_RETRY_COUNT       2U
#define APP_ADC_SAMPLE_COUNT         8U

#define APP_ANO_ADC_ID               0xF1U  /* ADC raw + speed percent + stop */
#define APP_ANO_TARGET_SPEED_ID      0xF2U  /* Desired speed (target RPM) */
#define APP_ANO_ACTUAL_SPEED_ID      0xF3U  /* Actual mechanical RPM */
#define APP_ANO_TORQUE_ID            0xF4U  /* Actual torque */
#define APP_ANO_MOTOR_TEMP_ID        0xF5U  /* Motor temperature */
#define APP_ANO_CONTROLLER_TEMP_ID   0xF6U  /* Controller temperature */
#define APP_ANO_VOLTAGE_CURRENT_ID   0xF7U  /* Bus voltage & current */
#define APP_MOTOR_DUAL_NODE1_MODE    0U     /* Set to 1 when both motors share node ID 1 */

#define ANO_HEAD      0xAB

#define ANO_SRC_ADDR  0x01      // STM32
#define ANO_DST_ADDR  0xFF      // PC上位机

/* 新增：刹车输入引脚定义 */
#define APP_BRAKE_GPIO_PORT      GPIOC
#define APP_BRAKE_GPIO_PIN       GPIO_PIN_1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile AppMotorStatus_t app_motors[2];
static const AppMotorConfig_t app_motor_configs[2] =
{
  {APP_MOTOR_NODE_ID_1, 1},
  {APP_MOTOR_NODE_ID_2, 1}
};
static uint8_t app_speed_percent = 0U;
static uint8_t app_stop_active = 1U;
static int16_t app_target_rpm[2] = {0};
static volatile uint32_t app_can_tx_error_count = 0U;
static volatile uint32_t app_adc_error_count = 0U;
static volatile uint32_t app_uart_error_count = 0U;
static uint16_t app_adc_raw = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void App_Init(void);
static void App_Task(void);
static uint16_t App_ReadAdcRaw(void);
static uint8_t App_ReadSpeedPercent(void);
static uint8_t App_ReadStopSwitch(void);
static void App_SendMotorCommands(uint8_t speed_percent, uint8_t stop_active);
static HAL_StatusTypeDef App_SendMotorCommand(uint8_t node_id, int16_t speed_rpm, uint8_t enable);
static void App_ProcessCanFrame(const CAN_RxHeaderTypeDef *header, const uint8_t data[8]);
static volatile AppMotorStatus_t *App_FindMotorByNode(uint8_t node_id);
static int8_t App_GetMotorDirection(uint8_t node_id);
static void App_CopyMotorSnapshot(AppMotorStatus_t snapshot[2], int16_t target_rpm[2]);
static void App_UpdateMotorOnlineFlags(uint32_t now);
static uint16_t App_GetLe16(const uint8_t *data);
static void App_UartReport(void);
static void App_AnoSendVoltageCurrent(const AppMotorStatus_t motors[2], uint32_t now);
static void App_AnoSendAdcReading(void);
static void App_AnoSendTargetSpeed(const AppMotorStatus_t motors[2], const int16_t target_rpm[2]);
static void App_AnoSendActualSpeed(const AppMotorStatus_t motors[2]);
static void App_AnoSendTorque(const AppMotorStatus_t motors[2]);
static void App_AnoSendMotorTemp(const AppMotorStatus_t motors[2]);
static void App_AnoSendControllerTemp(const AppMotorStatus_t motors[2]);
static void App_AnoAppendS16(uint8_t *payload, uint16_t *index, int16_t value);
static void App_AnoAppendU32(uint8_t *payload, uint16_t *index, uint32_t value);
static void App_AnoCalcCheck(
    uint8_t *buf,
    uint16_t len,
    uint8_t *sumcheck,
    uint8_t *addcheck)
{
  uint16_t i;
  uint8_t sc = 0U;
  uint8_t ac = 0U;

  for (i = 0U; i < len; i++)
  {
    sc += buf[i];
    ac += sc;
  }
  *sumcheck = sc;
  *addcheck = ac;
}

static void App_AnoSendFrame(
    uint8_t id,
    const uint8_t *payload,
    uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void App_Init(void)
{
  CAN_FilterTypeDef filter = {0};

  app_motors[0].node_id = APP_MOTOR_NODE_ID_1;
#if APP_MOTOR_DUAL_NODE1_MODE
  app_motors[1].node_id = APP_MOTOR_NODE_ID_1;
#else
  app_motors[1].node_id = APP_MOTOR_NODE_ID_2;
#endif

  HAL_ADCEx_Calibration_Start(&hadc1);

  /* CAN accepts all frames; application parsing below filters motor status IDs. */
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

}

static void App_Task(void)
{
  static uint32_t last_cmd_tick = 0U;
  static uint32_t last_uart_tick = 0U;
  uint32_t now = HAL_GetTick();

  if ((now - last_cmd_tick) >= APP_CAN_CMD_PERIOD_MS)
  {
    last_cmd_tick = now;
    app_speed_percent = App_ReadSpeedPercent();
    app_stop_active = App_ReadStopSwitch();
    App_SendMotorCommands(app_speed_percent, app_stop_active);
  }

  if ((now - last_uart_tick) >= APP_UART_PERIOD_MS)
  {
    last_uart_tick = now;
    App_UartReport();
  }
}

static uint16_t App_ReadAdcRaw(void)
{
  uint32_t sum = 0U;
  uint8_t sample_count = 0U;
  uint8_t i;

  for (i = 0U; i < APP_ADC_SAMPLE_COUNT; i++)
  {
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
      app_adc_error_count++;
      HAL_ADC_Stop(&hadc1);
      continue;
    }
    if (HAL_ADC_PollForConversion(&hadc1, 2U) == HAL_OK)
    {
      sum += HAL_ADC_GetValue(&hadc1);
      sample_count++;
    }
    else
    {
      app_adc_error_count++;
    }
    HAL_ADC_Stop(&hadc1);
  }

  if (sample_count == 0U)
  {
    return 0U;
  }

  return (uint16_t)(sum / sample_count);
}

static uint8_t App_ReadSpeedPercent(void)
{
  uint16_t adc_raw = App_ReadAdcRaw();
  app_adc_raw = adc_raw;
  uint32_t percent = ((uint32_t)adc_raw * 100U + 2047U) / 4095U;

  if (percent > 100U)
  {
    percent = 100U;
  }
  return (uint8_t)percent;
}

static uint8_t App_ReadStopSwitch(void)
{
    /* PC1 低电平 = 刹车（停止），高电平 = 运行 */
    if (HAL_GPIO_ReadPin(APP_BRAKE_GPIO_PORT, APP_BRAKE_GPIO_PIN) == GPIO_PIN_RESET)
    {
        return 1U;   // 停止
    }
    else
    {
        return 0U;   // 允许运行
    }
}

static void App_SendMotorCommands(uint8_t speed_percent, uint8_t stop_active)
{
    uint32_t rpm = ((uint32_t)speed_percent * APP_MOTOR_MAX_RPM + 50U) / 100U;
    uint8_t enable = (stop_active == 0U) ? 1U : 0U;
    uint8_t i;

#if APP_MOTOR_DUAL_NODE1_MODE
    const uint8_t motor_count = 1U;
#else
    const uint8_t motor_count = (uint8_t)(sizeof(app_motor_configs) / sizeof(app_motor_configs[0]));
#endif

    for (i = 0U; i < motor_count; i++)
    {
        uint8_t retry;
        uint8_t node_id = app_motor_configs[i].node_id;
        int16_t speed_rpm = enable ? (int16_t)((int32_t)rpm * App_GetMotorDirection(node_id)) : 0;
        HAL_StatusTypeDef status = HAL_ERROR;

        app_target_rpm[i] = speed_rpm;
        for (retry = 0U; retry < APP_CAN_TX_RETRY_COUNT; retry++)
        {
            status = App_SendMotorCommand(node_id, speed_rpm, enable);
            if (status == HAL_OK)
            {
                break;
            }
        }
        if (status != HAL_OK)
        {
            app_can_tx_error_count++;
            if (app_can_tx_error_count > 50U)
            {
                HAL_CAN_Stop(&hcan);
                HAL_CAN_Start(&hcan);
                HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
                app_can_tx_error_count = 0U;
            }
        }
    }
}

static HAL_StatusTypeDef App_SendMotorCommand(uint8_t node_id, int16_t speed_rpm, uint8_t enable)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint8_t tx_data[8] = {0};
  uint32_t tx_mailbox;
  uint16_t speed_raw;

  if (speed_rpm > (int16_t)APP_MOTOR_MAX_RPM)
  {
    speed_rpm = (int16_t)APP_MOTOR_MAX_RPM;
  }
  if (speed_rpm < -(int16_t)APP_MOTOR_MAX_RPM)
  {
    speed_rpm = -(int16_t)APP_MOTOR_MAX_RPM;
  }

  speed_raw = (uint16_t)((int32_t)speed_rpm + APP_SPEED_OFFSET_RPM);

  tx_header.StdId = APP_CAN_CMD_BASE_ID + node_id;
  tx_header.ExtId = 0U;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8U;
  tx_header.TransmitGlobalTime = DISABLE;

  /* Bit0 enable, Bit1..2 = 1 speed mode, Bit3 EBrakeCmd = 0 (MCU控制，即刹车就绪) */
  tx_data[0] = enable ? 0x0BU : 0x00U;
  tx_data[1] = (uint8_t)(APP_TORQUE_OFFSET_RAW & 0xFFU);
  tx_data[2] = (uint8_t)(APP_TORQUE_OFFSET_RAW >> 8);
  tx_data[3] = (uint8_t)(speed_raw & 0xFFU);
  tx_data[4] = (uint8_t)(speed_raw >> 8);

  return HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox);
}

static void App_ProcessCanFrame(const CAN_RxHeaderTypeDef *header, const uint8_t data[8])
{
  uint8_t node_id;
  volatile AppMotorStatus_t *motor;
  uint16_t raw;

  if ((header->IDE != CAN_ID_STD) || (header->DLC < 8U))
  {
    return;
  }

  if ((header->StdId >= (APP_CAN_STATUS1_BASE_ID + APP_MOTOR_NODE_ID_1)) &&
      (header->StdId <= (APP_CAN_STATUS1_BASE_ID + APP_MOTOR_NODE_ID_2 + 16U)))
  {
    node_id = (uint8_t)(header->StdId - APP_CAN_STATUS1_BASE_ID);
    motor = App_FindMotorByNode(node_id);
    if (motor != NULL)
    {
      motor->online = 1U;
      motor->enable = data[0] & 0x01U;
      motor->drive_mode = (data[0] >> 1) & 0x01U;
      motor->ebrake_released = (data[0] >> 2) & 0x01U;
      motor->system_state = (data[0] >> 3) & 0x03U;
      motor->torque_dNm = (int16_t)(App_GetLe16(&data[1]) - APP_TORQUE_OFFSET_RAW);
      motor->speed_rpm = (int16_t)((int32_t)App_GetLe16(&data[3]) - APP_SPEED_OFFSET_RPM);
      motor->mech_rpm = motor->speed_rpm / 10;
      motor->motor_temp_c = (int16_t)data[5] - 40;
      motor->controller_temp_c = (int16_t)data[6] - 40;
      motor->error_code = data[7];
      motor->last_status1_tick = HAL_GetTick();
      if (node_id == APP_MOTOR_NODE_ID_1)
      {
      }
    }
  }
  else if ((header->StdId >= (APP_CAN_STATUS2_BASE_ID + APP_MOTOR_NODE_ID_1)) &&
           (header->StdId <= (APP_CAN_STATUS2_BASE_ID + APP_MOTOR_NODE_ID_2 + 16U)))
  {
    node_id = (uint8_t)(header->StdId - APP_CAN_STATUS2_BASE_ID);
    motor = App_FindMotorByNode(node_id);
    if (motor != NULL)
    {
      raw = App_GetLe16(&data[2]);
      motor->online = 1U;
      motor->udc_dV = App_GetLe16(&data[0]);
      motor->idc_dA = (int16_t)raw - 10000;
      motor->iphase_dA = App_GetLe16(&data[4]);
      motor->limit_flags = App_GetLe16(&data[6]);
      motor->last_status2_tick = HAL_GetTick();
    }
  }
}

static volatile AppMotorStatus_t *App_FindMotorByNode(uint8_t node_id)
{
  uint8_t i;

  for (i = 0U; i < (uint8_t)(sizeof(app_motors) / sizeof(app_motors[0])); i++)
  {
    if (app_motors[i].node_id == node_id)
    {
      return &app_motors[i];
    }
  }
  return NULL;
}

static int8_t App_GetMotorDirection(uint8_t node_id)
{
  uint8_t i;

  for (i = 0U; i < (uint8_t)(sizeof(app_motor_configs) / sizeof(app_motor_configs[0])); i++)
  {
    if (app_motor_configs[i].node_id == node_id)
    {
      return (app_motor_configs[i].direction < 0) ? -1 : 1;
    }
  }
  return 1;
}

static void App_CopyMotorSnapshot(AppMotorStatus_t snapshot[2], int16_t target_rpm[2])
{
  uint8_t i;

  __disable_irq();
  for (i = 0U; i < (uint8_t)(sizeof(app_motors) / sizeof(app_motors[0])); i++)
  {
    snapshot[i] = app_motors[i];
    target_rpm[i] = app_target_rpm[i];
  }
  __enable_irq();
}

static void App_UpdateMotorOnlineFlags(uint32_t now)
{
  uint8_t i;

  __disable_irq();
  for (i = 0U; i < (uint8_t)(sizeof(app_motors) / sizeof(app_motors[0])); i++)
  {
    if ((now - app_motors[i].last_status1_tick) >= APP_MOTOR_ONLINE_TIMEOUT_MS)
    {
      app_motors[i].online = 0U;
    }
  }
  __enable_irq();
}

static uint16_t App_GetLe16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void App_UartReport(void)
{
  AppMotorStatus_t motors[2];
  int16_t target_rpm[2];
  uint32_t now = HAL_GetTick();

  App_UpdateMotorOnlineFlags(now);
  App_CopyMotorSnapshot(motors, target_rpm);

#if APP_MOTOR_DUAL_NODE1_MODE
  motors[1] = motors[0];
  target_rpm[1] = target_rpm[0];
#endif

  App_AnoSendVoltageCurrent(motors, now);
  App_AnoSendAdcReading();
  App_AnoSendTargetSpeed(motors, target_rpm);
  App_AnoSendActualSpeed(motors);
  App_AnoSendTorque(motors);
  App_AnoSendMotorTemp(motors);
  App_AnoSendControllerTemp(motors);
}

static void App_AnoSendVoltageCurrent(const AppMotorStatus_t motors[2], uint32_t now)
{
  uint8_t payload[32];
  uint16_t index = 0U;
  uint8_t i;
#if APP_MOTOR_DUAL_NODE1_MODE
  const uint8_t motor_count = 1U;
#else
  const uint8_t motor_count = (uint8_t)(sizeof(app_motor_configs) / sizeof(app_motor_configs[0]));
#endif

  App_AnoAppendU32(payload, &index, now);
  for (i = 0U; i < motor_count; i++)
  {
    payload[index++] = motors[i].node_id;
    App_AnoAppendS16(payload, &index, (int16_t)motors[i].udc_dV);
    App_AnoAppendS16(payload, &index, motors[i].idc_dA);
    App_AnoAppendS16(payload, &index, (int16_t)motors[i].iphase_dA);
  }
  App_AnoSendFrame(APP_ANO_VOLTAGE_CURRENT_ID, payload, index);
}

static void App_AnoSendAdcReading(void)
{
  uint8_t payload[10];
  uint16_t index = 0U;
  uint32_t now = HAL_GetTick();

  App_AnoAppendU32(payload, &index, now);
  App_AnoAppendS16(payload, &index, (int16_t)app_adc_raw);
  payload[index++] = app_speed_percent;
  payload[index++] = app_stop_active;
  App_AnoSendFrame(APP_ANO_ADC_ID, payload, index);
}

static void App_AnoSendTargetSpeed(const AppMotorStatus_t motors[2], const int16_t target_rpm[2])
{
  uint8_t payload[16];
  uint16_t index = 0U;
  uint8_t i;
#if APP_MOTOR_DUAL_NODE1_MODE
  const uint8_t motor_count = 1U;
#else
  const uint8_t motor_count = (uint8_t)(sizeof(app_motor_configs) / sizeof(app_motor_configs[0]));
#endif

  for (i = 0U; i < motor_count; i++)
  {
    payload[index++] = motors[i].node_id;
    App_AnoAppendS16(payload, &index, target_rpm[i]);
  }
  App_AnoSendFrame(APP_ANO_TARGET_SPEED_ID, payload, index);
}

static void App_AnoSendActualSpeed(const AppMotorStatus_t motors[2])
{
  uint8_t payload[16];
  uint16_t index = 0U;
  uint8_t i;
#if APP_MOTOR_DUAL_NODE1_MODE
  const uint8_t motor_count = 1U;
#else
  const uint8_t motor_count = (uint8_t)(sizeof(app_motor_configs) / sizeof(app_motor_configs[0]));
#endif

  for (i = 0U; i < motor_count; i++)
  {
    payload[index++] = motors[i].node_id;
    App_AnoAppendS16(payload, &index, motors[i].mech_rpm);
  }
  App_AnoSendFrame(APP_ANO_ACTUAL_SPEED_ID, payload, index);
}

static void App_AnoSendTorque(const AppMotorStatus_t motors[2])
{
  uint8_t payload[16];
  uint16_t index = 0U;
  uint8_t i;
#if APP_MOTOR_DUAL_NODE1_MODE
  const uint8_t motor_count = 1U;
#else
  const uint8_t motor_count = (uint8_t)(sizeof(app_motor_configs) / sizeof(app_motor_configs[0]));
#endif

  for (i = 0U; i < motor_count; i++)
  {
    payload[index++] = motors[i].node_id;
    App_AnoAppendS16(payload, &index, motors[i].torque_dNm);
  }
  App_AnoSendFrame(APP_ANO_TORQUE_ID, payload, index);
}

static void App_AnoSendMotorTemp(const AppMotorStatus_t motors[2])
{
  uint8_t payload[16];
  uint16_t index = 0U;
  uint8_t i;
#if APP_MOTOR_DUAL_NODE1_MODE
  const uint8_t motor_count = 1U;
#else
  const uint8_t motor_count = (uint8_t)(sizeof(app_motor_configs) / sizeof(app_motor_configs[0]));
#endif

  for (i = 0U; i < motor_count; i++)
  {
    payload[index++] = motors[i].node_id;
    App_AnoAppendS16(payload, &index, motors[i].motor_temp_c);
  }
  App_AnoSendFrame(APP_ANO_MOTOR_TEMP_ID, payload, index);
}

static void App_AnoSendControllerTemp(const AppMotorStatus_t motors[2])
{
  uint8_t payload[16];
  uint16_t index = 0U;
  uint8_t i;
#if APP_MOTOR_DUAL_NODE1_MODE
  const uint8_t motor_count = 1U;
#else
  const uint8_t motor_count = (uint8_t)(sizeof(app_motor_configs) / sizeof(app_motor_configs[0]));
#endif

  for (i = 0U; i < motor_count; i++)
  {
    payload[index++] = motors[i].node_id;
    App_AnoAppendS16(payload, &index, motors[i].controller_temp_c);
  }
  App_AnoSendFrame(APP_ANO_CONTROLLER_TEMP_ID, payload, index);
}

static void App_AnoAppendS16(uint8_t *payload, uint16_t *index, int16_t value)
{
  payload[(*index)++] = (uint8_t)((uint16_t)value & 0xFFU);
  payload[(*index)++] = (uint8_t)(((uint16_t)value >> 8) & 0xFFU);
}

static void App_AnoAppendU32(uint8_t *payload, uint16_t *index, uint32_t value)
{
  payload[(*index)++] = (uint8_t)(value & 0xFFU);
  payload[(*index)++] = (uint8_t)((value >> 8) & 0xFFU);
  payload[(*index)++] = (uint8_t)((value >> 16) & 0xFFU);
  payload[(*index)++] = (uint8_t)((value >> 24) & 0xFFU);
}

static void App_AnoSendFrame(
    uint8_t id,
    const uint8_t *payload,
    uint16_t len)
{
    uint8_t frame[128];

    uint16_t index = 0;
    uint16_t i;

    uint8_t sc;
    uint8_t ac;

    if(len > 100)
        return;

    /*-----------------------
      Frame Header
    -----------------------*/

    frame[index++] = ANO_HEAD;

    frame[index++] = ANO_SRC_ADDR;

    frame[index++] = ANO_DST_ADDR;

    frame[index++] = id;

    frame[index++] = (uint8_t)(len & 0xFF);
    frame[index++] = (uint8_t)(len >> 8);

    /*-----------------------
      Payload
    -----------------------*/

    for(i=0;i<len;i++)
    {
        frame[index++] = payload[i];
    }

    /*-----------------------
      Check
    -----------------------*/

    App_AnoCalcCheck(
        frame,
        index,
        &sc,
        &ac);

    frame[index++] = sc;
    frame[index++] = ac;

    /*-----------------------
      Wait DMA idle
    -----------------------*/
if (huart1.gState != HAL_UART_STATE_READY)
    {
        huart1.gState = HAL_UART_STATE_READY;
    }
    HAL_UART_Transmit(&huart1, frame, index, 100);


}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  App_Init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_Task();
  }
//while(1)
//{
//    uint8_t test[] = "HELLO\r\n";

//    HAL_UART_Transmit(
//        &huart1,
//        test,
//        sizeof(test)-1,
//        100);

//    for(volatile uint32_t i=0;i<500000;i++);
//}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_arg)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  if (hcan_arg->Instance != CAN1)
  {
    return;
  }

  while (HAL_CAN_GetRxFifoFillLevel(hcan_arg, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(hcan_arg, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
      App_ProcessCanFrame(&rx_header, rx_data);
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
