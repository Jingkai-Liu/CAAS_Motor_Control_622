#include "sd_logger.h"
#include "fatfs.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

#define SD_LOGGER_BATCH_BYTES       1024U
#define SD_LOGGER_WRITE_THRESHOLD    512U
#define SD_LOGGER_RECORDS_PER_WRITE   10U /* Worst-case CSV size exceeds one 512-byte sector. */
#define SD_LOGGER_WRITE_PERIOD_MS   1000U
#define SD_LOGGER_RETRY_PERIOD_MS   5000U
#define SD_LOGGER_FILE_NUMBER_MAX    999U

static SdLoggerRecord_t sd_logger_ring[SD_LOGGER_RING_SIZE];
static volatile uint16_t sd_logger_head;
static volatile uint16_t sd_logger_tail;
static volatile uint32_t sd_logger_dropped;
static FIL sd_logger_file;
static char sd_logger_path[20];
static char sd_logger_batch[SD_LOGGER_BATCH_BYTES];
static UINT sd_logger_batch_length;
static uint8_t sd_logger_batch_pending;
static uint8_t sd_logger_file_open;
static uint32_t sd_logger_last_write_tick;
static uint32_t sd_logger_last_retry_tick;
static SdLoggerStatus_t sd_logger_status = SD_LOGGER_NOT_READY;
static FRESULT sd_logger_last_error = FR_NOT_READY;

static uint16_t SdLogger_RingCountUnsafe(void);
static uint8_t SdLogger_Peek(SdLoggerRecord_t *record);
static void SdLogger_DiscardPeeked(void);
static void SdLogger_MakeFilePath(uint16_t number);
static FRESULT SdLogger_OpenNewFile(void);
static FRESULT SdLogger_CreateBatch(void);
static FRESULT SdLogger_WritePendingBatch(void);

void SdLogger_Init(void)
{
  sd_logger_head = 0U;
  sd_logger_tail = 0U;
  sd_logger_dropped = 0U;
  sd_logger_batch_length = 0U;
  sd_logger_batch_pending = 0U;
  sd_logger_file_open = 0U;
  sd_logger_last_write_tick = HAL_GetTick();
  /* Make the first mount/open attempt happen on the first task call. */
  sd_logger_last_retry_tick = HAL_GetTick() - SD_LOGGER_RETRY_PERIOD_MS;
  sd_logger_status = SD_LOGGER_NOT_READY;
  sd_logger_last_error = FR_NOT_READY;
}

/* This function is intentionally small: it is called at CAN interrupt level. */
void SdLogger_PushFromIsr(const SdLoggerRecord_t *record)
{
  uint16_t next = (uint16_t)((sd_logger_head + 1U) & (SD_LOGGER_RING_SIZE - 1U));

  if (next == sd_logger_tail)
  {
    sd_logger_dropped++;
    return;
  }
  sd_logger_ring[sd_logger_head] = *record;
  sd_logger_head = next;
}

void SdLogger_Task(void)
{
  uint32_t now = HAL_GetTick();
  FRESULT result;

  if (sd_logger_file_open == 0U)
  {
    if ((now - sd_logger_last_retry_tick) < SD_LOGGER_RETRY_PERIOD_MS)
    {
      return;
    }
    sd_logger_last_retry_tick = now;
    result = SdLogger_OpenNewFile();
    if (result != FR_OK)
    {
      sd_logger_last_error = result;
      sd_logger_status = SD_LOGGER_FILE_ERROR;
      return;
    }
  }

  if (sd_logger_batch_pending == 0U)
  {
    if ((SdLogger_GetPendingCount() < SD_LOGGER_RECORDS_PER_WRITE) &&
        ((now - sd_logger_last_write_tick) < SD_LOGGER_WRITE_PERIOD_MS))
    {
      return;
    }
    result = SdLogger_CreateBatch();
    if (result != FR_OK)
    {
      sd_logger_last_error = result;
      sd_logger_status = SD_LOGGER_FILE_ERROR;
      return;
    }
    if (sd_logger_batch_pending == 0U)
    {
      sd_logger_last_write_tick = now;
      return;
    }
  }

  result = SdLogger_WritePendingBatch();
  if (result != FR_OK)
  {
    /* The formatted batch is retained and will be retried; no popped samples are lost. */
    sd_logger_last_error = result;
    sd_logger_status = SD_LOGGER_FILE_ERROR;
    return;
  }
  sd_logger_last_write_tick = now;
  sd_logger_status = SD_LOGGER_OK;
}

uint16_t SdLogger_GetPendingCount(void)
{
  uint16_t count;
  __disable_irq();
  count = SdLogger_RingCountUnsafe();
  __enable_irq();
  return count;
}

uint32_t SdLogger_GetDroppedCount(void) { return sd_logger_dropped; }
SdLoggerStatus_t SdLogger_GetStatus(void) { return sd_logger_status; }
FRESULT SdLogger_GetLastFatFsError(void) { return sd_logger_last_error; }

static uint16_t SdLogger_RingCountUnsafe(void)
{
  return (uint16_t)((sd_logger_head - sd_logger_tail) & (SD_LOGGER_RING_SIZE - 1U));
}

static uint8_t SdLogger_Peek(SdLoggerRecord_t *record)
{
  uint8_t have_record = 0U;
  __disable_irq();
  if (sd_logger_tail != sd_logger_head)
  {
    *record = sd_logger_ring[sd_logger_tail];
    have_record = 1U;
  }
  __enable_irq();
  return have_record;
}

static void SdLogger_DiscardPeeked(void)
{
  __disable_irq();
  if (sd_logger_tail != sd_logger_head)
  {
    sd_logger_tail = (uint16_t)((sd_logger_tail + 1U) & (SD_LOGGER_RING_SIZE - 1U));
  }
  __enable_irq();
}

static FRESULT SdLogger_OpenNewFile(void)
{
  FRESULT result;
  UINT written;
  uint16_t number;
  static const char header[] =
      "\xEF\xBB\xBF时间戳_ms,电机ID,实际转速_rpm,目标转速_rpm,实际转矩_dNm,"
      "目标转矩_dNm,母线电压_dV,母线电流_dA,相电流_dA,控制器温度_℃,故障代码\r\n";

  result = f_mount(&SDFatFS, SDPath, 1U);
  if (result != FR_OK)
  {
    return result;
  }
  for (number = 0U; number <= SD_LOGGER_FILE_NUMBER_MAX; number++)
  {
    SdLogger_MakeFilePath(number);
    result = f_open(&sd_logger_file, sd_logger_path, FA_WRITE | FA_CREATE_NEW);
    if (result == FR_EXIST)
    {
      continue;
    }
    if (result != FR_OK)
    {
      return result;
    }
    result = f_write(&sd_logger_file, header, sizeof(header) - 1U, &written);
    if ((result != FR_OK) || (written != (sizeof(header) - 1U)))
    {
      (void)f_close(&sd_logger_file);
      return (result != FR_OK) ? result : FR_DISK_ERR;
    }
    result = f_sync(&sd_logger_file);
    if (result != FR_OK)
    {
      (void)f_close(&sd_logger_file);
      return result;
    }
    sd_logger_file_open = 1U;
    return FR_OK;
  }
  return FR_DENIED;
}

/* Avoid printf-family code while creating the first file; this also guarantees
 * an 8.3 name compatible with the project's FatFs configuration (_USE_LFN = 0). */
static void SdLogger_MakeFilePath(uint16_t number)
{
  sd_logger_path[0] = SDPath[0];
  sd_logger_path[1] = ':';
  sd_logger_path[2] = '/';
  sd_logger_path[3] = 'M';
  sd_logger_path[4] = 'L';
  sd_logger_path[5] = 'O';
  sd_logger_path[6] = 'G';
  sd_logger_path[7] = (char)('0' + ((number / 100U) % 10U));
  sd_logger_path[8] = (char)('0' + ((number / 10U) % 10U));
  sd_logger_path[9] = (char)('0' + (number % 10U));
  sd_logger_path[10] = '.';
  sd_logger_path[11] = 'C';
  sd_logger_path[12] = 'S';
  sd_logger_path[13] = 'V';
  sd_logger_path[14] = '\0';
}

static FRESULT SdLogger_CreateBatch(void)
{
  SdLoggerRecord_t record;
  char line[128];
  int length;

  sd_logger_batch_length = 0U;
  while (SdLogger_Peek(&record) != 0U)
  {
    length = snprintf(line, sizeof(line),
                      "%lu,%u,%d,%d,%d,%d,%u,%d,%u,%d,%u\r\n",
                      (unsigned long)record.timestamp_ms, record.motor_id,
                      record.actual_speed_rpm, record.target_speed_rpm,
                      record.actual_torque_dNm, record.target_torque_dNm,
                      record.bus_voltage_dV, record.bus_current_dA,
                      record.phase_current_dA, record.controller_temp_c,
                      record.fault_code);
    if ((length < 0) || ((uint32_t)length >= sizeof(line)))
    {
      /* Leave the record queued so an unexpected format error never loses it. */
      return FR_INT_ERR;
    }
    if (((uint32_t)length >= (sizeof(sd_logger_batch) - sd_logger_batch_length)))
    {
      break;
    }
    memcpy(&sd_logger_batch[sd_logger_batch_length], line, (size_t)length);
    sd_logger_batch_length += (UINT)length;
    SdLogger_DiscardPeeked();
    if (sd_logger_batch_length >= SD_LOGGER_WRITE_THRESHOLD)
    {
      break;
    }
  }
  sd_logger_batch_pending = (sd_logger_batch_length != 0U) ? 1U : 0U;
  return FR_OK;
}

static FRESULT SdLogger_WritePendingBatch(void)
{
  UINT written;
  FRESULT result = f_write(&sd_logger_file, sd_logger_batch, sd_logger_batch_length, &written);
  if ((result != FR_OK) || (written != sd_logger_batch_length))
  {
    return (result != FR_OK) ? result : FR_DISK_ERR;
  }
  result = f_sync(&sd_logger_file);
  if (result == FR_OK)
  {
    sd_logger_batch_pending = 0U;
    sd_logger_batch_length = 0U;
  }
  return result;
}
