#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <stdint.h>
#include "ff.h"

/* Keep this a power of two: one record is about 70 bytes in the CSV file. */
#define SD_LOGGER_RING_SIZE 128U

typedef struct
{
  uint32_t timestamp_ms;
  uint8_t  motor_id;
  int16_t  actual_speed_rpm;
  int16_t  target_speed_rpm;
  int16_t  actual_torque_dNm;
  int16_t  target_torque_dNm;
  uint16_t bus_voltage_dV;
  int16_t  bus_current_dA;
  uint16_t phase_current_dA;
  int16_t  controller_temp_c;
  uint8_t  fault_code;
} SdLoggerRecord_t;

typedef enum
{
  SD_LOGGER_OK = 0,
  SD_LOGGER_NOT_READY,
  SD_LOGGER_FILE_ERROR
} SdLoggerStatus_t;

void SdLogger_Init(void);
void SdLogger_PushFromIsr(const SdLoggerRecord_t *record);
void SdLogger_Task(void);
uint16_t SdLogger_GetPendingCount(void);
uint32_t SdLogger_GetDroppedCount(void);
SdLoggerStatus_t SdLogger_GetStatus(void);
FRESULT SdLogger_GetLastFatFsError(void);

#endif /* SD_LOGGER_H */
