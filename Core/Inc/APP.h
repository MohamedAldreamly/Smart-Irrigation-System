/*
 * APP.h
 *
 *  Created on: Jun 18, 2026
 *      Author: Mohamed AL Dreamly
 */

#ifndef INC_APP_H_
#define INC_APP_H_


#include "main.h"
#include "cmsis_os2.h"

/* ================= Soil Status Definitions ================= */

#define SOIL_WET   0
#define SOIL_DRY   1

/* ================= Pump Status Definitions ================= */

#define PUMP_STATUS_OFF   0
#define PUMP_STATUS_ON    1

/* ================= Tank Status Definitions ================= */

#define TANK_STATUS_EMPTY   0
#define TANK_STATUS_OK      1

/* ================= Water Tank Status ================= */

#define TANK_STATUS_EMPTY   0
#define TANK_STATUS_OK      1

/*
 * Event flags used to notify the Pump Control Task.
 */

#define SOIL_DRY_EVENT       (1U << 0)
#define SOIL_WET_EVENT       (1U << 1)
#define ERROR_RESET_EVENT    (1U << 2)
#define MANUAL_WATER_EVENT   (1U << 3)
#define WATER_LOW_EVENT      (1U << 4)
#define WATER_OK_EVENT       (1U << 5)
#define SYSTEM_ERROR_EVENT   (1U << 6)
#define SYSTEM_OK_EVENT      (1U << 7)


/* ================= Irrigation State Machine ================= */

typedef enum
{
    IRRIGATION_STATE_WET = 0,
    IRRIGATION_STATE_WATERING,
    IRRIGATION_STATE_WAIT_AFTER_WATERING,
    IRRIGATION_STATE_MANUAL_WATERING,

    IRRIGATION_STATE_WATER_TANK_EMPTY,
    IRRIGATION_STATE_MAX_ATTEMPTS_ERROR
} IrrigationState_t;

/* ================= System Error Types ================= */

typedef enum
{
    SYSTEM_ERROR_NONE = 0,
    SYSTEM_ERROR_WATER_TANK,
    SYSTEM_ERROR_MAX_ATTEMPTS,
    SYSTEM_ERROR_ADC_SENSOR,
    SYSTEM_ERROR_OLED_I2C
} SystemError_t;

/* ================= Queue Data Types ================= */

typedef struct
{
    uint16_t adcValue;
    uint8_t  soilStatus;
} SoilData_t;



typedef struct
{
    uint16_t adcValue;
    uint8_t  moisturePercent;
    uint8_t  soilStatus;
    uint8_t  pumpStatus;
    uint8_t  waterTankStatus;
    uint8_t  wateringAttempts;
    IrrigationState_t currentState;
    SystemError_t currentError;
} SystemStatus_t;

/* ================= Function Prototypes ================= */

void APP_Init(void);

void APP_SoilSensorTask(void *argument);
void APP_UartPrintTask(void *argument);
void APP_PumpControlTask(void *argument);
void APP_ButtonTask(void *argument);
void APP_WaterTankTask(void *argument);
void APP_SystemMonitorTask(void *argument);
void APP_SystemStatusTask(void *argument);
void APP_OledDisplayTask(void *argument);

#endif /* INC_APP_H_ */
