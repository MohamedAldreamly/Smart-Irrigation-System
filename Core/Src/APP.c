/*
 * APP.c
 *
 *  Created on: Jun 18, 2026
 *      Author:   Mohamed AL Dreamly
 */

#include "APP.h"

#include "ssd1306.h"
#include "ssd1306_fonts.h"

#include <stdio.h>
#include <string.h>

/* ================= External Handles from CubeMX ================= */

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;

/* ================= External RTOS Object Handles ================= */

extern osMessageQueueId_t SoilAdcQueueHandle;
extern osMessageQueueId_t SystemStatusQueueHandle;
extern osMessageQueueId_t OledStatusQueueHandle;

/*
 * Event Flags created by CubeMX.
 * Used to notify PumpControlTask about soil status.
 */
extern osEventFlagsId_t SoilEventHandle;


extern osMutexId_t UartMutexHandle;
extern osMutexId_t I2cMutexHandle;


/* ================= Private Configuration Defines ================= */

/*
 * Soil moisture threshold values with hysteresis.
 *
 * For most analog soil sensors:
 * Higher ADC value  -> dry soil
 * Lower ADC value   -> wet soil
 *
 * DRY threshold:
 * If ADC value becomes greater than this value,
 * the soil is considered dry.
 *
 * WET threshold:
 * If ADC value becomes lower than this value,
 * the soil is considered wet.
 *
 * Between the two thresholds, the system keeps the last soil status.
 */
#define SOIL_DRY_THRESHOLD   2800
#define SOIL_WET_THRESHOLD   2200

/*
 * ADC calibration values for soil moisture percentage.
 *
 * DRY value means sensor reading when soil is completely dry.
 * WET value means sensor reading when soil is very wet.
 *
 * Note:
 * Most soil moisture sensors give higher ADC value when dry,
 * and lower ADC value when wet.
 */
#define SOIL_ADC_DRY_VALUE      3500U
#define SOIL_ADC_WET_VALUE      1200U

/*
 * ADC sensor fault limits.
 * Values outside this range may indicate sensor disconnect or short circuit.
 */
#define ADC_SENSOR_MIN_VALID    50U
#define ADC_SENSOR_MAX_VALID    4050U

/*
 * Automatic watering configuration.
 *
 * The pump works in watering cycles:
 * 1) Turn ON for PUMP_WATERING_TIME_MS
 * 2) Turn OFF and wait for PUMP_REST_TIME_MS
 * 3) Check soil status again
 *
 * If the soil is still dry after several attempts,
 * the system enters ERROR state.
 */
#define PUMP_WATERING_TIME_MS        10000U
#define PUMP_REST_TIME_MS            5000U
#define PUMP_MAX_WATERING_ATTEMPTS   3U

/*
 * Manual watering duration.
 * When manual button is pressed, the pump turns ON for this time.
 */
#define MANUAL_WATER_TIME_MS      3000U

/*
 * System status report period.
 * The system status will be sent to SystemStatusTask every 2 seconds.
 */
#define SYSTEM_STATUS_REPORT_PERIOD_MS   2000U

/* ================= GPIO Configuration Defines ================= */

/*
 * Pump GPIO configuration.
 * We are using PB0 as pump output.
 */

#define PUMP_GPIO_PORT            GPIOB
#define PUMP_GPIO_PIN             GPIO_PIN_0

#define RESET_BUTTON_GPIO_PORT    GPIOB
#define RESET_BUTTON_GPIO_PIN     GPIO_PIN_1
#define BUTTON_PRESSED            GPIO_PIN_RESET

/*
 * Manual watering configuration.
 * PB10 is used as manual watering button.
 */
#define MANUAL_BUTTON_GPIO_PORT   GPIOB
#define MANUAL_BUTTON_GPIO_PIN    GPIO_PIN_10

/*
 * Water tank level sensor configuration.
 * PB11 is used as digital water level input.
 *
 * With Pull-up configuration:
 * GPIO_PIN_RESET -> Water available
 * GPIO_PIN_SET   -> Water low / tank empty
 */
#define WATER_LEVEL_GPIO_PORT     GPIOB
#define WATER_LEVEL_GPIO_PIN      GPIO_PIN_11

#define WATER_AVAILABLE           GPIO_PIN_RESET
#define WATER_LOW                 GPIO_PIN_SET

/*
 * System indication outputs.
 */
#define HEARTBEAT_LED_GPIO_PORT   GPIOB
#define HEARTBEAT_LED_GPIO_PIN    GPIO_PIN_12

#define ERROR_LED_GPIO_PORT       GPIOB
#define ERROR_LED_GPIO_PIN        GPIO_PIN_13

#define BUZZER_GPIO_PORT          GPIOB
#define BUZZER_GPIO_PIN           GPIO_PIN_14

#define LED_ON                    GPIO_PIN_SET
#define LED_OFF                   GPIO_PIN_RESET

#define BUZZER_ON                 GPIO_PIN_SET
#define BUZZER_OFF                GPIO_PIN_RESET

/*
 * Relay logic:
 * GPIO_PIN_SET   -> Pump ON
 * GPIO_PIN_RESET -> Pump OFF
 *
 * If your relay module is active-low, reverse these two values.
 */
#define PUMP_ON              GPIO_PIN_SET
#define PUMP_OFF             GPIO_PIN_RESET

/*
 * Enable or disable raw ADC debug printing.
 *
 * 1 = UartPrintTask prints ADC and soil status
 * 0 = UartPrintTask receives data but does not print
 */
#define ENABLE_ADC_DEBUG_PRINT   0


/* ================= Shared Application Variables ================= */

/*
 * Latest ADC and moisture values.
 * Updated by SoilSensorTask and used by status reporting.
 */
static volatile uint16_t g_latestAdcValue = 0;
static volatile uint8_t  g_latestMoisturePercent = 0;

/*
 * Error flags.
 * Each error has its own flag.
 * g_currentError stores the highest priority active error.
 */
static volatile uint8_t g_waterTankErrorActive = 0;
static volatile uint8_t g_maxAttemptsErrorActive = 0;
static volatile uint8_t g_adcSensorErrorActive = 0;
static volatile uint8_t g_oledI2cErrorActive = 0;

static volatile SystemError_t g_currentError = SYSTEM_ERROR_NONE;

/* ================= Private Function Prototypes ================= */

static void APP_UART_Print(char *msg);
static void APP_SendSystemStatus(SystemStatus_t *status);
static const char* APP_GetStateString(IrrigationState_t state);
static void APP_OLED_ShowStatus(SystemStatus_t *status);
static const char* APP_GetOledTitle(IrrigationState_t state);
static const char* APP_GetErrorString(SystemError_t error);
static uint8_t APP_CalculateMoisturePercent(uint16_t adcValue);
static uint8_t APP_IsAdcSensorValid(uint16_t adcValue);
static void APP_SetSystemError(SystemError_t error);
static void APP_ClearSystemError(SystemError_t error);
static SystemError_t APP_GetHighestPriorityError(void);
static void APP_UpdateCurrentError(void);


/* ================= APP Initialization ================= */

void APP_Init(void)
{
    /*
     * ADC calibration improves measurement accuracy.
     * It should be called once before using the ADC.
     */
    HAL_ADCEx_Calibration_Start(&hadc1);

    /*
     * Make sure pump is OFF when the system starts.
     */
    HAL_GPIO_WritePin(PUMP_GPIO_PORT, PUMP_GPIO_PIN, PUMP_OFF);

    /*
     * Make sure indication outputs are OFF at startup.
     */
    HAL_GPIO_WritePin(HEARTBEAT_LED_GPIO_PORT,
                      HEARTBEAT_LED_GPIO_PIN,
                      LED_OFF);

    HAL_GPIO_WritePin(ERROR_LED_GPIO_PORT,
                      ERROR_LED_GPIO_PIN,
                      LED_OFF);

    HAL_GPIO_WritePin(BUZZER_GPIO_PORT,
                      BUZZER_GPIO_PIN,
                      BUZZER_OFF);

    /*
     * Startup message to confirm that UART and application are working.
     * We use HAL_UART_Transmit directly here because the RTOS mutex
     * may not be created yet before the scheduler starts.
     */
    char msg[] = "Smart Irrigation System Started\r\n";

    HAL_UART_Transmit(&huart1,
                      (uint8_t *)msg,
                      strlen(msg),
                      HAL_MAX_DELAY);
}


/* ================= Soil Sensor Task ================= */

void APP_SoilSensorTask(void *argument)
{
    SoilData_t soilData;

    /*
     * Stores the last soil status.
     * Initial state is WET because the pump should be OFF at startup.
     */
    uint8_t lastSoilStatus = SOIL_WET;

    for (;;)
    {
        /*
         * Start ADC conversion manually.
         */
        HAL_ADC_Start(&hadc1);

        /*
         * Wait until ADC conversion is complete.
         */
        if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
        {
            /*
             * Read ADC value.
             * ADC range is 0 to 4095 for 12-bit resolution.
             */
        	soilData.adcValue = HAL_ADC_GetValue(&hadc1);

        	/*
        	 * Store latest ADC value for system status reporting.
        	 */
        	g_latestAdcValue = soilData.adcValue;

        	/*
        	 * Calculate latest moisture percentage.
        	 */
        	g_latestMoisturePercent = APP_CalculateMoisturePercent(soilData.adcValue);

        	/*
        	 * Detect ADC sensor fault.
        	 *
        	 * ADC sensor error has lower priority than water tank and max attempts.
        	 * The priority is handled by Error Manager.
        	 */
        	if (APP_IsAdcSensorValid(soilData.adcValue) == 0)
        	{
        	    APP_SetSystemError(SYSTEM_ERROR_ADC_SENSOR);
        	}
        	else
        	{
        	    APP_ClearSystemError(SYSTEM_ERROR_ADC_SENSOR);
        	}

            /*
             * Apply hysteresis logic:
             *
             * If ADC is higher than DRY threshold:
             *      Soil is dry.
             *
             * If ADC is lower than WET threshold:
             *      Soil is wet.
             *
             * If ADC is between both thresholds:
             *      Keep the previous soil status.
             */
            if (soilData.adcValue > SOIL_DRY_THRESHOLD)
            {
                soilData.soilStatus = SOIL_DRY;
            }
            else if (soilData.adcValue < SOIL_WET_THRESHOLD)
            {
                soilData.soilStatus = SOIL_WET;
            }
            else
            {
                soilData.soilStatus = lastSoilStatus;
            }

            /*
             * Send event only when soil status changes.
             */
            if (soilData.soilStatus != lastSoilStatus)
            {
                if (soilData.soilStatus == SOIL_DRY)
                {
                    osEventFlagsSet(SoilEventHandle, SOIL_DRY_EVENT);
                }
                else
                {
                    osEventFlagsSet(SoilEventHandle, SOIL_WET_EVENT);
                }

                /*
                 * Update last status after sending the event.
                 */
                lastSoilStatus = soilData.soilStatus;
            }

            /*
             * Send soil data to UART task through the queue.
             * UART will print ADC value and current soil status.
             */
            osMessageQueuePut(SoilAdcQueueHandle,
                              &soilData,
                              0,
                              0);
        }

        /*
         * Stop ADC after single conversion.
         */
        HAL_ADC_Stop(&hadc1);

        /*
         * Read sensor every 500 ms.
         */
        osDelay(500);
    }
}


/* ================= UART Print Task ================= */

void APP_UartPrintTask(void *argument)
{
    SoilData_t receivedSoilData;
    char uartMsg[100];

    for (;;)
    {
        /*
         * Wait until new soil data is received from the queue.
         */
        if (osMessageQueueGet(SoilAdcQueueHandle,
                              &receivedSoilData,
                              NULL,
                              osWaitForever) == osOK)
        {
#if ENABLE_ADC_DEBUG_PRINT

            /*
             * Print raw ADC value and soil status for debugging only.
             * In normal operation, SystemStatusTask prints the full report.
             */
            if (receivedSoilData.soilStatus == SOIL_DRY)
            {
                sprintf(uartMsg,
                        "ADC = %u | Soil Status = DRY\r\n",
                        receivedSoilData.adcValue);
            }
            else
            {
                sprintf(uartMsg,
                        "ADC = %u | Soil Status = WET\r\n",
                        receivedSoilData.adcValue);
            }

            APP_UART_Print(uartMsg);

#else

            /*
             * Debug printing is disabled.
             * Data is consumed from the queue to keep the queue from filling up.
             */
            (void)receivedSoilData;
            (void)uartMsg;

#endif
        }
    }
}

/* ================= Pump Control Task ================= */

void APP_PumpControlTask(void *argument)
{
    uint32_t flags;

    uint32_t pumpStartTime = 0;
    uint32_t pumpRestStartTime = 0;
    uint32_t manualStartTime = 0;
    uint32_t lastStatusReportTime = 0;

    uint8_t wateringAttempts = 0;
    uint8_t errorMessageSent = 0;


    /*
     * Stores the latest known soil status.
     * This is important because SoilSensorTask sends events only when the status changes.
     */
    uint8_t lastKnownSoilStatus = SOIL_WET;

    /*
     * Stores the water tank status.
     * 1 = Water is available
     * 0 = Water tank is empty
     */
    uint8_t waterTankOk = 0;

    /*
     * Stores the latest system status report.
     */

    SystemStatus_t systemStatus;

    /*
     * Initial system state.
     * The system starts in WET state and the pump is OFF.
     */
    IrrigationState_t currentState = IRRIGATION_STATE_WET;

    /*
     * Make sure the pump is OFF before starting the control loop.
     */
    HAL_GPIO_WritePin(PUMP_GPIO_PORT,
                      PUMP_GPIO_PIN,
                      PUMP_OFF);

    for (;;)
    {
        /*
         * Wait for any system event.
         *
         * Timeout = 500 ms:
         * This allows the task to wake up periodically and check timing logic,
         * such as watering duration, rest duration, and manual watering duration.
         */
        flags = osEventFlagsWait(SoilEventHandle,
                                 SOIL_DRY_EVENT |
                                 SOIL_WET_EVENT |
                                 ERROR_RESET_EVENT |
                                 MANUAL_WATER_EVENT |
                                 WATER_LOW_EVENT |
                                 WATER_OK_EVENT,
                                 osFlagsWaitAny,
                                 500);

        /*
         * If osEventFlagsWait returns timeout or error,
         * clear flags to avoid false event detection.
         */
        if ((flags & osFlagsError) != 0U)
        {
            flags = 0;
        }

        /*
         * Update the latest known soil status.
         */
        if ((flags & SOIL_DRY_EVENT) != 0U)
        {
            lastKnownSoilStatus = SOIL_DRY;
        }

        if ((flags & SOIL_WET_EVENT) != 0U)
        {
            lastKnownSoilStatus = SOIL_WET;
        }

        /*
         * Update water tank status.
         */
        if ((flags & WATER_OK_EVENT) != 0U)
        {
            waterTankOk = 1;
        }

        if ((flags & WATER_LOW_EVENT) != 0U)
        {
            waterTankOk = 0;

            /*
             * If water becomes low at any time,
             * force the pump OFF and enter WATER_TANK_EMPTY state.
             */
            HAL_GPIO_WritePin(PUMP_GPIO_PORT,
                              PUMP_GPIO_PIN,
                              PUMP_OFF);

            currentState = IRRIGATION_STATE_WATER_TANK_EMPTY;
            errorMessageSent = 0;
            osEventFlagsSet(SoilEventHandle, SYSTEM_ERROR_EVENT);
        }

        /*
         * Main irrigation state machine.
         */
        switch (currentState)
        {
            case IRRIGATION_STATE_WET:
            {
                /*
                 * State description:
                 * Soil is considered wet.
                 * Pump is OFF.
                 * System waits for manual watering or dry soil event.
                 */

                if ((flags & MANUAL_WATER_EVENT) != 0U)
                {
                    /*
                     * Manual watering is allowed only if water tank is not empty.
                     */
                    if (waterTankOk == 0)
                    {
                        char msg[] = "Manual watering blocked: water tank is empty.\r\n";
                        APP_UART_Print(msg);
                    }
                    else
                    {
                        char msg[] = "Manual watering started.\r\n";
                        APP_UART_Print(msg);

                        HAL_GPIO_WritePin(PUMP_GPIO_PORT,
                                          PUMP_GPIO_PIN,
                                          PUMP_ON);

                        manualStartTime = osKernelGetTickCount();

                        currentState = IRRIGATION_STATE_MANUAL_WATERING;
                    }
                }
                else if ((flags & SOIL_DRY_EVENT) != 0U)
                {
                    /*
                     * Automatic watering starts only if the tank has water.
                     */
                	if (waterTankOk == 0)
                	{
                	    char msg[] = "Automatic watering blocked: water tank is empty.\r\n";
                	    APP_UART_Print(msg);

                	    currentState = IRRIGATION_STATE_WATER_TANK_EMPTY;
            	        APP_SetSystemError(SYSTEM_ERROR_WATER_TANK);
                	    errorMessageSent = 0;
                	    osEventFlagsSet(SoilEventHandle, SYSTEM_ERROR_EVENT);
                	}
                    else
                    {
                        char msg[] = "Automatic watering started.\r\n";
                        APP_UART_Print(msg);

                        HAL_GPIO_WritePin(PUMP_GPIO_PORT,
                                          PUMP_GPIO_PIN,
                                          PUMP_ON);

                        pumpStartTime = osKernelGetTickCount();

                        wateringAttempts = 0;
                        errorMessageSent = 0;
                        lastKnownSoilStatus = SOIL_DRY;

                        currentState = IRRIGATION_STATE_WATERING;
                    }
                }

                break;
            }

            case IRRIGATION_STATE_WATERING:
            {
                /*
                 * State description:
                 * Automatic watering is active.
                 * Pump is ON.
                 *
                 * If soil becomes wet, stop watering.
                 * If watering time finishes, stop pump and move to rest/check state.
                 */

                if ((flags & SOIL_WET_EVENT) != 0U)
                {
                    HAL_GPIO_WritePin(PUMP_GPIO_PORT,
                                      PUMP_GPIO_PIN,
                                      PUMP_OFF);

                    char msg[] = "Soil is wet. Automatic watering stopped.\r\n";
                    APP_UART_Print(msg);

                    wateringAttempts = 0;
                    lastKnownSoilStatus = SOIL_WET;

                    currentState = IRRIGATION_STATE_WET;
        	        APP_SetSystemError(SYSTEM_ERROR_NONE);

                }
                else
                {
                    /*
                     * One watering cycle finished.
                     * This is not an error yet.
                     * The system will rest before checking the soil again.
                     */
                    if ((osKernelGetTickCount() - pumpStartTime) >= PUMP_WATERING_TIME_MS)
                    {
                        HAL_GPIO_WritePin(PUMP_GPIO_PORT,
                                          PUMP_GPIO_PIN,
                                          PUMP_OFF);

                        pumpRestStartTime = osKernelGetTickCount();

                        char msg[] = "Watering cycle finished. Resting before next check.\r\n";
                        APP_UART_Print(msg);

                        currentState = IRRIGATION_STATE_WAIT_AFTER_WATERING;
                    }
                }

                break;
            }

            case IRRIGATION_STATE_WAIT_AFTER_WATERING:
            {
                /*
                 * State description:
                 * Pump is OFF.
                 * System waits for water to spread in the soil before checking again.
                 */

                HAL_GPIO_WritePin(PUMP_GPIO_PORT,
                                  PUMP_GPIO_PIN,
                                  PUMP_OFF);

                if ((flags & SOIL_WET_EVENT) != 0U)
                {
                    /*
                     * Soil became wet during rest time.
                     * Return to normal WET state.
                     */
                    char msg[] = "Soil became wet during rest time.\r\n";
                    APP_UART_Print(msg);

                    wateringAttempts = 0;
                    lastKnownSoilStatus = SOIL_WET;

                    currentState = IRRIGATION_STATE_WET;
        	        APP_SetSystemError(SYSTEM_ERROR_NONE);

                }
                else
                {
                    /*
                     * After rest time, decide whether to start another watering cycle.
                     */
                    if ((osKernelGetTickCount() - pumpRestStartTime) >= PUMP_REST_TIME_MS)
                    {
                        if (lastKnownSoilStatus == SOIL_DRY)
                        {
                            wateringAttempts++;

                            /*
                             * If max attempts reached, enter ERROR state.
                             */
                            if (wateringAttempts >= PUMP_MAX_WATERING_ATTEMPTS)
                            {
                                currentState = IRRIGATION_STATE_MAX_ATTEMPTS_ERROR;
                                APP_SetSystemError(SYSTEM_ERROR_MAX_ATTEMPTS);
                                errorMessageSent = 0;
                                osEventFlagsSet(SoilEventHandle, SYSTEM_ERROR_EVENT);
                            }
                            else
                            {
                                /*
                                 * Start another watering cycle only if water tank is OK.
                                 */
                            	if (waterTankOk == 0)
                            	{
                            	    char msg[] = "ERROR: Cannot start next cycle. Water tank is empty.\r\n";
                            	    APP_UART_Print(msg);

                            	    currentState = IRRIGATION_STATE_WATER_TANK_EMPTY;
                                    APP_SetSystemError(SYSTEM_ERROR_WATER_TANK);
                            	    errorMessageSent = 0;
                            	    osEventFlagsSet(SoilEventHandle, SYSTEM_ERROR_EVENT);
                            	}
                                else
                                {
                                    char msg[] = "Soil still dry. Starting another watering cycle.\r\n";
                                    APP_UART_Print(msg);

                                    HAL_GPIO_WritePin(PUMP_GPIO_PORT,
                                                      PUMP_GPIO_PIN,
                                                      PUMP_ON);

                                    pumpStartTime = osKernelGetTickCount();

                                    currentState = IRRIGATION_STATE_WATERING;
                                }
                            }
                        }
                        else
                        {
                            /*
                             * Soil is no longer dry.
                             * Return to WET state.
                             */
                            wateringAttempts = 0;
                            currentState = IRRIGATION_STATE_WET;
                            APP_SetSystemError(SYSTEM_ERROR_NONE);
                        }
                    }
                }

                break;
            }

            case IRRIGATION_STATE_MANUAL_WATERING:
            {
                /*
                 * State description:
                 * Manual watering is active.
                 * Pump is ON for MANUAL_WATER_TIME_MS.
                 */

                if ((osKernelGetTickCount() - manualStartTime) >= MANUAL_WATER_TIME_MS)
                {
                    HAL_GPIO_WritePin(PUMP_GPIO_PORT,
                                      PUMP_GPIO_PIN,
                                      PUMP_OFF);

                    char msg[] = "Manual watering finished.\r\n";
                    APP_UART_Print(msg);

                    currentState = IRRIGATION_STATE_WET;
                    APP_SetSystemError(SYSTEM_ERROR_NONE);

                }

                break;
            }

            case IRRIGATION_STATE_WATER_TANK_EMPTY:
            {
                /*
                 * State description:
                 * Water tank is empty.
                 * Pump is always forced OFF.
                 * System stays here until water becomes available
                 * and the user presses the reset button.
                 */
                HAL_GPIO_WritePin(PUMP_GPIO_PORT,
                                  PUMP_GPIO_PIN,
                                  PUMP_OFF);

                /*
                 * Print error message only once.
                 */
                if (errorMessageSent == 0)
                {
                    char errorMsg[] = "ERROR: Water tank is empty. Pump disabled.\r\n";
                    APP_UART_Print(errorMsg);

                    errorMessageSent = 1;
                }

                /*
                 * Manual watering is blocked while tank is empty.
                 */
                if ((flags & MANUAL_WATER_EVENT) != 0U)
                {
                    char blockedMsg[] = "Manual watering blocked: water tank is empty.\r\n";
                    APP_UART_Print(blockedMsg);
                }

                /*
                 * If water becomes available, only update waterTankOk.
                 * The system will still wait for reset button to clear the error.
                 */
                if ((flags & WATER_OK_EVENT) != 0U)
                {
                    waterTankOk = 1;
                }

                /*
                 * Clear this error only if water tank is OK.
                 */
                if ((flags & ERROR_RESET_EVENT) != 0U)
                {
                    if (waterTankOk == 0)
                    {
                        char msg[] = "Cannot clear ERROR: water tank is still empty.\r\n";
                        APP_UART_Print(msg);
                    }
                    else
                    {
                        char resetMsg[] = "Water tank error cleared. System returned to WET state.\r\n";
                        APP_UART_Print(resetMsg);

                        errorMessageSent = 0;
                        pumpStartTime = 0;
                        pumpRestStartTime = 0;
                        manualStartTime = 0;
                        wateringAttempts = 0;
                        lastKnownSoilStatus = SOIL_WET;

                        currentState = IRRIGATION_STATE_WET;
                        APP_ClearSystemError(SYSTEM_ERROR_WATER_TANK);

                        osEventFlagsSet(SoilEventHandle, SYSTEM_OK_EVENT);
                    }
                }

                break;
            }

            case IRRIGATION_STATE_MAX_ATTEMPTS_ERROR:
            {
                /*
                 * State description:
                 * The system tried watering several times,
                 * but the soil remained dry.
                 *
                 * Possible reasons:
                 * - Soil sensor problem
                 * - Pump not delivering water
                 * - Water path blocked
                 * - Sensor placed far from wet area
                 */
                HAL_GPIO_WritePin(PUMP_GPIO_PORT,
                                  PUMP_GPIO_PIN,
                                  PUMP_OFF);

                /*
                 * Print error message only once.
                 */
                if (errorMessageSent == 0)
                {
                    char errorMsg[] = "ERROR: Soil still dry after max watering attempts.\r\n";
                    APP_UART_Print(errorMsg);

                    errorMessageSent = 1;
                }

                /*
                 * Manual watering is blocked in this error state.
                 */
                if ((flags & MANUAL_WATER_EVENT) != 0U)
                {
                    char blockedMsg[] = "Manual watering blocked: max attempts error active.\r\n";
                    APP_UART_Print(blockedMsg);
                }

                /*
                 * Clear this error with reset button.
                 * Water tank must also be OK before returning to normal operation.
                 */
                if ((flags & ERROR_RESET_EVENT) != 0U)
                {
                    if (waterTankOk == 0)
                    {
                        char msg[] = "Cannot clear ERROR: water tank is empty.\r\n";
                        APP_UART_Print(msg);

                        currentState = IRRIGATION_STATE_WATER_TANK_EMPTY;
                        APP_SetSystemError(SYSTEM_ERROR_WATER_TANK);
                        errorMessageSent = 0;
                        osEventFlagsSet(SoilEventHandle, SYSTEM_ERROR_EVENT);

                    }
                    else
                    {
                        char resetMsg[] = "Max attempts error cleared. System returned to WET state.\r\n";
                        APP_UART_Print(resetMsg);

                        errorMessageSent = 0;
                        pumpStartTime = 0;
                        pumpRestStartTime = 0;
                        manualStartTime = 0;
                        wateringAttempts = 0;
                        lastKnownSoilStatus = SOIL_WET;

                        currentState = IRRIGATION_STATE_WET;
                        APP_ClearSystemError(SYSTEM_ERROR_MAX_ATTEMPTS);
                        osEventFlagsSet(SoilEventHandle, SYSTEM_OK_EVENT);
                    }
                }

                break;
            }

            default:
            {
                /*
                 * Safety fallback:
                 * If an unknown state occurs, force pump OFF
                 * and enter MAX_ATTEMPTS_ERROR as a generic safe fault.
                 */
                HAL_GPIO_WritePin(PUMP_GPIO_PORT,
                                  PUMP_GPIO_PIN,
                                  PUMP_OFF);

                currentState = IRRIGATION_STATE_MAX_ATTEMPTS_ERROR;
                errorMessageSent = 0;
                osEventFlagsSet(SoilEventHandle, SYSTEM_ERROR_EVENT);

                break;
            }
        }

        /*
         * Send system status report periodically.
         * This prevents UART flooding with too many reports.
         */
        if ((osKernelGetTickCount() - lastStatusReportTime) >= SYSTEM_STATUS_REPORT_PERIOD_MS)
        {
            lastStatusReportTime = osKernelGetTickCount();

            systemStatus.adcValue = g_latestAdcValue;
            systemStatus.moisturePercent = g_latestMoisturePercent;
            systemStatus.soilStatus = lastKnownSoilStatus;

            systemStatus.pumpStatus =
                (HAL_GPIO_ReadPin(PUMP_GPIO_PORT, PUMP_GPIO_PIN) == PUMP_ON) ?
                PUMP_STATUS_ON : PUMP_STATUS_OFF;

            systemStatus.waterTankStatus =
                (waterTankOk == 1) ? TANK_STATUS_OK : TANK_STATUS_EMPTY;

            systemStatus.wateringAttempts = wateringAttempts;
            systemStatus.currentState = currentState;

            systemStatus.currentError = g_currentError;

            APP_SendSystemStatus(&systemStatus);

        }
    }
}

/* ================= Button Task ================= */

void APP_ButtonTask(void *argument)
{
    GPIO_PinState lastResetButtonState = GPIO_PIN_SET;
    GPIO_PinState currentResetButtonState;

    GPIO_PinState lastManualButtonState = GPIO_PIN_SET;
    GPIO_PinState currentManualButtonState;

    for (;;)
    {
        /*
         * Read reset button state.
         * Pull-up mode:
         * Not pressed = GPIO_PIN_SET
         * Pressed     = GPIO_PIN_RESET
         */
        currentResetButtonState = HAL_GPIO_ReadPin(RESET_BUTTON_GPIO_PORT,
                                                   RESET_BUTTON_GPIO_PIN);

        /*
         * Read manual watering button state.
         */
        currentManualButtonState = HAL_GPIO_ReadPin(MANUAL_BUTTON_GPIO_PORT,
                                                    MANUAL_BUTTON_GPIO_PIN);

        /*
         * Detect reset button falling edge.
         */
        if ((lastResetButtonState == GPIO_PIN_SET) &&
            (currentResetButtonState == BUTTON_PRESSED))
        {
            osDelay(50);

            if (HAL_GPIO_ReadPin(RESET_BUTTON_GPIO_PORT,
                                 RESET_BUTTON_GPIO_PIN) == BUTTON_PRESSED)
            {
                osEventFlagsSet(SoilEventHandle, ERROR_RESET_EVENT);
            }
        }

        /*
         * Detect manual watering button falling edge.
         */
        if ((lastManualButtonState == GPIO_PIN_SET) &&
            (currentManualButtonState == BUTTON_PRESSED))
        {
            osDelay(50);

            if (HAL_GPIO_ReadPin(MANUAL_BUTTON_GPIO_PORT,
                                 MANUAL_BUTTON_GPIO_PIN) == BUTTON_PRESSED)
            {
                osEventFlagsSet(SoilEventHandle, MANUAL_WATER_EVENT);
            }
        }

        lastResetButtonState = currentResetButtonState;
        lastManualButtonState = currentManualButtonState;

        osDelay(50);
    }
}

/* ================= Water Tank Task ================= */

void APP_WaterTankTask(void *argument)
{
    GPIO_PinState currentWaterState;
    GPIO_PinState lastWaterState = WATER_AVAILABLE;

    /*
     * Read initial water sensor state.
     */
    lastWaterState = HAL_GPIO_ReadPin(WATER_LEVEL_GPIO_PORT,
                                      WATER_LEVEL_GPIO_PIN);

    /*
     * Send initial tank status event.
     */
    if (lastWaterState == WATER_AVAILABLE)
    {
        osEventFlagsSet(SoilEventHandle, WATER_OK_EVENT);
    }
    else
    {
        osEventFlagsSet(SoilEventHandle, WATER_LOW_EVENT);
    }

    for (;;)
    {
        /*
         * Read water level sensor.
         */
        currentWaterState = HAL_GPIO_ReadPin(WATER_LEVEL_GPIO_PORT,
                                             WATER_LEVEL_GPIO_PIN);

        /*
         * Send event only when water level state changes.
         */
        if (currentWaterState != lastWaterState)
        {
            /*
             * Simple debounce/filter delay.
             */
            osDelay(50);

            currentWaterState = HAL_GPIO_ReadPin(WATER_LEVEL_GPIO_PORT,
                                                 WATER_LEVEL_GPIO_PIN);

            if (currentWaterState != lastWaterState)
            {
                if (currentWaterState == WATER_AVAILABLE)
                {
                    char msg[] = "Water tank status: OK.\r\n";
                    APP_UART_Print(msg);

                    osEventFlagsSet(SoilEventHandle, WATER_OK_EVENT);
                }
                else
                {
                    char msg[] = "ERROR: Water tank is empty. Pump disabled.\r\n";
                    APP_UART_Print(msg);

                    osEventFlagsSet(SoilEventHandle, WATER_LOW_EVENT);
                }

                lastWaterState = currentWaterState;
            }
        }

        /*
         * Check water level every 200 ms.
         */
        osDelay(200);
    }
}

/* ================= System Monitor Task ================= */

void APP_SystemMonitorTask(void *argument)
{
    uint32_t flags;
    uint8_t systemErrorActive = 0;

    for (;;)
    {
        /*
         * Wait for system status events.
         *
         * Timeout = 500 ms:
         * This allows the task to blink the heartbeat LED
         * even if no event is received.
         */
        flags = osEventFlagsWait(SoilEventHandle,
                                 SYSTEM_ERROR_EVENT | SYSTEM_OK_EVENT,
                                 osFlagsWaitAny,
                                 500);

        /*
         * If timeout or error happens, clear flags.
         */
        if ((flags & osFlagsError) != 0U)
        {
            flags = 0;
        }

        /*
         * Heartbeat LED toggle.
         * This indicates that the RTOS and system are still running.
         */
        HAL_GPIO_TogglePin(HEARTBEAT_LED_GPIO_PORT,
                           HEARTBEAT_LED_GPIO_PIN);

        /*
         * Update error active status based on the current highest priority error.
         *
         * Important:
         * Do not rely only on SYSTEM_OK_EVENT.
         * There may still be another active error after clearing one error.
         */
        if (g_currentError != SYSTEM_ERROR_NONE)
        {
            systemErrorActive = 1;
        }
        else
        {
            systemErrorActive = 0;
        }

        /*
         * If a system error is active, turn ON error LED.
         * If no error is active, turn OFF error LED and buzzer.
         */
        if (systemErrorActive == 1)
        {
            HAL_GPIO_WritePin(ERROR_LED_GPIO_PORT,
                              ERROR_LED_GPIO_PIN,
                              LED_ON);
        }
        else
        {
            HAL_GPIO_WritePin(ERROR_LED_GPIO_PORT,
                              ERROR_LED_GPIO_PIN,
                              LED_OFF);

            HAL_GPIO_WritePin(BUZZER_GPIO_PORT,
                              BUZZER_GPIO_PIN,
                              BUZZER_OFF);
        }

        /*
         * Buzzer alarm pattern while error is active.
         * The buzzer toggles every 500 ms.
         */
        if (systemErrorActive == 1)
        {
            HAL_GPIO_TogglePin(BUZZER_GPIO_PORT,
                               BUZZER_GPIO_PIN);
        }
        else
        {
            HAL_GPIO_WritePin(BUZZER_GPIO_PORT,
                              BUZZER_GPIO_PIN,
                              BUZZER_OFF);
        }
    }
}

/* ================= System Status Task ================= */

void APP_SystemStatusTask(void *argument)
{
    SystemStatus_t receivedStatus;
    char msg[180];

    for (;;)
    {
        /*
         * Wait until a new system status report is received.
         */
        if (osMessageQueueGet(SystemStatusQueueHandle,
                              &receivedStatus,
                              NULL,
                              osWaitForever) == osOK)
        {
            /*
             * Print a clear system report.
             */
        	sprintf(msg,
        	        "\r\n--- System Status ---\r\n"
        	        "State    : %s\r\n"
        	        "Error    : %s\r\n"
        	        "ADC      : %u\r\n"
        	        "Moisture : %u%%\r\n"
        	        "Soil     : %s\r\n"
        	        "Pump     : %s\r\n"
        	        "Tank     : %s\r\n"
        	        "Attempts : %u\r\n"
        	        "---------------------\r\n",
        	        APP_GetStateString(receivedStatus.currentState),
        	        APP_GetErrorString(receivedStatus.currentError),
        	        receivedStatus.adcValue,
        	        receivedStatus.moisturePercent,
        	        (receivedStatus.soilStatus == SOIL_DRY) ? "DRY" : "WET",
        	        (receivedStatus.pumpStatus == PUMP_STATUS_ON) ? "ON" : "OFF",
        	        (receivedStatus.waterTankStatus == TANK_STATUS_OK) ? "OK" : "EMPTY",
        	        receivedStatus.wateringAttempts);

            APP_UART_Print(msg);
        }
    }
}

/* ================= OLED Display Task ================= */

void APP_OledDisplayTask(void *argument)
{
    SystemStatus_t receivedStatus;

    /*
     * Initialize OLED display.
     * I2C access is protected by I2cMutex.
     */
    if (osMutexAcquire(I2cMutexHandle, osWaitForever) == osOK)
    {
        ssd1306_Init();

        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("Smart Irrigation", Font_7x10, White);
        ssd1306_SetCursor(0, 14);
        ssd1306_WriteString("System Started", Font_7x10, White);
        ssd1306_UpdateScreen();

        osMutexRelease(I2cMutexHandle);
    }

    osDelay(1000);

    for (;;)
    {
        /*
         * Wait until a new system status report is received.
         */
        if (osMessageQueueGet(OledStatusQueueHandle,
                              &receivedStatus,
                              NULL,
                              osWaitForever) == osOK)
        {
            APP_OLED_ShowStatus(&receivedStatus);
        }
    }
}
/* ================= System Status Helper ================= */

static void APP_SendSystemStatus(SystemStatus_t *status)
{
    /*
     * Send status to UART reporting task.
     */
    osMessageQueuePut(SystemStatusQueueHandle,
                      status,
                      0,
                      0);

    /*
     * Send status to OLED display task.
     *
     * A separate queue is used because RTOS queues are not broadcast.
     * If two tasks read from the same queue, each message is consumed once only.
     */
    osMessageQueuePut(OledStatusQueueHandle,
                      status,
                      0,
                      0);
}

static const char* APP_GetStateString(IrrigationState_t state)
{
    switch (state)
    {
        case IRRIGATION_STATE_WET:
            return "WET";

        case IRRIGATION_STATE_WATERING:
            return "AUTO_WATERING";

        case IRRIGATION_STATE_WAIT_AFTER_WATERING:
            return "WAIT_AFTER_WATERING";

        case IRRIGATION_STATE_MANUAL_WATERING:
            return "MANUAL_WATERING";

        case IRRIGATION_STATE_WATER_TANK_EMPTY:
            return "WATER_TANK_EMPTY";

        case IRRIGATION_STATE_MAX_ATTEMPTS_ERROR:
            return "MAX_ATTEMPTS_ERROR";

        default:
            return "UNKNOWN";
    }
}

/* ================= OLED Helper Functions ================= */


static const char* APP_GetOledTitle(IrrigationState_t state)
{
    switch (state)
    {
        case IRRIGATION_STATE_WET:
            return "SYSTEM OK";

        case IRRIGATION_STATE_WATERING:
            return "AUTO WATERING";

        case IRRIGATION_STATE_WAIT_AFTER_WATERING:
            return "RESTING";

        case IRRIGATION_STATE_MANUAL_WATERING:
            return "MANUAL WATER";

        case IRRIGATION_STATE_WATER_TANK_EMPTY:
            return "TANK EMPTY";

        case IRRIGATION_STATE_MAX_ATTEMPTS_ERROR:
            return "ERROR";

        default:
            return "UNKNOWN";
    }
}

static void APP_OLED_ShowStatus(SystemStatus_t *status)
{
    char line[32];

    /*
     * Protect I2C bus before accessing OLED.
     */
    if (osMutexAcquire(I2cMutexHandle, osWaitForever) == osOK)
    {
        ssd1306_Fill(Black);

        /*
         * Header line: clear title depending on system state.
         */
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString(APP_GetOledTitle(status->currentState),
                            Font_7x10,
                            White);

        /*
         * Line 2: ADC value and soil status.
         */
        ssd1306_SetCursor(0, 14);
        sprintf(line,
                "ADC:%u M:%u%%",
                status->adcValue,
                status->moisturePercent);
        ssd1306_WriteString(line, Font_6x8, White);

        /*
         * Line 3: Pump and tank status.
         */
        ssd1306_SetCursor(0, 24);
        sprintf(line,
                "Pump:%s Tank:%s",
                (status->pumpStatus == PUMP_STATUS_ON) ? "ON" : "OFF",
                (status->waterTankStatus == TANK_STATUS_OK) ? "OK" : "EMPTY");
        ssd1306_WriteString(line, Font_6x8, White);

        /*
         * Line 4: Attempts.
         */
        ssd1306_SetCursor(0, 34);
        sprintf(line,
                "Attempts:%u",
                status->wateringAttempts);
        ssd1306_WriteString(line, Font_6x8, White);

        /*
         * Line 5: Error status.
         */
        ssd1306_SetCursor(0, 48);
        sprintf(line, "Err:%s", APP_GetErrorString(status->currentError));
        ssd1306_WriteString(line, Font_6x8, White);

        switch (status->currentState)
        {
            case IRRIGATION_STATE_WET:
                ssd1306_WriteString("Soil moisture OK", Font_6x8, White);
                break;

            case IRRIGATION_STATE_WATERING:
                ssd1306_WriteString("Pump is running", Font_6x8, White);
                break;

            case IRRIGATION_STATE_WAIT_AFTER_WATERING:
                ssd1306_WriteString("Waiting for soil", Font_6x8, White);
                break;

            case IRRIGATION_STATE_MANUAL_WATERING:
                ssd1306_WriteString("Manual pump ON", Font_6x8, White);
                break;

            case IRRIGATION_STATE_WATER_TANK_EMPTY:
                ssd1306_WriteString("Refill water tank", Font_6x8, White);
                break;

            case IRRIGATION_STATE_MAX_ATTEMPTS_ERROR:
                ssd1306_WriteString("Check soil/pump", Font_6x8, White);
                break;

            default:
                ssd1306_WriteString("Unknown state", Font_6x8, White);
                break;
        }

        /*
         * Send buffer to OLED.
         */
        ssd1306_UpdateScreen();

        /*
         * Release I2C bus.
         */
        osMutexRelease(I2cMutexHandle);
    }
}

static void APP_UART_Print(char *msg)
{
    if (osMutexAcquire(UartMutexHandle, osWaitForever) == osOK)
    {
        HAL_UART_Transmit(&huart1,
                          (uint8_t *)msg,
                          strlen(msg),
                          HAL_MAX_DELAY);

        osMutexRelease(UartMutexHandle);
    }
}

static const char* APP_GetErrorString(SystemError_t error)
{
    switch (error)
    {
        case SYSTEM_ERROR_NONE:
            return "NONE";

        case SYSTEM_ERROR_WATER_TANK:
            return "WATER TANK";

        case SYSTEM_ERROR_MAX_ATTEMPTS:
            return "MAX ATTEMPTS";

        case SYSTEM_ERROR_ADC_SENSOR:
            return "ADC SENSOR";

        case SYSTEM_ERROR_OLED_I2C:
            return "OLED I2C";

        default:
            return "UNKNOWN";
    }
}

static uint8_t APP_IsAdcSensorValid(uint16_t adcValue)
{
    /*
     * Check if ADC value is inside the valid sensor range.
     */
    if ((adcValue < ADC_SENSOR_MIN_VALID) || (adcValue > ADC_SENSOR_MAX_VALID))
    {
        return 0;
    }

    return 1;
}

static uint8_t APP_CalculateMoisturePercent(uint16_t adcValue)
{
    uint32_t moisturePercent;

    /*
     * If ADC value is greater than or equal dry calibration value,
     * moisture is 0%.
     */
    if (adcValue >= SOIL_ADC_DRY_VALUE)
    {
        return 0;
    }

    /*
     * If ADC value is less than or equal wet calibration value,
     * moisture is 100%.
     */
    if (adcValue <= SOIL_ADC_WET_VALUE)
    {
        return 100;
    }

    /*
     * Convert ADC value to moisture percentage.
     *
     * Since higher ADC means drier soil:
     *
     * moisture = ((DRY_ADC - adc) * 100) / (DRY_ADC - WET_ADC)
     */
    moisturePercent =
        ((uint32_t)(SOIL_ADC_DRY_VALUE - adcValue) * 100U) /
        (SOIL_ADC_DRY_VALUE - SOIL_ADC_WET_VALUE);

    return (uint8_t)moisturePercent;
}

/* ================= Error Manager ================= */

static void APP_SetSystemError(SystemError_t error)
{
    switch (error)
    {
        case SYSTEM_ERROR_WATER_TANK:
            g_waterTankErrorActive = 1;
            break;

        case SYSTEM_ERROR_MAX_ATTEMPTS:
            g_maxAttemptsErrorActive = 1;
            break;

        case SYSTEM_ERROR_ADC_SENSOR:
            g_adcSensorErrorActive = 1;
            break;

        case SYSTEM_ERROR_OLED_I2C:
            g_oledI2cErrorActive = 1;
            break;

        case SYSTEM_ERROR_NONE:
        default:
            break;
    }

    APP_UpdateCurrentError();
}

static void APP_ClearSystemError(SystemError_t error)
{
    switch (error)
    {
        case SYSTEM_ERROR_WATER_TANK:
            g_waterTankErrorActive = 0;
            break;

        case SYSTEM_ERROR_MAX_ATTEMPTS:
            g_maxAttemptsErrorActive = 0;
            break;

        case SYSTEM_ERROR_ADC_SENSOR:
            g_adcSensorErrorActive = 0;
            break;

        case SYSTEM_ERROR_OLED_I2C:
            g_oledI2cErrorActive = 0;
            break;

        case SYSTEM_ERROR_NONE:
        default:
            break;
    }

    APP_UpdateCurrentError();
}

static SystemError_t APP_GetHighestPriorityError(void)
{
    /*
     * Error priority from highest to lowest:
     *
     * 1. WATER_TANK      -> Pump must stay OFF
     * 2. MAX_ATTEMPTS    -> Irrigation failed
     * 3. ADC_SENSOR      -> Soil reading may be invalid
     * 4. OLED_I2C        -> Display issue only
     */

    if (g_waterTankErrorActive == 1)
    {
        return SYSTEM_ERROR_WATER_TANK;
    }

    if (g_maxAttemptsErrorActive == 1)
    {
        return SYSTEM_ERROR_MAX_ATTEMPTS;
    }

    if (g_adcSensorErrorActive == 1)
    {
        return SYSTEM_ERROR_ADC_SENSOR;
    }

    if (g_oledI2cErrorActive == 1)
    {
        return SYSTEM_ERROR_OLED_I2C;
    }

    return SYSTEM_ERROR_NONE;
}

static void APP_UpdateCurrentError(void)
{
    g_currentError = APP_GetHighestPriorityError();
}
