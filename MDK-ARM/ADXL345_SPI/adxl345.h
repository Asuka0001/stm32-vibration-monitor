/**
  ******************************************************************************
  * @file    adxl345.h
  * @brief   ADXL345 3-Axis Accelerometer Driver Header with Velocity Integration
  * @author  Haolan Li
  * @date    Created: 2025-12-30, Updated: 2025-12-31
  ******************************************************************************
  */

#ifndef __ADXL345_H
#define __ADXL345_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <stdbool.h>

/* Constants -----------------------------------------------------------------*/
#define ADXL345_DEVICE_ID      0xE5    // Expected device ID

/* Error Codes ---------------------------------------------------------------*/
typedef enum {
    ADXL_OK               = 0x00,      // Operation successful
    ADXL_ERROR            = 0x01,      // Generic error
    ADXL_TIMEOUT          = 0x02,      // Communication timeout
    ADXL_INVALID_PARAM    = 0x03,      // Invalid parameter
    ADXL_DEVICE_NOT_FOUND = 0x04,      // Device ID mismatch
    ADXL_NOT_INITIALIZED  = 0x05,      // Device not initialized
} ADXL_StatusTypeDef;

/* Range Settings ------------------------------------------------------------*/
typedef enum {
    ADXL345_RANGE_2G  = 0x00,  // +-2g range
    ADXL345_RANGE_4G  = 0x01,  // +-4g range
    ADXL345_RANGE_8G  = 0x02,  // +-8g range
    ADXL345_RANGE_16G = 0x03   // +-16g range
} ADXL345_RangeTypeDef;

/* Data Rate Settings --------------------------------------------------------*/
typedef enum {
    ADXL345_DATARATE_0_10_HZ = 0x00,   // 0.10 Hz
    ADXL345_DATARATE_0_20_HZ = 0x01,   // 0.20 Hz
    ADXL345_DATARATE_0_39_HZ = 0x02,   // 0.39 Hz
    ADXL345_DATARATE_0_78_HZ = 0x03,   // 0.78 Hz
    ADXL345_DATARATE_1_56_HZ = 0x04,   // 1.56 Hz
    ADXL345_DATARATE_3_13_HZ = 0x05,   // 3.13 Hz
    ADXL345_DATARATE_6_25HZ  = 0x06,   // 6.25 Hz
    ADXL345_DATARATE_12_5_HZ = 0x07,   // 12.5 Hz
    ADXL345_DATARATE_25_HZ   = 0x08,   // 25 Hz
    ADXL345_DATARATE_50_HZ   = 0x09,   // 50 Hz
    ADXL345_DATARATE_100_HZ  = 0x0A,   // 100 Hz
    ADXL345_DATARATE_200_HZ  = 0x0B,   // 200 Hz
    ADXL345_DATARATE_400_HZ  = 0x0C,   // 400 Hz
    ADXL345_DATARATE_800_HZ  = 0x0D,   // 800 Hz
    ADXL345_DATARATE_1600_HZ = 0x0E,   // 1600 Hz
    ADXL345_DATARATE_3200_HZ = 0x0F    // 3200 Hz
} ADXL345_DataRateTypeDef;

/* Configuration Structure ---------------------------------------------------*/
typedef struct {
    ADXL345_RangeTypeDef      range;          // Measurement range
    ADXL345_DataRateTypeDef   dataRate;       // Output data rate
    uint8_t                   fullResolution; // Full resolution mode
    uint8_t                   spiMode;        // SPI mode (3-wire/4-wire)
} ADXL345_ConfigTypeDef;

/* Default Configuration -----------------------------------------------------*/
#define ADXL345_DEFAULT_CONFIG {               \
    .range = ADXL345_RANGE_16G,                \
    .dataRate = ADXL345_DATARATE_1600_HZ,      \
    .fullResolution = 1,                       \
    .spiMode = 0                               \
}

/* Raw Data Structure --------------------------------------------------------*/
typedef struct {
    int16_t x;      // Raw X-axis data
    int16_t y;      // Raw Y-axis data
    int16_t z;      // Raw Z-axis data
} ADXL345_RawData_t;

/* Acceleration Data Structure -----------------------------------------------*/
typedef struct {
    float x;        // X-axis acceleration (g)
    float y;        // Y-axis acceleration (g)
    float z;        // Z-axis acceleration (g)
} ADXL345_Acceleration_t;

/* Angle Data Structure ------------------------------------------------------*/
typedef struct {
    float roll;     // Roll angle (degrees)
    float pitch;    // Pitch angle (degrees)
} ADXL345_Angles_t;

/* Velocity Data Structure ---------------------------------------------------*/
typedef struct {
    float x;        // X-axis velocity (m/s)
    float y;        // Y-axis velocity (m/s)
    float z;        // Z-axis velocity (m/s)
} ADXL345_Velocity_t;




/* ZUPT Configuration Structure ----------------------------------------------*/
typedef struct {
    uint8_t  windowSize;                // Detection window size (samples)
    float    varianceThreshold;         // Acceleration vector variance threshold (g^2)
    float    magnitudeVarianceThreshold; // Acceleration magnitude variance threshold (g^2)
    uint16_t minStaticDuration;         // Min static duration (ms)
		float    linearRmsThreshold;         // New: RMS threshold for linear acceleration veto (g)
} ADXL345_ZUPT_Config_t;

/* Modified ZUPT Configuration for 1000Hz sampling */
#define ADXL345_DEFAULT_ZUPT_CONFIG {      \
    .windowSize = 60,                      /* 60ms window for zero-velocity detection */ \
    .varianceThreshold = 0.005f,           /* Threshold for acceleration variance */ \
    .magnitudeVarianceThreshold = 0.002f,  /* Threshold for magnitude variance */ \
    .minStaticDuration = 100,               /* Minimum static duration of 100ms */ \
		.linearRmsThreshold = 0.03f             /* Threshold for linear acceleration RMS veto */ \
}

/* Velocity Integration State ------------------------------------------------*/
typedef struct {
    ADXL345_Acceleration_t prevLinearAccel;  // Previous linear acceleration (gravity removed)
    ADXL345_Velocity_t     velocity;         // Current velocity

    uint8_t  staticCandidate;                // Potential static state flag
    uint32_t lastTimestamp;                  // Last sample timestamp (ms)
    uint8_t  isStatic;                       // ZUPT static state flag
    uint32_t staticStartTime;                // Static state start time

    #define MAX_ACCEL_BUFFER_SIZE 100
    ADXL345_Acceleration_t accelBuffer[MAX_ACCEL_BUFFER_SIZE];   // raw accel (ZUPT main)
    ADXL345_Acceleration_t linearBuffer[MAX_ACCEL_BUFFER_SIZE];  // linear accel (ZUPT veto)

    uint8_t bufferIndex;                     // Buffer current index
    uint8_t bufferFilled;                    // Buffer filled flag
} ADXL345_VelocityState_t;



/* Device Handle Structure ---------------------------------------------------*/
typedef struct {
    SPI_HandleTypeDef         *hspi;               // SPI handle
    GPIO_TypeDef              *csPort;             // Chip Select GPIO port
    uint16_t                  csPin;               // Chip Select GPIO pin
    ADXL345_ConfigTypeDef     config;              // Device configuration
    uint8_t                   isInitialized;       // Initialization flag
    float                     scaleFactor;         // Current scale factor (g/LSB)
    int8_t                    offset[3];           // Calibration offsets

    // Velocity integration members
    ADXL345_VelocityState_t   velState;            // Velocity state
    ADXL345_ZUPT_Config_t     zuptConfig;          // ZUPT configuration
    uint8_t                   velocityEnabled;     // Velocity tracking enabled

    // Timer-based sampling members
    volatile uint8_t          dataReadyFlag;       // Data ready flag for polling
    uint32_t                  lastSampleTime;      // Last sample timestamp (ms)
    uint32_t                  sampleInterval;      // Sample interval (ms)
    ADXL345_Acceleration_t    lastAccel;           // Last acceleration reading
		ADXL345_Acceleration_t 		lastRawAccel;


		float gravity[3];
    float accelBias[3];



} ADXL345_HandleTypeDef;

/* Function Prototypes -------------------------------------------------------*/

/* Initialization and Configuration Functions --------------------------------*/
ADXL_StatusTypeDef ADXL345_Init(ADXL345_HandleTypeDef *hdev,
                                SPI_HandleTypeDef *hspi,
                                GPIO_TypeDef *csPort,
                                uint16_t csPin);
ADXL_StatusTypeDef ADXL345_DeInit(ADXL345_HandleTypeDef *hdev);
ADXL_StatusTypeDef ADXL345_Config(ADXL345_HandleTypeDef *hdev,
                                 const ADXL345_ConfigTypeDef *config);

/* Basic Read Functions ------------------------------------------------------*/
ADXL_StatusTypeDef ADXL345_ReadRaw(ADXL345_HandleTypeDef *hdev,
                                   ADXL345_RawData_t *rawData);
ADXL_StatusTypeDef ADXL345_ReadAcceleration(ADXL345_HandleTypeDef *hdev,
                                            ADXL345_Acceleration_t *accel);
ADXL_StatusTypeDef ADXL345_ReadAverageAcceleration(ADXL345_HandleTypeDef *hdev,
                                                   ADXL345_Acceleration_t *accel,
                                                   uint8_t numSamples);

/* Calibration Functions -----------------------------------------------------*/
ADXL_StatusTypeDef ADXL345_AutoCalibrate(ADXL345_HandleTypeDef *hdev,
                                         uint8_t numSamples);
ADXL_StatusTypeDef ADXL345_SetCalibrationOffsets(ADXL345_HandleTypeDef *hdev,
                                                int8_t offsetX,
                                                int8_t offsetY,
                                                int8_t offsetZ);
ADXL_StatusTypeDef ADXL345_GetCalibrationOffsets(ADXL345_HandleTypeDef *hdev,
                                                int8_t *offsetX,
                                                int8_t *offsetY,
                                                int8_t *offsetZ);

/* Utility Functions ---------------------------------------------------------*/
ADXL_StatusTypeDef ADXL345_SelfTest(ADXL345_HandleTypeDef *hdev);
ADXL_StatusTypeDef ADXL345_ReadID(ADXL345_HandleTypeDef *hdev, uint8_t *deviceID);
ADXL_StatusTypeDef ADXL345_Reset(ADXL345_HandleTypeDef *hdev);

/* Angle Calculation Functions -----------------------------------------------*/
ADXL_StatusTypeDef ADXL345_CalculateAngles(ADXL345_HandleTypeDef *hdev,
                                          ADXL345_Angles_t *angles);
ADXL_StatusTypeDef ADXL345_CalculateAnglesFromAccel(ADXL345_Acceleration_t *accel,
                                                   ADXL345_Angles_t *angles);

/* Power Management Functions ------------------------------------------------*/
ADXL_StatusTypeDef ADXL345_SetPowerMode(ADXL345_HandleTypeDef *hdev,
                                       uint8_t powerMode);
ADXL_StatusTypeDef ADXL345_SetSleepMode(ADXL345_HandleTypeDef *hdev,
                                       uint8_t enable,
                                       uint8_t frequency);

/* Data Conversion Helpers ---------------------------------------------------*/
float ADXL345_RawToG(ADXL345_HandleTypeDef *hdev, int16_t rawValue);
int16_t ADXL345_GToRaw(ADXL345_HandleTypeDef *hdev, float gValue);

/* Velocity Integration Functions --------------------------------------------*/
ADXL_StatusTypeDef ADXL345_EnableVelocityTracking(ADXL345_HandleTypeDef *hdev,
                                                   uint8_t enable);
ADXL_StatusTypeDef ADXL345_ConfigZUPT(ADXL345_HandleTypeDef *hdev,
                                     const ADXL345_ZUPT_Config_t *config);
ADXL_StatusTypeDef ADXL345_UpdateVelocity(ADXL345_HandleTypeDef *hdev,
                                          ADXL345_Acceleration_t *rawAccel,
                                          ADXL345_Acceleration_t *linearAccel,
                                          uint32_t timestamp);

ADXL_StatusTypeDef ADXL345_GetVelocity(ADXL345_HandleTypeDef *hdev,
                                       ADXL345_Velocity_t *velocity);
ADXL_StatusTypeDef ADXL345_ResetVelocity(ADXL345_HandleTypeDef *hdev);

/* ZUPT Functions ------------------------------------------------------------*/
ADXL_StatusTypeDef ADXL345_DetectZeroVelocity(ADXL345_HandleTypeDef *hdev,
                                              uint8_t *isStatic);

/* Gravity Compensation ------------------------------------------------------*/
ADXL_StatusTypeDef ADXL345_RemoveGravity(ADXL345_HandleTypeDef *hdev,
                                        ADXL345_Acceleration_t *accel,
                                        ADXL345_Angles_t *angles);

/* Timer-based sampling functions (for 1000Hz sampling) */
ADXL_StatusTypeDef ADXL345_TimedSample(ADXL345_HandleTypeDef *hdev);
uint8_t ADXL345_IsDataReady(ADXL345_HandleTypeDef *hdev);
ADXL_StatusTypeDef ADXL345_GetLastAcceleration(ADXL345_HandleTypeDef *hdev,
                                               ADXL345_Acceleration_t *accel);

/* Sample interval configuration */
ADXL_StatusTypeDef ADXL345_SetSampleInterval(ADXL345_HandleTypeDef *hdev,
                                            uint32_t interval_ms);


/* Debug Functions -----------------------------------------------------------*/
void ADXL345_PrintZUPTStatus(ADXL345_HandleTypeDef *hdev);




#ifdef __cplusplus
}
#endif

#endif /* __ADXL345_H */