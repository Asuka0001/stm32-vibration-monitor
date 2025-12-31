#include "adxl345.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "vibration_monitor.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ADXL345_SPI_TIMEOUT           100
#define ADXL345_ID_READ_RETRIES       10
#define ADXL345_ID_READ_DELAY         10

#define ADXL345_REG_DEVID             0x00
#define ADXL345_REG_THRESH_TAP        0x1D
#define ADXL345_REG_OFSX              0x1E
#define ADXL345_REG_OFSY              0x1F
#define ADXL345_REG_OFSZ              0x20
#define ADXL345_REG_DUR               0x21
#define ADXL345_REG_LATENT            0x22
#define ADXL345_REG_WINDOW            0x23
#define ADXL345_REG_THRESH_ACT        0x24
#define ADXL345_REG_THRESH_INACT      0x25
#define ADXL345_REG_TIME_INACT        0x26
#define ADXL345_REG_ACT_INACT_CTL     0x27
#define ADXL345_REG_THRESH_FF         0x28
#define ADXL345_REG_TIME_FF           0x29
#define ADXL345_REG_TAP_AXES          0x2A
#define ADXL345_REG_ACT_TAP_STATUS    0x2B
#define ADXL345_REG_BW_RATE           0x2C
#define ADXL345_REG_POWER_CTL         0x2D
#define ADXL345_REG_INT_ENABLE        0x2E
#define ADXL345_REG_INT_MAP           0x2F
#define ADXL345_REG_INT_SOURCE        0x30
#define ADXL345_REG_DATA_FORMAT       0x31
#define ADXL345_REG_DATAX0            0x32
#define ADXL345_REG_DATAX1            0x33
#define ADXL345_REG_DATAY0            0x34
#define ADXL345_REG_DATAY1            0x35
#define ADXL345_REG_DATAZ0            0x36
#define ADXL345_REG_DATAZ1            0x37
#define ADXL345_REG_FIFO_CTL          0x38
#define ADXL345_REG_FIFO_STATUS       0x39

#define ADXL345_FORMAT_FULL_RES       (1 << 3)
#define ADXL345_FORMAT_JUSTIFY_MSB    (1 << 2)
#define ADXL345_FORMAT_RANGE_MASK     0x03

#define ADXL345_POWER_CTL_MEASURE     (1 << 3)
#define ADXL345_POWER_CTL_SLEEP       (1 << 2)
#define ADXL345_POWER_CTL_WAKEUP_MASK 0x03

#define ADXL345_SPI_READ_FLAG         0x80
#define ADXL345_SPI_MULTI_FLAG        0x40

#define CS_LOW(dev)    HAL_GPIO_WritePin((dev)->csPort, (dev)->csPin, GPIO_PIN_RESET)
#define CS_HIGH(dev)   HAL_GPIO_WritePin((dev)->csPort, (dev)->csPin, GPIO_PIN_SET)

#define MAX_WINDOW_SIZE 100

//#define ADXL_FILTER_ALPHA       0.85f
#define ADXL_ALPHA_STATIC   0.8f
#define ADXL_ALPHA_DYNAMIC  0.9f

#define ADXL_BIAS_LEARNING_RATE 0.001f

#define ADXL_MOTION_DEADZONE    0.03f

#define ADXL_VELOCITY_DAMPING   0.9992f

static ADXL_StatusTypeDef ADXL345_ReadRegister(ADXL345_HandleTypeDef *hdev,
                                              uint8_t reg, uint8_t *data, uint8_t length);
static ADXL_StatusTypeDef ADXL345_WriteRegister(ADXL345_HandleTypeDef *hdev,
                                               uint8_t reg, uint8_t value);
static void ADXL345_CalculateScaleFactor(ADXL345_HandleTypeDef *hdev);

static const float ADXL345_ScaleFactors[] = {
    0.0039f,
    0.0078f,
    0.0156f,
    0.0312f
};

ADXL_StatusTypeDef ADXL345_Init(ADXL345_HandleTypeDef *hdev,
                                SPI_HandleTypeDef *hspi,
                                GPIO_TypeDef *csPort,
                                uint16_t csPin)
{
    ADXL_StatusTypeDef status = ADXL_OK;
    uint8_t deviceID = 0;
    ADXL345_ConfigTypeDef defaultConfig = ADXL345_DEFAULT_CONFIG;
    ADXL345_ZUPT_Config_t defaultZUPT = ADXL345_DEFAULT_ZUPT_CONFIG;

    if (hdev == NULL || hspi == NULL || csPort == NULL) {
        return ADXL_INVALID_PARAM;
    }

    memset(hdev, 0, sizeof(ADXL345_HandleTypeDef));
    memset(hdev->gravity, 0, sizeof(hdev->gravity));
    memset(hdev->accelBias, 0, sizeof(hdev->accelBias));

    hdev->hspi = hspi;
    hdev->csPort = csPort;
    hdev->csPin = csPin;
    hdev->isInitialized = 0;

    memset(&hdev->velState, 0, sizeof(ADXL345_VelocityState_t));
    hdev->velState.lastTimestamp = HAL_GetTick();
    hdev->velocityEnabled = 0;

    memcpy(&hdev->zuptConfig, &defaultZUPT, sizeof(ADXL345_ZUPT_Config_t));

    hdev->dataReadyFlag = 0;
    hdev->lastSampleTime = 0;
    hdev->sampleInterval = 1;
    memset(&hdev->lastAccel, 0, sizeof(ADXL345_Acceleration_t));

    CS_HIGH(hdev);
    HAL_Delay(10);

    status = ADXL345_ReadID(hdev, &deviceID);
    if (status != ADXL_OK) {
        return status;
    }

    if (deviceID != ADXL345_DEVICE_ID) {
        return ADXL_DEVICE_NOT_FOUND;
    }

    status = ADXL345_Config(hdev, &defaultConfig);
    if (status != ADXL_OK) {
        return status;
    }

    hdev->isInitialized = 1;
		
		// Add
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    ADXL345_RawData_t rawInit;
    
    for(int i = 0; i < 50; i++) {
        ADXL345_ReadRaw(hdev, &rawInit);
        sumX += rawInit.x;
        sumY += rawInit.y;
        sumZ += rawInit.z;
        HAL_Delay(1); // 1ms delay for 1000Hz simulation during init
    }
    
    hdev->gravity[0] = ADXL345_RawToG(hdev, sumX / 50);
    hdev->gravity[1] = ADXL345_RawToG(hdev, sumY / 50);
    hdev->gravity[2] = ADXL345_RawToG(hdev, sumZ / 50);
    // 

    ADXL345_WriteRegister(hdev, ADXL345_REG_OFSX, 0);
    ADXL345_WriteRegister(hdev, ADXL345_REG_OFSY, 0);
    ADXL345_WriteRegister(hdev, ADXL345_REG_OFSZ, 0);

    return ADXL_OK;
}

ADXL_StatusTypeDef ADXL345_DeInit(ADXL345_HandleTypeDef *hdev)
{
    if (hdev == NULL) {
        return ADXL_INVALID_PARAM;
    }

    ADXL345_WriteRegister(hdev, ADXL345_REG_POWER_CTL, 0x00);

    memset(hdev, 0, sizeof(ADXL345_HandleTypeDef));

    return ADXL_OK;
}

ADXL_StatusTypeDef ADXL345_Config(ADXL345_HandleTypeDef *hdev,
                                 const ADXL345_ConfigTypeDef *config)
{
    ADXL_StatusTypeDef status = ADXL_OK;
    uint8_t dataFormat = 0;
    uint8_t powerCtl = 0;

    if (hdev == NULL || config == NULL) {
        return ADXL_INVALID_PARAM;
    }

    ADXL345_WriteRegister(hdev, ADXL345_REG_POWER_CTL, 0x00);
    HAL_Delay(5);

    dataFormat = (config->range & ADXL345_FORMAT_RANGE_MASK);

    if (config->fullResolution) {
        dataFormat |= ADXL345_FORMAT_FULL_RES;
    }

    if (config->spiMode) {
        dataFormat |= ADXL345_FORMAT_JUSTIFY_MSB;
    }

    status = ADXL345_WriteRegister(hdev, ADXL345_REG_DATA_FORMAT, dataFormat);
    if (status != ADXL_OK) {
        ADXL345_WriteRegister(hdev, ADXL345_REG_POWER_CTL, 0x00);
        return status;
    }

    status = ADXL345_WriteRegister(hdev, ADXL345_REG_BW_RATE, config->dataRate);
    if (status != ADXL_OK) {
        ADXL345_WriteRegister(hdev, ADXL345_REG_POWER_CTL, 0x00);
        return status;
    }

    powerCtl = ADXL345_POWER_CTL_MEASURE;
    status = ADXL345_WriteRegister(hdev, ADXL345_REG_POWER_CTL, powerCtl);
    if (status != ADXL_OK) {
        return status;
    }

    HAL_Delay(20);

    ADXL345_CalculateScaleFactor(hdev);

    memcpy(&hdev->config, config, sizeof(ADXL345_ConfigTypeDef));

    return ADXL_OK;
}

ADXL_StatusTypeDef ADXL345_ReadRaw(ADXL345_HandleTypeDef *hdev,
                                   ADXL345_RawData_t *rawData)
{
    ADXL_StatusTypeDef status = ADXL_OK;
    uint8_t data[6] = {0};

    if (hdev == NULL || rawData == NULL) {
        return ADXL_INVALID_PARAM;
    }

    if (!hdev->isInitialized) {
        return ADXL_NOT_INITIALIZED;
    }

    status = ADXL345_ReadRegister(hdev, ADXL345_REG_DATAX0, data, 6);
    if (status != ADXL_OK) {
        return status;
    }

    rawData->x = (int16_t)((data[1] << 8) | data[0]);
    rawData->y = (int16_t)((data[3] << 8) | data[2]);
    rawData->z = (int16_t)((data[5] << 8) | data[4]);

    return ADXL_OK;
}

ADXL_StatusTypeDef ADXL345_ReadAcceleration(ADXL345_HandleTypeDef *hdev,
                                            ADXL345_Acceleration_t *accel)
{
    ADXL_StatusTypeDef status = ADXL_OK;
    ADXL345_RawData_t rawData;

    if (hdev == NULL || accel == NULL) {
        return ADXL_INVALID_PARAM;
    }

    status = ADXL345_ReadRaw(hdev, &rawData);
    if (status != ADXL_OK) {
        return status;
    }

    accel->x = ADXL345_RawToG(hdev, rawData.x);
    accel->y = ADXL345_RawToG(hdev, rawData.y);
    accel->z = ADXL345_RawToG(hdev, rawData.z);

    return ADXL_OK;
}

ADXL_StatusTypeDef ADXL345_ReadAverageAcceleration(ADXL345_HandleTypeDef *hdev,
                                                   ADXL345_Acceleration_t *accel,
                                                   uint8_t numSamples)
{
    ADXL_StatusTypeDef status = ADXL_OK;
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    ADXL345_RawData_t rawData;

    if (hdev == NULL || accel == NULL || numSamples == 0) {
        return ADXL_INVALID_PARAM;
    }

    for (uint8_t i = 0; i < numSamples; i++) {
        status = ADXL345_ReadRaw(hdev, &rawData);
        if (status != ADXL_OK) {
            return status;
        }

        sumX += rawData.x;
        sumY += rawData.y;
        sumZ += rawData.z;

        HAL_Delay(5);
    }

    accel->x = ADXL345_RawToG(hdev, sumX / numSamples);
    accel->y = ADXL345_RawToG(hdev, sumY / numSamples);
    accel->z = ADXL345_RawToG(hdev, sumZ / numSamples);

    return ADXL_OK;
}

ADXL_StatusTypeDef ADXL345_AutoCalibrate(ADXL345_HandleTypeDef *hdev,
                                         uint8_t numSamples)
{
    ADXL_StatusTypeDef status = ADXL_OK;
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    ADXL345_RawData_t rawData;

    if (hdev == NULL || numSamples == 0) {
        return ADXL_INVALID_PARAM;
    }

    for (uint8_t i = 0; i < numSamples; i++) {
        status = ADXL345_ReadRaw(hdev, &rawData);
        if (status != ADXL_OK) {
            return status;
        }

        sumX += rawData.x;
        sumY += rawData.y;
        sumZ += rawData.z;

        HAL_Delay(5);
    }

    int16_t expectedZ = 256 >> (hdev->config.range & 0x03);

    int8_t offsetX = (int8_t)((-sumX / numSamples) / 4);
    int8_t offsetY = (int8_t)((-sumY / numSamples) / 4);
    int8_t offsetZ = (int8_t)((expectedZ - (sumZ / numSamples)) / 4);

    status = ADXL345_SetCalibrationOffsets(hdev, offsetX, offsetY, offsetZ);

    return status;
}

ADXL_StatusTypeDef ADXL345_SetCalibrationOffsets(ADXL345_HandleTypeDef *hdev,
                                                int8_t offsetX,
                                                int8_t offsetY,
                                                int8_t offsetZ)
{
    ADXL_StatusTypeDef status = ADXL_OK;

    if (hdev == NULL) {
        return ADXL_INVALID_PARAM;
    }

    status = ADXL345_WriteRegister(hdev, ADXL345_REG_OFSX, (uint8_t)offsetX);
    if (status != ADXL_OK) {
        return status;
    }

    status = ADXL345_WriteRegister(hdev, ADXL345_REG_OFSY, (uint8_t)offsetY);
    if (status != ADXL_OK) {
        return status;
    }

    status = ADXL345_WriteRegister(hdev, ADXL345_REG_OFSZ, (uint8_t)offsetZ);
    if (status != ADXL_OK) {
        return status;
    }

    hdev->offset[0] = offsetX;
    hdev->offset[1] = offsetY;
    hdev->offset[2] = offsetZ;

    return ADXL_OK;
}

ADXL_StatusTypeDef ADXL345_ReadID(ADXL345_HandleTypeDef *hdev, uint8_t *deviceID)
{
    ADXL_StatusTypeDef status = ADXL_OK;

    if (hdev == NULL || deviceID == NULL) {
        return ADXL_INVALID_PARAM;
    }

    for (uint8_t attempt = 0; attempt < ADXL345_ID_READ_RETRIES; attempt++) {
        status = ADXL345_ReadRegister(hdev, ADXL345_REG_DEVID, deviceID, 1);

        if (status == ADXL_OK && *deviceID == ADXL345_DEVICE_ID) {
            return ADXL_OK;
        }

        HAL_Delay(ADXL345_ID_READ_DELAY);
    }

    return (status == ADXL_OK) ? ADXL_DEVICE_NOT_FOUND : status;
}

ADXL_StatusTypeDef ADXL345_CalculateAngles(ADXL345_HandleTypeDef *hdev,
                                          ADXL345_Angles_t *angles)
{
    ADXL_StatusTypeDef status = ADXL_OK;
    ADXL345_Acceleration_t accel;

    if (hdev == NULL || angles == NULL) {
        return ADXL_INVALID_PARAM;
    }

    status = ADXL345_ReadAcceleration(hdev, &accel);
    if (status != ADXL_OK) {
        return status;
    }

    return ADXL345_CalculateAnglesFromAccel(&accel, angles);
}

ADXL_StatusTypeDef ADXL345_CalculateAnglesFromAccel(ADXL345_Acceleration_t *accel,
                                                   ADXL345_Angles_t *angles)
{
    if (accel == NULL || angles == NULL) {
        return ADXL_INVALID_PARAM;
    }

    angles->roll = atan2f(accel->y, accel->z) * 180.0f / M_PI;

    float pitch_denom = sqrtf(accel->x * accel->x + accel->z * accel->z);
    if (fabsf(pitch_denom) < 1e-6f) {
        pitch_denom = 1e-6f;
    }
    angles->pitch = atan2f(-accel->x, pitch_denom) * 180.0f / M_PI;

    return ADXL_OK;
}

float ADXL345_RawToG(ADXL345_HandleTypeDef *hdev, int16_t rawValue)
{
    if (hdev == NULL) {
        return 0.0f;
    }

    return rawValue * hdev->scaleFactor;
}

int16_t ADXL345_GToRaw(ADXL345_HandleTypeDef *hdev, float gValue)
{
    if (hdev == NULL) {
        return 0;
    }

    return (int16_t)(gValue / hdev->scaleFactor);
}

static ADXL_StatusTypeDef ADXL345_ReadRegister(ADXL345_HandleTypeDef *hdev,
                                              uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t txBuffer[10] = {0};
    uint8_t rxBuffer[10] = {0};

    if (length == 1) {
        txBuffer[0] = (reg & 0x3F) | ADXL345_SPI_READ_FLAG;
    } else {
        txBuffer[0] = (reg & 0x3F) | ADXL345_SPI_READ_FLAG | ADXL345_SPI_MULTI_FLAG;
    }

    memset(rxBuffer, 0xFF, sizeof(rxBuffer));

    CS_LOW(hdev);
    __NOP(); __NOP(); __NOP(); __NOP();

    HAL_StatusTypeDef spiStatus = HAL_SPI_TransmitReceive(hdev->hspi,
                                                          txBuffer,
                                                          rxBuffer,
                                                          length + 1,
                                                          ADXL345_SPI_TIMEOUT);

    CS_HIGH(hdev);

    if (spiStatus != HAL_OK) {
        return ADXL_ERROR;
    }

    memcpy(data, rxBuffer + 1, length);

    return ADXL_OK;
}

static ADXL_StatusTypeDef ADXL345_WriteRegister(ADXL345_HandleTypeDef *hdev,
                                               uint8_t reg, uint8_t value)
{
    uint8_t txBuffer[2] = {0};

    txBuffer[0] = reg & 0x3F;
    txBuffer[1] = value;

    CS_LOW(hdev);
    __NOP(); __NOP(); __NOP(); __NOP();

    HAL_StatusTypeDef spiStatus = HAL_SPI_Transmit(hdev->hspi,
                                                   txBuffer,
                                                   2,
                                                   ADXL345_SPI_TIMEOUT);

    CS_HIGH(hdev);

    if (spiStatus != HAL_OK) {
        return ADXL_ERROR;
    }

    return ADXL_OK;
}

static void ADXL345_CalculateScaleFactor(ADXL345_HandleTypeDef *hdev)
{
    if (hdev == NULL) {
        return;
    }

    uint8_t rangeIndex = hdev->config.range & ADXL345_FORMAT_RANGE_MASK;

    if (rangeIndex >= sizeof(ADXL345_ScaleFactors)/sizeof(ADXL345_ScaleFactors[0])) {
        rangeIndex = 0;
    }

    hdev->scaleFactor = ADXL345_ScaleFactors[rangeIndex];
}

ADXL_StatusTypeDef ADXL345_EnableVelocityTracking(ADXL345_HandleTypeDef *hdev,
                                                   uint8_t enable)
{
    if (hdev == NULL) {
        return ADXL_INVALID_PARAM;
    }

    if (!hdev->isInitialized) {
        return ADXL_NOT_INITIALIZED;
    }

    hdev->velocityEnabled = enable;

    if (enable) {
        ADXL345_ResetVelocity(hdev);
    }

    return ADXL_OK;
}

ADXL_StatusTypeDef ADXL345_ConfigZUPT(ADXL345_HandleTypeDef *hdev,
                                     const ADXL345_ZUPT_Config_t *config)
{
    if (hdev == NULL || config == NULL) {
        return ADXL_INVALID_PARAM;
    }

    if (config->windowSize > 100 || config->windowSize < 5) {
        return ADXL_INVALID_PARAM;
    }

    memcpy(&hdev->zuptConfig, config, sizeof(ADXL345_ZUPT_Config_t));

    return ADXL_OK;
}

ADXL_StatusTypeDef ADXL345_ResetVelocity(ADXL345_HandleTypeDef *hdev)
{
    if (hdev == NULL) {
        return ADXL_INVALID_PARAM;
    }

    memset(&hdev->velState, 0, sizeof(ADXL345_VelocityState_t));
    hdev->velState.lastTimestamp = HAL_GetTick();

    return ADXL_OK;
}

ADXL_StatusTypeDef ADXL345_GetVelocity(ADXL345_HandleTypeDef *hdev,
                                       ADXL345_Velocity_t *velocity)
{
    if (hdev == NULL || velocity == NULL) {
        return ADXL_INVALID_PARAM;
    }

    if (!hdev->velocityEnabled) {
        return ADXL_ERROR;
    }

    memcpy(velocity, &hdev->velState.velocity, sizeof(ADXL345_Velocity_t));

    return ADXL_OK;
}

ADXL_StatusTypeDef ADXL345_DetectZeroVelocity(ADXL345_HandleTypeDef *hdev,
                                              uint8_t *isStatic)
{
    if (hdev == NULL || isStatic == NULL) {
        return ADXL_INVALID_PARAM;
    }

    ADXL345_VelocityState_t *vs = &hdev->velState;
    ADXL345_ZUPT_Config_t   *cfg = &hdev->zuptConfig;

    *isStatic = 0;

    if (!vs->bufferFilled) {
        vs->staticCandidate = 0;
        vs->isStatic = 0;
        return ADXL_OK;
    }

    /* ---------- 1. Mean acceleration ---------- */
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    for (uint8_t i = 0; i < cfg->windowSize; i++) {
        sumX += vs->accelBuffer[i].x;
        sumY += vs->accelBuffer[i].y;
        sumZ += vs->accelBuffer[i].z;
    }

    float avgX = sumX / cfg->windowSize;
    float avgY = sumY / cfg->windowSize;
    float avgZ = sumZ / cfg->windowSize;

    /* ---------- 2. Acceleration variance ---------- */
    float variance = 0.0f;
    for (uint8_t i = 0; i < cfg->windowSize; i++) {
        float dx = vs->accelBuffer[i].x - avgX;
        float dy = vs->accelBuffer[i].y - avgY;
        float dz = vs->accelBuffer[i].z - avgZ;
        variance += (dx * dx + dy * dy + dz * dz);
    }
    variance /= cfg->windowSize;

    if (variance > cfg->varianceThreshold) {
        vs->staticCandidate = 0;
        vs->isStatic = 0;
        return ADXL_OK;
    }

    /* ---------- 3. Mean magnitude ---------- */
    float sumMag = 0.0f;
    for (uint8_t i = 0; i < cfg->windowSize; i++) {
        float x = vs->accelBuffer[i].x;
        float y = vs->accelBuffer[i].y;
        float z = vs->accelBuffer[i].z;
        sumMag += sqrtf(x*x + y*y + z*z);
    }
    float avgMag = sumMag / cfg->windowSize;

    /* ---------- 4. Magnitude variance ---------- */
    float magVariance = 0.0f;
    for (uint8_t i = 0; i < cfg->windowSize; i++) {
        float x = vs->accelBuffer[i].x;
        float y = vs->accelBuffer[i].y;
        float z = vs->accelBuffer[i].z;
        float mag = sqrtf(x*x + y*y + z*z);
        float diff = mag - avgMag;
        magVariance += diff * diff;
    }
    magVariance /= cfg->windowSize;

    if (magVariance > cfg->magnitudeVarianceThreshold) {
        vs->staticCandidate = 0;
        vs->isStatic = 0;
        return ADXL_OK;
    }
		
		/* ---------- 4.5 Linear acceleration RMS veto ---------- */
		/* Only use linear RMS to reject motion, not to break a static lock */
		if (!vs->isStatic) {
				float linEnergy = 0.0f;
				for (uint8_t i = 0; i < cfg->windowSize; i++) {
						float lx = vs->linearBuffer[i].x;
						float ly = vs->linearBuffer[i].y;
						float lz = vs->linearBuffer[i].z;
						linEnergy += (lx*lx + ly*ly + lz*lz);
				}

				float linearRms = sqrtf(linEnergy / cfg->windowSize);

				if (linearRms > cfg->linearRmsThreshold) {
						vs->staticCandidate = 0;
						vs->isStatic = 0;
						return ADXL_OK;
				}
		}

    /* ---------- 5. Duration check ---------- */
    uint32_t now = HAL_GetTick();

    if (!vs->staticCandidate) {
        vs->staticCandidate = 1;
        vs->staticStartTime = now;
        vs->isStatic = 0;
    } else {
        if ((now - vs->staticStartTime) >= cfg->minStaticDuration) {
            vs->isStatic = 1;
            *isStatic = 1;
        }
    }

    return ADXL_OK;
}



ADXL_StatusTypeDef ADXL345_RemoveGravity(ADXL345_HandleTypeDef *hdev,
                                         ADXL345_Acceleration_t *accel,
                                         ADXL345_Angles_t *angles) 
{
    if (hdev == NULL || accel == NULL) {
        return ADXL_INVALID_PARAM;
    }

    float mag = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);

    uint8_t isStatic = hdev->velState.isStatic;

    float currentAlpha = ADXL_ALPHA_DYNAMIC; 

    if (isStatic && (mag > 0.9f && mag < 1.1f)) {
        currentAlpha = ADXL_ALPHA_STATIC; 
    }

    hdev->gravity[0] = currentAlpha * hdev->gravity[0] + (1.0f - currentAlpha) * accel->x;
    hdev->gravity[1] = currentAlpha * hdev->gravity[1] + (1.0f - currentAlpha) * accel->y;
    hdev->gravity[2] = currentAlpha * hdev->gravity[2] + (1.0f - currentAlpha) * accel->z;

    accel->x = accel->x - hdev->gravity[0];
    accel->y = accel->y - hdev->gravity[1];
    accel->z = accel->z - hdev->gravity[2];

    return ADXL_OK;
}



ADXL_StatusTypeDef ADXL345_UpdateVelocity(ADXL345_HandleTypeDef *hdev,
                                          ADXL345_Acceleration_t *rawAccel,
                                          ADXL345_Acceleration_t *linearAccel,
                                          uint32_t timestamp)
{
    if (!hdev || !rawAccel || !linearAccel)
        return ADXL_INVALID_PARAM;
    if (!hdev->velocityEnabled)
        return ADXL_ERROR;

    ADXL345_VelocityState_t *vs = &hdev->velState;
    float dt = (timestamp - vs->lastTimestamp) / 1000.0f;

    if (dt <= 0.0f || dt > 0.1f) {
        vs->lastTimestamp = timestamp;
        return ADXL_ERROR;
    }

    vs->accelBuffer[vs->bufferIndex] = *rawAccel;
		vs->linearBuffer[vs->bufferIndex] = *linearAccel;

    vs->bufferIndex++;
    if (vs->bufferIndex >= hdev->zuptConfig.windowSize) {
        vs->bufferIndex = 0;
        vs->bufferFilled = 1;
    }

    static uint32_t zupt_counter = 0;
    zupt_counter++;
    
    uint8_t isStatic = vs->isStatic; 
    
    if (zupt_counter >= 5) {  
        zupt_counter = 0;
        ADXL345_DetectZeroVelocity(hdev, &isStatic);
        vs->isStatic = isStatic;
    }

    if (isStatic) {
        hdev->accelBias[0] += linearAccel->x * ADXL_BIAS_LEARNING_RATE;
        hdev->accelBias[1] += linearAccel->y * ADXL_BIAS_LEARNING_RATE;
        hdev->accelBias[2] += linearAccel->z * ADXL_BIAS_LEARNING_RATE;

        vs->velocity.x = 0.0f;
        vs->velocity.y = 0.0f;
        vs->velocity.z = 0.0f;

        vs->prevLinearAccel.x = 0.0f;
        vs->prevLinearAccel.y = 0.0f;
        vs->prevLinearAccel.z = 0.0f;

    } else {
        const float G_TO_MS2 = 9.81f;

        float ax = linearAccel->x - hdev->accelBias[0];
        float ay = linearAccel->y - hdev->accelBias[1];
        float az = linearAccel->z - hdev->accelBias[2];

        if (fabsf(ax) < ADXL_MOTION_DEADZONE) ax = 0.0f;
        if (fabsf(ay) < ADXL_MOTION_DEADZONE) ay = 0.0f;
        if (fabsf(az) < ADXL_MOTION_DEADZONE) az = 0.0f;

        vs->velocity.x += (vs->prevLinearAccel.x + ax) * 0.5f * dt * G_TO_MS2;
        vs->velocity.y += (vs->prevLinearAccel.y + ay) * 0.5f * dt * G_TO_MS2;
        vs->velocity.z += (vs->prevLinearAccel.z + az) * 0.5f * dt * G_TO_MS2;

        vs->velocity.x *= ADXL_VELOCITY_DAMPING;
        vs->velocity.y *= ADXL_VELOCITY_DAMPING;
        vs->velocity.z *= ADXL_VELOCITY_DAMPING;

        if (fabsf(vs->velocity.x) < 0.001f) vs->velocity.x = 0.0f;
        if (fabsf(vs->velocity.y) < 0.001f) vs->velocity.y = 0.0f;
        if (fabsf(vs->velocity.z) < 0.0001f) vs->velocity.z = 0.0f;

        vs->prevLinearAccel.x = ax;
        vs->prevLinearAccel.y = ay;
        vs->prevLinearAccel.z = az;
    }

    vs->lastTimestamp = timestamp;
    return ADXL_OK;
}

ADXL_StatusTypeDef ADXL345_SetSampleInterval(ADXL345_HandleTypeDef *hdev,
                                            uint32_t interval_ms)
{
    if (hdev == NULL) {
        return ADXL_INVALID_PARAM;
    }

    hdev->sampleInterval = interval_ms;
    return ADXL_OK;
}


ADXL_StatusTypeDef ADXL345_TimedSample(ADXL345_HandleTypeDef *hdev)
{
    if (!hdev) return ADXL_INVALID_PARAM;

    uint32_t now = HAL_GetTick();

    if (now - hdev->lastSampleTime < hdev->sampleInterval) {
        return ADXL_ERROR;
    }
    hdev->lastSampleTime = now;

    ADXL345_Acceleration_t rawAccel;
		
    if (ADXL345_ReadAcceleration(hdev, &rawAccel) != ADXL_OK) {
        return ADXL_ERROR;
    }
		hdev->lastRawAccel = rawAccel;  

    ADXL345_Angles_t angles;
    ADXL345_CalculateAnglesFromAccel(&rawAccel, &angles);

    ADXL345_Acceleration_t linearAccel = rawAccel;

    ADXL345_RemoveGravity(hdev, &linearAccel, &angles);

    ADXL345_UpdateVelocity(hdev, &rawAccel, &linearAccel, now);

    hdev->lastAccel = linearAccel;
    hdev->dataReadyFlag = 1;

    return ADXL_OK;
}



uint8_t ADXL345_IsDataReady(ADXL345_HandleTypeDef *hdev)
{
    if (hdev == NULL) {
        return 0;
    }

    uint8_t ready = hdev->dataReadyFlag;
    hdev->dataReadyFlag = 0;
    return ready;
}

ADXL_StatusTypeDef ADXL345_GetLastAcceleration(ADXL345_HandleTypeDef *hdev,
                                               ADXL345_Acceleration_t *accel)
{
    if (hdev == NULL || accel == NULL) {
        return ADXL_INVALID_PARAM;
    }

    if (!hdev->isInitialized) {
        return ADXL_NOT_INITIALIZED;
    }

    *accel = hdev->lastAccel;

    return ADXL_OK;
}

void ADXL345_PrintZUPTStatus(ADXL345_HandleTypeDef *hdev)
{
    if (!hdev->velState.bufferFilled) {
        printf("ZUPT: Buffer not filled yet\n");
        return;
    }

    ADXL345_VelocityState_t *vs  = &hdev->velState;
    ADXL345_ZUPT_Config_t   *cfg = &hdev->zuptConfig;

    /* ---------- Mean acceleration ---------- */
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    for (uint8_t i = 0; i < cfg->windowSize; i++) {
        sumX += vs->accelBuffer[i].x;
        sumY += vs->accelBuffer[i].y;
        sumZ += vs->accelBuffer[i].z;
    }

    float avgX = sumX / cfg->windowSize;
    float avgY = sumY / cfg->windowSize;
    float avgZ = sumZ / cfg->windowSize;

    /* ---------- Acceleration variance ---------- */
    float variance = 0.0f;
    for (uint8_t i = 0; i < cfg->windowSize; i++) {
        float dx = vs->accelBuffer[i].x - avgX;
        float dy = vs->accelBuffer[i].y - avgY;
        float dz = vs->accelBuffer[i].z - avgZ;
        variance += (dx * dx + dy * dy + dz * dz);
    }
    variance /= cfg->windowSize;

    /* ---------- Mean magnitude ---------- */
    float sumMag = 0.0f;
    for (uint8_t i = 0; i < cfg->windowSize; i++) {
        float x = vs->accelBuffer[i].x;
        float y = vs->accelBuffer[i].y;
        float z = vs->accelBuffer[i].z;
        sumMag += sqrtf(x*x + y*y + z*z);
    }
    float avgMag = sumMag / cfg->windowSize;

    /* ---------- Magnitude variance ---------- */
    float magVariance = 0.0f;
    for (uint8_t i = 0; i < cfg->windowSize; i++) {
        float x = vs->accelBuffer[i].x;
        float y = vs->accelBuffer[i].y;
        float z = vs->accelBuffer[i].z;
        float mag = sqrtf(x*x + y*y + z*z);
        float diff = mag - avgMag;
        magVariance += diff * diff;
    }
    magVariance /= cfg->windowSize;

    printf("ZUPT Status:\n");
    printf("  Accel Variance: %.6f (thresh: %.6f) %s\n",
           variance, cfg->varianceThreshold,
           (variance < cfg->varianceThreshold) ? "OK" : "HIGH");

    printf("  Mag Variance: %.6f (thresh: %.6f) %s\n",
           magVariance, cfg->magnitudeVarianceThreshold,
           (magVariance < cfg->magnitudeVarianceThreshold) ? "OK" : "HIGH");

    printf("  Avg Magnitude: %.4f g\n", avgMag);
    printf("  Static: %s\n", vs->isStatic ? "YES" : "NO");
}
