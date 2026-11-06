#ifndef ICM42670_H
#define ICM42670_H

#include "main.h"
#include <stdbool.h>

#define ICM42670_ID            0x67

/* Register Map (User Bank 0) */
#define REG_MCLK_RDY           0x00
#define REG_DEVICE_CONFIG      0x01
#define REG_SIGNAL_PATH_RESET  0x02
#define REG_DRIVE_CONFIG1      0x03
#define REG_DRIVE_CONFIG2      0x04
#define REG_DRIVE_CONFIG3      0x05
#define REG_INT_CONFIG         0x06
#define REG_TEMP_DATA1         0x09
#define REG_TEMP_DATA0         0x0A
#define REG_ACCEL_DATA_X1      0x0B
#define REG_ACCEL_DATA_X0      0x0C
#define REG_ACCEL_DATA_Y1      0x0D
#define REG_ACCEL_DATA_Y0      0x0E
#define REG_ACCEL_DATA_Z1      0x0F
#define REG_ACCEL_DATA_Z0      0x10
#define REG_GYRO_DATA_X1       0x11
#define REG_GYRO_DATA_X0       0x12
#define REG_GYRO_DATA_Y1       0x13
#define REG_GYRO_DATA_Y0       0x14
#define REG_GYRO_DATA_Z1       0x15
#define REG_GYRO_DATA_Z0       0x16
#define REG_TMST_FSYNCH        0x17
#define REG_TMST_FSYNCL        0x18
#define REG_PWR_MGMT0          0x1F
#define REG_GYRO_CONFIG0       0x20
#define REG_ACCEL_CONFIG0      0x21
#define REG_TEMP_CONFIG0       0x22
#define REG_GYRO_CONFIG1       0x23
#define REG_ACCEL_CONFIG1      0x24
#define REG_FIFO_CONFIG1       0x28
#define REG_FIFO_CONFIG2       0x29
#define REG_FIFO_CONFIG3       0x2A
#define REG_INT_SOURCE0        0x2B
#define REG_INT_SOURCE1        0x2C
#define REG_INTF_CONFIG0       0x35
#define REG_INTF_CONFIG1       0x36
#define REG_INT_STATUS_DRDY    0x39
#define REG_INT_STATUS         0x3A
#define REG_FIFO_COUNTH        0x3D
#define REG_FIFO_COUNTL        0x3E
#define REG_FIFO_DATA          0x3F
#define REG_WHO_AM_I           0x75
#define REG_BLK_SEL_W          0x79
#define REG_MADDR_W            0x7A
#define REG_M_W                0x7B
#define REG_BLK_SEL_R          0x7C
#define REG_MADDR_R            0x7D
#define REG_M_R                0x7E

#define REG_INT_CONFIG      0x06
#define REG_INT_SOURCE0     0x2B

/* Accel Full Scale [6:5] */
typedef enum {
    ICM42670_ACCEL_FS_16G = 0,
    ICM42670_ACCEL_FS_8G  = 1,
    ICM42670_ACCEL_FS_4G  = 2,
    ICM42670_ACCEL_FS_2G  = 3
} ICM42670_AccelFS_t;

/* Gyro Full Scale [6:5] */
typedef enum {
    ICM42670_GYRO_FS_2000DPS = 0,
    ICM42670_GYRO_FS_1000DPS = 1,
    ICM42670_GYRO_FS_500DPS  = 2,
    ICM42670_GYRO_FS_250DPS  = 3
} ICM42670_GyroFS_t;

/* Gyro ODR [3:0], min 12.5Hz */
typedef enum {
    ICM42670_GYRO_ODR_1600HZ  = 5,
    ICM42670_GYRO_ODR_800HZ   = 6,
    ICM42670_GYRO_ODR_400HZ   = 7,
    ICM42670_GYRO_ODR_200HZ   = 8,
    ICM42670_GYRO_ODR_100HZ   = 9,
    ICM42670_GYRO_ODR_50HZ    = 10,
    ICM42670_GYRO_ODR_25HZ    = 11,
    ICM42670_GYRO_ODR_12_5HZ  = 12
} ICM42670_GyroODR_t;

/* Accel ODR [3:0], 1.6k/800 LN only, <=6.25 LP only */
typedef enum {
    ICM42670_ACCEL_ODR_1600HZ    = 5,
    ICM42670_ACCEL_ODR_800HZ     = 6,
    ICM42670_ACCEL_ODR_400HZ     = 7,
    ICM42670_ACCEL_ODR_200HZ     = 8,
    ICM42670_ACCEL_ODR_100HZ     = 9,
    ICM42670_ACCEL_ODR_50HZ      = 10,
    ICM42670_ACCEL_ODR_25HZ      = 11,
    ICM42670_ACCEL_ODR_12_5HZ    = 12,
    ICM42670_ACCEL_ODR_6_25HZ    = 13,
    ICM42670_ACCEL_ODR_3_125HZ   = 14,
    ICM42670_ACCEL_ODR_1_5625HZ  = 15
} ICM42670_AccelODR_t;

/* Power Mode */
typedef enum {
    ICM42670_MODE_OFF       = 0,
    ICM42670_MODE_STANDBY   = 1,
    ICM42670_MODE_LOW_POWER = 2,
    ICM42670_MODE_LOW_NOISE = 3
} ICM42670_PowerMode_t;

/* Status codes */
typedef enum {
    ICM42670_OK = 0,
    ICM42670_ERR_MCLK_TIMEOUT,
    ICM42670_ERR_DEVICE_ID,
    ICM42670_ERR_RESET_FAIL
} ICM42670_Status_t;

/* Raw sensor data */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} ICM42670_Data_t;

/* Init config */
typedef struct {
    ICM42670_AccelFS_t   accel_fs;
    ICM42670_GyroFS_t    gyro_fs;
    ICM42670_AccelODR_t  accel_odr;
    ICM42670_GyroODR_t   gyro_odr;
    ICM42670_PowerMode_t accel_mode;
    ICM42670_PowerMode_t gyro_mode;
} ICM42670_Config_t;

/* Register dump for debugging */
typedef struct {
    uint8_t pwr_mgmt0;
    uint8_t gyro_config0;
    uint8_t accel_config0;
    uint8_t intf_config0;
    uint8_t mclk_rdy;
    uint8_t who_am_i;
} ICM42670_RegDump_t;

/* Device handle */
typedef struct {
    SPI_HandleTypeDef  *hspi;
    GPIO_TypeDef       *cs_port;
    uint16_t            cs_pin;
    
    uint8_t accel_fs;
    uint8_t gyro_fs;
    uint8_t accel_odr;
    uint8_t gyro_odr;
    
    float accel_sensitivity;  // LSB/g
    float gyro_sensitivity;   // LSB/dps
} ICM42670_Handle_t;

typedef struct {
    uint8_t tx_buf[14];
    uint8_t rx_buf[2][14];
    volatile uint8_t active_buf;
    volatile bool data_ready;
    ICM42670_Handle_t *handle;
    ICM42670_Config_t config;
    volatile uint32_t dma_count;
    uint32_t last_dma_count;
    uint8_t stall_counter;
    uint32_t recovery_count;
    uint32_t error_flags;
} ICM42670_DMA_Context_t;

/* API */
void ICM42670_GetDefaultConfig(ICM42670_Config_t *config);
ICM42670_Status_t ICM42670_Init(ICM42670_Handle_t *h, const ICM42670_Config_t *config);
bool ICM42670_SetAccelConfig(ICM42670_Handle_t *h, ICM42670_AccelFS_t fs, ICM42670_AccelODR_t odr);
bool ICM42670_SetGyroConfig(ICM42670_Handle_t *h, ICM42670_GyroFS_t fs, ICM42670_GyroODR_t odr);
bool ICM42670_SetPowerMode(ICM42670_Handle_t *h, ICM42670_PowerMode_t accel_mode, ICM42670_PowerMode_t gyro_mode);
void ICM42670_Reset(ICM42670_Handle_t *h);
uint8_t ICM42670_ReadReg(ICM42670_Handle_t *h, uint8_t reg);
void ICM42670_WriteReg(ICM42670_Handle_t *h, uint8_t reg, uint8_t value);
void ICM42670_Wakeup(ICM42670_Handle_t *h);
bool ICM42670_CheckDeviceID(ICM42670_Handle_t *h);

void ICM42670_ConvertToPhysical(ICM42670_Handle_t *h, ICM42670_Data_t *raw_data, float *accel_g, float *gyro_dps);

void ICM42670_DMA_Init(ICM42670_Handle_t *h);
bool ICM42670_DMA_GetData(ICM42670_Data_t *data);
void ICM42670_StartDMACapture(void);  
void ICM42670_DMAComplete(void);   
bool ICM42670_HealthCheck(void);
uint32_t ICM42670_GetDMACount(void);
uint32_t ICM42670_GetRecoveryCount(void);

#endif