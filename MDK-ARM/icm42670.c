#include "icm42670.h"
#include <string.h>
#include <stdio.h>

static const char* accel_fs_str[] = {"16g", "8g", "4g", "2g"};
static const char* gyro_fs_str[]  = {"2000", "1000", "500", "250"};
static const float gyro_odr_hz[] = {
    0, 0, 0, 0, 0,           // 0-4: Reserved
    1600, 800, 400, 200,     // 5-8
    100, 50, 25, 12.5f,      // 9-12
    0, 0, 0                  // 13-15: Reserved for gyro
};
static const float accel_odr_hz[] = {
    0, 0, 0, 0, 0,           // 0-4: Reserved
    1600, 800, 400, 200,     // 5-8
    100, 50, 25, 12.5f,      // 9-12
    6.25f, 3.125f, 1.5625f   // 13-15: LP mode only
};
static const float accel_sensitivity_table[] = {2048.0f, 4096.0f, 8192.0f, 16384.0f};  // LSB/g
static const float gyro_sensitivity_table[] = {16.384f, 32.768f, 65.536f, 131.072f};   // LSB/dps

static ICM42670_DMA_Context_t dma_ctx;

static void ICM42670_CS_Low(ICM42670_Handle_t *h)
{
    HAL_GPIO_WritePin(h->cs_port, h->cs_pin, GPIO_PIN_RESET);
    for (volatile int i = 0; i < 10; i++);
}

static void ICM42670_CS_High(ICM42670_Handle_t *h)
{
    for (volatile int i = 0; i < 10; i++);
    HAL_GPIO_WritePin(h->cs_port, h->cs_pin, GPIO_PIN_SET);
}

static void ICM42670_UpdateSensitivity(ICM42670_Handle_t *h)
{
    h->accel_sensitivity = accel_sensitivity_table[h->accel_fs & 0x03];
    h->gyro_sensitivity  = gyro_sensitivity_table[h->gyro_fs & 0x03];
}

static bool ICM42670_WaitMCLK(ICM42670_Handle_t *h, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms) {
        if (ICM42670_ReadReg(h, REG_MCLK_RDY) & 0x08) {
            return true;
        }
        HAL_Delay(1);
    }
    return false;
}

uint8_t ICM42670_ReadReg(ICM42670_Handle_t *h, uint8_t reg)
{
    uint8_t tx[2] = {reg | 0x80, 0x00};
    uint8_t rx[2] = {0};

    ICM42670_CS_Low(h);
    HAL_SPI_TransmitReceive(h->hspi, tx, rx, 2, HAL_MAX_DELAY);
    ICM42670_CS_High(h);

    return rx[1];
}

void ICM42670_WriteReg(ICM42670_Handle_t *h, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg & 0x7F, value};

    ICM42670_CS_Low(h);
    HAL_SPI_Transmit(h->hspi, tx, 2, HAL_MAX_DELAY);
    ICM42670_CS_High(h);
    
    if (reg == REG_SIGNAL_PATH_RESET || reg == REG_PWR_MGMT0) {
        HAL_Delay(1);
    }
}

void ICM42670_GetDefaultConfig(ICM42670_Config_t *config)
{
    config->accel_fs   = ICM42670_ACCEL_FS_4G;
    config->gyro_fs    = ICM42670_GYRO_FS_500DPS;
    config->accel_odr  = ICM42670_ACCEL_ODR_100HZ;
    config->gyro_odr   = ICM42670_GYRO_ODR_100HZ;
    config->accel_mode = ICM42670_MODE_LOW_NOISE;
    config->gyro_mode  = ICM42670_MODE_LOW_NOISE;
}

ICM42670_Status_t ICM42670_Init(ICM42670_Handle_t *h, const ICM42670_Config_t *config)
{
    HAL_Delay(50);
    ICM42670_Config_t default_cfg;
    
    if (config == NULL) {
        ICM42670_GetDefaultConfig(&default_cfg);
        config = &default_cfg;
    }
    
    ICM42670_CS_High(h);
    HAL_Delay(10);

    ICM42670_WriteReg(h, REG_BLK_SEL_W, 0x00);
    ICM42670_WriteReg(h, REG_BLK_SEL_R, 0x00);
    ICM42670_WriteReg(h, REG_PWR_MGMT0, 0x10);
    HAL_Delay(1);

    if (!ICM42670_WaitMCLK(h, 100)) {
        return ICM42670_ERR_MCLK_TIMEOUT;
    }

    if (ICM42670_ReadReg(h, REG_WHO_AM_I) != ICM42670_ID) {
        return ICM42670_ERR_DEVICE_ID;
    }

    ICM42670_WriteReg(h, REG_SIGNAL_PATH_RESET, 0x10);
    HAL_Delay(10);
    
    ICM42670_WriteReg(h, REG_PWR_MGMT0, 0x10);
    HAL_Delay(1);
    if (!ICM42670_WaitMCLK(h, 100)) {
        return ICM42670_ERR_RESET_FAIL;
    }

    uint8_t intf = ICM42670_ReadReg(h, REG_INTF_CONFIG0);
    intf |= (1 << 4);
    ICM42670_WriteReg(h, REG_INTF_CONFIG0, intf);

    uint8_t accel_cfg = ((config->accel_fs & 0x03) << 5) | (config->accel_odr & 0x0F);
    ICM42670_WriteReg(h, REG_ACCEL_CONFIG0, accel_cfg);
    
    uint8_t gyro_cfg = ((config->gyro_fs & 0x03) << 5) | (config->gyro_odr & 0x0F);
    ICM42670_WriteReg(h, REG_GYRO_CONFIG0, gyro_cfg);
    HAL_Delay(1);

    h->accel_fs  = config->accel_fs;
    h->gyro_fs   = config->gyro_fs;
    h->accel_odr = config->accel_odr;
    h->gyro_odr  = config->gyro_odr;
    ICM42670_UpdateSensitivity(h);

    uint8_t pwr = ((config->gyro_mode & 0x03) << 2) | (config->accel_mode & 0x03);
    ICM42670_WriteReg(h, REG_PWR_MGMT0, pwr);
    HAL_Delay(50);
    
    return ICM42670_OK;
}

bool ICM42670_SetAccelConfig(ICM42670_Handle_t *h, ICM42670_AccelFS_t fs, ICM42670_AccelODR_t odr)
{
    uint8_t cfg = ((fs & 0x03) << 5) | (odr & 0x0F);
    ICM42670_WriteReg(h, REG_ACCEL_CONFIG0, cfg);
    
    h->accel_fs  = fs;
    h->accel_odr = odr;
    ICM42670_UpdateSensitivity(h);
    
    return true;
}

bool ICM42670_SetGyroConfig(ICM42670_Handle_t *h, ICM42670_GyroFS_t fs, ICM42670_GyroODR_t odr)
{
    uint8_t cfg = ((fs & 0x03) << 5) | (odr & 0x0F);
    ICM42670_WriteReg(h, REG_GYRO_CONFIG0, cfg);
    
    h->gyro_fs  = fs;
    h->gyro_odr = odr;
    ICM42670_UpdateSensitivity(h);
    
    return true;
}

bool ICM42670_SetPowerMode(ICM42670_Handle_t *h, ICM42670_PowerMode_t accel_mode, ICM42670_PowerMode_t gyro_mode)
{
    uint8_t pwr = ((gyro_mode & 0x03) << 2) | (accel_mode & 0x03);
    ICM42670_WriteReg(h, REG_PWR_MGMT0, pwr);
    HAL_Delay(50);
    return true;
}

void ICM42670_Reset(ICM42670_Handle_t *h)
{
    uint8_t dummy = 0;
    
    ICM42670_WriteReg(h, 0x76, 0x00);
    ICM42670_WriteReg(h, 0x79, 0x00);
    ICM42670_WriteReg(h, 0x4B, 0x02);
    HAL_Delay(10);
    ICM42670_WriteReg(h, 0x11, 0x00);
    
    dummy = ICM42670_ReadReg(h, 0x37);
    (void)dummy;
    
    HAL_Delay(100);
}

void ICM42670_Wakeup(ICM42670_Handle_t *h)
{
    ICM42670_WriteReg(h, REG_PWR_MGMT0, 0x0F);
    HAL_Delay(50);
}

bool ICM42670_CheckDeviceID(ICM42670_Handle_t *h)
{
    uint8_t id = ICM42670_ReadReg(h, REG_WHO_AM_I);
    return (id == ICM42670_ID);
}

void ICM42670_DumpRegisters(ICM42670_Handle_t *h, ICM42670_RegDump_t *dump)
{
    dump->pwr_mgmt0    = ICM42670_ReadReg(h, REG_PWR_MGMT0);
    dump->gyro_config0 = ICM42670_ReadReg(h, REG_GYRO_CONFIG0);
    dump->accel_config0= ICM42670_ReadReg(h, REG_ACCEL_CONFIG0);
    dump->intf_config0 = ICM42670_ReadReg(h, REG_INTF_CONFIG0);
    dump->mclk_rdy     = ICM42670_ReadReg(h, REG_MCLK_RDY);
    dump->who_am_i     = ICM42670_ReadReg(h, REG_WHO_AM_I);
}

void ICM42670_ConvertToPhysical(ICM42670_Handle_t *h, ICM42670_Data_t *raw_data, 
                                float *accel_g, float *gyro_dps)
{
    accel_g[0] = raw_data->accel_x / h->accel_sensitivity;
    accel_g[1] = raw_data->accel_y / h->accel_sensitivity;
    accel_g[2] = raw_data->accel_z / h->accel_sensitivity;
    
    gyro_dps[0] = raw_data->gyro_x / h->gyro_sensitivity;
    gyro_dps[1] = raw_data->gyro_y / h->gyro_sensitivity;
    gyro_dps[2] = raw_data->gyro_z / h->gyro_sensitivity;
}


void ICM42670_DMA_Init(ICM42670_Handle_t *h)
{
    dma_ctx.handle = h;
    dma_ctx.active_buf = 0;
    dma_ctx.data_ready = false;
    dma_ctx.dma_count = 0;
    dma_ctx.last_dma_count = 0;
    dma_ctx.stall_counter = 0;
    
    dma_ctx.config.accel_fs = h->accel_fs;
    dma_ctx.config.gyro_fs = h->gyro_fs;
    dma_ctx.config.accel_odr = h->accel_odr;
    dma_ctx.config.gyro_odr = h->gyro_odr;
    dma_ctx.config.accel_mode = ICM42670_MODE_LOW_NOISE;
    dma_ctx.config.gyro_mode = ICM42670_MODE_LOW_NOISE;
    
    memset(dma_ctx.tx_buf, 0x00, sizeof(dma_ctx.tx_buf));
    dma_ctx.tx_buf[0] = REG_ACCEL_DATA_X1 | 0x80;
    
    ICM42670_WriteReg(h, REG_INT_CONFIG, 0x03);
    ICM42670_WriteReg(h, REG_INT_SOURCE0, 0x08);
}

void ICM42670_StartDMACapture(void)
{
    HAL_GPIO_WritePin(dma_ctx.handle->cs_port, 
                      dma_ctx.handle->cs_pin, GPIO_PIN_RESET);
    
    HAL_SPI_TransmitReceive_DMA(dma_ctx.handle->hspi,
                                dma_ctx.tx_buf,
                                dma_ctx.rx_buf[dma_ctx.active_buf],
                                14);
}

void ICM42670_DMAComplete(void)
{
    HAL_GPIO_WritePin(dma_ctx.handle->cs_port,
                      dma_ctx.handle->cs_pin, GPIO_PIN_SET);
    
    dma_ctx.active_buf ^= 1;
    dma_ctx.data_ready = true;
    dma_ctx.dma_count++;
}

bool ICM42670_DMA_GetData(ICM42670_Data_t *data)
{
    if (!dma_ctx.data_ready) {
        return false;
    }
    // Read from completed buffer
    uint8_t *rx = dma_ctx.rx_buf[dma_ctx.active_buf ^ 1];
    
    data->accel_x = (int16_t)((rx[1] << 8) | rx[2]);
    data->accel_y = (int16_t)((rx[3] << 8) | rx[4]);
    data->accel_z = (int16_t)((rx[5] << 8) | rx[6]);
    data->gyro_x  = (int16_t)((rx[7] << 8) | rx[8]);
    data->gyro_y  = (int16_t)((rx[9] << 8) | rx[10]);
    data->gyro_z  = (int16_t)((rx[11] << 8) | rx[12]);
    
    dma_ctx.data_ready = false;
    return true;
}

static void ICM42670_Recovery(void)
{
    __disable_irq();
    
    HAL_SPI_DMAStop(dma_ctx.handle->hspi);
    HAL_SPI_Abort(dma_ctx.handle->hspi);
    
    HAL_GPIO_WritePin(dma_ctx.handle->cs_port, 
                      dma_ctx.handle->cs_pin, GPIO_PIN_SET);
    
    dma_ctx.handle->hspi->State = HAL_SPI_STATE_READY;
    dma_ctx.handle->hspi->hdmarx->State = HAL_DMA_STATE_READY;
    dma_ctx.handle->hspi->hdmatx->State = HAL_DMA_STATE_READY;
    
    __HAL_SPI_CLEAR_OVRFLAG(dma_ctx.handle->hspi);
    __HAL_SPI_CLEAR_MODFFLAG(dma_ctx.handle->hspi);
    
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
    
    __enable_irq();
    
    HAL_Delay(50);
    
    ICM42670_WriteReg(dma_ctx.handle, REG_SIGNAL_PATH_RESET, 0x10);
    HAL_Delay(10);
    
    ICM42670_Init(dma_ctx.handle, &dma_ctx.config);
    
    dma_ctx.data_ready = false;
    dma_ctx.active_buf = 0;
    
    memset(dma_ctx.tx_buf, 0x00, sizeof(dma_ctx.tx_buf));
    dma_ctx.tx_buf[0] = REG_ACCEL_DATA_X1 | 0x80;
    
    ICM42670_WriteReg(dma_ctx.handle, REG_INT_CONFIG, 0x03);
    ICM42670_WriteReg(dma_ctx.handle, REG_INT_SOURCE0, 0x08);
    
    HAL_Delay(10);
}

bool ICM42670_HealthCheck(void)
{
    if (dma_ctx.dma_count == dma_ctx.last_dma_count) {
        dma_ctx.stall_counter++;
        
        if (dma_ctx.stall_counter >= 3) {
            ICM42670_Recovery();
            dma_ctx.stall_counter = 0;
            dma_ctx.recovery_count++;
            return true;
        }
    } else {
        dma_ctx.stall_counter = 0;
    }
    
    dma_ctx.last_dma_count = dma_ctx.dma_count;
    return false;
}

uint32_t ICM42670_GetDMACount(void)
{
    return dma_ctx.dma_count;
}

uint32_t ICM42670_GetRecoveryCount(void)
{
    return dma_ctx.recovery_count;
}