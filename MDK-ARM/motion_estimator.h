#ifndef MOTION_ESTIMATOR_H
#define MOTION_ESTIMATOR_H

#include "icm42670.h"
#include <stdbool.h>

#define ZUPT_WINDOW_SIZE 50
#define ZUPT_ACCEL_VAR_THRESH  0.0005f
#define ZUPT_GYRO_VAR_THRESH   0.5f

typedef enum {
    ME_STATE_CALIBRATING = 0,
    ME_STATE_RUNNING = 2
} ME_State_t;

typedef struct {
    float accel_buf[3][ZUPT_WINDOW_SIZE];
    float gyro_buf[3][ZUPT_WINDOW_SIZE];
    uint16_t index;
    uint16_t count;
} ZUPT_Window_t;

typedef struct {
    ICM42670_Handle_t *imu;
    
    ME_State_t state;
    
    uint16_t calib_samples;
    uint16_t calib_count;
    float gyro_sum[3];
    float accel_sum[3];
    
    float gyro_bias[3];
    float linear_accel_bias[3];
    
    float q[4];
    float alpha;
    
    uint8_t divider;
    uint8_t sample_count;
    float dt;
    
    float linear_accel[3];
    
    ZUPT_Window_t zupt;
    bool is_stationary;
    float zupt_accel_var;
    float zupt_gyro_var;
    
} MotionEstimator_Handle_t;

void ME_Init(MotionEstimator_Handle_t *me, ICM42670_Handle_t *imu, uint8_t divider);
void ME_SetDivider(MotionEstimator_Handle_t *me, uint8_t divider);
void ME_SetAlpha(MotionEstimator_Handle_t *me, float alpha);
void ME_UpdateGyroBias(MotionEstimator_Handle_t *me, float bx, float by, float bz);
void ME_Reset(MotionEstimator_Handle_t *me);
bool ME_Update(MotionEstimator_Handle_t *me, float ax, float ay, float az, float gx, float gy, float gz);
void ME_GetEuler(MotionEstimator_Handle_t *me, float *roll, float *pitch, float *yaw);
bool ME_IsStationary(MotionEstimator_Handle_t *me);
void ME_GetGyroBias(MotionEstimator_Handle_t *me, float *bx, float *by, float *bz);

#endif