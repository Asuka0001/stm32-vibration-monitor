#ifndef VELOCITY_ESTIMATOR_H
#define VELOCITY_ESTIMATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"

#define VE_RMS_WINDOW_SIZE 400

typedef struct {
    float sample_freq;
    float dt;
    
    arm_biquad_cascade_df2T_instance_f32 hp_accel[3];
    arm_biquad_cascade_df2T_instance_f32 hp_vel[3];
    float hp_accel_state[3][2];
    float hp_vel_state[3][2];
    
    float prev_accel[3];
    float velocity[3];
    
    float rms_buffer[VE_RMS_WINDOW_SIZE];
    float rms_sum;
    uint16_t rms_index;
    uint16_t rms_count;
    float velocity_rms;
    
    bool initialized;
} VelocityEstimator_Handle_t;

void VE_Init(VelocityEstimator_Handle_t *ve, float sample_freq);
void VE_Reset(VelocityEstimator_Handle_t *ve);
void VE_Update(VelocityEstimator_Handle_t *ve, float ax, float ay, float az, bool is_stationary);
void VE_GetVelocity(VelocityEstimator_Handle_t *ve, float *vx, float *vy, float *vz);
float VE_GetRMS(VelocityEstimator_Handle_t *ve);

#endif