#include "velocity_estimator.h"
#include <math.h>
#include <string.h>

//1.75Hz Highpass @ 400Hz, Butterworth
//static const float hp_coeffs[5] = {
//    0.9807518f,   // b0
//   -1.9615036f,   // b1
//    0.9807518f,   // b2
//    1.9611325f,   // -a1
//   -0.9618738f    // -a2
//};
//1.85Hz Highpass @ 400Hz, Butterworth
static const float hp_coeffs[5] = {
    0.9796590f,   // b0
   -1.9593180f,   // b1
    0.9796590f,   // b2
    1.9589100f,   // -a1
   -0.9597030f    // -a2
};
// 2Hz highpass @ 400Hz, Butterworth Q=0.7071
//static const float hp_coeffs[5] = {
//    0.9560440657f,   // b0
//   -1.9120881313f,   // b1
//    0.9560440657f,   // b2
//    1.9111970675f,   // -a1
//   -0.9129791951f    // -a2
//};

#define GRAVITY   9.80665f
#define MM_PER_M  1000.0f

static void init_biquad(arm_biquad_cascade_df2T_instance_f32 *instance, 
                        float *state, 
                        const float *coeffs)
{
    memset(state, 0, 2 * sizeof(float));
    arm_biquad_cascade_df2T_init_f32(instance, 1, coeffs, state);
}

static void reset_biquad_state(arm_biquad_cascade_df2T_instance_f32 *instance)
{
    memset(instance->pState, 0, 2 * sizeof(float));
}

void VE_Init(VelocityEstimator_Handle_t *ve, float sample_freq)
{
    memset(ve, 0, sizeof(VelocityEstimator_Handle_t));
    ve->sample_freq = sample_freq;
    ve->dt = 1.0f / sample_freq;
    
    for (int i = 0; i < 3; i++) {
        init_biquad(&ve->hp_accel[i], ve->hp_accel_state[i], hp_coeffs);
        init_biquad(&ve->hp_vel[i], ve->hp_vel_state[i], hp_coeffs);
    }
}

void VE_Reset(VelocityEstimator_Handle_t *ve)
{
    float freq = ve->sample_freq;
    
    memset(ve->velocity, 0, sizeof(ve->velocity));
    memset(ve->prev_accel, 0, sizeof(ve->prev_accel));
    memset(ve->rms_buffer, 0, sizeof(ve->rms_buffer));
    
    ve->rms_index = 0;
    ve->rms_count = 0;
    ve->rms_sum = 0.0f;
    ve->velocity_rms = 0.0f;
    
    ve->initialized = false;

    ve->sample_freq = freq;
    ve->dt = 1.0f / freq;
    
    for (int i = 0; i < 3; i++) {
        reset_biquad_state(&ve->hp_accel[i]);
        reset_biquad_state(&ve->hp_vel[i]);
    }
}

void VE_Update(VelocityEstimator_Handle_t *ve, float ax, float ay, float az, bool is_stationary)
{
    float accel[3] = {ax, ay, az};
    float filtered_accel[3];
    
    for (int i = 0; i < 3; i++) {
        arm_biquad_cascade_df2T_f32(&ve->hp_accel[i], &accel[i], &filtered_accel[i], 1);
    }
    
    if (!ve->initialized) {
        for (int i = 0; i < 3; i++) {
            ve->prev_accel[i] = filtered_accel[i];
        }
        ve->initialized = true;
        return;
    }
    
    float vel_sq_sum = 0.0f;
    
    if (is_stationary) {
        for (int i = 0; i < 3; i++) {
            ve->velocity[i] = 0.0f;
            ve->prev_accel[i] = filtered_accel[i];
            reset_biquad_state(&ve->hp_vel[i]);
        }
    } else {
        for (int i = 0; i < 3; i++) {
            float delta_v = 0.5f * (filtered_accel[i] + ve->prev_accel[i]) * ve->dt;
            float raw_vel = ve->velocity[i] + delta_v * MM_PER_M;
            arm_biquad_cascade_df2T_f32(&ve->hp_vel[i], &raw_vel, &ve->velocity[i], 1);
            ve->prev_accel[i] = filtered_accel[i];
            vel_sq_sum += ve->velocity[i] * ve->velocity[i];
        }
    }
    
    float old_val = ve->rms_buffer[ve->rms_index];
    ve->rms_buffer[ve->rms_index] = vel_sq_sum;
    
    if (ve->rms_count >= VE_RMS_WINDOW_SIZE) {
        ve->rms_sum -= old_val;
    }
    ve->rms_sum += vel_sq_sum;
    
    ve->rms_index = (ve->rms_index + 1) % VE_RMS_WINDOW_SIZE;
    if (ve->rms_count < VE_RMS_WINDOW_SIZE) {
        ve->rms_count++;
    }
    
    if (ve->rms_sum < 0.0f) {
        ve->rms_sum = 0.0f;
    }
    
    float rms = sqrtf(ve->rms_sum / ve->rms_count);
    if (!isfinite(rms)) {
        rms = 0.0f;
        ve->rms_sum = 0.0f;
        ve->rms_count = 0;
        memset(ve->rms_buffer, 0, sizeof(ve->rms_buffer));
    }
    
    ve->velocity_rms = rms;
}

void VE_GetVelocity(VelocityEstimator_Handle_t *ve, float *vx, float *vy, float *vz)
{
    *vx = ve->velocity[0];
    *vy = ve->velocity[1];
    *vz = ve->velocity[2];
}

float VE_GetRMS(VelocityEstimator_Handle_t *ve)
{
    return ve->velocity_rms;
}