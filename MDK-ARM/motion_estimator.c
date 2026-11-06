#include "motion_estimator.h"
#include "arm_math.h"
#include <math.h>
#include <string.h>

#define GRAVITY 9.80665f
#define BASE_FREQ 1600.0f
#define DEG2RAD 0.017453292f

#define GYRO_BIAS_ALPHA   0.001f   
#define ACCEL_BIAS_ALPHA  0.01f


static void normalize_quat(float q[4])
{
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 0.0001f) {
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;
    }
}

static void accel_to_quat(float ax, float ay, float az, float q[4])
{
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.0001f) {
        q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
        return;
    }
    ax /= norm;
    ay /= norm;
    az /= norm;
    
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    
    q[0] = cr * cp;
    q[1] = sr * cp;
    q[2] = cr * sp;
    q[3] = -sr * sp;
}

static void ZUPT_Init(ZUPT_Window_t *z)
{
    memset(z, 0, sizeof(ZUPT_Window_t));
}

static bool ZUPT_Update(ZUPT_Window_t *z, float ax, float ay, float az,
                        float gx, float gy, float gz,
                        float *out_accel_var, float *out_gyro_var)
{
    uint16_t idx = z->index;
    
    z->accel_buf[0][idx] = ax;
    z->accel_buf[1][idx] = ay;
    z->accel_buf[2][idx] = az;
    z->gyro_buf[0][idx] = gx;
    z->gyro_buf[1][idx] = gy;
    z->gyro_buf[2][idx] = gz;
    
    z->index = (idx + 1) % ZUPT_WINDOW_SIZE;
    if (z->count < ZUPT_WINDOW_SIZE) z->count++;
    
    if (z->count < ZUPT_WINDOW_SIZE) return false;
    
    float var_ax, var_ay, var_az;
    float var_gx, var_gy, var_gz;
    
    arm_var_f32(z->accel_buf[0], ZUPT_WINDOW_SIZE, &var_ax);
    arm_var_f32(z->accel_buf[1], ZUPT_WINDOW_SIZE, &var_ay);
    arm_var_f32(z->accel_buf[2], ZUPT_WINDOW_SIZE, &var_az);
    
    arm_var_f32(z->gyro_buf[0], ZUPT_WINDOW_SIZE, &var_gx);
    arm_var_f32(z->gyro_buf[1], ZUPT_WINDOW_SIZE, &var_gy);
    arm_var_f32(z->gyro_buf[2], ZUPT_WINDOW_SIZE, &var_gz);
    
    float accel_var = (var_ax + var_ay + var_az) * 0.333333f;
    float gyro_var = (var_gx + var_gy + var_gz) * 0.333333f;
    
    if (out_accel_var) *out_accel_var = accel_var;
    if (out_gyro_var) *out_gyro_var = gyro_var;
    
		return (accel_var < ZUPT_ACCEL_VAR_THRESH) && (gyro_var < ZUPT_GYRO_VAR_THRESH);
}

void ME_Init(MotionEstimator_Handle_t *me, ICM42670_Handle_t *imu, uint8_t divider)
{
    memset(me, 0, sizeof(MotionEstimator_Handle_t));
    me->imu = imu;
    me->alpha = 0.98f;
    me->calib_samples = 800;
    me->q[0] = 1.0f;
    ME_SetDivider(me, divider);
    ME_Reset(me);
}

void ME_SetDivider(MotionEstimator_Handle_t *me, uint8_t divider)
{
    if (divider == 0) divider = 1;
    me->divider = divider;
    me->dt = divider / BASE_FREQ;
}

void ME_SetAlpha(MotionEstimator_Handle_t *me, float alpha)
{
    me->alpha = alpha;
}

void ME_UpdateGyroBias(MotionEstimator_Handle_t *me, float bx, float by, float bz)
{
    me->gyro_bias[0] = bx;
    me->gyro_bias[1] = by;
    me->gyro_bias[2] = bz;
}

void ME_Reset(MotionEstimator_Handle_t *me)
{
    me->state = ME_STATE_CALIBRATING;
    me->calib_count = 0;
    me->accel_sum[0] = me->accel_sum[1] = me->accel_sum[2] = 0.0f;
    me->gyro_sum[0] = me->gyro_sum[1] = me->gyro_sum[2] = 0.0f;
    me->gyro_bias[0] = me->gyro_bias[1] = me->gyro_bias[2] = 0.0f;
    me->linear_accel_bias[0] = me->linear_accel_bias[1] = me->linear_accel_bias[2] = 0.0f;
    me->sample_count = 0;
    me->is_stationary = false;
    ZUPT_Init(&me->zupt);
}

bool ME_Update(MotionEstimator_Handle_t *me, float ax, float ay, float az, float gx, float gy, float gz)
{
    me->sample_count++;
    if (me->sample_count < me->divider) {
        return false;
    }
    me->sample_count = 0;

    if (me->state == ME_STATE_CALIBRATING) {
        me->accel_sum[0] += ax;
        me->accel_sum[1] += ay;
        me->accel_sum[2] += az;
        me->gyro_sum[0] += gx;
        me->gyro_sum[1] += gy;
        me->gyro_sum[2] += gz;
        me->calib_count++;
        
        if (me->calib_count >= me->calib_samples) {
            float avg_ax = me->accel_sum[0] / me->calib_count;
            float avg_ay = me->accel_sum[1] / me->calib_count;
            float avg_az = me->accel_sum[2] / me->calib_count;
            accel_to_quat(avg_ax, avg_ay, avg_az, me->q);

            me->gyro_bias[0] = me->gyro_sum[0] / me->calib_count;
            me->gyro_bias[1] = me->gyro_sum[1] / me->calib_count;
            me->gyro_bias[2] = me->gyro_sum[2] / me->calib_count;
            
            me->state = ME_STATE_RUNNING;
        }
        return false;
    }

    float wx = (gx - me->gyro_bias[0]) * DEG2RAD;
    float wy = (gy - me->gyro_bias[1]) * DEG2RAD;
    float wz = (gz - me->gyro_bias[2]) * DEG2RAD;
    
    float dt = me->dt;
    float dq0 = 0.5f * (-me->q[1]*wx - me->q[2]*wy - me->q[3]*wz) * dt;
    float dq1 = 0.5f * ( me->q[0]*wx + me->q[2]*wz - me->q[3]*wy) * dt;
    float dq2 = 0.5f * ( me->q[0]*wy - me->q[1]*wz + me->q[3]*wx) * dt;
    float dq3 = 0.5f * ( me->q[0]*wz + me->q[1]*wy - me->q[2]*wx) * dt;
    
    me->q[0] += dq0;
    me->q[1] += dq1;
    me->q[2] += dq2;
    me->q[3] += dq3;
    normalize_quat(me->q);

    float a_norm = sqrtf(ax*ax + ay*ay + az*az);
    if (fabsf(a_norm - 1.0f) < 0.1f) {
        float anx = ax / a_norm;
        float any = ay / a_norm;
        float anz = az / a_norm;
        
        float gx_est = 2.0f * (me->q[1]*me->q[3] - me->q[0]*me->q[2]);
        float gy_est = 2.0f * (me->q[0]*me->q[1] + me->q[2]*me->q[3]);
        float gz_est = me->q[0]*me->q[0] - me->q[1]*me->q[1] - me->q[2]*me->q[2] + me->q[3]*me->q[3];
        
        float ex = any * gz_est - anz * gy_est;
        float ey = anz * gx_est - anx * gz_est;
        float ez = anx * gy_est - any * gx_est;
        
        float gain = (1.0f - me->alpha) * 0.5f;
        me->q[1] += gain * ex;
        me->q[2] += gain * ey;
        me->q[3] += gain * ez;
        normalize_quat(me->q);
    }

    float gx_body = GRAVITY * 2.0f * (me->q[1]*me->q[3] - me->q[0]*me->q[2]);
    float gy_body = GRAVITY * 2.0f * (me->q[0]*me->q[1] + me->q[2]*me->q[3]);
    float gz_body = GRAVITY * (me->q[0]*me->q[0] - me->q[1]*me->q[1] - me->q[2]*me->q[2] + me->q[3]*me->q[3]);
    
    float raw_linear_accel[3];
    raw_linear_accel[0] = ax * GRAVITY - gx_body;
    raw_linear_accel[1] = ay * GRAVITY - gy_body;
    raw_linear_accel[2] = az * GRAVITY - gz_body;

    me->is_stationary = ZUPT_Update(&me->zupt, ax, ay, az, gx, gy, gz,
                                    &me->zupt_accel_var, &me->zupt_gyro_var);

    if (me->is_stationary) {
        me->gyro_bias[0] += GYRO_BIAS_ALPHA * (gx - me->gyro_bias[0]);
        me->gyro_bias[1] += GYRO_BIAS_ALPHA * (gy - me->gyro_bias[1]);
        me->gyro_bias[2] += GYRO_BIAS_ALPHA * (gz - me->gyro_bias[2]);

        me->linear_accel_bias[0] += ACCEL_BIAS_ALPHA * (raw_linear_accel[0] - me->linear_accel_bias[0]);
        me->linear_accel_bias[1] += ACCEL_BIAS_ALPHA * (raw_linear_accel[1] - me->linear_accel_bias[1]);
        me->linear_accel_bias[2] += ACCEL_BIAS_ALPHA * (raw_linear_accel[2] - me->linear_accel_bias[2]);
    }
    
    me->linear_accel[0] = raw_linear_accel[0] - me->linear_accel_bias[0];
    me->linear_accel[1] = raw_linear_accel[1] - me->linear_accel_bias[1];
    me->linear_accel[2] = raw_linear_accel[2] - me->linear_accel_bias[2];
    
    return true;
}

void ME_GetEuler(MotionEstimator_Handle_t *me, float *roll, float *pitch, float *yaw)
{
    float q0 = me->q[0], q1 = me->q[1], q2 = me->q[2], q3 = me->q[3];
    
    *roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
    *pitch = asinf(2.0f*(q0*q2 - q3*q1));
    *yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
}

bool ME_IsStationary(MotionEstimator_Handle_t *me)
{
    return me->is_stationary;
}