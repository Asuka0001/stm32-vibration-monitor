#include "signal_preprocess.h"
#include <math.h>
#include <string.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

static const float BUTTERWORTH_4TH_Q[2] = {0.5411961f, 1.3065630f};

static void calc_hp_biquad_coeffs(float *coeffs, float fc, float fs, float Q)
{
    float omega = 2.0f * PI * fc / fs;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * Q);
    
    float a0 = 1.0f + alpha;
    float b0 = (1.0f + cs) / 2.0f / a0;
    float b1 = -(1.0f + cs) / a0;
    float b2 = (1.0f + cs) / 2.0f / a0;
    float a1 = -2.0f * cs / a0;
    float a2 = (1.0f - alpha) / a0;
    
    coeffs[0] = b0;
    coeffs[1] = b1;
    coeffs[2] = b2;
    coeffs[3] = -a1;
    coeffs[4] = -a2;
}

static void calc_lp_biquad_coeffs(float *coeffs, float fc, float fs, float Q)
{
    float omega = 2.0f * PI * fc / fs;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * Q);
    
    float a0 = 1.0f + alpha;
    float b0 = (1.0f - cs) / 2.0f / a0;
    float b1 = (1.0f - cs) / a0;
    float b2 = (1.0f - cs) / 2.0f / a0;
    float a1 = -2.0f * cs / a0;
    float a2 = (1.0f - alpha) / a0;
    
    coeffs[0] = b0;
    coeffs[1] = b1;
    coeffs[2] = b2;
    coeffs[3] = -a1;
    coeffs[4] = -a2;
}

void Preprocess_Init(SignalPreprocess_t *pp, float sample_rate, float hp_cutoff, float lp_cutoff)
{
    memset(pp, 0, sizeof(SignalPreprocess_t));
    
    for (int i = 0; i < HP_NUM_STAGES; i++) {
        calc_hp_biquad_coeffs(&pp->hp_coeffs[i * 5], hp_cutoff, sample_rate, BUTTERWORTH_4TH_Q[i]);
    }
    
    for (int i = 0; i < LP_NUM_STAGES; i++) {
        calc_lp_biquad_coeffs(&pp->lp_coeffs[i * 5], lp_cutoff, sample_rate, BUTTERWORTH_4TH_Q[i]);
    }
    
    arm_biquad_cascade_df2T_init_f32(&pp->hp_inst, HP_NUM_STAGES, pp->hp_coeffs, pp->hp_state);
    arm_biquad_cascade_df2T_init_f32(&pp->lp_inst, LP_NUM_STAGES, pp->lp_coeffs, pp->lp_state);
}

void Preprocess_InitDefault(SignalPreprocess_t *pp)
{
    Preprocess_Init(pp, PREPROCESS_SAMPLE_RATE, PREPROCESS_HP_CUTOFF, PREPROCESS_LP_CUTOFF);
}

void Preprocess_Reset(SignalPreprocess_t *pp)
{
    memset(pp->hp_state, 0, sizeof(pp->hp_state));
    memset(pp->lp_state, 0, sizeof(pp->lp_state));
}

float Preprocess_Sample(SignalPreprocess_t *pp, float raw)
{
    float hp_out, lp_out;
    arm_biquad_cascade_df2T_f32(&pp->hp_inst, &raw, &hp_out, 1);
    arm_biquad_cascade_df2T_f32(&pp->lp_inst, &hp_out, &lp_out, 1);
    return lp_out;
}

void Preprocess_BatchProcess(SignalPreprocess_t *pp, const float *in, float *out, uint16_t len)
{
    arm_biquad_cascade_df2T_f32(&pp->hp_inst, in, out, len);
    arm_biquad_cascade_df2T_f32(&pp->lp_inst, out, out, len);
}

void Preprocess_RemoveDC(float *buf, uint16_t len)
{
    float dc;
    arm_mean_f32(buf, len, &dc);
    arm_offset_f32(buf, -dc, buf, len);
}

void Preprocess_Warmup(SignalPreprocess_t *pp, float first_sample, uint16_t cycles)
{
    for (uint16_t i = 0; i < cycles; i++) {
        Preprocess_Sample(pp, first_sample);
    }
}