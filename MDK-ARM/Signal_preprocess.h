#ifndef SIGNAL_PREPROCESS_H
#define SIGNAL_PREPROCESS_H

#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"

#define PREPROCESS_SAMPLE_RATE  1600.0f
#define PREPROCESS_HP_CUTOFF    1.0f
#define PREPROCESS_LP_CUTOFF    600.0f

#define HP_NUM_STAGES  2
#define LP_NUM_STAGES  2

typedef struct {
    arm_biquad_cascade_df2T_instance_f32 hp_inst;
    arm_biquad_cascade_df2T_instance_f32 lp_inst;
    float hp_coeffs[HP_NUM_STAGES * 5];
    float lp_coeffs[LP_NUM_STAGES * 5];
    float hp_state[HP_NUM_STAGES * 2];
    float lp_state[LP_NUM_STAGES * 2];
} SignalPreprocess_t;

void Preprocess_Init(SignalPreprocess_t *pp, float sample_rate, float hp_cutoff, float lp_cutoff);
void Preprocess_InitDefault(SignalPreprocess_t *pp);
void Preprocess_Reset(SignalPreprocess_t *pp);
float Preprocess_Sample(SignalPreprocess_t *pp, float raw);
void Preprocess_BatchProcess(SignalPreprocess_t *pp, const float *in, float *out, uint16_t len);
void Preprocess_RemoveDC(float *buf, uint16_t len);
void Preprocess_Warmup(SignalPreprocess_t *pp, float first_sample, uint16_t cycles);

#endif