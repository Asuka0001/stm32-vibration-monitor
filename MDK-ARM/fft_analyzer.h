#ifndef FFT_ANALYZER_H
#define FFT_ANALYZER_H
#include "main.h"
#include "arm_math.h"
#include <stdbool.h>
#include <stdint.h>
#define FFT_SIZE            1024
#define FFT_SAMPLE_RATE     1600.0f
#define FFT_NUM_PEAKS       3
#define FFT_MIN_FREQ        2.0f
#define FFT_MIN_MAGNITUDE   0.001f
#define FFT_VEL_MIN_FREQ    2.0f
#define FFT_VEL_MAX_FREQ    800.0f
#define FFT_GRAVITY         9.80665f
typedef struct {
    float freq;
    float magnitude;
    uint16_t bin;
} FFT_Peak_t;
typedef struct {
    float input_buf[2][FFT_SIZE];
    float window[FFT_SIZE];
    float fft_in[FFT_SIZE];             
    float fft_out[FFT_SIZE];         
    float mag_buf[FFT_SIZE / 2 + 1];
    float avg_mag[FFT_SIZE / 2 + 1];
    arm_rfft_fast_instance_f32 rfft_instance;
    
    uint16_t sample_idx;
    uint8_t  fill_buf;
    volatile bool process_pending;
    bool result_ready;
    
    FFT_Peak_t peaks[FFT_NUM_PEAKS];
    uint32_t frame_count;
    
    float velocity_rms;
    float avg_velocity_rms;
    volatile uint32_t overflow_count;
    
    uint16_t avg_count;
    uint16_t avg_target;
    bool avg_ready;
    
} FFT_Analyzer_t;
typedef enum {
    FFT_AXIS_X = 0,
    FFT_AXIS_Y,
    FFT_AXIS_Z,
    FFT_AXIS_COUNT
} FFT_Axis_t;
typedef struct {
    FFT_Analyzer_t axis[FFT_AXIS_COUNT];
    float total_velocity_rms;
    float total_avg_velocity_rms;
} FFT_TriAxis_t;
bool FFT_Init(FFT_Analyzer_t *fft);
bool FFT_AddSample(FFT_Analyzer_t *fft, float accel_g);
void FFT_Process(FFT_Analyzer_t *fft);
uint8_t FFT_GetPeaks(FFT_Analyzer_t *fft, FFT_Peak_t *peaks, uint8_t n);
uint16_t FFT_GetSpectrum(FFT_Analyzer_t *fft, float *mag_out, uint16_t len);
float FFT_GetVelocityRMS(FFT_Analyzer_t *fft);
void FFT_SetAveraging(FFT_Analyzer_t *fft, uint16_t num_averages);
void FFT_ResetAveraging(FFT_Analyzer_t *fft);
bool FFT_IsAverageReady(FFT_Analyzer_t *fft);
float FFT_GetAveragedVelocityRMS(FFT_Analyzer_t *fft);
uint16_t FFT_GetAveragedSpectrum(FFT_Analyzer_t *fft, float *mag_out, uint16_t len);
bool FFT_TriAxis_Init(FFT_TriAxis_t *tri);
void FFT_TriAxis_AddSamples(FFT_TriAxis_t *tri, float accel[3]);
bool FFT_TriAxis_Process(FFT_TriAxis_t *tri);
uint8_t FFT_TriAxis_GetPeaks(FFT_TriAxis_t *tri, FFT_Axis_t axis, FFT_Peak_t *peaks, uint8_t n);
bool FFT_TriAxis_IsReady(FFT_TriAxis_t *tri, FFT_Axis_t axis);
float FFT_TriAxis_GetVelocityRMS(FFT_TriAxis_t *tri, FFT_Axis_t axis);
float FFT_TriAxis_GetTotalVelocityRMS(FFT_TriAxis_t *tri);
void FFT_TriAxis_SetAveraging(FFT_TriAxis_t *tri, uint16_t num_averages);
void FFT_TriAxis_ResetAveraging(FFT_TriAxis_t *tri);
bool FFT_TriAxis_IsAverageReady(FFT_TriAxis_t *tri);
float FFT_TriAxis_GetAveragedVelocityRMS(FFT_TriAxis_t *tri, FFT_Axis_t axis);
float FFT_TriAxis_GetTotalAveragedVelocityRMS(FFT_TriAxis_t *tri);
static inline float FFT_BinToFreq(uint16_t bin) {
    return (float)bin * FFT_SAMPLE_RATE / FFT_SIZE;
}
static inline uint16_t FFT_FreqToBin(float freq) {
    return (uint16_t)(freq * FFT_SIZE / FFT_SAMPLE_RATE + 0.5f);
}
#endif