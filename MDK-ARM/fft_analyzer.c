#include "fft_analyzer.h"
#include "arm_math.h"
#include "signal_preprocess.h"
#include <string.h>
#include <math.h>

static void FFT_GenerateWindow(FFT_Analyzer_t *fft);
static void FFT_FindPeaks(FFT_Analyzer_t *fft);
static void FFT_CalcVelocityRMS(FFT_Analyzer_t *fft);
static void FFT_CalcVelocityRMSFromBuf(const float *mag, float *result);
static void FFT_UpdateAveraging(FFT_Analyzer_t *fft);

bool FFT_Init(FFT_Analyzer_t *fft)
{
    memset(fft->input_buf, 0, sizeof(fft->input_buf));
    memset(fft->fft_in, 0, sizeof(fft->fft_in));
    memset(fft->fft_out, 0, sizeof(fft->fft_out));
    memset(fft->mag_buf, 0, sizeof(fft->mag_buf));
    memset(fft->avg_mag, 0, sizeof(fft->avg_mag));
    memset(fft->peaks, 0, sizeof(fft->peaks));
    
    fft->sample_idx = 0;
    fft->fill_buf = 0;
    fft->process_pending = false;
    fft->result_ready = false;
    fft->frame_count = 0;
    fft->velocity_rms = 0.0f;
    fft->avg_velocity_rms = 0.0f;
    fft->overflow_count = 0;
    fft->avg_count = 0;
    fft->avg_target = 1;
    fft->avg_ready = false;
    FFT_GenerateWindow(fft);
    
    arm_status status = arm_rfft_fast_init_f32(&fft->rfft_instance, FFT_SIZE);
    
    return (status == ARM_MATH_SUCCESS);
}

static void FFT_GenerateWindow(FFT_Analyzer_t *fft)
{
    for (uint16_t i = 0; i < FFT_SIZE; i++) {
        fft->window[i] = 0.5f * (1.0f - cosf(2.0f * PI * i / (FFT_SIZE - 1)));
    }
}

bool FFT_AddSample(FFT_Analyzer_t *fft, float accel_g)
{
    uint8_t buf = fft->fill_buf;
    fft->input_buf[buf][fft->sample_idx] = accel_g;
    fft->sample_idx++;
    
    if (fft->sample_idx >= FFT_SIZE) {
        fft->sample_idx = 0;
        
        if (fft->process_pending) {
            fft->overflow_count++;  
        }
        
        fft->fill_buf = buf ^ 1;
        fft->process_pending = true;
        fft->result_ready = false;
        return true;
    }
    
    return false;
}

void FFT_Process(FFT_Analyzer_t *fft)
{
    if (!fft->process_pending) {
        return;
    }

    __disable_irq();
    uint8_t proc_buf = fft->fill_buf ^ 1;
    fft->process_pending = false;
    __enable_irq();
    
    float *input = fft->input_buf[proc_buf];
    
    Preprocess_RemoveDC(input, FFT_SIZE);
    arm_mult_f32(input, fft->window, fft->fft_in, FFT_SIZE);
    
    arm_rfft_fast_f32(&fft->rfft_instance, fft->fft_in, fft->fft_out, 0);
    
    fft->mag_buf[0] = fabsf(fft->fft_out[0]);
    fft->mag_buf[FFT_SIZE / 2] = fabsf(fft->fft_out[1]);
    arm_cmplx_mag_f32(&fft->fft_out[2], &fft->mag_buf[1], FFT_SIZE / 2 - 1);
    
    arm_scale_f32(fft->mag_buf, 2.828427f / FFT_SIZE, fft->mag_buf, FFT_SIZE / 2 + 1);
    
    FFT_FindPeaks(fft);
    FFT_CalcVelocityRMS(fft);
    FFT_UpdateAveraging(fft);
    
    fft->frame_count++;
    fft->result_ready = true;
}

static void FFT_UpdateAveraging(FFT_Analyzer_t *fft)
{
    if (fft->avg_target <= 1) {
        memcpy(fft->avg_mag, fft->mag_buf, sizeof(fft->avg_mag));
        fft->avg_velocity_rms = fft->velocity_rms;
        fft->avg_ready = true;
        return;
    }
    
    if (fft->avg_count == 0) {
        memcpy(fft->avg_mag, fft->mag_buf, sizeof(fft->avg_mag));
    } else {
        arm_add_f32(fft->avg_mag, fft->mag_buf, fft->avg_mag, FFT_SIZE / 2 + 1);
    }
    fft->avg_count++;
    
    if (fft->avg_count >= fft->avg_target) {
        arm_scale_f32(fft->avg_mag, 1.0f / fft->avg_target, fft->avg_mag, FFT_SIZE / 2 + 1);
        FFT_CalcVelocityRMSFromBuf(fft->avg_mag, &fft->avg_velocity_rms);
        fft->avg_count = 0;
        fft->avg_ready = true;
    } else {
        fft->avg_ready = false;
    }
}

static void FFT_CalcVelocityRMS(FFT_Analyzer_t *fft)
{
    FFT_CalcVelocityRMSFromBuf(fft->mag_buf, &fft->velocity_rms);
}

static void FFT_CalcVelocityRMSFromBuf(const float *mag, float *result)
{
    uint16_t start_bin = FFT_FreqToBin(FFT_VEL_MIN_FREQ);
    uint16_t end_bin = FFT_FreqToBin(FFT_VEL_MAX_FREQ);
    
    if (start_bin < 1) start_bin = 1;
    if (end_bin > FFT_SIZE / 2) end_bin = FFT_SIZE / 2;
    
    float sum_sq = 0.0f;
    float two_pi = 2.0f * PI;
    float scale = FFT_GRAVITY * 1000.0f;
    
		for (uint16_t i = start_bin; i <= end_bin; i++) {
				float freq = FFT_BinToFreq(i);
				if (freq < 1.0f) continue;  
				float accel_mag = mag[i];
				float vel_mag = (accel_mag * scale) / (two_pi * freq);
				sum_sq += vel_mag * vel_mag;
		}
    
    *result = sqrtf(sum_sq);
}

static float FFT_InterpolatePeakFreq(FFT_Analyzer_t *fft, uint16_t bin)
{
    if (bin < 1 || bin >= FFT_SIZE / 2) {
        return FFT_BinToFreq(bin);
    }
    
    float y0 = fft->mag_buf[bin - 1];
    float y1 = fft->mag_buf[bin];
    float y2 = fft->mag_buf[bin + 1];
    
    float denom = y0 - 2.0f * y1 + y2;
		if (fabsf(denom) < 1e-6f) {
				return FFT_BinToFreq(bin);
		}
		float delta = 0.5f * (y0 - y2) / denom;
		if (delta > 0.5f) delta = 0.5f;
		if (delta < -0.5f) delta = -0.5f;
		return (bin + delta) * FFT_SAMPLE_RATE / FFT_SIZE;
}

static void FFT_FindPeaks(FFT_Analyzer_t *fft)
{
    for (uint8_t i = 0; i < FFT_NUM_PEAKS; i++) {
        fft->peaks[i].freq = 0;
        fft->peaks[i].magnitude = 0;
        fft->peaks[i].bin = 0;
    }
    
    uint16_t start_bin = (uint16_t)(FFT_MIN_FREQ * FFT_SIZE / FFT_SAMPLE_RATE);
    if (start_bin < 1) start_bin = 1;
    
    for (uint16_t i = start_bin; i < FFT_SIZE / 2; i++) {
        float mag = fft->mag_buf[i];
        
        if (mag < FFT_MIN_MAGNITUDE) {
            continue;
        }
        
        bool is_peak = true;
				if (i > 0 && fft->mag_buf[i - 1] >= mag) is_peak = false;
				if (i < FFT_SIZE / 2 - 1 && fft->mag_buf[i + 1] >= mag) is_peak = false;
        
        if (!is_peak) {
            continue;
        }
        
        for (uint8_t j = 0; j < FFT_NUM_PEAKS; j++) {
            if (mag > fft->peaks[j].magnitude) {
                for (uint8_t k = FFT_NUM_PEAKS - 1; k > j; k--) {
                    fft->peaks[k] = fft->peaks[k - 1];
                }
                fft->peaks[j].magnitude = mag;
                fft->peaks[j].bin = i;
                fft->peaks[j].freq = FFT_InterpolatePeakFreq(fft, i);
                break;
            }
        }
    }
}

void FFT_SetAveraging(FFT_Analyzer_t *fft, uint16_t num_averages)
{
    if (num_averages < 1) num_averages = 1;
		if (num_averages > 256) num_averages = 256;  
		fft->avg_target = num_averages;
    fft->avg_count = 0;
    fft->avg_ready = false;
    memset(fft->avg_mag, 0, sizeof(fft->avg_mag));
}

void FFT_ResetAveraging(FFT_Analyzer_t *fft)
{
    fft->avg_count = 0;
    fft->avg_ready = false;
    memset(fft->avg_mag, 0, sizeof(fft->avg_mag));
}

bool FFT_IsAverageReady(FFT_Analyzer_t *fft)
{
    return fft->avg_ready;
}

float FFT_GetAveragedVelocityRMS(FFT_Analyzer_t *fft)
{
    return fft->avg_velocity_rms;
}

uint16_t FFT_GetAveragedSpectrum(FFT_Analyzer_t *fft, float *mag_out, uint16_t len)
{
    if (!fft->avg_ready || mag_out == NULL) {
        return 0;
    }
    
    uint16_t copy_len = (len < (FFT_SIZE / 2 + 1)) ? len : (FFT_SIZE / 2 + 1);
    memcpy(mag_out, fft->avg_mag, copy_len * sizeof(float));
    
    return copy_len;
}

uint8_t FFT_GetPeaks(FFT_Analyzer_t *fft, FFT_Peak_t *peaks, uint8_t n)
{
    if (!fft->result_ready || peaks == NULL) {
        return 0;
    }
    
    uint8_t count = (n < FFT_NUM_PEAKS) ? n : FFT_NUM_PEAKS;
    
    for (uint8_t i = 0; i < count; i++) {
        peaks[i] = fft->peaks[i];
    }
    
    return count;
}

uint16_t FFT_GetSpectrum(FFT_Analyzer_t *fft, float *mag_out, uint16_t len)
{
    if (!fft->result_ready || mag_out == NULL) {
        return 0;
    }
    
    uint16_t copy_len = (len < (FFT_SIZE / 2 + 1)) ? len : (FFT_SIZE / 2 + 1);
    
    memcpy(mag_out, fft->mag_buf, copy_len * sizeof(float));
    
    return copy_len;
}

float FFT_GetVelocityRMS(FFT_Analyzer_t *fft)
{
    return fft->velocity_rms;
}

bool FFT_TriAxis_Init(FFT_TriAxis_t *tri)
{
    bool success = true;
    tri->total_velocity_rms = 0.0f;
    tri->total_avg_velocity_rms = 0.0f;
    for (uint8_t i = 0; i < FFT_AXIS_COUNT; i++) {
        if (!FFT_Init(&tri->axis[i])) {
            success = false;
        }
    }
    return success;
}

void FFT_TriAxis_AddSamples(FFT_TriAxis_t *tri, float accel[3])
{
    for (uint8_t i = 0; i < FFT_AXIS_COUNT; i++) {
        FFT_AddSample(&tri->axis[i], accel[i]);
    }
}

bool FFT_TriAxis_Process(FFT_TriAxis_t *tri)
{
    bool any_processed = false;
    for (uint8_t i = 0; i < FFT_AXIS_COUNT; i++) {
        if (tri->axis[i].process_pending) {
            FFT_Process(&tri->axis[i]);
            any_processed = true;
        }
    }
    
    if (any_processed) {
        float sum_sq = 0.0f;
        for (uint8_t i = 0; i < FFT_AXIS_COUNT; i++) {
            float v = tri->axis[i].velocity_rms;
            sum_sq += v * v;
        }
        tri->total_velocity_rms = sqrtf(sum_sq);
        
        if (FFT_TriAxis_IsAverageReady(tri)) {
            sum_sq = 0.0f;
            for (uint8_t i = 0; i < FFT_AXIS_COUNT; i++) {
                float v = tri->axis[i].avg_velocity_rms;
                sum_sq += v * v;
            }
            tri->total_avg_velocity_rms = sqrtf(sum_sq);
        }
    }
    
    return any_processed;
}

uint8_t FFT_TriAxis_GetPeaks(FFT_TriAxis_t *tri, FFT_Axis_t axis, FFT_Peak_t *peaks, uint8_t n)
{
    if (axis >= FFT_AXIS_COUNT) return 0;
    return FFT_GetPeaks(&tri->axis[axis], peaks, n);
}

bool FFT_TriAxis_IsReady(FFT_TriAxis_t *tri, FFT_Axis_t axis)
{
    if (axis >= FFT_AXIS_COUNT) return false;
    return tri->axis[axis].result_ready;
}

float FFT_TriAxis_GetVelocityRMS(FFT_TriAxis_t *tri, FFT_Axis_t axis)
{
    if (axis >= FFT_AXIS_COUNT) return 0.0f;
    return tri->axis[axis].velocity_rms;
}

float FFT_TriAxis_GetTotalVelocityRMS(FFT_TriAxis_t *tri)
{
    return tri->total_velocity_rms;
}

void FFT_TriAxis_SetAveraging(FFT_TriAxis_t *tri, uint16_t num_averages)
{
    for (uint8_t i = 0; i < FFT_AXIS_COUNT; i++) {
        FFT_SetAveraging(&tri->axis[i], num_averages);
    }
}

void FFT_TriAxis_ResetAveraging(FFT_TriAxis_t *tri)
{
    for (uint8_t i = 0; i < FFT_AXIS_COUNT; i++) {
        FFT_ResetAveraging(&tri->axis[i]);
    }
}

bool FFT_TriAxis_IsAverageReady(FFT_TriAxis_t *tri)
{
    for (uint8_t i = 0; i < FFT_AXIS_COUNT; i++) {
        if (!tri->axis[i].avg_ready) {
            return false;
        }
    }
    return true;
}

float FFT_TriAxis_GetAveragedVelocityRMS(FFT_TriAxis_t *tri, FFT_Axis_t axis)
{
    if (axis >= FFT_AXIS_COUNT) return 0.0f;
    return tri->axis[axis].avg_velocity_rms;
}

float FFT_TriAxis_GetTotalAveragedVelocityRMS(FFT_TriAxis_t *tri)
{
    return tri->total_avg_velocity_rms;
}