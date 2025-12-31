#ifndef VIBRATION_MONITOR_H
#define VIBRATION_MONITOR_H

#include <stdint.h>
#include "adxl345.h"

typedef struct {
    float rms;
    float peak;
    float frequency;
    uint32_t sampleCount;
    uint32_t peaksDetected;
} VIBMON_Stats_t;

typedef struct {
    uint32_t windowSize;
    float energy_acc;
    float peak_acc;
    uint32_t sampleCount;
    VIBMON_Stats_t stats;
    float lastMag;
    float lastLastMag;
    uint32_t lastPeakSample;
    float peakThreshold;
    float freqSum;
    uint32_t freqCount;
    float samplingRate;
    float dc;
    float last_ac;
    uint32_t lastCross;
    float dc_high;
    float last_ac_high;
    uint32_t lastCross_high;
    float freqSum_high;
    uint32_t freqCount_high;
    uint32_t zeroCrossCount;
    uint32_t validCrossCount;
    float acPeakInPeriod;
} VIBMON_Handle_t;

typedef struct {
    float highpassCutoff;
    float lowpassCutoff;
    float peakThresholdRatio;
} VIBMON_Config_t;

void VIBMON_Init(VIBMON_Handle_t *vm, uint32_t windowSize, float samplingRate);
void VIBMON_Reset(VIBMON_Handle_t *vm);
void VIBMON_Update(VIBMON_Handle_t *vm, const ADXL345_Acceleration_t *rawAccel);
void VIBMON_GetStats(VIBMON_Handle_t *vm, VIBMON_Stats_t *out);

#endif