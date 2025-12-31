#include "vibration_monitor.h"
#include <string.h>
#include <math.h>

#define VIB_DC_ALPHA   0.995f
#define VIB_LF_THRESH  0.01f
		
void VIBMON_Init(VIBMON_Handle_t *vm, uint32_t windowSize, float samplingRate)
{
    if (!vm || windowSize == 0)
        return;
    
    memset(vm, 0, sizeof(VIBMON_Handle_t));
    vm->windowSize = windowSize;
    vm->samplingRate = samplingRate;
    vm->peakThreshold = 0.3f;
		
    //vm->dc = 1.0f;
		vm->dc = 0.0f;
    vm->last_ac = 0.0f;
    vm->lastCross = 0;
}

void VIBMON_Reset(VIBMON_Handle_t *vm)
{
    if (!vm)
        return;
    
    vm->energy_acc = 0.0f;
    vm->peak_acc = 0.0f;
    vm->sampleCount = 0;
    
    vm->lastMag = 0.0f;
    vm->lastLastMag = 0.0f;
    vm->lastPeakSample = 0;
    vm->freqSum = 0.0f;
    vm->freqCount = 0;
    vm->dc = 0.0f;
    vm->last_ac = 0.0f;
    vm->lastCross = 0;
}

void VIBMON_Update(VIBMON_Handle_t *vm,
                   const ADXL345_Acceleration_t *rawAccel)
{
    if (!vm || !rawAccel)
        return;
    
    float mag_squared = rawAccel->x * rawAccel->x +
                        rawAccel->y * rawAccel->y +
                        rawAccel->z * rawAccel->z;
    
    float magnitude = sqrtf(mag_squared);

    vm->sampleCount++;
    vm->dc = VIB_DC_ALPHA * vm->dc + (1.0f - VIB_DC_ALPHA) * magnitude;

    float ac = magnitude - vm->dc;
    
    if (ac > VIB_LF_THRESH && vm->last_ac <= VIB_LF_THRESH) {
        uint32_t now = vm->sampleCount;

        if (vm->lastCross > 0) {
            uint32_t samplesBetween = now - vm->lastCross;

            uint32_t minSamples = (uint32_t)(vm->samplingRate / 6.0f);
            uint32_t maxSamples = (uint32_t)(vm->samplingRate / 1.5f);

            if (samplesBetween >= minSamples &&
                samplesBetween <= maxSamples) {
                float freq = vm->samplingRate / samplesBetween;
                vm->freqSum += freq;
                vm->freqCount++;
            }
        }

        vm->lastCross = now;
    }

    vm->last_ac = ac;
    vm->energy_acc += mag_squared;
    
    if (magnitude > vm->peak_acc) {
        vm->peak_acc = magnitude;
    }
    
    vm->lastLastMag = vm->lastMag;
    vm->lastMag = magnitude;

    if (vm->sampleCount >= vm->windowSize) {
        vm->stats.rms = sqrtf(vm->energy_acc / vm->sampleCount);
        vm->stats.peak = vm->peak_acc;
        vm->stats.sampleCount = vm->sampleCount;
        
        if (vm->freqCount > 0) {
            vm->stats.frequency = vm->freqSum / vm->freqCount;
        } else {
            vm->stats.frequency = 0.0f;
        }
        
        VIBMON_Reset(vm);
    }
}

void VIBMON_GetStats(VIBMON_Handle_t *vm, VIBMON_Stats_t *out)
{
    if (!vm || !out)
        return;
    *out = vm->stats;
}