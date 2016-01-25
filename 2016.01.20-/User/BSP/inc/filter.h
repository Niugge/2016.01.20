#ifndef _FILTER_

#include "math.h"
#include "global.h"

void LPF2pSetCutoffFreq(float sample_freq, float cutoff_freq);
float LPF2pApply_1(float sample);
float LPF2pApply_2(float sample);
float LPF2pApply_2(float sample);
void Acc_Butterworth_Filter(MPU6050_DATA *acc_in,MPU6050_DATA *acc_out);
void ACC_Smooth_Filter(MPU6050_DATA *acc_in,MPU6050_DATA *acc_out);

#endif
