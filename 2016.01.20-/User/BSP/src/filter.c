#include "filter.h"

//-----Butterworth滤波器系数-----//
/* Low-pass ButterWorth filter */
/* Accelerometor   */
/* Order: 2 */
/* Cutoff Frequency: 30Hz */
//const static float b_acc[3] = {0.0913f, 0.1826f, 0.0913f};
//const static float a_acc[3] = {1.0f, -0.9824f, 0.3477f};

#define FILTER_NUM 	20
#define M_PI_F 3.1415926

static float b_acc[3];
static float a_acc[3];
/**************************************************

通过此函数可以获得二阶巴特沃斯的系数

****************************************************/
void LPF2pSetCutoffFreq(float sample_freq, float cutoff_freq)
{
		float fr =0;  
    float ohm =0;
    float c =0;
	
		fr= sample_freq/cutoff_freq;
		ohm=tanf(M_PI_F/fr);
		c=1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;
	
//    _cutoff_freq1 = cutoff_freq;
//    if (_cutoff_freq1 > 0.0f) 
//		{
				b_acc[0] = ohm*ohm/c;
				b_acc[1] = 2.0f*b_acc[0];
				b_acc[2] = b_acc[0];
				a_acc[1] = 2.0f*(ohm*ohm-1.0f)/c;
				a_acc[2] = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
//		}
}
/******************************************
加速度计二阶巴特沃斯滤波，


*******************************************/
static float           _delay_element_11;        // buffered sample -1
static float           _delay_element_21;        // buffered sample -2

float LPF2pApply_1(float sample)
{
	
		float delay_element_0 = 0, output=0;
//    if (_cutoff_freq1 <= 0.0f) {
//        // no filtering
//        return sample;
//    }
//		else
//		{
				delay_element_0 = sample - _delay_element_11 * a_acc[1] - _delay_element_21 * a_acc[2];
				// do the filtering
				if (isnan(delay_element_0) || isinf(delay_element_0)) {
						// don't allow bad values to propogate via the filter
						delay_element_0 = sample;
				}
				output = delay_element_0 * b_acc[0] + _delay_element_11 * b_acc[1] + _delay_element_21 * b_acc[2];
				
				_delay_element_21 = _delay_element_11;
				_delay_element_11 = delay_element_0;

				// return the value.  Should be no need to check limits
				return output;
//		}
}

static float           _delay_element_12;        // buffered sample -1
static float           _delay_element_22;        // buffered sample -2

float LPF2pApply_2(float sample)
{
	
		float delay_element_0 = 0, output=0;
//    if (_cutoff_freq1 <= 0.0f) {
//        // no filtering
//        return sample;
//    }
//		else
//		{
				delay_element_0 = sample - _delay_element_12 * a_acc[1] - _delay_element_22 * a_acc[2];
				// do the filtering
				if (isnan(delay_element_0) || isinf(delay_element_0)) {
						// don't allow bad values to propogate via the filter
						delay_element_0 = sample;
				}
				output = delay_element_0 * b_acc[0] + _delay_element_12 * b_acc[1] + _delay_element_22 * b_acc[2];
				
				_delay_element_22 = _delay_element_12;
				_delay_element_12 = delay_element_0;

				// return the value.  Should be no need to check limits
				return output;
//		}
}

static float           _delay_element_13;        // buffered sample -1
static float           _delay_element_23;        // buffered sample -2

float LPF2pApply_3(float sample)
{
	
		float delay_element_0 = 0, output=0;
//    if (_cutoff_freq1 <= 0.0f) {
//        // no filtering
//        return sample;
//    }
//		else
//		{
				delay_element_0 = sample - _delay_element_13 * a_acc[1] - _delay_element_23 * a_acc[2];
				// do the filtering
				if (isnan(delay_element_0) || isinf(delay_element_0)) {
						// don't allow bad values to propogate via the filter
						delay_element_0 = sample;
				}
				output = delay_element_0 * b_acc[0] + _delay_element_13 * b_acc[1] + _delay_element_23 * b_acc[2];
				
				_delay_element_23 = _delay_element_13;
				_delay_element_13 = delay_element_0;

				// return the value.  Should be no need to check limits
				return output;
//		}
}
/*****************************************************
函数：void Acc_Butterworth_Smooth(MPU6050_DATA *acc_in,MPU6050_DATA *acc_out)
输入：加速度计原始值
输出：滤波后的值
说明：巴特沃斯滤波
*******************************************************/
void Acc_Butterworth_Filter(MPU6050_DATA *acc_in,MPU6050_DATA *acc_out)
{
	acc_out->ax = LPF2pApply_1((float)acc_in->ax);
	acc_out->ay = LPF2pApply_2((float)acc_in->ay);
	acc_out->az = LPF2pApply_3((float)acc_in->az);
	
	acc_out->gx = acc_in->gx;
	acc_out->gy = acc_in->gy;
	acc_out->gz = acc_in->gz;
}

/*****************************************************
函数：void ACC_Smooth_Filter(MPU6050_DATA *acc_in,MPU6050_DATA *acc_out)
输入：加速度计原始值
输出：滤波后的值
说明：互动滤波
*******************************************************/
void ACC_Smooth_Filter(MPU6050_DATA *acc_in,MPU6050_DATA *acc_out)
{
	static uint8_t 	filter_cnt=0;
	static int16_t	ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];
	int32_t temp1=0,temp2=0,temp3=0;
	uint8_t i;

	ACC_X_BUF[filter_cnt] = acc_in->ax;
	ACC_Y_BUF[filter_cnt] = acc_in->ay;
	ACC_Z_BUF[filter_cnt] = acc_in->az;
	for(i=0;i<FILTER_NUM;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}
	acc_out->ax = temp1 / FILTER_NUM;
	acc_out->ay = temp2 / FILTER_NUM;
	acc_out->az = temp3 / FILTER_NUM;	
	filter_cnt++;
	if(filter_cnt==FILTER_NUM)	filter_cnt=0;
	acc_out->gx = acc_in->gx;
	acc_out->gy = acc_in->gy;
	acc_out->gz = acc_in->gz;
}
