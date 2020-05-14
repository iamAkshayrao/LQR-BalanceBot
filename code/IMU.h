#ifndef IMU_H
#define IMU_H
#include "Arduino.h"
#define f_cut 5

//float dT   = 0.02;              //0.012
#define dT 0.005
#define one_by_dT 200
#define alpha1 0.03

extern int16_t ax_raw, ay_raw , az_raw;
extern int16_t gx_raw , gy_raw , gz_raw;
extern float pitch_set_point ;
extern float pitch_dot_set_point ;
extern float pitch , previous_pitch, pitch_dot; 
extern float current_pitch ;
extern float yaw, previous_yaw , yaw_dot;

 /*************************************************************************************
 * Function Name:scale_values()
 * Input  : no inputs.
 * Output : no outputs.
 * Logic  : scales the raw mpu's values 
 * Example Call: scale_values
 *************************************************************************************/ 
void scale_values();

 /*************************************************************************************
 * Function Name: lowpassfliter
 * Input: reference to ax, ay, az
 * Output: no output
 * Logic: does low pass filter operation.
 * Example Call: lowpassfilter(float*, float *, float*)
 *************************************************************************************/ 

void lowpassfilter(float* ax_addr, float* ay_addr, float* az_addr);

 /*************************************************************************************
 * Function Name:highpassfilter
 * Input`: reference to gx and gy 
 * Output: no return value 
 * Logic`: `performs highpass filter operation
 * Example Call: highpassfilter(float *,float*)
 *************************************************************************************/ 
 
void highpassfilter(float* gx, float* gy);

 /*************************************************************************************
 * Function Name: complementary_filter.
 * Input  : no inputs.
 * Output : returns Bot's filtered pitch angle in radians.
 * Logic  : Function first scales the raw values obtained to +-2g, calls high pass 
 *          and low pass filter and combines their values to get filtered pitch
 * Example Call: complementary_filter().
 *************************************************************************************/
 
float complementary_filter();

/*************************************************************************************
 * Function Name: read_tilt_angle
 * Input  : no inputs.
 * Output : no outputs. updates global variable pitch and pitch_dot.
 * Logic  : calls complementary filter and stores previous pitch angle 
 *          and updates current pitch angle and calculates pitch velocity(pitch_dot).
 *        
 * Example Call: read_tilt_angle().
 *************************************************************************************/ 
void read_tilt_angle();

#endif
