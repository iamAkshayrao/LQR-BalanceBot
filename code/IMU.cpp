#include "IMU.h"

int16_t ax_raw =0, ay_raw =0, az_raw=0;
int16_t gx_raw =0, gy_raw =0, gz_raw=0;


float ax = 0, ay =0, az =0;
float gx =0, gy =0, gz = 0;

float ax_lp = 0, ay_lp = 0, az_lp =0;
float gx_hp =0, gy_hp =0, gz_hp =0;

float gx_minusOne = 0, gy_minusOne = 0, gz_minusOne = 0;

float Tau  = 1/(2*PI*f_cut);
float alpha = Tau*(Tau+dT);

extern float pitch_set_point = 0;
extern float pitch_dot_set_point = 0;
extern float pitch = 0, previous_pitch = 0, pitch_dot = 0; 
extern float current_pitch = 0;
extern float yaw = 0, previous_yaw = 0, yaw_dot = 0;

 /*************************************************************************************
 * Function Name:scale_values()
 * Input  : no inputs.
 * Output : no outputs.
 * Logic  : scales the raw mpu's values 
 * Example Call: scale_values
 *************************************************************************************/ 
void scale_values()
{
 ax=(float)(ax_raw)/16384;
 ay=(float)(ay_raw)/16384;
 az=(float)(az_raw)/16384;
 gx=(float)(gx_raw)/131;
 gy=(float)(gy_raw)/131;
 gz=(float)(gz_raw)/131;
}  

 /*************************************************************************************
 * Function Name: lowpassfliter
 * Input: reference to ax, ay, az
 * Output: no output
 * Logic: does low pass filter operation.
 * Example Call: lowpassfilter(float*, float *, float*)
 *************************************************************************************/ 

void lowpassfilter(float* ax_addr, float* ay_addr, float* az_addr)
{

  

  ax_lp=(1-alpha)*(*ax_addr) + (alpha*(ax_lp));
  ay_lp=(1-alpha)*(*ay_addr) + (alpha*(ay_lp));
  az_lp=(1-alpha)*(*az_addr) + (alpha*(az_lp));

  (*ax_addr)=ax_lp;
  (*ay_addr)=ay_lp;
  (*az_addr)=az_lp;

  
}

 /*************************************************************************************
 * Function Name:highpassfilter
 * Input`: reference to gx and gy 
 * Output: no return value 
 * Logic`: `performs highpass filter operation
 * Example Call: highpassfilter(float *,float*)
 *************************************************************************************/ 
 
void highpassfilter(float* gx, float* gy)
{

  gx_hp = (1-alpha)*gx_hp + (1-alpha)*((*gx) - gx_minusOne);
  gy_hp = (1-alpha)*gy_hp + (1-alpha)*((*gy) - gy_minusOne);
  
  gx_minusOne = *gx;
  gy_minusOne = *gy;


  (*gx)=gx_hp;
  (*gy)=gy_hp;
  
}

 /*************************************************************************************
 * Function Name: complementary_filter.
 * Input  : no inputs.
 * Output : returns Bot's filtered pitch angle in radians.
 * Logic  : Function first scales the raw values obtained to +-2g, calls high pass 
 *          and low pass filter and combines their values to get filtered pitch
 * Example Call: complementary_filter().
 *************************************************************************************/
 
float complementary_filter()
{
  
  scale_values();
  lowpassfilter(&ax, &ay, &az);
  highpassfilter(&gx, &gy);
  
  current_pitch   = current_pitch - (gy*dT);
  float acc_angle = atan2(ax,sqrt(az*az))*(180/PI); 

  current_pitch   = (1-alpha1)*current_pitch + (alpha1)*(acc_angle); //complementary filter
  return current_pitch*(PI/180); // converting to radians

}
