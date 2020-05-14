#include "Arduino.h"
#include "motors.h"


extern volatile float left_encoder_count =0;
extern volatile float right_encoder_count=0;
extern volatile float left_RPM=0, right_RPM=0, left_prev_count=0, right_prev_count=0;

extern float pwm_right_offset = 0; 
extern float pwm_left_offset= 0; 

extern float encoder_set_point = 0;
extern float velocity_set_point = 0;

extern float average_theta = 0;
extern float average_RPM = 0;

extern float v[2] = {0};
extern float u[2] = {0};
 /*************************************************************************************
 * Function Name:update_encoder_states
 * Input  : no inputs.
 * Output : no output, Updates global variable average theta, average RPM 
 *          in radians and rps.
 * Logic  : calculates left and right encoder count and thus calculates its RPM 
 *          in rps. Updates average_theta and average_velocity which is used in 
 *          the controller.
 *          
 * Example Call: update_encoder_states()
 *************************************************************************************/ 

void update_encoder_states()
{
 
   // Make a local copy of the global encoder count
  volatile float left_current_count = left_encoder_count;
  volatile float right_current_count = right_encoder_count;
  
  //                (Change in encoder count) 
  // RPS =   __________________________________________
  //          (Change in time --> 5ms) * (PPR --> 540)
  
  left_RPM  = (float)((left_current_count - left_prev_count)  *  RPM_radian_converter);      
  right_RPM = (float)((right_current_count - right_prev_count)*  RPM_radian_converter);    

  
  // Store current encoder count for next iteration
  
  left_prev_count = left_current_count;
  right_prev_count = right_current_count;
  
   //  previous_yaw = yaw;
  //yaw =  (+right_current_count - left_current_count )* Yaw_radian_multiplier;
  //yaw_dot = (yaw - previous_yaw)*one_by_dT;

  
  average_theta = ((left_current_count + right_current_count)*0.0116); // 2pi/540

  average_RPM = (left_RPM + right_RPM)*0.5; 
}

 /*************************************************************************************
 * Function Name: left_encoder_interrupt
 * Input  : no input
 * Output : no outputs. updates global left_encoder_count variable.
 * Logic  : handles interrupt for left encoder.  
 * 
 * Example Call: called automatically.
 *************************************************************************************/
 
void left_encoder_interrupt()
{
  int state = digitalRead(ENCA1);
  if(digitalRead(ENCA2)) 
  state ? left_encoder_count-- : left_encoder_count++;
  else 
  state ? left_encoder_count++ : left_encoder_count--;
}


 /*************************************************************************************
 * Function Name: right_encoder_interrupt
 * Input  : no input
 * Output : no outputs. updates global right_encoder_count variable.
 * Logic  : handles interrupt for right motor encoder.  
 * 
 * Example Call: called automatically.
 *************************************************************************************/ 
 
void right_encoder_interrupt()
{
  int state = digitalRead(ENCB1);
  if(digitalRead(ENCB2)) 
  state ? right_encoder_count++ : right_encoder_count--;
  else 
  state ? right_encoder_count-- : right_encoder_count++;
}

 /*************************************************************************************
 * Function Name: encoder_config
 * Input  : no input
 * Output : no outputs. 
 * Logic  : encoder wires are configured as interrupts 
 *          at rising and falling edges. 
 * 
 * Example Call: encoder_config()
 *************************************************************************************/


void encoder_config()
{
pinMode(ENCA1, INPUT_PULLUP); // Encoder 1 - Channel A 
pinMode(ENCA2, INPUT_PULLUP); // Encoder 1 - Channel B 

pinMode(ENCB1, INPUT_PULLUP); // Encoder 2 - Channel A 
pinMode(ENCB2, INPUT_PULLUP); // Encoder 2 - Channel B  

// Attach interrupts for the encoder input pins
attachInterrupt(digitalPinToInterrupt(ENCA1), left_encoder_interrupt, CHANGE);
attachInterrupt(digitalPinToInterrupt(ENCB1), right_encoder_interrupt, CHANGE);
}


/************************************************************************************
 * Function Name: drive_motors
 * Input  : no inputs.
 * Output : no output.
 * Logic  : used to drive motors based on calculated pwm from lqr controller.
 *          The calculated voltages are constrained and mapped w.r.t corresponding 
 *          motor's offset.
 * Example Call: drive_motors(), called by update_motors().
 *************************************************************************************/
 

void drive_motors()
{

//backward motion

if(v[0]<0&&v[1]<0)
{
v[0] = map(constrain(v[0]+pwm_right_offset ,-255, 255), 0, -255, RIGHT_PWM_MIN, 255);
v[1] = map(constrain(v[1]+pwm_left_offset,-255, 255), 0, -255, LEFT_PWM_MIN, 255);

analogWrite(enR,v[0]);
analogWrite(enL,v[1]); 


PORTA = B00001001; 
}

// forward motion.
else if(v[0]>=0&&v[1]>=0)///(v[0][0]>0&&v[1][0]>0)
{
v[0] = map(constrain(v[0]+pwm_right_offset ,-255, 255), 0, 255, RIGHT_PWM_MIN, 255);
v[1] = map(constrain(v[1]+pwm_left_offset,-255, 255), 0, 255, LEFT_PWM_MIN, 255);

   
analogWrite(enR,v[0]);
analogWrite(enL,v[1]);

PORTA = B00000110;
   
}


}
