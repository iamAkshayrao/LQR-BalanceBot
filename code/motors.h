 /* Time| RPM_radian_converter | one_by_dT |timer starting count
  
 * 5  2.327   200     0xFEC6
 * 8  1.4544  125     0XFE0B
 * 10 1.16355 100     0XFD8E
 * 12 0.9696  83.33   0XFD11
 * 15 0.7757  66.66   0XFC55
 * 18 0.6464  55.55   0XFB9A
 * 20 0.58177 50      0XFB1D
 * 25 0.4654  40      0XF9E4
*/
#ifndef MOTORS_H
#define MOTORS_H


#define RPM_radian_converter 2.327
#define Yaw_radian_multiplier 0.0020945
#define encoder_increment_rate 3

#define ENCA1  3 //19
#define ENCA2 5
#define ENCB1 2
#define ENCB2 10 //3

#define LEFT_PWM_MIN 45
#define RIGHT_PWM_MIN 70

#define in1 22
#define in2 23
#define enL 8

#define in3 25
#define in4 24                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
#define enR 13

extern volatile float left_encoder_count ;
extern volatile float right_encoder_count;
extern float encoder_set_point;
extern float velocity_set_point;

extern float pwm_right_offset ; 
extern float pwm_left_offset; 
extern float average_theta ;
extern float average_RPM ;
extern float v[2];
extern float u[2];


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

void update_encoder_states();

 /*************************************************************************************
 * Function Name: left_encoder_interrupt
 * Input  : no input
 * Output : no outputs. updates global left_encoder_count variable.
 * Logic  : handles interrupt for left encoder.  
 * 
 * Example Call: called automatically.
 *************************************************************************************/
 
void left_encoder_interrupt();

 /*************************************************************************************
 * Function Name: right_encoder_interrupt
 * Input  : no input
 * Output : no outputs. updates global right_encoder_count variable.
 * Logic  : handles interrupt for right motor encoder.  
 * 
 * Example Call: called automatically.
 *************************************************************************************/ 
 
void right_encoder_interrupt();

 /*************************************************************************************
 * Function Name: encoder_config
 * Input  : no input
 * Output : no outputs. 
 * Logic  : encoder wires are configured as interrupts 
 *          at rising and falling edges. 
 * 
 * Example Call: encoder_config()
 *************************************************************************************/


void encoder_config();

/************************************************************************************
 * Function Name: drive_motors
 * Input  : no inputs.
 * Output : no output.
 * Logic  : used to drive motors based on calculated pwm from lqr controller.
 *          The calculated voltages are constrained and mapped w.r.t corresponding 
 *          motor's offset.
 * Example Call: drive_motors(), called by update_motors().
 *************************************************************************************/
 

void drive_motors();

 /*************************************************************************************
 * Function Name: update_motors
 * Input  : no inputs.
 * Output : no outputs.
 * Logic  : calculates the required PWM using LQR gains. and calls 
 *          drive_motors function to control the motor. state space of the controller has 6 states , 
 *          which is divided into two independent controller.
 ****************************************************************************************          
            u[0] is the pitch and position controller output
            u[1] is the yaw controller output. yaw controller is not used because
            we are controlling manually through turns.  
            (v[0] + v[1]) affects only the dynamics of A and B and v[0] - v[1] 
            affects only the yaw dynamics. 
            Let, 𝑢[0] = v[0] + v[1], u[1] = v[0] - v[1]
            so, v[0] = (0.5(u[0] + u[1]))
                v[1] = (0.5(u[0] - u[1]))
 *****************************************************************************************                
 * Example Call: update_motors(). Called by update_encoder_states().
 *************************************************************************************/

void update_motors();
#endif
