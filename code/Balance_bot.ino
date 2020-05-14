/*
 * Team Id: eYRC#9
 * Author List: Akshay S Rao, Aliasgar AV, Ankit Kumar, Mohammed Rehab Sait.
 * Filename: eYRC-BP#9_Task5_Code
 * Theme: Biped patrol (BP).
 * Functions: setup, loop, timer1_init, timer5_init, start_timer1, start_timer5, set_mpu_offset,
 *            scale_values, lowpassfilter, highpassfilter, complementary_filter, read_tilt_angle,
 *            update_encoder_states, update_motors, drive_motor, left_encoder_interrupt, right_encoder_interrupt
 *            encoder_config, read_switches, read_joystick, 
 *            
 * Global Variables: 
*/ 

#include "I2Cdev.h"
#include "MPU6050.h"
#include "timers.h"
#include "motors.h"
#include "IMU.h"



#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;
#define OUTPUT_READABLE_ACCELGYRO

byte analogMSB1;
byte analogLSB1;
byte analogMSB2;
byte analogLSB2;
byte digitalData,digitalData0;
byte discardByte;
int joyposVert;
int joyposHorz;

/* 
 *  LQR gain matrix for 
 * angular position | pitch | angular velocity | pitch _velocity
*/

float k[4] ={18.8 ,  -7746.3  , 40.1 ,  -70};//22 ,-7458.4,  40  ,-60.8};

//  gains for yaw | yaw_dot
float k1[2] = {-2,   -0.3};


ISR(TIMER1_OVF_vect)
{
TIMSK1 = 0x00;  
//Serial.println(millis());
read_tilt_angle();  
//Serial.println(millis());
update_encoder_states();
TCNT1 = Timer_offset;
TIMSK1 = 0x01;

}


ISR(TIMER5_OVF_vect)
{
TIMSK5 = 0x00; 

//update_encoder_states(); 
//Serial.println("ae"); 
 update_motors();

TCNT5 = Timer_offset;
TIMSK5 = 0x01;
}


 /*************************************************************************************
 * Function Name: read_tilt_angle
 * Input  : no inputs.
 * Output : no outputs. updates global variable pitch and pitch_dot.
 * Logic  : calls complementary filter and stores previous pitch angle 
 *          and updates current pitch angle and calculates pitch velocity(pitch_dot).
 *        
 * Example Call: read_tilt_angle().
 *************************************************************************************/ 
 
void read_tilt_angle()
{
 
  previous_pitch = pitch;
  pitch = complementary_filter();
  pitch_dot = (pitch - previous_pitch)*one_by_dT;//100;
  
}

 /*************************************************************************************
 * Function Name: update_motors
 * Input  : no inputs.
 * Output : no outputs.
 * Logic  : calculates the required PWM using LQR gains. and calls 
 *          drive_motors function to control the motor. state space of the controller has 6 states , 
 *          which is divided into two independent controller.
 ****************************************************************************************          
 *          continuous time model
 *           //  pitch and position controller                  // yaw controller
 *          A = [0.00000    0.00000    1.00000    0.00000;     A2 =[ 0.00000    1.00000;                                                              
                 0.00000    0.00000    0.00000    1.00000;           0.00000  -12.16150];
                 0.00000   60.72967   -7.83691    7.83691;
                 0.00000   64.59751    4.42974   -4.42974];    B2 =[0.00000;
            B = [0.00000;                                          -5.70172];
                 0.00000;
                 10.17473;
                -5.75117];
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


void update_motors()
{
 
  u[0] = ( k[0]*(encoder_set_point  - average_theta)
          +k[1]*(pitch_set_point-pitch) 
          +k[2]*(velocity_set_point-average_RPM)
          +k[3]*(pitch_dot_set_point-pitch_dot));
          
  u[1] = (k1[0]*yaw +k1[1]*yaw_dot) ;
  v[0] = (0.5 *(u[0] + u[1])); // for right motor
  v[1] = (0.5 *(u[0] - u[1])); // for left motor
  drive_motors();

 
}

 /*************************************************************************************
 * Function Name: read_switches
 * Input  : no inputs.
 * Output : no outputs.
 * Logic  : reads the data from switches present in joystick
 *       
 * Example Call: read_switches(). Called by read_joystick().
 
 *************************************************************************************/ 

void read_switches()
{
  if((bitRead(digitalData, 3)) == 0)
{  
 
  digitalWrite(51,HIGH); //To give control input to elecctromagnet 1.
}
else
{
  digitalWrite(51,LOW);
  
}
if((bitRead(digitalData, 6)) == 0)
{  
 
  digitalWrite(53,HIGH);  //To give control input to elecctromagnet 2.
}
else
{  
 
  digitalWrite(53,LOW);
}

//*******LED*********
if((bitRead(digitalData0,0))==0)
{ 
  digitalWrite(45,HIGH); // To control Green LED.
  digitalWrite(49,HIGH); // To control Red LED.
  digitalWrite(47,LOW); // To control Buzzer.
}
else
{
  digitalWrite(45,LOW); // To control Green LED.
  digitalWrite(49,LOW); // To control Red LED.
  digitalWrite(47,HIGH); // To control Buzzer.
}
}

 /*************************************************************************************
 * Function Name: read_joystick
 * Input  : no inputs.( only xbee inputs)
 * Output : no outputs.
 * Logic  : reads the joystick values through xbee
 *          and sets different modes of operation for the bot
 *       
 * Example Call: read_joystick(). Called by loop().
 *************************************************************************************/ 


void read_joystick()
{

if(Serial1.available()>=18)//Making sure that the Serial buffer has complete dataframe
{

while(Serial1.read()!=0x7E);//Checking for start byte


for(int i=1;i<=10;i++) //Discarding the next 11 bytes since it contains information like sender address, length

//analog and digital masks
discardByte  = Serial1.read();
digitalData0 = Serial1.read();
digitalData  = Serial1.read(); //Reading Digital input received.
analogMSB1   = Serial1.read(); //Reading Analog Data 1 which is 16 bit in hexadecimal containing 10 bits useful information from 2-axis joystick.
analogLSB1   = Serial1.read();
analogMSB2   = Serial1.read(); //Reading Analog Data 2 which is 16 bit in hexadecimal containing 10 bits useful information from 2-axis joystick.
analogLSB2   = Serial1.read();
discardByte  = Serial1.read(); 

//Conversion from hexadecimal to decimal
int analogReading2 = analogLSB2+(analogMSB2*256);
int analogReading1 = analogLSB1+(analogMSB1*256);

joyposVert = analogReading1;
joyposHorz = analogReading2;

pwm_right_offset = 0;
pwm_left_offset  = 0;


k[0] = 18.8;
k[3]= -70.8;

read_switches();

// bridge mode
if((bitRead(digitalData,4)) == 0)
{
  
//gains for bridge mode  
k[0] =  22;
k[3] = -100.2;

encoder_set_point  = average_theta;  // make position set point whatever the encoder count is now.
pitch_set_point   -= 0.002;         // vary pitch step wise
velocity_set_point = 6;            // increase velocity
pitch_set_point    = constrain(pitch_set_point, -0.048, 0 );

}

// break mode
else if((bitRead(digitalData,0))==0)
{
  
  left_encoder_count=0;     
  right_encoder_count=0;
  encoder_set_point = 0;
}

// hold
else if((joyposVert==1023) && (joyposHorz==1023))
{ 
  pitch_set_point = 0;
  velocity_set_point = 0;
}

// backward mode

else if (joyposVert > 350 && joyposHorz == 1023)
{     
  encoder_set_point -= 0.1*encoder_increment_rate;

}

// forward mode

else if (joyposVert >= 0 && joyposVert<500 && joyposHorz == 1023) //forward
{    
  encoder_set_point += 0.1*encoder_increment_rate;
}

// Turn modes

else if (joyposVert == 1023 && joyposHorz > 400)
{
  pwm_right_offset = -80;
  pwm_left_offset  = +80;

}
else if (joyposVert == 1023 && joyposHorz >=0)
{

pwm_right_offset = +80;
pwm_left_offset  = -80;

}

Serial1.flush();
  }

}

 /*************************************************************************************
 * Function Name:set_mpu_offset
 * Input  : no inputs.
 * Output : no outputs.
 * Logic  : sets the callibrated mpu offsets
 * Example Call: set_mpu_offset() ,called by setup
 *************************************************************************************/ 

void set_mpu_offset()
{

accelgyro.setXAccelOffset(-353);
accelgyro.setYAccelOffset(-937);
accelgyro.setZAccelOffset(1097);
accelgyro.setXGyroOffset(-138);
accelgyro.setYGyroOffset(-21);
accelgyro.setZGyroOffset(38);
}


 /**************************************************************************************
 * Function Name: setup.
 * Input : no input.
 * Output: no output
 * Logic : initialize all parameters and serial communication
 * Example Call: automatically called.
 **************************************************************************************/




void setup() 
{
cli();
sei();

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

// initialize serial communication

Serial.begin(19200);
Serial1.begin(9600);

accelgyro.initialize();
set_mpu_offset();
encoder_config();

encoder_set_point = 0;
pitch_set_point = 0;
encoder_set_point = 0;
velocity_set_point = 0;


timer1_init();
timer5_init();
start_timer1();
start_timer5();

// pin config for buzzer,led etc.
pinMode(53,OUTPUT);
pinMode(51,OUTPUT);
pinMode(49,OUTPUT);
pinMode(47,OUTPUT);
pinMode(45,OUTPUT);
}

 /*************************************************************************************
 * Function Name: loop
 * Input  : no inputs.
 * Output : no outputs.
 * Logic  : calls read_joystick repeatedly.
 *       
 * Example Call: automatically called.
 *************************************************************************************/ 
void loop()
{
accelgyro.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
read_joystick();
}
