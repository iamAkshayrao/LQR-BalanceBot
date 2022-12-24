


#include <Arduino.h>


#define MagF            51                     // electromagnet pin

#define buzz_pin        31                     // electromagnet pin

#define RED_pin         43                     //LED Pin
#define Common_pin      45                     //LED Pin
#define GREEN_pin       47                     //LED Pin
#define BLUE_pin        49                     //LED Pin


#define InL1            13                      // motor pin
#define PWML            10                      // PWM motor pin  
#define InL2            9                       // motor pin  

#define InR1            7                       // motor pin
#define PWMR            6                       // PWM motor pin
#define InR2            4                       // motor pin 

int buzz_count=0;

 
void motor_init(){
    pinMode(MagF, OUTPUT);
    
    pinMode(InL1, OUTPUT);
    pinMode(InL2, OUTPUT);
    pinMode(PWML, OUTPUT);
    
    pinMode(InR1, OUTPUT);
    pinMode(InR2, OUTPUT);
    pinMode(PWMR, OUTPUT);
}
void motorForwardL(int PWM_val)  {
    analogWrite(PWML, PWM_val);
    digitalWrite(InL1, LOW);
    digitalWrite(InL2, HIGH);
}

void motorForwardR(int PWM_val)  {
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, HIGH);
}

void LED_init(){
    pinMode(RED_pin, OUTPUT);
    pinMode(Common_pin, OUTPUT);
    pinMode(GREEN_pin, OUTPUT);
    pinMode(BLUE_pin, OUTPUT);

    digitalWrite(RED_pin, HIGH);
    digitalWrite(Common_pin, HIGH);
    digitalWrite(GREEN_pin, HIGH);
    digitalWrite(BLUE_pin, HIGH);
}

void BUZZ_init(){
    pinMode(buzz_pin, OUTPUT);
    
    digitalWrite(buzz_pin, HIGH);
}

void MAG_init(){
    pinMode(MagF, OUTPUT);
    
    digitalWrite(MagF, LOW);
}

void MagPick(void)  {
  digitalWrite(MagF, HIGH);
}

void MagDrop(void)  {
  digitalWrite(MagF, LOW);
}

void setup() {
    Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
    motor_init();
    Serial.println("motor_init_finish");
    
    LED_init();
    Serial.println("LED_init_finish");

    BUZZ_init();
    Serial.println("BUZZ_init_finish");
    
    MAG_init();
    Serial.println("MAG_init_finish");
    
    timer1_init();                           //for periodic buzzer beep
    Serial.println("timer1_init_finish");
}

void loop() 
{
  int k=1;
  while(k==1)
{
    /* increasing motor speed in forward direction */
    for(int i=0; i<250; i=i+50){           
        motorForwardR(i);
        motorForwardL(i);  
        delay(1000);    
    }   
    
    /* decreasing motor speed in forward direction */
    for(int i=250; i>=0; i=i-50){
        motorForwardR(i);
        motorForwardL(i);  
        delay(1000);    
    }

    /* blinking LED in RGB*/
    for(int i=1; i<=3; i++){
        if(i%3==0){
            digitalWrite(RED_pin, HIGH);
            digitalWrite(GREEN_pin, HIGH); 
            digitalWrite(BLUE_pin, LOW); 
            delay(1000);
        }
        else if(i%2==0){
            digitalWrite(RED_pin, HIGH);
            digitalWrite(GREEN_pin, LOW); 
            digitalWrite(BLUE_pin, HIGH); 
            delay(1000);
        }
        else{
            digitalWrite(RED_pin, LOW);
            digitalWrite(GREEN_pin, HIGH); 
            digitalWrite(BLUE_pin, HIGH);        
            delay(1000);
        }
    }

    
    digitalWrite(RED_pin, LOW);
    digitalWrite(GREEN_pin, LOW); 
    digitalWrite(BLUE_pin, LOW); 
    /* blinking LED in RGB*/
    for(int i=1; i<=11; i++){
        if(i%2==0){
            MagPick();
            delay(1000);
        }
        if(i%2!=0){
            MagDrop();        
            delay(1000);
        }
    }
    k=k+1;
    
    
}  
    delay(5000);
}


void timer1_init()
{
    cli(); //Clears the global interrupts
    TIMSK1 = 0x01; //timer1 overflow interrupt enable
    TCCR1B = 0x00; //stop
    TCNT1H = 0x0B; //Counter higher 8 bit value
    TCNT1L = 0xDB; //Counter lower 8 bit value
    TCCR1A = 0x00;
    TCCR1C = 0x00;
    TCCR1B = 0x04; //start Timer, prescaler 256
    sei();   //Enables the global interrupts
}
ISR (TIMER1_OVF_vect)
{
    buzz_count=buzz_count+1;
    TCNT1H = 0x0B; //Counter higher 8 bit value
    TCNT1L = 0xDB; //Counter lower 8 bit value
    if (buzz_count>=30 & buzz_count%2==0){
        digitalWrite(buzz_pin, HIGH);
    }
    if (buzz_count>=30 & buzz_count%2!=0){
        digitalWrite(buzz_pin, LOW);
    }
    
} 
