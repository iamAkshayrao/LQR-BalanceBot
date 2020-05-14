#include "Arduino.h"
#include "timers.h" 
 
 /*************************************************************************************
 * Function Name: timer1_init
 * Input  : no inputs
 * Output : no outputs
 * Logic  : initiates timer 1
 * Example Call: timer1_init() called by setup().
 *************************************************************************************/

void timer1_init()
{
  TCCR1B = 0x00;    // Stop Timer
  TCNT1  = Timer_offset;
  OCR1A  = 0x0000;  // Output Compare Register (OCR) - Not used
  OCR1B  = 0x0000;  // Output Compare Register (OCR) - Not used
  OCR1C  = 0x0000;  // Output Compare Register (OCR) - Not used
  ICR1   = 0x0000;  // Input Capture Register (ICR)  - Not used
  TCCR1A = 0x00;
  TCCR1C = 0x00;
}


 /*************************************************************************************
 * Function Name: timer5_init
 * Input  : no inputs
 * Output : no outputs
 * Logic  : initiates timer 5
 * Example Call: timer5_init() called by setup().
 *************************************************************************************/

void timer5_init()
{
  TCCR5B = 0x00;    // Stop Timer
  TCNT5  = Timer_offset;
  OCR5A  = 0x0000;  // Output Compare Register (OCR) - Not used
  OCR5B  = 0x0000;  // Output Compare Register (OCR) - Not used
  OCR5C  = 0x0000;  // Output Compare Register (OCR) - Not used
  ICR5   = 0x0000;  // Input Capture Register (ICR)  - Not used
  TCCR5A = 0x00;
  TCCR5C = 0x00;
}

 /*************************************************************************************
 * Function Name: start_timer5
 * Input  : no inputs
 * Output : no outputs
 * Logic  : starts timer 5
 * Example Call: start_timer5() called by setup
 *************************************************************************************/

void start_timer5()
{
  TCCR5B = 0x04;    // Prescaler 256 1-0-0
  TIMSK5 = 0x01;    // Enable Timer Overflow Interrupt
}

 /*************************************************************************************
 * Function Name: start_timer1
 * Input  : no inputs
 * Output : no outputs
 * Logic  : starts timer 1
 * Example Call: start_timer1() called by setup
 *************************************************************************************/

void start_timer1()
{
  TCCR1B = 0x04;    // Prescaler 256 1-0-0
  TIMSK1 = 0x01;    // Enable Timer Overflow Interrupt
}
