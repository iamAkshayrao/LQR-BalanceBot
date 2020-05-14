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
#ifndef TIMERS_H
#define TIMERS_H

#define Timer_offset 0xFEC6

 /*************************************************************************************
 * Function Name: timer1_init
 * Input  : no inputs
 * Output : no outputs
 * Logic  : initiates timer 1
 * Example Call: timer1_init() called by setup().
 *************************************************************************************/ 
 
void timer1_init();

 /*************************************************************************************
 * Function Name: timer5_init
 * Input  : no inputs
 * Output : no outputs
 * Logic  : initiates timer 5
 * Example Call: timer5_init() called by setup().
 *************************************************************************************/ 
 
void timer5_init();

 /*************************************************************************************
 * Function Name: start_timer5
 * Input  : no inputs
 * Output : no outputs
 * Logic  : starts timer 5
 * Example Call: start_timer5() called by setup
 *************************************************************************************/
 
void start_timer5();

 /*************************************************************************************
 * Function Name: start_timer1
 * Input  : no inputs
 * Output : no outputs
 * Logic  : starts timer 1
 * Example Call: start_timer1() called by setup
 *************************************************************************************/
 
void start_timer1();

#endif