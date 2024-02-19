#ifndef P_CONTROLLER_H
#define P_CONTROLLER_H 
//#include "globals.h"
//#include <util/delay.h>
//#include <avr/io.h>
//#include <avr/interrupt.h>

//#define MAX_SPEED 100
//#define DELAY 100
//#define EPSILON 20
//#define SCALE_DENOM 100
//#define DESIRED 200
//#define KP .025f 
//#define MAX_TURN_SNIFF 300
//#define TURN_SPEED 35
#include "pound_defines.h"

motor_command compute_proportional(uint8_t left, uint8_t right); 
void motor(u08 num, int8_t speed); 
void test_motor_with_accel();
void print_signed_num(int16_t num);
void range_motors();
u08 turn_l();
u08 turn_r();
uint8_t calibrate_brightness(uint8_t pin); 
void sensorTest(); 
void p_controller(); 

#endif // P_CONTROLLER_H
