#ifndef POUND_DEFINES_H
#define POUND_DEFINES_H

#define MAX_SPEED 75
#define DELAY 100
#define EPSILON 20
#define SCALE_DENOM 100
#define DESIRED 210 
//#define KP .025f 
#define MAX_TURN_SNIFF 300
#define TURN_SPEED 35
#define DELAY 100
#define EPSILON 20
#define SCALE_DENOM 100
#define KP .3f
#define DEFAULT_SPEED 30
#define HIDDEN_IN 2
#define OUT_IN 3
#define NUM_INPUT 50
#define NUM_HIDDEN_NEURONS 3
#define NUM_OUT_NEURONS 2
#define RIGHT_SENSOR 4
#define LEFT_SENSOR 5
#define MATH_E 2.71828183f
#define LEARN_RATE 0.15f
#define EPOCHS 50

typedef struct motor_command{
 int8_t left_motor_speed; 
 int8_t right_motor_speed;
}motor_command;

#endif 
