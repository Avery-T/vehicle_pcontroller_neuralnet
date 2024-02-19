#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "p_controller.h"

//typedef struct motor_command
//{ 
//j  int8_t left_motor_speed; 
//  int8_t right_motor_speed; 
//} motor_command; 


//motor_command compute_proportional(uint8_t left, uint8_t right)
//{
//  motor_command val = {0}; 

 // if(right  < DESIRED  ){
//     val.left_motor_speed = (-right +  DESIRED) * KP;
//  } 

//  else {
//    val.left_motor_speed =  75;
//  } 
 
//  if(left < DESIRED){
//       val.right_motor_speed = (-left +  DESIRED) * KP;
//  } 
//   else {
//      val.right_motor_speed =  75; 
//  }
// return val;  
//} 

//void 90_degree_turn(uint8_t turn_direction); 
void p_controller()
{
   int16_t error = 0; 
   
   uint8_t right_turn_flag = 0; 
    uint8_t left_turn_flag = 0; 
   

    uint8_t white_brightness = 195; 
   //calibrate left and right sensor 
   //calibrated black sensor 
  uint8_t output_right_sensor = analog(4); 
  uint8_t output_left_sensor  = analog(5); 

 uint8_t pinsel0 =0;
 uint8_t pinsel1 = 1; 
 u08 turn_res = 0;
 while(!get_btn());   
 _delay_ms(400);

 while(get_btn() == 0){
       
      output_right_sensor = analog(4);  /// right side 
      output_left_sensor = analog(5); //left side 
     //spin the left motor to go to the right 
     // less than is the white bir
     if(output_right_sensor  < DESIRED  ){
       motor(0, ((-output_right_sensor +  DESIRED) * KP));
     } 

     else {
      motor(0, 75);
    } 
     if(output_left_sensor  < DESIRED  ){
       motor(1, (((-output_left_sensor +  DESIRED) * KP)));
     } 
    else 
    {
       motor(1,75); 
    }

    if (turn_res) {
     turn_l();
     turn_res = 0;
   }
   else if(output_right_sensor < 175 || output_left_sensor < 175)
   { 
     turn_res = turn_r(); 
   }

     _delay_ms(100);
    lcd_cursor(0,0); 
     print_num(output_right_sensor); 
     lcd_cursor(0,1); 
     print_num(output_left_sensor);  
  } 
}

u08 turn_l() {
  u08 sensor_r, sensor_l; 
 
  while(sensor_l < DESIRED || sensor_r < DESIRED) { 
    motor(0, -TURN_SPEED);
    motor(1, TURN_SPEED);
    sensor_r = analog(4);
    sensor_l = analog(5); 
  }
}

u08 turn_r()
{
  u08 sensor_r, sensor_l; 
 
  for(uint16_t i = 0; i<MAX_TURN_SNIFF; i++) 
  {
    clear_screen();
    lcd_cursor(0, 0);
    print_string("left"); 
    motor(0, TURN_SPEED);
    motor(1, -TURN_SPEED);
    sensor_r = analog(4);
    sensor_l = analog(5);
    
    if(sensor_l > DESIRED && sensor_r > DESIRED) { 
      return 0; 
    } 
  } 
  
  while(sensor_l < DESIRED || sensor_r < DESIRED) {
    motor(0, -TURN_SPEED);
    motor(1, TURN_SPEED);
    sensor_r = analog(4);
    sensor_l = analog(5);
  }

  for(u16 i = 0; i < 0xFF; i++) {
    motor(0, TURN_SPEED);
    motor(1, TURN_SPEED);
  }

  return 1;
}



void sensorTest()
{ 
   lcd_cursor(0,0); 
 
  uint8_t output0 = analog(5); 
  uint8_t output1 = analog(4);

  while(1)
  { 
     lcd_cursor(0,0); 
     output0 = analog(5); 
     output1 = analog(4); 
     print_num(output0); 
     lcd_cursor(0,1); 
     print_num(output1); 
     _delay_ms(100);
     clear_screen();
  }

} 

uint8_t calibrate_brightness(uint8_t pin) 
{
uint16_t temp = 0; 
  // calibration
  for(int i = 0; i<20; i++) 
	{
     temp+=analog(pin); 
  }
  
  return temp/20; 

} 
  

void motor(u08 num, int8_t speed) {
   //u16 mapped_speed = (u16)(speed + 100) * 255 / 200;
   u16 mapped_speed = ((u16)speed * EPSILON + 127 * SCALE_DENOM) / SCALE_DENOM;
   set_servo(num, num ? -mapped_speed : mapped_speed);
}

void test_motor_with_accel() {
   int8_t cur_speed = 0;
   clear_screen();
   u08 y_accel = get_accel_y() % 255;

   if (get_btn()) {
      cur_speed = 0;
   }

   else if (y_accel <= 245 && y_accel >= 215) {
      cur_speed += 1;
      if (cur_speed > MAX_SPEED) {
         cur_speed = MAX_SPEED;
      }
   }

   else if (y_accel >= 10 && y_accel <= 40) {
      cur_speed -= 1;
      if (cur_speed < -MAX_SPEED) {
         cur_speed = -MAX_SPEED;
      }
   }
   
   lcd_cursor(0, 0);
   if (cur_speed < 0) {
      print_string("-");
      lcd_cursor(1, 0);
      print_num(-1 * cur_speed);
   }
   else print_num(cur_speed);

   lcd_cursor(0, 1);
   print_num(y_accel);

   motor(0, cur_speed);
   motor(1, cur_speed);
   _delay_ms(250);
}

void range_motors() {
   for (int8_t i = 0; i <= 100; i++) {
      motor(0, i);
      motor(1, i);
      print_signed_num(i);
      _delay_ms(DELAY);
   }
   for (int8_t i = 100; i >= 0; i--) {
      motor(0, i);
      motor(1, i);
      print_signed_num(i);
      _delay_ms(DELAY);
   }
   for (int8_t i = 0; i >= -100; i--) {
      motor(0, i);
      motor(1, i);
      print_signed_num(i);
      _delay_ms(DELAY);
   }
   for (int8_t i = -100; i <= 0; i++) {
      motor(0, i);
      motor(1, i);
      print_signed_num(i);
      _delay_ms(DELAY);
   }
}

void print_signed_num(int16_t num) {
   clear_screen();
   lcd_cursor(0, 0);
   if (num < 0) {
      print_string("-");
      lcd_cursor(1, 0);
      print_num(-1 * num);
   }
   else print_num(num);
}
