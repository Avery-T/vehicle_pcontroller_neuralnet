#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "p_controller.h" 
#include "neuralnet.h" 
#include "pound_defines.h"


//void p_controller(); 
uint16_t set_epochs(); 
void get_sensor_input(sensor_reading *);
void neural_net_p_controller(neural_net *);
void stop_motors();
int main(void) {
    sensor_reading sensor_data[NUM_INPUT]; 
    uint16_t epochs = 0;
    init();  //initialize board hardware
    stop_motors(); 
    p_controller();
    stop_motors();
    get_sensor_input(sensor_data); 

    while(1){
    	epochs = set_epochs(); 
    	neural_net net = train_and_init(sensor_data, epochs); 
    	neural_net_p_controller(&net); 
   } 

    return 0;
}
void stop_motors()
{
	motor(0,0); 
  motor(1,0);
}
void neural_net_p_controller(neural_net * net) 
{
   clear_screen(); 
   normalized_data motor_command; 
   normalized_data sensor_data;  
   _delay_ms(400); 
   while(get_btn()==0)
   { 
     //normalizing thte sensor data
     sensor_data.right = (analog(4))/255.0f; 
     sensor_data.left = (analog(5))/255.0f; 
     motor_command = inference(sensor_data, net->h_layer, net->o_layer); 
     motor(0, motor_command.left * 100); 
     motor(1, motor_command.right * 100);  
   } 
  stop_motors(); 

}
uint16_t set_epochs() 
{
  
  uint16_t epocs = 0; 
  uint8_t counter = 0; 
  uint8_t accel_y = 0; 

  clear_screen(); 
  _delay_ms(400);
	while(get_btn() == 0) 
  {
     clear_screen(); 
     lcd_cursor(0,0);  
     print_string("training"); 
     lcd_cursor(0,1); 
     accel_y = get_accel_y(); 
     if(accel_y >200)
     { 
        counter+=1; 
     } 
	   print_num(counter); 
     _delay_ms(800); 
  } 
 
  epocs = counter * 50; 
 
 return epocs;   
}
void get_sensor_input(sensor_reading * sensor_data)
{
  _delay_ms(400); //btn debounce
  clear_screen(); 
  lcd_cursor(0,0); 
  print_string("Data"); 
  while(get_btn() == 0){ 
	for(uint8_t i = 0; i < NUM_INPUT; i++) 
  { 
    if(get_btn()) return; 
    lcd_cursor(5,0);  
    print_num(i); 
    lcd_cursor(0,1);
     _delay_ms(100); 
    sensor_data->left = analog(4); 
    sensor_data->right = analog(5); 
		print_num(sensor_data->left); 
    print_string(" "); 
    print_num(sensor_data->right);
  }
 }
} 

