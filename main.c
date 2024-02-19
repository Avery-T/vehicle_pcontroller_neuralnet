#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "p_controller.h" 
#include "neuralnet.h" 
#include "pound_defines.h"

//void p_controller(); 
void get_sensor_input(sensor_reading *);
void neural_net_p_controller(neural_net *);
int main(void) {
    sensor_reading sensor_data[NUM_INPUT]; 
    init();  //initialize board hardware
    init_lcd(); 
    init_adc(); 
    motor(0, 0);
    motor(1, 0);
    get_sensor_input(sensor_data); 
    clear_screen();
    neural_net net = train_and_init(sensor_data); 
    print_string("gotout"); 
    neural_net_p_controller(&net); 
  return 0;
}
void neural_net_p_controller(neural_net * net) 
{
   clear_screen(); 
   normalized_data motor_command; 
   normalized_data sensor_data;  
   while(1)
   { 
     //normalizing thte sensor data
     sensor_data.left = (analog(4))/255.0f; 
     sensor_data.right = (analog(5))/255.0f; 
     motor_command = inference(sensor_data, net->h_layer, net->o_layer); 
     motor(1, motor_command.left * 100); 
     motor(0, motor_command.right*100);  
   } 
  // print_num( net[0].o_layer->w[0]);
   // print_num( net[0].o_layer->out);
   // print_num( net[0].o_layer->bias);


}
void get_sensor_input(sensor_reading * sensor_data)
{
  lcd_cursor(0,0); 
  print_string("read"); 
  
	for(uint8_t i = 0; i < NUM_INPUT; i++) 
  { 
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

