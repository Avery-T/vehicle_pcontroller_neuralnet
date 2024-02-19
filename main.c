#include "globals.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "p_controller.h" 

//void p_controller(); 

int main(void) {
    init();  //initialize board hardware
    init_lcd(); 
    init_adc(); 
    motor(0, 0);
    motor(1, 0);

  //  p_controller();
	//	sensorTest(); 
    //p_controller(); 
  return 0;
}


