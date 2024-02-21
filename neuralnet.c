#include <stdlib.h>
#include "globals.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include  <avr/interrupt.h>
#include <avr/io.h> 
#include "neuralnet.h"
#include "pound_defines.h"


neural_net train_and_init (sensor_reading * sensor_data, uint16_t epochs) {
    neural_net net; 
    //init();  //initialize board hardware
    int32_t seed =1; //random
    srand(seed * seed % (RAND_MAX / 2));

    normalized_data input_data[NUM_INPUT];
    normalized_data target_data[NUM_INPUT];
   // hidden_neuron h_layer[NUM_HIDDEN_NEURONS];
  //  out_neuron o_layer[NUM_OUT_NEURONS];
    
    normalize_input(sensor_data, input_data);
    populate_dataset(sensor_data, target_data);

    randomize_hidden_layer(net.h_layer);
    randomize_output_layer(net.o_layer);

    
    for (int i = 0; i < epochs; i++) {
      
        for (int j = 0; j < NUM_INPUT; j++) {
            normalized_data actual = inference(input_data[j], net.h_layer, net.o_layer);
            back_prop(input_data[j], actual, target_data[j], net.h_layer, net.o_layer);
        }
      lcd_cursor(0,1); 
      print_string("epoc ");
      print_num(i); 
    }
    return net;
}

void back_prop(normalized_data input, normalized_data actual, normalized_data target, hidden_neuron* h_layer, out_neuron* o_layer) {
    hidden_neuron h_layer_new[NUM_HIDDEN_NEURONS];
    out_neuron o_layer_new[NUM_OUT_NEURONS];

    for (int i = 0; i < NUM_HIDDEN_NEURONS; i++) {
        h_layer_new[i] = (hidden_neuron){0};
    }

    for (int i = 0; i < NUM_OUT_NEURONS; i++) {
        float cur_out = *((float*)&actual + i);
        float cur_target = *((float*)&target + i);
        float d_err_d_net = (cur_out - cur_target) * (cur_out * (1 - cur_out));

        for (int j = 0; j < OUT_IN; j++) {
            o_layer_new[i].w[j] = o_layer[i].w[j] - LEARN_RATE * d_err_d_net * h_layer[j].out;
        }

        o_layer_new[i].bias = o_layer[i].bias - LEARN_RATE * d_err_d_net * -1.0f;

        for (int j = 0; j < NUM_HIDDEN_NEURONS; j++) {
            float d_neto_d_neth = o_layer[i].w[j] * h_layer[j].out * (1.0f - h_layer[j].out);

            for (int k = 0; k < HIDDEN_IN; k++) {
                h_layer_new[j].w[k] += 0.5f * h_layer[j].w[k] - LEARN_RATE * d_err_d_net * d_neto_d_neth * *((float*)&input + k);
            }

            h_layer_new[j].bias += 0.5f * h_layer[j].bias - LEARN_RATE * d_err_d_net * d_neto_d_neth * -1.0f;
        }
    }

    for (int i = 0; i < NUM_OUT_NEURONS; i++) {
        for (int j = 0; j < OUT_IN; j++) {
            o_layer[i].w[j] = o_layer_new[i].w[j];
        }

        o_layer[i].bias = o_layer_new[i].bias;
    }

    for (int i = 0; i < NUM_HIDDEN_NEURONS; i++) {
        for (int j = 0; j < HIDDEN_IN; j++) {
            h_layer[i].w[j] = h_layer_new[i].w[j];
        }

        h_layer[i].bias = h_layer_new[i].bias;
    }
}

normalized_data inference(normalized_data sensors, hidden_neuron* h_layer, out_neuron* o_layer) {
    normalized_data output = {0, 0};
    for (int i = 0; i < NUM_HIDDEN_NEURONS; i++) {
        h_layer[i].out = 0;
        for (int j = 0; j < HIDDEN_IN; j++) {
            h_layer[i].out += h_layer[i].w[j] * *((float*)&sensors + j);
        }
        h_layer[i].out -= h_layer[i].bias;

        h_layer[i].out = activation_function(h_layer[i].out);
    }

    for (int i = 0; i < NUM_OUT_NEURONS; i++) {
        o_layer[i].out = 0;
        for (int j = 0; j < OUT_IN; j++) {
            o_layer[i].out += o_layer[i].w[j] * h_layer[j].out;
        }
        o_layer[i].out -= o_layer[i].bias;

        o_layer[i].out = activation_function(o_layer[i].out);
    }

    output.left = o_layer[0].out;
    output.right = o_layer[1].out;
    return output;
}

float activation_function(float x) {
    return 1/(1 + powf(MATH_E, -x));
}

int32_t get_btn_seed() {
    int32_t counter = 0;
    // while (!get_btn()) {
    //     _delay_ms(1);
    //     counter++;
    // }
    // return counter;
}

void populate_dataset(sensor_reading* input_data, normalized_data* target_data) {
    for (int i = 0; i < NUM_INPUT; i++) {
        target_data[i] = normalize_target(compute_proportional(input_data[i].left, input_data[i].right));
    }
}

void normalize_input(sensor_reading* input_data, normalized_data* norm_input_data) {
    for (int i = 0; i < NUM_INPUT; i++) {
        norm_input_data[i].left = (float)input_data[i].left / 255.0f;
        norm_input_data[i].right = (float)input_data[i].right / 255.0f;
    }   
}

normalized_data normalize_target(motor_command cmd) {
    normalized_data data;
    data.left = (float)cmd.left_motor_speed/100.0f;
    data.right = (float)cmd.right_motor_speed/100.0f;

    return data;
}

void randomize_hidden_layer(hidden_neuron* h_layer) {
    for (int i = 0; i < NUM_HIDDEN_NEURONS; i++) {
        for (int j = 0; j < HIDDEN_IN; j++) {
            (h_layer[i]).w[j] = (float)rand() / RAND_MAX;
        }
        h_layer[i].bias = (float)rand() / RAND_MAX;
    }
}

void randomize_output_layer(out_neuron* o_layer) {
    for (int i = 0; i < NUM_OUT_NEURONS; i++) {
        for (int j = 0; j < OUT_IN; j++) {
            (o_layer[i]).w[j] = (float)rand() / RAND_MAX;
        }
        o_layer[i].bias = (float)rand() / RAND_MAX;
    }
}

motor_command compute_proportional(uint8_t left, uint8_t right)
{
    motor_command val = {0,0};

 //   val.left_motor_speed = left * 100.0f / 255.0f;
 //   val.right_motor_speed = right * 100.0f / 255.0f;
    
    val.left_motor_speed = right < DESIRED ? (-right + DESIRED) * (1) * KP : DEFAULT_SPEED;
    val.right_motor_speed = left < DESIRED ? (-left + DESIRED) * (1) * KP : DEFAULT_SPEED;

    return val;  
}


