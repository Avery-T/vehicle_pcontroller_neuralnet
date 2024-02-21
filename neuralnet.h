#ifndef __NERUAL_NET_H 
#define __NERUAL_NET_H 

#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "pound_defines.h"

typedef uint8_t u08;

typedef struct sensor_reading
{ 
  u08 left; 
  u08 right;
} sensor_reading; 

typedef struct __attribute__((packed)) normalized_data
{ 
  float left; 
  float right;
} normalized_data;

typedef struct hidden_neuron
{
    float w[HIDDEN_IN];
    float bias;
    float out;
} hidden_neuron; 

typedef struct out_neuron
{
    float w[OUT_IN];
    float bias;
    float out;
} out_neuron;
typedef struct neural_net
{
    out_neuron o_layer[NUM_OUT_NEURONS]; 
    hidden_neuron h_layer[NUM_HIDDEN_NEURONS]; 
} neural_net; 

motor_command compute_proportional(uint8_t left, uint8_t right); 

int32_t get_btn_seed();

void populate_dataset(sensor_reading* input, normalized_data* output);
void randomize_hidden_layer(hidden_neuron* h_layer);
void randomize_output_layer(out_neuron* o_layer);
normalized_data normalize_target(motor_command cmd);
void normalize_input(sensor_reading* input, normalized_data* norm_input);

void back_prop(normalized_data input, normalized_data actual, normalized_data target, hidden_neuron* h_layer, out_neuron* o_layer);
normalized_data inference(normalized_data sensors, hidden_neuron* h_layer, out_neuron* o_layer);
float activation_function(float x);

neural_net train_and_init(sensor_reading * sensor_data, uint16_t epochs);

//sensor_reading test_input[NUM_INPUT] = 
//{
//    {127, 127},
//    {127, 0},
//    {0, 127},
//    {0, 0},
//    {255, 255},
//    {250, 245}
//};

#endif 
