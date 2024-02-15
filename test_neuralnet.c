#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

typedef uint8_t u08;

#define MAX_SPEED 100
#define DELAY 100
#define EPSILON 20
#define SCALE_DENOM 100
#define DESIRED 200
#define KP .15f
#define DEFAULT_SPEED 75
#define HIDDEN_IN 2
#define OUT_IN 3
#define NUM_INPUT 50
#define NUM_HIDDEN_NEURONS 3
#define NUM_OUT_NEURONS 2
#define RIGHT_SENSOR 4
#define LEFT_SENSOR 5
#define MATH_E 2.71828183f
#define LEARN_RATE 0.005f
#define EPOCHS 3
#define MIN_GRADIENT 0.00001f
#define MAX_GRADIENT 0.3


typedef struct motor_command
{ 
  int8_t left_motor_speed; 
  int8_t right_motor_speed; 
} motor_command; 

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

motor_command compute_proportional(uint8_t left, uint8_t right); 

int32_t get_btn_seed();

void populate_dataset(normalized_data* input, normalized_data* output);
void randomize_hidden_layer(hidden_neuron* h_layer);
void randomize_output_layer(out_neuron* o_layer);
normalized_data normalize_target(motor_command cmd);

void back_prop(normalized_data input, normalized_data actual, normalized_data target, hidden_neuron* h_layer, out_neuron* o_layer);
normalized_data inference(normalized_data sensors, hidden_neuron* h_layer, out_neuron* o_layer);
float activation_function(float x);


int main(void) {
    //init();  //initialize board hardware
    int32_t seed =1; //random
    srand(seed * seed % (RAND_MAX / 2));

    normalized_data input_data[NUM_INPUT];
    normalized_data target_data[NUM_INPUT];
    hidden_neuron h_layer[NUM_HIDDEN_NEURONS];
    out_neuron o_layer[NUM_OUT_NEURONS];
    
    populate_dataset(input_data, target_data);

    randomize_hidden_layer(h_layer);
    randomize_output_layer(o_layer);

    for (int i = 0; i < EPOCHS; i++) {
        for (int j = 0; j < NUM_INPUT; j++) {
            normalized_data actual = inference(input_data[j], h_layer, o_layer);
            back_prop(input_data[j], actual, target_data[j], h_layer, o_layer);
            printf("actual:(%f,%f) target:(%f,%f)\n", actual.left, actual.right, target_data[j].left, target_data[j].right);
            printf("h_layer - bias %f out %f weight1 %f, weight 2 %f\n", h_layer[j].bias ,h_layer[j].out,h_layer[j].w[0], h_layer[j].w[1]);
            printf("o_layer - bias %f out %f weight1 %f, weight 2 %f, weight 2 %f\n", o_layer[j].bias ,o_layer[j].out,o_layer[j].w[0], o_layer[j].w[1],o_layer[j].w[2]);
        }
        printf("EPDONE \n\n\n\n");
    }

    while(1);

    return 0;
}

void back_prop(normalized_data input, normalized_data actual, normalized_data target, hidden_neuron* h_layer, out_neuron* o_layer) {
    hidden_neuron h_layer_new[NUM_HIDDEN_NEURONS];
    out_neuron o_layer_new[NUM_OUT_NEURONS];

    for (int i = 0; i < NUM_HIDDEN_NEURONS; i++) {
        h_layer_new[i] = (hidden_neuron){{0}, 0, 0};
    }

       for (int i = 0; i < NUM_OUT_NEURONS; i++){
        o_layer_new[i] = (out_neuron){{0}, 0, 0};
    }

    for (int i = 0; i < NUM_OUT_NEURONS; i++) {
        float cur_out = *((float*)&actual + i);
        float cur_target = *((float*)&target + i);
        float d_err_d_out = (cur_out - cur_target) * (cur_out * (1.0 - cur_out));
        //gradient cliping, gradient can become so small it equals inf and that breaks the our nerualnet
        int sign_change = 0; 
        if(d_err_d_out < 0){
            d_err_d_out *=-1;
            sign_change = 1; 
        }  

        d_err_d_out = (d_err_d_out < MIN_GRADIENT) ?  MIN_GRADIENT : d_err_d_out;  
        d_err_d_out = (d_err_d_out > MAX_GRADIENT) ?  MAX_GRADIENT : d_err_d_out;  

        if(sign_change){
          d_err_d_out *=-1;
          sign_change = 0;
        }  

        printf("gradient %f\n", d_err_d_out);

        for (int j = 0; j < OUT_IN; j++) {
            (o_layer_new[i]).w[j] = (o_layer[i]).w[j] - LEARN_RATE * d_err_d_out * h_layer[j].out;
        }
         printf("h_layer %f\n", h_layer[0].out);
         printf("h_layer %f\n", h_layer[1].out);
         printf("h_layer %f\n", h_layer[2].out);
         printf("o_layer_new %f\n", (o_layer_new[i]).w[0] );
           printf("o_layer_new %f\n", (o_layer_new[i]).w[1]);
             printf("o_layer_new %f\n", (o_layer_new[i]).w[1] );


        o_layer_new[i].bias = o_layer[i].bias - LEARN_RATE * d_err_d_out * (-1.0);

        for (int j = 0; j < NUM_HIDDEN_NEURONS; j++) {
            for (int k = 0; k < HIDDEN_IN; k++) {
                (h_layer_new[j]).w[k] += (h_layer[j]).w[k] - LEARN_RATE * d_err_d_out * ((o_layer[i]).w[j]) * h_layer[j].out * (1.0 - h_layer[j].out) * *((float*)&input + k);
            }

            h_layer_new[j].bias += h_layer[j].bias - LEARN_RATE * d_err_d_out * ((o_layer[i]).w[j]) * h_layer[j].out * (1.0 - h_layer[j].out) * (-1.0);
        }
    }

    for (int i = 0; i < NUM_OUT_NEURONS; i++) {
        for (int j = 0; j < OUT_IN; j++) {
            (o_layer[i]).w[j] = (o_layer_new[i]).w[j];
        }

        o_layer[i].bias = o_layer_new[i].bias;
        o_layer[i].out = o_layer_new[i].out;
    }

    for (int i = 0; i < NUM_HIDDEN_NEURONS; i++) {
        for (int j = 0; j < HIDDEN_IN; j++) {
            (h_layer[i]).w[j] = (h_layer_new[i]).w[j];
        }

        h_layer[i].bias = h_layer_new[i].bias;
        h_layer[i].out = h_layer_new[i].out;
    }
}

normalized_data inference(normalized_data sensors, hidden_neuron* h_layer, out_neuron* o_layer) {
    normalized_data output = {0, 0};
    for (int i = 0; i < NUM_HIDDEN_NEURONS; i++) {
        h_layer[i].out = 0;
        h_layer[i].out += ((h_layer[i]).w[0]) *  (sensors.left); //
        h_layer[i].out += ((h_layer[i]).w[1] ) * (sensors.right); //wtf
       
        printf("sensor: (sensors.left %f,sensors.right %f)\n", sensors.left,sensors.right);


    //    for (int j = 0; j < HIDDEN_IN; j++) {
     //       h_layer[i].out += (h_layer[i]).w[j] * *((float*)&sensors + j); //wtf
     //       printf("h_layer out in inference:%f\n", h_layer[i].out);
     //   }

        
        h_layer[i].out -= h_layer[i].bias;
         printf("h_layer.oute pre act:%f\n", h_layer[i].out);
        h_layer[i].out = activation_function(h_layer[i].out);
        printf("h_layer.post act:%f\n", h_layer[i].out);
    }

    for (int i = 0; i < NUM_OUT_NEURONS; i++) {
        o_layer[i].out = 0;
        for (int j = 0; j < OUT_IN; j++) {
            o_layer[i].out += (o_layer[i]).w[j] * h_layer[j].out;
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

//int32_t get_btn_seed() {
//    int32_t counter = 0;
//    while (!get_btn()) {
//        _delay_ms(1);
//        counter++;
//    }

//    return counter;
//}

void populate_dataset(normalized_data* input_data, normalized_data* target_data) {
   u08 right  = 0; 
   u08 left = 0; 

    for (int i = 0; i < NUM_INPUT; i++) {
     left = (i % 2) ?  230 : 155; 
      right = (i % 2) ? 230 : 155; 
        target_data[i] = normalize_target(compute_proportional(left, right));
        input_data[i].left = (float)left / 255.0f;
        input_data[i].right = (float)right / 255.0f;
        //input_data[i].left = (float)left;
        //input_data[i].right = (float)right;
    }   
}

normalized_data normalize_target(motor_command cmd) {
    normalized_data data;
    //data.left = (float)cmd.left_motor_speed/100.0f;
    //data.right = (float)cmd.right_motor_speed/100.0f;
    data.left = (float)cmd.left_motor_speed/1000.0f;
    data.right = (float)cmd.right_motor_speed/1000.0f;

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
                           //100 * 1 or 100 * 0
    //val.left_motor_speed = 100 * left;
    //val.right_motor_speed = 100 * right; 
    val.left_motor_speed = right < DESIRED ? (-right + DESIRED) * KP : DEFAULT_SPEED;
    val.right_motor_speed = left < DESIRED ? (-left + DESIRED) * KP : DEFAULT_SPEED;

    return val;  
}


