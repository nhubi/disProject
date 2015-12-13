#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "ms_noise.h"


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// This file contains everything related to the motorschema 'noise'.                              //
// The motorschema is needed for solving situations, when a robot is e.g. stuck behind an         //
// obstacle. Thanks to the noise vector, it will be directed into a random direction away from    //
// the obstacle.                                                                                  //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////


// initialize the step counter
int step_counter  = 0;
float x_rand      = 0;
float z_rand      = 0;
float prev_x_rand = 0;
float prev_z_rand = 0;


// true if the random generator has been initialized
bool seed_set = false;



void get_noise_vector(float* direction) {

    if(step_counter%noise_gen_frequency == 0){
        step_counter = 0;
        if(fading){
            prev_x_rand = x_rand;
            prev_z_rand = z_rand;
        }

        if (seed_set == false)
        {
           srand(time(NULL)*robot_id);
           seed_set = true;
        }
        
        // generate random x and z in [-1, -1]
        x_rand = rand()/(float)RAND_MAX * 2 - 1;
        z_rand = rand()/(float)RAND_MAX * 2 - 1;
    }

    if(fading){
        // if fading, the current direction is a linear combination of current and previous noise.
        float factor = (float)(step_counter+1)/noise_gen_frequency;
        direction[0] = factor*x_rand + (1.0 - factor)*prev_x_rand;
        direction[1] = factor*z_rand + (1.0 - factor)*prev_z_rand;
    
    } else {
        direction[0] = x_rand;
        direction[1] = z_rand;
    }

    // normalize the vector s.t. the norm is in [0, 1]
    direction[0] /= sqrt(2.0);
    direction[1] /= sqrt(2.0);

    // increase the step_counter
    step_counter++;
    
	return;
	
}

