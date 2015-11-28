#include <stdio.h>
#include <math.h>
#include <string.h>

#include "ms_avoid_static_obstacles.h"


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// This file contains everything related to the motorschema 'avoid_static_obstacles'.              //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////


int distances[NB_SENSORS];      // contains the values of each sensor
int perc_pointer           = 0;
int perc_window[NB_SENSORS][PERC_WINDOW_SIZE];  // the last measured perceptions





/*
 * Calculates a robot's vector that points to a direction which avoids obstacles.
 */
void get_stat_obst_avoidance_vector(float* direction, int robot_id, float lwb, float upb){

    // init direction
    direction[0] = 0;
    direction[1] = 0;

    float dir_norm;

    // wrap perc_pointer
    perc_pointer %= PERC_WINDOW_SIZE;
    
    // read the current distance sensor values and save them in distances[]
    int i, k;
    for(i = 0; i < NB_SENSORS; i++){
        perc_window[i][perc_pointer] = wb_distance_sensor_get_value(dist_sens[i]);
        
        // compute average over perception window
        for(k = 0; k < PERC_WINDOW_SIZE; k++){
            distances[i] += perc_window[i][k];
        }
        distances[i] /= PERC_WINDOW_SIZE;

        // compute direction (pointing in sensor's opposite direction)
        // --> see explanation about sensor orientation in robot_state.c
        direction[0] -= cos(sens_dir[i]) * distances[i];    // x-direction
        direction[1] -= sin(sens_dir[i]) * distances[i];    // z-direction
    }

    perc_pointer++;

    if(direction[0] != 0 || direction[1] != 0){
        dir_norm = norm(direction, 2);
        if(dir_norm > upb){
            normalize(direction, direction, 2);
            //dir_norm = norm(direction, 2);

        } else if(dir_norm < lwb){
            direction[0] = 0;
            direction[1] = 0;
            //dir_norm = norm(direction, 2);

        } else {
            direction[0] = (direction[0] - lwb) / (upb - lwb);
            direction[1] = (direction[1] - lwb) / (upb - lwb);
            //dir_norm = norm(direction, 2);
        }
    } else {dir_norm = 0;}
    if(robot_id == 0)
        printf("%s: avoid_obstacles (%2.4f,%2.4f) -- %4.4f -- <%d|%d|%d|%d|%d|%d>\n", robot_name, direction[0], direction[1], dir_norm,distances[5], distances[6], distances[7], distances[0], distances[1], distances[2]);
}
