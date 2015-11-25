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





/*
 * Calculates a robot's vector that points to a direction which avoids obstacles.
 */
void get_stat_obst_avoidance_vector(float* direction, int robot_id, float lwb, float upb){

    // init direction
    direction[0] = 0;
    direction[1] = 0;
    float dir_norm;
    
    // read the current distance sensor values and save them in distances[]
    int i;
    for(i = 0; i < NB_SENSORS; i++){
        distances[i] = wb_distance_sensor_get_value(dist_sens[i]);
        direction[0] -= cos(sens_dir[i]) * distances[i];
        direction[1] -= sin(sens_dir[i]) * distances[i];
    }

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

        printf("avoid_obstacles[%d]: (%f,%f) -- %f\n", robot_id, direction[0], direction[1], dir_norm);
}
