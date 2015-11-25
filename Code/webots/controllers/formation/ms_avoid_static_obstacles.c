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
void get_stat_obst_avoidance_vector(float* direction, int robot_id){

    // init direction
    direction[0] = 0;
    direction[1] = 0;
    
    // read the current distance sensor values and save them in distances[]
    int i;
    for(i = 0; i < NB_SENSORS; i++){
        distances[i] = wb_distance_sensor_get_value(dist_sens[i]);
        if(distances[i] > MIN_SENS){
            direction[0] += - sin(sens_dir[i]) * distances[i];
            direction[1] += - cos(sens_dir[i]) * distances[i];
        }
    }

    if(direction[0] != 0 || direction[1] != 0){
        normalize(direction, direction, 2);
    }

    if(robot_id == 0)
        printf("avoid_obstacles[%d]: (%f,%f)\n", robot_id, direction[0], direction[1]);
}
