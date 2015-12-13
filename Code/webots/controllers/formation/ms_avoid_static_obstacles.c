#include <stdio.h>
#include <math.h>
#include <string.h>

#include "ms_avoid_static_obstacles.h"


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// This file contains everything related to the motorschema 'avoid_static_obstacles'.             //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////


float distances[NB_SENSORS];                      // contains the values of each sensor
int perc_pointer = 0;
float perc_window[NB_SENSORS][PERC_WINDOW_SIZE];  // the last measured perceptions





/*
 * Calculates a robot's vector that points to a direction which avoids obstacles.
 */
void get_stat_obst_avoidance_vector(float* direction){

    // init direction (robot coordinates)
    float x_rob  = 0;
    float z_rob  = 0;

    // init direction (absolute coordinates)
    direction[0] = 0;
    direction[1] = 0;

    // the total weight of the accullulated sensor perception values
    float acc_weight;

    // the norm of the non-normalized direction vector 
    float dir_norm;

    // sens_back_weight defines, how much sensor 3 and 4 should be weighted in z direction. 
    float sens_back_weight = -((sin(sens_dir[0])+sin(sens_dir[1])) / sin(sens_dir[3]));

    // lower and upper bound (for not needing to use the long variable name)
    float lwb = avoid_obst_min_threshold;
    float upb = avoid_obst_max_threshold;



    // wrap perc_pointer
    perc_pointer %= PERC_WINDOW_SIZE;
    
    // read the current distance sensor values and save them in distances[]
    int i, k;
    for(i = 0; i < NB_SENSORS; i++){
        perc_window[i][perc_pointer] = wb_distance_sensor_get_value(dist_sens[i]);
        
        // compute average over perception window
        distances[i] = 0;
        acc_weight   = 0;
        for(k = 0; k < PERC_WINDOW_SIZE; k++){
            distances[i] += perc_window[i][k];
            acc_weight++;
            
            // give a higher weight to higher perception values
            if(perc_window[i][k] > 100){
                distances[i] += perc_window[i][k];
                acc_weight++;
            }
        }
        distances[i] /= acc_weight;

        // compute direction (pointing in sensor's opposite direction)
        // --> see explanation about sensor orientation in robot_state.c
        x_rob -= cos(sens_dir[i]) * distances[i];    // x-direction

        // since there are twice as many sensors in the front compared to the back, give higher
        // weight to sensors 3 and 4 in z-direction.
        if(i == 3 || i == 4){
            z_rob += sin(sens_dir[i]) * distances[i] * sens_back_weight;

        } else {
            z_rob += sin(sens_dir[i]) * distances[i];
        }
    }

    perc_pointer++;

    
    // convert to absolute coordinates
    direction[0] = x_rob*cosf(loc[robot_id][2]) + z_rob*sinf(loc[robot_id][2]);
    direction[1] = z_rob*cosf(loc[robot_id][2]) - x_rob*sinf(loc[robot_id][2]);
    

    // zoning
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
            normalize(direction, direction, 2);
            direction[0] = direction[0]*(dir_norm - lwb)/(upb-lwb);
            direction[1] = direction[1]*(dir_norm - lwb)/(upb-lwb);
            //dir_norm = norm(direction, 2);
        }
    } else {dir_norm = 0;}
}

void get_stat_obst_avoidance_vector_supervisor(float * direction) {
    direction[0]=0;
    direction[1]=0;
    float distance_vector[2];
    float distance;
    
    int i;
    for (i=0;i<6;i++) {
        distance_vector[0]=loc[robot_id][0]-obstacle_loc[i][0];
        distance_vector[1]=loc[robot_id][1]-obstacle_loc[i][1];
        distance=norm(distance_vector,2);
        normalize(distance_vector,distance_vector,2);
        if (distance<avoid_obst_min_threshold) {
            
        } else if (distance>avoid_obst_max_threshold) {
            distance_vector[0]=0;
            distance_vector[1]=0;
        } else {
            float factor=(avoid_obst_max_threshold-distance)/
                    (avoid_obst_max_threshold-avoid_obst_min_threshold);
            multiply_vector_by(distance_vector,2,factor);
        }
        direction[0]+=distance_vector[0];
        direction[1]+=distance_vector[1];
    }
    

}





