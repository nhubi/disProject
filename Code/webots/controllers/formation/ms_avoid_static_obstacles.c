#include <stdio.h>
#include <math.h>
#include <string.h>

#include "ms_avoid_static_obstacles.h"


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// This file contains everything related to the motorschema 'avoid_static_obstacles'.              //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////





/*
 * Calculates a robot's vector that points to a direction which avoids obstacles.
 */
void get_stat_obst_avoidance_vector(float* direction, int robot_id){
    // dummy direction
    direction[0] = 0;
    direction[1] = 1;
}
