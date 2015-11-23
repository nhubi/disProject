#include <stdio.h>
#include <math.h>
#include <string.h>

#include "ms_move_to_goal.h"


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// This file contains everything related to the motorschema 'move_to_goal'.                       //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////





/*
 * Calculates a robot's vector that points to a direction which avoids obstacles.
 */
void get_move_to_goal_vector(float* direction, int robot_id){
    // dummy direction
    direction[0] = 1;
    direction[1] = 0.0;
}

