#include <stdio.h>
#include <math.h>
#include <string.h>

#include "ms_keep_formation.h"


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// This file contains everything related to the motorschema 'keep_formation'.                     //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////






/*
 * Calculates a robot's vector that points to a direction which avoids obstacles.
 */
void get_keep_formation_vector(float* direction, int robot_id){
    direction[0] = 0.0;
    direction[1] = 0.0;
}
