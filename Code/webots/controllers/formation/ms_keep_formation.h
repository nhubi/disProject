////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// This file contains the definitions related to the motorschema 'keep_formation'.                //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef MS_KEEP_FORMATION_H
#define MS_KEEP_FORMATION_H


// includes
#include "robot_state.h"
#include "utils.h"



// declarations
extern const float robot_dist;      // Coefficient that determines the distance between the robots,
                                    // to be changed depending on the robots' size. 
float keep_formation_min_threshold; // < min threshold: dead zone, no correction of direction
float keep_formation_max_threshold; // > max threshold: ballistic zone, full correction of direction



// methods
void get_keep_formation_vector(float* direction, float* dir_goal);

#endif
