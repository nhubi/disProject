////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// This file contains the definitions related to the motorschema 'avoid_static_obstacles'.         //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef MS_AV_STAT_OB_H
#define MS_AV_STAT_OB_H



// includes

#include "robot_state.h"
#include "utils.h"



// definitions

#define PERC_WINDOW_SIZE 2 // defines, over how many steps to average the sensor perceptions 



// declarations

float sens_back_weight;
float avoid_obst_min_threshold;
float avoid_obst_max_threshold;



// methods

void get_stat_obst_avoidance_vector(float * direction);


#endif
