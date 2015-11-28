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

#define PERC_WINDOW_SIZE 3 // defines, over how many steps to average the sensor perceptions 



// initializations

// int whatever;



// methods

void get_stat_obst_avoidance_vector(float * direction, int robot_id, float lwb, float upb);


#endif
