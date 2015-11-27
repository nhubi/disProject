////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// This file contains the definitions related to the motorschema 'keep_formation'.                //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef MS_KEEP_FORMATION_H
#define MS_KEEP_FORMATION_H



// includes
#include "robot_state.h"



// definitions

//#define WHATEVER



// initializations
const float coef = 1.0; // Coefficient that determines the distance between the robots,
                     // to be changed depending on the robots' size. 


// methods
void get_keep_formation_vector(float* direction, int robot_id, int formation_type);
void get_formation_coordinates(float* coordinates, int robot_id, int formation_type);

#endif
