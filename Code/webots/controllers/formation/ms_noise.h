////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// This file contains the definitions related to the motorschema 'noise'.                         //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef MS_NOISE_H
#define MS_NOISE_H


#include "robot_state.h"
#include "utils.h"


// includes

//#include <???>



// definitions

//#define WHATEVER



// declarations

int noise_gen_frequency;  // defines, after how many steps a new random vector should be generated
bool fading;              // true, if nice transition is wished from one random vector to the next


// methods

void get_noise_vector(float* direction);


#endif
