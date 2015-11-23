#include <stdio.h>
#include <math.h>
#include <string.h>

#include "ms_move_to_goal.h"


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// This file contains everything related to the motorschema 'move_to_goal'.                       //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////





void get_move_to_goal_vector(float * direction, int robot_id) {
	int j;
	for (j=0;j<2;j++) {
		direction[j] = (migr[j]-unit_center[j]);
	}
	
	return;
}

