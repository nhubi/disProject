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
//float * get_move_to_goal_vector(int robot_id) {
//    float vector[3] = {0,0,0};
//    return vector;
//}

void get_move_to_goal_vector(float * direction, int robot_id) {
    

	int j;
	for (j=0;j<2;j++) {
		direction[j] = (migr[j]-unit_center[j]);
	}
	
	return;
}
