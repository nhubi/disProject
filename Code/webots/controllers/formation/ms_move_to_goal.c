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

void update_move_to_goal_vector(int robot_id, float speed[4][2], float MIGRATION_WEIGHT) {
	int j;
	for (j=0;j<2;j++) {
		speed[robot_id][j] += (migr[j]-loc[robot_id][j]) * MIGRATION_WEIGHT;
	}
	
	return;
}