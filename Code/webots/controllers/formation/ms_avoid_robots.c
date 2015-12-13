#include <stdio.h>
#include <math.h>
#include <string.h>

#include "ms_avoid_robots.h"


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// This file contains everything related to the motorschema 'avoid_robots'.                       //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////





void get_avoid_robot_vector(float* direction) {
	int i;
	float distance_matrix[FORMATION_SIZE][2];
	float weight_vector[FORMATION_SIZE]; // It's initialized to 0 next
	
	for (i=0; i<FORMATION_SIZE; i++) {
		if (i==robot_id) {
			weight_vector[i]=0;
			continue;
		}
		
		float robot_i_vector_distance[2];
		difference(loc[i],loc[robot_id],robot_i_vector_distance,2);
		float robot_i_distance=norm(robot_i_vector_distance,2);
		
		// direction_norm > MAX_THRESHOLD
		if (robot_i_distance>avoid_robot_max_threshold) {
			weight_vector[i]=0;
		} else {
			normalize(robot_i_vector_distance,robot_i_vector_distance,2);
			distance_matrix[i][0]=robot_i_vector_distance[0];
			distance_matrix[i][1]=robot_i_vector_distance[1];
			
			// robot_i_distance < MIN_THRESHOLD
			if (robot_i_distance<avoid_robot_min_threshold) {
				weight_vector[i]=1;
				return;
			} else {
				// robot_i_distance in the middle
				float multiplyer_factor=(avoid_robot_max_threshold-robot_i_distance)/(avoid_robot_max_threshold-avoid_robot_min_threshold);
				weight_vector[i]=multiplyer_factor;
			}
		}

	}

	
	// normalise weight_vector (if sum<1 continue, else normalise to sum 1)
	float weight_vector_sum=0;
	for (i=0; i<FORMATION_SIZE; i++) {
		weight_vector_sum+=weight_vector[i];
	}
	if (weight_vector_sum>1) {
		for (i=0; i<FORMATION_SIZE; i++) {
			weight_vector[i]/=weight_vector_sum;
		}
	}
	
	
	// Get the direction
	for (i=0; i<FORMATION_SIZE; i++) {
		// skip the case with weight_vector[i]=0
		if (weight_vector[i]<1e-10) {
			continue;
		}
		
		direction[0]-=(weight_vector[i]*distance_matrix[i][0]);
		direction[1]-=(weight_vector[i]*distance_matrix[i][1]);
	}
	return;
	
}

