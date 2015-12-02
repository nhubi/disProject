////////////////////////////////////////////////////////////////////////////////////////////////////
// This file contains all the implementation of the file fitness.h                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "fitness.h"

double compute_fitness(int FORMATION_SIZE) {
	double mean_v=0;
	double mean_delta_v=0;
	
	int i;
	for (i=0; i<FORMATION_SIZE; i++) {
		mean_v+=speed_sum[i][0];
		mean_delta_v+=speed_sum[i][1];
	}
	
	printf("%f\n",speed_sum[0][0]);
	printf("%f\n",speed_sum[1][0]);
	printf("%f\n",speed_sum[2][0]);
	printf("%f\n",speed_sum[3][0]);
	printf("%d\n",number_of_time_step);


	
	mean_v=mean_v/number_of_time_step/FORMATION_SIZE;
	mean_delta_v=mean_delta_v/number_of_time_step/FORMATION_SIZE;

	printf("%f\n",mean_v);
	printf("%f\n",mean_delta_v);
	
	return mean_v*(1-sqrt(mean_delta_v));
}

/*
 * Computes the speed of the robots (also the angular speed) given the last 2 positions
 */
void update_fitness_computation_for_robot(float loc[4][3],float prev_loc[4][3],float speed[4][3],int robot_id,float time_step) {
	// compute the speed
	compute_speed(loc,prev_loc,speed,robot_id,time_step);
	
	
	// compute the absolute value of the speed (speed_sum[robot_id][0])
	speed_sum[robot_id][0]+=sqrt(speed[robot_id][0]*speed[robot_id][0]+speed[robot_id][1]*speed[robot_id][1]);
	
	
	if (robot_id==0) {
//		printf("partial %f %f\n",speed[robot_id][0],speed[robot_id][1]);
//		printf("total speed %f\n",sqrt(speed[robot_id][0]*speed[robot_id][0]+speed[robot_id][1]*speed[robot_id][1]));
//		printf("sum %f\n",speed_sum[robot_id][0]	);
	}
	
	// compute the absolute value of the "turning speed"
	speed_sum[robot_id][1]+=fabs(speed[robot_id][2]);
	
	// update time_step
	if (robot_id==0) {
		number_of_time_step++;
	}
}


/*
 * Computes the speed of the robots (also the angular speed) given the last 2 positions
 */
void compute_speed(float loc[4][3],float prev_loc[4][3],float speed[4][3],int robot_id,float time_step) {
	int i;
	for (i=0; i<3; i++) {
		speed[robot_id][i]=(loc[robot_id][i]-prev_loc[robot_id][i])/time_step;
	}
}


/*
 * Called at the beginning of the simulation to reset it
 */
void reset_fitness_computation(int FORMATION_SIZE) {
	number_of_time_step=0;
	int i;
	for (i=0; i<FORMATION_SIZE; i++) {
		speed_sum[i][0]=0;
		speed_sum[i][1]=0;
	}
}
