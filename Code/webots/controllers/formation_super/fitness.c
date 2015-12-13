////////////////////////////////////////////////////////////////////////////////////////////////////
// This file contains all the implementation of the file fitness.h                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "fitness.h"

/* 
 * Pass to the function 1 if goal is reached, 0 if it's not reached
 */
double compute_fitness(int FORMATION_SIZE, float loc[4][3]) {
	double mean_v=0;
	double mean_delta_v=0;
	double mean_formation_distance=0;
	double mean_obstacle_term=0;
	
	double weight_v=1;
	double weight_delta_v=1;
	double weight_formation_distance=10;
	double weight_obstacle_term=1;
	double weight_goal_reached=0.5;
	
	float unit_center[2] = {0,0};

	
	// compute the unit center
         int i,j;	
         for(j = 0; j < 2; j++) {
            unit_center[j] = 0;
            for(i = 0; i < 4; i++) {
                unit_center[j] += loc[i][j];
            }
            unit_center[j] /= 4;
         }
         
         get_move_to_goal_vector(dir_goal,unit_center);
         float distance_to_goal=sqrt(dir_goal[0]*dir_goal[0]+dir_goal[1]*dir_goal[1]);

	double low_threshold_goal=0.1;
	double high_threshold_goal=0.5;
	double distance_goal_term;
	// compute the distance_to_goal_term
         if (distance_to_goal>high_threshold_goal) {
             distance_goal_term=0;
         } else if (distance_to_goal<low_threshold_goal) {
             distance_goal_term=1;
         } else {
             distance_goal_term=(high_threshold_goal-distance_to_goal)
                     /(high_threshold_goal-low_threshold_goal);
         }
	
	
	// other terms
	for (i=0; i<FORMATION_SIZE; i++) {
		mean_v+=speed_sum[i][0];
		mean_delta_v+=speed_sum[i][1];
		mean_formation_distance+=keep_formation_distance[i];
		mean_obstacle_term+=obstacle_term_sum[i];
	}
         

	
	mean_v=mean_v/number_of_time_step/FORMATION_SIZE;
	mean_delta_v=mean_delta_v/number_of_time_step/FORMATION_SIZE;
	mean_formation_distance=mean_formation_distance/FORMATION_SIZE/number_of_time_step;
	mean_obstacle_term=mean_obstacle_term/FORMATION_SIZE/number_of_time_step;


         printf("\n[FITNESS] mean_v................... = %f\n", mean_v);
         printf("[FITNESS] mean_delta_v............. = %f\n", mean_delta_v);
         printf("[FITNESS] mean_formation_distance.. = %f\n", mean_formation_distance);
         printf("[FITNESS] mean_obstacle_term....... = %f\n", mean_obstacle_term);
         printf("[FITNESS] distance_goal_term............. = %f\n", distance_goal_term);

    
	printf("[FITNESS] %1.3f^2 * (1 - sqrt(%1.1f * %1.3f)) * (1/(%1.1f * %1.3f)) * (%1.1f * (%1.3f + 0.05)) + * %1.1f * %f\n",
            mean_v,
            weight_delta_v,
            mean_delta_v,
            weight_formation_distance,
            mean_formation_distance,
            weight_obstacle_term,
            mean_obstacle_term,
            weight_goal_reached,
            distance_goal_term);


         
	
	return weight_v*mean_v*(1-sqrt(weight_delta_v*mean_delta_v))
          	*(1/(weight_formation_distance*mean_formation_distance))
          	/(weight_obstacle_term*(mean_obstacle_term+0.05))+ // to avoid case with mean=0
          	+weight_goal_reached*distance_goal_term;
}





/*
 * Computes all the quantities that have to be traced for fitness function
 */
void update_fitness_computation_for_robot(float loc[4][3],float prev_loc[4][3],float speed[4][3],int robot_id,float time_step,int formation_type) {
	update_speed_sum(loc,prev_loc,speed,robot_id,time_step);
	
	update_keep_formation_distance(loc,robot_id,formation_type);
	
	update_obstacle_term(loc,robot_id);
	// update time_step
	if (robot_id==0) {
		number_of_time_step++;
	}
}





/*
 * Computes the speed of the robots (also the angular speed) given the last 2 positions
 */
void update_speed_sum(float loc[4][3],float prev_loc[4][3],float speed[4][3],int robot_id,float time_step) {
    // compute the speed
    compute_speed(loc,prev_loc,speed,robot_id,time_step);
	
    // compute the absolute value of the speed (speed_sum[robot_id][0])
    speed_sum[robot_id][0]+=sqrt(speed[robot_id][0]*speed[robot_id][0]+speed[robot_id][1]*speed[robot_id][1]);
	
    // compute the absolute value of the "turning speed"
    speed_sum[robot_id][1]+=fabs(speed[robot_id][2]);
}





/*
 * Computes the obstacle term
 */
void update_obstacle_term(float loc[4][3],int robot_id) {
    float minimum_obstacle_distance=1000000;
    float distance_vector[2];
    float distance;
    float distance_lower_threshold=0.15;
    float distance_higher_threshold=0.3;
    
    int i;
    for (i=0;i<6;i++) {
        distance_vector[0]=loc[robot_id][0]-fitness_obstacle_loc[i][0];
        distance_vector[1]=loc[robot_id][1]-fitness_obstacle_loc[i][1];
        distance=sqrt(distance_vector[0]*distance_vector[0]+distance_vector[1]*distance_vector[1]);
        if (distance<minimum_obstacle_distance) {
            minimum_obstacle_distance=distance;
        }
    }
    
    if (minimum_obstacle_distance>distance_higher_threshold) {
        
    } else if (minimum_obstacle_distance<distance_lower_threshold) {
        obstacle_term_sum[robot_id]+=1;
    } else {
        obstacle_term_sum[robot_id]+=(distance_higher_threshold-minimum_obstacle_distance)
              /(distance_higher_threshold-distance_lower_threshold);
    }
    
    return;
}





/*
 * Computes the speed of the robots (also the angular speed) given the last 2 positions
 */
void update_keep_formation_distance(float loc[4][3],int robot_id, int formation_type) {
    
    float relative_coordinates[3] = {0, 0, 0};
    float absolute_coordinates[3] = {0, 0, 0};
    float direction[2] = {0,0};
    float unit_center[2] = {0,0};
    
    // compute the unit center
    int i,j;	
    for(j = 0; j < 3; j++) {
        unit_center[j] = 0;
        for(i = 0; i < 4; i++) {
            unit_center[j] += loc[i][j];
        }
        unit_center[j] /= 4;
    }
    
    get_move_to_goal_vector(dir_goal,unit_center);
    get_relative_formation_coordinates(relative_coordinates,formation_type,robot_id);
    get_absolute_formation_coordinates(absolute_coordinates, relative_coordinates, dir_goal,unit_center);
    direction[0] = absolute_coordinates[0] - loc[robot_id][0];
    direction[1] = absolute_coordinates[1] - loc[robot_id][1];

    // compute the norm of direction
    keep_formation_distance[robot_id]+=sqrt(direction[0]*direction[0]+direction[1]*direction[1]);
}


/*
 * Computes the speed of the robots (also the angular speed) given the last 2 positions
 */
void compute_speed(float loc[4][3],float prev_loc[4][3],float speed[4][3],int robot_id,float time_step) {
	int i;
	for (i=0; i<2; i++) {
		speed[robot_id][i]=(loc[robot_id][i]-prev_loc[robot_id][i])/time_step;
	}
	
	// for the difference in speed between wheels: DeltaV=2*DeltaTheta*(AXLE_LENGTH/2)/DeltaT
	speed[robot_id][2]=AXLE_LENGTH*(loc[robot_id][2]-prev_loc[robot_id][2])/time_step;
}


/*
 * Called at the beginning of the simulation to reset it
 */
void reset_fitness_computation(int FORMATION_SIZE,float migrx,float migrz,float obstacle_loc[6][2]) {
    number_of_time_step=0;
    int i;
    for (i=0; i<FORMATION_SIZE; i++) {
        speed_sum[i][0]=0;
        speed_sum[i][1]=0;
        keep_formation_distance[i]=0;
        obstacle_term_sum[i]=0;
    }
    
    // Goal
    migr[0]=migrx;
    migr[1]=migrz;
    
    //Obstacle
    for (i=0; i<6; i++) {
        fitness_obstacle_loc[i][0]=obstacle_loc[i][0];        
        fitness_obstacle_loc[i][1]=obstacle_loc[i][1];
    }
}




///////////////////////////////////////////////////////////////
// FROM HERE METHODS AND DEFINITIONS FROM ms_keep_formation  //
// TODO: Keep them like that?                                //
///////////////////////////////////////////////////////////////

//definitions
const float robot_dist = 1.0/12.0;

/* 
 * Computes the position where each robot should be if it where on formation,
 * in the carthesian system defined by:
 *    -> Origin : the unit_center
 *    -> Direction of the z-axis : move_to_goal vector
 *    -> Direction of the x-axis : perpendicular to the z-axis so that the angle 
 *       from the z-axis to the x-axis is -pi/2
 */
void get_relative_formation_coordinates(float* coordinates,int formation_type,int robot_id){
    if(formation_type == LINE) {
        coordinates[1] = 0.0;
        if(robot_id == 0)
            coordinates[0] = 3.0 * robot_dist;
        else if(robot_id == 1)
            coordinates[0] = 1.0 * robot_dist;
        else if(robot_id == 2)
            coordinates[0] = -1.0 * robot_dist;
        else
            coordinates[0] = -3.0 * robot_dist;
    } 
    else if(formation_type == COLUMN) {
        coordinates[0] = 0.0;
        if(robot_id == 0)
            coordinates[1] = 3.0 * robot_dist;
        else if(robot_id == 1)
            coordinates[1] = -1.0 * robot_dist;
        else if(robot_id == 2)
            coordinates[1] = 1.0 * robot_dist;
        else
            coordinates[1] = -3.0 * robot_dist;
    } 
    else if(formation_type == WEDGE) {
        if(robot_id == 0) {
            coordinates[0] = robot_dist;
            coordinates[1] = robot_dist;
        } else if(robot_id == 1) {
            coordinates[0] = 3.0 * robot_dist;
            coordinates[1] = -1.0 * robot_dist;
        } else if(robot_id == 2) {
            coordinates[0] = -1.0 * robot_dist;
            coordinates[1] = robot_dist;
        } else {
            coordinates[0] = -3.0 * robot_dist;
            coordinates[1] = -1.0 * robot_dist;
        }
    } 
    else if(formation_type == DIAMOND) {
        if(robot_id == 0) {
            coordinates[0] = 0.0;
            coordinates[1] = 2.0 * robot_dist;
        } else if(robot_id == 1) {
            coordinates[0] = 2.0 * robot_dist;
            coordinates[1] = 0.0;
        } else if(robot_id == 2) {
            coordinates[0] = -2.0 * robot_dist;
            coordinates[1] = 0.0;
        } else {
            coordinates[0] = 0.0;
            coordinates[1] = -2.0 * robot_dist;
        }
    } 
    else {
		printf("%d\n",formation_type);
        printf("Error: wrong formation type when getting robots formation local coordinates.\n"); 
        // this error message should never be printed
    }
}

/* 
 * Computes the position where each robot should be if it where on formation
 * in the carthesian system defined by the world, from the relative position
 * in the carthesian system defined by:
 *    -> Origin : the unit_center
 *    -> Direction of the z-axis : move_to_goal vector
 *    -> Direction of the x-axis : perpendicular to the z-axis so that the angle 
 *       from the z-axis to the x-axis is -pi/2
 */
void get_absolute_formation_coordinates(float* coordinates, float* relative_coordinates, float* dir_goal,float* unit_center) {
    float dist_to_goal = norm(dir_goal, 2);
    
    // Theta is the angle between the x-axis and the direction vector to the goal.  
    float cosTheta = dir_goal[0] / dist_to_goal;
    float sinTheta = dir_goal[1] / dist_to_goal;

    // Changing system coordinates, rotation + translation, taking care of the right angle Theta
    coordinates[0] = relative_coordinates[0] * sinTheta
                   + relative_coordinates[1] * cosTheta 
                   + unit_center[0];
    coordinates[1] = - relative_coordinates[0] * cosTheta
                   + relative_coordinates[1] * sinTheta 
                   + unit_center[1];
}

void get_move_to_goal_vector(float * direction,float* unit_center) {
	int j;
	for (j=0;j<2;j++) {
		direction[j] = (migr[j]-unit_center[j]);
	}
	return;
}

/*
 * Computes the norm of a vector of dimension dim. 
 */
float norm(float* vector, int dim) {
	float res = 0;
	int i;
	for(i = 0; i < dim; i++)
		res += vector[i]*vector[i];
	return sqrtf(res);
}
