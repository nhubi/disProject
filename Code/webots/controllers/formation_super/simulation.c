////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// The methods in this file are used to modify the world, communicating with the robots, starting //
// simulations etc.                                                                               //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "simulation.h"
#include "fitness.h"




// private methods
char valid_locs(int r_id);

// private variables
double new_loc[FORMATION_SIZE][3];
double new_rot[FORMATION_SIZE][4];





/*
 * Initialize robot positions and devices
 */
void reset(void) {
    wb_robot_init();
    emitter = wb_robot_get_device("emitter");
    if (emitter==0) printf("missing emitter\n");
	
    char rob[5] = "rob0";
    int i;
    for (i=0;i<FORMATION_SIZE;i++) {
        sprintf(rob,"rob%d",i);
        robs[i]          = wb_supervisor_node_get_from_def(rob);
        robs_trans[i]    = wb_supervisor_node_get_field(robs[i],"translation");
        robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
    }
	
    goal_id = wb_supervisor_node_get_from_def("goal-node");
    goal_field = wb_supervisor_node_get_field(goal_id,"translation");

    
    //obstacle initialisation    	
    char obs[5]="obs0";
    for (i=0;i<NUMBER_OF_OBSTACLES;i++) {
        sprintf(obs,"obs%d",i);
        obstacles[i]= wb_supervisor_node_get_from_def(obs);
        obstacles_trans[i]=wb_supervisor_node_get_field(obstacles[i],"translation");
        
        obstacle_loc[i][0] = wb_supervisor_field_get_sf_vec3f(obstacles_trans[i])[0];       // X
        obstacle_loc[i][1] = wb_supervisor_field_get_sf_vec3f(obstacles_trans[i])[2];       // Z
    }
    
    simulation_duration = 0;
}




/*
 * Send to each robot its initial position the goal's position and the formation type. 
 */
void send_init_poses(void){
    char buffer[255];	// Buffer for sending data
    int i;
     
    for (i = 0; i < FORMATION_SIZE; i++) {

        // Get robot's position
        loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0];       // X
        loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2];       // Z
        loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
		
        // Initialise prev_loc
        prev_loc[i][0]=loc[i][0];
        prev_loc[i][1]=loc[i][1];
        prev_loc[i][2]=loc[i][2];

        migrx = wb_supervisor_field_get_sf_vec3f(goal_field)[0];    // X
        offset = wb_supervisor_field_get_sf_vec3f(goal_field)[1];   // Y
        migrz = wb_supervisor_field_get_sf_vec3f(goal_field)[2];    // Z
        orient_migr = -atan2f(migrx,migrz);                         // angle of migration urge
		
        if (orient_migr<0) {
            orient_migr+=2*M_PI; // Keep value between 0 and 2PI
        }
	
        // Send it out
        sprintf(buffer,"%1d#%f#%f#%f##%f#%f#%1d",i,loc[i][0],loc[i][1],loc[i][2],migrx,migrz,formation_type);
        wb_emitter_send(emitter,buffer,strlen(buffer));

        // Run one step
        wb_robot_step(TIME_STEP);
    }
}





void send_weights(void){
    char buffer[255];	// Buffer for sending data
    int i;
     
    for (i=0;i<FORMATION_SIZE;i++) {
	
        // Send it out
        sprintf(buffer,"%1d#%1d#%f#%f#%f#%f#%f#%1d#%1d#%f#%f#%f#%f#%f#%f#%f#%f#",i,1,
        w_goal,w_keep_formation,w_avoid_robot,w_avoid_obstacles,w_noise,
        noise_gen_frequency,fading,
        avoid_obst_min_threshold,avoid_obst_max_threshold,move_to_goal_min_threshold,move_to_goal_max_threshold,
        avoid_robot_min_threshold,avoid_robot_max_threshold,keep_formation_min_threshold,keep_formation_max_threshold);

        wb_emitter_send(emitter,buffer,strlen(buffer));

        // Run one step
        wb_robot_step(TIME_STEP);
    }
}





void send_current_poses(void){
    char buffer[255]; // buffer for sending data
    int i;

    for (i = 0; i < FORMATION_SIZE; i++) {
        // Set prev_loc
        prev_loc[i][0]=loc[i][0];
        prev_loc[i][1]=loc[i][1];
        prev_loc[i][2]=loc[i][2];

        // Get data
        loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0];       // X
        loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2];       // Z
        loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA

        // Sending positions to the robots
        //sprintf(buffer,"%1d#%f#%f#%f#%f#%f#%1d",i+offset,loc[i][0],loc[i][1],loc[i][2], migrx, migrz, formation_type);
        sprintf(buffer,"%1d#%1d#%f#%f#%f#%f#%f#%1d#",i+offset,0,loc[i][0],loc[i][1],loc[i][2], migrx, migrz, formation_type);
        wb_emitter_send(emitter,buffer,strlen(buffer));				
    }
}





/*
 * updates the fitness value of each robot.
 */
void update_fitness(void){
    int i;
    for(i = 0; i < FORMATION_SIZE; i++){
        update_fitness_computation_for_robot(loc,prev_loc,speed,i,TIME_STEP*5/1000.0,formation_type);
    }
}





/*
 * Returns true if formation is close enough from goal node. 
 * TODO: should we also verify that the formation is still ok?
 * TODO: what about max timestep per simulation? (Ondine has implemented something somewhere i think)
 */
int simulation_has_ended(void) {
	float centre_x=0;
	float centre_y=0;
	float distance_to_goal=0;
	
	
	// compute the formation center
	int i;
	for (i=0; i<FORMATION_SIZE; i++) {
		centre_x+=loc[i][0];
		centre_y+=loc[i][1];
	}   
	centre_x/=FORMATION_SIZE;
	centre_y/=FORMATION_SIZE;
	
	distance_to_goal+=(centre_x-migrx)*(centre_x-migrx);
	distance_to_goal+=(centre_y-migrz)*(centre_y-migrz);
	distance_to_goal=sqrt(distance_to_goal);
		
	if (distance_to_goal<0.2) {
		return 1;
	}
	
	return 0;
}





/*
 * initializes random generator
 */
void init_rand_01(void) {
    srand(time(NULL));
}





/*
 * Generates random number in [0,1]
 */
float rand_01(void) {
    return ((float)rand())/((float)RAND_MAX);
}





/*
 * Randomly position the specified robot
 * TODO: Ondine
 */
void random_pos(int robot_id) {
    //printf("Setting random position for %d\n",robot_id);
    new_rot[robot_id][0] = 0.0;
    new_rot[robot_id][1] = 1.0;
    new_rot[robot_id][2] = 0.0;
    new_rot[robot_id][3] = 2.0*3.14159*rand_01();
  
    do {
        new_loc[robot_id][0] = ARENA_SIZE*rand_01() - ARENA_SIZE/2.0;
        new_loc[robot_id][2] = ARENA_SIZE*rand_01() - ARENA_SIZE/2.0;
    } while (!valid_locs(robot_id));

    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robs[robot_id],"translation"),
                                     new_loc[robot_id]);
    wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robs[robot_id],"rotation"), 
                                        new_rot[robot_id]);
}





/*
 * Makes sure no robots are overlapping
 * TODO: Ondine
 */
char valid_locs(int r_id) {
    int i;

    for (i = 0; i < FORMATION_SIZE; i++) {
        if (r_id == i)
            continue;

        float dist = pow(new_loc[i][0]-new_loc[r_id][0],2)+pow(new_loc[i][2]-new_loc[r_id][2],2);
        float max_dist = pow((2*ROB_DIST+0.01), 2);

        if (dist < max_dist)
            return 0;
    }
    return 1;
}
