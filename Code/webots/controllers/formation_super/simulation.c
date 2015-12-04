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




// private methods
char valid_locs(int r_id);

// private variables
double new_loc[FORMATION_SIZE][3];
double new_rot[FORMATION_SIZE][4];
double new_loc_obs[NB_OBSTACLES][3];
double new_rot_obs[NB_OBSTACLES][4];
double new_loc_goal[3];


/* 
 * Initialization common to the running world AND the pso worlds;
 * it should be called only once at the beginning of the run. 
 */
void initialize(void) {
  wb_robot_init();
  
  emitter = wb_robot_get_device("emitter");
  if (emitter == 0) printf("missing emmiter\n");
}


/*
 * Initialize robot positions and devices from the world given by the user.
 */
void reset(void) {
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
    
    simulation_duration = 0;
}


/* 
 * Initialize robot positions and devices such that:
 * - there is a barrier of obstacles
 * - robots are on one side; the goal on the other one.
 * Function used for PSO. 
 */
void reset_barrier_world(void) {
    char obs[5] = "obs0";
    char rob[5] = "rob0";
    int obs_id, i;
    float dist_between_obs = 0.1;
    
    // Set up the obstacles
    for (obs_id=0; obs_id<NB_OBSTACLES; obs_id++) {
      sprintf(obs,"obs%d",obs_id);
      obss[obs_id]          = wb_supervisor_node_get_from_def(obs);
      new_rot_obs[obs_id][0] = 0.0;
      new_rot_obs[obs_id][1] = 1.0;
      new_rot_obs[obs_id][2] = 0.0;
      new_rot_obs[obs_id][3] = 4.45059;
    
      new_loc_obs[obs_id][0] = -0.25 + obs_id*dist_between_obs;
      new_loc_obs[obs_id][1] = -1.25566e-13;
      new_loc_obs[obs_id][2] = -1.23;
      
      wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(obss[obs_id],"translation"),
                                     new_loc_obs[obs_id]);
      wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(obss[obs_id],"rotation"), 
                                        new_rot_obs[obs_id]);
    }
    
    // Set up the goal behind the wall of obstacles
    goal_id = wb_supervisor_node_get_from_def("goal-node");
    new_loc_goal[0] = 0.2;
    new_loc_goal[1] = 0.0;
    new_loc_goal[2] = -4.0;
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(goal_id,"translation"), new_loc_goal);
    goal_field = wb_supervisor_node_get_field(goal_id,"translation");

    // Randomly set up robots on the other side of the wall of obstacles
    for (i=0; i<FORMATION_SIZE; i++) {
        sprintf(rob,"rob%d",i);
        robs[i]          = wb_supervisor_node_get_from_def(rob);
        random_pos(i, -2.0, -1.5);
        robs_trans[i]    = wb_supervisor_node_get_field(robs[i],"translation");
        robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
    }
    
    simulation_duration = 0;

}



/*
 * Send to each robot its initial position the goal's position and the formation type. 
 */
void send_init_poses(void)
{
    char buffer[255];	// Buffer for sending data
    int i;
     
    for (i = 0; i < FORMATION_SIZE; i++) {

        // Get robot's position
        loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0];       // X
        loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2];       // Z
        loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA

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



/*
 * Send to each robot its current position and orientation. 
 * This function is here as an example for communication using the supervisor
 */
void send_current_poses(void){
    char buffer[255]; // buffer for sending data
    int i;

    for (i = 0; i < FORMATION_SIZE; i++) {
        // Get data
        loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0];       // X
        loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2];       // Z
        loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA

        // Sending positions to the robots
        sprintf(buffer,"%1d#%f#%f#%f#%f#%f",i+offset,loc[i][0],loc[i][1],loc[i][2], migrx, migrz);
        wb_emitter_send(emitter,buffer,strlen(buffer));				
    }
}





/*
 * Generates random number in [0,1]
 */
float rand_01(void) {
    srand(time(NULL));
    return ((float)rand())/((float)RAND_MAX);
}



/*
 * Randomly position the specified robot within a certain zone
 */
void random_pos(int robot_id, float x_min, float z_min) {
    //printf("Setting random position for %d\n",robot_id);

    new_rot[robot_id][0] = 0.0;
    new_rot[robot_id][1] = 1.0;
    new_rot[robot_id][2] = 0.0;
    new_rot[robot_id][3] = 2.0*3.14159*rand_01();

    do {
        new_loc[robot_id][0] = x_min + ARENA_SIZE*rand_01();
        new_loc[robot_id][2] = z_min + ARENA_SIZE*rand_01();
    } while (!valid_locs(robot_id));
    
    printf("Robot_id=%d, x=%f, z=%f\n", robot_id, new_loc[robot_id][0], (float)rand());
    
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robs[robot_id],"translation"),
                                     new_loc[robot_id]);
    wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robs[robot_id],"rotation"), 
                                        new_rot[robot_id]);
}




/*
 * Makes sure no robots are overlapping
 */
char valid_locs(int r_id) {
    int i;

    // This for loops stops when i = r_id because new_loc[i] for i > r_id are not initialized yet
    // when this function is called. Moreoever, it does not make sense to verify the distance between
    // a robot and itself. 
    for (i = 0; i < r_id; i++) {

        float dist = pow(new_loc[i][0]-new_loc[r_id][0],2)+pow(new_loc[i][2]-new_loc[r_id][2],2);
        float max_dist = pow((2*ROB_DIST+0.01), 2);

        if (dist < max_dist)
            return 0;
    }
    return 1;
}




