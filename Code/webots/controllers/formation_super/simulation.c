////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// The methods in this file are used to modify the world, communicating with the robots starting  //
// simulations etc.                                                                               //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "simulation.h"




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
}





/*
 * Send to each robot its initial position the goal's position and the formation type. 
 */
void send_init_poses(void)
{
    char buffer[255];	// Buffer for sending data
    int i;
     
    for (i=0;i<FORMATION_SIZE;i++) {

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
 * Generates random number in [0,1]
 */
float rand_01(void) {
    return ((float)rand_01())/((float)RAND_MAX);
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
