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




// Private methods
char valid_locs(int r_id);
char valid_locs_obs(int obs_id);



// Private variables

// Variables used for PSO
double new_loc[FORMATION_SIZE][3];
double new_rot[FORMATION_SIZE][4];
double new_loc_obs[NB_OBSTACLES][3];
double new_rot_obs[NB_OBSTACLES][4];
double new_loc_goal[3];

// Variables for the actual run
double initial_loc[FORMATION_SIZE][3];
double initial_rot[FORMATION_SIZE][4];
double initial_loc_obs[NB_OBSTACLES][3];
double initial_rot_obs[NB_OBSTACLES][4];
double initial_loc_goal[3];

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
    wb_robot_init();
    emitter = wb_robot_get_device("emitter");
    if (emitter==0) printf("missing emitter\n");
    char rob[5] = "rob0";
    char obs[5] = "obs0";
    int i,j;
    
    for (i=0; i<FORMATION_SIZE; i++) {
        sprintf(rob,"rob%d",i);
        robs[i]          = wb_supervisor_node_get_from_def(rob);
        initial_robs_trans[i]    = wb_supervisor_node_get_field(robs[i],"translation");
        initial_robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation"); 
    
        initial_loc[i][0] = wb_supervisor_field_get_sf_vec3f(initial_robs_trans[i])[0];       // X
        initial_loc[i][1] = wb_supervisor_field_get_sf_vec3f(initial_robs_trans[i])[1];       // Y
        initial_loc[i][2] = wb_supervisor_field_get_sf_vec3f(initial_robs_trans[i])[2];       // Z
        initial_rot[i][0] = wb_supervisor_field_get_sf_rotation(initial_robs_rotation[i])[0];            
        initial_rot[i][1] = wb_supervisor_field_get_sf_rotation(initial_robs_rotation[i])[1];           
        initial_rot[i][2] = wb_supervisor_field_get_sf_rotation(initial_robs_rotation[i])[2];           
        initial_rot[i][3] = wb_supervisor_field_get_sf_rotation(initial_robs_rotation[i])[3];    // THETA        
    }
    
    for (j=0; j<NB_OBSTACLES; j++) {
        sprintf(obs,"obs%d",j);
        obss[j] = wb_supervisor_node_get_from_def(obs);
        initial_obs_trans[j] = wb_supervisor_node_get_field(obss[j],"translation");
        initial_obs_rotation[j] = wb_supervisor_node_get_field(obss[j],"rotation");
        
        initial_loc_obs[j][0] = wb_supervisor_field_get_sf_vec3f(initial_obs_trans[j])[0];
        initial_loc_obs[j][1] = wb_supervisor_field_get_sf_vec3f(initial_obs_trans[j])[1];
        initial_loc_obs[j][2] = wb_supervisor_field_get_sf_vec3f(initial_obs_trans[j])[2];
      
        initial_rot_obs[j][0] = wb_supervisor_field_get_sf_rotation(initial_obs_rotation[j])[0];
        initial_rot_obs[j][1] = wb_supervisor_field_get_sf_rotation(initial_obs_rotation[j])[1];
        initial_rot_obs[j][2] = wb_supervisor_field_get_sf_rotation(initial_obs_rotation[j])[2];
        initial_rot_obs[j][3] = wb_supervisor_field_get_sf_rotation(initial_obs_rotation[j])[3];
    
    }
    
    goal_id = wb_supervisor_node_get_from_def("goal-node");
    initial_goal_field = wb_supervisor_node_get_field(goal_id,"translation");
    
    initial_loc_goal[0] = wb_supervisor_field_get_sf_vec3f(initial_goal_field)[0];
    initial_loc_goal[1] = wb_supervisor_field_get_sf_vec3f(initial_goal_field)[1];
    initial_loc_goal[2] = wb_supervisor_field_get_sf_vec3f(initial_goal_field)[2];
}





/* 
 * Initialize robot positions and devices such that:
 * - there is a barrier of obstacles
 * - robots are on one side; the goal on the other one.
 * Function used for PSO. 
 */
void reset_barrier_world(void) {
    int obs_id, i;
    float dist_between_obs = 0.1;
    
    // Set up the obstacles
    for (obs_id=0; obs_id<NB_OBSTACLES; obs_id++) {
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
    new_loc_goal[0] = 0.2;
    new_loc_goal[1] = 0.0;
    new_loc_goal[2] = -4.0;
    goal_field = wb_supervisor_node_get_field(goal_id,"translation");
    wb_supervisor_field_set_sf_vec3f(goal_field, new_loc_goal);

    // Randomly set up robots on the other side of the wall of obstacles
    for (i=0; i<FORMATION_SIZE; i++) {
        random_pos(i, -2.0, -1.5);
        robs_trans[i]    = wb_supervisor_node_get_field(robs[i],"translation");
        robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
    }
	
    goal_id = wb_supervisor_node_get_from_def("goal-node");
    goal_field = wb_supervisor_node_get_field(goal_id,"translation");

    
    //obstacle initialisation    	
    char obs[5]="obs0";
    for (i=0;i<NB_OBSTACLES;i++) {
        sprintf(obs,"obs%d",i);
        obstacles[i]= wb_supervisor_node_get_from_def(obs);
        obstacles_trans[i]=wb_supervisor_node_get_field(obstacles[i],"translation");
        
        obstacle_loc[i][0] = wb_supervisor_field_get_sf_vec3f(obstacles_trans[i])[0];  // X
        obstacle_loc[i][1] = wb_supervisor_field_get_sf_vec3f(obstacles_trans[i])[2];  // Z
    }
    
    simulation_duration = 0;

}





/* 
 * Initialize robot positions and devices with a difficult obstacle_avoidance configuration.
 * Function used for PSO. 
 */
void reset_world2(void) {
    int obs_id, i;
    
    // Set the obstacles
    if(NB_OBSTACLES < 3)
    {
        printf("PSO with the difficult configuration cannot be run on a world with less than 3 obstacles.\n");
        return;
    } else {
    
        int dist_between_obs = 1;
        new_rot_obs[0][0] = 0.0; new_rot_obs[0][1] = 1.0; new_rot_obs[0][2] = 0.0; new_rot_obs[0][3] = 4.45059;
        new_loc_obs[0][0] = -0.119863; new_loc_obs[0][1] = 0.0; new_loc_obs[0][2] = -0.188514; 
        new_rot_obs[1][0] = 0.0; new_rot_obs[1][1] = 1.0; new_rot_obs[1][2] = 0.0; new_rot_obs[1][3] = 4.45059;
        new_loc_obs[1][0] = -0.0147719; new_loc_obs[1][1] = 0.0; new_loc_obs[1][2] = -0.15636;
        new_rot_obs[2][0] = 0.0; new_rot_obs[2][1] = 1.0; new_rot_obs[2][2] = 0.0; new_rot_obs[2][3] = 4.45059; 
        new_loc_obs[2][0] = 0.0546256; new_loc_obs[2][1] = 0.0; new_loc_obs[2][2] = -0.0362992;
        for (obs_id=3; obs_id<NB_OBSTACLES; obs_id++) {
          new_rot_obs[obs_id][0] = 0.0;
          new_rot_obs[obs_id][1] = 1.0;
          new_rot_obs[obs_id][2] = 0.0;
          new_rot_obs[obs_id][3] = 4.45059;
        
          new_loc_obs[obs_id][0] = -0.25 + obs_id*dist_between_obs;
          new_loc_obs[obs_id][1] = -1.25566e-13;
          new_loc_obs[obs_id][2] = -1.23;
        }
        
        for (obs_id=0; obs_id<NB_OBSTACLES; obs_id++) {
          wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(obss[obs_id],"translation"),
                                         new_loc_obs[obs_id]);
          wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(obss[obs_id],"rotation"), 
                                         new_rot_obs[obs_id]);
        }
    }
    
    // Set the goal 
    new_loc_goal[0] = 0.351711;
    new_loc_goal[1] = 0.0;
    new_loc_goal[2] = -2.21704;
    goal_field = wb_supervisor_node_get_field(goal_id,"translation");
    wb_supervisor_field_set_sf_vec3f(goal_field, new_loc_goal);

    // Set the robots
    double new_loc2[FORMATION_SIZE][3] = { {-0.26479, -3.19812e-05, 0.439438}, 
                {-0.158156, -2.35412e-06, 0.436885}, 
                {-0.0499819, -3.19812e-05, 0.438491}, 
                {0.0586924, -3.19812e-05, 0.436266} };
    double new_rot2[FORMATION_SIZE][4] = { {-0.995072, 0.0946832, -0.0294367, 7.27241e-05}, 
                {-0.747912, 0.655124, -0.106958, -0.00746706}, 
                {-0.999563, 0, -0.0295695, 7.27241e-05},
                {-0.995072, 0.0946832, -0.0294367, 7.27241e-05} };
    for (i=0; i<FORMATION_SIZE; i++) {
        wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"),
                                     new_loc2[i]);
        wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"), 
                                     new_rot2[i]);
        robs_trans[i]    = wb_supervisor_node_get_field(robs[i],"translation");
        robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
    }
    
    simulation_duration = 0;
}





/* 
 * Initialize robot positions and devices such that:
 * - there are 6 obstacles
 * - robots are on one side; the goal on the other one
 * - all features are randomly positioned in determined areas. 
 * Function used for PSO. 
 */
void reset_random_world(void) {
    int obs_id, i;
    
    // Set up the obstacles
    for (obs_id=0; obs_id<NB_OBSTACLES; obs_id++) {
      random_pos_obs(obs_id, -2.0, -3.5);
    }
    
    printf("OK\n");
    
    // Set up the goal behind the wall of obstacles
    new_loc_goal[0] = 0.2;
    new_loc_goal[1] = 0.0;
    new_loc_goal[2] = -4.0;
    goal_field = wb_supervisor_node_get_field(goal_id,"translation");
    wb_supervisor_field_set_sf_vec3f(goal_field, new_loc_goal);

    // Randomly set up robots on the other side of the wall of obstacles
    for (i=0; i<FORMATION_SIZE; i++) {
        random_pos(i, -2.0, -1.5);
        robs_trans[i]    = wb_supervisor_node_get_field(robs[i],"translation");
        robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
    }
    
    simulation_duration = 0;

}





/*
 * Reinitializes robots and devices as decided in the initial world by the user. 
 */
void reset_to_initial_values(void) {
    int obs_id, i;
    
    // Set up the obstacles
    for (obs_id=0; obs_id<NB_OBSTACLES; obs_id++) {
      wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(obss[obs_id],"translation"),
                                     initial_loc_obs[obs_id]);
      wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(obss[obs_id],"rotation"), 
                                     initial_rot_obs[obs_id]);
    }
    
    // Set up the goal behind the wall of obstacles

    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(goal_id,"translation"), initial_loc_goal);

    // Randomly set up robots on the other side of the wall of obstacles
    for (i=0; i<FORMATION_SIZE; i++) {
        robs_trans[i]    = initial_robs_trans[i];
        robs_rotation[i] = initial_robs_rotation[i];
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

        migrx  = wb_supervisor_field_get_sf_vec3f(goal_field)[0];   // X
        offset = wb_supervisor_field_get_sf_vec3f(goal_field)[1];   // Y
        migrz  = wb_supervisor_field_get_sf_vec3f(goal_field)[2];   // Z
       
        orient_migr = -atan2f(migrx,migrz);                         // angle of migration urge
        if (orient_migr<0) {
            orient_migr+=2*M_PI; // Keep value between 0 and 2PI
        }
	
        // Send it out
        sprintf(buffer, "%1d#%1d#%f#%f#%f##%f#%f#%d#",
                        i,          // robot ID
                        0,          // 0 if we are sending poses, 1 if we are sending weights
                        loc[i][0],
                        loc[i][1],
                        loc[i][2],
                        migrx,
                        migrz,
                        formation_type);
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
        sprintf(buffer, "%1d#%1d#%f#%f#%f#%f#%f#%1d#%1d#%f#%f#%f#%f#%f#%f#%f#%f#",
                        i,          // robot ID
                        1,          // 0 if we are sending poses, 1 if we are sending weights
                        w_goal,
                        w_keep_formation,
                        w_avoid_robot,
                        w_avoid_obstacles,
                        w_noise,
                        noise_gen_frequency,
                        fading,
                        avoid_obst_min_threshold,
                        avoid_obst_max_threshold,
                        move_to_goal_min_threshold,
                        move_to_goal_max_threshold,
                        avoid_robot_min_threshold,
                        avoid_robot_max_threshold,
                        keep_formation_min_threshold,
                        keep_formation_max_threshold);

        wb_emitter_send(emitter,buffer,strlen(buffer));

        // Run one step
        wb_robot_step(TIME_STEP);
    }
}


void send_real_run_init_poses(void) {
    char buffer[255];	// Buffer for sending data
    int i;

    for (i = 0; i < FORMATION_SIZE; i++) {

        // Get robot's position
        loc[i][0] = initial_loc[i][0]; // X
        loc[i][1] = initial_loc[i][2]; // Z
        loc[i][2] = initial_rot[i][3]; // THETA
                
        wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"),
                                     initial_loc[i]);
        wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"), 
                                     initial_rot[i]);
                                     
        migrx = initial_loc_goal[0];    // X
        offset = initial_loc_goal[1];   // Y
        migrz = initial_loc_goal[2];    // Z
        
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
        // Set prev_loc
        prev_loc[i][0]=loc[i][0];
        prev_loc[i][1]=loc[i][1];
        prev_loc[i][2]=loc[i][2];

        // Get data
        loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0];       // X
        loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2];       // Z
        loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA

        // Sending positions to the robots
        sprintf(buffer,"%1d#%1d#%f#%f#%f#",i+offset,0,loc[i][0],loc[i][1],loc[i][2]);
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
		
	if (distance_to_goal<0.1) {
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
 * Randomly position the specified robot within a certain zone
 */
void random_pos(int robot_id, float x_min, float z_min) {

    new_rot[robot_id][0] = 0.0;
    new_rot[robot_id][1] = 1.0;
    new_rot[robot_id][2] = 0.0;
    new_rot[robot_id][3] = 2.0*3.14159*rand_01();

    do {
        new_loc[robot_id][0] = x_min + ARENA_SIZE*rand_01();
        new_loc[robot_id][2] = z_min + ARENA_SIZE*rand_01();
    } while (!valid_locs(robot_id));
    
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robs[robot_id],"translation"),
                                    new_loc[robot_id]);
    wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robs[robot_id],"rotation"), 
                                    new_rot[robot_id]);
}





/*
 * Randomly position the specified obstacle within a certain zone
 */
void random_pos_obs(int obs_id, float x_min, float z_min) {
    //printf("Setting random position for %d\n",robot_id);

    new_rot_obs[obs_id][0] = 0.0;
    new_rot_obs[obs_id][1] = 1.0;
    new_rot_obs[obs_id][2] = 0.0;
    new_rot_obs[obs_id][3] = 4.45059;

    // Note that it does not matter if obstacles are overlapping
    do {
        new_loc_obs[obs_id][0] = x_min + ARENA_SIZE*rand_01();
        new_loc_obs[obs_id][2] = z_min + ARENA_SIZE*rand_01();
    } while (!valid_locs_obs(obs_id));
    
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(obss[obs_id],"translation"),
                                    new_loc_obs[obs_id]);
    wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(obss[obs_id],"rotation"), 
                                    new_rot_obs[obs_id]);
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





/*
 * Makes sure obstacles are not overlapping too much
 */
char valid_locs_obs(int obs_id) {
    int i;

    // This for loops stops when i = obs_id because new_loc_obs[i] for i > obs_id are not initialized yet
    // when this function is called. Moreoever, it does not make sense to verify the distance between
    // an obstacle and itself. 
    for (i = 0; i < obs_id; i++) {

        float dist = pow(new_loc_obs[i][0]-new_loc_obs[obs_id][0],2)+pow(new_loc_obs[i][2]-new_loc_obs[obs_id][2],2);
        float max_dist = pow((2*0.1+0.01), 2);

        if (dist < max_dist)
            return 0;
    }
    return 1;
}
