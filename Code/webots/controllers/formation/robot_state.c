#include <stdio.h>
#include <math.h>
#include <string.h>

#include "robot_state.h"
#include "utils.h"


////////////////////////////////////////////////////////////////////////////////////////////////////
// This file contains everything related to the robots' current states                            //
//  - current position                                                                            //
//  - current speed                                                                               //
//  - etc.                                                                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////



// definitions

// values from: https://www.cyberbotics.com/guide/using-the-e-puck-robot.php
// - The forward direction of the e-puck (the direction the eye of the camera is looking 
//   to) is given by the negative z-axis. The direction vector of the camera is pointing 
//   in the opposite direction, namely the direction of the positive z-axis
// - The axle's direction is given by the positive x-axis. It points to the robot's right (in 
//   driving direction)
const float sens_dir[NB_SENSORS] = {1.27,
                                    0.77,
                                    0.00,
                                    5.21,
                                    4.21,
                                    3.14159,
                                    2.37,
                                    1.87};



/*
 * Reset the robot's devices and get its ID
 */
void reset(void) {
	wb_robot_init();
	
    // enable receiver
	receiver = wb_robot_get_device("receiver");     // receiver and emitter for communication with
	emitter  = wb_robot_get_device("emitter");      // supervisor.
	receiver2 = wb_robot_get_device("receiver2");   // receiver2 and emitter2 for local 
	emitter2  = wb_robot_get_device("emitter2");    // communication
	wb_receiver_enable(receiver,64);
         wb_receiver_enable(receiver2,64);

	
	if (emitter == 0) printf("missing emitter\n");
	
	int i;
	char s[4]="ps0";
	for(i=0; i<NB_SENSORS; i++) {
		dist_sens[i] = wb_robot_get_device(s);  // the device name is specified in the world file
		s[2]++;                                 // increases the device number
	}

	robot_name = (char*)wb_robot_get_name(); 

    // enable distance sensors
	for (i=0; i<NB_SENSORS; i++) {
		wb_distance_sensor_enable(dist_sens[i], TIME_STEP);
	}

	//Reading the robot's name (and robot id from the robot's name)
	sscanf(robot_name,"rob%d",&robot_id_u);
	
	robot_id = robot_id_u%FORMATION_SIZE;     // normalize between 0 and FORMATION_SIZE-1
	for (i=0; i<FORMATION_SIZE; i++) {
		initialized[i] = 0;                   // Set initialization to 0 (= not yet initialized)
		initialized_weights[i]=0;			  // Set initialization of weights to 0(= not yet initialized)
	}

    // restart time
    time_steps_since_start = 0;
	
	printf("Reset: robot %d\n",robot_id);
}





/*
 * Initialize robot's position
 * The robot receive infomation on his ID, position and the goal
 */
void initial_pos(void){
	char *inbuffer;
	int rob_nb,initial_pos_flag;
	float rob_x, rob_z, rob_theta; // Robot position and orientation
	
	
	while (initialized[robot_id] == 0) {
		
		// wait for message
		while (wb_receiver_get_queue_length(receiver) == 0)	{
            	wb_robot_step(TIME_STEP);
        }
		
		inbuffer = (char*) wb_receiver_get_data(receiver);
		sscanf(inbuffer,"%d#%d#%f#%f#%f##%f#%f#%d",
                        &rob_nb,
                        &initial_pos_flag,
                        &rob_x,
                        &rob_z,
                        &rob_theta,
                        &migr[0],
                        &migr[1],
                        &formation_type);
		

		if (rob_nb == robot_id && initial_pos_flag==0) {
			// Initialize self position
			loc[rob_nb][0] = rob_x;                 // x-position
			loc[rob_nb][1] = rob_z;                 // z-position
			loc[rob_nb][2] = rob_theta;             // theta
			prev_loc[rob_nb][0] = loc[rob_nb][0];
			prev_loc[rob_nb][1] = loc[rob_nb][1];
			initialized[rob_nb] = 1;                // initialized = true
		}
		
//if(robot_id ==0)
//printf("My positions: %f, %f\n", loc[0][0], loc[0][1]);
		
		wb_receiver_next_packet(receiver);
	}
}





/*
 * Initialize robot's weights
 */
void initial_weights(void){
	char *inbuffer;
	int rob_nb,initial_pos_flag; // if initial_pos_flag==1 the supervisor is sending weights
	
	while (initialized_weights[robot_id] == 0) {
		
		// wait for message
		while (wb_receiver_get_queue_length(receiver) == 0)	{
			wb_robot_step(TIME_STEP);
		}
		
		// temporal variable
		// motorschema weights
		float w_goal_temp;
		float w_keep_formation_temp;
		float w_avoid_robot_temp;
		float w_avoid_obstacles_temp;
		float w_noise_temp;
		
		// thresholds
		float avoid_robot_min_threshold_temp;
		float avoid_robot_max_threshold_temp;
		float avoid_obst_min_threshold_temp;
		float avoid_obst_max_threshold_temp;
		float keep_formation_min_threshold_temp;
		float keep_formation_max_threshold_temp;
		float move_to_goal_min_threshold_temp;
		float move_to_goal_max_threshold_temp;
		
		// noise
		int noise_gen_frequency_temp;  // defines, after how many steps a new random vector should be generated
		int fading_temp;              // true, if nice transition is wished from one random vector to the next

		
		
		inbuffer = (char*) wb_receiver_get_data(receiver);
		sscanf(inbuffer,"%d#%d#%f#%f#%f#%f#%f#%d#%d#%f#%f#%f#%f#%f#%f#%f#%f",&rob_nb,&initial_pos_flag,
			   &w_goal_temp,&w_keep_formation_temp,&w_avoid_robot_temp,&w_avoid_obstacles_temp,&w_noise_temp,
			   &noise_gen_frequency_temp,&fading_temp,
			   &avoid_obst_min_threshold_temp,&avoid_obst_max_threshold_temp,
			   &move_to_goal_min_threshold_temp,&move_to_goal_max_threshold_temp,
			   &avoid_robot_min_threshold_temp,&avoid_robot_max_threshold_temp,
			   &keep_formation_min_threshold_temp,&keep_formation_max_threshold_temp);

		// Only info about self will be taken into account at first.
		
		if (rob_nb == robot_id && initial_pos_flag==1)
		{
			// Initialize robot's weights
			// motorschema weights
			w_goal=w_goal_temp;
			w_keep_formation=w_keep_formation_temp;
			w_avoid_robot=w_avoid_robot_temp;
			w_avoid_obstacles=w_avoid_obstacles_temp;
			w_noise=w_noise_temp;
			
			// thresholds
			avoid_robot_min_threshold    = avoid_robot_min_threshold_temp;
			avoid_robot_max_threshold    = avoid_robot_max_threshold_temp;
			avoid_obst_min_threshold     = avoid_obst_min_threshold_temp;
			avoid_obst_max_threshold     = avoid_obst_max_threshold_temp;
			keep_formation_min_threshold = keep_formation_min_threshold_temp;
			keep_formation_max_threshold = keep_formation_max_threshold_temp;
			move_to_goal_min_threshold   = move_to_goal_min_threshold_temp;
			move_to_goal_max_threshold   = move_to_goal_max_threshold_temp;
			
			// noise
			noise_gen_frequency=noise_gen_frequency_temp;
			fading=fading_temp;
			
			initialized_weights[rob_nb] = 1;    // initialized = true
			
		}
		wb_receiver_next_packet(receiver);
	}
}





/*
 * Computes the unit center of all the robots from their actual
 * positions
 * (Ondine) Positions of the other robots will then have to be determined 
 * with local communication
 */
void compute_unit_center(void) {
	int i,j;	
	for(j = 0; j < 3; j++) {
          	unit_center[j] = 0;
		for(i = 0; i < FORMATION_SIZE; i++) 
			unit_center[j] += loc[i][j];
		unit_center[j] /= FORMATION_SIZE;
	}
}





/*
 * Updates robot position with wheel speeds
 * Used for odometry
 */
void update_self_motion(int msl, int msr) {
	float theta = loc[robot_id][2];
	
	// Compute deltas of the robot (time steps in seconds instead of ms)
	float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * TIME_STEP/1000;
	float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * TIME_STEP/1000;
	float du = (dr + dl)/2.0;
	float dtheta = (dr - dl)/AXLE_LENGTH;
	
	// Compute deltas in the environment
	float dx = -du * sinf(theta);
	float dz = -du * cosf(theta);
	
	// Update position
	loc[robot_id][0] += dx;
	loc[robot_id][1] += dz;
	loc[robot_id][2] += dtheta;
	
	// Keep orientation within 0, 2pi
	if (loc[robot_id][2] > 2*M_PI) loc[robot_id][2] -= 2.0*M_PI;
	if (loc[robot_id][2] < 0) loc[robot_id][2] += 2.0*M_PI;
}





/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(int *msl, int *msr)
{
	// Compute wanted position from speed vector and current location
    // x in robot coordinates
	float x = speed[robot_id][0]*cosf(loc[robot_id][2]) - speed[robot_id][1]*sinf(loc[robot_id][2]);
    // z in robot coordinates
	float z = -speed[robot_id][0]*sinf(loc[robot_id][2]) - speed[robot_id][1]*cosf(loc[robot_id][2]); 
	
	float Ku = 0.2;                 // Forward control coefficient
	float Kw = 10.0;                // Rotational control coefficient
	float range = sqrtf(x*x + z*z); // Distance to the wanted position
	float bearing = -atan2(x, z);	// Orientation of the wanted position
	
	// Compute forward control
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*range*sinf(bearing);
	
	// Convert to wheel speeds!
	*msl = 50*(u - AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;
	*msr = 50*(u + AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;
	limit(msl,MAX_SPEED);
	limit(msr,MAX_SPEED);
}
