#include <stdio.h>
#include <math.h>
#include <string.h>

#include "utils.h"
#include "robot_state.h"

// Motorschemas
#include "ms_move_to_goal.h"
#include "ms_keep_formation.h"
#include "ms_avoid_static_obstacles.h"
#include "local_communications.h"
#include "avoid_robot.h"

// The swarm's center
// Defined also in robot state, it will have to be defined here with local_communications
// float unit_center[2];





/*
 * Combines the vectors from the 4 motorschemas and computes the corresponding wheel speed vector.
 * This vector can then be translated in actual wheel speeds.
 */
void computeDirection(){

    // direction vector for each motorschema
    float dir_goal[2]            = {0, 0};
    float dir_keep_formation[2]  = {0, 0};
    float dir_avoid_robot[2]     = {0, 0};
    float dir_avoid_obstacles[2] = {0, 0};

    // each motorschema's weight
    float w_goal            = 1;
    float w_keep_formation  = 1;
    float w_avoid_robot     = 1;
    float w_avoid_obstacles = 5;

    float min_avoid_obst_thrsh = 60;
    float max_avoid_obst_thrsh = 200;

    //TODO: We probably need a more complicated combination method than just weighted addition
    //      --> state machine?

    get_move_to_goal_vector(dir_goal, robot_id);
    get_keep_formation_vector(dir_keep_formation, robot_id);
    get_stat_obst_avoidance_vector(dir_avoid_obstacles, robot_id, min_avoid_obst_thrsh, max_avoid_obst_thrsh);
    get_avoid_robot_vector(dir_avoid_robot,robot_id);


    int d;
    //for each dimension d...
    for(d = 0; d < 2; d++){
        // init speed to (0,0)
        speed[robot_id][d] = 0;

        // combine the direction vectors.
        speed[robot_id][d] += w_goal            * dir_goal[d];
        speed[robot_id][d] += w_keep_formation  * dir_keep_formation[d];
        speed[robot_id][d] += w_avoid_robot     * dir_avoid_robot[d];
        speed[robot_id][d] += w_avoid_obstacles * dir_avoid_obstacles[d];
    }
    if(robot_id < 2)
        printf("\t  --> Direction[%d]: (%f,%f)\n", robot_id, speed[robot_id][0], speed[robot_id][1]);
}





/* 
 * The main function
 */
int main(){
    // for I14, sending current position to neighbors
    // char outbuffer[255];
  	
    // Set the weight at the beginning
    move_to_goal_min_threshold=0.1;
    move_to_goal_max_threshold=0.5;
    avoid_robot_min_threshold=0.05;
    avoid_robot_max_threshold=0.1;
    
    
    int msl, msr;                   // Wheel speeds
    int rob_nb;                     // Robot number
    float rob_x, rob_z, rob_theta;  // Robot position and orientation
    char *inbuffer;                 // Buffer for the receiver node
	
	
    // In this initialization, the common goal is communicated to the robot
    reset();        // Resetting the robot
    initial_pos();  // Initializing the robot's position
	
    msl = 0; msr = 0;
	

	
    // Forever
    for(;;){

        // Get information from other robots
        int count = 0;
        while (wb_receiver_get_queue_length(receiver) > 0 && count < FORMATION_SIZE) {
        inbuffer = (char*) wb_receiver_get_data(receiver);
        sscanf(inbuffer,"%d#%f#%f#%f",&rob_nb,&rob_x,&rob_z,&rob_theta);
			
        // check that received message comes from a member of the flock
        if ((int) rob_nb/FORMATION_SIZE == (int) robot_id/FORMATION_SIZE && (int) rob_nb%FORMATION_SIZE == (int) robot_id%FORMATION_SIZE ) {
            rob_nb %= FORMATION_SIZE;

            // If robot is not initialised, initialise it. 
            if (initialized[rob_nb] == 0) {
                loc[rob_nb][0] = rob_x;
                loc[rob_nb][1] = rob_z;
                loc[rob_nb][2] = rob_theta;
                prev_loc[rob_nb][0] = loc[rob_nb][0];
                prev_loc[rob_nb][1] = loc[rob_nb][1];
                initialized[rob_nb] = 1;

            // Otherwise, update its position.
            } else {

                // Remember previous position
                prev_loc[rob_nb][0] = loc[rob_nb][0];
                prev_loc[rob_nb][1] = loc[rob_nb][1];

                // save current position
                loc[rob_nb][0] = rob_x;
                loc[rob_nb][1] = rob_z;
                loc[rob_nb][2] = rob_theta;
            }


            // Calculate speed
            speed[rob_nb][0] = (1/TIME_STEP/1000)*(loc[rob_nb][0]-loc[rob_nb][0]);
            speed[rob_nb][1] = (1/TIME_STEP/1000)*(loc[rob_nb][1]-prev_loc[rob_nb][1]);
            count++;
        }
        //printf("robot id %d\n",robot_id);
        //printf("Positions %f %f\n%f %f\n%f %f\n%f %f\n",loc[0][0],loc[0][1],loc[1][0],loc[1][1],loc[2][0],loc[2][1],loc[3][0],loc[3][1]);
        wb_receiver_next_packet(receiver);
    }
		

    // compute current position according to motor speeds (msl, msr)
    prev_loc[robot_id][0] = loc[robot_id][0];
    prev_loc[robot_id][1] = loc[robot_id][1];
		
//		update_self_motion(msl,msr);
		
    // Send a ping to the other robot
    send_ping();
		
    // Receive other robot's ping
    //printf("Fuori dal while\n");
    process_received_ping_messages(robot_id);
    //printf("After received\n");

		
    // Compute localisation of the other robots
    compute_other_robots_localisation(robot_id);

    // Compute the unit center at each iteration
    compute_unit_center();
        
    // Get direction vectors from each motorscheme and combine them in speed table
    computeDirection();


    // Compute wheel speeds from speed vectors
    compute_wheel_speeds(&msl, &msr);

    // set your speeds here (I just put a constant number which you need to overwrite)
    wb_differential_wheels_set_speed(msl,msr);
		

    // Send current position to neighbors, uncomment for I14, don't forget to uncomment the 
    // declration of "outbuffer" at the begining of this function.
    // sprintf(outbuffer,"%1d#%f#%f#%f", robot_id, loc[robot_id][0], loc[robot_id][1], loc[robot_id][2]);
    // wb_emitter_send(emitter,outbuffer,strlen(outbuffer));
	

    // Continue one step
    wb_robot_step(TIME_STEP);
    }
}
