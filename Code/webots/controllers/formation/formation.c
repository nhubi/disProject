#include <stdio.h>
#include <math.h>
#include <string.h>

#include "utils.h"
#include "robot_state.h"

// Motorschemas
#include "ms_move_to_goal.h"
#include "ms_keep_formation.h"
#include "ms_avoid_static_obstacles.h"

// The swarm's center
float unit_center[2];





/*
 * Each robot sends a ping message, so the other robots can measure relative range and bearing to 
 * the sender. This is useful if you want to use relative range and bearing. This would be more 
 * realistic and less dependent on the supervisor. Try to make this work use 
 * process_received_ping_messages() function in lab 4 as a base to calculate range and bearing to 
 * the other robots.
 */
void send_ping(void)  {
    char out[10];
    strcpy(out,robot_name);  // in the ping message we send the name of the robot.
    wb_emitter_send(emitter,out,strlen(out)+1); 
}





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
    float w_avoid_obstacles = 1;

    //TODO: We probably need a more complicated combination method than just weighted addition
    //      --> state machine?

    get_move_to_goal_vector(dir_goal, robot_id);
    get_keep_formation_vector(dir_keep_formation, robot_id);
    get_stat_obst_avoidance_vector(dir_avoid_obstacles, robot_id);

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
}





/* 
 * The main function
 */
int main(){
	// for I14, sending current position to neighbors
	// char outbuffer[255];
	
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
//			printf("initialCheck %i %i\n",(int) rob_nb/FORMATION_SIZE,(int) robot_id/FORMATION_SIZE);
			
            // check that received message comes from a member of the flock
            if ((int) rob_nb/FORMATION_SIZE == (int) robot_id/FORMATION_SIZE) {
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
//					printf("\n got update robot[%d] = (%f,%f) \n",rob_nb,loc[rob_nb][0],loc[rob_nb][1]);

					// Remember previous position
					prev_loc[rob_nb][0] = loc[rob_nb][0];
					prev_loc[rob_nb][1] = loc[rob_nb][1];

                    			// save current position
					loc[rob_nb][0] = rob_x;
					loc[rob_nb][1] = rob_z;
					loc[rob_nb][2] = rob_theta;
				}

//				printf("speedStarting %f %f\n",(1/TIME_STEP/1000)*(loc[rob_nb][0]-prev_loc[rob_nb][0]),(1/TIME_STEP/1000)*(loc[rob_nb][1]-prev_loc[rob_nb][1]));
//				printf("Location %f %f\n",loc[rob_nb][0],loc[rob_nb][0]);

                		// Calculate speed
				speed[rob_nb][0] = (1/TIME_STEP/1000)*(loc[rob_nb][0]-loc[rob_nb][0]);
				speed[rob_nb][1] = (1/TIME_STEP/1000)*(loc[rob_nb][1]-prev_loc[rob_nb][1]);
				count++;
			}
			wb_receiver_next_packet(receiver);
		}
		

		// compute current position according to motor speeds (msl, msr)
		prev_loc[robot_id][0] = loc[robot_id][0];
		prev_loc[robot_id][1] = loc[robot_id][1];
		
		update_self_motion(msl,msr);


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
