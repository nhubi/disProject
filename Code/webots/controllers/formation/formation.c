#include <stdio.h>
#include <math.h>
#include <string.h>

#include "utils.h"
#include "robot_state.h"
#include "local_communications.h"

// Motorschemas
#include "ms_move_to_goal.h"
#include "ms_keep_formation.h"
#include "ms_avoid_static_obstacles.h"
#include "ms_avoid_robots.h"
#include "ms_noise.h"


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
    float dir_noise[2]           = {0, 0};

    bool goal_reached = false;


    // compute the direction vectors
    get_move_to_goal_vector(dir_goal, &goal_reached);
    get_keep_formation_vector(dir_keep_formation, dir_goal);
    get_stat_obst_avoidance_vector(dir_avoid_obstacles);
    get_avoid_robot_vector(dir_avoid_robot);
    get_noise_vector(dir_noise);


    // are we there already?
    if(goal_reached) {
        printf("Robot%d with goal_dir = (%f,%f) says \"WOOHOOOOO... Goal reached.\"\n", robot_id, dir_goal[0], dir_goal[1]);
        return;
    }

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
		speed[robot_id][d] += w_noise           * dir_noise[d]; 
     }
}





/* 
 * The main function
 */
int main(){

    int msl, msr;                   // Wheel speeds
    int rob_nb;                     // Robot number
    float rob_x, rob_z, rob_theta;  // Robot position and orientation
    char *inbuffer;                 // Buffer for the receiver node

    msl = 0; msr = 0;
	int useless_variable;       //TODO: @Stef, WTF is this? :'D
	


    reset();           // Resetting the robot
    initial_pos();     // Initializing the robot's position
    initial_weights(); //Initializing the robot's weights
	
	
    // Forever
    for(;;){

        // Get information from other robots
        int count = 0;
        while (wb_receiver_get_queue_length(receiver) > 0 && count < FORMATION_SIZE) {
            inbuffer = (char*) wb_receiver_get_data(receiver);
		    sscanf(inbuffer,"%d#%d#%f#%f#%f",&rob_nb,&useless_variable,&rob_x,&rob_z,&rob_theta);
			
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
            wb_receiver_next_packet(receiver);
            
            time_steps_since_start++;

            if(time_steps_since_start*64/1000 - (float)time_steps_since_start*64/1000 == 0 && time_steps_since_start*64/1000 % 10 == 0){
                printf("[%ds] Robot%d is still running.\n", time_steps_since_start*64/1000, robot_id);
            }
        }

	
        // compute current position according to motor speeds (msl, msr)
        //update_self_motion(msl,msr);
		
        // Send a ping to the other robots, receive their ping and use it for computing their position
        send_ping();
        process_received_ping_messages(robot_id);
        compute_other_robots_localisation(robot_id);

        // Compute the unit center of the flock
        compute_unit_center();
        
        // Get direction vectors from each motorscheme and combine them in speed table
        computeDirection();

        // Compute wheel speeds from speed vectors and forward them to differential motors
        compute_wheel_speeds(&msl, &msr);
        wb_differential_wheels_set_speed(msl,msr);


        // Continue one step
        wb_robot_step(TIME_STEP);
    }
}
