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
void computeDirection(void){

    // direction vector for each motorschema
    float dir_goal[2]            = {0, 0};
    float dir_keep_formation[2]  = {0, 0};
    float dir_avoid_robot[2]     = {0, 0};
    float dir_avoid_obstacles[2] = {0, 0};
    float dir_noise[2]           = {0, 0};


    // compute the direction vectors
    get_move_to_goal_vector(dir_goal);
    get_keep_formation_vector(dir_keep_formation, dir_goal);
    get_stat_obst_avoidance_vector(dir_avoid_obstacles);
    get_avoid_robot_vector(dir_avoid_robot);
    get_noise_vector(dir_noise);

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

    // make sure that the resulting vector is not too long. Otherwise, it would permanently get 
    // limited in compuete_wheel_speeds(). 
    // To avoid any limiting, the max length of the vector should be around 1.5. However, choosing 4
    // is still working pretty well (it occurs rarely that the vector's norm is so large).
    for(d = 0; d < 2; d++){
        speed[robot_id][d] /= (w_goal + w_keep_formation + w_avoid_robot + w_avoid_obstacles + w_noise);
        speed[robot_id][d] *= 4;
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

    bool initialized = false;       // becomes true as soon as robot receives weights.
    int msg_type;                   // detected message type

    msl = 0; msr = 0;

    reset(); // Reset the robot

    // Forever
    for(;;){
        int count = 0;
        while (count < FORMATION_SIZE) {
            
            // wait for message
            while (wb_receiver_get_queue_length(receiver) == 0){
            	wb_robot_step(TIME_STEP);
            }

            inbuffer = (char*) wb_receiver_get_data(receiver);

            // Check, what kind of message you've received. Then process the msg accordingly.
            switch(inbuffer[0] - '0'){
                case MSG_POSITION_INIT:
                    init_pos(inbuffer);
                    count++;
                    
                    // when new positions are received, robot needs to wait for new params.
                    initialized = false;
                    break;

                case MSG_INIT_PARAMS:
                    init_params(inbuffer);
                    count++;

                    // when params are received, robot can start.
                    initialized = true;
                    break;

                case MSG_POSITION:
                    sscanf(inbuffer, "%d##%1d#%f#%f#%f##%f#%f",
                        &msg_type,
                        &rob_nb,
                        &rob_x,
                        &rob_z,
                        &rob_theta,
                        &migr[0],
                        &migr[1]);
                    rob_nb %= FORMATION_SIZE;
                    loc[rob_nb][0] = rob_x;
                    loc[rob_nb][1] = rob_z;
                    loc[rob_nb][2] = rob_theta;
                    prev_loc[rob_nb][0] = loc[rob_nb][0];
                    prev_loc[rob_nb][1] = loc[rob_nb][1];
                    count++;
                    break;

                default:
                    printf("WARNING: Unknown message type.");
            }
            wb_receiver_next_packet(receiver);
        }
		
        if(initialized){
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
        }

            // Continue one step
            wb_robot_step(TIME_STEP);
            time_steps_since_start++;
    }
    
    return 0;
}
