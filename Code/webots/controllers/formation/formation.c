#include <stdio.h>
#include <math.h>
#include <string.h>

#include "utils.h"
#include "robot_state.h"

// Motorschemas
#include "ms_move_to_goal.h"
#include "ms_keep_formation.h"
#include "ms_avoid_static_obstacles.h"





#define MIGRATION_WEIGHT 0.1     // Wheight of attraction towards the common goal. default 0.01





// Each sensor's weight for Braitenberg 
// TODO: move Braitenberg to ms_avoid_static_obstacles
int braitenberg_weights_right[16] = {0,0,0,0,1,2,3,4};
int braitenberg_weights_left[16]  = {4,3,2,1,0,0,0,0};

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
 * Combines the vectors from the 4 motorschemas and computes the corresponding wheel speeds.
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

    //TODO: - call get_*_vector methods
    //      - combine them
    //      - compute wheel speed

    get_move_to_goal_vector(dir_goal, robot_id);
    get_keep_formation_vector(dir_keep_formation, robot_id);
    get_stat_obst_avoidance_vector(dir_avoid_obstacles, robot_id);


    int d = 0;
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

    printf("\nDirection of robot%d = (%f,%f) \n",robot_id,speed[robot_id][0],speed[robot_id][1]);
}





/* 
 * The main function
 */
int main(){
	// for I14, sending current position to neighbors
	// char outbuffer[255];
	
	int msl, msr;                   // Wheel speeds
	int bmsl, bmsr, sum_sensors;    // Braitenberg parameters
	int i;                          // Loop counter
	int rob_nb;                     // Robot number
	float rob_x, rob_z, rob_theta;  // Robot position and orientation
	int distances[NB_SENSORS];      // Array for the distance sensor readings
	char *inbuffer;                 // Buffer for the receiver node
	int max_sens;                   // Store highest sensor value
    
	
	
	// In this initialization the common goal is communicated to the robot
	reset();        // Resetting the robot
	initial_pos();  // Initializing the robot's position
	
	msl = 0; msr = 0;
	max_sens = 0;
	

	
	// Forever
	for(;;){
        // TODO: move Braitenberg to ms_avoid_static_obstacles
        // TODO: write function that combines vectors received computed from motorschemas...
        //       The resulting vector needs to be translated into wheel speeds.

/*
        // initialize variables
		bmsl = 0; 
        bmsr = 0;
		sum_sensors = 0;
		max_sens = 0;

        // Braitenberg
		for(i=0;i<NB_SENSORS;i++) {
			distances[i]=wb_distance_sensor_get_value(dist_sens[i]); //Read sensor values
			sum_sensors += distances[i]; // Add up sensor values
			max_sens = max_sens>distances[i]?max_sens:distances[i]; // Check if new highest sensor value
			
			// Weighted sum of distance sensor values for Braitenberg vehicle
			bmsr += braitenberg_weights_right[i] * distances[i];
			bmsl += braitenberg_weights_left[i]  * distances[i];
		}
		
		// Adapt Braitenberg values (empirical tests)
		bmsl/=MIN_SENS; bmsr/=MIN_SENS;
		
*/


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

                // Otherwise, update the its position.
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
		

		// Compute self position & speed
		prev_loc[robot_id][0] = loc[robot_id][0];
		prev_loc[robot_id][1] = loc[robot_id][1];
		
		update_self_motion(msl,msr);



		speed[robot_id][0] = (1/TIME_STEP/1000)*(loc[robot_id][0]-prev_loc[robot_id][0]);
		speed[robot_id][1] = (1/TIME_STEP/1000)*(loc[robot_id][1]-prev_loc[robot_id][1]);
		
		// Reynold's rules with all previous info (updates the speed[][] table)
		// reynolds_rules();


        // Get direction vectors from each motorscheme and combine them
        computeDirection();

		
		// Compute wheels speed from Reynold's speed
		compute_wheel_speeds(&msl, &msr);
//		printf("wheelSpeed %d %d\n",msl,msr);


		// Adapt speed instinct to distance sensor values
		if (sum_sensors > NB_SENSORS*MIN_SENS) {
			msl -= msl*max_sens/(2*MAX_SENS);
			msr -= msr*max_sens/(2*MAX_SENS);
        }
		
/*
		// Add Braitenberg
		msl += bmsl;
		msr += bmsr;
*/


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
