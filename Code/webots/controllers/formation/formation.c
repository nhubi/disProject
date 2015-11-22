#include <stdio.h>
#include <math.h>
#include <string.h>

#include "utils.h"
#include "robot_state.h"

// Motorschemas
#include "ms_avoid_static_obstacles.h"
#include "ms_move_to_goal.h"
#include "ms_keep_formation.h"





#define MIGRATION_WEIGHT    0.1     // Wheight of attraction towards the common goal. default 0.01





// Each sensor's weight for Braitenberg 
// TODO: move Braitenberg to ms_avoid_static_obstacles
int braitenberg_weights_right[16] = {4,3,2,1,0,0,0,0};
int braitenberg_weights_left[16]  = {0,0,0,0,1,2,3,4};








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





//TODO: replace reynolds rules with keep_formation and avoid_moving_obstacles motorschemes. 
/*
 * Update speed according to Reynold's rules
 */
void reynolds_rules() {
	int i, j;			// Loop counters
	float avg_loc[2] = {0,0};	// Flock average positions
	float avg_speed[2] = {0,0};	// Flock average speeds
	float cohesion[2] = {0,0};
	float dispersion[2] = {0,0};
	float consistency[2] = {0,0};
	
	/* Compute averages over the whole flock */
	for(i=0; i<FORMATION_SIZE; i++) {
		if (i == robot_id)
		{
			// don't consider yourself for the average
			continue;
		}
		for (j=0;j<2;j++)
		{
			avg_speed[j] += speed[i][j];
			avg_loc[j] += loc[i][j];
		}
	}
	
	for (j=0;j<2;j++)
	{
		avg_speed[j] /= FORMATION_SIZE-1;
		avg_loc[j] /= FORMATION_SIZE-1;
	}
	
	/* Reynold's rules */
	
	/* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
//	for (j=0;j<2;j++) {
//		// If center of mass is too far
//		if (fabsf(loc[robot_id][j]-avg_loc[j]) > RULE1_THRESHOLD)
//		{
//			cohesion[j] = avg_loc[j] - loc[robot_id][j];   // Relative distance to the center of the swarm
//		}
//	}
	
	
	/* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
//	for (k=0;k<FORMATION_SIZE;k++) {
//		if (k != robot_id) {        // Loop on flockmates only
//			// If neighbor k is too close (Euclidean distance)
//			if (pow(loc[robot_id][0]-loc[k][0],2)+pow(loc[robot_id][1]-loc[k][1],2) < RULE2_THRESHOLD)
//			{
//				for (j=0;j<2;j++)
//				{
//					dispersion[j] = loc[robot_id][j] -loc[k][j];	// Relative distance to k
//				}
//			}
//		}
//	}
	
	/* Rule 3 - Consistency/Alignment: match the speeds of flockmates */
//	for (j=0;j<2;j++)
//	{
//		consistency[j] =  speed[robot_id][j]- avg_speed[j]; 		  // difference speed to the average
//	}
//	
//	// aggregation of all behaviors with relative influence determined by weights
//         printf("Migr %f %f\n",migr[0],migr[1]);
//	for (j=0;j<2;j++)
//	{
//		speed[robot_id][j] = cohesion[j] * RULE1_WEIGHT;
//		speed[robot_id][j] +=  dispersion[j] * RULE2_WEIGHT;
//		speed[robot_id][j] +=  consistency[j] * RULE3_WEIGHT;
		
//		speed[robot_id][j] += (migr[j]-loc[robot_id][j]) * MIGRATION_WEIGHT;
//	}
//	get_move_to_goal_vector(10);
    
    compute_unit_center();
    //printf("Unit center: (%f, %f)\n",unit_center[0], unit_center[1]);

    float dir_goal[2];
	get_move_to_goal_vector(dir_goal, robot_id);
    speed[robot_id][0] = dir_goal[0];
    speed[robot_id][1] = dir_goal[1];
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
//		bmsl+=66; bmsr+=72;
		


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
		
//		printf("speedBeforeReynolds %f %f\n",speed[robot_id][0],speed[robot_id][1]);
		
		// Reynold's rules with all previous info (updates the speed[][] table)
		reynolds_rules();
		
//		printf("speed %f %f\n",speed[robot_id][0],speed[robot_id][1]);
		
		// Compute wheels speed from Reynold's speed
		compute_wheel_speeds(&msl, &msr);
//		printf("wheelSpeed %d %d\n",msl,msr);


		// Adapt speed instinct to distance sensor values
		if (sum_sensors > NB_SENSORS*MIN_SENS) {
			msl -= msl*max_sens/(2*MAX_SENS);
			msr -= msr*max_sens/(2*MAX_SENS);
		}
		
//		printf("Wheel speed after adapt speed %d %d\n",msl,msr);

		
		// Add Braitenberg
		msl += bmsl;
		msr += bmsr;
		
//		printf("Wheel speed after braitenberg %d %d\n",msl,msr);


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
  
