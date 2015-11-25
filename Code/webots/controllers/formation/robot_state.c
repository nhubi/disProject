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





/*
 * Reset the robot's devices and get its ID
 */
void reset(void) {
	wb_robot_init();

	receiver = wb_robot_get_device("receiver");
	emitter  = wb_robot_get_device("emitter");
	if (emitter == 0)printf("missing emitter\n");
	
	int i;
	char s[4]="ps0";
	for(i=0; i<NB_SENSORS; i++) {
		dist_sens[i] = wb_robot_get_device(s);  // the device name is specified in the world file
		s[2]++;                                 // increases the device number
	}

	robot_name = (char*)wb_robot_get_name(); 

    // enable distance sensors
	for(i=0; i<NB_SENSORS; i++) {
		wb_distance_sensor_enable(dist_sens[i],64);
	}

    // enable receiver
	wb_receiver_enable(receiver,64);

	//Reading the robot's name.
	sscanf(robot_name,"rob%d",&robot_id_u);   // read robot id from the robot's name
	
	robot_id = robot_id_u%FORMATION_SIZE;   // normalize between 0 and FORMATION_SIZE-1
	
	for(i=0; i<FORMATION_SIZE; i++) {
		initialized[i] = 0;                 // Set initialization to 0 (= not yet initialized)
	}
	
	
	printf("Reset: robot %d\n",robot_id);
}





/*
 * Initialize robot's position
 * (Stefano) The robot receive infomation on his ID, position and the goal
 */
void initial_pos(void){
	char *inbuffer;
	int rob_nb;
	float rob_x, rob_z, rob_theta; // Robot position and orientation
	
	
	while (initialized[robot_id] == 0) {
		
		// wait for message
		while (wb_receiver_get_queue_length(receiver) == 0)	{
            	wb_robot_step(TIME_STEP);
        }
		
		inbuffer = (char*) wb_receiver_get_data(receiver);
		sscanf(inbuffer,"%d#%f#%f#%f##%f#%f",&rob_nb,&rob_x,&rob_z,&rob_theta,&migr[0],&migr[1]);
		// Only info about self will be taken into account at first.
		
		//robot_nb %= FORMATION_SIZE;
		if (rob_nb == robot_id)
		{
			// Initialize self position
			loc[rob_nb][0] = rob_x;     // x-position
			loc[rob_nb][1] = rob_z;     // z-position
			loc[rob_nb][2] = rob_theta; // theta
			prev_loc[rob_nb][0] = loc[rob_nb][0];
			prev_loc[rob_nb][1] = loc[rob_nb][1];
			initialized[rob_nb] = 1;    // initialized = true
			//printf("initialPos %f %f %f %f\n",loc[rob_nb][0],loc[rob_nb][1],prev_loc[rob_nb][0],prev_loc[rob_nb][1]);
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
//	printf("reynolds %f %f %f\n",migr[0],loc[robot_id][0],speed[robot_id][0]);

	// Compute wanted position from Reynold's speed and current location
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
	
//	printf("Wheel speed after reynolds %d %d\n",*msl,*msr);
}



