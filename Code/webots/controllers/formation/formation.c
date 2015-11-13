#include <stdio.h>
#include <math.h>
#include <string.h>

#include <webots/robot.h>

#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#define NB_SENSORS	  8	    // Number of distance sensors
#define MIN_SENS          350       // Minimum sensibility value
#define MAX_SENS          4096      // Minimum sensibility value
#define MAX_SPEED         800       // Maximum speed
#define FORMATION_SIZE    4	    // Size of flock
#define TIME_STEP	  64	    // [ms] Length of time step

#define AXLE_LENGTH 	0.052	   // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS	0.00628	   // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS	0.0205	   // Wheel radius (meters)
#define DELTA_T		    0.064	   // Time-step (seconds)

/////////////////////////////////////////////
// from here I've inserted the paramethers //

//#define RULE1_THRESHOLD     0.20   // Threshold to activate aggregation rule. default 0.20
//#define RULE1_WEIGHT        0.6	   // Weight of aggregation rule. default 0.6

//#define RULE2_THRESHOLD     0.15   // Threshold to activate dispersion rule. default 0.15
//#define RULE2_WEIGHT        1.0	   // Weight of dispersion rule. default 1.0

//#define RULE3_WEIGHT        0.10   // Weight of consistency rule. default 0.1

#define MIGRATION_WEIGHT    0.01   // Wheight of attraction towards the common goal. default 0.01

/////////////////////////////////////////////

// I don't know what's this
int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // Maze
//int e_puck_matrix[16] = {17,29,12,10,8,-38,-56,-76,-72,-58,-36,8,10,12,28,18}; // Crossing


WbDeviceTag ds[NB_SENSORS];	    // Handle for the infra-red distance sensors
WbDeviceTag receiver;		    // Handle for the receiver node
WbDeviceTag emitter;		    // Handle for the emitter node

int robot_id_u, robot_id;	// Unique and normalized (between 0 and FORMATION_SIZE-1), robot ID
char* robot_name; 



/*------------ you should add all the rest of the required variables here and add your functions or modify and complete the existing ones
*/
float loc[FORMATION_SIZE][3];	// X, Z, Theta of all robots
float prev_loc[FORMATION_SIZE][3];	// Previous X, Z, Theta values
float speed[FORMATION_SIZE][2];	// Speeds calculated with Reynold's rules
int initialized[FORMATION_SIZE];	// != 0 if initial positions have been received
float migr[2];	                // Migration vector

/*
 * Reset the robot's devices and get its ID
 */
static void reset() {
	wb_robot_init();

	receiver = wb_robot_get_device("receiver");
	emitter = wb_robot_get_device("emitter");
	if (emitter ==0 )printf("missing emitter\n");
	
	int i;
	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
		s[2]++;							// increases the device number
	}
	robot_name=(char*) wb_robot_get_name(); 
	for(i=0;i<NB_SENSORS;i++) {
		wb_distance_sensor_enable(ds[i],64);
	}
	wb_receiver_enable(receiver,64);

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"rob%d",&robot_id); // read robot id from the robot's name
	
	robot_id = robot_id_u%FORMATION_SIZE;	  // normalize between 0 and FORMATION_SIZE-1
	
	for(i=0; i<FORMATION_SIZE; i++) {
		initialized[i] = 0; 		  // Set initialization to 0 (= not yet initialized)
	}
	
	
	printf("Reset: robot %d\n",robot_id);
}


/*
 each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 This is useful if you want to use relative range and bearing which is also used in the paper. this would be 
 more realistic and less dependent on the supervisor, Try to make this work 
 use process_received_ping_messages() function in lab 4 as a base to calculate range and bearing to the other robots
*/
void send_ping(void)  {
         char out[10];
	strcpy(out,robot_name);  // in the ping message we send the name of the robot.
	wb_emitter_send(emitter,out,strlen(out)+1); 
}

/*
 * Keep given float number within interval {-limit, limit}
 */
void limitf(float *number, int limit) {
	if (*number > limit)
		*number = (float)limit;
	if (*number < -limit)
		*number = (float)-limit;
}

/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(int *number, int limit) {
	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}

/*
 * Updates robot position with wheel speeds
 * Used for odometry
 */
void update_self_motion(int msl, int msr) {
	float theta = loc[robot_id][2];
	
	// Compute deltas of the robot
	float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
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
	// Compute wanted position from Reynold's speed and current location
	float x = speed[robot_id][0]*cosf(loc[robot_id][2]) - speed[robot_id][1]*sinf(loc[robot_id][2]); // x in robot coordinates
	float z = -speed[robot_id][0]*sinf(loc[robot_id][2]) - speed[robot_id][1]*cosf(loc[robot_id][2]); // z in robot coordinates
	
	float Ku = 0.2;   // Forward control coefficient
	float Kw = 10.0;  // Rotational control coefficient
	float range = sqrtf(x*x + z*z);	  // Distance to the wanted position
	float bearing = -atan2(x, z);	  // Orientation of the wanted position
	
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

/*
 * Initialize robot's position
 */
void initial_pos(void){
	char *inbuffer;
	int rob_nb;
	float rob_x, rob_z, rob_theta; // Robot position and orientation
	
	
	
	while (initialized[robot_id] == 0) {
		
		// wait for message
		while (wb_receiver_get_queue_length(receiver) == 0)	wb_robot_step(TIME_STEP);
		
		inbuffer = (char*) wb_receiver_get_data(receiver);
		sscanf(inbuffer,"%d#%f#%f#%f##%f#%f",&rob_nb,&rob_x,&rob_z,&rob_theta, &migr[0], &migr[1]);
		// Only info about self will be taken into account at first.
		
		//robot_nb %= FORMATION_SIZE;
		if (rob_nb == robot_id)
		{
			// Initialize self position
			loc[rob_nb][0] = rob_x; 		// x-position
			loc[rob_nb][1] = rob_z; 		// z-position
			loc[rob_nb][2] = rob_theta; 		// theta
			prev_loc[rob_nb][0] = loc[rob_nb][0];
			prev_loc[rob_nb][1] = loc[rob_nb][1];
			initialized[rob_nb] = 1; 		// initialized = true
		}
		wb_receiver_next_packet(receiver);
	}
}

/*
 * Update speed according to Reynold's rules
 */
void reynolds_rules() {
	int i, j, k;			// Loop counters
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
//	for (j=0;j<2;j++)
//	{
//		speed[robot_id][j] = cohesion[j] * RULE1_WEIGHT;
//		speed[robot_id][j] +=  dispersion[j] * RULE2_WEIGHT;
//		speed[robot_id][j] +=  consistency[j] * RULE3_WEIGHT;
//		speed[robot_id][j] += (migr[j]-loc[robot_id][j]) * MIGRATION_WEIGHT;
//	}
}


// the main function
int main(){
	// for I14, sending current position to neighbors
	// char outbuffer[255];
	
	int msl, msr;			// Wheel speeds
	int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
	int i;				// Loop counter
	int rob_nb;			// Robot number
	float rob_x, rob_z, rob_theta;  // Robot position and orientation
	int distances[NB_SENSORS];	// Array for the distance sensor readings
	char *inbuffer;			// Buffer for the receiver node
	int max_sens;			// Store highest sensor value
	
	reset();			// Resetting the robot
	initial_pos();			// Initializing the robot's position
	
	msl = 0; msr = 0;
	max_sens = 0;

	
	
	

	
	// Forever
	for(;;){
  	 /* Get information */
		bmsl = 0; bmsr = 0;
		sum_sensors = 0;
		max_sens = 0;
		/* Braitenberg */
		for(i=0;i<NB_SENSORS;i++) {
			distances[i]=wb_distance_sensor_get_value(ds[i]); //Read sensor values
			sum_sensors += distances[i]; // Add up sensor values
			max_sens = max_sens>distances[i]?max_sens:distances[i]; // Check if new highest sensor value
			
			// Weighted sum of distance sensor values for Braitenberg vehicle
			bmsr += e_puck_matrix[i] * distances[i];
			bmsl += e_puck_matrix[i+NB_SENSORS] * distances[i];
		}
		
		// Adapt Braitenberg values (empirical tests)
		bmsl/=MIN_SENS; bmsr/=MIN_SENS;
		bmsl+=66; bmsr+=72;
		
		/* Get information */
		int count = 0;
		while (wb_receiver_get_queue_length(receiver) > 0 && count < FORMATION_SIZE) {
			inbuffer = (char*) wb_receiver_get_data(receiver);
			sscanf(inbuffer,"%d#%f#%f#%f",&rob_nb,&rob_x,&rob_z,&rob_theta);
			
			if ((int) rob_nb/FORMATION_SIZE == (int) robot_id/FORMATION_SIZE) {
				rob_nb %= FORMATION_SIZE;
				if (initialized[rob_nb] == 0) {
					// Get initial positions
					loc[rob_nb][0] = rob_x; //x-position
					loc[rob_nb][1] = rob_z; //z-position
					loc[rob_nb][2] = rob_theta; //theta
					prev_loc[rob_nb][0] = loc[rob_nb][0];
					prev_loc[rob_nb][1] = loc[rob_nb][1];
					initialized[rob_nb] = 1;
				} else {
					// Get position update
					//				printf("\n got update robot[%d] = (%f,%f) \n",rob_nb,loc[rob_nb][0],loc[rob_nb][1]);
					prev_loc[rob_nb][0] = loc[rob_nb][0];
					prev_loc[rob_nb][1] = loc[rob_nb][1];
					loc[rob_nb][0] = rob_x; //x-position
					loc[rob_nb][1] = rob_z; //z-position
					loc[rob_nb][2] = rob_theta; //theta
				}
				
				speed[rob_nb][0] = (1/DELTA_T)*(loc[rob_nb][0]-prev_loc[rob_nb][0]);
				speed[rob_nb][1] = (1/DELTA_T)*(loc[rob_nb][1]-prev_loc[rob_nb][1]);
				count++;
			}
			
			wb_receiver_next_packet(receiver);
		}

		
          // Receive and process the required messages
		
		
		
		// apply your algorithm and compute the wheel speeds
		
		// Compute self position & speed
		prev_loc[robot_id][0] = loc[robot_id][0];
		prev_loc[robot_id][1] = loc[robot_id][1];
		
		update_self_motion(msl,msr);
		
		speed[robot_id][0] = (1/DELTA_T)*(loc[robot_id][0]-prev_loc[robot_id][0]);
		speed[robot_id][1] = (1/DELTA_T)*(loc[robot_id][1]-prev_loc[robot_id][1]);
		
		// Reynold's rules with all previous info (updates the speed[][] table)
		reynolds_rules();
		
		// Compute wheels speed from Reynold's speed
		compute_wheel_speeds(&msl, &msr);
		
		// Adapt speed instinct to distance sensor values
		if (sum_sensors > NB_SENSORS*MIN_SENS) {
			msl -= msl*max_sens/(2*MAX_SENS);
			msr -= msr*max_sens/(2*MAX_SENS);
		}
		
		// Add Braitenberg
		msl += bmsl;
		msr += bmsr;

		
		// set your speeds here I just put a constant number which you need to overwrite
  	    wb_differential_wheels_set_speed(20,20);
		
		// Send current position to neighbors, uncomment for I14, don't forget to uncomment the declration of "outbuffer" at the begining of this function.
		// sprintf(outbuffer,"%1d#%f#%f#%f",robot_id,loc[robot_id][0],loc[robot_id][1], loc[robot_id][2]);
		// wb_emitter_send(emitter,outbuffer,strlen(outbuffer));
	
		// Continue one step
		wb_robot_step(TIME_STEP);
	}
}  
  
