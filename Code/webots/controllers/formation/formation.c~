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



WbDeviceTag ds[NB_SENSORS];	    // Handle for the infra-red distance sensors
WbDeviceTag receiver;		    // Handle for the receiver node
WbDeviceTag emitter;		    // Handle for the emitter node

int robot_id;					//  robot ID
char* robot_name; 


/*------------ you should add all the rest of the required variables here and add your functions or modify and complete the existing ones
*/

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
	for(i=0;i<NB_SENSORS;i++)
         wb_distance_sensor_enable(ds[i],64);
 
	wb_receiver_enable(receiver,64);

	sscanf(robot_name,"rob%d",&robot_id); // read robot id from the robot's name
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


// the main function
int main(){ 
	
   
  
  
	reset();	
	
	// Forever
	for(;;){
  	 /* Get information */
    
  	 	
          // Receive and process the required messages					
  	 
  	    
  		  		
  	 // apply your algorithm and compute the wheel speeds
          
          
          
          // set your speeds here I just put a constant number which you need to overwrite
  	 wb_differential_wheels_set_speed(20,20);
      
  	 // Continue one step
  	 wb_robot_step(TIME_STEP);
	}
}  
  
