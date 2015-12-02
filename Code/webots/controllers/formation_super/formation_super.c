#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#include "fitness.h"

#define FORMATION_SIZE  4	// Number of robots in formation
#define TIME_STEP      64	// [ms] Length of time step

//Formation types
#define DEFAULT_FORMATION "line"// Line formation as default

WbNodeRef  robs[FORMATION_SIZE];            // Robots nodes
WbFieldRef robs_trans[FORMATION_SIZE];      // Robots translation fields
WbFieldRef robs_rotation[FORMATION_SIZE];   // Robots rotation fields
WbDeviceTag emitter;                        // Single emitter

float loc[FORMATION_SIZE][3];               // Location of everybody in the formation: X,Y and Theta
float prev_loc[FORMATION_SIZE][3];          // Previous location of everybody
float speed[FORMATION_SIZE][3];             // Speed computed between the last 2 positions

int formation_type;

// Variable for goal:
int offset;         // Offset of robots number
float migrx,migrz;  // Migration vector
float orient_migr;  // Migration orientation
WbNodeRef goal_id;  // Goal node
WbFieldRef goal_field;    // Goal translation field
int t;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Declaration of all the paramethers that have to be modified by PSO. They have to be passed to the robot controllers. //

// motorschema weights
float w_goal;
float w_keep_formation;
float w_avoid_robot;
float w_avoid_obstacles;
float w_noise;

// Noise paramethers
int noise_gen_frequency;  // defines, after how many steps a new random vector should be generated
int fading;              // 1, if nice transition is wished from one random vector to the next, otherwise 0

// Thresholds
float avoid_obst_min_threshold;
float avoid_obst_max_threshold;
float move_to_goal_min_threshold;
float move_to_goal_max_threshold;
float avoid_robot_min_threshold;
float avoid_robot_max_threshold;
float keep_formation_min_threshold; // < min threshold: dead zone, no correction of direction
float keep_formation_max_threshold; // > max threshold: ballistic zone, full correction of direction
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//You can add the rest of the required variables in a similar fashion and initialize them similar to the robots in the simplest case!

/*----------------- for the goal and obstacles you might also need to ask the supervisor depending on the approach you take:
 1) absolute positions 2) relative distances based on the perception of the robot 
 approach 2 is more realistic and preferred. However approach 1 also exists incase you run into problems with implementing local perception
 most of the codes that you need for both approaches can be found in the labs (lab4 is particularly useful) 
Think about the obstacles, do you need to know their exact position? 
how can you differentiate between a robot and an obstacle if you use local perception? in one of your labs you had communication between robots where the distance 
  between a pair of robots could be found
if you decide to use the absolute positions of all the obstacles through the supervisor,how can you make it more realistic? e.g define a region around the robot where you include the obstacles? 
The more realistic and less dependent on the supervisor you are, the better.
goal is the red cylinder which can be moved around between different trials of your code
*/




/*
 * Initialize robot positions and devices
 */
void reset(void) {
	wb_robot_init();
	emitter = wb_robot_get_device("emitter");
	if (emitter==0) printf("missing emitter\n");
	
	char rob[5] = "rob0";
	int i;
	for (i=0;i<FORMATION_SIZE;i++) {
		sprintf(rob,"rob%d",i);
		robs[i]          = wb_supervisor_node_get_from_def(rob);
		robs_trans[i]    = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
	}
	goal_id = wb_supervisor_node_get_from_def("goal-node");
	goal_field = wb_supervisor_node_get_field(goal_id,"translation");
}





/*
 * this is sending the robots information only, for the obstacles and the goal you should complete
 * this function if you decide to go with absolute positions, however you can think about the 
 * relative distances which are more realistic and try to use that data (you can find the codes for 
 * doing this in lab4)
 */

void send_init_poses(void)
{
    char buffer[255];	// Buffer for sending data
    int i;
     
    for (i=0;i<FORMATION_SIZE;i++) {
        // Get data
        loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0];       // X
        loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2];       // Z
        loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
//		printf("Supervisor %f %f %f\n",loc[i][0],loc[i][1],loc[i][2]);

        migrx = wb_supervisor_field_get_sf_vec3f(goal_field)[0];  //X
        offset = wb_supervisor_field_get_sf_vec3f(goal_field)[1]; //Y
        migrz = wb_supervisor_field_get_sf_vec3f(goal_field)[2];  //Z
        //printf("Migratory instinct: (%f, %f)\n",migrx,migrz);
        orient_migr = -atan2f(migrx,migrz); // angle of migration urge
		
        if (orient_migr<0) {
            orient_migr+=2*M_PI; // Keep value between 0 and 2PI
        }
	
        // Send it out
        // the scond paramether is 0 if we are sending poses, 1 if we are sending weights
        sprintf(buffer,"%1d#%1d#%f#%f#%f##%f#%f#%d#",i,0,loc[i][0],loc[i][1],loc[i][2],migrx,migrz,formation_type);
        //printf("%1d#%f#%f#%f\n",i,loc[i][0],loc[i][1],loc[i][2]);
        wb_emitter_send(emitter,buffer,strlen(buffer));
        
        printf("%d\n",formation_type);

        // Run one step
        wb_robot_step(TIME_STEP);
    }
}

void send_weights(void)
{
    char buffer[255];	// Buffer for sending data
    int i;
     
    for (i=0;i<FORMATION_SIZE;i++) {
	
        // Send it out
        sprintf(buffer,"%1d#%1d#%f#%f#%f#%f#%f#%1d#%1d#%f#%f#%f#%f#%f#%f#%f#%f#",i,1,
            w_goal,w_keep_formation,w_avoid_robot,w_avoid_obstacles,w_noise,
            noise_gen_frequency,fading,
            avoid_obst_min_threshold,avoid_obst_max_threshold,move_to_goal_min_threshold,move_to_goal_max_threshold,
            avoid_robot_min_threshold,avoid_robot_max_threshold,keep_formation_min_threshold,keep_formation_max_threshold);

        wb_emitter_send(emitter,buffer,strlen(buffer));

        // Run one step
        wb_robot_step(TIME_STEP);
    }
}

int simulation_is_ended(void) {
	float centre_x=0;
	float centre_y=0;
	float distance_to_goal=0;
	
	
	// compute the formation center
	int i;
	for (i=0; i<FORMATION_SIZE; i++) {
		centre_x+=loc[i][0];
		centre_y+=loc[i][1];
	}
	centre_x/=FORMATION_SIZE;
	centre_y/=FORMATION_SIZE;
	
	distance_to_goal+=(centre_x-migrx)*(centre_x-migrx);
	distance_to_goal+=(centre_y-migrz)*(centre_y-migrz);
	distance_to_goal=sqrt(distance_to_goal);
		
	if (distance_to_goal<0.2) {
		return 1;
	}
	
	return 0;
}

/*
 * Main function.
 */
 
int main(int argc, char *args[]) {
    char buffer[255]; // buffer for sending data
	
	//////////////////
	// DEFINE GOAL  //
	//////////////////

	/*if (argc == 4) {
          	migrx = atoi(args[1]);
          	offset = atoi(args[2]); // migration urge on x direction
          	migrz = atoi(args[3]); // migration urge on z direction
          	printf("Migratory instinct: (%f, %f)\n",migrx,migrz);
	} else {
          	printf("No goal defined in supervisor->controllerArgs");
          	return 1;
	}
    orient_migr = -atan2f(migrx,migrz); // angle of migration urge
  	if (orient_migr<0) {
          	orient_migr+=2*M_PI; // Keep value between 0 and 2PI
	}*/
    
    char* formation = DEFAULT_FORMATION; 
    if(argc == 2) {
          formation_type = atoi(args[1]); // The type of formation is decided by 
                               // the user through the world
    }
    
    if(formation_type == 0)
          formation = "line";
    else if(formation_type == 1)
          formation = "column";
    else if(formation_type == 2)
          formation = "wedge";
    else if(formation_type == 3)
          formation = "diamond";
    
    // reset and communication part
    reset();
    printf("Supervisor resetted.\n");
    send_init_poses(); //this function is here as an example for communication using the supervisor
    printf("Init poses sent.\n Chosen formation: %s.\n", formation);
    
    // declare the starting weights
    
    // each motorschema's weight
    w_goal            = 1;
    w_keep_formation  = 5;
    w_avoid_robot     = 1;
    w_avoid_obstacles = 5;
    w_noise           = 2;

    // thresholds
    avoid_obst_min_threshold     =  60;
    avoid_obst_max_threshold     = 200;
    move_to_goal_min_threshold   =   0.1;
    move_to_goal_max_threshold   =   0.5;
    avoid_robot_min_threshold    =   0.05;
    avoid_robot_max_threshold    =   0.1;
    keep_formation_min_threshold =   0.03;
    keep_formation_max_threshold =   0.1;


    // noise parameters
    noise_gen_frequency = 10;
    fading              = 1;
    
    // sending weights
    send_weights();
    printf("Weights sent\n");

	// setting up the fitness computation
	reset_fitness_computation(FORMATION_SIZE);
	
    int i; // necessary declaration - not possible to declare it inside the for loop
    // infinite loop
    for(;;) {
        wb_robot_step(TIME_STEP);

        if (t%10 == 0) { // every 10 TIME_STEP (640ms)
            for (i=0;i<FORMATION_SIZE;i++) {
                // Get data
                loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0];       // X
                loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2];       // Z
                loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA

				// Process the value
				update_fitness_computation_for_robot(loc,prev_loc,speed,i,TIME_STEP*10);

                // Sending positions to the robots
                sprintf(buffer,"%1d#%1d#%f#%f#%f#%f#%f#%1d#",i+offset,0,loc[i][0],loc[i][1],loc[i][2], migrx, migrz, formation_type);
                wb_emitter_send(emitter,buffer,strlen(buffer));
				
            }
			
            //////////////////////////////////////////////////
          	// Here we should then add the fitness function //
            //////////////////////////////////////////////////
			if (simulation_is_ended()) {
				break;
			}
          	
        }
        t += TIME_STEP;

    }
    
    float fitness=compute_fitness(FORMATION_SIZE);
    printf("fitness = %f\n",fitness);
}





