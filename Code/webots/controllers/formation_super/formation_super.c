#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#define FORMATION_SIZE  4	// Number of robots in formation
#define TIME_STEP      64	// [ms] Length of time step

//Formation types
#define DEFAULT_FORMATION "line"    // Line formation as default

WbNodeRef  robs[FORMATION_SIZE];            // Robots nodes
WbFieldRef robs_trans[FORMATION_SIZE];      // Robots translation fields
WbFieldRef robs_rotation[FORMATION_SIZE];   // Robots rotation fields
WbDeviceTag emitter;                        // Single emitter

float loc[FORMATION_SIZE][3];               // Location of everybody in the formation: X,Y and Theta

int formation_type;

// Variable for goal:
int offset;             // Offset of robots number
float migrx,migrz;      // Migration vector
float orient_migr;      // Migration orientation
WbNodeRef goal_id;      // Goal node
WbFieldRef goal_field;  // Goal translation field
int t;




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
 * Send to each robot its initial position the goal's position and the formation type. 
 */
void send_init_poses(void)
{
    char buffer[255];	// Buffer for sending data
    int i;
     
    for (i=0;i<FORMATION_SIZE;i++) {

        // Get robot's position
        loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0];       // X
        loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2];       // Z
        loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA

        migrx = wb_supervisor_field_get_sf_vec3f(goal_field)[0];    // X
        offset = wb_supervisor_field_get_sf_vec3f(goal_field)[1];   // Y
        migrz = wb_supervisor_field_get_sf_vec3f(goal_field)[2];    // Z
        orient_migr = -atan2f(migrx,migrz);                         // angle of migration urge
		
        if (orient_migr<0) {
            orient_migr+=2*M_PI; // Keep value between 0 and 2PI
        }
	
        // Send it out
        sprintf(buffer,"%1d#%f#%f#%f##%f#%f#%1d",i,loc[i][0],loc[i][1],loc[i][2],migrx,migrz,formation_type);
        wb_emitter_send(emitter,buffer,strlen(buffer));

        // Run one step
        wb_robot_step(TIME_STEP);
    }
}





/*
 * Main function.
 */
int main(int argc, char *args[]) {
    char buffer[255]; // buffer for sending data
    
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

	
    int i; // counter for inner for loop
    
    // infinite loop
    for(;;) {
        wb_robot_step(TIME_STEP);

        if (t%10 == 0) { // every 10 TIME_STEP (640ms)
            for (i=0;i<FORMATION_SIZE;i++) {
                // Get data
                loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0];       // X
                loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2];       // Z
                loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA

                // Sending positions to the robots
                sprintf(buffer,"%1d#%f#%f#%f#%f#%f#%1d",i+offset,loc[i][0],loc[i][1],loc[i][2], migrx, migrz, formation_type);
                wb_emitter_send(emitter,buffer,strlen(buffer));				
            }

            //////////////////////////////////////////////////
          	// Here we should then add the fitness function //
            //////////////////////////////////////////////////
          	
        }
        t += TIME_STEP;
    }
}





