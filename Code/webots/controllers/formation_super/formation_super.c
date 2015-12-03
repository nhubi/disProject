#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "simulation.h"
#include "pso_ocba.h"



//Formation types
#define DEFAULT_FORMATION "line"    // Line formation as default



int t;





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





