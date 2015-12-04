#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "simulation.h"
#include "pso_ocba.h"



//Formation types
#define DEFAULT_FORMATION "line"    // Line formation as default






/*
 * Main function.
 */
int main(int argc, char *args[]) {
    
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
    initialize();
    reset();
    printf("Supervisor resetted.\n");
    send_init_poses();
    printf("Init poses sent.\n Chosen formation: %s.\n", formation);

    
    // infinite loop
    for(;;) {
        wb_robot_step(TIME_STEP);

        // every 10 TIME_STEP (640ms)
        if (simulation_duration % 10 == 0) {
            send_current_poses();

            //////////////////////////////////////////////////
          	// Here we should then add the fitness function //
            //////////////////////////////////////////////////
          	
        }
        simulation_duration += TIME_STEP;
    }
}





