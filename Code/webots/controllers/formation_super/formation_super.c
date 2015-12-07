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
    reset();
    printf("Supervisor resetted.\n");
    send_init_poses(); //this function is here as an example for communication using the supervisor
    printf("Init poses sent.\n Chosen formation: %s.\n", formation);

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

    // TESTING PSO HERE


    int d;

    parameter_ranges[0][0] = -1;
    parameter_ranges[0][1] =  1;
    parameter_ranges[1][0] = -1;
    parameter_ranges[1][1] =  4;
    parameter_ranges[2][0] = -1;
    parameter_ranges[2][1] =  4;
    parameter_ranges[3][0] =  3;
    parameter_ranges[3][1] =  9.2;
    parameter_ranges[4][0] = -0.8;
    parameter_ranges[4][1] = -0.7;

    float optimal_params[DIMENSIONALITY];
    pso_ocba(optimal_params);

    printf("The optimal parameters are: \n");
    for(d = 0; d < DIMENSIONALITY; d++){
        printf("    Dimension %d: %1.4f\n", d, optimal_params[d]);
    }

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

    
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
