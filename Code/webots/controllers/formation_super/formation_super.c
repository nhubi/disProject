#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "simulation.h"
#include "fitness.h"
#include "pso_ocba.h"





//Formation types
#define DEFAULT_FORMATION "line"    // Line formation as default





/*
 * Main function.
 */
int main(int argc, char *args[]) {

    // The type of formation is decided by the user through the world
    char* formation = DEFAULT_FORMATION; 
    if(argc == 2) {
        formation_type = atoi(args[1]); 
    }
    
    if(formation_type == 0)
          formation = "line";
    else if(formation_type == 1)
          formation = "column";
    else if(formation_type == 2)
          formation = "wedge";
    else if(formation_type == 3)
          formation = "diamond";
    
    
    // each motorschema's weight
    w_goal            = 1;
    w_keep_formation  = 5;
    w_avoid_robot     = 1;
    w_avoid_obstacles = 5;
    w_noise           = 3;

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

    // setting up the fitness computation TODO: put it in PSO somewhere?
    reset_fitness_computation(FORMATION_SIZE,migrx,migrz,obstacle_loc);

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// DUMMY PARAMETERS FOR TESTING                                                                   //
//                                                                                                //
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
    /*pso_ocba(optimal_params);

    printf("The optimal parameters are: \n");
    for(d = 0; d < DIMENSIONALITY; d++){
        printf("    Dimension %d: %1.4f\n", d, optimal_params[d]);
    }
    printf("\n\n");*/

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

    
    // reset and communication part
    reset();
    printf("Supervisor resetted.\n");
    send_init_poses(); //this function is here as an example for communication using the supervisor
    printf("Init poses sent.\n Chosen formation: %s.\n", formation);
    
    // sending weights
    send_weights();
    printf("Weights sent\n");

    // infinite loop
    for(;;) {
        wb_robot_step(TIME_STEP);
        simulation_duration += TIME_STEP;

        // every 10 TIME_STEP (640ms)
        if (simulation_duration % 10 == 0) {
            send_current_poses();

            update_fitness();

            if (simulation_has_ended()) {
                printf("\n\n\n\n______________________________________JUDIHUI!______________________________________\n\n");
                break;
            }
        }
    }
    
    float fitness=compute_fitness(FORMATION_SIZE);
    printf("fitness = %f\n",fitness);
    return 0;
}
