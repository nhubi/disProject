#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "simulation.h"
#include "fitness.h"
#include "pso_ocba.h"





//Formation types
#define DEFAULT_FORMATION "line"     // Line formation as default

#define NB_PSO_WALL_RUNS           15   // Number of runs for PSO with a wall of obstacles
#define NB_PSO_WORLD2_RUNS         7    // Number of runs for PSO with a difficult configuration
#define NB_PSO_RANDOM_RUNS         20   // Number of runs for PSO with a random positionning
#define MAX_IT_PSO                 5000 // Number of iteration per PSO run



/*
 * Main function.
 */
int main(int argc, char *args[]) {
    float fitness;

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
    initialize();
    reset();
    printf("Beginning of PSO simulation\n");

    //PSO runs with a world with a wall of obstacles
    int sim; 
    for(sim = 0; sim < NB_PSO_WALL_RUNS; sim++) {
        printf("PSO simulation with a wall of obstacle n°%d\n", sim+1);

        reset_barrier_world();
        reset_fitness_computation(FORMATION_SIZE, migrx, migrz, obstacle_loc);
        printf("Supervisor reset.\n");

        send_init_poses();
        printf("Init poses sent.\n Chosen formation: %s.\n", formation);

        // sending weights
        send_weights();
        printf("Weights sent.");
        
        // pso loop
        int t;
        for(t = 0; t < MAX_IT_PSO; t++) {   //should run for about 4min of real time
            wb_robot_step(TIME_STEP);
    
            // every 10 TIME_STEP (640ms)
            if (simulation_duration % 10 == 0) {
                send_current_poses();

                update_fitness();
            }
            simulation_duration += TIME_STEP;
            
            if (simulation_has_ended()){
                break;
            }
        }
        fitness = compute_fitness(FORMATION_SIZE);
        printf("fitness = %f\n",fitness);
    }
    
    //PSO runs with a world with a difficult configuration
    for(sim = 0; sim < NB_PSO_WORLD2_RUNS; sim++) {
        printf("PSO simulation with a difficult configuration n°%d\n", sim+1);

        reset_world2();
        reset_fitness_computation(FORMATION_SIZE, migrx, migrz, obstacle_loc);
        printf("Supervisor reset.\n");

        send_init_poses();
        printf("Init poses sent.\n Chosen formation: %s.\n", formation);

        // sending weights
        send_weights();
        printf("Weights sent.");
    
        // pso loop
        int t;
        for(t = 0; t < MAX_IT_PSO; t++) {   //should run for about 4min of real time
            wb_robot_step(TIME_STEP);
    
            // every 10 TIME_STEP (640ms)
            if (simulation_duration % 10 == 0) {
                send_current_poses();

                update_fitness();
            }
            simulation_duration += TIME_STEP;
            
            if (simulation_has_ended()){
                break;
            }
        }
        fitness = compute_fitness(FORMATION_SIZE);
        printf("fitness = %f\n",fitness);
    }
    
    //PSO runs with a random world
    for(sim = 0; sim < NB_PSO_RANDOM_RUNS; sim++) {
        printf("PSO simulation with random positions n°%d\n", sim+1);
        reset_random_world();
        printf("Supervisor reset.\n");
        send_init_poses();
        printf("Init poses sent.\n Chosen formation: %s.\n", formation);
    

        // sending weights
        send_weights();

        // pso loop
        int t;
        for(t = 0; t < MAX_IT_PSO; t++) {   //should run for about 4min of real time
            wb_robot_step(TIME_STEP);
    
            // every 10 TIME_STEP (640ms)
            if (simulation_duration % 10 == 0) {
                send_current_poses();
    
                update_fitness();
            }
            simulation_duration += TIME_STEP;
            if (simulation_has_ended()){
                break;
            }
        }
        fitness = compute_fitness(FORMATION_SIZE);
        printf("fitness = %f\n",fitness);
    }
    
    // Real simulation with optimized parameters:
    initialize();
    // reset and communication part
    printf("Beginning of the real simulation\n");
    reset_to_initial_values();
    printf("Supervisor reset.\n");
    send_real_run_init_poses();
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
    
    fitness = compute_fitness(FORMATION_SIZE);
    printf("fitness = %f\n",fitness);
    return 0;
}
