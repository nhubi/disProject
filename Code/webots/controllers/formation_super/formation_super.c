#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "simulation.h"
#include "fitness.h"
#include "pso_ocba.h"



//Formation types
#define DEFAULT_FORMATION "line"        // Line formation as default
#define NB_PSO_WALL_RUNS           2   // Number of runs for PSO with a wall of obstacles
#define NB_PSO_WORLD2_RUNS         2    // Number of runs for PSO with a difficult configuration
#define NB_PSO_RANDOM_RUNS         5   // Number of runs for PSO with a random positionning
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
    printf("Chosen formation: %s.\n", formation);
    
    
////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              PSO                                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

/*
    // each motorschema's weight
    parameter_ranges[0][0] =  0;
    parameter_ranges[0][1] = 10;
    parameter_ranges[1][0] =  0;
    parameter_ranges[1][1] = 10;
    parameter_ranges[2][0] =  0;
    parameter_ranges[2][1] = 10;
    parameter_ranges[3][0] =  0;
    parameter_ranges[3][1] = 10;
    parameter_ranges[4][0] =  0;
    parameter_ranges[4][1] = 10;

    // thresholds
    parameter_ranges[ 5][0] =  30;      // obstacle avoidance min
    parameter_ranges[ 5][1] = 200;
    parameter_ranges[ 6][0] =   0;      // obstacle avoidance min + range
    parameter_ranges[ 6][1] = 400;
    parameter_ranges[ 7][0] =   0.001;  // move_to_goal min
    parameter_ranges[ 7][1] =   0.5;
    parameter_ranges[ 8][0] =   0.001;  // move_to_goal min + range
    parameter_ranges[ 8][1] =   1;
    parameter_ranges[ 9][0] =   0.001;  // avoid_robot min
    parameter_ranges[ 9][1] =   0.5;
    parameter_ranges[10][0] =   0.001;  // avoid_robot min + range
    parameter_ranges[10][1] =   1;
    parameter_ranges[11][0] =   0.001;  // keep_formation min
    parameter_ranges[11][1] =   1;
    parameter_ranges[12][0] =   0.001;  // keep_formation min + range
    parameter_ranges[12][1] =   1;
    
    // noise
    parameter_ranges[13][0] =   1;      // noise_gen frequency
    parameter_ranges[13][1] =  20;
    parameter_ranges[14][0] =   0;      // fade or not
    parameter_ranges[14][1] =   1;



    float optimal_params[DIMENSIONALITY];
    initialize();
    reset();
    
    pso_ocba(optimal_params);

     // each motorschema's weight
     w_goal            = optimal_params[0];
     w_keep_formation  = optimal_params[1];
     w_avoid_robot     = optimal_params[2];
     w_avoid_obstacles = optimal_params[3];
     w_noise           = optimal_params[4];
 
     // thresholds
     avoid_obst_min_threshold     = optimal_params[5];
     avoid_obst_max_threshold     = optimal_params[5] + optimal_params[6];
     move_to_goal_min_threshold   = optimal_params[7];
     move_to_goal_max_threshold   = optimal_params[7] + optimal_params[8];
     avoid_robot_min_threshold    = optimal_params[9];
     avoid_robot_max_threshold    = optimal_params[9] + optimal_params[10];
     keep_formation_min_threshold = optimal_params[11];
     keep_formation_max_threshold = optimal_params[11] + optimal_params[12];
 
     // noise parameters
     noise_gen_frequency = optimal_params[13];
     fading              = round(optimal_params[14]); // = 0 or 1
 
     printf("\n\n\nThe optimal parameters are: \n");
     printf("___________ w_goal...................... = %f\n", w_goal);
     printf("___________ w_keep_formation............ = %f\n", w_keep_formation);
     printf("___________ w_avoid_robo................ = %f\n", w_avoid_robot);
     printf("___________ w_avoid_obstacles........... = %f\n", w_avoid_obstacles);
     printf("___________ w_noise..................... = %f\n", w_noise);
     printf("___________ noise_gen_frequency......... = %d\n", noise_gen_frequency);
     printf("___________ fading...................... = %d\n", fading);
     printf("___________ avoid_obst_min_threshold.... = %f\n", avoid_obst_min_threshold);
     printf("___________ avoid_obst_max_threshold.... = %f\n", avoid_obst_max_threshold);
     printf("___________ move_to_goal_min_threshold.. = %f\n", move_to_goal_min_threshold);
     printf("___________ move_to_goal_max_threshold.. = %f\n", move_to_goal_max_threshold);
     printf("___________ avoid_robot_min_threshold... = %f\n", avoid_robot_min_threshold);
     printf("___________ avoid_robot_max_threshold... = %f\n", avoid_robot_max_threshold);
     printf("___________ keep_formation_min_threshold = %f\n", keep_formation_min_threshold);
     printf("___________ keep_formation_max_threshold = %f\n", keep_formation_max_threshold);
     printf("\n\n");
    
    */
    
    // each motorschema's weight
    w_goal            = 1;
    w_keep_formation  = 5;
    w_avoid_robot     = 1;
    w_avoid_obstacles = 5;
    w_noise           = 2;//3;

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
    
    // reset and communication part
    initialize();
    reset();
    printf("Beginning of PSO simulation\n");

    //PSO runs with a world with a wall of obstacles
    int sim; 
    int end = 0;
    for(sim = 0; sim < NB_PSO_WALL_RUNS; sim++) {
        printf("PSO simulation with a wall of obstacle n°%d\n", sim+1);

        reset_barrier_world();
        reset_fitness_computation(FORMATION_SIZE, migrx, migrz, obstacle_loc);
        printf("Supervisor reset.\n");

        send_init_poses();
        printf("Init poses sent.\n Chosen formation: %s.\n", formation);

        // sending weights
        send_weights();
        printf("Weights sent.\n");
        
        // pso loop
        int t;
        end = 0;
        for(t = 0; t < MAX_IT_PSO; t++) {   //should run for about 4min of real time
            wb_robot_step(TIME_STEP);
    
            // every 10 TIME_STEP (640ms)
            if (simulation_duration % 10 == 0) {
                send_current_poses();

                update_fitness();
            }
            simulation_duration += TIME_STEP;
            
            if (simulation_has_ended()){
                end = 1;
                printf("Goal reached\n");
                break;
            }
        }
        fitness = compute_fitness(FORMATION_SIZE, end);
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
        printf("Weights sent.\n");
    
        // pso loop
        int t;
        end = 0;
        for(t = 0; t < MAX_IT_PSO; t++) {   //should run for about 4min of real time
            wb_robot_step(TIME_STEP);
    
            // every 10 TIME_STEP (640ms)
            if (simulation_duration % 10 == 0) {
                send_current_poses();

                update_fitness();
            }
            simulation_duration += TIME_STEP;
            
            if (simulation_has_ended()){
                end = 1;
                printf("Goal reached\n");
                break;
            }
        }
        fitness = compute_fitness(FORMATION_SIZE, end);
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
        printf("Weights sent.\n");
        
        // pso loop
        int t;
        end = 0;
        for(t = 0; t < MAX_IT_PSO; t++) {   //should run for about 4min of real time
            wb_robot_step(TIME_STEP);
    
            // every 10 TIME_STEP (640ms)
            if (simulation_duration % 10 == 0) {
                send_current_poses();
    
                update_fitness();
            }
            simulation_duration += TIME_STEP;
            if (simulation_has_ended()){
                end = 1;
                printf("Goal reached\n");
                break;
            }
        }
        fitness = compute_fitness(FORMATION_SIZE, end);
        printf("fitness = %f\n",fitness);
    }
    
////////////////////////////////////////////////////////////////////////////////////////////////////
//                                         REAL RUN                                               //
////////////////////////////////////////////////////////////////////////////////////////////////////

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
    
    bool is_goal_reached=false;
    // infinite loop
    for(;;) {
        wb_robot_step(TIME_STEP);

        // every 10 TIME_STEP (640ms)
        if (simulation_duration % 10 == 0) {
            send_current_poses();

            update_fitness();
        }
        simulation_duration += TIME_STEP;
        if (simulation_has_ended()) {
            is_goal_reached=true;
            char buffer[255]; // buffer for sending data
            int robot_id;
            for(robot_id = 0; robot_id < FORMATION_SIZE; robot_id++) {
                sprintf(buffer,"%1d#%1d#%f#%f#%f##%f#%f#%1d",robot_id,0,migrx,migrz,loc[robot_id][2],migrx,migrz,1);
                wb_emitter_send(emitter,buffer,strlen(buffer));
            }
            printf("\n\n\n\n______________________________________GOAL REACHED!______________________________________\n\n");
            break;
        }
    }
    
    if (is_goal_reached){
        fitness = compute_fitness(FORMATION_SIZE,1);
    } else {
        fitness = compute_fitness(FORMATION_SIZE,1);
    }
    printf("fitness = %f\n",fitness);
    
    while (1) wb_robot_step(TIME_STEP); //wait forever
    
    return 0;
}