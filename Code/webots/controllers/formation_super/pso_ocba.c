////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// Implementation of PSO OCBA (Particle Swarm Optimisation with Optimal Computing Budget          //
// Allocation) according to the paper "A Distributed Noise-Resistant Particle Swarm Optimization  //
// Algorithm for High-Dimensional Multi-Robot Learning" (Di Mario, Navarro, Martinoli; IEEE 2015) //
//                                                                                                //
//      Initialize particles                                                                      //
//          for NB_ITERATIONS iterations do                                                       //
//              for N_p particles do                                                              //
//                  evaluate new particle position n0 times                                       //
//                                                                                                //
//              remaining_budget = iteration_budget - n0 * N_p                                    //
//              while remaining_budget > 0                                                        //
//                  Allocate DELTA samples among current positions and personal bests using OCBA. //
//                  Evaluate allocated samples.                                                   //
//                  Recalculate mean and variance for new evaluations.                            //
//                  remaining_budget = remaining_budget - DELTA                                   //
//                                                                                                //
//              for N_p particles do                                                              //
//                  Update personal best                                                          //
//                  Update neighbourhood best                                                     //
//                  Update particle position                                                      //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pso_ocba.h"
#include "simulation.h"
#include "fitness.h"


// private methods
void init_particles(void);
void move_particles(void);
void evaluate_position(float position[DIMENSIONALITY], float* mean, float* var, int* nb_samples, int budget);
float evaluate_parameters(float* params);
void convert_pos_to_params(float position[DIMENSIONALITY], float parameters[DIMENSIONALITY]);
void ocba(int * remaining_budget);
int ratio_comparison (const void * elem1, const void * elem2);





/*
 * Executes the PSO OCBA algorithm.
 */
void pso_ocba(float parameters[DIMENSIONALITY]){
    int i;  // iteration
    int p;  // particle pointer
    int d;  // dimension pointer
    int n;  // neighbour pointer
    int remaining_budget;

    init_particles();

    for(i = 0; i < NB_ITERATIONS; i++){
        sprintf(label, "Iteration: %d",i);
        wb_supervisor_set_label(0,label,0.01,0.01,0.1,0xffffff,0);
        // evaluate new particle position n0 times        
        for(p = 0; p < POPULATION_SIZE; p++){
            // reset the number of samples per iteration to 0
            perf_samples[p] = 0 ;
            evaluate_position(positions[p], &perf_mean[p], &perf_var[p], &perf_samples[p], N_ZERO);
        }

        remaining_budget = ITERATION_BUDGED - N_ZERO * POPULATION_SIZE;

        // allocate and spend remaining budget
        // in the first iteration, p_best values do not exist yet. Therefore, don't execute ocba.
        if(i != 0){
            while(remaining_budget > 0){
                ocba(&remaining_budget);
                
                for(p = 0; p < POPULATION_SIZE; p++){
                    evaluate_position(positions[p], &perf_mean[p], &perf_var[p], &perf_samples[p], additional_budget[p]);
                    evaluate_position(p_best_pos[p], &p_best_val[p], &p_best_var[p], &p_best_samples[p], additional_budget[p + POPULATION_SIZE]);
                }
            }
        }

        
        printf("Best values after iteration %d:\n", i);

        // update personal best
        for(p = 0; p < POPULATION_SIZE; p++) {
            if(p_best_val[p] < perf_mean[p]){
                p_best_val[p]     = perf_mean[p];
                p_best_var[p]     = perf_var[p];
                p_best_samples[p] = perf_samples[p];

                for(d = 0; d < DIMENSIONALITY; d++){
                    p_best_pos[p][d] = positions[p][d];
                }
            }
            printf(" - Particle %d: (mean, var, #samps) = (%1.4f, %1.4f, %d)\n", 
                p, p_best_val[p], p_best_var[p], p_best_samples[p]);
            
            // save result
            if (i==0) {
                result[i][p][0]=perf_mean[p];
                result[i][p][1]=perf_mean[p];                
            } else {
                result[i][p][0]=perf_mean[p];
                if (result[i][p][0]>result[i-1][p][1]) {            
                    result[i][p][1]=perf_mean[p];   
                } else {
                    result[i][p][1]=result[i-1][p][1];   
                }
            }
        }
        printf("============================================================\n\n\n");

        // update neighbourhood best (needs to be done AFTER all p_bests are found)
        for(p = 0; p < POPULATION_SIZE; p++) {
            int neighbour_id;
            for(n = 0; n < 2*NB_NEIGHBOURS; n++){
                neighbour_id = neighbours[p][n];

                // compare with the neighbours' best position?
                if(n_best_val[p] < p_best_val[neighbour_id]){
                    n_best_val[p] = p_best_val[neighbour_id];
                    for(d = 0; d < DIMENSIONALITY; d++){
                        n_best_pos[p][d] = p_best_pos[neighbour_id][d];
                    }
                }
            }
        }
        
        // move particles according to current, best personal and best neighbourhood position 
        move_particles();
    }


    // find globally best performance, print them
    float best_val = 0;
    int best_idx = 0;
    printf("[PSO] Best positions: \n");
    for(p = 0; p < POPULATION_SIZE; p++){
        if(p_best_val[p] > best_val){
            best_val = p_best_val[p];
            best_idx = p;
        }
        printf("[PSO] - Candidate%d: (%f, %f) --> mean: %f, var: %f, #samples: %d\n", p, p_best_pos[p][0], p_best_pos[p][1], p_best_val[p], p_best_var[p], p_best_samples[p]);
    }
    convert_pos_to_params(p_best_pos[best_idx], parameters);
    
    
    printf("\n\nSIMULATION ENDED\n\n");
    int j;
    for (i=0;i<NB_ITERATIONS;i++) {
        printf("Iteration %d\n",i);
        
        for (j=0;j<POPULATION_SIZE;j++) {
            printf("%f %f\n",result[i][j][0],result[i][j][1]);
        }
    
    }
}





/*
 * Initializes particle parameters.
 */
void init_particles(void){
    int p;  // particle pointer
    int d;  // dimension pointer
    int n;  // neighbour pointer
    
    for(p = 0; p < POPULATION_SIZE; p++){
        perf_samples[p] = 0;
        n_best_val[p]   = 0;
        p_best_val[p]   = 0;

        // init particle's position and velocity
        for(d = 0; d < DIMENSIONALITY; d++){
            positions[p][d]  = rand_01() * NORMALIZED_BORDER;
            velocities[p][d] = 0;
        }

        // init particle's neighbours 
        //  --> particles with index: p-1-NB_NEIGHBOURS < i < p+1+NB_NEIGHBOURS
        // Taking the modulo of negative values does not work. Therefore (POP_SIZE+p-1-n)%POP_SIZE.
        for(n = 0; n < NB_NEIGHBOURS; n++){
            neighbours[p][n] = (POPULATION_SIZE + p - 1 - n) % POPULATION_SIZE;
            neighbours[p][n + NB_NEIGHBOURS] = (p + 1 + n) % POPULATION_SIZE;
        }
    }
}





/*
 * Evaluates a particle's position. Out of a certain amount of samples, the mean and the variance
 * are calculated and saved in mean and var. 
 * Attributes: 
 *  - position:   the position to be evaluated
 *  - mean:       the position's current mean performance
 *  - var:        the position's current variance 
 *  - sb_samples: the number of samples to calculate the position's current mean performance
 *  - budget:     the number of times, this position can be evaluated
 */
void evaluate_position(float position[DIMENSIONALITY], float* mean, float* var, int* nb_samples, int budget){
    int s;  // sample pointer
    float parameters[DIMENSIONALITY];

    for(s = 0; s < budget; s++){

        // increment the sample counter (important to do this at the beginning of the for loop)
        *nb_samples = *nb_samples + 1;

        convert_pos_to_params(position, parameters);

        float sample = evaluate_parameters(parameters);
        
        // if it's the first sample, don't do the online-calculation.
        if(*nb_samples == 1){
            *mean = sample;
            *var  = 0;

        } else {
            // online calculation of mean and variance
            float previous_mean = *mean;

            // for readability: n is the number of samples made for calculating the current mean.
            int n = *nb_samples;

            // update mean and variance
            *mean = ((n-1) * previous_mean + sample) / n;
            *var  = (n-2)/(n-1) * (*var) + pow(sample - previous_mean, 2)/n;
        }
    }
}





/*
 * Runs simulation and computes performance. The performance value should not be negative.
 */
float evaluate_parameters(float* params){
    float performance = 0;  // return value
    float single_perf = 0;  // performance of one single pso run
    bool end_run = false;   // true if time is not out and a single simulation ends
    
    // each motorschema's weight
    w_goal            = params[0];
    w_keep_formation  = params[1];
    w_avoid_robot     = params[2];
    w_avoid_obstacles = params[3];
    w_noise           = 1;/*
    w_noise           = params[4];*/


    // thresholds
    avoid_obst_min_threshold     = 0.15;
    avoid_obst_max_threshold     = 0.3;
    move_to_goal_min_threshold   = 0.1;
    move_to_goal_max_threshold   = 0.5;
    avoid_robot_min_threshold    = 0.05;
    avoid_robot_max_threshold    = 0.1;
    keep_formation_min_threshold = 0.1;
    keep_formation_max_threshold = 0.2;

    // noise parameters
    noise_gen_frequency = 10;
    fading              = 1; // = 0 or 1
    
    /*
    // thresholds
    avoid_obst_min_threshold     = params[5];
    avoid_obst_max_threshold     = params[5] + params[6];
    move_to_goal_min_threshold   = params[7];
    move_to_goal_max_threshold   = params[7] + params[8];
    avoid_robot_min_threshold    = params[9];
    avoid_robot_max_threshold    = params[9] + params[10];
    keep_formation_min_threshold = params[11];
    keep_formation_max_threshold = params[11] + params[12];

    // noise parameters
    noise_gen_frequency = params[13];
    fading              = round(params[14]); // = 0 or 1
    */
    
    // show the tested parameters.
    printf("\n\n\nTesting the following configuration: \n");
    printf(" - w_goal...................... = %f\n", w_goal);
    printf(" - w_keep_formation............ = %f\n", w_keep_formation);
    printf(" - w_avoid_robo................ = %f\n", w_avoid_robot);
    printf(" - w_avoid_obstacles........... = %f\n", w_avoid_obstacles);
    printf(" - w_noise..................... = %f\n", w_noise);
    printf(" - noise_gen_frequency......... = %d\n", noise_gen_frequency);
    printf(" - fading...................... = %d\n", fading);
    printf(" - avoid_obst_min_threshold.... = %f\n", avoid_obst_min_threshold);
    printf(" - avoid_obst_max_threshold.... = %f\n", avoid_obst_max_threshold);
    printf(" - move_to_goal_min_threshold.. = %f\n", move_to_goal_min_threshold);
    printf(" - move_to_goal_max_threshold.. = %f\n", move_to_goal_max_threshold);
    printf(" - avoid_robot_min_threshold... = %f\n", avoid_robot_min_threshold);
    printf(" - avoid_robot_max_threshold... = %f\n", avoid_robot_max_threshold);
    printf(" - keep_formation_min_threshold = %f\n", keep_formation_min_threshold);
    printf(" - keep_formation_max_threshold = %f\n", keep_formation_max_threshold);
    printf("\n\n");

    //PSO runs with a world with a wall of obstacles
    if(PSO_WALL)
    {
        printf("[PSO] Simulation with a wall of obstacle\n");
        reset_barrier_world();
        reset_fitness_computation(FORMATION_SIZE, migrx, migrz, obstacle_loc);
        printf("[PSO] Supervisor reset.\n");

        send_init_poses();
        printf("[PSO] Init poses sent.\n");

        // sending weights
        send_weights();
        printf("[PSO] Weights sent.\n");
        
        // pso loop (nb iterations is limited)
        int t;
        for(t = 0; t*64/1000 < MAX_EVAL_DURATION; t++) {
            wb_robot_step(TIME_STEP);
    
            // every 10 TIME_STEP (640ms)
            if (simulation_duration % 10 == 0) {
                send_current_poses();

                update_fitness();
            }

            simulation_duration += TIME_STEP;
            end_run = simulation_has_ended();
            if (end_run) {
                printf("[PSO] Goal reached in formation\n");
                break;
            }

        }
        
        if (end_run) {
            single_perf = compute_fitness(FORMATION_SIZE,loc);
        } else {
            single_perf = compute_fitness(FORMATION_SIZE,loc);
        }

        printf("[FITNESS] Single fitness = %f\n\n", single_perf); 
        performance += single_perf;
    }
    
    //PSO runs with a world with a difficult configuration
    if(PSO_HARD) {
        printf("[PSO] Simulation with a difficult configuration\n"); 
        reset_world2();
        reset_fitness_computation(FORMATION_SIZE, migrx, migrz, obstacle_loc);
        printf("[PSO] Supervisor reset.\n");

        send_init_poses();
        printf("[PSO] Init poses sent.\n");

        // sending weights
        send_weights();
        printf("[PSO] Weights sent.\n");
    
        // pso loop (nb iterations is limited)
        int t;
        for(t = 0; t*64/1000 < MAX_EVAL_DURATION; t++) {
            wb_robot_step(TIME_STEP);
    
            // every 10 TIME_STEP (640ms)
            if (simulation_duration % 10 == 0) {
                send_current_poses();

                update_fitness();
            }
            simulation_duration += TIME_STEP;
            
            end_run = simulation_has_ended();
            if (end_run) {
                printf("[PSO] Goal reached in formation\n");
                break;
            }
        }
        
        if (end_run) {
            single_perf = compute_fitness(FORMATION_SIZE,loc);
        } else {
            single_perf = compute_fitness(FORMATION_SIZE,loc);
        }

        printf("[FITNESS] Single fitness: %f\n\n", single_perf); 
        performance += single_perf;
    }
    
    //PSO runs with a random world
    if(PSO_RANDOM) {
        printf("[PSO] Simulation with random positions\n");
        reset_random_world();
        reset_fitness_computation(FORMATION_SIZE, migrx, migrz, obstacle_loc);
        printf("[PSO] Supervisor reset.\n");
        
        send_init_poses();
        printf("[PSO] Init poses sent.\n");
    
        // sending weights
        send_weights();
        printf("[PSO] Weights sent.\n");
    
        // pso loop (nb iterations is limited)
        int t;
        for(t = 0; t*64/1000 < MAX_EVAL_DURATION; t++) {
            wb_robot_step(TIME_STEP);
    
            // every 10 TIME_STEP (640ms)
            if (simulation_duration % 10 == 0) {
                send_current_poses();
    
                update_fitness();
            }
            simulation_duration += TIME_STEP;
            end_run = simulation_has_ended();
            if (end_run) {
                printf("[PSO] Goal reached in formation\n");
                break;
            }
        }
        
        if (end_run) {
            single_perf = compute_fitness(FORMATION_SIZE,loc);
        } else {
            single_perf = compute_fitness(FORMATION_SIZE,loc);
        }

        printf("[FITNESS] Single fitness: %f\n\n", single_perf); 
        performance += single_perf;
    }
    
    if(PSO_WALL || PSO_HARD || PSO_RANDOM)
        performance /= (PSO_WALL + PSO_HARD + PSO_RANDOM);
    
    printf("[FITNESS] Total fitness = %f\n\n\n",performance);
    return performance;
}






/*
 * Computes each particle's velocity and new position.
 */
void move_particles(void){
    int p;      // particle pointer
    int d;      // dimension pointer
    int v_norm; // norm of velocity vector

    for(p = 0; p < POPULATION_SIZE; p++){
        v_norm = 0;

        for(d = 0; d < DIMENSIONALITY; d++){
            velocities[p][d] = INERTIA * velocities[p][d];
            velocities[p][d] += rand_01()*PERSONAL_WEIGHT * (p_best_pos[p][d]-positions[p][d]);
            velocities[p][d] += rand_01()*NEIGHBOURHOOD_WEIGHT * (n_best_pos[p][d]-positions[p][d]);
        
            v_norm += velocities[p][d] * velocities[p][d];
        }

        v_norm = sqrt(v_norm);

        // make sure that particles don't move faster than V_MAX.
        if(v_norm > V_MAX){
            for(d = 0; d < DIMENSIONALITY; d++){
                velocities[p][d] /= v_norm * V_MAX;
            }
        }
        
        // compute new position
        for(d = 0; d < DIMENSIONALITY; d++){
            positions[p][d] = positions[p][d] + velocities[p][d];
            if(positions[p][d] > NORMALIZED_BORDER){
                positions[p][d] = NORMALIZED_BORDER;

            } else if(positions[p][d] < 0){
                positions[p][d] = 0;
            }
        }
    }
}





/*
 * Converts a particle position into controller parameters
 */
void convert_pos_to_params(float position[DIMENSIONALITY], float parameters[DIMENSIONALITY]){
    int d;      // dimension pointer
    float factor;

    for(d = 0; d < DIMENSIONALITY; d++){
        factor = (parameter_ranges[d][1] - parameter_ranges[d][0]) / (float)NORMALIZED_BORDER;
        parameters[d] = position[d] * factor + parameter_ranges[d][0];
    }
}





/*
 * Computes each particle's additionnal budget.
 * Equations 7 and 8 in the Di Mario paper allow us to calculate: 
 *  - N_b: the number of samples for the best position
 *  - N_i/N_j: the ratio between samples for position i and samples for position j
 * --> ocba() calculates N_i/N_s, for each i, where N_s is the second best position.
 * 
 * Positions considered for re-evaluation are all new positions (performance value saved in 
 * perf_mean[]) and all personal best positions (performance values saved in p_best_val[]).
 * 
 *  - int * remaining_budget: the remaining budget is decreased according to the totally 
 *                            assigned budget (which should be around DELTA)
 *  - bool firt_iter: since the best values do not yet exist in the first iteration, this 
 */
void ocba(int * remaining_budget){

    float best_mean = 0;        // mean value of the best position
    float best_var  = 0;        // variance of the best mean value
    int b = -1;                 // index of the best position (init to 'illegal' -1)

    float scnd_best_mean = 0;   // mean value of the second best position
    float scnd_best_var  = 0;   // variance of second best mean value
    int s = -1;                 // index of the second best position (init to 'illegal' -1)

    int p;  // position pointer

    // find best and second best position (first go through current pos, then through p_best pos)
    for(p = 0; p < POPULATION_SIZE; p++){
        if(perf_mean[p] > scnd_best_mean){
            if(perf_mean[p] > best_mean){
                // best becomes second best, perf_mean[p] becomes best
                s = b;
                scnd_best_mean = best_mean;
                scnd_best_var  = best_var;
                b = p;
                best_mean = perf_mean[p];
                best_var  = perf_var[p];

            } else {
                s = p;
                scnd_best_mean = perf_mean[p];
                scnd_best_var  = perf_var[p];
            }
        }
    }
    for(p = 0; p < POPULATION_SIZE; p++){
        if(p_best_val[p] > scnd_best_mean){
            if(p_best_val[p] > best_mean){
                // best becomes second best, p_best_val[p] becomes best
                s = b;
                scnd_best_mean = best_mean;
                scnd_best_var  = best_var;

                b = p + POPULATION_SIZE;
                best_mean = p_best_val[p];
                best_var  = p_best_var[p];

            } else {
                s = p + POPULATION_SIZE;
                scnd_best_mean = p_best_val[p];
                scnd_best_var  = p_best_var[p];
            }
        }
    }
    // we will devide through this value later --> it should not be 0.
    scnd_best_var += 0.0000000000001;


    // compute ratio[i] = N_i/N_s
    ratio[s] = 1;           // we compare all positions to second best, hence ratio[s] = 1.
    float delta_bs = best_mean - scnd_best_mean;
    float delta_bi;
    for(p = 0; p < POPULATION_SIZE; p++){

        // fill in ratios for perf_mean values
        // add small value to delta_bi s.t. != 0 (because we will devide through delta_bi below)
        if(p != b && p != s){
            delta_bi = best_mean - perf_mean[p] + 0.0000000000001;
            ratio[p] = (perf_var[p] / scnd_best_var) * pow(delta_bs / delta_bi, 2);
        }

        // fill in ratios for p_pest_val values
        if(p + POPULATION_SIZE != b && p + POPULATION_SIZE != s){
            delta_bi = best_mean - p_best_val[p] + 0.0000000000001;
            ratio[p + POPULATION_SIZE] = (p_best_var[p] / scnd_best_var) * pow(delta_bs / delta_bi, 2);
        }
    }


    // compute N_b according to equation 8 in Di Mario paper (assuming that N_s = 1)
    float sum_Ni2_over_var = 0;
    for(p = 0; p < POPULATION_SIZE; p++){
        if(p != b){
            sum_Ni2_over_var += pow(ratio[p], 2)/perf_var[p];
        }
        if(p + POPULATION_SIZE != b){
            sum_Ni2_over_var += pow(ratio[p + POPULATION_SIZE], 2)/p_best_var[p];
        }
    }
    ratio[b] = best_var * sqrt(sum_Ni2_over_var);


    // create an array with all ratio indexes...
    int ratio_indexes[2*POPULATION_SIZE];
    for(p = 0; p < 2*POPULATION_SIZE; p++){
        ratio_indexes[p] = p;
    }

    // sort the ratio_indexes such that ratio_indexes[0] is the index of the biggest ratio[]
    float element_size = sizeof(ratio_indexes)/sizeof(*ratio_indexes);
    qsort(ratio_indexes, element_size, sizeof(*ratio_indexes), ratio_comparison);


    // initialise additional_budget array
    for(p = 0; p < 2*POPULATION_SIZE; p++){
        additional_budget[p] = 0;
    }

    // distribute the additional budget among the DELTA candidates with the highest ratios.
    float sum_ratio = 0;    // the sum of all ratios
    int idx;
    for(p = 0; p < DELTA; p++){
         sum_ratio += ratio[ratio_indexes[p]];
    }
    for(p = 0; p < DELTA; p++){
        idx = ratio_indexes[p];
        additional_budget[idx] = round(DELTA * ratio[idx] / sum_ratio);
        *remaining_budget -= additional_budget[idx];
    }


    // debug messages
/*
    printf("\n==========================================================\n");
    printf("|| REMAINING BUDGET = %d\n", *remaining_budget);
    printf("==========================================================\n\n");

    for(p = 0; p < DELTA; p++){
        idx = ratio_indexes[p];
        printf("[OCBA] Candidate %d: \n", idx);
        if(idx < POPULATION_SIZE){
            printf("___________ position = (%f, %f)\n", positions[idx][0], positions[idx][1]);
            printf("___________ (mean, var, #samples) = (%1.3f, %1.3f, %d)\n", perf_mean[idx], perf_var[idx], perf_samples[idx]);
        } else {
            printf("___________ position = (%f, %f)\n", p_best_pos[idx-POPULATION_SIZE][0], p_best_pos[idx-POPULATION_SIZE][1]);
            printf("___________ (mean, var, #samples) = (%1.3f, %1.3f, %d)\n", p_best_val[idx-POPULATION_SIZE], p_best_var[idx-POPULATION_SIZE], p_best_samples[idx-POPULATION_SIZE]);
        }        
        printf("___________ additional budget = %d * %1.2f / %1.2f = %1.2f = %d \n", DELTA, ratio[idx], sum_ratio, DELTA * ratio[idx] / sum_ratio, additional_budget[idx]);
        if(idx == b)
            printf("> > > > > > BEST CANDIDATE < < < < < <\n");
        if(idx == s)
            printf("> > > > > > 2nd BEST CANDIDATE < < < < < <\n");
        printf("\n\n");
    }
    printf("\n\n");
*/
}






/*
 * Compares two ratios (used to sort the ratio indexes)
 */
int ratio_comparison (const void * elem1, const void * elem2){
    int f = *((int*)elem1);
    int s = *((int*)elem2);
    if (ratio[f] < ratio[s]) return  1;
    if (ratio[f] > ratio[s]) return -1;
    return 0;
}
