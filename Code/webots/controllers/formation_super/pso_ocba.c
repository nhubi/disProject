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
#include <string.h>

#include "pso_ocba.h"
#include "simulation.h"


// private methods
void init_particles(void);
void move_particles(void);
void evaluate_position(int particle_id, int nb_samples);
float evaluate_parameters(float* params);
void convert_pos_to_params(float position[DIMENSIONALITY], float parameters[DIMENSIONALITY]);





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

        // evaluate new particle position n0 times        
        for(p = 0; p < POPULATION_SIZE; p++){
            // reset the number of samples per iteration to 0
            perf_samples[p] = 0 ;
            evaluate_position(p, N_ZERO);
        }


        remaining_budget = ITERATION_BUDGED - N_ZERO * POPULATION_SIZE;

        // allocate and spend remaining budget
        // TODO: OCBA stuff...


        // update values
        for(p = 0; p < POPULATION_SIZE; p++) {
            // update personal best
            if(p_best_val[p] < perf_mean[p]){
                p_best_val[p] = perf_mean[p];
                for(d = 0; d < DIMENSIONALITY; d++){
                    p_best_pos[p][d] = positions[p][d];
                }
            }

            // update neighbourhood best
            int neighbour_id;

            for(n = 0; n < NB_NEIGHBOURS; n++){
                neighbour_id = neighbours[p][n];

                // TODO: do we need to compare with the neighbours' current or best position?
                if(n_best_val[p] < p_best_val[neighbour_id]){
                    n_best_val[p] = p_best_val[neighbour_id];
                    for(d = 0; d < DIMENSIONALITY; d++){
                        n_best_pos[p][d] = p_best_pos[neighbour_id][d];
                    }
                }
            }         
        }

        move_particles();
    }


    // find globally best performance
    float best_val = 0;
    int best_idx = 0;

    for(p = 0; p < POPULATION_SIZE; p++){
        if(p_best_val[p] > best_val){
            best_val = p_best_val[p];
            best_idx = p;
        }
    }

    convert_pos_to_params(p_best_pos[best_idx], parameters);
}





/*
 * Initializes particle parameters.
 */
void init_particles(void){
    int p;  // particle pointer
    int d;  // dimension pointer

    // init the random generator
    init_rand_01();
    
    for(p = 0; p < POPULATION_SIZE; p++){
        perf_samples[p] = 0;
        p_best_val[p]   = 0;
        n_best_val[p]   = 0;

        for(d = 0; d < DIMENSIONALITY; d++){
            positions[p][d]  = rand_01() * NORMALIZED_BORDER;
            velocities[p][d] = 0;
        }
    }
}





/*
 * Evaluates a particle's position. Out of a certain amount of samples, the mean and the variance
 * are calculated and saved in perf_mean[] and perf_var[].
 */
void evaluate_position(int particle_id, int nb_samples){
    int s;  // sample pointer
    float parameters[DIMENSIONALITY];

printf("[PSO] Particle%d: \n", particle_id);

    for(s = 0; s < nb_samples; s++){

        // increment the sample counter (important to do this at the beginning of the for loop)
        perf_samples[particle_id]++;

        convert_pos_to_params(positions[particle_id], parameters);

        float sample = evaluate_parameters(parameters);
        
        // if it's the first sample, don't do the online-calculation.
        if(perf_samples[particle_id] == 1){
            perf_mean[particle_id] = sample;
            perf_var[particle_id]  = 0;

        } else {
            // online calculation of mean and variance
            float previous_mean = perf_mean[particle_id];

            // for readability: n is the number of samples made for calculating the current mean.
            int n = perf_samples[particle_id];

            // update mean and variance
            perf_mean[particle_id] = ((n-1) * previous_mean + sample) / n;
            perf_var[particle_id] = (n-2)/(n-1) * perf_var[particle_id] + pow(sample - previous_mean, 2)/n;
        }
    }
printf("[PSO]     Mean performance and variance: %1.2f; %1.2f\n\n", perf_mean[particle_id], perf_var[particle_id]);
}





/*
 * Runs simulation and computes performance.
 * TODO: currently, this is just a dummy function.
 *       Later, we will start the simulation here and execute the fitness function.
 */
float evaluate_parameters(float* params){
    int d;                  // dimension pointer
    float performance = 0;  // return value

    for(d = 0; d < DIMENSIONALITY; d++){
        performance += pow(params[d], 2);
    }
    printf("[PSO]     Evaluating (%1.4f,%1.4f) --> %1.4f\n", params[0], params[1], performance);

    return performance;
}






/*
 * Computes each particle's velocity and new position.
 */
void move_particles(void){
    int p;  // particle pointer
    int d;  // dimension pointer

    for(p = 0; p < POPULATION_SIZE; p++){
        for(d = 0; d < DIMENSIONALITY; d++){
            velocities[p][d] = INERTIA * velocities[p][d];
            velocities[p][d] += rand_01()*PERSONAL_WEIGHT * (p_best_pos[p][d]-positions[p][d]);
            velocities[p][d] += rand_01()*NEIGHBOURHOOD_WEIGHT * (n_best_pos[p][d]-positions[p][d]);
            
            // particle's new position in dimension d
            positions[p][d] = positions[p][d] + velocities[p][d];
            if(positions[p][d] > NORMALIZED_BORDER){
                positions[p][d] = NORMALIZED_BORDER;

            } else if(positions[p][d] < 0){
                positions[p][d] = 0;
            }
        }
        //printf("[PSO] Particle%d's position: (%1.4f,%1.4f)\n", p, positions[p][0], positions[p][1]);
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
