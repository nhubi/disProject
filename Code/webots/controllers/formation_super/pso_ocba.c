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

        // evaluate new particle position n0 times        
        for(p = 0; p < POPULATION_SIZE; p++){
            // reset the number of samples per iteration to 0
            perf_samples[p] = 0 ;
            evaluate_position(positions[p], &perf_mean[p], &perf_var[p], &perf_samples[p], N_ZERO);
        }


        remaining_budget = ITERATION_BUDGED - N_ZERO * POPULATION_SIZE;

        // allocate and spend remaining budget
        int debug;
        // in the first iteration, p_best values do not exist yet. 
        if(i != 0){
            while(remaining_budget > 0){
                debug = remaining_budget;
                ocba(&remaining_budget);
                if(debug == remaining_budget)
                    remaining_budget = -1;

                printf("\n=========================================\n");
                printf("|| REMAINING BUDGET = %d\n", remaining_budget);
                printf("=========================================\n\n");
                
                // spend the allocated budget
                for(p = 0; p < POPULATION_SIZE; p++){
                    evaluate_position(positions[p], &perf_mean[p], &perf_var[p], &perf_samples[p], additional_budget[p]);
                    evaluate_position(p_best_pos[p], &p_best_val[p], &p_best_var[p], &p_best_samples[p], additional_budget[p + POPULATION_SIZE]);
                }
            }
        }


        // update values
        for(p = 0; p < POPULATION_SIZE; p++) {
            // update personal best
            if(p_best_val[p] < perf_mean[p]){
                p_best_val[p]     = perf_mean[p];
                p_best_var[p]     = perf_var[p];
                p_best_samples[p] = perf_samples[p];

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
        
        // move particles according to current, best personal and best neighbourhood position 
        move_particles();
    }


    // find globally best performance
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
}





/*
 * Initializes particle parameters.
 * TODO init each particle's neighbours
 */
void init_particles(void){
    int p;  // particle pointer
    int d;  // dimension pointer

    // init the random generator
    init_rand_01();
    
    for(p = 0; p < POPULATION_SIZE; p++){
        perf_samples[p] = 0;
        n_best_val[p]   = 0;        // TODO
        p_best_val[p]   = 0;

        for(d = 0; d < DIMENSIONALITY; d++){
            positions[p][d]  = rand_01() * NORMALIZED_BORDER;
            velocities[p][d] = 0;
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

//printf("[PSO] Particle%d: \n", particle_id);
//printf("[PSO]     Mean performance and variance: %1.2f; %1.2f\n\n", *mean, *var);
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
        performance += (rand_01() - 0.5);  // add some random noise
    }
    //printf("[PSO]     Evaluating (%1.4f,%1.4f) --> %1.4f\n", params[0], params[1], performance);

    return performance;
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


    // compute ratio[i] = N_i/N_s
    ratio[s] = 1;           // we compare all positions to second best, hence ratio[s] = 1.
    float delta_bs = best_mean - scnd_best_mean;
    float delta_bi;
    float sum_ratio = 0;    // the sum of all ratios

    scnd_best_var += 0.0000000000001;   // we will devide through this value later
                                        // --> it should not be 0.

    for(p = 0; p < POPULATION_SIZE; p++){

        // fill in ratios for perf_mean values
        // add small value to delta_bi s.t. != 0 (because we will devide through delta_bi below)
        if(p != b && p != s){
            delta_bi = best_mean - perf_mean[p] + 0.0000000000001;

            // avoid division by 0
            ratio[p] = (perf_var[p] / scnd_best_var) * pow(delta_bs / delta_bi, 2);
        }

        // fill in ratios for p_pest_val values (in the first iterations these values don't exist)
        if(p + POPULATION_SIZE != b && p + POPULATION_SIZE != s){
            delta_bi = best_mean - p_best_val[p] + 0.0000000000001;

            // avoid division by 0
            ratio[p + POPULATION_SIZE] = (p_best_var[p] / scnd_best_var) * pow(delta_bs / delta_bi, 2);
        }
    }

    // compute N_b assuming, N_s = 1
    float sum_Ni2_over_var = 0;
    for(p = 0; p < POPULATION_SIZE; p++){
        if(p != b){
            sum_Ni2_over_var += pow(ratio[p], 2)/perf_var[p];
        }
        if(p + POPULATION_SIZE != b){
            sum_Ni2_over_var += pow(ratio[p + POPULATION_SIZE], 2)/p_best_var[p];
        }
    }
    

    // compute N_b according to equation 8 in Di Mario paper (assuming that N_s = 1)
    ratio[b] = best_var * sqrt(sum_Ni2_over_var);


    // create an array with all ratio indexes...
    int ratio_indexes[2*POPULATION_SIZE];
    for(p = 0; p < 2*POPULATION_SIZE; p++){
        ratio_indexes[p] = p;
    }

    // sort the ratio_indexes such that ratio_indexes[0] is the index of the biggest ratio[]
    float element_size = sizeof(ratio_indexes)/sizeof(*ratio_indexes);
    qsort(ratio_indexes, element_size, sizeof(*ratio_indexes), ratio_comparison);


    // init additional_budget
    for(p = 0; p < 2*POPULATION_SIZE; p++){
        additional_budget[p] = 0;
    }

    // distribute the additional budget among the candidates with the DELTA highest ratios.
    for(p = 0; p < DELTA; p++){
         sum_ratio += ratio[ratio_indexes[p]];
    }
    for(p = 0; p < DELTA; p++){
        int idx = ratio_indexes[p];
        additional_budget[idx] = round(DELTA * ratio[idx] / sum_ratio);

        *remaining_budget -= additional_budget[idx];

        // debugging
        printf("[OCBA] Candidate %d: \n", idx);
        if(idx < POPULATION_SIZE){
            printf("___________ position = (%f, %f)\n", positions[idx][0], positions[idx][1]);
            printf("___________ (mean, var, samples) = (%1.3f, %1.3f, %d)\n", perf_mean[idx], perf_var[idx], perf_samples[idx]);
        } else {
            printf("___________ position = (%f, %f)\n", p_best_pos[idx-POPULATION_SIZE][0], p_best_pos[idx-POPULATION_SIZE][1]);
            printf("___________ (mean, var, samples) = (%1.3f, %1.3f, %d)\n", p_best_val[idx-POPULATION_SIZE], p_best_var[idx-POPULATION_SIZE], p_best_samples[idx-POPULATION_SIZE]);
        }        
        printf("___________ additional budget = %d * %1.2f / %1.2f = %1.2f = %d \n", DELTA, ratio[idx], sum_ratio, DELTA * ratio[idx] / sum_ratio, additional_budget[idx]);
        if(idx == b)
            printf("> > > > > > BEST CANDIDATE < < < < < <\n");
        if(idx == s)
            printf("> > > > > > 2nd BEST CANDIDATE < < < < < <\n");
        printf("\n\n");
    }
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






/*
 * Computes each particle's velocity and new position.
 */
void move_particles(void){
    int p;  // particle pointer
    int d;  // dimension pointer
    int v_norm; //norm of velocity vector

    for(p = 0; p < POPULATION_SIZE; p++){
        v_norm = 0;

        for(d = 0; d < DIMENSIONALITY; d++){
            velocities[p][d] = INERTIA * velocities[p][d];
            velocities[p][d] += rand_01()*PERSONAL_WEIGHT * (p_best_pos[p][d]-positions[p][d]);
            velocities[p][d] += rand_01()*NEIGHBOURHOOD_WEIGHT * (n_best_pos[p][d]-positions[p][d]);
        
            v_norm += velocities[p][d] * velocities[p][d];
        }

        v_norm = sqrt(v_norm);

        // shorten velocity vector if necessary
        if(v_norm > V_MAX){
            for(d = 0; d < DIMENSIONALITY; d++){
                velocities[p][d] /= v_norm * V_MAX;
            }
        }
        
        for(d = 0; d < DIMENSIONALITY; d++){
            // particle's new position in dimension d
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

