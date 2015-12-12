////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// This file contains the declarations related to the PSO OCBA algorithm.                         //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef PSO_OCBA_H
#define PSO_OCBA_H



// includes

//#include "something.h"



// definitions

// PSO general parameters
#define DIMENSIONALITY        15     // nb parameters to optimize
#define POPULATION_SIZE       30     // nb particles
#define NB_NEIGHBOURS          5     // size of neighbourhoods
#define MAX_EVAL_DURATION    100     // max time (in seconds) of evaluation run
#define PERSONAL_WEIGHT        2     // PSO function parameter
#define NEIGHBOURHOOD_WEIGHT   2     // PSO function parameter
#define INERTIA                0.6   // PSO function parameter
#define V_MAX                  2     // max velocity of particles
#define NORMALIZED_BORDER     10     // particle dimensions are normalized to [0, NORMALIZED_BORDER]

//#define NB_PSO_WALL_RUNS       2     // Number of runs for PSO with a wall of obstacles
//#define NB_PSO_WORLD2_RUNS     2     // Number of runs for PSO with a difficult configuration
//#define NB_PSO_RANDOM_RUNS     5     // Number of runs for PSO with a random positionning
#define PSO_WALL                 1     // 1 if we run PSO on a world with a wall of obstacles; 0 if not
#define PSO_HARD                 1     // 1 if we run PSO on a world with a difficult obstacle avoidance configuration; 0 if not
#define PSO_RANDOM               1     // 1 if we run PSO on a world with a random configuration; 0 if not



// PSO OCBA specific parameters
#define NB_ITERATIONS    10
#define ITERATION_BUDGED 80     // ca. 2 * nb_parameter * POPULATION_SIZE
#define N_ZERO            2     // nb initial evaluations
#define DELTA             4     // nb samples for 'remaining-budged evaluations'




// declarations

float parameter_ranges[DIMENSIONALITY][2];          // each parameter's valid range (min and max)
float velocities[POPULATION_SIZE][DIMENSIONALITY];  // each particle's current velocity
float positions[POPULATION_SIZE][DIMENSIONALITY];   // each particle's current position
float neighbours[POPULATION_SIZE][2*NB_NEIGHBOURS]; // each particle's neighbours
float perf_mean[POPULATION_SIZE];                   // each particle's current mean performance 
float perf_var[POPULATION_SIZE];                    // each particle's current performance variance
int perf_samples[POPULATION_SIZE];                  // nb samples for each particle's perf mean/var
float p_best_pos[POPULATION_SIZE][DIMENSIONALITY];  // each particle's best position
float p_best_val[POPULATION_SIZE];                  // each particle's best performance
float p_best_var[POPULATION_SIZE];                  // variance of each particle's best performance
int p_best_samples[POPULATION_SIZE];                // nb samples for each particle's best perf mean/var
float n_best_pos[POPULATION_SIZE][DIMENSIONALITY];  // best position in each particle's neighbourhood
float n_best_val[POPULATION_SIZE];                  // best performance in each particle's neighbourhood
int additional_budget[2*POPULATION_SIZE];           // ocba calculated budget for each particle + best

float ratio[2*POPULATION_SIZE]; // ratios N_i/N_s for each position i (where N_s is assumed to be 1)


// methods

void pso_ocba(float parameters[DIMENSIONALITY]);


#endif
