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
#define POPULATION_SIZE      20     // nb particles
#define EVALUATION_SPAN      30     // max time (in seconds) of evaluation run
#define PERSONAL_WEIGHT       2     // PSO function parameter
#define NEIGHBOURHOOD_WEIGHT  2     // PSO function parameter
#define INERTIA               0.8   // PSO function parameter
#define V_MAX                20     // max velocity of particles

// PSO OCBA specific parameters
#define NB_ITERATIONS 10
#define ITERATION_BUDGED 150    // ca. 10 * nb_parameter
#define N_ZERO 2                // nb initial evaluations
#define DELTA 4                 // nb samples for 'remaining-budged evaluations'



// declarations

//float something;



// methods

void pso_ocba();


#endif
