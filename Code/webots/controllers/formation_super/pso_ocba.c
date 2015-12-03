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
