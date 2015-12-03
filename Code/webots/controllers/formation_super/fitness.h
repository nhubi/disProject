////////////////////////////////////////////////////////////////////////////////////////////////////
// This file contains all the definitions and functions for the PSO fitness function              //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef FITNESS_H
#define FITNESS_H

#include <math.h>
#include <stdio.h>

#define AXLE_LENGTH         0.052   // Distance between wheels of robot (meters)

// At every time step we add the quantity, at the end we compute the mean
// We store the speed and the variation of angle in absolute value
float speed_sum[4][2]; 
int number_of_time_step;         // necessary to compute the mean

double compute_fitness(int FORMATION_SIZE);

void compute_speed(float loc[4][3],float prev_loc[4][3],float speed[4][3],int robot_id,float time_step);

void update_fitness_computation_for_robot(float loc[4][3],float prev_loc[4][3],float speed[4][3],int robot_id,float time_step);

void reset_fitness_computation(int FORMATION_SIZE);


#endif
