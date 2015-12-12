////////////////////////////////////////////////////////////////////////////////////////////////////
// This file contains all the definitions and functions for the PSO fitness function              //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef FITNESS_H
#define FITNESS_H

#include <math.h>
#include <stdio.h>

#define AXLE_LENGTH         0.052   // Distance between wheels of robot (meters)

// Global variables
float fitness_obstacle_loc[6][2]; // Position of obstacle

// At every time step we add the quantity, at the end we compute the mean
float speed_sum[4][2]; // Speed and the variation of angle in absolute value
float keep_formation_distance[4]; // Sum of the distances from the place in formation where they should be
float obstacle_term_sum[4]; // Sum of the maximum values of the sensors
int number_of_time_step;         // necessary to compute the mean


double compute_fitness(int formation_size);

void compute_speed(float loc[4][3],float prev_loc[4][3],float speed[4][3],int robot_id,float time_step);

void update_fitness_computation_for_robot(float loc[4][3],float prev_loc[4][3],float speed[4][3],int robot_id,float time_step,int formation_type);

void update_speed_sum(float loc[4][3],float prev_loc[4][3],float speed[4][3],int robot_id,float time_step);

void update_obstacle_term(float loc[4][3],int robot_id);

void update_keep_formation_distance(float loc[4][3],int robot_id, int formation_type);

void reset_fitness_computation(int formation_size,float migrx,float migrz,float obstacle_loc[6][2]);


// FROM HERE METHODS AND DECLARATIONS FROM ms_keep_formation
// TODO: Keep them here?

//Formation types
#define LINE          0
#define COLUMN        1
#define WEDGE         2
#define DIAMOND       3

// declarations
float dir_goal[2];
float migr[2];
extern const float robot_dist;      // Coefficient that determines the distance between the robots,
                                    // to be changed depending on the robots' size. 
// methods
void get_relative_formation_coordinates(float* coordinates,int formation_type,int robot_id);
void get_absolute_formation_coordinates(float* coordinates, float* relative_coordinates, float* dir_goal,float* unit_center);
void get_move_to_goal_vector(float * direction,float* unit_center);
float norm(float* vector, int dim);


#endif
