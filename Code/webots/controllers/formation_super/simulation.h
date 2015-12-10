////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// This file contains the declarations related to 'simulation.c'.                                 //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef SIMULATION_H
#define SIMULATION_H



// includes

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>



// definitions

#define FORMATION_SIZE       4	    // Number of robots in formation
#define TIME_STEP           64	    // [ms] Length of time step
#define ARENA_SIZE           0.94   // TODO: is this value correct? copied from pso_obs_sup.c, lab08
#define ROB_DIST             0.035  // Distance between robots in formation
#define NUMBER_OF_OBSTACLES  6      // Number of obstacles 



// declarations

// devices
WbNodeRef robs[FORMATION_SIZE];             // Robots nodes
WbFieldRef robs_trans[FORMATION_SIZE];      // Robots translation fields
WbFieldRef robs_rotation[FORMATION_SIZE];   // Robots rotation fields
WbDeviceTag emitter;                        // Single emitter

// obstacles
WbNodeRef  obstacles[NUMBER_OF_OBSTACLES];          // Obstacles nodes
WbFieldRef obstacles_trans[NUMBER_OF_OBSTACLES];    // Obstacles translation fields
float obstacle_loc[NUMBER_OF_OBSTACLES][2];         // obstacle locations (X,Y coordinates)

// Variables for goal:
int offset;             // Offset of robots number
float migrx,migrz;      // Migration vector
float orient_migr;      // Migration orientation
WbNodeRef goal_id;      // Goal node
WbFieldRef goal_field;  // Goal translation field

float loc[FORMATION_SIZE][3];       // Each robot's locations: X,Y and Theta
float prev_loc[FORMATION_SIZE][3];  // Previous location of everybody
float speed[FORMATION_SIZE][3];     // Speed computed between the last 2 positions
int formation_type;                 // line, column, wedge or diamond
int simulation_duration;            // ms since simulation start

// motorschema weights
float w_goal;
float w_keep_formation;
float w_avoid_robot;
float w_avoid_obstacles;
float w_noise;

// Noise paramethers
int noise_gen_frequency; // defines, after how many steps a new random vector should be generated
int fading;              // 1, if nice transition is wished from one random vector to the next, otherwise 0

// Thresholds
float avoid_obst_min_threshold;
float avoid_obst_max_threshold;
float move_to_goal_min_threshold;
float move_to_goal_max_threshold;
float avoid_robot_min_threshold;
float avoid_robot_max_threshold;
float keep_formation_min_threshold; // < min threshold: dead zone, no correction of direction
float keep_formation_max_threshold; // > max threshold: ballistic zone, full correction of direction



// methods

void reset(void);
void send_init_poses(void);
void init_rand_01(void);
float rand_01(void);
void random_pos(int robot_id);
void send_current_poses(void);
void update_fitness(void);
void send_weights(void);
int simulation_has_ended(void);


#endif
