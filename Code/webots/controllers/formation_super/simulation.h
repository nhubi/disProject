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

#define FORMATION_SIZE  4	    // Number of robots in formation
#define NB_OBSTACLES    6      // Number of present obstacles
#define TIME_STEP      64	    // [ms] Length of time step
#define ARENA_SIZE      4.0    // size of the arena where can the robots be randomly positioned
#define ROB_DIST        0.035



// declarations

// devices
WbNodeRef robs[FORMATION_SIZE];             // Robots nodes
WbFieldRef robs_trans[FORMATION_SIZE];      // Robots translation fields
WbFieldRef robs_rotation[FORMATION_SIZE];   // Robots rotation fields
WbFieldRef initial_robs_trans[FORMATION_SIZE];
WbFieldRef initial_robs_rotation[FORMATION_SIZE];

WbNodeRef obss[NB_OBSTACLES];               // Obstacles nodes
WbFieldRef initial_obs_trans[NB_OBSTACLES];
WbFieldRef initial_obs_rotation[NB_OBSTACLES];

WbDeviceTag emitter;                        // Single emitter

// Variables for goal:
int offset;             // Offset of robots number
float migrx,migrz;      // Migration vector
float orient_migr;      // Migration orientation
WbNodeRef goal_id;      // Goal node
WbFieldRef goal_field;  // Goal translation field
WbFieldRef initial_goal_field;

float loc[FORMATION_SIZE][3];   // Each robot's locations: X,Y and Theta
int formation_type;             // line, column, wedge or diamond
int simulation_duration;        // ms since simulation start



// methods
void initialize(void);
void reset(void);
void reset_barrier_world(void);
void reset_world2(void);
void reset_to_initial_values(void);
void send_init_poses(void);
void send_real_run_init_poses(void);
void send_current_poses(void);
float rand_01(void);
void random_pos(int robot_id, float x_min, float z_min);

#endif
