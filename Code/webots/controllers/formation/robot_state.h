////////////////////////////////////////////////////////////////////////////////////////////////////
// This file contains any definition related to the robots' current states                        //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H



// includes

#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/distance_sensor.h>
#include <webots/differential_wheels.h>



// Definitions

// Robot specs
#define AXLE_LENGTH         0.052   // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS     0.00628 // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS        0.0205  // Wheel radius (meters)
#define NB_SENSORS          8       // Number of distance sensors
#define MIN_SENS          350       // sensibility value
#define MAX_SENS         4096       // Minimum sensibility value
#define MAX_SPEED         800       // Maximum speed

// Formation types
#define LINE          0
#define COLUMN        1
#define WEDGE         2
#define DIAMOND       3

// Message types
#define MSG_INIT_PARAMS     1
#define MSG_POSITION_INIT   2
#define MSG_POSITION        3

// Others
#define FORMATION_SIZE      4       // Size of flock
#define TIME_STEP          64       // [ms] Length of time step



// initializations

int robot_id_u, robot_id;           // Unique and normalized robot ID(between 0 and FORMATION_SIZE-1)
char* robot_name; 


// != 0 if initial positions have been received
int initialized[FORMATION_SIZE];
int initialized_weights[FORMATION_SIZE];


// Locations
float loc[FORMATION_SIZE][3];               // X, Z, Theta
                                            //  --> Theta is between negative z-axis and forward 
                                            //      direction, measured counter-clockwise
float prev_loc[FORMATION_SIZE][3];          // Previous X, Z, Theta values
float prev_relative_pos[FORMATION_SIZE][3];
float relative_pos[FORMATION_SIZE][3];
float unit_center[3];		    	        // X, Z, Theta of the unit center of all robots

float obstacle_loc[6][2];
float tryVariable;

// Migration vector
float migr[2];


// Speeds in absolute x and z direction
float speed[FORMATION_SIZE][2];


// angles between negative x-axis and sensor directions
extern const float sens_dir[NB_SENSORS];


// Devices
WbDeviceTag dist_sens[NB_SENSORS];  // Handle for the infra-red distance sensors
WbDeviceTag receiver;		        // Handle for the receiver node
WbDeviceTag emitter;		        // Handle for the emitter node
WbDeviceTag receiver2;		        // Handle for the receiver node
WbDeviceTag emitter2;		        // Handle for the emitter node


// Type of formation
int formation_type;


// motorschema weights
float w_goal;
float w_keep_formation;
float w_avoid_robot;
float w_avoid_obstacles;
float w_noise;

// thresholds
float avoid_robot_min_threshold;
float avoid_robot_max_threshold;
float avoid_obst_min_threshold;
float avoid_obst_max_threshold;
float keep_formation_min_threshold; // < min threshold: dead zone, no correction of direction
float keep_formation_max_threshold; // > max threshold: ballistic zone, full correction of direction
float move_to_goal_min_threshold;
float move_to_goal_max_threshold;

// noise
int noise_gen_frequency;  // defines, after how many steps a new random vector should be generated
int fading;               // true, if nice transition is wished from one random vector to the next

// time steps since reset
int time_steps_since_start;



// Methods

void reset(void);
void init_pos(char* inbuffer);
void init_params(char* inbuffer);
void compute_unit_center(void);
void update_self_motion(int msl, int msr);
void compute_wheel_speeds(int *msl, int *msr);

#endif
