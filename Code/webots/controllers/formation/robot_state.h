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



// Robot specs

#define AXLE_LENGTH         0.052   // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS     0.00628 // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS        0.0205  // Wheel radius (meters)
#define NB_SENSORS          8       // Number of distance sensors
#define MIN_SENS          350       // sensibility value
#define MAX_SENS         4096       // Minimum sensibility value
#define MAX_SPEED         800       // Maximum speed



// other definitions

#define FORMATION_SIZE      4       // Size of flock
#define TIME_STEP          64       // [ms] Length of time step



// initializations

int robot_id_u, robot_id;           // Unique and normalized robot ID(between 0 and FORMATION_SIZE-1)
char* robot_name; 

int initialized[FORMATION_SIZE];    // != 0 if initial positions have been received
float prev_loc[FORMATION_SIZE][3];  // Previous X, Z, Theta values
float loc[FORMATION_SIZE][3];       // X, Z, Theta of all robots
float prev_relative_pos[FORMATION_SIZE][3];
float relative_pos[FORMATION_SIZE][3];
float speed[FORMATION_SIZE][2];     // Speeds in x and z direction
float unit_center[3];		    	// X, Z, Theta of the unit center of all robots

WbDeviceTag dist_sens[NB_SENSORS];  // Handle for the infra-red distance sensors
WbDeviceTag receiver;		        // Handle for the receiver node
WbDeviceTag emitter;		        // Handle for the emitter node

float migr[2];//={25, 25};          // Migration vector



// methods

void reset(void);
void initial_pos(void);
void compute_unit_center(void);
void update_self_motion(int msl, int msr);
void compute_wheel_speeds(int *msl, int *msr);


#endif
