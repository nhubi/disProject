#include <stdio.h>
#include <math.h>
#include <string.h>

#include "ms_keep_formation.h"


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// This file contains everything related to the motorschema 'keep_formation'.                     //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////



//definitions
const float robot_dist = 1.0/12.0;



/*
 * Calculates a robot's vector that points to a direction which avoids obstacles.
 */
void get_keep_formation_vector(float* direction, float* dir_goal){//, int robot_id){
    if(FORMATION_SIZE != 4)
    {
        printf("No formation is set for %d robots. Formation is considered only for a set of 4 robots.\n", FORMATION_SIZE);
        direction[0] = 0.0;
        direction[1] = 0.0;
        return;
    }
    
    float relative_coordinates[3] = {0, 0, 0};
    float absolute_coordinates[3] = {0, 0, 0};
    
    get_relative_formation_coordinates(relative_coordinates);
    get_absolute_formation_coordinates(absolute_coordinates, relative_coordinates, dir_goal);
    direction[0] = absolute_coordinates[0] - loc[robot_id][0];
    direction[1] = absolute_coordinates[1] - loc[robot_id][1];
}


/* 
 * Computes the position where each robot should be if it where on formation,
 * in the carthesian system defined by:
 *    -> Origin : the unit_center
 *    -> Direction of the z-axis : move_to_goal vector
 *    -> Direction of the x-axis : perpendicular to the z-axis so that the angle 
 *       from the z-axis to the x-axis is -pi/2
 */
void get_relative_formation_coordinates(float* coordinates){//, int robot_id) {
    if(formation_type == LINE) {
        coordinates[1] = 0.0;
        if(robot_id == 0)
            coordinates[0] = robot_dist;
        else if(robot_id == 1)
            coordinates[0] = 3.0 * robot_dist;
        else if(robot_id == 2)
            coordinates[0] = -1.0 * robot_dist;
        else
            coordinates[0] = -3.0 * robot_dist;
    } 
    else if(formation_type == COLUMN) {
        coordinates[0] = 0.0;
        if(robot_id == 0)
            coordinates[1] = 3.0 * robot_dist;
        else if(robot_id == 1)
            coordinates[1] = -1.0 * robot_dist;
        else if(robot_id == 2)
            coordinates[1] = 1.0 * robot_dist;
        else
            coordinates[1] = -3.0 * robot_dist;
    } 
    else if(formation_type == WEDGE) {
        if(robot_id == 0) {
            coordinates[0] = robot_dist;
            coordinates[1] = robot_dist;
        } else if(robot_id == 1) {
            coordinates[0] = 3.0 * robot_dist;
            coordinates[1] = -1.0 * robot_dist;
        } else if(robot_id == 2) {
            coordinates[0] = -1.0 * robot_dist;
            coordinates[1] = robot_dist;
        } else {
            coordinates[0] = -3.0 * robot_dist;
            coordinates[1] = -1.0 * robot_dist;
        }
    } 
    else if(formation_type == DIAMOND) {
        if(robot_id == 0) {
            coordinates[0] = 0.0;
            coordinates[1] = 2.0 * robot_dist;
        } else if(robot_id == 1) {
            coordinates[0] = 2.0 * robot_dist;
            coordinates[1] = 0.0;
        } else if(robot_id == 2) {
            coordinates[0] = -2.0 * robot_dist;
            coordinates[1] = 0.0;
        } else {
            coordinates[0] = 0.0;
            coordinates[1] = -2.0 * robot_dist;
        }
    } 
    else {
        printf("Error: wrong formation type when getting robots formation local coordinates.\n"); 
        // this error message should never be printed
    }
}


/* 
 * Computes the position where each robot should be if it where on formation
 * in the carthesian system defined by the world, from the relative position
 * in the carthesian system defined by:
 *    -> Origin : the unit_center
 *    -> Direction of the z-axis : move_to_goal vector
 *    -> Direction of the x-axis : perpendicular to the z-axis so that the angle 
 *       from the z-axis to the x-axis is -pi/2
 */
void get_absolute_formation_coordinates(float* coordinates, float* relative_coordinates, float* dir_goal) {
    float dist_to_goal = norm(dir_goal, 2);
    
    // Theta is the angle between the x-axis and the direction vector to the goal.  
    float cosTheta = dir_goal[0] / dist_to_goal;
    float sinTheta = dir_goal[1] / dist_to_goal;

    // Changing system coordinates, rotation + translation, taking care of the right angle Theta
    coordinates[0] = relative_coordinates[0] * sinTheta
                   + relative_coordinates[1] * cosTheta 
                   + unit_center[0];
    coordinates[1] = - relative_coordinates[0] * cosTheta
                   + relative_coordinates[1] * sinTheta 
                   + unit_center[1];  
}

