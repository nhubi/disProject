#include <stdio.h>
#include <math.h>
#include <string.h>

#include "ms_keep_formation.h"


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// This file contains everything related to the motorschema 'keep_formation'.                     //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////



/*
 * Calculates a robot's vector that points to a direction which avoids obstacles.
 */
void get_keep_formation_vector(float* direction, int robot_id, int formation_type){
    if(FORMATION_SIZE != 4)
    {
        printf("No formation is set for %d robots. Formation is considered only for a set of 4 robots.\n", FORMATION_SIZE);
        direction[0] = 0.0;
        direction[1] = 0.0;
        return;
    }
    
    direction[0] = 0.0;
    direction[1] = 0.0;
}


/* 
 * Computes the position where each robot should be if it where on formation,
 * in the carthesian system defined by:
 *    -> Origin : the unit_center
 *    -> Direction of the z-axis : move_to_goal vector
 *    -> Direction of the x-axis : perpendicular to the z-axis so that the angle 
 *       from the z-axis to the x-axis is -pi/2
 */
void get_formation_coordinates(float* coordinates, int robot_id, int formation_type) {
    if(formation_type == LINE) {
        coordinates[1] = 0.0;
        if(robot_id == 0)
            coordinates[0] = coef;
        else if(robot_id == 1)
            coordinates[0] = 3.0 * coef;
        else if(robot_id == 2)
            coordinates[0] = -1.0 * coef;
        else
            coordinates[0] = -3.0 * coef;
    } 
    else if(formation_type == COLUMN) {
        coordinates[0] = 0.0;
        if(robot_id == 0)
            coordinates[1] = 3.0 * coef;
        else if(robot_id == 1)
            coordinates[1] = -1.0 * coef;
        else if(robot_id == 2)
            coordinates[1] = 1.0 * coef;
        else
            coordinates[1] = -3.0 * coef;
    } 
    else if(formation_type == WEDGE) {
        if(robot_id == 0) {
            coordinates[0] = coef;
            coordinates[1] = coef;
        } else if(robot_id == 1) {
            coordinates[0] = 3.0 * coef;
            coordinates[1] = -1.0 * coef;
        } else if(robot_id == 2) {
            coordinates[0] = -1.0 * coef;
            coordinates[1] = coef;
        } else {
            coordinates[0] = -3.0 * coef;
            coordinates[1] = -1.0 * coef;
        }
    } 
    else if(formation_type == DIAMOND) {
        if(robot_id == 0) {
            coordinates[0] = 0.0;
            coordinates[1] = 2.0 * coef;
        } else if(robot_id == 1) {
            coordinates[0] = 2.0 * coef;
            coordinates[1] = 0.0;
        } else if(robot_id == 2) {
            coordinates[0] = -2.0 * coef;
            coordinates[1] = 0.0;
        } else {
            coordinates[0] = 0.0;
            coordinates[1] = -2.0 * coef;
        }
    } 
    else {
        printf("Error: wrong formation type when getting robots formation local coordinates.\n"); 
        // this error message should never be printed
    }
}





