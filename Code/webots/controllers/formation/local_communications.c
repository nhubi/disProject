#include <stdio.h>
#include <math.h>
#include <string.h>

#include "local_communications.h"


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// This file contains the definitions related to the local communications                         //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 * Each robot sends a ping message, so the other robots can measure relative range and bearing to 
 * the sender. This is useful if you want to use relative range and bearing. This would be more 
 * realistic and less dependent on the supervisor. Try to make this work use 
 * process_received_ping_messages() function in lab 4 as a base to calculate range and bearing to 
 * the other robots.
 */
 
void send_ping(void)  {
    char out[10];
    strcpy(out,robot_name);  // in the ping message we send the name of the robot.
    wb_emitter_send(emitter,out,strlen(out)+1); 
}

/*
 * processing all the received ping messages, and calculate range and bearing to the other robots
 * the range and bearing are measured directly out of message RSSI and direction
*/
void process_received_ping_messages(int robot_id) {
    const double *message_direction;
    double message_rssi; // Received Signal Strength indicator
    double theta;
    double range;
    char *inbuffer;	// Buffer for the receiver node
    int other_robot_id;
    int count=0;
    while (wb_receiver_get_queue_length(receiver) > 0) {// && count < FORMATION_SIZE) {
        count++;
        inbuffer = (char*) wb_receiver_get_data(receiver);
        message_direction = wb_receiver_get_emitter_direction(receiver);
        message_rssi = wb_receiver_get_signal_strength(receiver);
        double y = message_direction[2];
        double x = message_direction[1];

        theta =	-atan2(y,x);
        theta = theta + loc[robot_id][2]; // find the relative theta;
        range = sqrt((1/message_rssi)); 

        other_robot_id = (int)(inbuffer[3]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!
        printf("Other robot %d\n",other_robot_id);
		
        // Get position update
        prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
        prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];
        prev_relative_pos[other_robot_id][2] = relative_pos[other_robot_id][2];

        relative_pos[other_robot_id][0] = range*cos(theta);  // relative x pos
        relative_pos[other_robot_id][1] = -1.0 * range*sin(theta);   // relative y pos
        relative_pos[other_robot_id][2] = theta; // relative theta pos 

        //printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, my theta %g\n",robot_name,other_robot_id,relative_pos[other_robot_id][0],relative_pos[other_robot_id][1],my_position[2]*180.0/3.141592,my_position[2]*180.0/3.141592);

		
        // (Stefano) not important by now
        //relative_speed[other_robot_id][0] = (1/DELTA_T)*(relative_pos[other_robot_id][0]-prev_relative_pos[other_robot_id][0]);
        //relative_speed[other_robot_id][1] = (1/DELTA_T)*(relative_pos[other_robot_id][1]-prev_relative_pos[other_robot_id][1]);
        wb_receiver_next_packet(receiver);
    }
}

/*
 * Compute the localisation of the other robot using relative_pos
 */
void compute_other_robots_localisation(int robot_id) {
    int i,j;
    for (i=0;i<FORMATION_SIZE;i++) {
        if (i==robot_id) {
            continue;
        }
        for (j=0;j<3;j++) {
            loc[i][j]=loc[robot_id][j]+relative_pos[i][j];
        }
    }
}