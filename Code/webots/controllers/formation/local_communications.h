////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                //
// This file contains the definitions related to the local communications                         //
//                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef LOCAL_COMMUNICATIONS
#define LOCAL_COMMUNICATIONS


#include "robot_state.h"

void send_ping(void);

void process_received_ping_messages(int robot_id);

void compute_other_robots_localisation(int robot_id);




#endif