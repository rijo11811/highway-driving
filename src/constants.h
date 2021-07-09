#ifndef CONSTS_H
#define CONSTS_H

// impacts default behavior for most states
double SPEED_LIMIT = 49/2.24;//mps

// all traffic in lane (besides ego) follow these speeds
vector<double> LANE_SPEEDS = {49/2.24 , 49/2.24 , 49/2.24};

double MAX_ACCEL = 8; //mps*2

double T= 5;

#endif  // HELPERS_H
