//
// Created by NEUEM2 on 04.08.2019.
//

#ifndef COST_H
#define COST_H

#include <vector>

using std::vector;

float inefficiency_cost(float target_speed,
                         int intended_lane,
                         int final_lane,
                         vector<float> lane_speeds);


#endif //PATH_PLANNING_COST_H
