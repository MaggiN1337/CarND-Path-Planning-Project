//
// Updated by Marcus Neuert on 04.08.2019.
//

#include <vector>

using std::vector;

float inefficiency_cost(float target_speed,
                        int intended_lane,
                        int final_lane,
                        vector<float> lane_speeds) {
    // Cost becomes higher for trajectories with intended lane and final lane
    //   that have traffic slower than vehicle's target speed.

    float cost = (2*target_speed - lane_speeds[intended_lane]
                  - lane_speeds[final_lane])/target_speed;

    return cost;
}

