#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include <map>
#include <vector>

using std::vector;
using std::map;

enum class ActionState {
  KEEP_LANE,
  LANE_CHANGE_LEFT,
  LANE_CHANGE_RIGHT,
  PREPARE_LANE_CHANGE_LEFT,
  PREPARE_LANE_CHANGE_RIGHT
};

int get_lane(double d, int n_lane);

double compute_cost(vector<vector<double>> sensor_fusion,
                    ActionState action, double car_s, double car_d,
                    double target_speed);

#endif
