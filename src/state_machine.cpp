#include "state_machine.hpp"
#include <limits>

int get_lane(double d, int n_lane) {
  // Each lane is 4m width
  int width = 4;
  if (d < 0) {
    // no lane;
    return -1;
  } else if (d >= 12) {
    return -1;
  }

  return static_cast<int>(d) / 4;
}

double compute_cost(vector<vector<double>> sensor_fusion,
                    ActionState action, double car_s, double car_d,
                    double target_speed) {
  map<int, double> front_gaps;
  // Number of lane
  int n_lane = 3;
  for (auto lane = 0; lane < 3; ++lane) {
    front_gaps[lane] = std::numeric_limits<double>::max();
  }

  for (const auto& entry : sensor_fusion) {
    // [ id, x, y, vx, vy, s, d]
    auto id   = entry[0];
    auto lane = get_lane(entry[6], n_lane);
    auto s    = entry[5];
    auto gap  = s - car_s;

    if (lane >= 0 && 0 < gap && gap < front_gaps[lane]) {
      front_gaps[lane] = gap;
    }
  }

  int future_lane;
  double turn_penalty = 0.0;

  if (action == ActionState::LANE_CHANGE_RIGHT ||
      action == ActionState::PREPARE_LANE_CHANGE_RIGHT) {
    future_lane = get_lane(car_d + 4, n_lane);
    turn_penalty = 5;
  } else if (action == ActionState::LANE_CHANGE_LEFT ||
             action == ActionState::PREPARE_LANE_CHANGE_LEFT) {
    future_lane = get_lane(car_d - 4, n_lane);
    turn_penalty = 5;
  } else if (action == ActionState::KEEP_LANE) {
    future_lane = get_lane(car_d, n_lane);
  }

  if (future_lane == -1) {
    // out of road;
    return std::numeric_limits<double>::max();
  }
  return 1.5 * target_speed / front_gaps[future_lane] + turn_penalty;
}
