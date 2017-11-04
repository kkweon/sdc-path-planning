#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include <fplus/fplus.hpp>
#include <vector>
#include "json.hpp"

using json = nlohmann::json;
using namespace fplus;
using Doubles = std::vector<double>;

// Format: [ id, x, y, vx, vy, s, d]
struct SensorData {
  int id;
  double x, y;
  double vx, vy;
  double s, d;
};

struct VehicleState {
  double x, y;
  double yaw;
  double s, d;
  double v, target_speed;
  std::vector<SensorData> sensor_fusions;
  json previous_path_x;
  json previous_path_y;

  double end_path_s;
  double end_path_d;
};

VehicleState parse_JSON(const json& data);
SensorData parse_sensor_JSON(const Doubles& data);
double get_velocity(const SensorData& sensor_data);
double get_adjusted_s(const SensorData& sensor_data, int prev_size);

maybe<int> get_driving_lane(const VehicleState& state);
maybe<int> get_driving_lane(const SensorData& sensor_data);

double get_distance(const VehicleState& state, const SensorData& sensor_data);
double get_future_distance(const VehicleState& state, const SensorData& sensor_data);
bool will_collide(const VehicleState& state, const SensorData& sensor_data);

#endif
