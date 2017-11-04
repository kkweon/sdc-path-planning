#include "vehicle.hpp"
#include "variables.hpp"

int __get_driving_lane(double d) {
  return static_cast<int>(d) / env::LANE_WIDTH;
}

bool __invalid_lane(int lane) {
  if (lane < 0 || lane >= env::NUMBER_OF_LANES) {
    return true;
  }
  return false;
}

double get_future_distance(const VehicleState& state, const SensorData& sensor_data) {
  int prev_size = state.previous_path_x.size();
  double s_adjusted = get_adjusted_s(sensor_data, prev_size);
  return s_adjusted - state.end_path_s;
}

bool will_collide(const VehicleState& state, const SensorData& sensor_data) {
  auto dist = get_distance(state, sensor_data);
  if (fabs(dist) < 20) {
    return true;
  }
  return false;
}

maybe<int> get_driving_lane(const VehicleState& state) {
  int lane = __get_driving_lane(state.d);
  if (__invalid_lane(lane)) {
    return nothing<int>();
  }
  return just<int>(lane);
}

VehicleState parse_JSON(const json& data) {
  VehicleState b;

  b.x   = data[1]["x"];
  b.y   = data[1]["y"];
  b.s   = data[1]["s"];
  b.d   = data[1]["d"];
  b.yaw = data[1]["yaw"];
  b.v   = data[1]["speed"];

  // Previous path data given to the Planner
  b.previous_path_x = data[1]["previous_path_x"];
  b.previous_path_y = data[1]["previous_path_y"];
  // Previous path's end s and d values
  b.end_path_s = data[1]["end_path_s"];
  b.end_path_d = data[1]["end_path_d"];

  // Sensor Fusion Data, a list of all other cars on the same side of
  // the road.
  std::vector<std::vector<double>> _fusion = data[1]["sensor_fusion"];
  b.sensor_fusions =
      fwd::apply(_fusion, fwd::transform(parse_sensor_JSON));

  return b;
}

SensorData parse_sensor_JSON(const Doubles& data) {
  // each sensor_fusion: [ id, x, y, vx, vy, s, d]
  SensorData b;

  b.id = data[0];
  b.x  = data[1];
  b.y  = data[2];
  b.vx = data[3];
  b.vy = data[4];
  b.s  = data[5];
  b.d  = data[6];

  return b;
}

maybe<int> get_driving_lane(const SensorData& sensor) {
  int lane = __get_driving_lane(sensor.d);
  if (__invalid_lane(lane)) {
    return nothing<int>();
  }
  return just<int>(lane);
}

double get_velocity(const SensorData& sensor_data) {
  return sqrt(square(sensor_data.vx) + square(sensor_data.vy));
}

double get_adjusted_s(const SensorData& sensor_data, int prev_size) {
  double v = get_velocity(sensor_data);
  double s = sensor_data.s;
  return s + v * env::UPDATE_FREQ * static_cast<double>(prev_size);
}

double get_distance(const VehicleState& state, const SensorData& sensor_data) {
  return state.s - sensor_data.s;
}
