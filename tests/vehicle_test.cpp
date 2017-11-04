#include "vehicle.hpp"
#include "catch.hpp"
#include "variables.hpp"

TEST_CASE("get_velocity can compute velocity", "[get_velocity]") {
  // vx = 10, vy = 10
  // then speed is sqrt(square(vx) + square(vy))
  SensorData temp{0, 0, 0, 10, 10, 0, 0};
  REQUIRE(get_velocity(temp) == Approx(sqrt(200)));
}

TEST_CASE("get_driving_lane for VehicleState", "[get_driving_lane]") {
  // d = 10 -> lane: 2
  VehicleState temp;
  temp.d = 10;
  REQUIRE(get_driving_lane(temp) == just<int>(2));

  temp.d = -10;
  REQUIRE(get_driving_lane(temp) == nothing<int>());

  temp.d = 12.1;
  REQUIRE(get_driving_lane(temp) == nothing<int>());
}

TEST_CASE("get_driving_lane for SensorData", "[get_driving_lane]") {
  // d = 10 -> lane: 2
  SensorData temp;
  temp.d = 10;
  REQUIRE(get_driving_lane(temp) == just<int>(2));

  temp.d = -10;
  REQUIRE(get_driving_lane(temp) == nothing<int>());

  temp.d = 12.1;
  REQUIRE(get_driving_lane(temp) == nothing<int>());
}

TEST_CASE("get_adjusted_s for SensorData", "[get_adjusted_s]") {
  // velocity sqrt(200);
  SensorData temp{0, 0, 0, 10, 10, 0, 0};
  temp.s = 10;
  int prev_size = 10;
  REQUIRE(get_adjusted_s(temp, prev_size) == Approx(temp.s + static_cast<double>(prev_size) * env::UPDATE_FREQ * sqrt(200)));
}

TEST_CASE("get_distance", "[get_distance]") {
  SensorData temp;
  temp.s = 0;
  VehicleState state;
  state.s = 10;
  REQUIRE(get_distance(state, temp) == Approx(10.0));


  temp.s = 20;
  REQUIRE(get_distance(state, temp) == Approx(-10.0));
}
