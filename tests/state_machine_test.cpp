#include "state_machine.hpp"
#include "catch.hpp"

SCENARIO("compute_cost returns cost successfully", "[compute_cost]") {
  GIVEN("there is no car in front") {
    vector<vector<double>> sensor_fusions{
        // id, x, y, vx, vy, s, d
        {0, 775.99, 1421.6, 0, 0, 6721.839, -277.6729},
        {1, 775.8, 1425.2, 0, 0, 6719.219, -280.1494}};

    ActionState action  = ActionState::KEEP_LANE;
    double car_s        = 6800;
    double car_d        = 10;
    double target_speed = 49.5;

    THEN("the cost is zero") {
      REQUIRE(compute_cost(sensor_fusions, action, car_s, car_d,
                           target_speed) <= 1e-7);
    }
  }

  GIVEN("Driving mid lane") {
    double car_s        = 90;
    double car_d        = 6.;
    double target_speed = 49.5;

    WHEN("Only left lane is empty") {
      vector<vector<double>> sensor_fusions{{0, 775.99, 1421.6, 0, 0, 100, 6.},
                                            {1, 775.8, 1425.2, 0, 0, 100, 10.}};

      ActionState action  = ActionState::PREPARE_LANE_CHANGE_LEFT;

      THEN("cost left lane change is lower than KEEP_LANE") {
        auto left_cost = compute_cost(sensor_fusions, action, car_s, car_d, target_speed);

        auto center_cost = compute_cost(sensor_fusions, ActionState::KEEP_LANE, car_s, car_d, target_speed);

        REQUIRE(left_cost < center_cost);

      }

      THEN("cost left lane change is lower than Right Lane") {
        auto left_cost = compute_cost(sensor_fusions, action, car_s, car_d, target_speed);

        auto right_cost = compute_cost(sensor_fusions, ActionState::PREPARE_LANE_CHANGE_RIGHT, car_s, car_d, target_speed);

        REQUIRE(left_cost < right_cost);

      }
    }
  }
}


SCENARIO("get_lane/2 correctly works", "[get_lane]") {
  GIVEN("3 Lanes") {
    int n_lane = 3;
    WHEN("d is 0") {
      THEN("result is 0") {
        REQUIRE(get_lane(0, n_lane) == 0);
      }
    }
    WHEN("d is 4.5") {
      THEN("result is 1") {
        REQUIRE(get_lane(4.5, n_lane) == 1);
      }
    }

    WHEN("d is 10") {
      THEN("result lane is 2") {
        REQUIRE(get_lane(10, n_lane) == 2);
      }
    }
  }
}
