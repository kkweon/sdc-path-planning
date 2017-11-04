#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helper_math.hpp"
#include "json.hpp"
#include "map.hpp"
#include "spline.h"
#include "state_machine.hpp"
#include "variables.hpp"
#include "vehicle.hpp"

using namespace std;
// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1         = s.find_first_of("[");
  auto b2         = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from the file
  string map_file_ = "../data/highway_map.csv";
  ifstream in_map_(map_file_.c_str(), ifstream::in);
  string line;
  build_map(in_map_, map_waypoints_x, map_waypoints_y, map_waypoints_s,
            map_waypoints_dx, map_waypoints_dy);

  // Initial state
  int lane       = 1;
  double ref_vel = 0.0;

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &lane, &ref_vel,
               &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char* data,
                                  size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // Main car's localization Data

          VehicleState state = parse_JSON(j);
          int prev_size      = state.previous_path_x.size();
          json msgJson;

          // TODO: define a path made up of (x,y) points that the car will visit
          // sequentially every .02 seconds
          if (prev_size > 0) {
            state.s = state.end_path_s;
          }

          // bool too_close          = false;
          // maybe<int> current_lane = get_driving_lane(state);
          // bool left_is_safe       = true;
          // bool right_is_safe      = true;

          // double space_on_left  = 10000;
          // double space_on_right = 10000;

          // if (current_lane == just<int>(0)) {
          //   left_is_safe = false;
          // } else if (current_lane == just<int>(2)) {
          //   right_is_safe = false;
          // }

          const auto distances = fwd::apply(
              state.sensor_fusions,
              fwd::transform([&state](const auto& sensor) {
                maybe<int> lane = get_driving_lane(sensor);
                double dist     = get_future_distance(state, sensor);
                return std::make_pair(
                    lane == nothing<int>() ? -1 : unsafe_get_just(lane), dist);
              }),
              fwd::pairs_to_map_grouped(), fwd::map_to_pairs(),
              fwd::transform([](const auto& pair_in) {
                const auto dists = pair_in.second;
                const auto min_positive =
                    fwd::apply(dists, fwd::keep_if(fwd::is_positive()),
                               fwd::minimum_maybe());
                const auto max_negative =
                    fwd::apply(dists, fwd::keep_if(fwd::is_negative()),
                               fwd::maximum_maybe());

                double upper_bound = min_positive == nothing<double>()
                                         ? 10000
                                         : unsafe_get_just(min_positive);
                double lower_bound = max_negative == nothing<double>()
                                         ? -10000
                                         : unsafe_get_just(max_negative);
                return std::make_pair(pair_in.first,
                                      std::make_pair(upper_bound, lower_bound));
              }));

          bool too_close  = false;
          bool left_safe  = lane == 0 ? false : true;
          bool right_safe = lane == env::NUMBER_OF_LANES - 1 ? false : true;

          double left_space;
          double right_space;

          for (const auto& row : distances) {
            // valid [lane_id, [front_dist, back_dist]]
            const int check_lane = row.first;
            const double front_dist = row.second.first;
            const double back_dist  = row.second.second;

            if (check_lane == lane) {
              if (front_dist <= 30) {
                too_close = true;
              }
            } else if (row.first != -1) {
              int lane_diff = check_lane - lane;

              assert(fabs(lane_diff) <= 2);

              if (lane_diff == 1) {
                // right lane
                if (front_dist <= 30 || back_dist >= -20) {
                  right_safe = false;
                } else {
                  right_space = front_dist - back_dist;
                }
              } else if (lane_diff == -1) {
                // left-lane
                if (front_dist <= 30 || back_dist >= -20) {
                  left_safe = false;
                } else {
                  left_space = front_dist - back_dist;
                }
              }
            }
          }


          if (too_close) {
            if (!left_safe && !right_safe) {
              ref_vel -= .4;
            } else if (left_safe && right_safe) {
              if (left_space > right_space) {
                lane--;
              } else {
                lane++;
              }
            } else if (left_safe) {
              lane--;
            } else if (right_safe) {
              lane++;
            }
          } else if (ref_vel < env::MAX_VELOCITY) {
            ref_vel += .4;
          }


          // A list of widely spaced (x, y) waypoints, evenly spaced at 30m
          // It's going to be interpolated later
          vector<double> ptsx;
          vector<double> ptsy;

          // Reference state (x, y, yaw)
          // at car position or at the end of previous path point
          double ref_x   = state.x;
          double ref_y   = state.y;
          double ref_yaw = deg2rad(state.yaw);

          if (prev_size < 2) {
            // If the previous size is almost empty
            // use current points(state.x, state.y) and previous points
            double prev_car_x = state.x - cos(state.yaw);
            double prev_car_y = state.y - sin(state.yaw);

            ptsx.push_back(prev_car_x);
            ptsy.push_back(prev_car_y);

            ptsx.push_back(state.x);
            ptsy.push_back(state.y);
          } else {
            // If there are enough points, set to the last point of the previous
            // path
            ref_x = state.previous_path_x[prev_size - 1];
            ref_y = state.previous_path_y[prev_size - 1];

            double ref_x_prev = state.previous_path_x[prev_size - 2];
            double ref_y_prev = state.previous_path_y[prev_size - 2];
            ref_yaw           = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsy.push_back(ref_y_prev);

            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y);
          }

          // Create evenly 30m spaced waypoints ahead of the reference points
          // From the closest to the farthest
          double s_close = state.s + 30;
          double s_mid   = state.s + 60;
          double s_far   = state.s + 90;

          // There are 3 lanes with each width of 4m
          // if lane == 0 then 2 is center and so on
          double d_keep = 2 + 4 * lane;

          auto getXY_with_map = [&map_waypoints_s, &map_waypoints_x,
                                 &map_waypoints_y](double s, double d) {
            return getXY(s, d, map_waypoints_s, map_waypoints_x,
                         map_waypoints_y);
          };

          vector<vector<double>> next_wps;

          next_wps.push_back(getXY_with_map(s_close, d_keep));
          next_wps.push_back(getXY_with_map(s_mid, d_keep));
          next_wps.push_back(getXY_with_map(s_far, d_keep));

          for (vector<double>& wp : next_wps) {
            ptsx.push_back(wp[0]);
            ptsy.push_back(wp[1]);
          }

          // change perspective from the `ref` point of view
          // transform such that car is driving along at the angle 0 at (0, 0)
          // coordinates
          // makes the computation easier
          // y ^
          //   |          driving direction
          //   |           ---->
          //   |
          //   |
          //   |  +---------+
          //   +--+ car  -> +----------> x
          //      +---------+
          for (auto i = 0; i < ptsx.size(); ++i) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          tk::spline s;
          s.set_points(ptsx, ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (auto i = 0; i < state.previous_path_x.size(); ++i) {
            next_x_vals.push_back(state.previous_path_x[i]);
            next_y_vals.push_back(state.previous_path_y[i]);
          }

          double target_x    = 30.0;
          double target_y    = s(target_x);
          double target_dist = sqrt(square(target_x) + square(target_y));

          double x_add_on = 0;

          for (int i = 1; i <= 50 - state.previous_path_x.size(); ++i) {
            // N * 0.02 * ref_vel = target_dist
            // divide by 2.24 to convert miles/h to m/h
            double N       = target_dist / (.02 * ref_vel / 2.24);
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // Rotate back to the normal coordinates
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          // END
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          // this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse* res, uWS::HttpRequest req, char* data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char* message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
