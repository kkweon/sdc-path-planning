#ifndef MAP_HPP
#define MAP_HPP
#include <fstream>
#include <vector>
#include "helper_math.hpp"

using std::vector;
using std::ifstream;
using Doubles = vector<double>;

double distance(double x1, double y1, double x2, double y2);

int getClosestWaypoint(double x, double y, const vector<double>& maps_x,
                       const vector<double>& maps_y);

int getNextWaypoint(double x, double y, double theta,
                    const vector<double>& maps_x, const vector<double>& maps_y);

vector<double> getFrenet(double x, double y, double theta,
                         const vector<double>& maps_x,
                         const vector<double>& maps_y);

vector<double> getXY(double s, double d, const vector<double>& maps_s,
                     const vector<double>& maps_x,
                     const vector<double>& maps_y);

void build_map(ifstream& in_map_, Doubles& map_waypoints_x,
               Doubles& map_waypoints_y, Doubles& map_waypoints_s,
               Doubles& map_waypoints_dx, Doubles& map_waypoints_dy);

#endif
