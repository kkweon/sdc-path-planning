#ifndef HELPER_MATH_HPP
#define HELPER_MATH_HPP

#include <cmath>

// For converting back and forth between radians and degrees.
constexpr double PI() { return M_PI; }
constexpr double deg2rad(double x) { return x * PI() / 180; }
constexpr double rad2deg(double x) { return x * 180 / PI(); }
constexpr double sigmoid(double x) { return 1. / (1. + exp(-x)); }

#endif
