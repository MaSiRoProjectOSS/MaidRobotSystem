/**
 * @file arduino_math.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.2
 * @date 2023-02-23
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef ARDUINO_FUNC_ARDUINO_MATH_HPP
#define ARDUINO_FUNC_ARDUINO_MATH_HPP

#include <math.h>

#define PI         (3.1415926535897932384626433832795)
#define HALF_PI    (1.5707963267948966192313216916398)
#define TWO_PI     (6.283185307179586476925286766559)
#define DEG_TO_RAD (0.017453292519943295769236907684886)
#define RAD_TO_DEG (57.295779513082320876798154814105)
#define EULER      (2.718281828459045235360287471352)

#define abs(x)                    ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
long map(long, long, long, long, long);
#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define pow(a, b) ((x) ^ (b))
#define sq(x)     ((x) * (x))
// sqrt()

#endif
