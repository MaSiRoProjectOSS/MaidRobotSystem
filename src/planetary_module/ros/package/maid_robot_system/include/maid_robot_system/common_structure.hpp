/**
 * @file common_structure.hpp
 * @brief
 * @date 2020-03-01
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#ifndef MAID_ROBOT_SYSTEM_COMMON_STRUCTURE_HPP
#define MAID_ROBOT_SYSTEM_COMMON_STRUCTURE_HPP

#include <math.h>
#include <random>

/**
 * @brief Get the Random object
 *
 * @param min
 * @param max
 * @return double
 */
static double func_rand(double min, double max)
{
    return min + ((max - min) * ((double)rand() / (1.0 + (double)RAND_MAX)));
}

class StVector {
public:
    double x = 0; //!< x-axis
    double y = 0; //!< y-axis
    double z = 0; //!< z-axis

    StVector(double input_x = 0.0, double input_y = 0.0, double input_z = 0.0)
    {
        set(input_x, input_y, input_z);
    }

    void set(double input_x = 0.0, double input_y = 0.0, double input_z = 0.0)
    {
        x = input_x;
        y = input_y;
        z = input_z;
    }
};

class St2DPostion {
public:
    double x     = 0; //!< x-axis
    double y     = 0; //!< y-axis
    double angle = 0;
    double size  = 0;

    St2DPostion(double input_x = 0.0, double input_y = 0.0, double input_angle = 0.0, double input_size = 0.0)
    {
        set(input_x, input_y, input_angle, input_size);
    }

    void set(double input_x = 0.0, double input_y = 0.0, double input_angle = 0.0, double input_size = 0.0)
    {
        this->x     = input_x;
        this->y     = input_y;
        this->angle = input_angle;
        this->size  = input_size;
    }
};

class StColor {
public:
    int r     = 0;
    int g     = 0;
    int b     = 0;
    int alpha = 0;

    StColor(int input_r = 0, int input_g = 0, int input_b = 0, int input_alpha = 0)
    {
        set(input_r, input_g, input_b, input_alpha);
    }

    void set(int input_r = 0, int input_g = 0, int input_b = 0, int input_alpha = 0)
    {
        this->r     = input_r;
        this->g     = input_g;
        this->b     = input_b;
        this->alpha = input_alpha;
    }
};
class StRectangle {
public:
    double x      = 0;
    double y      = 0;
    double width  = 0;
    double height = 0;

    StRectangle(double input_x = 0.0, double input_y = 0.0, double input_width = 0.0, double input_height = 0.0)
    {
        set(input_x, input_y, input_width, input_height);
    }

    void set_size(double input_width, double input_height)
    {
        width  = input_width;
        height = input_height;
    }

    void set_axis(double input_x, double input_y)
    {
        x = input_x;
        y = input_y;
    }

    void set(double input_x, double input_y, double input_width, double input_height)
    {
        set_axis(input_x, input_y);
        set_size(input_width, input_height);
    }
};

#endif // _LIB_AXIS_H_
