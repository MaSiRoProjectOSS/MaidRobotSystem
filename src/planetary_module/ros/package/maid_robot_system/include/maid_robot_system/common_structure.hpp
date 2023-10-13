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
        this->set(input_x, input_y, input_z);
    }

    void set(double input_x, double input_y, double input_z)
    {
        this->x = input_x;
        this->y = input_y;
        this->z = input_z;
    }
};

class St2DRectangle {
public:
    int x        = 0; //!< x-axis
    int y        = 0; //!< y-axis
    int width    = 0;
    int height   = 0;
    double angle = 0;

    St2DRectangle(int input_x = 0, int input_y = 0, int input_width = 0, int input_height = 0, double input_angle = 0.0)
    {
        this->set(input_x, input_y, input_width, input_height, input_angle);
    }

    void set(int input_x, int input_y, int input_width, int input_height, double input_angle)
    {
        this->x      = input_x;
        this->y      = input_y;
        this->width  = input_width;
        this->height = input_height;
        this->angle  = input_angle;
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
        this->set(input_r, input_g, input_b, input_alpha);
    }

    void set(int input_r, int input_g, int input_b, int input_alpha)
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
        this->set(input_x, input_y, input_width, input_height);
    }

    void set_size(double input_width, double input_height)
    {
        this->width  = input_width;
        this->height = input_height;
    }

    void set_axis(double input_x, double input_y)
    {
        this->x = input_x;
        this->y = input_y;
    }

    void set(double input_x, double input_y, double input_width, double input_height)
    {
        this->set_axis(input_x, input_y);
        this->set_size(input_width, input_height);
    }
};

#endif // _LIB_AXIS_H_
