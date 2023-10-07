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

/**
 * @brief Vector structs
 */
class StVector {
public:
    double x = 0; //!< x-axis
    double y = 0; //!< y-axis
    double z = 0; //!< z-axis

    /**
     * @brief Construct a new st lib axis object
     *
     * @param input_x X軸の位置
     * @param input_y Y軸の位置
     * @param input_z Z軸の位置
     */
    StVector(double input_x = 0.0, double input_y = 0.0, double input_z = 0.0)
    {
        set(input_x, input_y, input_z);
    }

    /**
     * @brief 値を設定する
     *
     * @param input_x X軸の位置
     * @param input_y Y軸の位置
     * @param input_z Z軸の位置
     */
    void set(double input_x = 0.0, double input_y = 0.0, double input_z = 0.0)
    {
        x = input_x;
        y = input_y;
        z = input_z;
    }
};

/**
 * @brief 矩形を扱う際の構造体
 */
class StRectangle {
public:
    double x      = 0;
    double y      = 0;
    double width  = 0;
    double height = 0;

    /**
     * @brief Construct a new st lib rectangles object
     *
     * @param input_x       X軸の位置を設定する
     * @param input_y       Y軸の位置を設定する
     * @param input_width   幅を設定する
     * @param input_height  高さを設定する
     */
    StRectangle(double input_x = 0.0, double input_y = 0.0, double input_width = 0.0, double input_height = 0.0)
    {
        set(input_x, input_y, input_width, input_height);
    }
    /**
     * @brief Set the size object
     *
     * @param input_width   幅を設定する
     * @param input_height  高さを設定する
     */
    void set_size(double input_width, double input_height)
    {
        width  = input_width;
        height = input_height;
    }
    /**
     * @brief Set the axis object
     *
     * @param input_x       X軸の位置を設定する
     * @param input_y       Y軸の位置を設定する
     */
    void set_axis(double input_x, double input_y)
    {
        x = input_x;
        y = input_y;
    }

    /**
     * @brief 値を設定する
     *
     * @param input_x       X軸の位置を設定する
     * @param input_y       Y軸の位置を設定する
     * @param input_width   幅を設定する
     * @param input_height  高さを設定する
     */
    void set(double input_x, double input_y, double input_width, double input_height)
    {
        set_axis(input_x, input_y);
        set_size(input_width, input_height);
    }
};

#endif // _LIB_AXIS_H_
