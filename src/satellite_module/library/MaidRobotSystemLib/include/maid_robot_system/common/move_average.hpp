/**
 * @file move_average.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Classes that manage moving averages
 * @version 0.1
 * @date 2023-02-11
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MAID_ROBOT_SYSTEM_LIB_COMMON_MOVE_AVERAGE_HPP
#define MAID_ROBOT_SYSTEM_LIB_COMMON_MOVE_AVERAGE_HPP

#include <Arduino.h>

namespace maid_robot_system
{
namespace common
{

/**
 * @brief Classes that manage moving averages
 *
 * @param NUM : Number of arrays to store in moving average
 */
template <int NUM = 10>
class MoveAverage {
public:
    /**
     * @brief Construct a new Move Average object
     *
     * @param init_value: Value to be initialized when reset() is performed
     */
    MoveAverage(int init_value = 0);

public:
    /**
     * @brief Calculate moving averages
     *
     * @param value : Value to be added
     * @return int : Calculation Results
     */
    double set(int value);
    /**
     * @brief Calculate the amount of change in the added value
     *
     * @param value : Value to be added
     * @return double
     */
    double set_delta(int value);
    /**
     * @brief Reset stored information
     *
     * @param init_value
     */
    void reset(int init_value);
    /**
     * @brief Reset with the value set in the constructor
     */
    void reset();

    /**
     * @brief Get the value of a moving average in text
     *
     * @return String: [0], [1], [2] ...
     */
    String get_text();

public:
    /**
     * @brief The result of move_average()
     *
     */
    double average;
    /**
     * @brief The result of move_average_delta()
     *
     */
    double average_delta;

private:
    int _data_array[NUM]  = { 0 }; /*!< The array of move average */
    int _delta_array[NUM] = { 0 }; /*!< Array of changes */
    int _data_size        = 0;     /*!< data size */
    int _delta_size       = 0;     /*!< delta size */
    int _init_value       = 0;     /*!< initialized value */

    int _max_size = NUM; /*!< Maximum number of data */

private:
    const double RESOLUTION_US = 1000000.0; /*!< Checkpoint Time */
};

template <int NUM>
MoveAverage<NUM>::MoveAverage(int init_value)
{
    this->_init_value = init_value;
    this->reset(this->_init_value);
}

template <int NUM>
double MoveAverage<NUM>::set(int value)
{
    long sum_data = value;
    if (this->_max_size > this->_data_size) {
        this->_data_size++;
    }
    for (int i = (this->_data_size - 1); 0 < i; i--) {
        this->_data_array[i] = this->_data_array[i - 1];
        sum_data += this->_data_array[i];
    }
    this->_data_array[0] = value;
    this->average        = (double)sum_data / (double)this->_data_size;
    return this->average;
}

template <int NUM>
double MoveAverage<NUM>::set_delta(int value)
{
    static unsigned long previous_us = micros();
    long sum_data                    = 0;
    double sum_time                  = 0;

    unsigned long current_us = micros();
    unsigned long elapsed    = current_us - previous_us;
    previous_us              = current_us;
    if (this->_max_size > this->_data_size) {
        this->_data_size++;
    }

    for (int i = (this->_data_size - 1); 0 < i; i--) {
        this->_data_array[i]  = this->_data_array[i - 1];
        this->_delta_array[i] = this->_delta_array[i - 1];
        sum_data += this->_data_array[i];
        sum_time += this->_delta_array[i];
    }
    if (this->_max_size > this->_delta_size) {
        this->_delta_size++;
    }
    this->_data_array[0] = value;
    sum_data += this->_data_array[0];

    this->_delta_array[0] = elapsed;
    sum_time += this->_delta_array[0];

    this->average       = (double)sum_data / (double)this->_data_size;
    this->average_delta = sum_data / (sum_time / this->RESOLUTION_US);
    return this->average_delta;
}

template <int NUM>
void MoveAverage<NUM>::reset(int init_value)
{
    for (int i = 0; i < this->_max_size; i++) {
        this->_data_array[i]  = init_value;
        this->_delta_array[i] = 0;
    }
    this->_data_size    = 0;
    this->average       = init_value;
    this->average_delta = 0;
}

template <int NUM>
void MoveAverage<NUM>::reset()
{
    this->reset(this->_init_value);
}

template <int NUM>
String MoveAverage<NUM>::get_text()
{
    String buffer;
    char char_buf[200];
    sprintf(char_buf, "[SIZE: %d/%d] ", this->_data_size, this->_max_size);
    buffer += char_buf;
    for (int i = 0; i < this->_data_size; i++) {
        sprintf(char_buf, ((0 == i) ? "%d" : ", %d"), this->_data_array[i]);
        buffer += char_buf;
    }
    return buffer;
}

} // namespace common
} // namespace maid_robot_system

using namespace maid_robot_system::common;

#endif
