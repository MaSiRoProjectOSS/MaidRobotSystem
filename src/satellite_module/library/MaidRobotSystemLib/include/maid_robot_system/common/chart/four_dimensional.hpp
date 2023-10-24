/**
 * @file four_dimensional.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.3
 * @date 2023-03-29
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef MAID_ROBOT_SYSTEM_LIB_FOUR_DIMENSIONAL_HPP
#define MAID_ROBOT_SYSTEM_LIB_FOUR_DIMENSIONAL_HPP

#include <vector>

namespace maid_robot_system
{
namespace common
{

class FourDimensionalChart {
public:
    class FourDimensionalChartData {
    public:
        struct FourDimensionalChartValue {
        public:
            unsigned long time_ms;
            float x;
            float y;
            float z;
            float w;
        };

    public:
        FourDimensionalChartData(float in_x = 0, float in_y = 0, float in_z = 0, float in_w = 0, unsigned long in_time_ms = 0);

    public:
        FourDimensionalChartValue data;
        bool is_saved;
        bool is_marked;

    public:
        void clear(float in_x, float in_y, float in_z, float in_w, unsigned long in_time_ms);
        void set(float in_x, float in_y, float in_z, float in_w);
        void set(float in_x, float in_y, float in_z, float in_w, unsigned long in_time_ms);

        bool save();
        bool mark();
    };

public:
    FourDimensionalChart();

    bool set(float in_x, float in_y, float in_z, float in_w);
    /**
     * @brief Set the value of the candle stick.
     *
     * @param value     The value of the candle stick.
     * @return true     If the time is greater than the time of the last candle stick.
     * @return false    If the time is less than the time of the last candle stick.
     */
    bool set(float in_x, float in_y, float in_z, float in_w, unsigned long in_time_ms);
    /**
     * @brief Get the list object
     *
     * @return std::vector<CandleStickData>*
     */
    std::vector<FourDimensionalChartData> *get_list();

    size_t size();

private:
    /**
     * @brief Remove the data that has already been saved to the database.
     */
    int _remove_list();

private:
    std::vector<FourDimensionalChartData> _list;

private:
    const int MAX_LIST_SIZE    = 5000;
    const int RETAIN_DATA_SIZE = 1;
};

} // namespace common
} // namespace maid_robot_system

using namespace maid_robot_system::common;

#endif
