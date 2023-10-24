/**
 * @file line_chart.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.3
 * @date 2023-03-29
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef MAID_ROBOT_SYSTEM_LIB_LINE_CHART_HPP
#define MAID_ROBOT_SYSTEM_LIB_LINE_CHART_HPP

#include <vector>

namespace maid_robot_system
{
namespace common
{

class LineChart {
public:
    class LineChartData {
    public:
        struct LineChartValue {
        public:
            unsigned long time_ms;
            float value;
        };

    public:
        LineChartData(float in_value = 0, unsigned long in_time_ms = 0);

    public:
        LineChartValue data;
        bool is_saved;
        bool is_marked;

    public:
        void clear(float in_value, unsigned long in_time_ms);
        void set(float in_value);
        /**
         * @brief Set the value of the candle stick.
         *
         * @param value     The value of the candle stick.
         * @param time      The time of the candle stick.
         */
        void set(float in_value, unsigned long in_time_ms);

        bool save();
        bool mark();
    };

public:
    LineChart();

    bool set(float in_value);
    /**
     * @brief Set the value of the candle stick.
     *
     * @param value     The value of the candle stick.
     * @return true     If the time is greater than the time of the last candle stick.
     * @return false    If the time is less than the time of the last candle stick.
     */
    bool set(float in_value, unsigned long in_time_ms);
    /**
     * @brief Get the list object
     *
     * @return std::vector<CandleStickData>*
     */
    std::vector<LineChartData> *get_list();

    size_t size();

private:
    /**
     * @brief Remove the data that has already been saved to the database.
     */
    int _remove_list();

private:
    std::vector<LineChartData> _list;

private:
    const int MAX_LIST_SIZE    = 5000;
    const int RETAIN_DATA_SIZE = 1;
};

} // namespace common
} // namespace maid_robot_system

using namespace maid_robot_system::common;

#endif
