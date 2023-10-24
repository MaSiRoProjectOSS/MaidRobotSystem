/**
 * @file candle_stick.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief
 * @version 0.23.3
 * @date 2023-03-29
 *
 * @copyright Copyright (c) 2023 / MaSiRo Project.
 *
 */
#ifndef MAID_ROBOT_SYSTEM_LIB_CANDLE_STICK_HPP
#define MAID_ROBOT_SYSTEM_LIB_CANDLE_STICK_HPP

#include <vector>

namespace maid_robot_system
{
namespace common
{

class CandleStick {
public:
    class CandleStickData {
    public:
        struct CandleStickValue {
        public:
            float open;
            float close;
            float high;
            float low;
            unsigned int number_of_samples;
            float average;
            bool increase;

            unsigned long start_ms;
            unsigned long end_ms;
        };

    public:
        CandleStickData();

    public:
        CandleStickValue data;
        bool is_saved;
        bool is_marked;

    public:
        void clear();
        void set(float in_value);
        /**
         * @brief Set the value of the candle stick.
         *
         * @param value     The value of the candle stick.
         * @param time      The time of the candle stick.
         */
        void set(float in_value, unsigned long in_time_ms);
        /**
         * @brief Don't update data
         */
        void fin();
        bool save();
        bool mark();

    private:
        bool _initialized;
        bool _closed;
    };

public:
    CandleStick(unsigned long resolution_ms = (1000 * 60));

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
    std::vector<CandleStickData> *get_list();

    size_t size();

private:
    /**
     * @brief Remove the data that has already been saved to the database.
     */
    int _remove_list();

private:
    std::vector<CandleStickData> _list;
    unsigned long _time_next_ms;
    unsigned long _resolution_ms;

private:
    const int MAX_LIST_SIZE = 5000;
};

} // namespace common
} // namespace maid_robot_system

using namespace maid_robot_system::common;

#endif
