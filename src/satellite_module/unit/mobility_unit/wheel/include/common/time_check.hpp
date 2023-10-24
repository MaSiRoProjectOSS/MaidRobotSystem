/**
 * @file time_check.hpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief Check elapsed time
 * @version 0.1
 * @date 2023-02-12
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef COMMON_TIME_CHECK_HPP
#define COMMON_TIME_CHECK_HPP

/**
 * @brief Check elapsed time
 */
class TimeCheck {
public:
    ///////////////////////////////////////////////////////////
    // Constructor
    ///////////////////////////////////////////////////////////
    /**
     * @brief Construct a new Time Check object
     */
    TimeCheck(unsigned long period_time = 1000);

public:
    ///////////////////////////////////////////////////////////
    // Control function
    ///////////////////////////////////////////////////////////
    /**
     * @brief Update measurement check position
     */
    void update();

    /**
     * @brief return to startup
     */
    void from_scratch();

    ///////////////////////////////////////////////////////////
    // Check function
    ///////////////////////////////////////////////////////////
    /**
     * @brief Check to see if the argument time has elapsed.
     * the current time is used as the start time if it is confirmed that the argument time has elapsed.
     *
     * @param checking_time : This value is the deadline.
     * @return true  : Elapsed since the last checkpoint.
     * @return false : Not yet.
     */
    bool check_passing(unsigned long checking_time);
    /**
     * @brief Check to see if the argument time has elapsed.
     * the current time is used as the start time if it is confirmed that the argument time has elapsed.
     * check time that set during configuration
     *
     * @return true  : Elapsed since the last checkpoint.
     * @return false : Not yet.
     */
    bool check_passing();

    /**
     * @brief Check to see if the argument time has elapsed.
     *
     * @param checking_time : This value is the deadline.
     * @return true  : Elapsed since the last checkpoint.
     * @return false : Not yet.
     */
    bool check_time_over(unsigned long checking_time);
    /**
     * @brief Check to see if the argument time has elapsed.
     * check time that set during configuration
     *
     * @return true  : Elapsed since the last checkpoint.
     * @return false : Not yet.
     */
    bool check_time_over();

    ///////////////////////////////////////////////////////////
    // Getter
    ///////////////////////////////////////////////////////////
    /**
     * @brief Check to see if the argument time has elapsed.
     * Set the current time as the checkpoint if it is confirmed that the argument time has elapsed.
     *
     * @param checking_time : This value is the deadline.
     * @return unsigned long : elapsed time
     */
    unsigned long get_elapsed_lap_time(unsigned long checking_time);
    /**
     * @brief Check to see if the argument time has elapsed.
     * Set the current time as the checkpoint if it is confirmed that the argument time has elapsed.
     *
     * @return unsigned long : elapsed time
     */
    unsigned long get_elapsed_lap_time();

    /**
     * @brief Get the elapsed time
     *
     * @return unsigned long : elapsed time
     */
    unsigned long get_elapsed_time();

    /**
     * @brief Get the elapsed time / Set the current time as the checkpoint.
     *
     * @return unsigned long : elapsed time
     */
    unsigned long get_lap_time();

    ///////////////////////////////////////////////////////////
    // Calculate function
    ///////////////////////////////////////////////////////////
    /**
     * @brief Calculate the cycloid curve.
     *
     * @param time_cycle : Specify the cycle
     * @return double : Calculated Results
     * Returns 1 when the specified time elapses.
     */
    double get_s_curve_flag(unsigned long time_cycle);

    /**
     * @brief Calculate the sin cycle
     *
     * @param time_cycle : Specify the cycle
     * @return double : Calculated Results
     */
    double get_sin_cycle(unsigned long time_cycle);

    /**
     * @brief Calculate the cos cycle
     *
     * @param time_cycle : Specify the cycle
     * @return double : Calculated Results
     */
    double get_cos_cycle(unsigned long time_cycle);

private:
    unsigned long _lap_start_time; /*!< Checkpoint start time */
    unsigned long _period_time;    /*!< Initial set time for one lap */
};

#endif
