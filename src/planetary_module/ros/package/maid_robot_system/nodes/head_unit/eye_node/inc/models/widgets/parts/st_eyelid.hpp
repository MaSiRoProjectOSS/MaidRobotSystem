/**
 * @file st_eyelid.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-10-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "models/data_structure.hpp"

#include <vector>

namespace maid_robot_system
{
class StEyelid {
public:
    // =============================
    // Constructor
    // =============================
    StEyelid();
    ~StEyelid();

public:
    // =============================
    // PUBLIC : Variable
    // =============================
    std::vector<StImageMap> store;
    StRectangle rect;
    StVector pos_center;
    bool exit_eyelid = false;
    QColor ciliary_color;

public:
    // =============================
    // PUBLIC : Function
    // =============================
    void open_eye(uint wink_time_millisecond);
    void wink(uint wink_time_millisecond);
    bool enable_motion();

public:
    // =============================
    // PUBLIC : Setter
    // =============================
    void set_elapsed(uint current_time);

public:
    // =============================
    // PUBLIC : Getter
    // =============================
    uint get_elapsed();

private:
    // =============================
    // PRIVATE : Function
    // =============================
    void _fin_wink();

private:
    // =============================
    // PRIVATE : Variable
    // =============================
    bool _flag_next_blink       = false;
    uint _eye_blink_time        = 0;
    uint _elapsedIndex          = 0;
    uint _start_time            = 0xFFFFFFFF;
    uint _wink_time_millisecond = 0;
    bool _wink                  = false;
    uint _current_time          = 0;
};

} // namespace maid_robot_system
