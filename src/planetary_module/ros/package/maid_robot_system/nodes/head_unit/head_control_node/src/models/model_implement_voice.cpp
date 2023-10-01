/**
 * @file model_implement_voice.cpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "models/model_implement.hpp"

namespace maid_robot_system
{
bool ModelImplement::set_value_voice(std::string text, int command, double seconds)
{
    // TODO : Verbal control is determined from the database
    bool result = false;
    switch (this->_mode) {
        case MRS_MODE::MRS_MODE_NONE:
            switch (command) {
                case 26:
                case 27:
                    this->_temp_overall.flag_eyelid_close = true;
                    break;
                case 28:
                    this->_temp_overall.flag_eyelid_wink = true;
                    this->_temp_overall.count_continue   = this->_temp_overall.COUNT_CONTINUE_MAX;
                    break;
                case 29:
                    this->_temp_overall.flag_eyelid_smile = true;
                    break;

                case 20:
                    this->_temp_overall.flag_eyelid_close = false;
                    this->_temp_overall.flag_eyelid_smile = false;
                    break;
                default:
                    break;
            }

            break;
        default:
            break;
    }
    return result;
}
} // namespace maid_robot_system
