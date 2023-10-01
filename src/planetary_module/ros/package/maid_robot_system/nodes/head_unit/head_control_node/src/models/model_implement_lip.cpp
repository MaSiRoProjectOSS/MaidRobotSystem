/**
 * @file model_implement_lip.cpp
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
void ModelImplement::get_msg_lip(maid_robot_system_interfaces::msg::MrsLip &msg)
{
    // TODO
    bool result        = false;
    static int percent = 0;
    if (percent != this->_msg_lip.percent) {
        percent = std::min(std::max(this->_msg_lip.percent, this->param.lip_min), this->param.lip_max);
    }
    msg.percent = percent;
}
} // namespace maid_robot_system
