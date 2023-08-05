/**
 * @file main.cpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "ros/node_implement.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<maid_robot_system::NodeImplement>( //
            MRS_PACKAGES_MAID_ROBOT_SYSTEM,
            MRS_NODE_CONDITION_MONITORING,
            argc,
            argv));
    rclcpp::shutdown();
    return 0;
}
