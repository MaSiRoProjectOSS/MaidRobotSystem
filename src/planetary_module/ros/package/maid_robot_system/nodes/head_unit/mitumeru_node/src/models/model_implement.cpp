/**
 * @file modeleft.implement.cpp
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
bool ModelImplement::calculate(maid_robot_system_interfaces::msg::MrsHitomi &msg_hitomi,
                               maid_robot_system_interfaces::msg::MrsKubi &msg_kubi,
                               maid_robot_system_interfaces::msg::MrsKuchibiru &msg_kuchibiru)
{
    bool result = false;
    if (human_find_flag == 0 or (human_find_flag == 1 and human_confront_flag == 0)) {
        if (time.time() - timer_tracking_timeout > TIME_timer_tracking_timeout) {
            timer_tracking_timeout = time.time() - 0.5;
            send_eye_target        = Pose2D(x = 0, y = 0);
            neck_cmd_pose.linear.x = 0;
            To_face_yaw            = 0;
        }
    }
    // TODO
    /*
一定時間経過後も人が見つからない場合は、首を中央に戻す
*/

    /*
    送信メッセージに格納するように変更する。
    left/rightでの判定を分ける。


get_eye_cmd_angle.linear.x; // 感情
get_eye_cmd_angle.linear.y;  // 瞳のサイズ
get_eye_cmd_angle.linear.z;  // 距離
get_eye_cmd_angle.angular.y; // y 座標
get_eye_cmd_angle.angular.z; // x 座標

                if (cap_mode == "left"):
                    offset_z_angle = -40.0
                    offset_y_angle = 0.0
                    target_angle_z_R = to_face_yaw + offset_z_angle
                    target_angle_y_R = to_face_pitch + offset_y_angle
                if (cap_mode == "right"):
                    offset_z_angle = -12.0
                    offset_y_angle = 0.0
                    target_angle_z_L = to_face_yaw + offset_z_angle
                    target_angle_y_L = to_face_pitch + offset_y_angle

                target_angle_z = (target_angle_z_L + target_angle_z_R) / 2.0
                target_angle_y = (target_angle_y_L + target_angle_y_R) / 2.0

                eye_cmd_angle.angular.y = target_angle_y / -2.0
                eye_cmd_angle.angular.z = target_angle_z / 1.5
                # eye_size
                eye_cmd_angle.linear.y = 0.98
                # eye_cmd_angle.linear.z = 500 - 500 * (abs(target_angle_z_L - target_angle_z_R) /10.0)

                abs(target_angle_z_L - target_angle_z_R)/10.0

                if (human_confront_flag == 1):
                    neck_cmd_pose.linear.x = 111
                    neck_cmd_pose.angular.y = target_angle_y / 2.5
                    neck_cmd_pose.angular.z = target_angle_z / -2.5
                    neck_cmd_pose.angular.x = target_roll / 4.0

                    # print(neck_cmd_pose.angular.z)
                    pub_neck_cmd_angle.publish(neck_cmd_pose)

                debug_image = draw_bounding_rect(use_b_rect, debug_image, b_rect)


            if (human_find_flag == 0 or (human_find_flag == 1 and human_confront_flag == 0)):

                if (time.time() - timer_tracking_timeout > TIME_timer_tracking_timeout):
                    timer_tracking_timeout = time.time() - 0.5
                    send_eye_target = Pose2D(x=0, y=0)
                    neck_cmd_pose.linear.x = 0
                    pub_eye_target.publish(send_eye_target)
                    to_face_yaw = 0
                    pub_neck_cmd_angle.publish(neck_cmd_pose)

    */
    target_angle_z          = (target_angle_z_L + target_angle_z_R) / 2.0;
    target_angle_y          = (target_angle_y_L + target_angle_y_R) / 2.0;
    eye_cmd_angle.angular.y = target_angle_y / -2.0;
    eye_cmd_angle.angular.z = target_angle_z / 1.5;
    //# eye_size
    eye_cmd_angle.linear.y = 0.98;
    if (human_confront_flag == 1) {
        neck_cmd_pose.linear.x  = 111;
        neck_cmd_pose.angular.y = target_angle_y / 2.5;
        neck_cmd_pose.angular.z = target_angle_z / -2.5;
        neck_cmd_pose.angular.x = target_roll / 4.0;
    }
    if (true == result) {
        this->_get_value_hitomi(&msg_hitomi);
        this->_get_value_kubi(&msg_kubi);
        this->_get_value_kuchibiru(&msg_kuchibiru);
    }
    return result;
}
// =============================
// PUBLIC : Function
// =============================
bool ModelImplement::set_value_voice(std::string text, int command)
{
    bool result = false;
    switch (this->_mode) {
        case MRS_MODE::MRS_MODE_NONE:
            break;
        default:
            break;
    }
    return result;
}
bool ModelImplement::set_value_ar(ModelStructure::INPUT_TYPE type, int id)
{
    bool result = false;
    switch (type) {
        case ModelStructure::INPUT_TYPE::Aright.LEFT:
            result = true;
            break;
        case ModelStructure::INPUT_TYPE::Aright.RIGHT:
            result = true;
            break;
        default:
            break;
    }
    return result;
}
bool ModelImplement::set_value_pose(ModelStructure::INPUT_TYPE type, const maid_robot_system_interfaces::msg::PoseDetection)
{
    bool result = false;
    switch (type) {
        case ModelStructure::INPUT_TYPE::POSE_LEFT:
            result = this->_calculate_pose(msg, this->_eye_left));
            break;
        case ModelStructure::INPUT_TYPE::POSE_RIGHT:
            result = this->_calculate_pose(msg, this->_eye_right));
            break;
        default:
            break;
    }
    return result;
}

void ModelImplement::_get_value_hitomi(maid_robot_system_interfaces::msg::MrsHitomi &msg)
{
    msg.emotions = this->_msg_hitomi.emotions;
    msg.size     = this->_msg_hitomi.size;
    msg.distance = this->_msg_hitomi.distance;
    msg.x        = this->_msg_hitomi.x;
    msg.y        = this->_msg_hitomi.y;
}
void ModelImplement::_get_value_kubi(maid_robot_system_interfaces::msg::MrsKubi &msg)
{
    msg.x = this->_msg_kubi.x;
    msg.y = this->_msg_kubi.y;
    msg.z = this->_msg_kubi.z;
    msg.w = this->_msg_kubi.w;
}
void ModelImplement::_get_value_kuchibiru(maid_robot_system_interfaces::msg::MrsKuchibiru &msg)
{
    msg.x = this->_msg_kuchibiru.x;
    msg.y = this->_msg_kuchibiru.y;
    msg.z = this->_msg_kuchibiru.z;
    msg.w = this->_msg_kuchibiru.w;
}
bool ModelImplement::_calculate_pose(const maid_robot_system_interfaces::msg::PoseDetection msg, st_eye &data)
{
    // ターゲットの位置を決定する。
    bool result              = false;
    data.detected            = msg.human_detected;
    double target_roll       = 0;
    double x                 = data.landmark.nose.x;
    double y                 = data.landmark.nose.y;
    double z                 = data.landmark.nose.z;
    bool human_confront_flag = false;
    if (true == data.detected) {
        result = true;

        if (data.landmark.nose.y < data.landmark.left.shoulder.y and data.landmark.nose.y < data.landmark.right.shoulder.y) {
            if (data.landmark.left.index.exist == 1 and data.landmark.left.shoulder.exist == 1) {
                if (data.landmark.left.index.y < data.landmark.left.shoulder.y) {
                    x = data.landmark.left.index.x;
                    y = data.landmark.left.index.y;
                    z = data.landmark.left.index.z;
                }
            }
            if (data.landmark.right.index.exist == 1 and data.landmark.right.shoulder.exist == 1) {
                if (data.landmark.right.index.y < data.landmark.right.shoulder.y) {
                    x = data.landmark.right.index.x;
                    y = data.landmark.right.index.y;
                    z = data.landmark.right.index.z;
                }
            }

            // 人が正対するとき 右目ｘ座標は左目ｘ座標より大きい 人が後ろ向きでも目の位置は推定される
            if (data.landmark.right.eye.exist == 1 and data.landmark.left.eye.exist == 1) {
                if (data.landmark.right.eye.x > data.landmark.left.eye.x) {
                    human_confront_flag = true;
                    // ロール角度を計算
                    double eye_dx = data.landmark.right.eye.x - data.landmark.left.eye.x;
                    double eye_dy = data.landmark.right.eye.y - data.landmark.left.eye.y;
                    target_roll   = (math.degrees(math.atan2(eye_dy, eye_dx)));
                }
            }
        }
        data.x           = (x - 0.5) + data.offset_x_angle;
        data.y           = (y - 0.5) + data.offset_y_angle;
        data.target_roll = target_roll:
    }

    return result;
}

// =============================
// Constructor
// =============================
ModelImplement::ModelImplement()
{
    this->_eye_left.offset_z_angle  = -40.0;
    this->_eye_left.offset_y_angle  = 0.0;
    this->_eye_right.offset_z_angle = -12.0;
    this->_eye_right.offset_y_angle = 0.0;
}

ModelImplement ::~ModelImplement()
{
}

} // namespace maid_robot_system
