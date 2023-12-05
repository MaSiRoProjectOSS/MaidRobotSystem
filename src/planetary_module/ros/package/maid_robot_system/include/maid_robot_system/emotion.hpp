/**
 * @file emotion.hpp
 * @brief
 * @date 2021-11-08
 *
 * @copyright Copyright (c) MaSiRo Project. 2021-.
 *
 */
#ifndef MAID_ROBOT_SYSTEM_EMOTION_HPP
#define MAID_ROBOT_SYSTEM_EMOTION_HPP

namespace maid_robot_system
{
/**
 * @brief 表情一覧
 */
typedef enum miens_t
{
    miens_normal = 0,  //!  0 : normal
    miens_smile,       //!  1 : smile
    miens_wink_left,   //!  2 : wink(left) to normal
    miens_wink_right,  //!  3 : wink(right) to normal
    miens_close_left,  //!  4 : close left eyes
    miens_close_right, //!  5 : close right eyes
    miens_close,       //!  6 : close both eyes
    miens_keep_normal, //!  7 : don't blink
} MIENS;

/**
 * @brief 瞳のエフェクト
 */
typedef enum cornea_variation_t
{
    cornea_variation_normal, //! 通常
    cornea_variation_order,  //! オーダー受付
    CORNEA_VARIATION_MAX,
} CORNEA_VARIATION;

/**
 * @brief 状態
 */
typedef enum management_t
{
    management_wakeup = 0, //! 起床
    management_stay,       //! 待機
    management_knock,      //! 確認
    management_sleepy,     //! 就寝
    MANAGEMENT_MAX,
} MANAGEMENT;

} // namespace maid_robot_system

#endif
