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
    miens_normal = 0,  //! ノーマル
    miens_smile,       //! スマイル
    miens_wink_left,   //! 左目ウィンク（ウィンク後、ノーマルへ）
    miens_wink_right,  //! 右目ウィンク（ウィンク後、ノーマルへ）
    miens_close_left,  //! 左目だけ閉じる
    miens_close_right, //! 右目だけ閉じる
    miens_close,       //! 両目閉じる
} MIENS;

/**
 * @brief 瞳のエフェクト
 */
typedef enum pupil_variation_t
{
    pupil_variation_normal, //! 通常
    pupil_variation_order,  //! オーダー受付
    PUPIL_VARIATION_MAX,
} PUPIL_VARIATION;

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
