/**
 * @file eye_widget_paint.cpp
 * @brief
 * @date 2020-03-28
 *
 * @copyright Copyright (c) MaSiRo Project. 2020-.
 *
 */
#include "models/widgets/eye_widget.hpp"

namespace maid_robot_system
{
/**
 * @brief
 *
 */
void EyeWidget::UpdateScreen()
{
    static bool drawing = false; /* 読みだし元は一つだし、一定周期なのでmutex使うまでもなく*/

    if (false == drawing) {
        mtx_.lock(); /* ロックを取得する */
        drawing = true;

        //********************************************************************************//
        if ((0 != this->eyeball.right.target.x)    //
            || (0 != this->eyeball.right.target.y) //
            || (0 != this->eyeball.left.target.x)  //
            || (0 != this->eyeball.left.target.y)  //
            || (false == flag_first_request)) {
            int elapsed_times = (current_time.elapsed() - last_ros_msg_time);

            if ((LOST_ROS_MSG_TIMEOUT_SECONDS * 1000) < elapsed_times) {
                if (false == flag_first_request) {
                    this->eyelid.set_emotion(NEXT_EMOTION_INIT);
                    flag_first_request = true;
                }

                this->eyeball.set_default();
                last_ros_msg_time = current_time.elapsed();
            }
            if (0 > elapsed_times) {
                last_ros_msg_time = current_time.elapsed();
            }
        }

        this->make_image();
        mtx_.unlock(); /* ロックを手放す */
        update();
        //********************************************************************************//
        drawing = false;
    }
    if (true == flag_voiceId) {
        int current      = current_time.elapsed();
        int voiceId_time = (current - last_voiceId_time);
        if ((CTRL_HITOMI_CMD_VOICE_CLEAR * 1000) < voiceId_time) {
            flag_voiceId = false;
            // サークルの色解除
            this->eyeball.set_state_pupil(PartsEyeball::PupilState::Normal);
        }
        if (0 > voiceId_time) {
            voiceId_time = current;
        }
    }
    this->log();
    logger.set_index(logger.ST_INDEX_TOTAL, 1);
}

///============================== main loop =======================================================================================

/**
 * @brief
 *
 */
void EyeWidget::make_image()
{
    //********************************************************************************//
    int current = current_time.elapsed();
    int buf     = current;
    //********************************************************************************//
    /* ============================================= */
    // Pre-calculation
    /* ============================================= */
    this->eyelid.calc_animation(current);
    this->eyeball.calc_draw_pixel(current, this->eyelid.send_animation);
    //********************************************************************************//
#if DRAW_PUPIL_OUTSIDE
    buf = current_time.elapsed();
    this->eyeball.draw_outside();
    logger.set_index(logger.ST_INDEX_MAKE_PUPIL_OUTSIDE, current_time.elapsed() - buf);
#endif
    //********************************************************************************//
#if DRAW_PUPIL_INSIDE
    buf = current_time.elapsed();
    this->eyeball.draw_inside();
    logger.set_index(logger.ST_INDEX_MAKE_PUPIL_INSIDE, current_time.elapsed() - buf);
#endif
}

/**
 * @brief
 *
 * @param event
 */
void EyeWidget::paintEvent(QPaintEvent *event)
{
    //********************************************************************************//
    int current = current_time.elapsed();
    int buf     = current;
    /* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
    //********************************************************************************//
    /* ============================================= */
    // Drawing
    /* ============================================= */
    QPainter painter(this);

    if (true != painter.isActive()) {
        painter.begin(this);
    }

    painter.setRenderHint(QPainter::SmoothPixmapTransform);
#if DEBUG_VIEW
    //painter.setViewport(0,0,this->window_size_x * 0.5, this->window_size_y * 0.5);
    painter.scale(1920 / WINDOW_SIZE_X, 1920 / WINDOW_SIZE_X);
#endif
    //********************************************************************************//
    buf = current_time.elapsed();
    /* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
    //********************************************************************************//
    //eye draw
    painter.drawPixmap(this->eyeball.right.draw_postion.x,
                       this->eyeball.right.draw_postion.y,
                       this->eyeball.right.draw_postion.width,
                       this->eyeball.right.draw_postion.height,
                       this->eyeball.eyeball_origin_r);
    painter.drawPixmap(this->eyeball.left.draw_postion.x,
                       this->eyeball.left.draw_postion.y,
                       this->eyeball.left.draw_postion.width,
                       this->eyeball.left.draw_postion.height,
                       this->eyeball.eyeball_origin_l);
    //********************************************************************************//
    logger.set_index(logger.ST_INDEX_DRAW_EYEBALL, current_time.elapsed() - buf);
    buf = current_time.elapsed();
    /* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
#if DRAW_PUPIL_OUTSIDE
    //********************************************************************************//
    //pupil outside
    painter.drawPixmap(this->eyeball.right.draw_pupil_anime.x, this->eyeball.right.draw_pupil_anime.y, this->eyeball.pupil_outside);
    painter.drawPixmap(this->eyeball.left.draw_pupil_anime.x, this->eyeball.left.draw_pupil_anime.y, this->eyeball.pupil_outside);
    //********************************************************************************//
    logger.set_index(logger.ST_INDEX_DRAW_PUPIL_OUTSIDE, current_time.elapsed() - buf);
    buf = current_time.elapsed();
    /* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
#endif
#if DRAW_PUPIL_INSIDE
    //********************************************************************************//
    // pupil inside
    painter.drawPixmap(this->eyeball.right.draw_pupil_anime2.x, this->eyeball.right.draw_pupil_anime2.y, this->eyeball.pupil_inside);
    painter.drawPixmap(this->eyeball.left.draw_pupil_anime2.x, this->eyeball.left.draw_pupil_anime2.y, this->eyeball.pupil_inside);
    //********************************************************************************//
    logger.set_index(logger.ST_INDEX_DRAW_PUPIL_INSIDE, current_time.elapsed() - buf);
    buf = current_time.elapsed();
    /* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
#endif
    //********************************************************************************//
    // eyelid
    painter.drawPixmap(this->eyelid.right.pos.x, this->eyelid.right.pos.y, this->eyelid.get_eye_id(TARGET_EYE_RIGHT));
    painter.drawPixmap(this->eyelid.left.pos.x, this->eyelid.left.pos.y, this->eyelid.get_eye_id(TARGET_EYE_LEFT));
#if DEBUG_OUTPUT_WIDGET
    static int buf_a   = -1;
    static int buf_cnt = 0;

    if (buf_a != eyelid.send_animation) {
        if (0 == eyelid.send_animation) {
            printf("[%9.3f] ms - animation:%3d   [%d]\n", eyelid.eye_blink_time, eyelid.send_animation, buf_cnt);
            buf_cnt = 0;
        }

        buf_cnt++;
        buf_a = eyelid.send_animation;
    }

#endif
    //********************************************************************************//
    logger.set_index(logger.ST_INDEX_DRAW_EYELID, current_time.elapsed() - buf);
    /* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */
    //********************************************************************************//
    //painter.endNativePainting();
    painter.end();
    logger.set_index(logger.ST_INDEX_FPS, current_time.elapsed() - current);
    //********************************************************************************//
}

} // namespace maid_robot_system
