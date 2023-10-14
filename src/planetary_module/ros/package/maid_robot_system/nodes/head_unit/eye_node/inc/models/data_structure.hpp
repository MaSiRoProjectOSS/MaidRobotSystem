/**
 * @file data_structure.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-08-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MRS_EYE_NODE_DATA_STRUCTURE_HPP
#define MRS_EYE_NODE_DATA_STRUCTURE_HPP

#include "calibration.hpp"
#include "maid_robot_system/common_structure.hpp"

#include <QPixmap>
#include <Qt>
#include <string>
#include <vector>

namespace maid_robot_system
{
class StImageMap {
public:
    std::vector<QPixmap> data;
    int max;
    int id;
};

class StParameter {
public:
    class StImageInfo {
    public:
        int id           = 0;
        std::string name = "";
        std::vector<std::string> files;
        bool mirror = false;

        StImageInfo(int in_id, std::string in_name, bool in_mirror = false)
        {
            this->set(in_id, in_name, in_mirror);
        }
        void set(int in_id, std::string in_name, bool in_mirror = false)
        {
            this->id     = in_id;
            this->name   = in_name;
            this->mirror = in_mirror;
        }
    };
    class StImageList {
    public:
        std::vector<StImageInfo> eyeball;
        std::vector<StImageInfo> eyelid;
        std::vector<StImageInfo> cornea_outside;
        std::vector<StImageInfo> cornea_inside;
    };
    class StCornea {
    public:
        bool enable    = true;
        double speed   = 1.0;
        StVector scale = 1.0;
        int alpha      = 255;
        void set(bool in_enable, double in_speed, double in_scale_x, double in_scale_y, int in_alpha)
        {
            this->enable  = in_enable;
            this->speed   = in_speed;
            this->scale.x = in_scale_x;
            this->scale.y = in_scale_y;
            this->alpha   = in_alpha;
        }
    };

    class StEyeSettings {
    public:
        // cornea
        StCornea cornea_outside;
        StCornea cornea_inside;
        // eyelid
        St2DRectangle eyelid;
        StVector eyelid_scale;
        // eyeball
        St2DRectangle eyeball;
        StVector eyeball_scale;
        StVector eyeball_center;
        // image
        StImageList image;
    };
    StParameter()
    {
        // File information
        this->setting_file = "";
        this->path         = "--";
        this->name         = "--";
        // Settings : Image
        this->imageFlag = Qt::OrderedAlphaDither;
        // Settings : Screen
        this->brightness = 100;
        this->eyelid_color.set(255, 255, 255, 255);
        this->ciliary_color.set(231, 183, 147, 255);
        this->screen_size.set(0, 0, 640, 480, 0);
        this->view_size.set(0, 0, 640, 480, 0);
        this->screen_resolution = 1.0;
        // blink time
        this->blink_time_quickly = 150.0f;
        this->blink_time_min     = (500.0f - 100.0f);
        this->blink_time_max     = (500.0f + 100.0f);
        this->blink_time_limit   = 15000.0f;
        this->blink_time_offset  = 0.0f;

        // Settings : Parts : eyelid
        this->left_eye.eyelid.set(400, 80, 160, 160, 0);
        this->right_eye.eyelid.set(80, 80, 160, 160, 0);

        // Settings : Parts : eyeball
        this->left_eye.eyeball.set(0, 320, 320, 480, 0);

        this->right_eye.eyeball.set(0, 0, 320, 480, 0);
        this->left_eye.eyeball_center.set(0, 0, 0);
        this->right_eye.eyeball_center.set(0, 0, 0);

        // Settings : Parts : cornea
        this->left_eye.cornea_outside.set(true, 1.0, 1.0, 1.0, 255);
        this->left_eye.cornea_inside.set(true, 1.0, 0.8, 0.8, 255);

        this->right_eye.cornea_outside.set(true, 1.0, 1.0, 1.0, 255);
        this->right_eye.cornea_inside.set(true, 1.0, 0.8, 0.8, 255);

        // image
        this->left_eye.image.eyelid.clear();
        this->left_eye.image.eyeball.clear();
        this->left_eye.image.cornea_outside.clear();
        this->left_eye.image.cornea_inside.clear();
        this->right_eye.image.eyelid.clear();
        this->right_eye.image.eyeball.clear();
        this->right_eye.image.cornea_outside.clear();
        this->right_eye.image.cornea_inside.clear();
    }

public:
    // ----------------------------------- //
    // File information
    // ----------------------------------- //
    std::string setting_file;
    std::string path;
    std::string name;
    // ----------------------------------- //
    // Settings : Image
    // ----------------------------------- //
    Qt::ImageConversionFlag imageFlag;

    // ----------------------------------- //
    // Settings : Screen
    // ----------------------------------- //
    int brightness;
    St2DRectangle screen_size;
    St2DRectangle view_size;
    double screen_resolution;

    // ----------------------------------- //
    // Settings
    // ----------------------------------- //
    StColor eyelid_color;
    StColor ciliary_color;
    StEyeSettings left_eye;
    StEyeSettings right_eye;

    // ----------------------------------- //
    // blink time
    // ----------------------------------- //
    float blink_time_quickly = 150.0f;
    float blink_time_min     = (500.0f - 100.0f);
    float blink_time_max     = (500.0f + 100.0f);
    float blink_time_limit   = 15000.0f;
    float blink_time_offset  = 0.0f;
};

} // namespace maid_robot_system

#endif
