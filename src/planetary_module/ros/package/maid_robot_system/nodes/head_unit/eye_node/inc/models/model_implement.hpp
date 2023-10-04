/**
 * @file model_implement.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-07-15
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MRS_EYE_NODE_MODEL_IMPLEMENT_HPP
#define MRS_EYE_NODE_MODEL_IMPLEMENT_HPP

#include "data_structure.hpp"

#include <string>

namespace maid_robot_system
{
class ModelImplement {
public:
    // =============================
    // PUBLIC : Function
    // =============================
    bool calculate();
    void init(int argc, char **argv);
   void set_msg_eye (int emotions,
     int pupil_effect,
     float size,
     float distance,

     float left_x,
     float left_y,

     float right_x,
     float right_y);

   // =============================
   // PUBLIC : Getter
   // =============================
   std::string get_setting_file();
   int get_brightness();
   int get_color_r();
   int get_color_g();
   int get_color_b();

   // =============================
   // PUBLIC : Setter
   // =============================
   bool set_setting_file(std::string value);
   bool set_brightness(int value);
   bool set_color_r(int value);
   bool set_color_g(int value);
   bool set_color_b(int value);

   private:
   // =============================
   // PRIVATE : Function
   // =============================
   bool _set_param(std::string json_file);
   std::string _read_file(const std::string &path);

   private:
   // =============================
   // PRIVATE : parameter
   // =============================
   StParameter _param;

   // =============================
   // PRIVATE : Variable
   // =============================

   public:
   // =============================
   // Constructor
   // =============================
   ModelImplement();
   ~ModelImplement();
};

} // namespace maid_robot_system

#endif
