/**
 * @file model_structure.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-08-20
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MRS_NODE_MITUMERU_MODEL_STRUCTURE_HPP
#define MRS_NODE_MITUMERU_MODEL_STRUCTURE_HPP

namespace maid_robot_system
{
class ModelStructure {
public:
    enum INPUT_TYPE
    {
        VOICE,
        AR_LEFT,
        AR_RIGHT,
        POSE_LEFT,
        POSE_RIGHT,
    };
};

} // namespace maid_robot_system

#endif
