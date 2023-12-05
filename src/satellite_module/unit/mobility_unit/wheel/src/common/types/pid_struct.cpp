/**
 * @file pid_struct.cpp
 * @author Akari (masiro.to.akari@gmail.com)
 * @brief A structure that manages PID
 * @version 0.1
 * @date 2023-02-12
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#include "common/types/pid_struct.hpp"

PIDstruct::PIDstruct(float proportional, float integral, float differential)
{
    this->set(proportional, integral, differential);
    this->previous_P = proportional;
}

void PIDstruct::set(float proportional, float integral, float differential)
{
    this->previous_P = this->P;

    this->P = proportional;
    this->I = integral;
    this->D = differential;
}
