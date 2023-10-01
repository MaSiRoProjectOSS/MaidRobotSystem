/**
 * @file angle_calculus.hpp
 * @brief
 * @version 0.23.7
 * @date 2023-09-18
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef MAID_ROBOT_SYSTEM_INTERFACES_ANGLE_CALCULUS_HPP
#define MAID_ROBOT_SYSTEM_INTERFACES_ANGLE_CALCULUS_HPP

#include <array>
#include <cmath>

namespace maid_robot_system
{
class Vector3 {
public:
    float x;
    float y;
    float z;
    Vector3(float x_axis = 0, float y_axis = 0, float z_axis = 0)
    {
        this->x = x_axis;
        this->y = y_axis;
        this->z = z_axis;
    }
};
class Vector4 {
public:
    float x;
    float y;
    float z;
    float w;
    Vector4(float x_axis = 0, float y_axis = 0, float z_axis = 0, float w_axis = 0)
    {
        this->x = x_axis;
        this->y = y_axis;
        this->z = z_axis;
        this->w = w_axis;
    }
};

class RotationMatrix {
public:
    std::array<float, 9> elements;
    RotationMatrix(std::array<float, 9> elements) : elements(elements)
    {
    }
    RotationMatrix operator*(const RotationMatrix m) const
    {
        return RotationMatrix({ elements[0] * m.elements[0] + elements[3] * m.elements[1] + elements[6] * m.elements[2],
                                elements[1] * m.elements[0] + elements[4] * m.elements[1] + elements[7] * m.elements[2],
                                elements[2] * m.elements[0] + elements[5] * m.elements[1] + elements[8] * m.elements[2],
                                elements[0] * m.elements[3] + elements[3] * m.elements[4] + elements[6] * m.elements[5],
                                elements[1] * m.elements[3] + elements[4] * m.elements[4] + elements[7] * m.elements[5],
                                elements[2] * m.elements[3] + elements[5] * m.elements[4] + elements[8] * m.elements[5],
                                elements[0] * m.elements[6] + elements[3] * m.elements[7] + elements[6] * m.elements[8],
                                elements[1] * m.elements[6] + elements[4] * m.elements[7] + elements[7] * m.elements[8],
                                elements[2] * m.elements[6] + elements[5] * m.elements[7] + elements[8] * m.elements[8] });
    }
    Vector3 operator*(const Vector3 v) const
    {
        return Vector3(elements[0] * v.x + elements[3] * v.y + elements[6] * v.z,
                       elements[1] * v.x + elements[4] * v.y + elements[7] * v.z,
                       elements[2] * v.x + elements[5] * v.y + elements[8] * v.z);
    }
    float &operator[](const size_t index)
    {
        return elements[index];
    }
    float &at(const size_t row, const size_t column)
    {
        return elements[row + column * 3];
    }
};

enum class EulerOrder
{
    EULER_ORDER_XYZ,
    EULER_ORDER_XZY,
    EULER_ORDER_YXZ,
    EULER_ORDER_YZX,
    EULER_ORDER_ZXY,
    EULER_ORDER_ZYX
};

class EulerAngle {
public:
    float x;
    float y;
    float z;
    EulerOrder order;
    EulerAngle(float x_axis = 0, float y_axis = 0, float z_axis = 0, EulerOrder euler_order = EulerOrder::EULER_ORDER_XYZ)
    {
        this->x     = x_axis;
        this->y     = y_axis;
        this->z     = z_axis;
        this->order = euler_order;
    }
};
class Quaternion {
public:
    float x;
    float y;
    float z;
    float w;
    Quaternion(float x_axis = 0, float y_axis = 0, float z_axis = 0, float w_axis = 0)
    {
        this->x = x_axis;
        this->y = y_axis;
        this->z = z_axis;
        this->w = w_axis;
    }
    Quaternion operator*(const Quaternion q) const
    {
        return Quaternion(w * q.x - z * q.y + y * q.z + x * q.w,
                          z * q.x + w * q.y - x * q.z + y * q.w,
                          -y * q.x + x * q.y + w * q.z + z * q.w,
                          -x * q.x - y * q.y - z * q.z + w * q.w);
    }
    Vector3 rotate(const Vector3 v) const
    {
        auto vq = Quaternion(v.x, v.y, v.z, 0);
        auto cq = Quaternion(-this->x, -this->y, -this->z, this->w);
        auto mq = *this * vq * cq;
        return Vector3(mq.x, mq.y, mq.z);
    }
    Quaternion conjugate(const Quaternion q)
    {
        return Quaternion(-q.x, -q.y, -q.z, q.w);
    }
};

static RotationMatrix to_RotationMatrix(EulerAngle e)
{
    auto cx = std::cos(e.x);
    auto sx = std::sin(e.x);
    auto cy = std::cos(e.y);
    auto sy = std::sin(e.y);
    auto cz = std::cos(e.z);
    auto sz = std::sin(e.z);
    switch (e.order) {
        case EulerOrder::EULER_ORDER_XYZ:
            return RotationMatrix({ cy * cz, //
                                    sx * sy * cz + cx * sz,
                                    -cx * sy * cz + sx * sz,
                                    -cy * sz,
                                    -sx * sy * sz + cx * cz,
                                    cx * sy * sz + sx * cz,
                                    sy,
                                    -sx * cy,
                                    cx * cy });
        case EulerOrder::EULER_ORDER_XZY:
            return RotationMatrix({ cy * cz, //
                                    cx * cy * sz + sx * sy,
                                    sx * cy * sz - cx * sy,
                                    -sz,
                                    cx * cz,
                                    sx * cz,
                                    sy * cz,
                                    cx * sy * sz - sx * cy,
                                    sx * sy * sz + cx * cy });
        case EulerOrder::EULER_ORDER_YXZ:
            return RotationMatrix({ sx * sy * sz + cy * cz, //
                                    cx * sz,
                                    sx * cy * sz - sy * cz,
                                    sx * sy * cz - cy * sz,
                                    cx * cz,
                                    sx * cy * cz + sy * sz,
                                    cx * sy,
                                    -sx,
                                    cx * cy });
        case EulerOrder::EULER_ORDER_YZX:
            return RotationMatrix({ cy * cz,
                                    sz,
                                    -sy * cz, //
                                    -cx * cy * sz + sx * sy,
                                    cx * cz,
                                    cx * sy * sz + sx * cy,
                                    sx * cy * sz + cx * sy,
                                    -sx * cz,
                                    -sx * sy * sz + cx * cy });
        case EulerOrder::EULER_ORDER_ZXY:
            return RotationMatrix({ -sx * sy * sz + cy * cz, //
                                    sx * sy * cz + cy * sz,
                                    -cx * sy,
                                    -cx * sz,
                                    cx * cz,
                                    sx,
                                    sx * cy * sz + sy * cz,
                                    -sx * cy * cz + sy * sz,
                                    cx * cy });
        case EulerOrder::EULER_ORDER_ZYX:
            return RotationMatrix({ cy * cz,
                                    cy * sz,
                                    -sy, //
                                    sx * sy * cz - cx * sz,
                                    sx * sy * sz + cx * cz,
                                    sx * cy,
                                    cx * sy * cz + sx * sz,
                                    cx * sy * sz - sx * cz,
                                    cx * cy });
    }
    throw "conversion of euler angle to rotation matrix is failed.";
}
static EulerAngle to_EulerAngle(RotationMatrix m, EulerOrder order)
{
    if (order == EulerOrder::EULER_ORDER_XYZ) {
        auto sy       = m.at(0, 2);
        auto unlocked = std::abs(sy) < 0.99999f;
        return EulerAngle(unlocked ? std::atan2(-m.at(1, 2), m.at(2, 2)) : std::atan2(m.at(2, 1), m.at(1, 1)), //
                          std::asin(sy),
                          unlocked ? std::atan2(-m.at(0, 1), m.at(0, 0)) : 0, //
                          order);
    } else if (order == EulerOrder::EULER_ORDER_XZY) {
        auto sz       = -m.at(0, 1);
        auto unlocked = std::abs(sz) < 0.99999f;
        return EulerAngle(unlocked ? std::atan2(m.at(2, 1), m.at(1, 1)) : std::atan2(-m.at(1, 2), m.at(2, 2)), //
                          unlocked ? std::atan2(m.at(0, 2), m.at(0, 0)) : 0,
                          std::asin(sz),
                          order);
    } else if (order == EulerOrder::EULER_ORDER_YXZ) {
        auto sx       = -m.at(1, 2);
        auto unlocked = std::abs(sx) < 0.99999f;
        return EulerAngle(std::asin(sx),
                          unlocked ? std::atan2(m.at(0, 2), m.at(2, 2)) :
                                     std::atan2(-m.at(2, 0),                 //
                                                m.at(0, 0)),                 //
                          unlocked ? std::atan2(m.at(1, 0), m.at(1, 1)) : 0, //
                          order);
    } else if (order == EulerOrder::EULER_ORDER_YZX) {
        auto sz       = m.at(1, 0);
        auto unlocked = std::abs(sz) < 0.99999f;
        return EulerAngle(unlocked ? atan2(-m.at(1, 2), m.at(1, 1)) : 0,                             //
                          unlocked ? atan2(-m.at(2, 0), m.at(0, 0)) : atan2(m.at(0, 2), m.at(2, 2)), //
                          std::asin(sz),
                          order);
    } else if (order == EulerOrder::EULER_ORDER_ZXY) {
        auto sx       = m.at(2, 1);
        auto unlocked = std::abs(sx) < 0.99999f;
        return EulerAngle(std::asin(sx),
                          unlocked ? std::atan2(-m.at(2, 0), m.at(2, 2)) : 0, //
                          unlocked ? std::atan2(-m.at(0, 1),                  //
                                                m.at(1, 1)) :
                                     std::atan2(m.at(1, 0), //
                                                m.at(0, 0)),
                          order);
    } else if (order == EulerOrder::EULER_ORDER_ZYX) {
        auto sy       = -m.at(2, 0);
        auto unlocked = std::abs(sy) < 0.99999f;
        return EulerAngle(unlocked ? std::atan2(m.at(2, 1), m.at(2, 2)) : 0, //
                          std::asin(sy),
                          unlocked ? std::atan2(m.at(1, 0), m.at(0, 0)) : std::atan2(-m.at(0, 1), m.at(1, 1)), //
                          order);
    }
    throw "conversion of rotation matrix to euler angle is failed.";
}
static Quaternion to_Quaternion(EulerAngle e)
{
    auto cx = std::cos(0.5f * e.x);
    auto sx = std::sin(0.5f * e.x);
    auto cy = std::cos(0.5f * e.y);
    auto sy = std::sin(0.5f * e.y);
    auto cz = std::cos(0.5f * e.z);
    auto sz = std::sin(0.5f * e.z);
    switch (e.order) {
        case EulerOrder::EULER_ORDER_XYZ:
            return Quaternion(cx * sy * sz + sx * cy * cz, //
                              -sx * cy * sz + cx * sy * cz,
                              cx * cy * sz + sx * sy * cz,
                              -sx * sy * sz + cx * cy * cz);
        case EulerOrder::EULER_ORDER_XZY:
            return Quaternion(-cx * sy * sz + sx * cy * cz, //
                              cx * sy * cz - sx * cy * sz,
                              sx * sy * cz + cx * cy * sz,
                              sx * sy * sz + cx * cy * cz);
        case EulerOrder::EULER_ORDER_YXZ:
            return Quaternion(cx * sy * sz + sx * cy * cz, //
                              -sx * cy * sz + cx * sy * cz,
                              cx * cy * sz - sx * sy * cz,
                              sx * sy * sz + cx * cy * cz);
        case EulerOrder::EULER_ORDER_YZX:
            return Quaternion(sx * cy * cz + cx * sy * sz, //
                              sx * cy * sz + cx * sy * cz,
                              -sx * sy * cz + cx * cy * sz,
                              -sx * sy * sz + cx * cy * cz);
        case EulerOrder::EULER_ORDER_ZXY:
            return Quaternion(-cx * sy * sz + sx * cy * cz, //
                              cx * sy * cz + sx * cy * sz,
                              sx * sy * cz + cx * cy * sz,
                              -sx * sy * sz + cx * cy * cz);
        case EulerOrder::EULER_ORDER_ZYX:
            return Quaternion(sx * cy * cz - cx * sy * sz, //
                              sx * cy * sz + cx * sy * cz,
                              -sx * sy * cz + cx * cy * sz,
                              sx * sy * sz + cx * cy * cz);
    }
    throw "conversion of euler angle to quaternion is failed.";
}
static RotationMatrix to_RotationMatrix(Quaternion q)
{
    auto xy2 = q.x * q.y * 2;
    auto xz2 = q.x * q.z * 2;
    auto xw2 = q.x * q.w * 2;
    auto yz2 = q.y * q.z * 2;
    auto yw2 = q.y * q.w * 2;
    auto zw2 = q.z * q.w * 2;
    auto ww2 = q.w * q.w * 2;
    return RotationMatrix({ ww2 + 2 * q.x * q.x - 1,
                            xy2 + zw2, //
                            xz2 - yw2,
                            xy2 - zw2,
                            ww2 + 2 * q.y * q.y - 1,
                            yz2 + xw2,
                            xz2 + yw2,
                            yz2 - xw2,
                            ww2 + 2 * q.z * q.z - 1 });
}
static Quaternion to_Quaternion(RotationMatrix m)
{
    auto px = m.at(0, 0) - m.at(1, 1) - m.at(2, 2) + 1;
    auto py = -m.at(0, 0) + m.at(1, 1) - m.at(2, 2) + 1;
    auto pz = -m.at(0, 0) - m.at(1, 1) + m.at(2, 2) + 1;
    auto pw = m.at(0, 0) + m.at(1, 1) + m.at(2, 2) + 1;

    auto selected = 0;
    auto max      = px;
    if (max < py) {
        selected = 1;
        max      = py;
    }
    if (max < pz) {
        selected = 2;
        max      = pz;
    }
    if (max < pw) {
        selected = 3;
        max      = pw;
    }

    if (selected == 0) {
        auto x = std::sqrt(px) * 0.5f;
        auto d = 1 / (4 * x);
        return Quaternion(x,
                          (m.at(1, 0) + m.at(0, 1)) * d, //
                          (m.at(0, 2) + m.at(2, 0)) * d,
                          (m.at(2, 1) - m.at(1, 2)) * d);
    } else if (selected == 1) {
        auto y = std::sqrt(py) * 0.5f;
        auto d = 1 / (4 * y);
        return Quaternion((m.at(1, 0) + m.at(0, 1)) * d,
                          y, //
                          (m.at(2, 1) + m.at(1, 2)) * d,
                          (m.at(0, 2) - m.at(2, 0)) * d);
    } else if (selected == 2) {
        auto z = std::sqrt(pz) * 0.5f;
        auto d = 1 / (4 * z);
        return Quaternion((m.at(0, 2) + m.at(2, 0)) * d, //
                          (m.at(2, 1) + m.at(1, 2)) * d,
                          z,
                          (m.at(1, 0) - m.at(0, 1)) * d);
    } else if (selected == 3) {
        auto w = std::sqrt(pw) * 0.5f;
        auto d = 1 / (4 * w);
        return Quaternion((m.at(2, 1) - m.at(1, 2)) * d, //
                          (m.at(0, 2) - m.at(2, 0)) * d,
                          (m.at(1, 0) - m.at(0, 1)) * d,
                          w);
    }
    throw "conversion of rotation matrix to quaternion is failed.";
}
static EulerAngle to_EulerAngle(Quaternion q, EulerOrder order)
{
    if (order == EulerOrder::EULER_ORDER_XYZ) {
        auto sy       = 2 * q.x * q.z + 2 * q.y * q.w;
        auto unlocked = std::abs(sy) < 0.99999f;
        return EulerAngle(unlocked ? std::atan2(-(2 * q.y * q.z - 2 * q.x * q.w), //
                                                2 * q.w * q.w + 2 * q.z * q.z - 1) :
                                     std::atan2(2 * q.y * q.z + 2 * q.x * q.w, //
                                                2 * q.w * q.w + 2 * q.y * q.y - 1),
                          std::asin(sy),
                          unlocked ? std::atan2(-(2 * q.x * q.y - 2 * q.z * q.w), //
                                                2 * q.w * q.w + 2 * q.x * q.x - 1) :
                                     0,
                          order);
    } else if (order == EulerOrder::EULER_ORDER_XZY) {
        auto sz       = -(2 * q.x * q.y - 2 * q.z * q.w);
        auto unlocked = std::abs(sz) < 0.99999f;
        return EulerAngle(unlocked ? std::atan2(2 * q.y * q.z + 2 * q.x * q.w, //
                                                2 * q.w * q.w + 2 * q.y * q.y - 1) :
                                     std::atan2(-(2 * q.y * q.z - 2 * q.x * q.w), //
                                                2 * q.w * q.w + 2 * q.z * q.z - 1),
                          unlocked ? std::atan2(2 * q.x * q.z + 2 * q.y * q.w, //
                                                2 * q.w * q.w + 2 * q.x * q.x - 1) :
                                     0,
                          std::asin(sz),
                          order);
    } else if (order == EulerOrder::EULER_ORDER_YXZ) {
        auto sx       = -(2 * q.y * q.z - 2 * q.x * q.w);
        auto unlocked = std::abs(sx) < 0.99999f;
        return EulerAngle(std::asin(sx),
                          unlocked ? std::atan2(2 * q.x * q.z + 2 * q.y * q.w, //
                                                2 * q.w * q.w + 2 * q.z * q.z - 1) :
                                     std::atan2(-(2 * q.x * q.z - 2 * q.y * q.w), //
                                                2 * q.w * q.w + 2 * q.x * q.x - 1),
                          unlocked ? std::atan2(2 * q.x * q.y + 2 * q.z * q.w, //
                                                2 * q.w * q.w + 2 * q.y * q.y - 1) :
                                     0,
                          order);
    } else if (order == EulerOrder::EULER_ORDER_YZX) {
        auto sz       = 2 * q.x * q.y + 2 * q.z * q.w;
        auto unlocked = std::abs(sz) < 0.99999f;
        return EulerAngle(unlocked ? atan2(-(2 * q.y * q.z - 2 * q.x * q.w), //
                                           2 * q.w * q.w + 2 * q.y * q.y - 1) :
                                     0,
                          unlocked ? atan2(-(2 * q.x * q.z - 2 * q.y * q.w), //
                                           2 * q.w * q.w + 2 * q.x * q.x - 1) :
                                     atan2(2 * q.x * q.z + 2 * q.y * q.w, //
                                           2 * q.w * q.w + 2 * q.z * q.z - 1),
                          std::asin(sz),
                          order);
    } else if (order == EulerOrder::EULER_ORDER_ZXY) {
        auto sx       = 2 * q.y * q.z + 2 * q.x * q.w;
        auto unlocked = std::abs(sx) < 0.99999f;
        return EulerAngle(std::asin(sx),
                          unlocked ? std::atan2(-(2 * q.x * q.z - 2 * q.y * q.w), //
                                                2 * q.w * q.w + 2 * q.z * q.z - 1) :
                                     0,
                          unlocked ? std::atan2(-(2 * q.x * q.y - 2 * q.z * q.w), //
                                                2 * q.w * q.w + 2 * q.y * q.y - 1) :
                                     std::atan2(2 * q.x * q.y + 2 * q.z * q.w, //
                                                2 * q.w * q.w + 2 * q.x * q.x - 1),
                          order);
    } else if (order == EulerOrder::EULER_ORDER_ZYX) {
        auto sy       = -(2 * q.x * q.z - 2 * q.y * q.w);
        auto unlocked = std::abs(sy) < 0.99999f;
        return EulerAngle(unlocked ? std::atan2(2 * q.y * q.z + 2 * q.x * q.w, //
                                                2 * q.w * q.w + 2 * q.z * q.z - 1) :
                                     0,
                          std::asin(sy),
                          unlocked ? std::atan2(2 * q.x * q.y + 2 * q.z * q.w, //
                                                2 * q.w * q.w + 2 * q.x * q.x - 1) :
                                     std::atan2(-(2 * q.x * q.y - 2 * q.z * q.w), 2 * q.w * q.w + 2 * q.y * q.y - 1),
                          order);
    }
    throw "conversion of quaternion to euler angle is failed.";
}

static float deg_to_rad(float value)
{
    // rad = deg * (PI / 180)
    return value * (M_PI / 180);
}
static float rad_to_deg(float value)
{
    //deg = rad ∗ (180 / PI)
    return value * (180 / M_PI);
}

} // namespace maid_robot_system

#endif // MAID_ROBOT_SYSTEM_INTERFACES_MRS_MODE_HPP
