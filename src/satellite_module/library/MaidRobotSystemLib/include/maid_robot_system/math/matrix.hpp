/**
 * @file matrix.hpp
 * @author Claude (claude.masiro@gmail.com)
 * @brief
 * @version 0.23.4
 * @date 2023-02-20
 *
 * @ref Appendix: http://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/manipulator.html
 *
 * @copyright Copyright (c) MaSiRo Project. 2023-.
 *
 */
#ifndef ARM_CONTROLLER_MATRIX_HPP
#define ARM_CONTROLLER_MATRIX_HPP

#include <math.h>

/*********************************************************
 * Math Definition
 *********************************************************/
#define DEG_TO_RAD_FACTOR (3.141593f / 180.0f)

/**
 * @brief Class for performing matrix calculations
 *
 * @param MX_SIZE : Number of matrix row and column
 */
template <int MX_SIZE = 4>
class Matrix {
public:
    /**
     * @brief store matrix elements
     *
     */
    float data[MX_SIZE][MX_SIZE];

    /**
     * @brief Define matrix addition with operator "+"
     *
     * @return Matrix
     */
    Matrix operator+(Matrix r);

    /**
     * @brief Define matrix substitution with operator "="
     *
     * @param r : Matrix
     * @return Matrix
     */
    Matrix operator=(Matrix r);

    /**
     * @brief Define matrix substitution with operator "="
     *
     * @param in : two-dimensional array
     * @return Matrix
     */
    Matrix operator=(float in[MX_SIZE][MX_SIZE]);

    /**
     * @brief create Homogeneous transformation matrix from i-1 coordinate from i coordinate
     *
     * @param a : z axis distance between i-1 and i.
     * @param m_alfa : z axis difference between i-1 and i.
     * @param d : x axis distance between i-1 and i.
     * @param m_sita : x axis difference between i-1 and i.
     * @return Matrix
     */
    Matrix trans(float a, float m_alfa, float d, float m_sita);

    /**
     * @brief calculate Matrix product
     *
     * @param x : Matrix
     * @param y : Matrix
     * @return Matrix
     */
    void Matrix_product(Matrix x, Matrix y);

    /**
     * @brief create Homogeneous transformation matrix from i-1 coordinate from i coordinate
     *
     * @param a : z axis distance between i-1 and i.
     * @param m_alfa : z axis difference between i-1 and i.
     * @param d : x axis distance between i-1 and i.
     * @param m_sita : x axis difference between i-1 and i.
     * @return Matrix
     */
    void trans_matrix(float a, float m_alfa, float d, float m_sita);

private:
    /**
     * @brief set matrix elements with Matrix
     *
     * @param r : Matrix
     */
    void _set(Matrix r);

    /**
     * @brief set matrix elements with two-dimensional array
     *
     * @param in : two-dimensional array
     */
    void _set(float in[MX_SIZE][MX_SIZE]);
};

template <int MX_SIZE>
Matrix<MX_SIZE> Matrix<MX_SIZE>::operator=(Matrix r)
{
    this->_set(r);
    return *this;
}

template <int MX_SIZE>
Matrix<MX_SIZE> Matrix<MX_SIZE>::operator=(float in[MX_SIZE][MX_SIZE])
{
    this->_set(in);
    return *this;
}

template <int MX_SIZE>
Matrix<MX_SIZE> Matrix<MX_SIZE>::operator+(Matrix r)
{
    Matrix<MX_SIZE> buffer;
    for (int i = 0; i < MX_SIZE; i++) {
        for (int j = 0; j < MX_SIZE; j++) {
            float term = 0;
            for (int k = 0; k < MX_SIZE; k++) {
                term = term + this->data[i][k] * r.data[k][j];
            }
            buffer.data[i][j] = term;
        }
    }
    this->_set(buffer);
    return *this;
}

template <int MX_SIZE>
void Matrix<MX_SIZE>::_set(Matrix r)
{
    for (int i = 0; i < MX_SIZE; i++) {
        for (int j = 0; j < MX_SIZE; j++) {
            this->data[i][j] = r.data[i][j];
        }
    }
}

template <int MX_SIZE>
void Matrix<MX_SIZE>::_set(float in[MX_SIZE][MX_SIZE])
{
    for (int i = 0; i < MX_SIZE; i++) {
        for (int j = 0; j < MX_SIZE; j++) {
            this->data[i][j] = in[i][j];
        }
    }
}

template <int MX_SIZE>
Matrix<MX_SIZE> Matrix<MX_SIZE>::trans(float a, float m_alfa, float d, float m_sita)
{
    float alfa = m_alfa * DEG_TO_RAD_FACTOR;
    float sita = m_sita * DEG_TO_RAD_FACTOR;

    float T_M[MX_SIZE][MX_SIZE] //
            = { { cos(sita), -sin(sita), 0, a },
                { cos(alfa) * sin(sita), cos(alfa) * cos(sita), -sin(alfa), -d * sin(alfa) },
                { sin(alfa) * sin(sita), sin(alfa) * cos(sita), cos(alfa), d * cos(alfa) },
                { 0, 0, 0, 1 } };

    this->_set(T_M);
    return *this;
}

template <int MX_SIZE>
void Matrix<MX_SIZE>::Matrix_product(Matrix<MX_SIZE> x, Matrix<MX_SIZE> y)
{
    for (int i = 0; i < MX_SIZE; i++) {
        for (int j = 0; j < MX_SIZE; j++) {
            float term = 0;
            for (int k = 0; k < MX_SIZE; k++) {
                term = term + x.data[i][k] * y.data[k][j];
            }
            this->data[i][j] = term;
        }
    }
}

template <int MX_SIZE>
void Matrix<MX_SIZE>::trans_matrix(float a, float m_alfa, float d, float m_sita)
{
    float alfa = m_alfa * DEG_TO_RAD_FACTOR;
    float sita = m_sita * DEG_TO_RAD_FACTOR;

    float T_M[MX_SIZE][MX_SIZE] /* Homogeneous transformation matrix */
            = { { cos(sita), -sin(sita), 0, a },
                { cos(alfa) * sin(sita), cos(alfa) * cos(sita), -sin(alfa), -d * sin(alfa) },
                { sin(alfa) * sin(sita), sin(alfa) * cos(sita), cos(alfa), d * cos(alfa) },
                { 0, 0, 0, 1 } };

    /* return the created Homogeneous transformation matrix */
    _set(T_M);
}

/**
* @brief Class for performing vector calculations
*
*/
template <int MX_SIZE = 4>
class Vector {
public:
    float data[MX_SIZE];

    /**
     * @brief set vector elements.
     *
     * @param in : one-dimensional array
     */
    void set_elements(float in[MX_SIZE]);
};

template <int MX_SIZE>
void Vector<MX_SIZE>::set_elements(float in[MX_SIZE])
{
    for (int i = 0; i < MX_SIZE; i++) {
        this->data[i] = in[i];
    }
}

/**
* @brief Class for performing matrix and vector calculations
*
*/
template <int MX_SIZE = 4>
class Tensor {
public:
    /**
     * @brief calculate product of Matrix and Vector
     *
     * @param v : Vector
     * @param m : Matrix
     * @return Vector
     */
    Vector<MX_SIZE> Mat_vec_product(Vector<MX_SIZE> v, Matrix<MX_SIZE> m);
};

template <int MX_SIZE>
Vector<MX_SIZE> Tensor<MX_SIZE>::Mat_vec_product(Vector<MX_SIZE> v, Matrix<MX_SIZE> m)
{
    Vector<MX_SIZE> c;
    for (int i = 0; i < MX_SIZE; i++) {
        float term = 0;
        for (int k = 0; k < MX_SIZE; k++) {
            term = term + m.data[i][k] * v.data[k];
        }
        c.data[i] = term;
    }
    return c;
}

#endif
