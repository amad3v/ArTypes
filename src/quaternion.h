/**
 * @file quaternion.h
 * @date 04.01.22
 * @author amad3v (amad3v@gmail.com)
 * @version 0.0.1
 *
 * @brief Quaternion type.
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __LIB_CUSTOM_TYPE_QUATERNION_H__
#define __LIB_CUSTOM_TYPE_QUATERNION_H__

#include <cmath>
#include "def.h"
#include "vector.h"
#include "matrix.h"

/**
 * @class Quaternion
 * @brief Creates a quaternion representation.
 */
class Quaternion {
  public:
    /**
     * @brief Scalar part.
     */
    float w;
    /**
     * @brief @f$i@f$ of the vector quaternion.
     */
    float x;
    /**
     * @brief @f$j@f$ of the vector quaternion.
     */
    float y;
    /**
     * @brief @f$k@f$ of the vector quaternion.
     */
    float z;

    /**
     * @brief Construct a new Quaternion object.
     * If no arguments provided, it creates a unity quaternion.
     *
     * @param a Scalar part.
     * @param b @f$i@f$ of the vector part.
     * @param c @f$j@f$ of the vector part.
     * @param d @f$k@f$ of the vector part.
     */
    explicit Quaternion(
        float a = 1.0f,
        float b = 0.0f,
        float c = 0.0f,
        float d = 0.0f);

    /**
     * @brief Computes the norm of the vector squared.
     *
     * @return Norm squared.
     */
    float normSqr() const;

    /**
     * @brief Computes the norm of the quaternion.
     *
     * @return Norm.
     */
    float norm() const;

    /**
     * @brief Normalise the quaternion.
     */
    void normalize();

    /**
     * @brief Creates a normalised version of the quaternion.
     *
     * @return Normalised Quaternion.
     */
    Quaternion normalised() const;

    /**
     * @brief Creates the conjugate of the quaternion.
     *
     * @return Quaternion conjugate.
     */
    Quaternion conjugate() const;

    /**
     * @brief Create quaternion from angles.
     *
     * @param roll roll angle @f$\phi@f$ in radians.
     * @param pitch pitch angle @f$\theta@f$ in radians.
     * @param yaw yaw angle @f$\psi@f$ in radians.
     * @return Quaternion
     */
    static Quaternion fromAngles(float roll, float pitch, float yaw) {
        // half angles
        float halfR { roll / 2.0f };
        float halfP { pitch / 2.0f };
        float halfY { yaw / 2.0f };
        // cos angles
        float cosR { cosf(halfR) };
        float cosP { cosf(halfP) };
        float cosY { cosf(halfY) };
        // sin angles
        float sinR { sinf(halfR) };
        float sinP { sinf(halfP) };
        float sinY { sinf(halfY) };

        return Quaternion {
            cosR * cosP * cosY + sinR * sinP * sinY,
            sinR * cosP * cosY - cosR * sinP * sinY,
            cosR * sinP * cosY + sinR * cosP * sinY,
            cosR * cosP * sinY - sinR * sinP * cosY,
        };
    }

    /**
     * @brief Clear content.
     * Sets the quaternion to a unit quaternion.
     */
    void clear();

    /**
     * @brief Computes the angle of the quaternion.
     *
     * @param inDegrees if the result in degrees or radians. defaults to
     * radians.
     * @return angle in the chosen unit.
     */
    float angle(bool inDegrees = false);

    /**
     * @brief The vector part of the quaternion
     *  (@f$\vec{i},\,\vec{j}\,\vec{k}@f$).
     *
     * @return axis of rotation.
     */
    Vector axis() const;

    /**
     * @brief Elementwise division by a scalar.
     *
     * @param n Divisor.
     * @return Quaternion
     */
    Quaternion operator/(float n) const;

    /**
     * @brief Compound assignment division by a scalar.
     *
     * @param n Divisor.
     * @return Quaternion&
     */
    Quaternion& operator/=(float n);

    /**
     * @brief Compound assignment addition with a quaternion.
     *
     * @param rhs Quaternion.
     * @return Quaternion&
     */
    Quaternion& operator+=(const Quaternion& rhs);

    /**
     * @brief Compound assignment subtraction with a quaternion.
     *
     * @param rhs Quaternion.
     * @return Quaternion&
     */
    Quaternion& operator-=(const Quaternion& rhs);

    /**
     * @brief Elementwise multiplication by a scalar.
     *
     * @param n Divisor.
     * @return Quaternion
     */
    Quaternion operator*(float n) const;

    Quaternion operator-() const;

    /**
     * @brief Addition with a quaternion.
     *
     * @param rhs Quaternion.
     * @return Quaternion&
     */
    Quaternion operator+(const Quaternion& rhs) const;

    /**
     * @brief Elementwise multiplication (Hadamard) by a vector.
     * It is done as if the vector is a quaternion with a scalar equals 0.
     *
     * @param rhs #Vector.
     * @return Quaternion
     */
    Quaternion operator*(const Vector& rhs) const;

    /**
     * @brief Elementwise multiplication (Hadamard) by a quaternion.
     *
     * @param rhs Quaternion.
     * @return Quaternion
     */
    Quaternion operator*(const Quaternion& rhs) const;

    /**
     * @brief Compund multiplication (Hadamard) by a quaternion.
     *
     * @param rhs Quaternion.
     * @return Quaternion&
     */
    Quaternion& operator*=(const Quaternion& rhs);

    /**
     * @brief Elementwise multiplication by a scalar.
     *
     * @param rhs Divisor.
     * @return Quaternion&
     */
    Quaternion& operator*=(float rhs);

    /**
     * @brief Equality operator.
     *
     * @param rhs Quaternion.
     * @return true if equal.
     * @return false otherwise.
     */
    bool operator==(const Quaternion& rhs) const;

    /**
     * @brief Inequality operator.
     *
     * @param rhs Quaternion.
     * @return true if equal.
     * @return false otherwise.
     */
    bool operator!=(const Quaternion& rhs) const;

    // Matrix RotationMatrix();

    /**
     * @brief Converts quaternion to a rotation matrix.
     *
     * @return Rotation Matrix.
     */
    Matrix toRotationMatrix() const;

    /**
     * @brief Reassign values from #Matrix.
     *
     * @param mat #Matrix.
     */
    void fromMatrix(const Matrix& mat);

    /**
     * @brief Reassign values from another quaternion.
     *
     * @param q #Quaternion.
     */
    void fromQuaternion(const Quaternion& q);

    /**
     * @brief Set the axis of rotation
     *  (@f$\vec{i},\,\vec{j}\,\vec{k}@f$)
     *
     * @param v #Vector
     */
    void setAxis(const Vector& v);

    /**
     * @brief Get the axis of rotation
     *  ( @f$\vec{i},\,\vec{j}\,\vec{k}@f$ )
     *
     * @return #Vector
     */
    Vector getAxis() const;

    /**
     * @brief set a member's value by index.
     * order coefficients: @f$w,x,y,z \Longleftrightarrow 0,1,2,3@f$
     *
     * @param idx element's index
     * @param value New value
     */
    void set(size_t idx, float value);

    /**
     * @brief Checks if the quaternion is a unit quaternion.
     * w = 1, x = y = z = 0
     *
     * @return true if a unit quaternion.
     * @return false otherwise
     */
    bool isUnit() const;

    /**
     * @brief Creates new Quaternion from an array,
     *  order coefficients: @f$w,x,y,z \Longleftrightarrow 0,1,2,3@f$
     *
     * @param array
     * @return Quaternion
     */
    static Quaternion fromArray(const float array[4]) {
        return Quaternion {
            array[0],
            array[1],
            array[2],
            array[3],
        };
    }
};

/**
 * @brief Multiplication if Quaternion on the right hand side.
 *
 * @param f scalar.
 * @param q Quaternion.
 * @return Quaternion
 */
Quaternion operator*(float f, const Quaternion& q);

#endif /* __LIB_CUSTOM_TYPE_QUATERNION_H__ */