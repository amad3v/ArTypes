/**
 * @file vector.h
 * @date 04.01.22
 * @author amad3v (amad3v@gmail.com)
 * @version 0.0.1
 *
 * @brief 3D vector type.
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __LIB_CUSTOM_TYPE_VECTOR_H__
#define __LIB_CUSTOM_TYPE_VECTOR_H__

#include <cmath>
#include "def.h"

// to avoid cyclic dependencies.
// The header is included in the source file.
class Quaternion;

/**
 * @class Vector
 * @brief Creates a 3D vector.
 */
class Vector {
  public:
    /**
     * @brief Component along x-axis.
     */
    float x;
    /**
     * @brief Component along y-axis.
     */
    float y;
    /**
     * @brief Component along z-axis.
     */
    float z;

    /**
     * @brief Construct a new Vector object.
     * If no arguments provided, it creates a zero vector.
     *
     * @param a x-axis, defaults to 0.
     * @param b y-axis, defaults to 0.
     * @param c z-axis, defaults to 0.
     */
    explicit Vector(float a = 0.0f, float b = 0.0f, float c = 0.0f);

    /**
     * @brief Static method to create a vector with
     * the same value.
     *
     * @param a Value for 3 axes.
     * @return Vector
     */
    static Vector identical(float a) {
        return Vector {
            a,
            a,
            a,
        };
    }

    /**
     * @brief Verify if the vector is 0.
     *
     * @return true if all axes components equal 0.
     * @return false otherwise.
     */
    bool isNil() const;

    /**
     * @brief Verify if the vector is NAN.
     *
     * @return true if one axis component equal NAN.
     * @return false otherwise.
     */
    bool isNaN() const;

    /**
     * @brief Computes the norm of the vector.
     *
     * @return norm.
     */
    float norm() const;

    /**
     * @brief Computes the norm of the vector squared.
     *
     * @return Norm squared.
     */
    float normSqr() const;

    /**
     * @brief Normalise the vector.
     */
    void normalise();

    /**
     * @brief Normalised Vector.
     */
    Vector normalised() const;

    /**
     * @brief Invalidate the vector by assigning NaN to all its members.
     *
     * Alias of #setUndefined.
     */
    void setNaN();

    /**
     * @brief Computes the cross product of 2 vectors.
     *
     * @param rhs Vector.
     * @return Cross product Vector .
     */
    Vector cross(const Vector& rhs) const;

    /**
     * @brief Computes the dot product of 2 vectors.
     *
     * @param rhs Vector.
     * @return Dot product.
     */
    float dot(const Vector& rhs) const;

    /**
     * @brief Invalidate the vector by assigning NaN to all its members.
     *
     * Alias of #setNaN.
     */
    void setUndefined();

    /**
     * @brief Verify that all members are not equal to 0, assigns 1 to any 0
     * values.
     *
     */
    void noZeros();

    /**
     * @brief Element-wise division by a scalar.
     *
     * @param n Divisor.
     * @return Vector
     */
    Vector operator/(float n) const;

    /**
     * @brief Element-wise division by a vector.
     *
     * @param rhs Vector divisor.
     * @return Vector
     */
    Vector operator/(const Vector& rhs) const;

    /**
     * @brief Element-wise multiplication by a scalar.
     *
     * @param n Multiplier.
     * @return Vector
     */
    Vector operator*(float n) const;

    /**
     * @brief Element-wise multiplication by a vector.
     *
     * @param rhs Vector multiplier.
     * @return Vector
     */
    Vector operator*(const Vector& rhs) const;

    /**
     * @brief Element-wise multiplication (Hadamard) by a quaternion.
     *
     * It is done as if the vector is a quaternion with a scalar equals 0.
     *
     * @param rhs #Quaternion.
     * @return Quaternion
     */
    Quaternion operator*(const Quaternion& rhs) const;

    /**
     * @brief Element-wise subtraction with a vector.
     *
     * @param rhs Vector.
     * @return Vector
     */
    Vector operator-(const Vector& rhs) const;

    /**
     * @brief Element-wise subtraction with a scalar.
     *
     * @param rhs Scalar
     * @return Vector
     */
    Vector operator-(float rhs) const;

    /**
     * @brief Negation.
     *
     * @return Vector
     */
    Vector operator-() const;

    /**
     * @brief Element-wise addition with a vector.
     *
     * @param rhs Vector.
     * @return Vector
     */
    Vector operator+(const Vector& rhs) const;

    /**
     * @brief Element-wise addition with a scalar.
     *
     * @param rhs Scalar.
     * @return Vector
     */
    Vector operator+(float rhs) const;

    /**
     * @brief Element-wise coumpound assignemnt division by a scalar.
     *
     * @param n Divisor (scalar).
     */
    Vector& operator/=(float n);

    /**
     * @brief Element-wise coumpound assignemnt multiplication by a scalar.
     *
     * @param n Multiplier.
     */
    Vector& operator*=(float n);

    /**
     * @brief Element-wise coumpound assignemnt multiplication by a vector.
     *
     * @param rhs Vector multiplier.
     */
    Vector& operator*=(const Vector& rhs);

    /**
     * @brief Element-wise coumpound assignemnt addition with a vector.
     *
     * @param rhs Vector.
     */
    Vector& operator+=(const Vector& rhs);

    /**
     * @brief Element-wise coumpound assignemnt subtraction with a vector.
     *
     * @param rhs Vector.
     */
    Vector& operator-=(const Vector& rhs);

    /**
     * @brief Assign vector values from another vector.
     *
     * @param rhs Vector to copy from.
     */
    Vector& operator=(const Vector& rhs);

    /**
     * @brief Raise to the power of @p x.
     *
     * @param n Exponent.
     * @return Vector
     */
    Vector power(float n) const;

    /**
     * @brief Computes the sum of the vector's elements
     *
     * @return @f$\sum\left(x,y,z\right)@f$
     */
    float sum() const;

    /**
     * @brief Set vector's elements to 0.
     */
    void clear();

    /**
     * @brief Computes the square root of the vector's elements.
     *
     * @return Vector
     */
    Vector sqrt() const;

    /**
     * @brief Sets the vector's elements to their absolute value.
     *
     * @return Vector
     */
    Vector absf() const;
};

/**
 * @brief Multiplication if Vector on the right hand side.
 *
 * @param f scalar.
 * @param v #Vector.
 * @return #Vector
 */
Vector operator*(float f, const Vector& v);

/**
 * @brief Addition if Vector on the right hand side.
 *
 * @param f scalar.
 * @param v #Vector.
 * @return #Vector
 */
Vector operator+(float f, const Vector& v);

/**
 * @brief Substraction if Vector on the right hand side.
 *
 * @param f scalar.
 * @param v #Vector.
 * @return #Vector
 */
Vector operator-(float f, const Vector& v);

#endif /* __LIB_CUSTOM_TYPE_VECTOR_H__ */