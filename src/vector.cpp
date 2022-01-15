/**
 * @file vector.cpp
 * @date 04.01.22
 * @author amad3v (amad3v@gmail.com)
 * @version 0.0.1
 *
 * @brief 3D vector type.
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "vector.h"

#include "quaternion.h"

Vector::Vector(float a, float b, float c) : x { a }, y { b }, z { c } {}

bool Vector::isNil() const {
    return (x == 0.0f) && (y == 0.0f) && (z == 0.0f);
}

float Vector::norm() const {
    return sqrtf(normSqr());
}

float Vector::normSqr() const {
    return cst::sqr(x) + cst::sqr(y) + cst::sqr(z);
}

void Vector::normalise() {
    *this /= norm();
}

Vector Vector::normalised() const {
    return *this / norm();
}

void Vector::setNaN() {
    x = NAN;
    y = NAN;
    z = NAN;
}

Vector Vector::cross(const Vector& rhs) const {
    return Vector {
        y * rhs.z - z * rhs.y,
        z * rhs.x - x * rhs.z,
        x * rhs.y - y * rhs.x,
    };
}

float Vector::dot(const Vector& rhs) const {
    return x * rhs.x + y * rhs.y;
}

void Vector::setUndefined() {
    x = NAN;
    y = NAN;
    z = NAN;
}

void Vector::noZeros() {
    if (x == 0.0f) {
        x = 1.0f;  // zero slope not allowed
    }
    if (y == 0.0f) {
        y = 1.0f;
    }
    if (z == 0.0f) {
        z = 1.0f;
    }
}

Vector Vector::operator/(float n) const {
    return Vector {
        x / n,
        y / n,
        z / n,
    };
}

Vector Vector::operator/(const Vector& rhs) const {
    return Vector {
        x / rhs.x,
        y / rhs.y,
        z / rhs.z,
    };
}

Vector Vector::operator*(float n) const {
    return Vector {
        x * n,
        y * n,
        z * n,
    };
}

Vector Vector::operator*(const Vector& rhs) const {
    return Vector {
        x * rhs.x,
        y * rhs.y,
        z * rhs.z,
    };
}

Quaternion Vector::operator*(const Quaternion& rhs) const {
    return Quaternion {
        -x * rhs.x - y * rhs.y - z * rhs.z,
        x * rhs.w - y * rhs.z + z * rhs.y,
        x * rhs.z + y * rhs.w - z * rhs.x,
        -x * rhs.y + y * rhs.x + z * rhs.w,
    };
}

Vector Vector::operator-(const Vector& rhs) const {
    return Vector {
        x - rhs.x,
        y - rhs.y,
        z - rhs.z,
    };
}

Vector Vector::operator-() const {
    return Vector {
        -x,
        -y,
        -z,
    };
}

Vector Vector::operator+(const Vector& rhs) const {
    return Vector {
        x + rhs.x,
        y + rhs.y,
        z + rhs.z,
    };
}

Vector Vector::operator+(float rhs) const {
    return Vector {
        x + rhs,
        y + rhs,
        z + rhs,
    };
}

Vector& Vector::operator/=(float n) {
    x /= n;
    y /= n;
    z /= n;

    return *this;
}

Vector& Vector::operator*=(float n) {
    x *= n;
    y *= n;
    z *= n;

    return *this;
}

Vector& Vector::operator*=(const Vector& rhs) {
    x *= rhs.x;
    y *= rhs.y;
    z *= rhs.z;

    return *this;
}

Vector& Vector::operator+=(const Vector& rhs) {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;

    return *this;
}

Vector& Vector::operator-=(const Vector& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;

    return *this;
}

Vector& Vector::operator=(const Vector& rhs) {
    x = rhs.x;
    y = rhs.y;
    z = rhs.z;

    return *this;
}

float Vector::sum() const {
    return x + y + z;
}

Vector Vector::power(float n) const {
    return Vector {
        powf(x, n),
        powf(y, n),
        powf(z, n),
    };
}

void Vector::clear() {
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
}

Vector Vector::sqrt() const {
    return Vector {
        sqrtf(x),
        sqrtf(y),
        sqrtf(z),
    };
}

Vector Vector::absf() const {
    return Vector {
        fabsf(x),
        fabsf(y),
        fabsf(z),
    };
}

Vector Vector::operator-(float rhs) const {
    return Vector {
        x - rhs,
        y - rhs,
        z - rhs,
    };
}

bool Vector::isNaN() const {
    return x == NAN || y == NAN || z == NAN;
}

Vector operator*(float f, const Vector& v) {
    return v * f;
}

Vector operator+(float f, const Vector& v) {
    return v + f;
}

Vector operator-(float f, const Vector& v) {
    return v - f;
}