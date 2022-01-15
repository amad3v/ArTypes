/**
 * @file quaternion.cpp
 * @date 04.01.22
 * @author amad3v (amad3v@gmail.com)
 * @version 0.0.1
 *
 * @brief Quaternion type.
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "quaternion.h"

Quaternion::Quaternion(float a, float b, float c, float d) :
    w { a },
    x { b },
    y { c },
    z { d } {}

float Quaternion::normSqr() const {
    return cst::sqr(w) + cst::sqr(x) + cst::sqr(y) + cst::sqr(z);
}

float Quaternion::norm() const {
    return sqrtf(normSqr());
}

void Quaternion::normalize() {
    *this /= norm();
}

float Quaternion::angle(bool inDegrees) {
    if (inDegrees) {
        return (2.0f * acosf(w)) * cst::RAD_TO_DEG;
    }

    return 2.0f * acosf(w);
}

Vector Quaternion::axis() const {
    return Vector {
        x,
        y,
        z,
    };
}

Quaternion Quaternion::normalised() const {
    return *this / norm();
}

Quaternion Quaternion::conjugate() const {
    return Quaternion {
        w,
        -x,
        -y,
        -z,
    };
}

Quaternion Quaternion::operator/(float n) const {
    return Quaternion {
        w / n,
        x / n,
        y / n,
        z / n,
    };
}

Quaternion& Quaternion::operator/=(float n) {
    w /= n;
    x /= n;
    y /= n;
    z /= n;

    return *this;
}

Quaternion& Quaternion::operator+=(const Quaternion& rhs) {
    w += rhs.w;
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;

    return *this;
}

Quaternion& Quaternion::operator-=(const Quaternion& rhs) {
    w -= rhs.w;
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;

    return *this;
}

Quaternion Quaternion::operator*(float n) const {
    return Quaternion {
        w * n,
        x * n,
        y * n,
        z * n,
    };
}

Quaternion Quaternion::operator-() const {
    return Quaternion {
        -w,
        -x,
        -y,
        -z,
    };
}

Quaternion Quaternion::operator+(const Quaternion& rhs) const {
    return Quaternion {
        w + rhs.w,
        x + rhs.x,
        y + rhs.y,
        z + rhs.z,
    };
}

Quaternion Quaternion::operator*(const Vector& rhs) const {
    return Quaternion {
        -rhs.x * x - rhs.y * y - rhs.z * z,
        rhs.x * w - rhs.y * z + rhs.z * y,
        rhs.x * z + rhs.y * w - rhs.z * x,
        -rhs.x * y + rhs.y * x + rhs.z * w,
    };
}

Quaternion Quaternion::operator*(const Quaternion& rhs) const {
    return Quaternion {
        rhs.w * w - rhs.x * x - rhs.y * y - rhs.z * z,
        rhs.w * x + rhs.x * w - rhs.y * z + rhs.z * y,
        rhs.w * y + rhs.x * z + rhs.y * w - rhs.z * x,
        rhs.w * z - rhs.x * y + rhs.y * x + rhs.z * w,
    };
}

bool Quaternion::operator==(const Quaternion& rhs) const {
    return w == rhs.w && x == rhs.x && y == rhs.y && z == rhs.z;
}

bool Quaternion::operator!=(const Quaternion& rhs) const {
    return !(*this == rhs);
}

void Quaternion::clear() {
    w = 1.0;
    x = 0.0;
    y = 0.0;
    z = 0.0;
}

// Matrix3x3 Quaternion::RotationMatrix() {
//     const float ww { sqr(w) };
//     const float xx { sqr(x) };
//     const float yy { sqr(y) };
//     const float zz { sqr(z) };

//     return Matrix3x3 {
//         ww + xx - yy - zz,      2.0f * (x * y - w * z), 2.0f * (w * y + x *
//         z), 2.0f * (w * z + x * y), ww - xx + yy - zz,      2.0f * (y * z - w
//         * x), 2.0f * (x * z - w * y), 2.0f * (w * x + y * z), ww - xx - yy +
//         zz,
//     };
// }

Matrix3x3 Quaternion::toRotationMatrix() const {
    const float twx = x * w;
    const float twy = y * w;
    const float twz = z * w;
    const float txx = cst::sqr(x);
    const float txy = y * x;
    const float txz = z * x;
    const float tyy = cst::sqr(y);
    const float tyz = z * y;
    const float tzz = cst::sqr(z);

    const float a00 = 1.0f - (tyy + tzz);
    const float a01 = txy - twz;
    const float a02 = txz + twy;
    const float a10 = txy + twz;
    const float a11 = 1.0f - (txx + tzz);
    const float a12 = tyz - twx;
    const float a20 = txz - twy;
    const float a21 = tyz + twx;
    const float a22 = 1.0f - (txx + tyy);

    return Matrix3x3 {
        a00, a01, a02, a10, a11, a12, a20, a21, a22,
    };
}

void Quaternion::setAxis(const Vector& v) {
    x = v.x;
    y = v.y;
    z = v.z;
}

Vector Quaternion::getAxis() const {
    return Vector {
        x,
        y,
        z,
    };
}

bool Quaternion::isUnit() const {
    return w == 1.0f && x == 0.0f && y == 0.0f && z == 0.0f;
}

void Quaternion::fromQuaternion(const Quaternion& q) {
    w = q.w;
    x = q.x;
    y = q.y;
    z = q.z;
}

void Quaternion::fromMatrix(const Matrix3x3& mat) {
    // This algorithm comes from  "Quaternion Calculus and Fast Animation",
    // Ken Shoemake, 1987 SIGGRAPH course notes
    float t = mat.trace();

    if (t > 0.0f) {
        t = sqrtf(t + 1.0f);
        w = 0.5f * t;
        t = 0.5f / t;
        x = (mat.coeff(2, 1) - mat.coeff(1, 2)) * t;
        y = (mat.coeff(0, 2) - mat.coeff(2, 0)) * t;
        z = (mat.coeff(1, 0) - mat.coeff(0, 1)) * t;
    } else {
        size_t i = 0;
        if (mat.coeff(1, 1) > mat.coeff(0, 0)) {
            i = 1;
        }

        if (mat.coeff(2, 2) > mat.coeff(i, i)) {
            i = 2;
        }

        size_t j = (i + 1) % 3;
        size_t k = (j + 1) % 3;

        t = sqrtf(mat.coeff(i, i) - mat.coeff(j, j) - mat.coeff(k, k) + 1.0f);
        set(i, 0.5f * t);
        t = 0.5f / t;
        w = (mat.coeff(k, j) - mat.coeff(j, k)) * t;
        set(j, (mat.coeff(j, i) + mat.coeff(i, j)) * t);
        set(k, (mat.coeff(k, i) + mat.coeff(i, k)) * t);
    }
}

void Quaternion::set(size_t idx, float value) {
    switch (idx) {
        case 0:
            w = value;
            break;
        case 1:
            x = value;
            break;
        case 2:
            y = value;
            break;
        case 3:
            z = value;
            break;

        default:
            break;
    }
}

Quaternion& Quaternion::operator*=(const Quaternion& rhs) {
    *this = *this * rhs;

    return *this;
}

Quaternion& Quaternion::operator*=(float rhs) {
    *this = *this * rhs;

    return *this;
}

Quaternion operator*(float f, const Quaternion& q) {
    return q * f;
}