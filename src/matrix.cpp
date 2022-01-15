/**
 * @file matrix.cpp
 * @date 04.01.22
 * @author amad3v (amad3v@gmail.com)
 * @version 0.0.1
 *
 * @brief Matrix 3x3 type.
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "matrix.h"
#include "quaternion.h"

Matrix3x3::Matrix3x3(const float mat[]) {
    for (size_t i {}; i < MATRIX_LEN; i++) {
        members[i] = mat[i];
    }
}

Matrix3x3::Matrix3x3(
    float b11,
    float b12,
    float b13,
    float b21,
    float b22,
    float b23,
    float b31,
    float b32,
    float b33) :
    members {} {
    float tmp[MATRIX_LEN] { b11, b12, b13, b21, b22, b23, b31, b32, b33 };
    reset(tmp);
}

size_t Matrix3x3::index(size_t r, size_t c) const {
    return MATRIX_COLS * r + c;
}

float Matrix3x3::coeff(size_t r, size_t c) const {
    return members[index(r, c)];
}

void Matrix3x3::set(size_t r, size_t c, float value) {
    members[index(r, c)] = value;
}

void Matrix3x3::set(size_t i, float value) {
    members[index(i, i)] = value;
}

float Matrix3x3::trace() const {
    return coeff(0, 0) + coeff(1, 1) + coeff(2, 2);
}

void Matrix3x3::reset(const float lst[]) {
    for (size_t i {}; i < MATRIX_LEN; i++) {
        members[i] = lst[i];
    }
}

Vector Matrix3x3::col(size_t idx) const {
    return Vector {
        members[index(0, idx)],
        members[index(1, idx)],
        members[index(2, idx)],
    };
}

Vector Matrix3x3::row(size_t idx) const {
    return Vector {
        members[index(idx, 0)],
        members[index(idx, 1)],
        members[index(idx, 2)],
    };
}

Matrix3x3 Matrix3x3::transpose() const {
    float tmp[MATRIX_LEN];

    for (size_t c {}; c < MATRIX_COLS; c++) {
        for (size_t r {}; r < MATRIX_ROWS; r++) {
            tmp[index(c, r)] = members[MATRIX_COLS * r + c];
        }
    }

    return Matrix3x3 { tmp };
}

void Matrix3x3::fromVectors(
    const Vector& vx,
    const Vector& vy,
    const Vector& vz,
    bool row) {
    if (row) {
        float tmp[MATRIX_LEN] {
            vx.x, vx.y, vx.z, vy.x, vy.y, vy.z, vz.x, vz.y, vz.z,
        };
        reset(tmp);
    } else {
        float tmp[MATRIX_LEN] {
            vx.x, vy.x, vz.x, vx.y, vy.y, vz.y, vx.z, vy.z, vz.z,
        };

        reset(tmp);
    }
}

Quaternion Matrix3x3::toQuaternion() const {
    float w { 0.5f * sqrtf(1.0f + trace()) };
    float w4 { w * 4.0f };

    float x { (coeff(1, 2) - coeff(2, 1)) / w4 };
    float y { (coeff(2, 0) - coeff(0, 2)) / w4 };
    float z { (coeff(0, 1) - coeff(1, 0)) / w4 };

    return Quaternion { w, x, y, z }.normalised();
}

Vector Matrix3x3::operator*(const Vector& rhs) const {
    return Vector {
        coeff(0, 0) * rhs.x + coeff(0, 1) * rhs.y + coeff(0, 2) * rhs.z,
        coeff(1, 0) * rhs.x + coeff(1, 1) * rhs.y + coeff(1, 2) * rhs.z,
        coeff(2, 0) * rhs.x + coeff(2, 1) * rhs.y + coeff(2, 2) * rhs.z,
    };
}

float Matrix3x3::det() {
    return coeff(0, 0) * (coeff(1, 1) * coeff(2, 2) - coeff(1, 2) * coeff(2, 1))
        + coeff(0, 1) * (coeff(1, 2) * coeff(2, 0) - coeff(1, 0) * coeff(2, 2))
        + coeff(0, 2) * (coeff(1, 0) * coeff(2, 1) - coeff(1, 1) * coeff(2, 0));
}