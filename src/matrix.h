/**
 * @file matrix.h
 * @date 04.01.22
 * @author amad3v (amad3v@gmail.com)
 * @version 0.0.1
 *
 * @brief Matrix 3x3 type.
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __LIB_CUSTOM_TYPE_MATRIX_H__
#define __LIB_CUSTOM_TYPE_MATRIX_H__

#include "def.h"
#include "vector.h"

// to avoid cyclic dependencies.
// The header is included in the source file.
class Quaternion;

/**
 * @brief Matrix rows count.
 */
const size_t MATRIX_ROWS { 3 };

/**
 * @brief Matrix columns count.
 */
const size_t MATRIX_COLS { 3 };

/**
 * @brief Length of the internal storage (1D array).
 */
const size_t MATRIX_LEN { MATRIX_ROWS * MATRIX_COLS };

/**
 * @class Matrix3x3
 * @brief Creates a 3 by 3 matrix representation.
 */
class Matrix3x3 {
  private:
    /**
     * @brief Internal storage of matrix elements.
     */
    float members[9];

    /**
     * @brief Computes the a matrix element's index (2D to 1D).
     *
     * @param r Row index.
     * @param c Column index.
     * @return index of the element in the internal storage.
     */
    size_t index(size_t r, size_t c) const;

  public:
    /**
     * @brief Construct a new Matrix.
     * all members default to 0.
     *
     * @f$\begin{bmatrix}a_{11} & a_{12} & a_{13}\\
     * a_{21} & a_{22} & a_{23}\\
     * a_{31} & a_{32} & a_{33}
     * \end{bmatrix}@f$
     */
    explicit Matrix3x3(
        float b11 = 0.0f,
        float b12 = 0.0f,
        float b13 = 0.0f,
        float b21 = 0.0f,
        float b22 = 0.0f,
        float b23 = 0.0f,
        float b31 = 0.0f,
        float b32 = 0.0f,
        float b33 = 0.0f);

    /**
     * @brief Construct a new Matrix from an array
     *
     * @param mat Array of 9
     */
    explicit Matrix3x3(const float mat[]);

    /**
     * @brief Static method to merge/combine 3 vectors into a matrix.
     * xs for the first column, ys for the second and zs for the third.
     *
     * @f$\begin{bmatrix}x_{1}\\
        y_{1}\\
        z_{1}
        \end{bmatrix}\begin{bmatrix}x_{2}\\
        y_{2}\\
        z_{2}
        \end{bmatrix}\begin{bmatrix}x_{3}\\
        y_{3}\\
        z_{3}
        \end{bmatrix}\Longrightarrow\begin{bmatrix}x_{1} & x_{2} & x_{3}\\
        y_{1} & y_{2} & y_{3}\\
        z_{1} & z_{2} & z_{3}
        \end{bmatrix}@f$
   * @param v1 Column 1.
   * @param v2 Column 2.
   * @param v3 Column 3.
   * @return Matrix3x3
   */
    static Matrix3x3
    merge(const Vector& v1, const Vector& v2, const Vector& v3) {
        return Matrix3x3(v1.x, v2.x, v3.x, v1.y, v2.y, v3.y, v1.z, v2.z, v3.z);
    }

    /**
     * @brief Static method to create an identity matrix.
     *
     * @return Matrix3x3
     */
    static Matrix3x3 identity() {
        return Matrix3x3 {
            1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
        };
    }

    /**
     * @brief Computes the sum of the diagonal elements.
     * @f$trace=a_{11}+a_{22}+a_{33}@f$
     * @return float
     */
    float trace() const;

    /**
     * @brief Convert a matrix to quaternion.
     *
     * @return Quaternion
     */
    Quaternion toQuaternion() const;

    /**
     * @brief Matrix-#Vector product.
     *
     * @param rhs Vector.
     * @return Vector
     */
    Vector operator*(const Vector& rhs) const;

    /**
     * @brief Matrix determinant.
     */
    float det();

    /**
     * @brief Retrieve matrix member at row @p r and column @p c
     *
     * @param r row index [0..2]
     * @param c column index [0..2]
     * @return Matrix member.
     */
    float coeff(size_t r, size_t c) const;

    /**
     * @brief Set matrix member value.
     *
     * @param r row index [0..2]
     * @param c column index [0..2]
     * @param value New value, defaults to 0
     */
    void set(size_t r, size_t c, float value = 0.0f);

    /**
     * @brief Set matrix member value.
     *
     * @param i diagonal position [0..2]
     * @param value New value, defaults to 0
     */
    void set(size_t i, float value = 0.0f);

    /**
     * @brief Set matrix memebers from vectors
     *
     * @param vx row 1 or column 1, depends on @p row
     * @param vy row 2 or column 2, depends on @p row
     * @param vz row 3 or column 3, depends on @p row
     * @param row if true, treats vectors as rows, otherwise as columns
     */
    void fromVectors(
        const Vector& vx,
        const Vector& vy,
        const Vector& vz,
        bool row = true);

    /**
     * @brief Reset the matrix from an array.
     *
     * @param lst Array of length 9.
     */
    void reset(const float lst[]);

    /**
     * @brief Return a column vector
     *
     * @param idx Index of the column to fetch.
     * @return column @p idx
     */
    Vector col(size_t idx) const;

    /**
     * @brief Return a row vector
     *
     * @param idx Index of the row to fetch.
     * @return row @p idx
     */
    Vector row(size_t idx) const;

    /**
     * @brief Transpose of a matrix.
     *
     * @return Matrix3x3
     */
    Matrix3x3 transpose() const;
};

using Matrix = Matrix3x3;

#endif /* __LIB_CUSTOM_TYPE_MATRIX_H__ */