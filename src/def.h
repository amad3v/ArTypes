/**
 * @file def.h
 * @date 04.01.22
 * @author amad3v (amad3v@gmail.com)
 * @version 0.0.1
 *
 * @brief Helpers for types.
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __LIB_CUSTOM_TYPES_DEF_H__
#define __LIB_CUSTOM_TYPES_DEF_H__

namespace cst {
const float RAD_TO_DEG { 57.295779513082320876798154814105 };

/**
 * @brief Computes the square of a number.
 *
 * @param f
 * @return Number @p f squared.
 */
inline float sqr(float f) {
    return f * f;
}
}  // namespace cst

#endif /* __LIB_CUSTOM_TYPES_DEF_H__ */