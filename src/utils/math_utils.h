#ifndef __MATH_UTILS_H__
#define __MATH_UTILS_H__

#include <cmath>

namespace math_utils {

const double PI = 3.14159265358979;
const float PIf = 3.14159265358979;

inline double toRad(double deg) {
    return deg / 180 * math_utils::PI;
}

inline double toDeg(double rad) {
    return rad * 180 / math_utils::PI;
}

inline float toRad(float deg) {
    return deg / 180 * math_utils::PIf;
}

inline float toDeg(float rad) {
    return rad * 180 / math_utils::PIf;
}

const float FLOAT_UNDEFINED = MAXFLOAT;
inline float isUndefined(float value) {
    return value == FLOAT_UNDEFINED;
}

inline float undefined() {
    return FLOAT_UNDEFINED;
}

};

using math_utils::toRad;
using math_utils::toDeg;

#endif
