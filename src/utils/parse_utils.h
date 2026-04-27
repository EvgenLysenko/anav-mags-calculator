#ifndef __PARSE_UTILS_H__
#define __PARSE_UTILS_H__

#include <string>
#include <list>
#include "math_utils.h"

class ParseUtils {
public:
    static double parseDouble(const char* str, double defaultValue);
    static float parseFloat(const char* str, float defaultValue = math_utils::FLOAT_UNDEFINED);
};

#endif
