#include "parse_utils.h"

using namespace std;

double ParseUtils::parseDouble(const char* str, double defaultValue)
{
    if (!str || !*str)
        return defaultValue;

    return atof(str);
}

float ParseUtils::parseFloat(const char* str, float defaultValue)
{
    if (!str || !*str)
        return defaultValue;

    return atof(str);
}
