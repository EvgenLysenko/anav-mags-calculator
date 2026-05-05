#include "StringUtils.h"

int StringUtils::compare(const unsigned char* src, const unsigned char* txt, unsigned short size)
{
    while (size--) {
        if (*txt++ != *src++)
            return 0;
    }

    return size == (unsigned short)-1;
}

const unsigned char* StringUtils::contains(const unsigned char* text, unsigned short textSize, const unsigned char* searchText, unsigned short searchTextSize)
{
    while (textSize) {
        if (searchTextSize > textSize)
            return 0;

        if (*text == *searchText) {
            if (compare(text, searchText, searchTextSize))
                return text;
        }

        ++text;
        --textSize;
    }

    return 0;
}
