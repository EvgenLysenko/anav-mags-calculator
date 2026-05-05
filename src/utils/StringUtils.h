#ifndef __STRING_UTILS_H__
#define __STRING_UTILS_H__

class StringUtils
{
public:
    static int compare(const unsigned char* src, const unsigned char* txt, unsigned short size);
    static const unsigned char* contains(const unsigned char* text, unsigned short textSize, const unsigned char* searchText, unsigned short searchTextSize);
};

#endif
