#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <string>

class Config
{
public:
    static std::string readString(const char* path, const std::string& defaultValue);
    static std::string readString(const char* path) { return readString(path, ""); }
    static double readDouble(const char* path, double defaultValue);
    static bool readBool(const char* path, bool defaultValue = false);

    static int readInt(const char* path, int defaultValue);
    static float readFloat(const char* path, float defaultValue);

    static int readArrayString(const char* path, void (*callback)(const std::string& str, int index, void* data), void* data);
};

#endif
