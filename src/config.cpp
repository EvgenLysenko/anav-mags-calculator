#include "config.h"

#include "picojson/picojson.h"

#include <cstdint>
#include <fstream>
#include <string>
#include <list>

using namespace std;

static const char* configFileName = "mags-logger.conf";
static picojson::value conf;
static bool confIsLoaded = false;

static void readConfig()
{
    if (!confIsLoaded) {
        std::ifstream is(configFileName);

        is >> conf;

        confIsLoaded = true;
    }
}

static bool compare(const string& name, const char* first, const char* last)
{
    if (name.size() != last - first + 1)
        return false;

    for (string::const_iterator i = name.begin(); i != name.end() && first <= last; ++i, ++first) {
        if (*i != *first)
            return false;
    }

    return true;
}

static const picojson::value* configGetObject(const picojson::value* object, const char* first, const char* last)
{
    if (!object)
        return 0;

    if (object->is<picojson::object>()) {
        const picojson::object& o = object->get<picojson::object>();
        for (picojson::object::const_iterator i = o.begin(); i != o.end(); ++i) {
            if (compare(i->first, first, last)) {
                return &(i->second);
            }
        }
    }

    return 0;
}

static const picojson::value* configGetValue(const char* path)
{
    if (!path)
        return 0;

    readConfig();

    const char* first = path;
    const char* last = path;

    const picojson::value* object = &conf;

    while (*last && *last != '\n' && *last != '\r') {
        if (*last == '/') {
            object = configGetObject(object, first, last - 1);
            first = last + 1;
        }

        ++last;
    }

    if (first <= last)
        object = configGetObject(object, first, last - 1);

    return object;
}

string Config::readString(const char* path, const string& defaultValue)
{
    const picojson::value* value = configGetValue(path);

    if (value) {
        if (value->is<string>())
            return value->get<string>();
    }

    return defaultValue;
}

double Config::readDouble(const char* path, double defaultValue)
{
    const picojson::value* value = configGetValue(path);

    if (value) {
        if (value->is<double>())
            return value->get<double>();
    }

    return defaultValue;
}

bool Config::readBool(const char* path, bool defaultValue)
{
    const picojson::value* value = configGetValue(path);

    if (value) {
        if (value->is<bool>())
            return value->get<bool>();
    }

    return defaultValue;
}

int Config::readInt(const char* path, int defaultValue)
{
    const picojson::value* object = configGetValue(path);

    if (object) {
        if (object->is<double>()) {
            double value = object->get<double>();

            if (value >= 0)
                return (int)(value + 0.5);
            else
                return (int)(value - 0.5);
        }
    }

    return defaultValue;
}

float Config::readFloat(const char* path, float defaultValue)
{
    const picojson::value* object = configGetValue(path);

    if (object) {
        if (object->is<double>()) {
            double value = object->get<double>();

            return (float)value;
        }
    }

    return defaultValue;
}

int Config::readArrayString(const char* path, void (*callback)(const std::string& str, int index, void* data), void* data)
{
    int size = 0;

    const picojson::value* object = configGetValue(path);
    if (object) {
        if (object->is<picojson::array>()) {
            const picojson::array& arr = object->get<picojson::array>();
            for (const picojson::value& value: arr) {
                if (value.is<string>()) {
                    if (callback)
                        callback(value.get<string>(), size, data);

                    ++size;
                }
            }
        }
    }

    return size;
}
