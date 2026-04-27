#ifndef __TIME_UTILS_H__
#define __TIME_UTILS_H__

class TimeUtils {
public:
    static long getTime();

    static bool isTimeout(long time, long timeout);

    static bool isTimeout(long currentTime, long time, long timeout) {
        return time + timeout <= currentTime;
    }
};

#endif
