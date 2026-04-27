#include <unistd.h>
#include <sys/time.h>
#include "time_utils.h"

static struct timeval getTimeval();

static struct timeval startTime = getTimeval();

static struct timeval getTimeval()
{
    struct timeval time;
    gettimeofday(&time, NULL);
    return time;
}

long TimeUtils::getTime()
{
    struct timeval time;
    gettimeofday(&time, NULL);

    long milliseconds = ((time.tv_sec - startTime.tv_sec) * 1000 + (time.tv_usec - startTime.tv_usec) / 1000.0) + 0.5;

    return milliseconds;
}

bool TimeUtils::isTimeout(long time, long timeout)
{
    const long currentTime = getTime();

    return time + timeout <= currentTime;
}
