#include "time_trigger.h"
#include "time_utils.h"

TimeTrigger::TimeTrigger(long period, int count): period(period), count(count)
{
    this->time = TimeUtils::getTime();
}

bool TimeTrigger::isFired(long curTime)
{
    if (count == 0) { // 0 means finished, so we return false
        return false;
    }

    // -1 means infinite, and > 0 means finite, so we don't need to check the count
    if (TimeUtils::isTimeout(curTime, time, period)) {
        time = curTime;

        if (count > 0) { // > 0 means finite, so we decrement the count
            --count;
        }

        return true;
    }
    else {
        return false;
    }
}

bool TimeTrigger::isFired()
{
    return isFired(TimeUtils::getTime());
}
