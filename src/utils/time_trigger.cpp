#include "time_trigger.h"
#include "time_utils.h"

TimeTrigger::TimeTrigger(long period, int count, bool startImmediately): period(period), count(count)
{
    this->time = startImmediately ? 0 : TimeUtils::getTime();
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

void TimeTrigger::start(long period, int count, bool startImmediately)
{
    this->period = period;
    this->count = count;

    this->time = startImmediately ? 0 :TimeUtils::getTime();
}