#include "time_trigger.h"
#include "time_utils.h"

TimeTrigger::TimeTrigger(long period): period(period) {
    this->time = TimeUtils::getTime();
}

bool TimeTrigger::isFired(long curTime) {
    if (TimeUtils::isTimeout(curTime, time, period)) {
        time = curTime;
        return true;
    }
    else {
        return false;
    }
}

bool TimeTrigger::isFired() {
    return isFired(TimeUtils::getTime());
}
