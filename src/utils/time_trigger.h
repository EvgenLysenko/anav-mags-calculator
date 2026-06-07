#ifndef __TIME_TRIGGER_H__
#define __TIME_TRIGGER_H__

class TimeTrigger
{
protected:
    const long period = 0;
    long time = 0;

public:
    TimeTrigger(long period);

    bool isFired();
    bool isFired(long curTime);
};

#endif
