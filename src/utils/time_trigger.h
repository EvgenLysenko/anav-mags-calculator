#ifndef __TIME_TRIGGER_H__
#define __TIME_TRIGGER_H__

class TimeTrigger
{
protected:
    const long period = 0;
    long time = 0;
    int count = 0;

public:
    TimeTrigger(long period, int count = -1);

    bool isFired();
    bool isFired(long curTime);
};

#endif
