#ifndef __TIME_TRIGGER_H__
#define __TIME_TRIGGER_H__

class TimeTrigger
{
public:
    static const int INFINITE = -1;

protected:
    long period = 0;
    int count = 0;
    long time = 0;

public:
    TimeTrigger() : period(0), count(0), time(0) {}
    TimeTrigger(long period, int count = INFINITE, bool startImmediately = false);
    void start(long period, int count = INFINITE, bool startImmediately = false);
    bool isFired();
    bool isFired(long curTime);
    bool isFinished() const { return count == 0; }
    void stop() { count = 0; }
};

#endif
