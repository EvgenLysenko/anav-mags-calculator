#ifndef __TIME_TRIGGER_H__
#define __TIME_TRIGGER_H__

class TimeTrigger
{
public:
    static const int INFINITE = -1;
    static const int STOPPED = 0;

protected:
    long period = 0;
    int countLeft = STOPPED;
    long time = 0;

public:
    TimeTrigger() : period(0), countLeft(STOPPED), time(0) {}
    TimeTrigger(long period, int countLeft = INFINITE, bool startImmediately = false);
    TimeTrigger(long period, bool startImmediately);
    void start(long period, int countLeft = INFINITE, bool startImmediately = false);
    bool isFired();
    bool isFired(long curTime);
    bool isFinished() const { return countLeft == STOPPED; }
    void stop() { countLeft = STOPPED; }

    long getPreiod() const { return period; }
    long getCountLeft() const { return countLeft; }
};

#endif
