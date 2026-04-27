#ifndef __TASK_BASE_H__
#define __TASK_BASE_H__

#include <cstdint>
#include <sys/types.h>
#include <thread>
#include <string>

class TaskBase
{
public:
    TaskBase() {};
    virtual ~TaskBase();

    protected:
    std::thread _thread;
    bool _isStopped = true;

public:
    virtual const char* getTaskName() const = 0;
    virtual void start();
    virtual void stop();
    virtual void run();
    virtual bool isStopped() const { return _isStopped; }
    virtual __useconds_t loop(__useconds_t default_timeout) = 0;

    virtual void onStarted();
};

#endif
