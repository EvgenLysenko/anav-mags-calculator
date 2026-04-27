#include "task_base.h"
#include "logging.h"
#include "unistd.h"

static const __useconds_t RUN_THREAD_TIMEOUT_IDLE = 1000 * 1000; // 1s

TaskBase::~TaskBase()
{
    _isStopped = true;

    if (_thread.joinable()) {
        _thread.join();
    }

    logInfo("TASK - stopped");
}

void TaskBase::start()
{
    if (!_isStopped) {
        logWarning("TASK %s - already started", getTaskName());
        return;
    }

    _isStopped = false;

    logInfo("TASK %s - start", getTaskName());

    _thread = std::thread(&TaskBase::run, this);
}

void TaskBase::onStarted()
{
    logInfo("TASK %s - started", getTaskName());
}

void TaskBase::stop()
{
    logInfo("TASK %s - stop", getTaskName());

    _isStopped = true;
}

void TaskBase::run()
{
    logInfo("TASK %s - run", getTaskName());

    onStarted();

    while ( !_isStopped )
    {
        __useconds_t timeout = loop(RUN_THREAD_TIMEOUT_IDLE);

        if ( timeout )
            usleep(timeout);
    }

    logInfo("TASK %s - finished", getTaskName());
}
