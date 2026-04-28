#include "mags_calculator.h"
#include "logging.h"
#include "utils/time_utils.h"
#include <mutex>

int MagsCalculator::WINDOW_SIZE = 1024;

static std::mutex recordsMutex; 

static const long RUN_THREAD_TIMEOUT_IN_PROGRESS = TaskBase::ms2Time(100);
static const long RUN_THREAD_TIMEOUT_IDLE = TaskBase::ms2Time(1000); // 1s

void MagsCalculator::onStarted()
{
    logInfo("MagsCalculator - started");

    recordsAccumulator.reserve(MagsCalculator::WINDOW_SIZE);
}

void MagsCalculator::onMagsDataReceived(uint32_t time, const MagsLogger::Mag& mag0, const MagsLogger::Mag& mag1)
{
    {
        std::lock_guard<std::mutex> lock(recordsMutex);

        recordsAccumulator.push_back({time, mag0, mag1});

        if ((int)recordsAccumulator.size() >= WINDOW_SIZE) {
            records.swap(recordsAccumulator);
        }
    }

    static TimerTrigger trigger(1000);
    if (trigger.isFired())
        logInfo("MagsCalculator - mags received. total: %d", (int)recordsAccumulator.size());
}

__useconds_t MagsCalculator::loop(__useconds_t default_timeout)
{
    const long curTime = TimeUtils::getTime();

    if (!records.empty()) {
        onRecordsAccumulated();
    }

    return recordsAccumulator.empty() ? default_timeout : RUN_THREAD_TIMEOUT_IN_PROGRESS;
}

void MagsCalculator::onRecordsAccumulated()
{
    logInfo("MagsCalculator - onRecordsAccumulated. size: %d", (int)records.size());
    // TODO
    records.clear();
}
