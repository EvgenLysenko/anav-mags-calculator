#ifndef __MAGS_CALCULATOR_H__
#define __MAGS_CALCULATOR_H__

#include "mags_logger.h"
#include <fstream>
#include <vector>
#include <string>

class MagsCalculator: public TaskBase, public MagsLogger::IMagsDataListener
{
public:
    static int WINDOW_SIZE;
    virtual const char* getTaskName() const { return "MagsCalculator"; }

protected:
    struct Record {
        uint32_t time;
        MagsLogger::Mag mags[2];
    };

    std::vector<Record> recordsAccumulator;
    std::vector<Record> records;

protected:
    virtual void onStarted();
    __useconds_t loop(__useconds_t default_timeout);

public:
    virtual void onMagsDataReceived(uint32_t time, const MagsLogger::Mag& mag0, const MagsLogger::Mag& mag1);

protected:
    void onRecordsAccumulated();
};

#endif
