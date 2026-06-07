#ifndef __COMMAND_REQUESTER_H__
#define __COMMAND_REQUESTER_H__

#include "mags-logger/nmea_sentence_reader.h"
#include <string>

class CommandRequester
{
public:
    CommandRequester() {}
    INmeaSentenceSender* sender = 0;
    int repeatCount = 0;
    long repeatTimeout = 0;

    void init(INmeaSentenceSender* nmeaSentenceSender) {
        this->sender = nmeaSentenceSender;
    }

protected:
    int id = 0;
    int tag = 0;
    std::string command;
    bool commandRequested = false;
    long commandSendTime = 0;
    int sendCountLeft = 0;
    bool commandIsConfirmed = true;

public:
    void setCommand(const std::string& command, int repeatCount, long repeatTimeout, int id, int tag = 0) {
        this->id = id;
        this->tag = tag;
        this->command = command;
        this->repeatCount = repeatCount;
        this->repeatTimeout = repeatTimeout;
        this->commandSendTime = 0;
        this->sendCountLeft = repeatCount;
        this->commandIsConfirmed = false;
    }

    int getId() const { return id; }
    int getTag() const { return tag; }
    const std::string& getCommand() const { return command; }
    void confirm(bool confirmed) { commandIsConfirmed = confirmed; }
    void confirmOk() { commandIsConfirmed = true; }

    bool isFinished() const { return commandIsConfirmed || sendCountLeft == 0; }
    bool isCommandConfirmed() const;

    virtual void loop();
};


#endif
