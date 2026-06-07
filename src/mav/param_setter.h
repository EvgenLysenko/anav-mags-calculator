#ifndef __PARAM_SETTER_H__
#define __PARAM_SETTER_H__

#include "mavlink_provider.h"

class ParamSetter: public IMavlinkReceiver {
public:
    ParamSetter(MavlinkProvider* mavProvider): mavProvider(mavProvider) {}
    MavlinkProvider* const mavProvider;
    int repeatCount = 0;
    long repeatTimeout = 0;

protected:
    int id = 0;
    int tag = 0;
    std::string paramName;
    float paramValue;
    bool commandRequested = false;
    long commandSendTime = 0;
    int sendCountLeft = 0;
    bool commandIsConfirmed = true;

public:
    void setParam(const std::string& paramName, float paramValue, long repeatTimeout, int repeatCount, int id, int tag = 0) {
        this->id = id;
        this->tag = tag;
        this->paramName = paramName;
        this->paramValue = paramValue;
        this->repeatTimeout = repeatTimeout;
        this->repeatCount = repeatCount;
        this->commandSendTime = 0;
        this->sendCountLeft = repeatCount;
        this->commandIsConfirmed = false;
    }

    void doNow();

    int getId() const { return id; }
    int getTag() const { return tag; }
    const std::string& getParamName() const { return paramName; }
    float getParamValue() const { return paramValue; }
    void confirm(bool confirmed) { commandIsConfirmed = confirmed; }
    void confirmOk() { commandIsConfirmed = true; }

    bool isFinished() const { return commandIsConfirmed || sendCountLeft == 0; }
    bool isCommandConfirmed() const;

    virtual void loop();
    virtual void onMessageReceived(const mavlink_message_t& mavlink_message);
};

#endif
