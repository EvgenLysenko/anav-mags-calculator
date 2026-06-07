#include "param_setter.h"
#include "utils/time_utils.h"
#include "logging.h"
#include <cstring>

void ParamSetter::onMessageReceived(const mavlink_message_t& mavlink_message)
{
    if (isFinished())
        return;

    if (mavlink_message.msgid == MAVLINK_MSG_ID_PARAM_VALUE) {
        mavlink_param_value_t param_value;
        mavlink_msg_param_value_decode(&mavlink_message, &param_value);

        // param_id is a 16-byte field that is not guaranteed to be null-terminated.
        char paramId[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN + 1] = {0};
        memcpy(paramId, param_value.param_id, MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN);

        logInfo("Mags - param value: %s = %f", paramId, param_value.param_value);

        if (paramName == paramId) {
            if (param_value.param_value == paramValue)
                confirm(true);

            logInfo("Mags - param set confirmed: %s = %f", paramId, param_value.param_value);
        }
    }
}

void ParamSetter::loop()
{
    if (isFinished())
        return;

    const long curTime = TimeUtils::getTime();

    if (TimeUtils::isTimeout(curTime, commandSendTime, repeatTimeout)) {
        logInfo("Mags - param requester: \"%s\"  size: %d  tries: %d", paramName.c_str(), (int)paramName.size(), sendCountLeft);

        if (mavSender) {
            mavSender->sendParamSet(paramName.c_str(), paramValue);
        }
        else {
            logError("Mags - param requester - sender not defined");
        }

        commandSendTime = curTime;

        if (sendCountLeft > 0)
            --sendCountLeft;

        if (sendCountLeft == 0) {
            logInfo("Mags - param requester: \"%s\"  tries: 0  finished", paramName.c_str());
        }
    }
}
