#include "command_requester.h"
#include "time_utils.h"
#include "logging.h"

void CommandRequester::loop()
{
    if (isFinished())
        return;

    const long curTime = TimeUtils::getTime();

    if (TimeUtils::isTimeout(curTime, commandSendTime, repeatTimeout)) {
        logInfo("Mags - command requester: \"%s\"  size: %d  tries: %d", command.c_str(), (int)command.size(), sendCountLeft);

        if (sender) {
            sender->send((const unsigned char*)command.c_str(), command.size());
        }
        else {
            logError("Mags - command requester - sender not defined");
        }

        commandSendTime = curTime;
        
        if (sendCountLeft > 0)
            --sendCountLeft;

        if (sendCountLeft == 0) {
            logInfo("Mags - command requester: \"%s\"  tries: 0  finished", command.c_str());
        }
    }
}
