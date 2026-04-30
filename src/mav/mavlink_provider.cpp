#include "mavlink_provider.h"
#include <cstring>
#include "mavlink/common/mavlink.h"
#include "mavlink/common/common.h"
#include "mavlink/mavlink_types.h"
#include "mavlink/protocol.h"
#include "mavlink/mavlink_helpers.h"
#include "mavlink/message.hpp"
#include <mutex>
#include "logging.h"

uint8_t buff[MAVLINK_MAX_PACKET_LEN];

std::mutex sendMutex;

void MavlinkProvider::send(const mavlink_message_t& mavlink_message)
{
    std::lock_guard lock(sendMutex);

    messagesToSend.push_back(mavlink_message);
}

void MavlinkProvider::process()
{
    if (!connection || !connection->isConnected())
        return;

    processRead();
    processSend();
}

void MavlinkProvider::processRead()
{
    int receivedSize = 0;
    if (connection) {
        //memset(buff, 0, sizeof(buff));
        receivedSize = connection->read(buff, sizeof(buff));
    }

    //logInfo("receivedSize: %d", receivedSize);

    if (receivedSize > 0) {
        mavlink_message_t message;
        mavlink_status_t status;

        for (int i = 0; i < receivedSize; ++i) {
            uint8_t msgReceived = mavlink_parse_char(MAVLINK_COMM_1, buff[i], &message, &status);
            if (msgReceived == MAVLINK_FRAMING_OK) {
                //logInfo("received msg: %d", message.msgid);
                onMessageReceived(message);
            }
            else if (msgReceived != MAVLINK_FRAMING_INCOMPLETE){
                logInfo("received msg bed: %d : [%d]", receivedSize, buff[0]);
            }
        }
    }
}

void MavlinkProvider::processSend()
{
    std::lock_guard lock(sendMutex);

    if (!messagesToSend.empty()) {
        const mavlink_message_t& message = messagesToSend.front();

        int sentBytes = connection->send((unsigned char*)&message, sizeof(mavlink_message_t));

        if (sentBytes != sizeof(mavlink_message_t))
            logInfo("send incomplete - id:%d from %d/%d sent: %d/%d", message.msgid, message.sysid, message.compid, sentBytes, (int)sizeof(mavlink_message_t));

        messagesToSend.pop_front();
    }
}

void MavlinkProvider::onMessageReceived(const mavlink_message_t& message)
{
    switch (message.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
        const uint8_t heartbeatType = mavlink_msg_heartbeat_get_type(&message);

        switch (heartbeatType) {
        case MAV_TYPE_GCS:
            if (gsc_component_id != message.compid || gsc_system_id != message.sysid)
                logInfo("gsc online: %d/%d", message.sysid, message.compid);

            gsc_system_id = message.sysid; 
            gsc_component_id = message.compid;
            break;

        case MAV_TYPE_FIXED_WING:
            if (plane_component_id != message.compid || plane_system_id != message.sysid)
                logInfo("Plane online: %d/%d", message.sysid, message.compid);

            plane_system_id = message.sysid;
            plane_component_id = message.compid;

            compSystemId = plane_system_id;
            break;
        };

        break;
    }
    case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(&message, &ack);
        logInfo("recv MAVLINK_MSG_ID_COMMAND_ACK: command: %d  result %d  progress %d  to %d/%d", ack.command, ack.result, ack.progress, ack.target_system, ack.target_component);
        break;
    }
    default:
        break;
    }

    for (std::list<IMavlinkReceiver*>::iterator mavlinkReceiver = mavlinkReceivers.begin(); mavlinkReceiver != mavlinkReceivers.end(); ++mavlinkReceiver) {
        (*mavlinkReceiver)->onMessageReceived(message);
    }
}
