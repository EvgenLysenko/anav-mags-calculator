#ifndef __MAVLINK_PROVIDER_H__
#define __MAVLINK_PROVIDER_H__

#include "connection/connection.h"
#include "mavlink/common/mavlink.h"
#include "mavlink/mavlink_types.h"
#include "flight_mode.h"
#include "utils/math_utils.h"
#include <list>
#include <string>

class IMavlinkReceiver {
public:
    virtual void onMessageReceived(const mavlink_message_t& mavlink_message) = 0;
};

class IMavlinkSender {
public:
    virtual void send(const mavlink_message_t& mavlink_message) = 0;
};

class MavlinkProvider: public IMavlinkSender
{
public:
    MavlinkProvider(Connection* connection): connection(connection) {}

protected:
    Connection* connection;
    std::list<IMavlinkReceiver*> mavlinkReceivers;

    std::list<mavlink_message_t> messagesToSend;

    void processRead();
    void processSend();

    int plane_system_id = -1;
    int plane_component_id = -1;
    int gsc_system_id = -1;
    int gsc_component_id = -1;
    int compSystemId = -1;
    int compComponentId = MAV_COMP_ID_ONBOARD_COMPUTER;
;
    void onMessageReceived(const mavlink_message_t& mavlink_message);

public:
    bool isConnected() const { return connection && connection->isConnected(); }
    bool isReady() const { return compSystemId != -1; }
    int getGscSystemId() const { return gsc_system_id; };
    int getGscComponentId() const { return gsc_component_id; };
    int getCompSystemId() const { return compSystemId; };
    int getCompComponentId() const { return compComponentId; };

    bool isPlaneReady() const { return plane_system_id != -1 || plane_component_id != -1; }
    bool isGscReady() const { return gsc_system_id != -1 || gsc_component_id != -1; }

    static int32_t radToDeg7(double rad) {
        return (int32_t)(rad * 180 / math_utils::PI * 1e7 + 0.5);
    }
    static double deg7ToRad(int32_t value) {
        return (double)(value * math_utils::PI / 180. / 1e7);
    }
    static double deg7ToRad(float value) {
        return (double)(value * math_utils::PI / 180. / 1e7);
    }

    void addMavlinkReceiver(IMavlinkReceiver* messageReceiver) {
        this->mavlinkReceivers.push_back(messageReceiver);
    }

    void process();
    virtual void send(const mavlink_message_t& mavlink_message);

    // messages
    void sendHeartBeat(int mav_mode);

    void sendCommandAck(int command, int result) { sendCommandAck(plane_system_id, plane_component_id, command, result); }
    void sendCommandAck(int target_system_id, int target_component_id, int command, int result);

    void sendCommandLong(uint8_t target_system, uint8_t target_component, MAV_CMD command, uint8_t confirmation,
        float param1, float param2, float param3, float param4, float param5, float param6, float param7);
    void sendCommandLong(MAV_CMD command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7) {
        sendCommandLong(plane_system_id, plane_component_id, command, confirmation, param1, param2, param3, param4, param5, param6, param7);
    }
    void sendCommandLong(MAV_CMD command, float param1, float param2, float param3, float param4, float param5, float param6, float param7) {
        sendCommandLong(plane_system_id, plane_component_id, command, 0, param1, param2, param3, param4, param5, param6, param7);
    }
    void sendCommandLong(MAV_CMD command, float param1, float param2) {
        sendCommandLong(plane_system_id, plane_component_id, command, 0, param1, param2, 0, 0, 0, 0, 0);
    }

    void sendCommandInt(uint8_t target_system, uint8_t target_component, MAV_CMD command, uint8_t frame, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z);

    void sendRequestMessageInterval(int messageId, int period);

    void sendFlightMode(int flightModeId);
    void sendFlightMode(int flightModeId, int customMode, FlightMode flightMode);

    void sendArm(bool forced = false);
    void sendDisarm(bool forced = false);
    void sendRequestParam(const char* paramId);

    void sendGuidedPosition(double lat, double lon, float alt);
    void sendGuidedPosition(double lat, double lon, float alt, MAV_FRAME mav_frame);

    void sendAttitude(float roll, float pitch, float yaw, float throttle);

    void sendStatustext(MAV_SEVERITY severity, const std::string& text);
};

#endif
