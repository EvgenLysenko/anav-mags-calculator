#include "mavlink_provider.h"
#include "logging.h"
#include <cstring>

#include <fstream>

void MavlinkProvider::sendHeartBeat(int mav_mode)
{
    mavlink_heartbeat_t heartbeat;
    heartbeat.type = MAV_TYPE::MAV_TYPE_ONBOARD_CONTROLLER;
    heartbeat.autopilot = MAV_AUTOPILOT::MAV_AUTOPILOT_INVALID;
    heartbeat.base_mode = (uint8_t)mav_mode;//MAV_MODE::MAV_MODE_GUIDED_ARMED;//MAV_MODE_MANUAL_ARMED;
    heartbeat.custom_mode = 0;
    heartbeat.system_status = MAV_STATE::MAV_STATE_ACTIVE;

    mavlink_message_t message;
    mavlink_msg_heartbeat_encode_chan(compSystemId, compComponentId, 0, &message, &heartbeat);

    send(message);

    static bool tested = false;
    if (!tested) {
        std::ofstream testOut("mav_heartbeat_1_191-no_pack.txt", std::ios::binary);
        std::ofstream testOut2("mav_heartbeat_1_191-pack.txt", std::ios::binary);
        tested = true;
        mavlink_msg_heartbeat_encode_chan(1, 191, 0, &message, &heartbeat);
        testOut.write((char*)&message, sizeof(mavlink_message_t));
        testOut.close();

        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &message);
        testOut2.write((char*)&buf, len);
        testOut2.close();

        logInfo("TEST");
    }
}

void MavlinkProvider::sendCommandAck(int target_system_id, int target_component_id, int command, int result)
{
    mavlink_command_ack_t com = {0};
    com.target_system = target_system_id;
    com.target_component = target_component_id;
    com.command = command;
    com.result = result;

    mavlink_message_t message;
    mavlink_msg_command_ack_encode(compSystemId, compComponentId, &message, &com);

    logInfo("send ACK: from %d/%d  to %d/%d  command: %d result: %d", compSystemId, compComponentId, target_system_id, target_component_id, command, result);

    send(message);
}

void MavlinkProvider::sendCommandLong(
    uint8_t target_system, uint8_t target_component, MAV_CMD command, uint8_t confirmation,
    float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    mavlink_command_long_t com;

    com.target_system = target_system;
    com.target_component = target_component;
    com.command = command;
    com.confirmation = confirmation;

    com.param1 = param1;
    com.param2 = param2;
    com.param3 = param3;
    com.param4 = param4;
    com.param5 = param5;
    com.param6 = param6;
    com.param7 = param7;

    //logInfo("Mavlink - send_command_long: from %d/%d to %d/%d", compSystemId, compComponentId, target_system, target_component);

    mavlink_message_t message;
    mavlink_msg_command_long_encode(compSystemId, compComponentId, &message, &com);

    send(message);
}

void MavlinkProvider::sendCommandInt(uint8_t target_system, uint8_t target_component, MAV_CMD command, 
    uint8_t frame, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z)
{
    mavlink_command_int_t com = {0};

    com.target_system = target_system;
    com.target_component = target_component;
    com.command = command;
    com.current = 0;
    com.autocontinue = 0;
    com.frame = frame;

    com.param1 = param1;
    com.param2 = param2;
    com.param3 = param3;
    com.param4 = param4;
    com.x = x;
    com.y = y;
    com.z = z;

    mavlink_message_t message;
    mavlink_msg_command_int_encode(compSystemId, compComponentId, &message, &com);

    send(message);
}

void MavlinkProvider::sendRequestMessageInterval(int messageId, int period)
{
    mavlink_command_long_t command;
    command.target_system = plane_system_id;
    command.target_component = plane_component_id;
    command.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    command.confirmation = 1;
    command.param1 = messageId;
    command.param2 = period;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(compSystemId, compComponentId, &message, &command);

    send(message);
}


void MavlinkProvider::sendFlightMode(int flightModeId)
{
    mavlink_command_long_t command;
    command.target_system = plane_system_id;
    command.target_component = plane_component_id;
    command.command = MAV_CMD_DO_SET_MODE;
    command.confirmation = 0;
    command.param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;// | MAV_MODE_FLAG_GUIDED_ENABLED;
    command.param2 = flightModeId;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(compSystemId, compComponentId, &message, &command);

    logInfo("send FLIGHT_MODE: from %d/%d  mode: %d  sub: %d", compSystemId, compComponentId, (int)command.param1, flightModeId);

    send(message);
}

void MavlinkProvider::sendFlightMode(int flightModeId, int customMode, FlightMode flightMode)
{
    logInfo("send FLIGHT_MODE: from %d/%d  custom_mode: %d  sub_mode: %d", compSystemId, compComponentId, customMode, flightMode);

    //MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_STABILIZE_ENABLED,
    sendCommandLong(MAV_CMD::MAV_CMD_DO_SET_MODE, customMode, flightMode);
}

void MavlinkProvider::sendArm(bool forsed)
{
    logInfo("send ARM: from %d/%d  forsed: %d", compSystemId, compComponentId, forsed); // TODO forced
    sendCommandLong(MAV_CMD::MAV_CMD_COMPONENT_ARM_DISARM, 1, 0);
}

void MavlinkProvider::sendDisarm(bool forsed)
{
    logInfo("send DISARM: from %d/%d  forsed: %d", compSystemId, compComponentId, forsed); // TODO forced
    sendCommandLong(MAV_CMD::MAV_CMD_COMPONENT_ARM_DISARM, 0, 0);
}

void MavlinkProvider::sendRequestParam(const char* paramId)
{
    mavlink_message_t message;

    logInfo("send PARAM_REQUEST: from %d/%d  to %d/%d  param: %s", compSystemId, compComponentId, plane_system_id, plane_component_id, paramId);

    char buff[MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN];
    for (int i = 0; i < sizeof(buff); ++i) {
        if (paramId && *paramId) {
            buff[i] = *paramId++;
        }
        else {
            paramId = 0;
            buff[i] = 0;
        }
    }

    mavlink_msg_param_request_read_pack(compSystemId, compComponentId, &message, plane_system_id, plane_component_id, buff, -1);

    send(message);
}

void MavlinkProvider::sendGuidedPosition(double lat, double lon, float alt)
{
    mavlink_mission_item_int_t mission_item;
    mission_item.target_system = plane_system_id;
    mission_item.target_component = plane_component_id;
    mission_item.command = MAV_CMD_NAV_WAYPOINT; 
    mission_item.current = 2;
    mission_item.frame = MAV_FRAME::MAV_FRAME_GLOBAL_RELATIVE_ALT;//MAV_FRAME::MAV_FRAME_GLOBAL;
    mission_item.autocontinue = 0;
    mission_item.mission_type = MAV_MISSION_TYPE::MAV_MISSION_TYPE_MISSION;
    mission_item.x = radToDeg7(lat);
    mission_item.y = radToDeg7(lon);
    mission_item.z = alt;

    mavlink_message_t message;
    mavlink_msg_mission_item_int_encode(compSystemId, compComponentId, &message, &mission_item);

    // LOG_F(INFO, "mission_item: cmd %d  cur %d  frame %d  type %d  x %d  y %d  z %f", mission_item.command, mission_item.current,
    //     mission_item.frame, mission_item.mission_type, mission_item.x, mission_item.y, mission_item.z);

    send(message);
}

void MavlinkProvider::sendGuidedPosition(double lat, double lon, float alt, MAV_FRAME mav_frame)
{
    mavlink_mission_item_int_t mission_item;
    mission_item.target_system = plane_system_id;
    mission_item.target_component = plane_component_id;
    mission_item.command = MAV_CMD_NAV_WAYPOINT; 
    mission_item.current = 2;
    mission_item.frame = mav_frame;
    mission_item.autocontinue = 0;
    mission_item.mission_type = MAV_MISSION_TYPE::MAV_MISSION_TYPE_MISSION;
    mission_item.x = radToDeg7(lat);
    mission_item.y = radToDeg7(lon);
    mission_item.z = alt;

    mavlink_message_t message;
    mavlink_msg_mission_item_int_encode(compSystemId, compComponentId, &message, &mission_item);

    // LOG_F(INFO, "mission_item: cmd %d  cur %d  frame %d  type %d  x %d  y %d  z %f", mission_item.command, mission_item.current,
    //     mission_item.frame, mission_item.mission_type, mission_item.x, mission_item.y, mission_item.z);

    send(message);
}

struct Quaternion {
    float w;
    float x;
    float y;
    float z;
};

void rotationRollPitchYaw(float roll, float pitch, float yaw, Quaternion* quaternion)
{
    const float halfRoll = roll * 0.5f;
    const float halfPitch = pitch * 0.5f;
    const float halfYaw = yaw * 0.5f;

    const float sinRoll = sinf(halfRoll);
    const float cosRoll = cosf(halfRoll);
    const float sinPitch = sinf(halfPitch);
    const float cosPitch = cosf(halfPitch);
    const float sinYaw = sinf(halfYaw);
    const float cosYaw = cosf(halfYaw);

    quaternion->w = (cosYaw * cosPitch * cosRoll) + (sinYaw * sinPitch * sinRoll);
    quaternion->x = (cosYaw * cosPitch * sinRoll) - (sinYaw * sinPitch * cosRoll);
    quaternion->y = (cosYaw * sinPitch * cosRoll) + (sinYaw * cosPitch * sinRoll);
    quaternion->z = (sinYaw * cosPitch * cosRoll) - (cosYaw * sinPitch * sinRoll);
}

void MavlinkProvider::sendAttitude(float roll, float pitch, float yaw, float throttle)
{
    uint8_t type_mask = ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE; 

    if (math_utils::isUndefined(roll))
        roll = 0;
    else
        type_mask |= ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE;

    if (math_utils::isUndefined(pitch))
        pitch = 0;
    else
        type_mask |= ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE;

    if (math_utils::isUndefined(yaw))
        yaw = 0;
    else
        type_mask |= ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE;

    if (math_utils::isUndefined(throttle))
        throttle = 0;
    else
        type_mask |= ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE;

    if (type_mask == ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE)
        return;


    type_mask = 0xFF & (~type_mask);

    Quaternion quaternion;
    rotationRollPitchYaw(roll, pitch, yaw, &quaternion);

    float q[4] = {0};
    q[0] = quaternion.w;
    q[1] = quaternion.x;
    q[2] = quaternion.y;
    q[3] = quaternion.z;

    //LOG_F(INFO, "send ATTITUDE: %0.1f %0.1f %0.1f  %.3f %.3f %.3f %.3f mask: %d(%d)", toDeg(roll), toDeg(pitch), toDeg(yaw), q[0], q[1], q[2], q[3], type_mask, ~type_mask);

    mavlink_message_t message;

    mavlink_msg_set_attitude_target_pack(compSystemId, compComponentId, &message, 1000, plane_system_id, plane_component_id,
        type_mask, q, 0, 0, 0, throttle, q);

    send(message);
}

void MavlinkProvider::sendStatustext(MAV_SEVERITY severity, const std::string& text)
{
    mavlink_message_t message;
    char _text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];

    int id = 0;
    int left = text.length();
    int chunk_seq = 0;
    while (left > 0) {
        if (left <= MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN) {
            memset(_text, 0, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1);
            memcpy(_text, text.c_str() + chunk_seq * MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN, left);
            logInfo("send: %s  size: %d", _text, left);
            mavlink_msg_statustext_pack(compSystemId, compComponentId, &message, severity, _text, id, chunk_seq);
        }
        else {
            memcpy(_text, text.c_str() + chunk_seq * MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
            logInfo("send: %s  size: %d", _text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
            mavlink_msg_statustext_pack(compSystemId, compComponentId, &message, severity, _text, id, chunk_seq);
        }

        send(message);

        left -= MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN;
        ++chunk_seq;
    }
}
