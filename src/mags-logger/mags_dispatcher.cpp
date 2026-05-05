#include "mags_logger.h"
#include "logging.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include "utils/time_utils.h"
#include "mav/mav_utils.h"
#include "ip.h"

const static MAV_CMD MAGS_COMMAND_LONG_ID = MAV_CMD::MAV_CMD_USER_2;

using namespace std;

static const long CCR_SET_REQUEST_PERIOD = 1000;

static const int STATUS_BIT_MAGS_ONLINE = 0x1;
static const int STATUS_BIT_GPS_ONLINE = 0x2;
static const int STATUS_BIT_LOG_STARTED = 0x4;
static const int STATUS_BIT_OUT_MAGS = 0x8;
static const int STATUS_BIT_OUT_ACCEL = 0x10;
static const int STATUS_BIT_FULL_TRACE_ENEBLED = 0x20;
static const int STATUS_BIT_DEBUG_ENABLED = 0x40;

void MagsLogger::onMessageReceived(const mavlink_message_t& message)
{
    switch (message.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
        const uint8_t type = mavlink_msg_heartbeat_get_type(&message);
        this->plane_base_mode = mavlink_msg_heartbeat_get_base_mode(&message);
        const uint8_t autopilot = mavlink_msg_heartbeat_get_autopilot(&message);
        if (type == MAV_TYPE::MAV_TYPE_FIXED_WING && autopilot != MAV_AUTOPILOT::MAV_AUTOPILOT_INVALID) {
            const FlightMode flightMode = (FlightMode)mavlink_msg_heartbeat_get_custom_mode(&message);
            if (flightMode != this->flightMode) {
                this->flightMode = flightMode;
                logInfo("Plane - FlightMode: %d", (int)flightMode);
            }
        }
        break;
    }
    case MAVLINK_MSG_ID_SYSTEM_TIME:
        on_msg_system_time_receved(&message);
        break;
    case MAVLINK_MSG_ID_GPS_RAW_INT:
        on_msg_gps_raw_received(&message);
        break;
    case MAVLINK_MSG_ID_ATTITUDE:
        on_msg_attitude_received(&message);
        break;
    case MAVLINK_MSG_ID_COMMAND_LONG:
        on_command_long_received(&message);
        break;
}
}

void MagsLogger::on_msg_system_time_receved(const mavlink_message_t* message)
{
    const unsigned long unixTime = (unsigned long)(mavlink_msg_system_time_get_time_unix_usec(message) / 1000000);

    if (!gpsSystemUnuxTime && unixTime) {
        logInfo("Mags - system time receved: %lu %lu", mavlink_msg_system_time_get_time_unix_usec(message), unixTime);

        const time_t _time = unixTime;
        const tm* t = gmtime(&_time);
        if (t) {
            logInfo("Mags - system time receved: %04d-%02d-%02d %02d:%02d:%02d", 1900 + t->tm_year, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
        }
    }

    gpsSystemUnuxTime = unixTime;
}

void MagsLogger::on_msg_gps_raw_received(const mavlink_message_t* message)
{
    const uint8_t fix_type = mavlink_msg_gps_raw_int_get_fix_type(message);
    if (fix_type > GPS_FIX_TYPE_NO_FIX) {
        if (!gpsFixed) {
            LOG_F(INFO, "Mags - GPS FIXED");
            gpsFixed = true;
        }

        position.lat = MavUtils::int_to_deg(mavlink_msg_gps_raw_int_get_lat(message));
        position.lon = MavUtils::int_to_deg(mavlink_msg_gps_raw_int_get_lon(message));
        position.alt = (float)mavlink_msg_gps_raw_int_get_alt(message) * .001f;

        positionReceived = true;
        gpsReceivedTime = TimeUtils::getTime();
        gpsCounter.add();
    }
}

void MagsLogger::on_msg_attitude_received(const mavlink_message_t* message)
{
    attitude.roll = MavUtils::rad_to_deg(mavlink_msg_attitude_get_roll(message));
    attitude.pitch = MavUtils::rad_to_deg(mavlink_msg_attitude_get_pitch(message));
    attitude.yaw = MavUtils::rad_to_deg(mavlink_msg_attitude_get_yaw(message));

    attitudeReceivedTime = TimeUtils::getTime();

    attitudeCounter.add();
}

void MagsLogger::on_command_long_received(const mavlink_message_t* message)
{
    const MAV_CMD command = static_cast<MAV_CMD>(mavlink_msg_command_long_get_command(message));

    if (command == MAGS_COMMAND_LONG_ID) {
        mavlinkProvider->sendCommandAck(message->sysid, message->compid, MAV_RESULT_ACCEPTED, 0);

        const MagsCommandId magCommandId = static_cast<MagsCommandId>(mavlink_msg_command_long_get_param1(message) + 0.5f);
        switch (magCommandId) {
        case MAGS_CCR_SET: {
            const int ccr = static_cast<int>(mavlink_msg_command_long_get_param2(message) + 0.5f);
            LOG_F(INFO, "Mags - on_command_long received: MAGS_CCR_SET: %d", ccr);
            onCcrSet(ccr);
            break;
        }
        case MAGS_SETTINGS_REQUEST: {
            LOG_F(INFO, "Mags - on_command_long received: MAGS_SETTINGS_REQUEST");
            sendSettings();
            break;
        }
        case MAGS_LOGGING_START: {
            LOG_F(INFO, "Mags - on_command_long received: MAGS_LOGGING_START");
            startLogging();
            break;
        }
        case MAGS_LOGGING_STOP: {
            LOG_F(INFO, "Mags - on_command_long received: MAGS_LOGGING_STOP");
            onLoggingStop();
            break;
        }
        case MAGS_SET_OUT_MAGS: {
            LOG_F(INFO, "Mags - on_command_long received: MAGS_SET_OUT_MAGS");
            onSwitchToMags();
            break;
        }
        case MAGS_SET_OUT_ACCEL: {
            LOG_F(INFO, "Mags - on_command_long received: MAGS_SET_OUT_ACCEL");
            onSwitchToAccel();
            break;
        }
        case MAGS_FULL_TRACE_ENABLE: {
            MagsLogger::MAGS_FULL_TRACE_ENABLED = static_cast<int>(mavlink_msg_command_long_get_param2(message) + 0.5f) > 0;
            LOG_F(INFO, "Mags - on_command_long received: MAGS_FULL_TRACE_ENABLE: %d", MagsLogger::MAGS_FULL_TRACE_ENABLED ? 1 : 0);
            break;
        }
        case MAGS_DEBUG_ENABLE: {
            MagsLogger::DEBUG_OUT_ENABLED = static_cast<int>(mavlink_msg_command_long_get_param2(message) + 0.5f) > 0;
            LOG_F(INFO, "Mags - on_command_long received: MAGS_DEBUG_ENABLE: %d", MagsLogger::DEBUG_OUT_ENABLED ? 1 : 0);
            break;
        }
        case MAGS_IP: {
            ipRequested = true;
            LOG_F(INFO, "Mags - on_command_long received: MAGS_IP");
            break;
        }
        default:
            LOG_F(INFO, "Mags - on_command_long received: %d", command);
            break;
        }
    }
}

void MagsLogger::sendCommand(MagsCommandId magsCommandId, float param1, float param2, float param3, float param4, float param5, float param6)
{
    mavlinkProvider->sendCommandLong(mavlinkProvider->getGscSystemId(), mavlinkProvider->getGscComponentId(),
        MAGS_COMMAND_LONG_ID, 0, magsCommandId, param1, param2, param3, param4, param5, param6);
}

void MagsLogger::sendStatus()
{
    const int status = 0 |
        (magsReceivedCounter.count > 0 ? STATUS_BIT_MAGS_ONLINE : 0) |
        (gpsFixed ? STATUS_BIT_GPS_ONLINE : 0) |
        (logStarted ? STATUS_BIT_LOG_STARTED : 0) |
        (outMagsDetected ? STATUS_BIT_OUT_MAGS : 0) |
        (outAccelDetected ? STATUS_BIT_OUT_ACCEL : 0) |
        (MagsLogger::MAGS_FULL_TRACE_ENABLED ? STATUS_BIT_FULL_TRACE_ENEBLED : 0) |
        (MagsLogger::DEBUG_OUT_ENABLED ? STATUS_BIT_DEBUG_ENABLED : 0)
    ;

    const int logoutTime = (TimeUtils::getTime() - logStartedTime) / 1000;

    sendCommand(MAGS_STATUS, magsLogoutCounter.count, logoutTime, magsReceivedCounter.count, (gpsFixed ? (float)gpsCounter.count : (float)-1), attitudeCounter.count, status);
    logInfo("Mags - STATUS: mags: %d  log fps: %d  gps fix: %d  gps: %d  att: %d  out mags: %d  accel: %d", magsReceivedCounter.count, magsLogoutCounter.count, gpsFixed, gpsCounter.count, attitudeCounter.count,
        (outMagsDetected ? 1 : 0), (outAccelDetected ? 1 : 0));
}

void MagsLogger::sendSensorStatus()
{
    const int ParamsMaxNumber = 4;
    int online[ParamsMaxNumber] = {0};
    int magIdx = 0;
    for (const Mag& mag: mags) {
        if (magIdx < ParamsMaxNumber) {
            online[magIdx] = mag.online;
        }
        else
            online[magIdx] = -1;

        ++magIdx;
    }

    for (; magIdx < ParamsMaxNumber; ++magIdx) {
        online[magIdx] = -1;
    }

    sendCommand(MAGS_SENSORS_STATUS, 0, online[0], online[1], online[2], online[3]);
    logInfo("Mags - SENSORS: idx: %d  online: %d, %d, %d, %d", 0, online[0], online[1], online[2], online[3]);
}

void MagsLogger::sendMagsValues()
{
    sendCommand(MAGS_MAGS_VALUES, 
        (mags.size() > 0 ? mags[0].x : 0), (mags.size() > 0 ? mags[0].y : 0), (mags.size() > 0 ? mags[0].z : 0),
        (mags.size() > 1 ? mags[1].x : 0), (mags.size() > 1 ? mags[1].y : 0), (mags.size() > 1 ? mags[1].z : 0)
    );

    logInfo("Mags - SEND mag1[%c]: %5d %5d %5d   mag2[%c]: %5d %5d %5d",
        mags.size() > 0 ? '+' : '-', mags.size() > 0 ? (int)mags[0].x : 0, mags.size() > 0 ? (int)mags[0].y : 0, mags.size() > 0 ? (int)mags[0].z : 0,
        mags.size() > 0 ? '+' : '-', mags.size() > 1 ? (int)mags[1].x : 0, mags.size() > 1 ? (int)mags[1].y : 0, mags.size() > 1 ? (int)mags[1].z : 0
    );
}

void MagsLogger::sendAccelValues()
{
    sendCommand(MAGS_ACCEL_VALUES, accel.x, accel.y, accel.z, accelMag.x, accelMag.y, accelMag.z);

    logInfo("Mags - SEND Accel: %5d %5d %5d   Mag: %5d %5d %5d", accel.x, accel.y, accel.z, accelMag.x, accelMag.y, accelMag.z);
}

void MagsLogger::sendSettings()
{
    logInfo("Mags - SETTINGS: count: %d  CCR: %d", (int)mags.size(), ccrReceived);
    sendCommand(MAGS_SETTINGS, mags.size(), ccrReceived);
}

void MagsLogger::sendIp()
{
    ip = getIp();

    logInfo("Mags - send IP: ip: %d.%d.%d.%d", (int)(ip & 0xFF), (int)((ip >> 8) & 0xFF), (int)((ip >> 16) & 0xFF), (int)((ip >> 24) & 0xFF));
    sendCommand(MAGS_IP, ip);
}

void MagsLogger::onLoggingStop()
{
    logStarted = false;
    mavlinkProvider->sendStatustext(MAV_SEVERITY_INFO, "Mags - log stopped");

    sendSensorStatus();
    sendSettings();
}

void MagsLogger::onSwitchToMags()
{
    LOG_F(INFO, "Mags - set out: MAGS");

    outSetCommandRequester.sendCommand("$MAG,OUT,ACCEL,DISABLE\n$MAG,OUT,MAGS,ENABLE\n\0", 5, 1000, MAGS_SET_OUT_MAGS);
}

void MagsLogger::onSwitchToAccel()
{
    LOG_F(INFO, "Mags - set out: ACCEL");

    outSetCommandRequester.sendCommand("$MAG,OUT,MAGS,DISABLE\n$MAG,OUT,ACCEL,ENABLE\n", 5, 1000, MAGS_SET_OUT_ACCEL);
}
