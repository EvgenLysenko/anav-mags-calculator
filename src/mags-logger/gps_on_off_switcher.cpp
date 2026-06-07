#include "gps_on_off_switcher.h"
#include "logging.h"
#include "config.h"
#include <cstdio>
#include <cstring>

const char* AHRS_GPS_USE_PARAM_NAME = "AHRS_GPS_USE";
const int GPS_ON_OFF_REPEAT_COUNT = 7;
const int GPS_ON_OFF_REPEAT_TIMEOUT = 1000;

GPSOnOffSwitcher::GPSOnOffSwitcher(MavlinkProvider* mavlinkProvider):
    mavlinkProvider(mavlinkProvider),
    gpsOnOff_AHRS_GPS_USE_Setter(mavlinkProvider)
{
}

GPSOnOffSwitcher::~GPSOnOffSwitcher()
{
}

void GPSOnOffSwitcher::init()
{
    //EK3_SRC_GPS = Config::readInt("gps_on_off_switcher/ekf_src_gps", EK3_SRC_GPS);
    //EK3_SRC_NO_GPS = Config::readInt("gps_on_off_switcher/ekf_src_no_gps", EK3_SRC_NO_GPS);
    //EKF_SOURCE_AUX_FUNCTION = Config::readInt("gps_on_off_switcher/ekf_source_aux_function", EKF_SOURCE_AUX_FUNCTION);

    logInfo("GPS Swither - EKF source sets: gps: %d  no_gps: %d", EK3_SRC_GPS, EK3_SRC_NO_GPS);
    logInfo("GPS Swither - EKF source aux function: %d", EKF_SOURCE_AUX_FUNCTION);
}

void GPSOnOffSwitcher::onMessageReceived(const mavlink_message_t& message)
{
    gpsOnOff_AHRS_GPS_USE_Setter.onMessageReceived(message);
    //gpsOnOff_EK3_SOURCE_SET_Requester.onMessageReceived(message);

    if (message.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
        onStatusTextReceived(message);
    }
}

void GPSOnOffSwitcher::onStatusTextReceived(const mavlink_message_t& message)
{
    mavlink_statustext_t statustext;
    mavlink_msg_statustext_decode(&message, &statustext);

    // The text field is not guaranteed to be null-terminated.
    char text[sizeof(statustext.text) + 1] = {0};
    memcpy(text, statustext.text, sizeof(statustext.text));

    // ArduPilot emits "Using EKF Source Set N" (1-based) when the active EKF3
    // source set changes, e.g. via the EKF Source Set aux function we trigger.
    logInfo("GPS Swither - status text received: %s", text);

    int sourceSet = 0;
    if (sscanf(text, "Using EKF Source Set %d", &sourceSet) != 1)
        return;

    onEkfSourceSetDetected(sourceSet);
}

void GPSOnOffSwitcher::onEkfSourceSetDetected(int sourceSet)
{
    currentEkfSourceSet = sourceSet;

    if (sourceSet == EK3_SRC_GPS) {
        logInfo("GPS Swither - detected EKF source set %d (GPS)", sourceSet);
        if (gpsRequestedType == GPS_REQUEST_TYPE_ON) {
            logInfo("GPS Swither - GPS ON request finished");
            gpsOnOff_EK3_SOURCE_SET_Trigger.stop();
        }
    }
    else if (sourceSet == EK3_SRC_NO_GPS) {
        logInfo("GPS Swither - detected EKF source set %d (NO GPS)", sourceSet);
        if (gpsRequestedType == GPS_REQUEST_TYPE_OFF) {
            logInfo("GPS Swither - GPS OFF request finished");
            gpsOnOff_EK3_SOURCE_SET_Trigger.stop();
        }
    }
    else {
        logInfo("GPS Swither - detected EKF source set %d", sourceSet);
    }
}

void GPSOnOffSwitcher::loop()
{
    if (isFinished())
        return;

    gpsOnOff_AHRS_GPS_USE_Setter.loop();
    //gpsOnOff_EK3_SOURCE_SET_Trigger.loop();

    if (gpsRequestedType != GPS_REQUEST_TYPE_NONE) {
        if (gpsOnOff_EK3_SOURCE_SET_Trigger.isFired()) {
            if (gpsRequestedType == GPS_REQUEST_TYPE_ON) {
                doGpsOn();
            }
            else if (gpsRequestedType == GPS_REQUEST_TYPE_OFF) {  
                doGpsOff();
            }
        }

        if (gpsOnOff_AHRS_GPS_USE_Setter.isFinished() && gpsOnOff_EK3_SOURCE_SET_Trigger.isFinished()) {// && gpsOnOff_EK3_SOURCE_SET_Requester.isFinished()) {
            if (gpsRequestedType == GPS_REQUEST_TYPE_ON) {
                if (isEkfSourceGps()) {
                    gpsRequestResult = GPS_REQUEST_RESULT_ON_SUCCESS;
                    logInfo("GPS Swither - GPS ON request finished successfully");
                }
                else {
                    gpsRequestResult = GPS_REQUEST_RESULT_ON_FAILED;
                    logInfo("GPS Swither - GPS ON request failed");
                }
            }
            else if (gpsRequestedType == GPS_REQUEST_TYPE_OFF) {
                if (isEkfSourceNoGps()) {
                    gpsRequestResult = GPS_REQUEST_RESULT_OFF_SUCCESS;
                    logInfo("GPS Swither - GPS OFF request finished successfully");
                }
                else {
                    gpsRequestResult = GPS_REQUEST_RESULT_OFF_FAILED;
                    logInfo("GPS Swither - GPS OFF request failed");
                }
            }
            else {
                gpsRequestResult = static_cast<GpsRequestResult>(GPS_REQUEST_RESULT_UNDEFINED | GPS_REQUEST_RESULT_FAILED);
                logInfo("GPS Swither - GPS %d request failed", (int)gpsRequestedType);
            }

            gpsRequestedType = GPS_REQUEST_TYPE_NONE;
        }
    }
}

void GPSOnOffSwitcher::requestGpsOn()
{
    logInfo("GPS Swither - GPS ON");
    gpsRequestedType = GPS_REQUEST_TYPE_ON;
    gpsRequestResult = GPS_REQUEST_RESULT_ON_IN_PROGRESS;

    gpsOnOff_AHRS_GPS_USE_Setter.setParam(AHRS_GPS_USE_PARAM_NAME, 1.0f, MAV_PARAM_TYPE_UINT8, GPS_ON_OFF_REPEAT_TIMEOUT, GPS_ON_OFF_REPEAT_COUNT, gpsRequestedType);
    gpsOnOff_AHRS_GPS_USE_Setter.doNow();

    gpsOnOff_EK3_SOURCE_SET_Trigger.start(GPS_ON_OFF_REPEAT_TIMEOUT, GPS_ON_OFF_REPEAT_COUNT);
    doGpsOn();
}

void GPSOnOffSwitcher::requestGpsOff()
{
    logInfo("GPS Swither - GPS OFF");
    gpsRequestedType = GPS_REQUEST_TYPE_OFF;
    gpsRequestResult = GPS_REQUEST_RESULT_OFF_IN_PROGRESS;

    gpsOnOff_AHRS_GPS_USE_Setter.setParam(AHRS_GPS_USE_PARAM_NAME, 0.0f, MAV_PARAM_TYPE_UINT8, GPS_ON_OFF_REPEAT_TIMEOUT, GPS_ON_OFF_REPEAT_COUNT, gpsRequestedType);
    gpsOnOff_AHRS_GPS_USE_Setter.doNow();

    gpsOnOff_EK3_SOURCE_SET_Trigger.start(GPS_ON_OFF_REPEAT_TIMEOUT, GPS_ON_OFF_REPEAT_COUNT);
    doGpsOff();
}

void GPSOnOffSwitcher::doGpsOn()
{
    if (!mavlinkProvider->isPlaneReady()) {
        logWarning("GPS Swither - do GPS ON: plane not ready");
        return;
    }

    logInfo("GPS Swither - do GPS ON: EKF source set %d", EK3_SRC_GPS);
    mavlinkProvider->sendDoAuxFunction(EKF_SOURCE_AUX_FUNCTION, EK3_SRC_GPS - 1); // -1 because low position is 0
    mavlinkProvider->sendStatusText(MAV_SEVERITY_INFO, "GPS Swither - GPS enable (EKF SRC)");
}

void GPSOnOffSwitcher::doGpsOff()
{
    if (!mavlinkProvider->isPlaneReady()) {
        logWarning("GPS Swither - do GPS OFF: plane not ready");
        return;
    }

    logInfo("GPS Swither - do GPS OFF: EKF source set %d", EK3_SRC_NO_GPS);
    mavlinkProvider->sendDoAuxFunction(EKF_SOURCE_AUX_FUNCTION, EK3_SRC_NO_GPS - 1); // -1 because low position is 0
    mavlinkProvider->sendStatusText(MAV_SEVERITY_INFO, "GPS Swither - GPS disable (EKF SRC)");
}
