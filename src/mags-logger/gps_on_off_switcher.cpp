#include "gps_on_off_switcher.h"
#include "logging.h"
#include "config.h"

const char* AHRS_GPS_USE_PARAM_NAME = "AHRS_GPS_USE";
const int GPS_ON_OFF_REPEAT_COUNT = 5;
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
}

void GPSOnOffSwitcher::loop()
{
    if (isFinished())
        return;

    gpsOnOff_AHRS_GPS_USE_Setter.loop();
    //gpsOnOff_EK3_SOURCE_SET_Trigger.loop();

    if (gpsRequestType != GPS_REQUEST_TYPE_NONE) {
        if (gpsOnOff_EK3_SOURCE_SET_Trigger.isFired()) {
            if (gpsRequestType == GPS_REQUEST_TYPE_ON) {
                doGpsOn();
            }
            else if (gpsRequestType == GPS_REQUEST_TYPE_OFF) {  
                doGpsOff();
            }
        }

        if (gpsOnOff_AHRS_GPS_USE_Setter.isFinished() && gpsOnOff_EK3_SOURCE_SET_Trigger.isFinished()) {// && gpsOnOff_EK3_SOURCE_SET_Requester.isFinished()) {
            gpsRequestType = GPS_REQUEST_TYPE_NONE;

            // TODO check if the command was successful
        }
    }
}

void GPSOnOffSwitcher::requestGpsOn()
{
    logInfo("GPS Swither - GPS ON");
    gpsRequestType = GPS_REQUEST_TYPE_ON;
    gpsOnOff_AHRS_GPS_USE_Setter.setParam(AHRS_GPS_USE_PARAM_NAME, 1.0f, MAV_PARAM_TYPE_UINT8, GPS_ON_OFF_REPEAT_TIMEOUT, GPS_ON_OFF_REPEAT_COUNT, gpsRequestType);
    gpsOnOff_AHRS_GPS_USE_Setter.doNow();

    gpsOnOff_EK3_SOURCE_SET_Trigger.start(GPS_ON_OFF_REPEAT_TIMEOUT, GPS_ON_OFF_REPEAT_COUNT);
    doGpsOn();
}

void GPSOnOffSwitcher::requestGpsOff()
{
    logInfo("GPS Swither - GPS OFF");
    gpsRequestType = GPS_REQUEST_TYPE_OFF;

    gpsOnOff_AHRS_GPS_USE_Setter.setParam(AHRS_GPS_USE_PARAM_NAME, 0.0f, MAV_PARAM_TYPE_UINT8, GPS_ON_OFF_REPEAT_TIMEOUT, GPS_ON_OFF_REPEAT_COUNT, gpsRequestType);
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
