#ifndef __GPS_ON_OFF_SWITCHER_H__
#define __GPS_ON_OFF_SWITCHER_H__

#include "mav/mavlink_provider.h"
#include "mav/param_setter.h"
#include "utils/time_trigger.h"

class GPSOnOffSwitcher: public IMavlinkReceiver
{
public:
    GPSOnOffSwitcher(MavlinkProvider* mavlinkProvider);
    ~GPSOnOffSwitcher();

    void init();
    void loop();

    static const int EK3_SRC_GPS = 1;
    static const int EK3_SRC_NO_GPS = 2;

    // ArduPilot RCx_OPTION used to select the active EKF source set via aux function.
    // Switch position low/middle/high maps to source set 1/2/3.
    static const int EKF_SOURCE_AUX_FUNCTION = 90;

protected:
    MavlinkProvider* const mavlinkProvider;

public:
    virtual void onMessageReceived(const mavlink_message_t& message);
    bool isFinished() const { return gpsRequestedType == GPS_REQUEST_TYPE_NONE; }

    // Active EKF3 source set as last reported by the plane via STATUSTEXT
    // ("Using EKF Source Set N"). -1 = unknown / not yet detected.
    int currentEkfSourceSet = -1;
    int getCurrentEkfSourceSet() const { return currentEkfSourceSet; }
    bool isEkfSourceGps() const { return currentEkfSourceSet == EK3_SRC_GPS; }
    bool isEkfSourceNoGps() const { return currentEkfSourceSet == EK3_SRC_NO_GPS; }

public:
    enum GpsRequestType {
        GPS_REQUEST_TYPE_NONE = 0,
        GPS_REQUEST_TYPE_ON,
        GPS_REQUEST_TYPE_OFF,
    };

    enum GpsRequestResult {
        GPS_REQUEST_RESULT_NA = 0,
        GPS_REQUEST_RESULT_SUCCESS = 0x1,
        GPS_REQUEST_RESULT_ON = 0x2,
        GPS_REQUEST_RESULT_OFF = 0x4,
        GPS_REQUEST_RESULT_UNDEFINED = GPS_REQUEST_RESULT_ON | GPS_REQUEST_RESULT_OFF,
        GPS_REQUEST_RESULT_IN_PROGRESS = 0x8,
        GPS_REQUEST_RESULT_FAILED = 0x10,
        //GPS_REQUEST_RESULT_TIMEOUT = 0x20,
        GPS_REQUEST_RESULT_ON_SUCCESS = GPS_REQUEST_RESULT_ON | GPS_REQUEST_RESULT_SUCCESS,
        GPS_REQUEST_RESULT_ON_FAILED = GPS_REQUEST_RESULT_ON | GPS_REQUEST_RESULT_FAILED,
        GPS_REQUEST_RESULT_ON_IN_PROGRESS = GPS_REQUEST_RESULT_ON | GPS_REQUEST_RESULT_IN_PROGRESS,
        //GPS_REQUEST_RESULT_ON_TIMEOUT = GPS_REQUEST_RESULT_ON | GPS_REQUEST_RESULT_TIMEOUT,
        GPS_REQUEST_RESULT_OFF_SUCCESS = GPS_REQUEST_RESULT_OFF | GPS_REQUEST_RESULT_SUCCESS,
        GPS_REQUEST_RESULT_OFF_FAILED = GPS_REQUEST_RESULT_OFF | GPS_REQUEST_RESULT_FAILED,
        GPS_REQUEST_RESULT_OFF_IN_PROGRESS = GPS_REQUEST_RESULT_OFF | GPS_REQUEST_RESULT_IN_PROGRESS,
        //GPS_REQUEST_RESULT_OFF_TIMEOUT = GPS_REQUEST_RESULT_OFF | GPS_REQUEST_RESULT_TIMEOUT,
    };

    void requestGpsOn();
    void requestGpsOff();
    GpsRequestResult getGpsRequestResult() const { return gpsRequestResult; }

protected:
    GpsRequestType gpsRequestedType = GPS_REQUEST_TYPE_NONE;
    GpsRequestResult gpsRequestResult = GPS_REQUEST_RESULT_NA;
    ParamSetter gpsOnOff_AHRS_GPS_USE_Setter;
    TimeTrigger gpsOnOff_EK3_SOURCE_SET_Trigger;
    void doGpsOn();
    void doGpsOff();

protected:
    void onStatusTextReceived(const mavlink_message_t& message);
    void onEkfSourceSetDetected(int sourceSet);
};

#endif
