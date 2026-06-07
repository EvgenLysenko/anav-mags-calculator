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
    bool isFinished() const { return gpsRequestType == GPS_REQUEST_TYPE_NONE; }

public:
    enum GpsRequestType {
        GPS_REQUEST_TYPE_NONE = 0,
        GPS_REQUEST_TYPE_ON,
        GPS_REQUEST_TYPE_OFF,
        // GPS_REQUEST_TYPE_ON_AUX,
        // GPS_REQUEST_TYPE_OFF_AUX,
    };

    GpsRequestType gpsRequestType = GPS_REQUEST_TYPE_NONE;
    ParamSetter gpsOnOff_AHRS_GPS_USE_Setter;
    TimeTrigger gpsOnOff_EK3_SOURCE_SET_Trigger;
    void requestGpsOn();
    void requestGpsOff();
    void doGpsOn();
    void doGpsOff();
};

#endif
