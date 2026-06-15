#ifndef __GPS_SOURCE_SWITCHER_H__
#define __GPS_SOURCE_SWITCHER_H__

#include "mav/mavlink_provider.h"
#include "mav/param_setter.h"
#include "utils/time_trigger.h"

// Manually selects which GPS receiver the autopilot uses:
//  - FC GPS:        connected directly to the flight controller (GPS instance 0)
//  - External GPS:  connected to the companion computer and injected over MAVLink
//                   as the second GPS instance (instance 1)
//
// Selection uses ArduPilot's manual GPS selection:
//   GPS_AUTO_SWITCH = 0 (use primary) and GPS_PRIMARY = 0 (FC) or 1 (external).
// With GPS_AUTO_SWITCH = 0 the autopilot uses GPS_PRIMARY as the active instance.
class GpsSourceSwitcher: public IMavlinkReceiver
{
public:
    GpsSourceSwitcher(MavlinkProvider* mavlinkProvider);
    ~GpsSourceSwitcher();

    void init();
    void loop();
    virtual void onMessageReceived(const mavlink_message_t& message);

    // GPS_AUTO_SWITCH value that makes the autopilot honour GPS_PRIMARY.
    static const int GPS_AUTO_SWITCH_USE_PRIMARY = 0;

    // GPS_PRIMARY values (GPS instance index).
    static const int GPS_PRIMARY_FC = 0;        // flight controller GPS
    static const int GPS_PRIMARY_EXTERNAL = 1;  // companion computer GPS

    enum GpsSource {
        GPS_SOURCE_NONE = 0,
        GPS_SOURCE_FC,
        GPS_SOURCE_EXTERNAL,
    };

    // Switch to the flight-controller GPS (instance 0).
    void requestGpsFc();
    // Switch to the external/companion GPS (instance 1).
    void requestGpsExternal();

    bool isFinished() const { return requestedSource == GPS_SOURCE_NONE; }
    GpsSource getRequestedSource() const { return requestedSource; }

    // Active primary GPS instance as reported by the autopilot (-1 = unknown).
    // Only meaningful when automatic switchover is disabled (GPS_AUTO_SWITCH = 0);
    // when auto switch is active the autopilot picks the instance, so -1 is returned.
    int getCurrentPrimary() const;

    // Raw values last reported by the autopilot via PARAM_VALUE (-1 = unknown).
    int getReportedPrimary() const { return reportedPrimary; }
    int getReportedAutoSwitch() const { return reportedAutoSwitch; }

protected:
    MavlinkProvider* const mavlinkProvider;

    GpsSource requestedSource = GPS_SOURCE_NONE;
    int reportedPrimary = -1;
    int reportedAutoSwitch = -1;

    ParamSetter gpsAutoSwitchSetter;
    ParamSetter gpsPrimarySetter;
    TimeTrigger refreshTrigger;

    void requestGpsSource(GpsSource source, int primary);
    void onParamValue(const mavlink_message_t& message);

protected:
    void on_msg_gps_raw_received(const mavlink_message_t* message);
};

#endif
