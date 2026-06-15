#include "gps_source_switcher.h"
#include "logging.h"
#include <cstring>
#include <cmath>

const bool _GPS_TEST_ENABLED = true;

static const char* GPS_AUTO_SWITCH_PARAM_NAME = "GPS_AUTO_SWITCH";
static const char* GPS_PRIMARY_PARAM_NAME = "GPS_PRIMARY";

static const long GPS_SOURCE_REPEAT_TIMEOUT = 1000;
static const int GPS_SOURCE_REPEAT_COUNT = 7;

// How often to read back the active GPS selection from the autopilot.
static const long GPS_SOURCE_REFRESH_PERIOD = 5000;

GpsSourceSwitcher::GpsSourceSwitcher(MavlinkProvider* mavlinkProvider):
    mavlinkProvider(mavlinkProvider),
    gpsAutoSwitchSetter(mavlinkProvider),
    gpsPrimarySetter(mavlinkProvider)
{
}

GpsSourceSwitcher::~GpsSourceSwitcher()
{
}

void GpsSourceSwitcher::init()
{
    logInfo("GPS Source Switcher - init: FC primary=%d  external primary=%d", GPS_PRIMARY_FC, GPS_PRIMARY_EXTERNAL);
    refreshTrigger.start(GPS_SOURCE_REFRESH_PERIOD);
}

int GpsSourceSwitcher::getCurrentPrimary() const
{
    // GPS_PRIMARY only determines the active GPS when auto switchover is disabled.
    if (reportedAutoSwitch == GPS_AUTO_SWITCH_USE_PRIMARY)
        return reportedPrimary;

    return -1;  // unknown or autopilot is auto-selecting the GPS
}

void GpsSourceSwitcher::onMessageReceived(const mavlink_message_t& message)
{
    gpsAutoSwitchSetter.onMessageReceived(message);
    gpsPrimarySetter.onMessageReceived(message);

    switch (message.msgid) {
    case MAVLINK_MSG_ID_PARAM_VALUE:
        onParamValue(message);
        break;
    case MAVLINK_MSG_ID_GPS_RAW_INT:
        on_msg_gps_raw_received(&message);
        break;
    }
}

void GpsSourceSwitcher::onParamValue(const mavlink_message_t& message)
{
    mavlink_param_value_t param_value;
    mavlink_msg_param_value_decode(&message, &param_value);

    // param_id is a 16-byte field that is not guaranteed to be null-terminated.
    char paramId[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN + 1] = {0};
    memcpy(paramId, param_value.param_id, MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN);

    const int value = (int)lroundf(param_value.param_value);

    if (strcmp(paramId, GPS_PRIMARY_PARAM_NAME) == 0) {
        if (value != reportedPrimary)
            logInfo("GPS Source Switcher - GPS_PRIMARY reported: %d", value);
        reportedPrimary = value;
    }
    else if (strcmp(paramId, GPS_AUTO_SWITCH_PARAM_NAME) == 0) {
        if (value != reportedAutoSwitch)
            logInfo("GPS Source Switcher - GPS_AUTO_SWITCH reported: %d", value);
        reportedAutoSwitch = value;
    }
}

void GpsSourceSwitcher::loop()
{
    // Periodically read back the active GPS selection so the reported primary
    // stays accurate even if it is changed from outside this companion.
    if (refreshTrigger.isFired() && mavlinkProvider->isPlaneReady()) {
        mavlinkProvider->sendParamRequest(GPS_PRIMARY_PARAM_NAME);
        mavlinkProvider->sendParamRequest(GPS_AUTO_SWITCH_PARAM_NAME);
    }

    if (isFinished())
        return;

    gpsAutoSwitchSetter.loop();
    gpsPrimarySetter.loop();

    if (gpsAutoSwitchSetter.isFinished() && gpsPrimarySetter.isFinished()) {
        logInfo("GPS Source Switcher - switch finished: source %d  reported primary %d  auto_switch %d",
            (int)requestedSource, reportedPrimary, reportedAutoSwitch);
        requestedSource = GPS_SOURCE_NONE;
    }

    static bool gpsRawIntRequested = false;
    if (!gpsRawIntRequested && mavlinkProvider->isPlaneReady()) {
        gpsRawIntRequested = true;
        mavlinkProvider->sendRequestMessageInterval(MAVLINK_MSG_ID_GPS_RAW_INT, 200000);
        logInfo("GPS Source Switcher - requested GPS_RAW_INT interval");
   }
}

void GpsSourceSwitcher::requestGpsFc()
{
    logInfo("GPS Source Switcher - request FC GPS (primary %d)", GPS_PRIMARY_FC);
    requestGpsSource(GPS_SOURCE_FC, GPS_PRIMARY_FC);
}

void GpsSourceSwitcher::requestGpsExternal()
{
    logInfo("GPS Source Switcher - request external GPS (primary %d)", GPS_PRIMARY_EXTERNAL);
    requestGpsSource(GPS_SOURCE_EXTERNAL, GPS_PRIMARY_EXTERNAL);
}

void GpsSourceSwitcher::requestGpsSource(GpsSource source, int primary)
{
    requestedSource = source;

    // Disable automatic GPS switchover so GPS_PRIMARY is honoured.
    gpsAutoSwitchSetter.setParam(GPS_AUTO_SWITCH_PARAM_NAME, (float)GPS_AUTO_SWITCH_USE_PRIMARY, MAV_PARAM_TYPE_INT8, GPS_SOURCE_REPEAT_TIMEOUT, GPS_SOURCE_REPEAT_COUNT, source);
    gpsAutoSwitchSetter.doNow();

    gpsPrimarySetter.setParam(GPS_PRIMARY_PARAM_NAME, (float)primary, MAV_PARAM_TYPE_INT8, GPS_SOURCE_REPEAT_TIMEOUT, GPS_SOURCE_REPEAT_COUNT, source);
    gpsPrimarySetter.doNow();
}

void GpsSourceSwitcher::on_msg_gps_raw_received(const mavlink_message_t* message)
{
    //const uint8_t fix_type = mavlink_msg_gps_raw_int_get_fix_type(message);
    mavlink_gps_raw_int_t gps_raw_int;
    mavlink_msg_gps_raw_int_decode(message, &gps_raw_int);

    if (_GPS_TEST_ENABLED) {
        mavlink_gps_input_t gps_input = {0};
        gps_input.time_usec = gps_raw_int.time_usec;
        gps_input.gps_id = 1;
        gps_input.fix_type = gps_raw_int.fix_type;
        gps_input.lat = gps_raw_int.lat + 700;
        gps_input.lon = gps_raw_int.lon + 1000;
        gps_input.alt = gps_raw_int.alt * .001f;
        gps_input.satellites_visible = gps_raw_int.satellites_visible;
        gps_input.yaw = gps_raw_int.yaw;  // TODO: get yaw from gps_raw_int
        mavlink_message_t msg;
        mavlink_msg_gps_input_encode(mavlinkProvider->getCompSystemId(), mavlinkProvider->getCompComponentId(), &msg, &gps_input);
        mavlinkProvider->send(msg);
        logInfo("GPS Source Switcher - sent GPS_INPUT: to %d/%d  gps_id: %d  fix: %d  lat: %d  lon: %d  alt: %.1f  sats: %d",
            mavlinkProvider->getCompSystemId(), mavlinkProvider->getCompComponentId(), 1, gps_raw_int.fix_type, gps_raw_int.lat, gps_raw_int.lon, gps_raw_int.alt * .001f, gps_raw_int.satellites_visible);
    }
}
