#ifndef __GPS_INPUT_TASK_H__
#define __GPS_INPUT_TASK_H__

#include "task_base.h"
#include "connection/connection.h"
#include "mav/mavlink_provider.h"

class CsvParser;

// Parses NMEA sentences from a uBlox NEO-M8N GPS (connected to the companion
// computer, e.g. via UART7) and injects the position/velocity into the flight
// controller as a GPS_INPUT message (a separate GPS instance).
//
// Runs on its own thread: it owns the serial connection, frames NMEA sentences,
// parses GGA (fix/position/altitude/sats/HDOP) and RMC (position/velocity), and
// forwards a GPS_INPUT to the autopilot on every GGA sentence.
class GpsInputTask: public TaskBase
{
public:
    GpsInputTask(Connection* connection, MavlinkProvider* mavlinkProvider);

    virtual const char* getTaskName() const { return "GpsInputTask"; }

    void init();

protected:
    Connection* const connection;
    MavlinkProvider* const mavlinkProvider;

    // GPS instance the data is injected as (GPS_INPUT.gps_id). 0 = first GPS,
    // 1 = second GPS. The companion GPS is normally the second instance.
    int gpsId = 1;

    // uBlox auto-configuration (UBX) applied on every (re)connect.
    bool configureUbloxEnabled = true;
    bool gpsOnly = true;        // GPS constellation only (needed for 10 Hz on M8N)
    int rateHz = 10;            // module NMEA output rate
    int targetBaud = 115200;    // operating serial baud after UBX-CFG-PRT
    int initialBaud = 9600;     // module baud at first connect (factory default)
    bool configured = false;    // config already sent on the current connection

    // GPS_INPUT is injected on a fixed timer (decoupled from NMEA arrival) so the
    // frame cadence stays above ArduPilot's ~5 Hz health threshold even if the
    // module runs at exactly 5 Hz. The latest parsed fix is (re)sent each tick.
    int sendRateHz = 10;
    long sendPeriodMs = 125;
    long lastSendTime = 0;

    bool gpsDataReady = false;      // at least one GGA parsed
    long lastNmeaUpdateTime = 0;    // wall time of the last GGA update (staleness guard)
    uint16_t lastSentWeek = 0;      // monotonic guard for injected GPS time
    uint32_t lastSentTimeMs = 0;

    virtual void onStarted();
    virtual __useconds_t loop(__useconds_t default_timeout);

    // Send a UBX configuration frame to the module (UBX-CFG-PRT / GNSS / MSG / RATE)
    // to set baud, GPS-only mode, NMEA output, and navigation rate.
    void configureUblox();
    void sendUbx(uint8_t msgClass, uint8_t msgId, const uint8_t* payload, uint16_t length);
    void flushRead();

    // --- NMEA line framing ---
    static const int NMEA_LINE_MAX = 256;
    unsigned char readBuf[1024];
    char line[NMEA_LINE_MAX];
    int lineLength = 0;
    bool collecting = false;

    void processRead();
    void onSentence(const char* sentence, int length);

    // --- NMEA parsing ---
    void parseGga(CsvParser& parser);
    void parseRmc(CsvParser& parser);
    void sendGpsInput();

    // GPS time of the latest fix advanced to "now" (keeps the injected time
    // monotonic between real NMEA updates).
    void currentGpsTime(uint16_t& week, uint32_t& timeMs);

    struct GpsState {
        int32_t lat = 0;        // degE7
        int32_t lon = 0;        // degE7
        float alt = 0;          // m (MSL)
        int fixType = 0;        // GPS_INPUT fix_type (0/1 = no fix, 3 = 3D, ...)
        int satellites = 0;
        float hdop = 0;
        float vn = 0;           // m/s, North
        float ve = 0;           // m/s, East
        bool hasPosition = false;
        bool hasVelocity = false;

        // UTC date (from RMC) + GPS time of the latest fix.
        int year = 0;           // full year, e.g. 2026
        int month = 0;          // 1..12
        int day = 0;            // 1..31
        bool hasDate = false;
        uint16_t gpsWeek = 0;   // base GPS week of the latest fix
        uint32_t gpsTimeMs = 0; // base time of week (ms) of the latest fix
        long fixWallTimeMs = 0; // local time when the base GPS time was set
        bool hasGpsTime = false;
    } gpsState;

    long lastConnectTryTime = 0;
};

#endif
