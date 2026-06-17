#include "gps_input_task.h"
#include "logging.h"
#include "config.h"
#include "utils/CsvParser.h"
#include "utils/time_utils.h"
#include "utils/time_trigger.h"
#include <cstring>
#include <cmath>
#include <unistd.h>

static const long GPS_INPUT_LOOP_PERIOD_MS = 10;       // serial poll period
static const long GPS_INPUT_RECONNECT_PERIOD_MS = 2000;

// Stop re-injecting the last fix if the NMEA stream goes quiet for this long, so
// the autopilot sees the GPS drop out instead of a frozen position.
static const long GPS_INPUT_STALE_MS = 1500;
static const long GPS_INPUT_UBX_STEP_DELAY_US = 50000;  // 50 ms between UBX commands

static const float KNOTS_TO_MPS = 0.514444f;
static const float DEG_TO_RAD = 0.0174532925f;

// NMEA latitude/longitude come in (d)ddmm.mmmm form (degrees + minutes).
static int32_t nmeaToDegE7(double nmea, bool negative)
{
    const int degrees = (int)(nmea / 100.0);
    const double minutes = nmea - degrees * 100.0;
    double deg = degrees + minutes / 60.0;

    if (negative)
        deg = -deg;

    return (int32_t)lround(deg * 1e7);
}

// Map an NMEA GGA fix-quality field to a GPS_INPUT fix_type. GGA does not
// distinguish 2D from 3D, so any valid fix is reported as 3D.
static int ggaQualityToFixType(int quality)
{
    switch (quality) {
    case 0:  return 0;   // no fix
    case 1:  return 3;   // GPS fix -> 3D
    case 2:  return 4;   // DGPS
    case 4:  return 5;   // RTK fixed
    case 5:  return 5;   // RTK float
    default: return 3;
    }
}

// Number of days from 1970-01-01 to the given proleptic-Gregorian civil date.
// (Howard Hinnant's days_from_civil algorithm.)
static long daysFromCivil(int y, int m, int d)
{
    y -= m <= 2;
    const long era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (long)doe - 719468;
}

// GPS does not count leap seconds, so GPS = UTC + leap seconds. 18 since 2017-01-01.
static const int GPS_UTC_LEAP_SECONDS = 18;

// Convert a UTC date + UTC seconds-of-day into GPS week number and time of week (ms).
static void utcToGpsTime(int year, int month, int day, double secondsOfDay,
    uint16_t& gpsWeek, uint32_t& gpsTimeMs)
{
    // GPS epoch is 1980-01-06 (a Sunday). daysFromCivil(1980, 1, 6) == 3657.
    static const long GPS_EPOCH_DAYS = 3657;
    const long daysSinceEpoch = daysFromCivil(year, month, day) - GPS_EPOCH_DAYS;

    long week = daysSinceEpoch / 7;
    const long dayOfWeek = daysSinceEpoch % 7;

    double ms = (dayOfWeek * 86400.0 + secondsOfDay + GPS_UTC_LEAP_SECONDS) * 1000.0;

    // leap seconds can nudge the sample past the week boundary - normalize it
    const double MS_PER_WEEK = 604800000.0;
    while (ms >= MS_PER_WEEK) { ms -= MS_PER_WEEK; ++week; }
    while (ms < 0)            { ms += MS_PER_WEEK; --week; }

    gpsWeek = (uint16_t)week;
    gpsTimeMs = (uint32_t)(ms + 0.5);
}

// NMEA UTC time field (hhmmss.ss) -> seconds since midnight.
static double nmeaTimeToSeconds(double hhmmss)
{
    const int hh = (int)(hhmmss / 10000.0);
    const double rem = hhmmss - hh * 10000.0;
    const int mm = (int)(rem / 100.0);
    const double ss = rem - mm * 100.0;
    return hh * 3600.0 + mm * 60.0 + ss;
}

GpsInputTask::GpsInputTask(Connection* connection, MavlinkProvider* mavlinkProvider):
    TaskBase(),
    connection(connection),
    mavlinkProvider(mavlinkProvider)
{
}

void GpsInputTask::init()
{
    gpsId = Config::readInt("gps/gpsId", gpsId);
    configureUbloxEnabled = Config::readBool("gps/configure", configureUbloxEnabled);
    gpsOnly = Config::readBool("gps/gpsOnly", gpsOnly);
    rateHz = Config::readInt("gps/rateHz", rateHz);
    if (rateHz < 1)
        rateHz = 1;

    targetBaud = Config::readInt("gps/connection/port", targetBaud);
    initialBaud = Config::readInt("gps/initialBaud", initialBaud);
    if (initialBaud <= 0)
        initialBaud = targetBaud;

    sendRateHz = Config::readInt("gps/sendRateHz", sendRateHz);
    if (sendRateHz < 1)
        sendRateHz = 1;
    sendPeriodMs = 1000 / sendRateHz;

    logInfo("GPS Input - init: gpsId %d  configure %d  gpsOnly %d  module rate %d Hz  baud %d->%d  inject %d Hz",
        gpsId, (int)configureUbloxEnabled, (int)gpsOnly, rateHz, initialBaud, targetBaud, sendRateHz);
}

void GpsInputTask::onStarted()
{
    logInfo("GPS Input - started");
    init();
}

__useconds_t GpsInputTask::loop(__useconds_t default_timeout)
{
    if (!connection)
        return default_timeout;

    if (!connection->isConnected()) {
        configured = false;
        const long curTime = TimeUtils::getTime();
        if (TimeUtils::isTimeout(curTime, lastConnectTryTime, GPS_INPUT_RECONNECT_PERIOD_MS)) {
            lastConnectTryTime = curTime;
            logWarning("GPS Input - not connected, trying to connect");
            connection->connect();
        }
        return ms2Time(GPS_INPUT_RECONNECT_PERIOD_MS);
    }

    if (configureUbloxEnabled && !configured) {
        configureUblox();
        configured = true;
    }

    processRead();

    // Inject at a fixed cadence (decoupled from NMEA arrival) while the fix is fresh.
    const long now = TimeUtils::getTime();
    if (gpsDataReady &&
        (now - lastNmeaUpdateTime) < GPS_INPUT_STALE_MS &&
        TimeUtils::isTimeout(now, lastSendTime, sendPeriodMs)) {
        lastSendTime = now;
        sendGpsInput();
    }

    return ms2Time(GPS_INPUT_LOOP_PERIOD_MS);
}

void GpsInputTask::processRead()
{
    const int received = connection->read(readBuf, sizeof(readBuf));
    if (received <= 0)
        return;

    for (int i = 0; i < received; ++i) {
        const char ch = (char)readBuf[i];

        if (ch == '$') {
            collecting = true;
            lineLength = 0;
            line[lineLength++] = ch;
        }
        else if (collecting) {
            if (ch == '\r' || ch == '\n') {
                line[lineLength] = 0;
                if (lineLength > 1)
                    onSentence(line, lineLength);
                collecting = false;
                lineLength = 0;
            }
            else if (lineLength < NMEA_LINE_MAX - 1) {
                line[lineLength++] = ch;
            }
            else {
                // sentence too long - drop it and resync on the next '$'
                collecting = false;
                lineLength = 0;
            }
        }
    }
}

// Build a UBX frame (sync chars + header + payload + Fletcher checksum) and write
// it to the GPS serial port.
void GpsInputTask::sendUbx(uint8_t msgClass, uint8_t msgId, const uint8_t* payload, uint16_t length)
{
    unsigned char buf[96];
    int n = 0;

    buf[n++] = 0xB5;
    buf[n++] = 0x62;
    buf[n++] = msgClass;
    buf[n++] = msgId;
    buf[n++] = (unsigned char)(length & 0xFF);
    buf[n++] = (unsigned char)(length >> 8);

    for (uint16_t i = 0; i < length && n < (int)sizeof(buf) - 2; ++i)
        buf[n++] = payload[i];

    // Fletcher-8 checksum over everything after the two sync bytes
    unsigned char ckA = 0;
    unsigned char ckB = 0;
    for (int i = 2; i < n; ++i) {
        ckA += buf[i];
        ckB += ckA;
    }
    buf[n++] = ckA;
    buf[n++] = ckB;

    connection->send(buf, n);
}

void GpsInputTask::flushRead()
{
    for (int i = 0; i < 8; ++i) {
        const int received = connection->read(readBuf, sizeof(readBuf));
        if (received <= 0)
            break;
    }
}

static void appendGnssBlock(uint8_t* payload, int& idx, uint8_t gnssId, bool enabled, uint8_t maxTrkCh)
{
    payload[idx++] = gnssId;
    payload[idx++] = enabled ? 4 : 0;   // resTrkCh
    payload[idx++] = enabled ? maxTrkCh : 0;
    payload[idx++] = 0;                 // reserved1
    const uint32_t flags = enabled ? 1u : 0u;
    payload[idx++] = (uint8_t)(flags & 0xFF);
    payload[idx++] = (uint8_t)((flags >> 8) & 0xFF);
    payload[idx++] = (uint8_t)((flags >> 16) & 0xFF);
    payload[idx++] = (uint8_t)((flags >> 24) & 0xFF);
}

void GpsInputTask::configureUblox()
{
    const uint16_t measRateMs = (uint16_t)(1000 / rateHz);
    const int currentBaud = connection->getPort();

    logInfo("GPS Input - configuring uBlox: %d Hz  gpsOnly %d  baud %d -> %d",
        rateHz, (int)gpsOnly, currentBaud, targetBaud);

    // Step 1: switch module serial baud (UBX-CFG-PRT) if needed.
    if (currentBaud != targetBaud) {
        const uint8_t cfgPrt[20] = {
            0x01, 0x00,             // UART1
            0x00, 0x00,             // txReady
            0xD0, 0x08, 0x00, 0x00, // 8N1
            (uint8_t)(targetBaud & 0xFF),
            (uint8_t)((targetBaud >> 8) & 0xFF),
            (uint8_t)((targetBaud >> 16) & 0xFF),
            (uint8_t)((targetBaud >> 24) & 0xFF),
            0x07, 0x00,             // in: UBX + NMEA + RTCM3
            0x03, 0x00,             // out: UBX + NMEA
            0x00, 0x00,             // flags
            0x00, 0x00,             // reserved
        };
        sendUbx(0x06, 0x00, cfgPrt, sizeof(cfgPrt));
        usleep(GPS_INPUT_UBX_STEP_DELAY_US);

        if (!connection->setPort(targetBaud)) {
            logError("GPS Input - failed to reopen serial at %d baud", targetBaud);
            return;
        }

        usleep(GPS_INPUT_UBX_STEP_DELAY_US);
        flushRead();
    }

    // Step 2: GPS-only constellation (UBX-CFG-GNSS) - required for 10 Hz on M8N.
    if (gpsOnly) {
        uint8_t cfgGnss[64] = {0};
        int idx = 0;
        cfgGnss[idx++] = 0;    // version
        cfgGnss[idx++] = 16;   // numTrkChHw
        cfgGnss[idx++] = 16;   // numTrkChUse
        cfgGnss[idx++] = 7;    // numConfigBlocks
        appendGnssBlock(cfgGnss, idx, 0, true, 8);   // GPS
        appendGnssBlock(cfgGnss, idx, 1, false, 0);  // SBAS
        appendGnssBlock(cfgGnss, idx, 2, false, 0);  // Galileo
        appendGnssBlock(cfgGnss, idx, 3, false, 0);  // BeiDou
        appendGnssBlock(cfgGnss, idx, 4, false, 0);  // IMES
        appendGnssBlock(cfgGnss, idx, 5, false, 0);  // QZSS
        appendGnssBlock(cfgGnss, idx, 6, false, 0);  // GLONASS
        sendUbx(0x06, 0x3E, cfgGnss, (uint16_t)idx);
        usleep(GPS_INPUT_UBX_STEP_DELAY_US);
    }

    // Step 3: trim NMEA output to GGA + RMC (UBX-CFG-MSG).
    struct MsgRate { uint8_t id; uint8_t rate; };
    static const MsgRate msgRates[] = {
        {0x00, 1},  // GGA
        {0x04, 1},  // RMC
        {0x01, 0},  // GLL
        {0x02, 0},  // GSA
        {0x03, 0},  // GSV
        {0x05, 0},  // VTG
        {0x07, 0},  // GST
        {0x08, 0},  // ZDA
    };

    for (unsigned i = 0; i < sizeof(msgRates) / sizeof(*msgRates); ++i) {
        const uint8_t payload[3] = {0xF0, msgRates[i].id, msgRates[i].rate};
        sendUbx(0x06, 0x01, payload, sizeof(payload));
        usleep(GPS_INPUT_UBX_STEP_DELAY_US / 5);
    }

    // Step 4: navigation rate (UBX-CFG-RATE).
    const uint8_t cfgRate[6] = {
        (uint8_t)(measRateMs & 0xFF), (uint8_t)(measRateMs >> 8),
        0x01, 0x00,     // navRate = 1
        0x01, 0x00,     // timeRef = GPS
    };
    sendUbx(0x06, 0x08, cfgRate, sizeof(cfgRate));

    logInfo("GPS Input - uBlox configured: %d Hz  GGA+RMC  baud %d", rateHz, targetBaud);
}

void GpsInputTask::onSentence(const char* sentence, int length)
{
    if (length < 6 || sentence[0] != '$')
        return;

    // skip '$' so the first CSV field is the talker+type token, e.g. "GNGGA"
    CsvParser parser(sentence + 1, length - 1);

    char type[8] = {0};
    parser.nextString(type, sizeof(type) - 1);

    const int typeLength = (int)strlen(type);
    if (typeLength < 3)
        return;

    // match on the 3-char sentence type, ignoring the talker id (GP/GN/GL/...)
    const char* sentenceType = type + (typeLength - 3);

    if (strcmp(sentenceType, "GGA") == 0)
        parseGga(parser);
    else if (strcmp(sentenceType, "RMC") == 0)
        parseRmc(parser);
}

// GGA: <time>,<lat>,<NS>,<lon>,<EW>,<quality>,<numSats>,<hdop>,<alt>,<altUnit>,...
void GpsInputTask::parseGga(CsvParser& parser)
{
    double utcTime = 0;
    const bool hasTime = parser.nextDouble(utcTime);

    double lat = 0;
    double lon = 0;
    const bool hasLat = parser.nextDouble(lat);
    char ns[2] = {0};
    parser.nextString(ns, 1);
    const bool hasLon = parser.nextDouble(lon);
    char ew[2] = {0};
    parser.nextString(ew, 1);

    int quality = 0;
    parser.nextInt(quality);
    int sats = 0;
    parser.nextInt(sats);
    float hdop = 0;
    parser.nextFloat(hdop);
    float alt = 0;
    const bool hasAlt = parser.nextFloat(alt);

    gpsState.fixType = ggaQualityToFixType(quality);
    gpsState.satellites = sats;
    gpsState.hdop = hdop;
    if (hasAlt)
        gpsState.alt = alt;

    if (hasLat && hasLon && quality > 0) {
        gpsState.lat = nmeaToDegE7(lat, ns[0] == 'S');
        gpsState.lon = nmeaToDegE7(lon, ew[0] == 'W');
        gpsState.hasPosition = true;
    }
    else {
        gpsState.hasPosition = false;
    }

    if (hasTime && gpsState.hasDate) {
        utcToGpsTime(gpsState.year, gpsState.month, gpsState.day, nmeaTimeToSeconds(utcTime),
            gpsState.gpsWeek, gpsState.gpsTimeMs);
        gpsState.fixWallTimeMs = TimeUtils::getTime();
        gpsState.hasGpsTime = true;
    }
    else {
        gpsState.hasGpsTime = false;
    }

    // The actual GPS_INPUT send happens on the fixed-rate timer in loop().
    gpsDataReady = true;
    lastNmeaUpdateTime = TimeUtils::getTime();
}

// RMC: <time>,<status>,<lat>,<NS>,<lon>,<EW>,<speedKnots>,<courseDeg>,<date>,...
void GpsInputTask::parseRmc(CsvParser& parser)
{
    parser.nextDouble();    // UTC time (ignored)
    char status[2] = {0};
    parser.nextString(status, 1);   // A = valid, V = warning

    double lat = 0;
    double lon = 0;
    const bool hasLat = parser.nextDouble(lat);
    char ns[2] = {0};
    parser.nextString(ns, 1);
    const bool hasLon = parser.nextDouble(lon);
    char ew[2] = {0};
    parser.nextString(ew, 1);

    float speedKnots = 0;
    const bool hasSpeed = parser.nextFloat(speedKnots);
    float course = 0;
    const bool hasCourse = parser.nextFloat(course);

    int date = 0;   // ddmmyy
    if (parser.nextInt(date) && date > 0) {
        gpsState.day = date / 10000;
        gpsState.month = (date / 100) % 100;
        gpsState.year = 2000 + (date % 100);
        gpsState.hasDate = true;
    }

    if (hasLat && hasLon && status[0] == 'A') {
        gpsState.lat = nmeaToDegE7(lat, ns[0] == 'S');
        gpsState.lon = nmeaToDegE7(lon, ew[0] == 'W');
        gpsState.hasPosition = true;
    }

    if (hasSpeed && hasCourse) {
        const float speedMps = speedKnots * KNOTS_TO_MPS;
        const float courseRad = course * DEG_TO_RAD;
        gpsState.vn = speedMps * cosf(courseRad);
        gpsState.ve = speedMps * sinf(courseRad);
        gpsState.hasVelocity = true;
    }
    else {
        gpsState.hasVelocity = false;
    }
}

// Advance the latest fix's GPS time to "now" and keep it monotonic, so re-sent
// frames carry a steadily increasing timestamp between real NMEA updates.
void GpsInputTask::currentGpsTime(uint16_t& week, uint32_t& timeMs)
{
    if (!gpsState.hasGpsTime) {
        week = 0;
        timeMs = 0;
        return;
    }

    long elapsed = TimeUtils::getTime() - gpsState.fixWallTimeMs;
    if (elapsed < 0)
        elapsed = 0;

    long w = gpsState.gpsWeek;
    long tow = (long)gpsState.gpsTimeMs + elapsed;

    const long MS_PER_WEEK = 604800000L;
    while (tow >= MS_PER_WEEK) {
        tow -= MS_PER_WEEK;
        ++w;
    }

    // never step backwards relative to the previously injected timestamp
    if (w < lastSentWeek || (w == lastSentWeek && tow < (long)lastSentTimeMs)) {
        w = lastSentWeek;
        tow = lastSentTimeMs;
    }

    week = (uint16_t)w;
    timeMs = (uint32_t)tow;

    lastSentWeek = week;
    lastSentTimeMs = timeMs;
}

void GpsInputTask::sendGpsInput()
{
    uint16_t ignoreFlags = GPS_INPUT_IGNORE_FLAG_VDOP |
        GPS_INPUT_IGNORE_FLAG_VEL_VERT |
        GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
        GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
        GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY;

    if (!gpsState.hasVelocity)
        ignoreFlags |= GPS_INPUT_IGNORE_FLAG_VEL_HORIZ;

    uint16_t timeWeek = 0;
    uint32_t timeWeekMs = 0;
    currentGpsTime(timeWeek, timeWeekMs);

    mavlinkProvider->sendGpsInput(gpsId, gpsState.fixType, gpsState.lat, gpsState.lon, gpsState.alt,
        gpsState.satellites, gpsState.hdop, 0.0f, gpsState.vn, gpsState.ve, 0.0f, ignoreFlags,
        timeWeek, timeWeekMs);

    static TimeTrigger trigger(10000, true);
    static int gpsCounter = 0;
    ++gpsCounter;
    if (trigger.isFired()) {
        logInfo("GPS Input - send: fps: %d/%d  id %d  fix %d  sats %d  lat %d  lon %d  alt %.1f  hdop %.2f  vn %.2f  ve %.2f  week %d  tow %u",
            (int)(gpsCounter / (trigger.getPreiod() / 1000)), gpsCounter, gpsId, gpsState.fixType, gpsState.satellites, gpsState.lat, gpsState.lon, gpsState.alt,
            gpsState.hdop, gpsState.vn, gpsState.ve, timeWeek, timeWeekMs);
    }
}
