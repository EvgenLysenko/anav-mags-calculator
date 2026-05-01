#include "mags_logger.h"
#include "logging.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <iomanip> // Required for std::setprecision
#include "config.h"
#include "utils/time_utils.h"
#include "utils/CsvParser.h"
#include <cstring>

bool MagsLogger::MAGS_FULL_TRACE_ENABLED = false;
bool MagsLogger::DEBUG_OUT_ENABLED = true;

const static int MAGS_MAX_NUMBER = 4;

using namespace std;

static const long CCR_SET_REQUEST_PERIOD = 1000;
static const int CCR_SET_REQUEST_TRIES = 10; // 10000

static const long RUN_THREAD_TIMEOUT_IN_PROGRESS = 10; // 0.1ms
static const long RUN_THREAD_TIMEOUT_IDLE = 1000; // 1s
static const long CONSOLE_OUT_PERIOD = 10000; // 60s
static const long LOG_OUT_PERIOD_MIN = 1000;

const unsigned short MAGS_CCR_DEFAULT = 100;
const unsigned short MAGS_CCR_MAX = 0xFFFF;

static const long STATUS_SEND_PERIOD = 1000;
static const long SENSORS_SEND_PERIOD = 5000;
static const long SETTINGS_SEND_PERIOD = 10000;
static const long MAGS_VALUES_SEND_PERIOD = 3000; 

MagsLogger::MagsLogger(INmeaSentenceSender* nmeaSentenceSender, MavlinkProvider* mavlinkProvider):
    TaskBase(),
    nmeaSentenceSender(nmeaSentenceSender),
    mavlinkProvider(mavlinkProvider)
{
    ccrGetRequester.init(nmeaSentenceSender);
    ccrSetRequester.init(nmeaSentenceSender);
    outSetCommandRequester.init(nmeaSentenceSender);
    mavlinkProvider->addMavlinkReceiver(this);
}

void MagsLogger::onCcrSet(int ccr, int repeatCount)
{
    logInfo("Mags - set CCR: %d", ccr);

    char text[32];
    sprintf(text, "$MAG,SET,CCR,%d\n", ccr);
    ccrSetRequester.sendCommand(text, repeatCount, CCR_SET_REQUEST_PERIOD, MAGS_CCR_SET, ccr);
    onCcrGet();
}

void MagsLogger::onCcrGet()
{
    ccrGetRequester.sendCommand("$MAG,GET,CCR\n", 10, CCR_SET_REQUEST_PERIOD, MAGS_CCR_SET);
}

void MagsLogger::onNmeaSentenceReceived(const unsigned char* buffer, int length)
{
    if (MAGS_FULL_TRACE_ENABLED)
        logInfo("Mags - trace: %s", (const char*)buffer);

    if (length < 2 || buffer[0] != '$')
        return;

    if (length > 2 && buffer[1] == 'M' && buffer[2] == ',') {
        magsReceivedCounter.add();

        //for (Mag& mag: mags) {
        //    mag.online = 1;
        //}

        const long curTime = TimeUtils::getTime();
        magsReceivedTime = curTime;
        memcpy(magsReceivedSentence, buffer, length);
        magsReceivedSize = length;
        magsReceived = true;

        static long magsDetectedLastTime = 0;
        static long accelDetectedLastTime = 0;
        CsvParser parser((const char*)buffer, length);
        char tmp[32 + 1] = { 0 };
        memset(tmp, 0, sizeof(tmp) / sizeof(*tmp));
        const int tmpSize = sizeof(tmp) / sizeof(*tmp) - 1;

        // $M,mags-time,mx1,my1,mz1,mx2,my2,mz2,accel-time

        // mags detection
        parser.nextString(tmp, tmpSize);

        int magsTime = 0;
        if (parser.nextInt(magsTime) && magsTime)
            magsDetectedLastTime = curTime;

        outMagsDetected = !TimeUtils::isTimeout(curTime, magsDetectedLastTime, 1000);

        int mx = 0, my = 0, mz = 0;
        parser.nextInt(mx);
        parser.nextInt(my);
        parser.nextInt(mz);

        if (mags.size() > 0) {
            mags[0].time = curTime;
            mags[0].x = mx;
            mags[0].y = my;
            mags[0].z = mz;
            mags[0].online = mx || my || mz;
        }

        parser.nextInt(mx);
        parser.nextInt(my);
        parser.nextInt(mz);

        if (mags.size() > 1) {
            mags[0].time = curTime;
            mags[1].x = mx;
            mags[1].y = my;
            mags[1].z = mz;
            mags[1].online = mx || my || mz;
        }

        if (magsDataListener)
            magsDataListener->onMagsDataReceived(magsTime, mags[0], mags[1]);

        // accel detection
        int accelTime = 0;
        if (parser.nextInt(accelTime) && accelTime)
            accelDetectedLastTime = curTime;

        outAccelDetected = !TimeUtils::isTimeout(curTime, accelDetectedLastTime, 1000);

        if (DEBUG_OUT_ENABLED) {
            static TimerTrigger trigger(10000);
            if (trigger.isFired(curTime)) {
                logInfo("Mags - %s", (const char*)buffer);
                logInfo("Mags - debug - mags data: %s  count %d  time mags %d  accel %d,  mags detect time: %ld  detected %d  vector: %d, %d, %d",
                        (const char*)buffer, (int)mags.size(), magsTime, accelTime, magsDetectedLastTime, (int)outMagsDetected,
                        mx, my, mz);
                logInfo("Mags - SEND mag1[%c]: %5d %5d %5d   mag2[%c]: %5d %5d %5d",
                    mags.size() > 0 ? '+' : '-', mags.size() > 0 ? (int)mags[0].x : 0, mags.size() > 0 ? (int)mags[0].y : 0, mags.size() > 0 ? (int)mags[0].z : 0,
                    mags.size() > 0 ? '+' : '-', mags.size() > 1 ? (int)mags[1].x : 0, mags.size() > 1 ? (int)mags[1].y : 0, mags.size() > 1 ? (int)mags[1].z : 0
                );
            }
        }

        if (outAccelDetected) {
            static TimerTrigger trigger(1000);
            if (trigger.isFired(curTime)) {
                parser.nextInt(accel.x);
                parser.nextInt(accel.y);
                parser.nextInt(accel.z);
                parser.nextInt(gyro.x);
                parser.nextInt(gyro.y);
                parser.nextInt(gyro.z);
                parser.nextInt(accelMag.x);
                parser.nextInt(accelMag.y);
                parser.nextInt(accelMag.z);
            }
        }

        if (!outMagsDetected && !outAccelDetected) {
            static TimerTrigger trigger(10000);
            if (trigger.isFired(curTime)) {
                logInfo("Mags - no detect: %s  time mags %d  accel %d,  mags detect time: %ld  detected %d", (const char*)buffer, magsTime, accelTime, magsDetectedLastTime, (int)outMagsDetected);
            }
        }
        else if (outMagsDetected) {
            if (mags.size() >= 2 && ((!mags[0].x && !mags[0].y && !mags[0].z) || (!mags[1].x && !mags[1].y && !mags[1].z))) {
                static TimerTrigger trigger(10000);
                if (trigger.isFired(curTime)) {
                    logInfo("Mags - no mags data: %s  time mags %d  accel %d,  mags detect time: %ld  detected %d  vector: %d, %d, %d",
                            (const char*)buffer, magsTime, accelTime, magsDetectedLastTime, (int)outMagsDetected,
                            mx, my, mz);
                    logInfo("Mags - %s", (const char*)buffer);
                }
            }
        }
    }
    else if (length > 8 && buffer[1] == 'M' && buffer[2] == 'A' && buffer[3] == 'G' && buffer[4] == ',' &&
                           buffer[5] == 'C' && buffer[6] == 'C' && buffer[7] == 'R' && buffer[8] == ',') {
        logInfo("Mags - CCR - %s", (const char*)buffer);

        // $MAG,CCR,123,123,123,123,123,123*XX
        CsvParser parser((const char*)buffer, length);

        char tmp[8 + 1];
        memset(tmp, 0, 8 + 1);
        parser.nextString(tmp, 8);
        logInfo("Mags - %s", tmp);

        memset(tmp, 0, 8);
        parser.nextString(tmp, 8);
        logInfo("Mags - %s", tmp);

        int ccr = 0;
        if (parser.nextInt(ccr)) {
            logInfo("Mags - CCR parsed: %d", ccr);

            ccrReceived = ccr;

            if (!ccrGetRequester.isFinished()) {
                ccrGetRequester.confirm(true);
            }

            if (!ccrSetRequester.isFinished()) {
                ccrSetRequester.confirm(ccrSetRequester.getTag() == ccr);

                if (!ccrSetRequester.isFinished())
                    onCcrGet();

                if (logStarted)
                    startLogging();
            }

            //sendSettings();
        }
        else {
            logInfo("Mags - ccr: NA");
        }
    }
}

static void createDir(const char* dirName)
{
    struct stat st = { 0 };

    if (stat(dirName, &st) == -1) {
        mkdir(dirName, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }
}

static string createFileName(string dir, unsigned long unixTime, int ccrValue)
{
    if (dir.length() > 0 && (dir[dir.length() - 1] != '/' || dir[dir.length() - 1] != '\\')) {
        dir += "/";
    }

    const time_t currTime = unixTime ? unixTime : ::time(NULL);
    if (currTime == (time_t)(-1))
        return dir + "mags.log";

    const tm* t = gmtime(&currTime);
    if (!t)
        return dir + "mags.log";

    char result[63];
    sprintf(result, "mags_%04d%02d%02d-%02d%02d%02d_%d.log", 1900 + t->tm_year, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec, ccrValue);

    return dir + result;
}

void MagsLogger::onStarted()
{
    logInfo("MagsLogger - started");

    initParams();
}

void MagsLogger::initParams()
{
    logDir = Config::readString("mags/logDir", "./logs");
    int count = Config::readInt("mags/count", 2);
    if (count > MAGS_MAX_NUMBER)
        count = MAGS_MAX_NUMBER;

    if (count > 0) {
        mags.resize(count);
        int idx = 0;
        for (Mag& mag: mags) {
            memset(&mag, 0, sizeof(mag));
            mag.idx = idx++;
            mag.logged = false;
            mag.online = false;
        }
    }

    logInfo("Mags - count: %d  read %d", (int)mags.size(), count);

    int ccr = Config::readInt("mags/ccr", MAGS_CCR_DEFAULT);
    logInfo("Mags - ccr: %d", ccr);
    onCcrSet(ccr, CCR_SET_REQUEST_TRIES);

    x_inverted = Config::readBool("mags/x_inverted", false);
    y_inverted = Config::readBool("mags/y_inverted", false);
    z_inverted = Config::readBool("mags/z_inverted", false);
    xy_switched = Config::readBool("mags/xy_switched", false);

    logInfo("Mags - inverted: x: %d  y: %d  z: %d", x_inverted, y_inverted, z_inverted);
    logInfo("Mags - xy_switched: %d", xy_switched);
}

void MagsLogger::logInit()
{
    const string logFileName = createFileName(logDir, gpsSystemUnuxTime, ccrReceived);

    if (logFile.is_open())
        logFile.close();

    logFile.open(logFileName.c_str(), std::ios::app);
    if (!logFile.is_open()) {
        logWarning("Mags - log file open FILED: %s", logFileName.c_str());
    }
    else {
        logInfo("Mags - log file created: %s", logFileName.c_str());

        logFile << "time, $M, mags_time, ";

        for (vector<Mag>::size_type i = 0; i < mags.size(); ++i) {
            const int n = i + 1;
            logFile << "mx" << n << ", my" << n << ", mz" << n << ", ";
        }

        logFile << "ax, ay, az, gx, gy, gz,,,, *CRC, ";

        logFile << "gps_time, lat, lon, alt, att_time, roll, pitch, yaw"; //, att_time(ms)";
        logFile << std::endl;
    }
}

void MagsLogger::magsInit()
{
    for (vector<Mag>::size_type i = 0; i < mags.size(); ++i) {
        Mag& mag = mags[i];
        mag.idx = i;
        logInfo("Mags - mag num: %d", mag.idx + 1);
    }
}

void MagsLogger::startLogging()
{
    createDir(logDir.c_str());
    logInit();
    logStarted = true;
    logStartedTime = TimeUtils::getTime();
}

__useconds_t MagsLogger::loop(__useconds_t default_timeout)
{
    const long curTime = TimeUtils::getTime();    

    static long counterTime = 0;
    if (TimeUtils::isTimeout(curTime, counterTime, 1000)) {
        counterTime = curTime;
        magsReceivedCounter.restart();
        magsLogoutCounter.restart();
        gpsCounter.restart();
        attitudeCounter.restart();
    }

    if (!magsInitialized) {
        magsInitialized = true;
        magsInit();
        consoleOutTime = curTime - (CONSOLE_OUT_PERIOD - 10000);
    }

    if (!logStarted) {
        startLogging();
    }

    // dispatcher
    if (magsInitialized) {
        // message for MP - mags is online
        if (mavlinkProvider->isGscReady()) {
            static bool magsStartedMessageSent = false;
            if (!magsStartedMessageSent) {
                magsStartedMessageSent = true;

                mavlinkProvider->sendStatustext(MAV_SEVERITY_INFO, "Mags - started");
            }

            static bool magsOnlineMessageSent = false;
            if (!magsOnlineMessageSent) {
                if (magsReceivedCounter.count > 0) {
                    magsOnlineMessageSent = true;

                    mavlinkProvider->sendStatustext(MAV_SEVERITY_INFO, "Mags - online");

                    sendStatus();
                }
            }
        }
    }     
    static long sendStatusTime = curTime;
    if (TimeUtils::isTimeout(curTime, sendStatusTime, STATUS_SEND_PERIOD)) {
        sendStatus();
        sendStatusTime = curTime;
    }

    static long sendSensorTime = curTime;
    if (TimeUtils::isTimeout(curTime, sendSensorTime, SENSORS_SEND_PERIOD)) {
        sendSensorStatus();
        sendSensorTime = curTime;
    }

    static long sendSettingsTime = curTime;
    if (TimeUtils::isTimeout(curTime, sendSettingsTime, SETTINGS_SEND_PERIOD)) {
        sendSettings();
        sendSettingsTime = curTime;
    }

    static long sendMagsValuesTime = curTime;
    if (TimeUtils::isTimeout(curTime, sendMagsValuesTime, MAGS_VALUES_SEND_PERIOD)) {
        if (outMagsDetected)
            sendMagsValues();

        if (outAccelDetected)
            sendAccelValues();

        sendMagsValuesTime = curTime;
    }
    // dispatcher

    ccrGetRequester.loop();
    ccrSetRequester.loop();
    outSetCommandRequester.loop();

    magsLogValues();

    return magsInitialized || logStarted ? RUN_THREAD_TIMEOUT_IN_PROGRESS : default_timeout;
}

void MagsLogger::readValues(Mag& mag)
{
    mag.time = TimeUtils::getTime();

    if (xy_switched)
        swap(mag.x, mag.y);

#ifdef MAGS_TESTS_EMULATE_DATA
    mag.mx = xy_switched ? 1000 : 50;
    mag.my = xy_switched ? 50 : 1000;
    mag.mz = 1500;
#endif

    if (x_inverted)
        mag.x = -mag.x;
    if (y_inverted)
        mag.y = -mag.y;
    if (z_inverted)
        mag.z = -mag.z;
}

void MagsLogger::magsLogValues()
{
    const long currTime = TimeUtils::getTime();

    static bool magDataAppeared = false;
    if (!magDataAppeared) {
        for (const Mag& mag: mags) {
            if (isDataPresent(mag)) {
                magDataAppeared = true;
                //consoleOutTime = 0;
                consoleOutTime = currTime - (CONSOLE_OUT_PERIOD - 2000);
                break;
            }
        }
    }

    if (CONSOLE_OUT_PERIOD > 0 && TimeUtils::isTimeout(consoleOutTime, CONSOLE_OUT_PERIOD)) {
        int magsOnlineCount = 0;
        
        for (const Mag& mag: mags) {
            if (isDataPresent(mag))
                ++magsOnlineCount;

            logInfo("Mags - mag %d: %7ld %7ld %7ld", mag.idx, mag.x, mag.y, mag.z);
        }

        consoleOutTime = currTime;
        logInfo("Mags - online: %d/%d   gps: %s   logs: %s  fps: %d   fps mags: %3d  gps: %2d  att: %2d", magsOnlineCount, (int)mags.size(),
              (gpsFixed ? "on" : "off"), (logStarted ? "on" : "off"), magsLogoutCounter.count, magsReceivedCounter.count, gpsCounter.count, attitudeCounter.count);
    }

    // mags always is friquently
    if (logStarted && magsReceived) {
        magsLogoutCounter.add();
        logFile << magsReceivedTime << ",";
        logFile.write((const char*)magsReceivedSentence, magsReceivedSize);

        magsReceived = false;

        logFile << std::endl;
    }

    for (Mag& mag: mags) {
        mag.logged = true;
    }
}

void MagsLogger::CommandRequester::loop()
{
    if (isFinished())
        return;

    const long curTime = TimeUtils::getTime();

    if (TimeUtils::isTimeout(curTime, commandSendTime, repeatTimeout)) {
        logInfo("Mags - command requester: \"%s\"  size: %d  tries: %d", command.c_str(), (int)command.size(), sendCountLeft);

        if (sender) {
            sender->send((const unsigned char*)command.c_str(), command.size());
        }
        else {
            logError("Mags - command requester - sender not defined");
        }

        commandSendTime = curTime;
        
        if (sendCountLeft > 0)
            --sendCountLeft;

        if (sendCountLeft == 0) {
            logInfo("Mags - command requester: \"%s\"  tries: 0  finished", command.c_str());
        }
    }
}

TimerTrigger::TimerTrigger(long period): period(period) {
    this->time = TimeUtils::getTime();
}

bool TimerTrigger::isFired(long curTime) {
    if (TimeUtils::isTimeout(curTime, time, period)) {
        time = curTime;
        return true;
    }
    else {
        return false;
    }
}

bool TimerTrigger::isFired() {
    return isFired(TimeUtils::getTime());
}
