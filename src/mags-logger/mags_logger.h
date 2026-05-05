#ifndef __MAGS_LOGGER_H__
#define __MAGS_LOGGER_H__

#include "task_base.h"
#include "nmea_sentence_reader.h"
#include <fstream>
#include <vector>
#include <string>
#include "mav/mavlink_provider.h"
#include "common/position.h"
#include "common/attitude.h"

struct Counter {
    unsigned int count = 0;
    unsigned int counter = 0;

    void add() { ++counter; }
    void restart() {
        count = counter;
        counter = 0;
    }
};

class TimerTrigger
{
protected:
    const long period = 0;
    long time = 0;

public:
    TimerTrigger(long period);

    bool isFired();
    bool isFired(long curTime);
};

class MagsLogger: public TaskBase, public INmeaSentenceReceiver, public IMavlinkReceiver
{
public:
    MagsLogger(INmeaSentenceSender* nmeaSentenceSender, MavlinkProvider* mavlinkProvider);

    virtual const char* getTaskName() const { return "MagsLogger"; }
    //virtual void run();

    virtual void onNmeaSentenceReceived(const unsigned char* buffer, int length);

    uint32_t ip = 0;
    bool ipRequested = true;

    static bool MAGS_FULL_TRACE_ENABLED;
    static bool DEBUG_OUT_ENABLED;

    class CommandRequester {
    public:
        CommandRequester() {}
        INmeaSentenceSender* sender = 0;
        int repeatCount = 0;
        long repeatTimeout = 0;

        void init(INmeaSentenceSender* nmeaSentenceSender) {
            this->sender = nmeaSentenceSender;
        }

    protected:
        int id = 0;
        int tag = 0;
        std::string command;
        bool commandRequested = false;
        long commandSendTime = 0;
        int sendCountLeft = 0;
        bool commandIsConfirmed = true;

    public:
        void sendCommand(const std::string& command, int repeatCount, long repeatTimeout, int id, int tag = 0) {
            this->id = id;
            this->tag = tag;
            this->command = command;
            this->repeatCount = repeatCount;
            this->repeatTimeout = repeatTimeout;
            this->commandSendTime = 0;
            this->sendCountLeft = repeatCount;
            this->commandIsConfirmed = false;
        }

        int getId() const { return id; }
        int getTag() const { return tag; }
        const std::string& getCommand() const { return command; }
        void confirm(bool confirmed) { commandIsConfirmed = confirmed; }
        void confirmOk() { commandIsConfirmed = true; }

        bool isFinished() const { return commandIsConfirmed || sendCountLeft == 0; }
        bool isCommandConfirmed() const;

        virtual void loop();
    };

public:
    enum MagsCommandId {
        MAGS_SENSOR1_STATUS = 2001,
        MAGS_SENSOR2_STATUS = 2002,
        MAGS_SENSOR3_STATUS = 2003,
        MAGS_SENSOR4_STATUS = 2004,
        MAGS_SENSORS_STATUS = 2011,
        MAGS_STATUS = 2012,
        MAGS_CCR_SET = 2013,
        MAGS_SETTINGS = 2014,
        MAGS_SETTINGS_REQUEST = 2015,
        MAGS_LOGGING_START = 2016,
        MAGS_LOGGING_STOP = 2017,
        MAGS_SET_OUT_MAGS = 2018,
        MAGS_SET_OUT_ACCEL = 2019,
        MAGS_MAGS_VALUES = 2020,
        MAGS_ACCEL_VALUES = 2021,
        MAGS_FULL_TRACE_ENABLE = 2022,
        MAGS_DEBUG_ENABLE = 2023,
        MAGS_IP = 2024,
    };

    struct Mag {
        int idx;
        long x;
        long y;
        long z;
        uint32_t time;
        bool logged;
        bool online;
    };

    struct Vector {
        int x = 0;
        int y = 0;
        int z = 0;
    };

    static bool isDataPresent(const Mag& mag) {
        return mag.x || mag.y || mag.z;
    }

    bool wait_gps = true;

    class IMagsDataListener {
    public:
        virtual void onMagsDataReceived(uint32_t time, const Mag& mag0, const Mag& mag1) = 0;
    };

    void setMagsDataListener(IMagsDataListener* magsDataListener) {this->magsDataListener = magsDataListener; }

protected:
    INmeaSentenceSender* const nmeaSentenceSender;
    IMagsDataListener* magsDataListener = 0;

    std::ofstream logFile;
    std::string logDir;

    std::vector<Mag> mags;
    Vector accel;
    Vector gyro;
    Vector accelMag;
    
    void startLogging();
    void magsInit();
    void magsRead();
    void magsLogValues();
    void logInit();
    void initParams();

    bool magsInitialized = false;
    bool gpsFixed = false;
    bool logStarted = false;
    long logStartedTime = 0;
    bool planeMessagesConfigured = false;

    bool outAccelDetected = false;
    bool outMagsDetected = false;

    long consoleOutTime = 0;

    char magsReceivedSentence[1024];
    int magsReceivedSize = 0;
    bool magsReceived = false;
    long magsReceivedTime = 0;

    unsigned long gpsSystemUnuxTime = 0;

    Counter magsReceivedCounter;
    Counter magsLogoutCounter;
    Counter attitudeCounter;
    Counter gpsCounter;

    bool x_inverted = false;
    bool y_inverted = false;
    bool z_inverted = false;
    bool xy_switched = false;

    unsigned short ccrReceived = 0;
    CommandRequester ccrGetRequester;
    CommandRequester ccrSetRequester;

    CommandRequester outSetCommandRequester;

    bool isStatusMessageSent = false;

protected:
    virtual void onStarted();
    void receive_message();
    __useconds_t loop(__useconds_t default_timeout);

    void readValues(Mag& mag);

protected:
    void onCcrSet(int ccr, int repeatCount = 10);
    void onCcrGet();
    void onLoggingStop();
    void onSwitchToMags();
    void onSwitchToAccel();
    void returnStatus();

protected:
    MavlinkProvider* const mavlinkProvider;
    virtual void onMessageReceived(const mavlink_message_t& message);

    void on_msg_system_time_receved(const mavlink_message_t* message);
    void on_msg_gps_raw_received(const mavlink_message_t* message);
    void on_msg_attitude_received(const mavlink_message_t* message);
    void on_command_long_received(const mavlink_message_t* message);

    void sendCommand(MagsCommandId magsCommandId, float param1, float param2 = 0, float param3 = 0, float param4 = 0, float param5 = 0, float param6 = 0);
    void sendStatus();
    void sendSensorStatus();
    void sendMagsValues();
    void sendAccelValues();
    void sendSettings();
    void sendIp();

    FlightMode flightMode = FlightMode::Manual;
    uint8_t plane_base_mode = 0;
    bool isArmed() const { return plane_base_mode & MAV_MODE_FLAG::MAV_MODE_FLAG_SAFETY_ARMED; }

    Position position;
    bool positionReceived = false;

    long gpsReceivedTime = 0;

    Attitude attitude;
    long attitudeReceivedTime = 0;
};

#endif
