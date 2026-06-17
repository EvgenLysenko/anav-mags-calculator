#include <cstdio>
#include <signal.h>
#include <execinfo.h>
#include <unistd.h>
#include "connection/connection.h"
#include "logging.h"
#include "config.h"
#include "mags-logger/mags_logger.h"
#include "mags-logger/nmea_sentence_reader.h"
#include "mags-logger/mags_calculator.h"
#include "mags-logger/gps_input_task.h"
#include "ip.h"

void sigsegHandler(int sig)
{
    fprintf(stderr, "Error: SIGSEGV signal %d\n", sig);

    void* arr[10];
    size_t size = backtrace(arr, sizeof(arr) / sizeof(*arr));
    backtrace_symbols_fd(arr, size, STDERR_FILENO);
    exit(0);
}

void sigpipeHandler(int sig)
{
    fprintf(stderr, "Error: SIGPIPE signal %d\n", sig);
    exit(0);
}

void logsInit(int argc, char* argv[])
{
    loguru::g_preamble_date = false;
    loguru::g_preamble_time = true;
    loguru::g_preamble_uptime = false;
    loguru::g_preamble_thread = false;
    loguru::g_preamble_file = false;
    loguru::g_preamble_verbose = true;
    loguru::g_preamble_pipe = true;
    loguru::g_colorlogtostderr = true;

    loguru::init(argc, argv);
    loguru::add_file("mags-logger.log", loguru::Append, loguru::Verbosity_MAX);
}

static Connection* createConnection(const char* name, const Connection::Credentials& credentials)
{
    logInfo("main - %s connection: type: \"%s\" address: \"%s\" port: \"%d\"", name, credentials.type.c_str(), credentials.address.c_str(), credentials.port);

    Connection* connection = ConnectionFactory::create(credentials);
    if (connection) {
        logInfo("main - %s connection: created", name);

        bool result = connection->connect();
        logInfo("main - %s connection: connect: %d", name, (int)result);
        logInfo("main - %s connection: connected: %d", name, (int)connection->isConnected());
    }
    else {
        logError("main - %s connection: creating failed", name);
    }


    return connection;
}

int main(int argc, char* argv[])
{
    signal(SIGSEGV, sigsegHandler);
    signal(SIGPIPE, sigpipeHandler);

    logsInit(argc, argv);

    logInfo("Mags - v2.3.1");

    // mavlink
    Connection::Credentials credentials;
    credentials.type = Config::readString("mavlink/connection/type", Connection::Credentials::COM);
    credentials.address = Config::readString("mavlink/connection/address", "/dev/ttyS1");
    credentials.port = Config::readInt("mavlink/connection/port", 115200);

    Connection* mavConnection = createConnection("mavlink", credentials);
    MavlinkProvider mavlinkProvider(mavConnection);

    // sensors
    credentials.type = Config::readString("mags/connection/type", Connection::Credentials::COM);
    credentials.address = Config::readString("mags/connection/address", "/dev/ttyS2");
    credentials.port = Config::readInt("mags/connection/port", 115200);

    Connection* magsConnection = createConnection("mags", credentials);
    NmeaSentenceReader nmeaSentenceReader(magsConnection);
    MagsLogger magsLogger(&nmeaSentenceReader, &mavlinkProvider);
    nmeaSentenceReader.addNmeaSentenceListener(&magsLogger);

    MagsLogger::MAGS_FULL_TRACE_ENABLED = Config::readBool("debug/trace", false);
    logInfo("MAGS_FULL_TRACE_ENABLED: %d", (int)MagsLogger::MAGS_FULL_TRACE_ENABLED);

    magsLogger.start();

    // calculator
    MagsCalculator::WINDOW_SIZE = Config::readInt("mags/windowSize", MagsCalculator::WINDOW_SIZE);
    logInfo("WINDOW_SIZE: %d", (int)MagsCalculator::WINDOW_SIZE);

    MagsCalculator magsCalculator;
    magsLogger.setMagsDataListener(&magsCalculator);
    magsCalculator.start();

    // companion-computer GPS (uBlox NEO-M8N on UART7) -> flight controller (GPS_INPUT)
    credentials.type = Config::readString("gps/connection/type", Connection::Credentials::COM);
    credentials.address = Config::readString("gps/connection/address", "/dev/ttyS7");
    const int gpsTargetBaud = Config::readInt("gps/connection/port", 115200);
    const int gpsInitialBaud = Config::readInt("gps/initialBaud", 9600);
    credentials.port = gpsInitialBaud > 0 ? gpsInitialBaud : gpsTargetBaud;

    Connection* gpsConnection = createConnection("gps", credentials);
    GpsInputTask gpsInputTask(gpsConnection, &mavlinkProvider);
    gpsInputTask.start();

    const uint32_t ip = getIp();
    logInfo("ip: %d.%d.%d.%d", (int)(ip & 0xFF), (int)((ip >> 8) & 0xFF), (int)((ip >> 16) & 0xFF), (int)((ip >> 24) & 0xFF));
    magsLogger.ip = ip;

    while (true) {
        if (!magsConnection || !magsConnection->isConnected()) {
            //logInfo("connection lost");
            //break;
        }

        nmeaSentenceReader.process();
        mavlinkProvider.process();

        //magsLogger.run();

        usleep(100);
    }

    logInfo("stop");

    gpsInputTask.stop();

    if (magsConnection) {
        magsConnection->disconnect();
        delete magsConnection;
    }

    if (gpsConnection) {
        gpsConnection->disconnect();
        delete gpsConnection;
    }

    logInfo("exit");
}
