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

void segHandler(int sig)
{
    fprintf(stderr, "Error: signal %d", sig);

    void* arr[10];
    size_t size = backtrace(arr, sizeof(arr) / sizeof(*arr));
    backtrace_symbols_fd(arr, size, STDERR_FILENO);
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
    }
    else {
        logError("main - %s connection: creating failed", name);
    }

    logInfo("main - %s connection: connected: %d", name, (int)connection->isConnected());

    return connection;
}

int main(int argc, char* argv[])
{
    signal(SIGSEGV, segHandler);

    logsInit(argc, argv);

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

    while (true) {
        if (!magsConnection || !magsConnection->isConnected()) {
            //logInfo("connection lost");
            //break;
        }

        nmeaSentenceReader.process();

        //magsLogger.run();

        usleep(100);
    }

    logInfo("stop");

    if (magsConnection) {
        magsConnection->disconnect();
        delete magsConnection;
    }

    logInfo("exit");
}
