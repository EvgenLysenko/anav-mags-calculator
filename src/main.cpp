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

int main(int argc, char* argv[])
{
    signal(SIGSEGV, segHandler);

    logsInit(argc, argv);

    Connection::Credentials credentials;
    credentials.type = Config::readString("connection/type", Connection::Credentials::COM);
    credentials.address = Config::readString("connection/address", "/dev/ttyS1");
    credentials.port = Config::readInt("connection/port", 115200);

    logInfo("connection - mags : type: \"%s\" address: \"%s\" port: \"%d\"", credentials.type.c_str(), credentials.address.c_str(), credentials.port);

    Connection* connection = ConnectionFactory::create(credentials);
    if (connection) {
        logInfo("connection - mags created");
        bool result = connection->connect();
        logInfo("connection - mags connect: %d", (int)result);
    }
    else {
        logError("connection - mags creating failed");
    }

    logInfo("connection - mags connected: %d", (int)connection->isConnected());

    NmeaSentenceReader nmeaSentenceReader(connection);

    MagsLogger magsLogger(&nmeaSentenceReader);
    nmeaSentenceReader.addNmeaSentenceListener(&magsLogger);

    MagsLogger::MAGS_FULL_TRACE_ENABLED = Config::readBool("debug/trace", false);
    logInfo("MAGS_FULL_TRACE_ENABLED: %d", (int)MagsLogger::MAGS_FULL_TRACE_ENABLED);

    magsLogger.start();

    MagsCalculator::WINDOW_SIZE = Config::readInt("mags/windowSize", MagsCalculator::WINDOW_SIZE);
    logInfo("WINDOW_SIZE: %d", (int)MagsCalculator::WINDOW_SIZE);

    MagsCalculator magsCalculator;
    magsLogger.setMagsDataListener(&magsCalculator);
    magsCalculator.start();

    while (true) {
        if (!connection || !connection->isConnected()) {
            //logInfo("connection lost");
            //break;
        }

        nmeaSentenceReader.process();

        //magsLogger.run();

        usleep(100);
    }

    logInfo("stop");

    if (connection) {
        connection->disconnect();
        delete connection;
    }

    logInfo("exit");
}
