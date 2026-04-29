#include "nmea_sentence_reader.h"
#include <cstring>
#include <mutex>
#include "logging.h"

static std::mutex sendMutex;

static const int BUFFER_SIZE = 4096;
static const int SEND_BUFFER_SIZE = 1024;
static unsigned char rxBuffer[BUFFER_SIZE];
static unsigned char lineReceived[BUFFER_SIZE + 1];

int NmeaSentenceReader::send(const unsigned char* buffer, int length)
{
    std::lock_guard lock(sendMutex);

    if (sendLength == 0) {
        logInfo("send: %d", length);
        if (length > SEND_BUFFER_SIZE)
            length = SEND_BUFFER_SIZE;

        memcpy(sendBuf, buffer, length);
        sendLength = length;

        return length;
    }
    else {
        return 0;
    }
}

void NmeaSentenceReader::process()
{
    if (!connection || !connection->isConnected())
        return;

    processRead();
    processSend();
}

void NmeaSentenceReader::processRead()
{
    int receivedSize = 0;
    if (connection) {
        //memset(buff, 0, sizeof(buff));
        receivedSize = connection->read(rxBuffer, BUFFER_SIZE);
    }
    //logInfo("receivedSize: %d", receivedSize);

    if (receivedSize <= 0)
        return;

    //logInfo("receivedSize: %d", receivedSize);

    unsigned char* b = rxBuffer;

    while (receivedSize) {
        receiveChar(*b++);

        --receivedSize;
    }
}

void NmeaSentenceReader::receiveChar(unsigned char ch)
{
    if (state == 0) {
        if (ch == '$') {
            state = 1;
            bufferSize = 0;
        //logInfo("received: $");
            lineReceived[bufferSize++] = ch;
        }
    }
    else if (state == 1) {
        if (ch == '\0' || ch == '\r' || ch == '\n') {
            if (bufferSize > 0) {
                lineReceived[bufferSize] = 0;
                onNmeaSentenceReceived(lineReceived, bufferSize);
                //logInfo("received: \\n %d", bufferSize);
            }

            state = 0;
            bufferSize = 0;
        }
        else {
            if (bufferSize < BUFFER_SIZE) {
                lineReceived[bufferSize++] = ch;
            } 
            else {
                lineReceived[bufferSize] = 0;
                onNmeaSentenceReceived(lineReceived, bufferSize);
                //logInfo("received max: \\n %d", bufferSize);
                state = 0;
                bufferSize = 0;
            }
        }
    }
}

void NmeaSentenceReader::processSend()
{
    std::lock_guard lock(sendMutex);

    if (sendLength > 0) {
        logInfo("connection->send: %d", sendLength);
        int sentBytes = connection->send(sendBuf, sendLength);
        if (sentBytes)
            sendLength = 0;
    }
}

void NmeaSentenceReader::onNmeaSentenceReceived(const unsigned char* buffer, int length)
{
    for (std::list<INmeaSentenceReceiver*>::iterator r = receivers.begin(); r != receivers.end(); ++r) {
        (*r)->onNmeaSentenceReceived(buffer, length);
    }
}
