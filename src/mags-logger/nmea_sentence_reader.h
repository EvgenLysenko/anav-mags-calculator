#ifndef __NMEA_SENTENCE_READER_H__
#define __NMEA_SENTENCE_READER_H__

#include "connection/connection.h"
#include <list>

class INmeaSentenceReceiver {
public:
    virtual void onNmeaSentenceReceived(const unsigned char* buffer, int length) = 0;
};

class INmeaSentenceSender {
public:
    virtual int send(const unsigned char* buffer, int length) = 0;
};

class NmeaSentenceReader: public INmeaSentenceSender
{
public:
    NmeaSentenceReader(Connection* connection): connection(connection) {}

protected:
    //unsigned char buffer[];
    unsigned char sendBuf[1024];
    int sendLength = 0;
    Connection* connection;
    int bufferSize = 0;
    int state = 0;

    void receiveChar(unsigned char ch);
    void processRead();
    void processSend();

    std::list<INmeaSentenceReceiver*> receivers;
    void onNmeaSentenceReceived(const unsigned char* buffer, int length);
    
public:
    bool isConnected() const { return connection && connection->isConnected(); }
    void process();
    virtual int send(const unsigned char* buffer, int length);

    void addNmeaSentenceListener(INmeaSentenceReceiver* nmeaSentenceReceiver) {
        receivers.remove(nmeaSentenceReceiver);
        receivers.push_back(nmeaSentenceReceiver);
    }

    void remNmeaSentenceListener(INmeaSentenceReceiver* nmeaSentenceReceiver) {
        receivers.remove(nmeaSentenceReceiver);
    }
};

#endif
