#ifndef __COM_CONNECTION_H__
#define __COM_CONNECTION_H__

#include "connection.h"

class ComConnection: public Connection
{
protected:
    int fd;

public:
    ComConnection(const Connection::Credentials& credentials);
    virtual ~ComConnection() {};

    virtual bool connect();
    virtual void disconnect();
    virtual bool isConnected() const;
    virtual int send(const unsigned char* buf, int size);
    virtual int read(unsigned char* buf, int size);

    bool setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);
};

#endif
