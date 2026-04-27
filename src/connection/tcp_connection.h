#ifndef __TCP_CONNECTION_H__
#define __TCP_CONNECTION_H__

#include "connection.h"

class TcpConnection: public Connection
{
protected:
    int sockfd;

public:
    TcpConnection(const Connection::Credentials& credentials);
    virtual ~TcpConnection() {};

    virtual bool connect();
    virtual void disconnect();
    virtual bool isConnected() const;
    virtual int send(const unsigned char* buf, int size);
    virtual int read(unsigned char* buf, int size);
};

#endif
