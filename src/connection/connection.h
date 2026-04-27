#ifndef __CONNECTION_H__
#define __CONNECTION_H__

#include <string>

class Connection
{
public:
    struct Credentials {
        static const std::string TCP;
        static const std::string COM;

        Credentials() {}
        Credentials(const Credentials& credentials) {
            *this = credentials;
        }

        std::string type;
        std::string address;
        int port;
    };

    Connection(const Credentials& credentials);
    virtual ~Connection() {};

    const Credentials credentials;

    const Credentials& getCredentials() const { return credentials; }

    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() const = 0;
    virtual int send(const unsigned char* buf, int size) = 0;
    virtual int read(unsigned char* buf, int size) = 0;
};

class ConnectionFactory
{
public:
    static Connection* create(const Connection::Credentials& credentials);
};

#endif
