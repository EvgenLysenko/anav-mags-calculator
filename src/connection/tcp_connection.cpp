#include "tcp_connection.h"
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include "logging.h"

TcpConnection::TcpConnection(const Connection::Credentials& credentials)
    : Connection(credentials)
{
    sockfd = -1;
}

bool TcpConnection::connect()
{
    logInfo("TcpConnection - connect");
    if (sockfd < 0)
        sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (sockfd < 0) {
        logError("TcpConnection - ERROR opening socket");
        return false;
    }
    else
        logInfo("TcpConnection - socket created");

    //hostent *server = gethostbyname("altair.no-ip.org");
    hostent *server = gethostbyname(credentials.address.c_str());
    if (server == NULL)
    {
        logWarning("TcpConnection - ERROR, no such host");
        return false;
    }
    else
        logInfo("TcpConnection - server found");

    logInfo("TcpConnection - hostent: %d.%d.%d.%d  len: %d", (int)(unsigned char)server->h_addr[0], (int)(unsigned char)server->h_addr[1], (int)(unsigned char)server->h_addr[2], (int)(unsigned char)server->h_addr[3], server->h_length);

    struct sockaddr_in serv_addr;
    memset((char*)&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    memcpy((char*)&serv_addr.sin_addr.s_addr, (char*)server->h_addr, server->h_length);

    const unsigned char* a = (unsigned char*)(&serv_addr.sin_addr.s_addr);
    logInfo("TcpConnection - server: %d.%d.%d.%d : %d", (int)(a[0]), (int)a[1], (int)a[2], (int)a[3], serv_addr.sin_port);

    serv_addr.sin_port = htons(credentials.port);

    logInfo("TcpConnection - port: %d / %d", serv_addr.sin_port, credentials.port);

    int result = ::connect(sockfd, (sockaddr*)&serv_addr, sizeof(serv_addr));
    if (result < 0) {
        logError("TcpConnection - ERROR connecting");
        return false;
    }

    logInfo("TcpConnection - connected saccessfull to: %s:%d", credentials.address.c_str(), credentials.port);

    return true;
}

void TcpConnection::disconnect()
{
    sockfd = -1;
}

bool TcpConnection::isConnected() const
{
    return sockfd >= 0;
}

int TcpConnection::send(const unsigned char* data, int size)
{
    return ::send(sockfd, data, size, 0);
}

static int checkRead(int socketFd, int millisecond)
{
    fd_set socks;
    FD_ZERO(&socks);
    FD_SET(socketFd, &socks);

    timeval timeout;
    timeout.tv_sec = millisecond / 1000;
    timeout.tv_usec = millisecond % 1000;
    
    const int res = select(socketFd + 1, &socks, 0, 0, &timeout);
    if(res == SO_ERROR) {
        //log("select cocket ERROR!!\n");
        //log_error();

        return -1;
    }

    return FD_ISSET(socketFd, &socks) != 0;
}

int TcpConnection::read(unsigned char* buf, int size)
{
    const int res = checkRead(sockfd, 10);
    if (res < 0) {
        logError("TcpConnection - socket error");
        return -1;
    }

    const int receivedBytes = recv(sockfd, buf, size, 0);

    return receivedBytes;
}
