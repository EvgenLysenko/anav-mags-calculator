#include "connection.h"
#include "tcp_connection.h"
#include "com_connection.h"

const std::string Connection::Credentials::TCP = "tcp";
const std::string Connection::Credentials::COM = "com";

Connection::Connection(const Credentials& credentials)
:credentials(credentials)
{
    
}

Connection* ConnectionFactory::create(const Connection::Credentials& credentials)
{
    Connection* connection = 0;

    if (credentials.type == Connection::Credentials::TCP)
        connection = new TcpConnection(credentials);
    else if (credentials.type == Connection::Credentials::COM)
        connection = new ComConnection(credentials);

    return connection;
}
