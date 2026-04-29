#include "com_connection.h"
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include "logging.h"

static int boudMap[][2] = {
    {1200, B1200},
    {1800, B1800},
    {9600, B9600},
    {19200, B19200},
    {38400, B38400},
    {57600, B57600},
    {115200, B115200},
    {230400, B230400},
    {460800, B460800},
    {921600, B921600},
};

static int set_boud_rate(struct termios& config, int baud)
{
    //baud = 115115;
    speed_t boudFound = 0;

    for (int i = 0; i < sizeof(boudMap)/sizeof(*boudMap); ++i) {
        if (boudMap[i][0] == baud) {
            boudFound = boudMap[i][1];
            break;
        }
    }

    if (!boudFound) {
        logError("ComConnection - unexpected baud rate:  %d", baud);
        boudFound = baud;
        logWarning("ComConnection - trying to set baud rate:  %d", boudFound);
    }

    int result = cfsetispeed(&config, boudFound);
    if (result < 0) {
        logError("ComConnection - could not set desired baud rate: %d (%d) - cfsetispeed", baud, boudFound);
        return result;
    }

    result = cfsetospeed(&config, boudFound);
    if (result < 0) {
        logError("ComConnection - could not set desired baud rate: %d (%d) - cfsetospeed", baud, boudFound);
        return result;
    }

    return result;
}

static int set_interface_attribs(int fd, int speed, int parity)
{
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        logError("ComConnection - error %d from tcgetattr", errno);
        return -1;
    }

    if (set_boud_rate(tty, speed) < 0) {
        logError("ComConnection - error set baud rate: %d", speed);
        return -1;
    }

    // disable IGNBRK for mismatched speed tests; otherwise receive break as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 0;            // 0 seconds read timeout

    // no XON/XOFF software flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    tty.c_iflag &= ~(BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars

    tty.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);  // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    int res = tcsetattr (fd, TCSANOW, &tty);
    if (res != 0) {
        logError("ComConnection - error %d from tcsetattr", errno);
        return res;
    }

    return res;
}

ComConnection::ComConnection(const Connection::Credentials& credentials)
    : Connection(credentials)
{
    fd = -1;
}

bool ComConnection::connect()
{
    logInfo("ComConnection - connect serial: address: %s / %d", credentials.address.c_str(), credentials.port);

    fd = open(credentials.address.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        logError("ComConnection - error %d opening serial %s: %s", errno, credentials.address.c_str(), strerror(errno));
        return false;
    }

    bool result = set_interface_attribs(fd, credentials.port, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    if (result != 0) {
        logError("ComConnection - failure, could not configure port. Exiting");
    }
    else {
        logInfo("ComConnection - connected to port %s with baud %d, 8 data bits, no parity, 1 stop bit (8N1)", credentials.address.c_str(), credentials.port);
    } 

    //tcflush(fd, TCIFLUSH);

    return result == 0;
}

void ComConnection::disconnect()
{
    if (fd >= 0)
        close(fd);
}

bool ComConnection::isConnected() const
{
    return fd >= 0;
}

int ComConnection::send(const unsigned char* data, int size)
{
    if (fd < 0)
        return -1;

    int bytesSent = ::write(fd, data, size);

    return bytesSent;
}

int ComConnection::read(unsigned char* buf, int size)
{
    if (fd < 0)
        return -1;

    int bytesReceived = ::read(fd, buf, size);

    return bytesReceived;
}
