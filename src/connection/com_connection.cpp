#include "com_connection.h"
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include "logging.h"

static int open_port(const char* port)
{
    // Open serial port
    // O_RDWR - Read and write
    // O_NOCTTY - Ignore special chars like CTRL-C
    // fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    int fd = open(port, O_RDWR | O_NOCTTY);

    // Check for Errors
    if (fd == -1) {
        /* Could not open the port. */
        return (-1);
    }
    // Finalize
    else {
        fcntl(fd, F_SETFL, 0);
    }

    // Done!
    return fd;
} 

static int set_interface_attribs(int fd, int speed, int parity)
{
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        logError("ComConnection - error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0) {
        logError("ComConnection - error %d from tcsetattr", errno);
        return -1;
    }

    return 0;
}

static void set_blocking(int fd, int should_block)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0) {
        logError ("ComConnection - error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
        logError ("ComConnection - error %d setting term attributes", errno);
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

    bool success = setup_port(credentials.port, 8, 1, false, false); 
    //set_interface_attribs(fd, credentials.port, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    //set_blocking(fd, 0);                // set no blocking
    if (!success)
    {
        logError("ComConnection - failure, could not configure port. Exiting");
        //exit(1);
    }

    // if (fd <= 0)
    // {
    //   logError("Connection attempt to port ``%s'' with baud ``%d'', 8N1 failed",
    //   addr.c_str(), conn_param); status = CONNECTOR_ERROR;
    // }
    else
    {
        logInfo("ComConnection - connected to port ``%s'' with baud ``%d'', 8 data bits, no parity, 1 stop bit (8N1)", credentials.address.c_str(), credentials.port);
    } 
    return success;
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

    int bytesSent = write(fd, data, size);

    return bytesSent;
}

int ComConnection::read(unsigned char* buf, int size)
{
    if (fd < 0)
        return -1;

    int bytesReceived = ::read(fd, buf, size);

    return bytesReceived;
}

bool ComConnection::setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
    // Check file descriptor
    if (!isatty(fd)) {
        logError("ComConnection - file descriptor ``%d'' is NOT a serial port", fd);
        return false;
    }

    // Read file descritor configuration
    struct termios config;
    if (tcgetattr(fd, &config) < 0) {
        logError("ComConnection - could not read configuration of fd ``%d''", fd);
        return false;
    }

    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

#ifdef OLCUC
    config.c_oflag &= ~OLCUC;
#endif

#ifdef ONOEOT
    config.c_oflag &= ~ONOEOT;
#endif

    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;

    // One input byte is enough to return from read()
    // Inter-character timer off
    config.c_cc[VMIN] = 0;
    config.c_cc[VTIME] = 0; // was 0

    // Get the current options for the port
    ////struct termios options;
    ////tcgetattr(fd, &options);

    // Apply conn_param
    switch (baud) {
    case 1200:
        if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0) {
            logError("ComConnection - could not set desired baud rate of ``%d'' Baud", baud);
            return false;
        }
        break;
    case 1800:
        cfsetispeed(&config, B1800);
        cfsetospeed(&config, B1800);
        break;
    case 9600:
        cfsetispeed(&config, B9600);
        cfsetospeed(&config, B9600);
        break;
    case 19200:
        cfsetispeed(&config, B19200);
        cfsetospeed(&config, B19200);
        break;
    case 38400:
        if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0) {
            logError("ComConnection - could not set desired baud rate of ``%d'' Baud", baud);
            return false;
        }
        break;
    case 57600:
        if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0) {
            logError("ComConnection - could not set desired baud rate of ``%d'' Baud", baud);
            return false;
        }
        break;
    case 115200:
        if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0) {
            logError("ComConnection - could not set desired baud rate of ``%d'' Baud", baud);
            return false;
        }
        break;

    // These two non-standard (by the 70'ties ) rates are fully supported on
    // current Debian and Mac OS versions (tested since 2010).
    case 460800:
        if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0) {
            logError("ComConnection - could not set desired baud rate of ``%d'' Baud", baud);
            return false;
        }
        break;
    case 921600:
        if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0) {
            logError("ComConnection - could not set desired baud rate of ``%d'' Baud", baud);
            return false;
        }
        break;
    default:
        logError("ComConnection - desired baud rate ``%d'' could not be set", baud);
        return false;
    }

    // Finally, apply the configuration
    if (tcsetattr(fd, TCSAFLUSH, &config) < 0) {
        logError("ComConnection - could not set configuration of fd ``%d''", fd);
        return false;
    }

    // Done!
    return true;
}
