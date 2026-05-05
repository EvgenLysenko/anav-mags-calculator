#include <iostream>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include "logging.h"
#include "config.h"
#include "utils/StringUtils.h"

uint32_t getIp()
{
    uint32_t ip = 0;

    struct ifaddrs *ifaddr, *ifa;
    char host[NI_MAXHOST];

    if (getifaddrs(&ifaddr) == -1) {
        logError("getifaddrs");
        return ip;
    }

    for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr)
            continue;

        int family = ifa->ifa_addr->sa_family;

        if (family == AF_INET) {// || family == AF_INET6) {
            int s = getnameinfo(ifa->ifa_addr,
                                (family == AF_INET) ? sizeof(struct sockaddr_in) : sizeof(struct sockaddr_in6),
                                host, NI_MAXHOST,
                                nullptr, 0, NI_NUMERICHOST);
            if (s != 0) {
                logError("getnameinfo() failed: %s", gai_strerror(s));
                continue;
            }
 
            logInfo("Interface: %s Address: %s", ifa->ifa_name, host);
            logInfo("size: %d", (int)sizeof(ifa->ifa_addr->sa_data));
            if (StringUtils::contains((const unsigned char*)ifa->ifa_name, 4, (const unsigned char*)"eth0", 4) != 0 ||
                StringUtils::contains((const unsigned char*)ifa->ifa_name, 5, (const unsigned char*)"wlan0", 5) != 0
            ) {
                if (sizeof(ifa->ifa_addr->sa_data) >= 6) {
                    const unsigned char* a = (unsigned char*)(ifa->ifa_addr->sa_data);
                    logInfo("ip detected: %d.%d.%d.%d", (int)a[2], (int)a[3], (int)(a[4]), (int)a[5]);
                    ip = (a[2] | (a[3] << 8) | (a[4] << 16) | (a[5] << 24)) & 0xFFFFFFFF;
                }
            }
        }
    }

    freeifaddrs(ifaddr);

    return ip;
}
