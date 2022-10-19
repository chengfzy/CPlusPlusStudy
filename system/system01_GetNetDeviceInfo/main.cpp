#include <arpa/inet.h>
#include <glog/logging.h>
#include <net/if.h>
#include <netinet/if_ether.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <common/common.hpp>

using namespace std;
using namespace fmt;
using namespace common;

void showAllInfo() {
    LOG(INFO) << Section("Get All Network Info");
    ifconf ifc;
    ifreq ifr[10];
    memset(&ifc, 0, sizeof(ifconf));

    // open socket
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    CHECK(fd >= 0) << "cannot open socket";

    ifc.ifc_len = 10 * sizeof(ifreq);
    ifc.ifc_ifcu.ifcu_buf = (char*)ifr;
    ioctl(fd, SIOCGIFCONF, (char*)&ifc);
    int ifCount = ifc.ifc_len / sizeof(ifreq);
    for (int i = 0; i < ifCount; ++i) {
        // get IPv4 address
        char ipAddr[INET_ADDRSTRLEN] = {'\0'};
        inet_ntop(AF_INET, &(reinterpret_cast<sockaddr_in*>(&(ifr[i].ifr_ifru.ifru_addr))->sin_addr), ipAddr,
                  INET_ADDRSTRLEN);
        LOG(INFO) << format("interface {}, IPv4: {}", ifr[i].ifr_ifrn.ifrn_name, ipAddr);
    }

    close(fd);
}

void showAllMacAddress() {
    LOG(INFO) << Section("Get All Network Mac Address");

    // open socket
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    CHECK(fd >= 0) << "cannot open socket";

    // get device count
    ifconf ifc;
    char buf[1024];
    ifc.ifc_len = sizeof(buf);
    ifc.ifc_ifcu.ifcu_buf = buf;
    if (ioctl(fd, SIOCGIFCONF, &ifc) == -1) {
        LOG(ERROR) << "ioctrl error";
    }
    int ifCount = ifc.ifc_len / sizeof(ifreq);
    for (auto it = ifc.ifc_ifcu.ifcu_req; it != ifc.ifc_ifcu.ifcu_req + ifc.ifc_len / sizeof(ifreq); ++it) {
        ifreq ifr;
        strncpy(ifr.ifr_ifrn.ifrn_name, it->ifr_ifrn.ifrn_name, IFNAMSIZ);

        // get IPv4 address
        string ipAddress;
        if (ioctl(fd, SIOCGIFADDR, &ifr) == 0) {
            char ipAddr[INET_ADDRSTRLEN] = {'\0'};
            inet_ntop(AF_INET, &(reinterpret_cast<sockaddr_in*>(&(ifr.ifr_ifru.ifru_addr))->sin_addr), ipAddr,
                      INET_ADDRSTRLEN);
            ipAddress = ipAddr;
        }

        // get mac address
        string macAddress;
        if (ioctl(fd, SIOCGIFHWADDR, &ifr) == 0) {
            unsigned char* mac = reinterpret_cast<unsigned char*>(ifr.ifr_ifru.ifru_hwaddr.sa_data);
            macAddress =
                format("{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        }

        LOG(INFO) << format("interface {}, IPv4: {}, mac: {}", it->ifr_ifrn.ifrn_name, ipAddress, macAddress);
    }

    close(fd);
}

void getIpAddress(const string& device) {
    ifconf ifc;
    memset(&ifc, 0, sizeof(ifconf));

    // open socket
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    CHECK(fd >= 0) << "cannot open socket";

    ifreq ifr;
    memset(&ifr, 0, sizeof(ifreq));
    ifr.ifr_ifru.ifru_addr.sa_family = AF_INET;
    strncpy(ifr.ifr_ifrn.ifrn_name, device.c_str(), IFNAMSIZ);

    int ret = ioctl(fd, SIOCGIFADDR, &ifr);
    if (ret == -1) {
        if (errno == 19) {
            LOG(ERROR) << format("no such device: {}", device);
            return;
        } else if (errno == 99) {
            LOG(ERROR) << format("no IPv4 address assigned to device: {}", device);
            return;
        }
    }

    char ipAddr[INET_ADDRSTRLEN] = {'\0'};
    inet_ntop(AF_INET, &(reinterpret_cast<sockaddr_in*>(&(ifr.ifr_ifru.ifru_addr))->sin_addr), ipAddr, INET_ADDRSTRLEN);
    close(fd);

    string ipAddress(ipAddr);
    LOG(INFO) << format("{}: {}", device, ipAddress);
}

void getMacAddress(const string& device) {
    // open socket
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    CHECK(fd >= 0) << "cannot open socket";

    ifreq ifr;
    strncpy(ifr.ifr_name, device.c_str(), IFNAMSIZ);
    if (ioctl(fd, SIOCGIFHWADDR, &ifr) == -1 && errno == 10) {
        LOG(ERROR) << format("no such device: {}", device);
        return;
    }

    if (ifr.ifr_ifru.ifru_addr.sa_family == ARPHRD_LOOPBACK) {
        LOG(WARNING) << format("a loopback device: {}, mac address is always: 00:00:00:00:00:00", device);
        return;
    }

    if (ifr.ifr_ifru.ifru_addr.sa_family != ARPHRD_ETHER) {
        LOG(WARNING) << format("not an ethernet device: {}", device);
        return;
    }

    unsigned char* mac = reinterpret_cast<unsigned char*>(ifr.ifr_hwaddr.sa_data);
    auto macAddress =
        fmt::format("{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    LOG(INFO) << format("mac address of {}: {}", device, macAddress);
    close(fd);
}

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    showAllInfo();

    showAllMacAddress();

    // get IPv4 address
    LOG(INFO) << Section("Get IPv4 Address of Network Device");
    getIpAddress("enp0s31f6");
    getIpAddress("wlp0s20f3");
    getIpAddress("docker0");
    getIpAddress("lo");

    // get mac address
    LOG(INFO) << Section("Get Mac Address of Network Device");
    getMacAddress("enp0s31f6");
    getMacAddress("wlp0s20f3");
    getMacAddress("docker0");
    getMacAddress("lo");

    closeLog();
    return 0;
};
