#include <glog/logging.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <common/common.hpp>

using namespace std;
using namespace fmt;
using namespace common;

string getMacAddress(const string& netName) {
    struct ifreq ifr;
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    CHECK(fd >= 0) << "cannot open socket";

    strcpy(ifr.ifr_name, netName.c_str());
    if (ioctl(fd, SIOCGIFHWADDR, &ifr) < 0) {
        LOG(FATAL) << "ioctl error";
    }

    unsigned char* mac = reinterpret_cast<unsigned char*>(ifr.ifr_hwaddr.sa_data);
    auto macAddress = fmt::format("{:02X}:{:X}:{:X}:{:X}:{:X}:{:X}", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    LOG(INFO) << format("mac address of {}: {}", netName, macAddress);
    close(fd);

    return macAddress;
}

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    auto mac1 = getMacAddress("wlp0s20f3");

    closeLog();
    return 0;
};
