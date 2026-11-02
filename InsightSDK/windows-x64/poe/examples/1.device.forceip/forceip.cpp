// Copyright (c) 2026 Looper Robotics. All rights reserved.
// Licensed under the MIT License.

// LooperSDK example: give a network camera a static address.
//
// It works even when the camera is unreachable - wrong address, wrong subnet,
// no route from this host - because the request is a broadcast addressed by
// MAC rather than by IP.
//
// Supported modes:
//   static     persistent, survives reboot
//   temporary  reverts on the next reboot
//   dhcp       return the camera to DHCP

#include <looper/looper.hpp>

#include <iomanip>
#include <iostream>
#include <string>

namespace {

void usage(const char *argv0) {
    std::cout <<
        "Usage:\n"
        "  " << argv0 << " list [bind-addr]\n"
        "  " << argv0 << " static    <mac> <ip> [netmask] [gateway] [bind-addr]\n"
        "  " << argv0 << " temporary <mac> <ip> [netmask] [gateway] [bind-addr]\n"
        "  " << argv0 << " dhcp      <mac> [bind-addr]\n"
        "\n"
        "Examples:\n"
        "  " << argv0 << " list\n"
        "  " << argv0 << " static 00:11:22:33:44:55 192.168.1.50 255.255.255.0 192.168.1.1\n"
        "  " << argv0 << " temporary 00:11:22:33:44:55 192.168.1.99\n"
        "  " << argv0 << " dhcp 00:11:22:33:44:55\n"
        << std::endl;
}

void printDevices(const std::vector<lp::DeviceInfo> &devices) {
    if(devices.empty()) {
        std::cout << "No cameras found." << std::endl;
        return;
    }
    for(const auto &d: devices) {
        std::cout << "  " << std::left << std::setw(19) << d.mac
                  << std::setw(14) << d.serial
                  << std::setw(16) << d.ip
                  << d.ipModeString() << std::endl;
    }
}

}  // namespace

int main(int argc, char **argv) try {
    if(argc < 2) {
        usage(argv[0]);
        return 1;
    }
    const std::string cmd = argv[1];

    if(cmd == "list") {
        lp::Context ctx(argc > 2 ? argv[2] : "");
        printDevices(ctx.discover());
        return 0;
    }

    if(cmd == "dhcp") {
        if(argc < 3) {
            usage(argv[0]);
            return 1;
        }
        const std::string mac = argv[2];
        lp::Context ctx(argc > 3 ? argv[3] : "");
        ctx.forceIp(mac, lp::IpConfig::Dhcp());
        std::cout << mac << ": switched to DHCP (takes effect on the next boot)" << std::endl;
        return 0;
    }

    if(cmd != "static" && cmd != "temporary") {
        usage(argv[0]);
        return 1;
    }
    if(argc < 4) {
        usage(argv[0]);
        return 1;
    }

    const std::string mac        = argv[2];
    const std::string ip         = argv[3];
    const std::string netmask    = (argc > 4) ? argv[4] : "255.255.255.0";
    const std::string gateway    = (argc > 5) ? argv[5] : "";
    const std::string bindAddr   = (argc > 6) ? argv[6] : "";
    const bool        persistent = (cmd == "static");

    lp::Context ctx(bindAddr);

    std::cout << "Before:" << std::endl;
    printDevices(ctx.discover());

    const auto config = persistent ? lp::IpConfig::Static(ip, netmask, gateway)
                                   : lp::IpConfig::Temporary(ip, netmask, gateway);
    ctx.forceIp(mac, config);

    std::cout << "\n" << mac << " -> " << ip << "/" << netmask;
    if(!gateway.empty()) {
        std::cout << " gw " << gateway;
    }
    std::cout << (persistent ? "  [persistent]" : "  [temporary, reverts on reboot]") << std::endl;

    // The camera acknowledges before it applies the change, so confirm by
    // discovering again rather than trusting the acknowledgement alone.
    std::cout << "\nRe-checking ..." << std::endl;
    printDevices(ctx.discover(/*timeoutMs=*/3000));
    return 0;
}
catch(const lp::Error &e) {
    std::cerr << "Insight SDK error: " << e.what() << std::endl;
    return 1;
}
