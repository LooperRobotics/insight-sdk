// Copyright (c) 2026 Looper Robotics. All rights reserved.
// Licensed under the MIT License.

// LooperSDK example: list every camera on the local segment.
//
// Discovery is a UDP broadcast, so it finds cameras that are not routable
// from this host as well, such as one on another subnet, which can then be
// fixed with the forceip example.

#include <looper/looper.hpp>

#include <iomanip>
#include <iostream>

int main(int argc, char **argv) try {
    // Optional argument: the local address to broadcast from. Needed when the
    // host has several NICs and the cameras are not on the default route.
    const std::string bindAddress = (argc > 1) ? argv[1] : "";

    lp::Context ctx(bindAddress);
    std::cout << "Searching for cameras";
    if(!bindAddress.empty()) {
        std::cout << " via " << bindAddress;
    }
    std::cout << " ..." << std::endl;

    const auto devices = ctx.discover(/*timeoutMs=*/1500);
    if(devices.empty()) {
        std::cout << "No cameras found.\n\n"
                  << "Things worth checking:\n"
                  << "  - the camera and this host are on the same switch\n"
                  << "  - the camera is powered on and has finished booting\n"
                  << "  - with several NICs, pass the local address: "
                  << argv[0] << " 192.168.1.1" << std::endl;
        return 1;
    }

    std::cout << "\nFound " << devices.size() << " camera(s):\n" << std::endl;
    std::cout << std::left
              << std::setw(3)  << "#"
              << std::setw(12) << "MODEL"
              << std::setw(14) << "SERIAL"
              << std::setw(19) << "MAC"
              << std::setw(16) << "IP"
              << std::setw(16) << "NETMASK"
              << std::setw(12) << "MODE"
              << std::setw(8)  << "CLOCK" << std::endl;
    std::cout << std::string(100, '-') << std::endl;

    int index = 0;
    for(const auto &d: devices) {
        std::cout << std::left
                  << std::setw(3)  << index++
                  << std::setw(12) << d.model
                  << std::setw(14) << d.serial
                  << std::setw(19) << d.mac
                  << std::setw(16) << d.ip
                  << std::setw(16) << d.netmask
                  << std::setw(12) << d.ipModeString()
                  << std::setw(8)  << d.timeSyncString() << std::endl;
    }

    // A camera on link-local fell back there because DHCP timed out and its
    // static address was already taken by another unit on the segment.
    for(const auto &d: devices) {
        if(d.ipMode == lp::IpMode::LinkLocal) {
            std::cout << "\nNote: " << d.serial << " is on a link-local address, so DHCP"
                      << " timed out and its static address was already in use."
                      << " Give it an address of its own with the forceip example."
                      << std::endl;
            break;
        }
    }
    // Timestamps from an unsynchronised camera share no base with the others,
    // so anything that fuses data across cameras has to know.
    for(const auto &d: devices) {
        if(d.timeSync == lp::TimeSync::No) {
            std::cout << "\nNote: " << d.serial << " has an unsynchronised clock;"
                      << " its timestamps cannot be compared with the other cameras'."
                      << std::endl;
            break;
        }
    }
    return 0;
}
catch(const lp::Error &e) {
    std::cerr << "Insight SDK error: " << e.what() << std::endl;
    return 1;
}
