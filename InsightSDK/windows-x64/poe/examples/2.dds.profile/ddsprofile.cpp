// Copyright (c) 2026 Looper Robotics. All rights reserved.
// Licensed under the MIT License.

// LooperSDK example: pin Fast DDS to the interface that reaches the cameras.
//
// The cameras publish over ROS 2. Fast DDS announces one locator per local
// interface, and a camera sends a copy of every sample to each announced
// locator, so a host with several interfaces (Ethernet, Wi-Fi, VPN, ...)
// multiplies the bandwidth every camera has to send. Restricting DDS to the
// interface that reaches the cameras avoids the duplicates.

#include <looper/looper.hpp>

#include <iostream>

int main(int argc, char **argv) try {
    const std::string bindAddress = (argc > 1) ? argv[1] : "";
    const std::string outPath     = (argc > 2) ? argv[2] : "looper_fastdds_profile.xml";

    lp::Context ctx(bindAddress);
    std::cout << "Searching for cameras ..." << std::endl;

    const auto devices = ctx.discover(/*timeoutMs=*/1500);
    if(devices.empty()) {
        std::cout << "No cameras found, so there is no interface to pin to.\n"
                  << "Connect a camera first, or name the NIC: "
                  << argv[0] << " 192.168.1.1" << std::endl;
        return 1;
    }

    // One route lookup per camera, rather than guessing from the NIC list.
    // Cameras on a second segment contribute a second address.
    const auto addresses = lp::localAddressesFor(devices);
    if(addresses.empty()) {
        std::cout << "Found " << devices.size()
                  << " camera(s) but none of them is routable from here."
                  << std::endl;
        return 1;
    }

    lp::writeDdsProfile(outPath, addresses);

    std::cout << "\nFound " << devices.size() << " camera(s), pinning DDS to";
    for(const auto &a: addresses) {
        std::cout << " " << a;
    }
    std::cout << "\nWrote " << outPath << "\n\n"
              << "Export this before starting any subscriber, including the\n"
              << "ros2 CLI - Fast DDS reads it once, when the participant is\n"
              << "created, so a process already running will not pick it up:\n\n"
              << "  export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/" << outPath
              << std::endl;
    return 0;
}
catch(const lp::Error &e) {
    std::cerr << "Insight SDK error: " << e.what() << std::endl;
    return 1;
}
