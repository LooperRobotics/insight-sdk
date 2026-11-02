// Copyright (c) 2026 Looper Robotics. All rights reserved.
// Licensed under the MIT License.

// LooperSDK example: IMU and VIO pose through per-stream callbacks.
//
// Callbacks run on each stream's receive thread, so this keeps the latest
// sample under a mutex and prints it from the main thread once a second.

#include <looper/stream.hpp>

#include <chrono>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <thread>

int main(int argc, char **argv) try {
    if(argc < 2) {
        std::cerr << "usage: " << argv[0] << " <camera-ip>" << std::endl;
        return 1;
    }
    lp::Device dev(argv[1]);

    std::mutex mutex;
    lp::Frame  imu, pose;
    dev.start(lp::Stream::Imu, [&](const lp::Frame &f) {
        std::lock_guard<std::mutex> lock(mutex);
        imu = f;
    });
    dev.start(lp::Stream::Pose, [&](const lp::Frame &f) {
        std::lock_guard<std::mutex> lock(mutex);
        pose = f;
    });

    for(int i = 0; i < 10 && dev.connected(); i++) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        const lp::StreamStats si = dev.stats(lp::Stream::Imu);
        const lp::StreamStats sp = dev.stats(lp::Stream::Pose);

        std::lock_guard<std::mutex> lock(mutex);
        if(imu) {
            const lp::ImuData d = imu.imu();
            std::printf("imu  #%-7llu acc [%7.3f %7.3f %7.3f] gyr [%7.4f %7.4f %7.4f]\n",
                        (unsigned long long)si.frames, d.acc[0], d.acc[1], d.acc[2],
                        d.gyr[0], d.gyr[1], d.gyr[2]);
        }
        if(pose) {
            const lp::PoseData p = pose.pose();
            if(lp::isTracking(p)) {
                std::printf("pose #%-7llu p [%7.3f %7.3f %7.3f] q [%.4f %.4f %.4f %.4f]\n",
                            (unsigned long long)sp.frames, p.position[0], p.position[1],
                            p.position[2], p.orientation[0], p.orientation[1],
                            p.orientation[2], p.orientation[3]);
            } else {
                std::printf("pose #%-7llu not tracking (status %d)\n",
                            (unsigned long long)sp.frames, p.status);
            }
        }
    }
    return 0;
}
catch(const lp::Error &e) {
    std::cerr << "Insight SDK error: " << e.what() << std::endl;
    return 1;
}
