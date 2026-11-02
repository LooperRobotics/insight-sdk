// Copyright (c) 2026 Looper Robotics. All rights reserved.
// Licensed under the MIT License.

// LooperSDK example: time-matched image FrameSets from a Pipeline.
//
// Prints each set's frames and how far color is from the stereo capture,
// then writes the last set to disk: IR as PGM, depth as 16-bit PGM
// (millimetres), color as the JPEG the camera sent.

#include <looper/stream.hpp>

#include <cstdio>
#include <fstream>
#include <iostream>

static void writePgm(const std::string &path, const lp::Frame &f) {
    std::ofstream out(path, std::ios::binary);
    const bool    wide = f.format() == lp::Format::Z16;
    out << "P5\n" << f.width() << " " << f.height() << "\n" << (wide ? 65535 : 255) << "\n";
    for(uint32_t y = 0; y < f.height(); y++) {
        const uint8_t *row = f.data() + static_cast<size_t>(y) * f.stride();
        if(!wide) {
            out.write(reinterpret_cast<const char *>(row), f.width());
            continue;
        }
        for(uint32_t x = 0; x < f.width(); x++) {  // PGM is big-endian
            const char be[2] = {static_cast<char>(row[2 * x + 1]), static_cast<char>(row[2 * x])};
            out.write(be, 2);
        }
    }
}

int main(int argc, char **argv) try {
    if(argc < 2) {
        std::cerr << "usage: " << argv[0] << " <camera-ip>" << std::endl;
        return 1;
    }
    lp::Device dev(argv[1]);
    const lp::Intrinsics in = dev.intrinsics(lp::Stream::Depth);
    std::printf("depth intrinsics (rectified): %ux%u fx %.2f fy %.2f cx %.2f cy %.2f\n",
                in.width, in.height, in.p[0], in.p[5], in.p[2], in.p[6]);

    lp::Config cfg;
    cfg.enableAllImageStreams();
    lp::Pipeline pipe(dev);
    pipe.start(cfg);

    lp::FrameSet last;
    for(int i = 0; i < 30; i++) {
        lp::FrameSet fs = pipe.waitForFrames(2000);
        if(!fs) {
            std::cerr << "no frameset within 2 s" << std::endl;
            return 1;
        }
        const lp::Frame depth = fs.depth();
        const lp::Frame color = fs.color();
        const double    colorOffsetMs =
            (static_cast<double>(color.timestamp()) - static_cast<double>(fs.timestamp())) / 1e6;
        std::printf("set t=%.3f s  ir %ux%u  depth %ux%u center %u mm  color %ux%u %+.1f ms\n",
                    fs.timestamp() / 1e9, fs.irLeft().width(), fs.irLeft().height(),
                    depth.width(), depth.height(), depth.depthAt(depth.width() / 2, depth.height() / 2),
                    color.width(), color.height(), colorOffsetMs);
        last = fs;
    }

    writePgm("ir_left.pgm", last.irLeft());
    writePgm("ir_right.pgm", last.irRight());
    writePgm("depth.pgm", last.depth());
    std::ofstream("color.jpg", std::ios::binary)
        .write(reinterpret_cast<const char *>(last.color().data()),
               static_cast<std::streamsize>(last.color().size()));
    std::cout << "wrote ir_left.pgm ir_right.pgm depth.pgm color.jpg" << std::endl;
    return 0;
}
catch(const lp::Error &e) {
    std::cerr << "Insight SDK error: " << e.what() << std::endl;
    return 1;
}
