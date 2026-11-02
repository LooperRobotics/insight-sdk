// Copyright (c) 2026 Looper Robotics. All rights reserved.
// Licensed under the MIT License.

// LooperSDK example: live view of every image stream with OpenCV.
//
//   viewer <camera-ip>      q or Esc to quit
//
// Depth is shown 0-5 m in a colour map, black where there is no depth.

#include <looper/stream.hpp>

#include <opencv2/opencv.hpp>

#include <iostream>

static cv::Mat toImage(const lp::Frame &f) {
    if(!f) {
        return cv::Mat();
    }
    const int h = static_cast<int>(f.height());
    const int w = static_cast<int>(f.width());
    void     *p = const_cast<uint8_t *>(f.data());
    switch(f.format()) {
    case lp::Format::Gray8:
        return cv::Mat(h, w, CV_8UC1, p, f.stride()).clone();
    case lp::Format::Z16: {
        cv::Mat depth(h, w, CV_16UC1, p, f.stride()), d8, out;
        depth.convertTo(d8, CV_8UC1, 255.0 / 5000.0);
        cv::applyColorMap(d8, out, cv::COLORMAP_JET);
        out.setTo(cv::Scalar(0, 0, 0), depth == 0);
        return out;
    }
    case lp::Format::Mjpeg:
        return cv::imdecode(cv::Mat(1, static_cast<int>(f.size()), CV_8UC1, p), cv::IMREAD_COLOR);
    case lp::Format::Nv12: {
        cv::Mat out;
        cv::cvtColor(cv::Mat(h * 3 / 2, w, CV_8UC1, p), out, cv::COLOR_YUV2BGR_NV12);
        return out;
    }
    default:
        return cv::Mat();
    }
}

int main(int argc, char **argv) try {
    if(argc < 2) {
        std::cerr << "usage: " << argv[0] << " <camera-ip>" << std::endl;
        return 1;
    }
    lp::Device   dev(argv[1]);
    lp::Pipeline pipe(dev);
    lp::Config   cfg;
    cfg.enableAllImageStreams();
    pipe.start(cfg);

    for(;;) {
        const lp::FrameSet fs = pipe.waitForFrames(100);
        if(fs) {
            for(lp::Stream s : {lp::Stream::IrLeft, lp::Stream::IrRight, lp::Stream::Depth, lp::Stream::Color}) {
                const cv::Mat img = toImage(fs.get(s));
                if(!img.empty()) {
                    cv::imshow(lp::streamName(s), img);
                }
            }
        }
        const int key = cv::waitKey(1) & 0xFF;
        if(key == 'q' || key == 27 || !dev.connected()) {
            break;
        }
    }
    return 0;
}
catch(const lp::Error &e) {
    std::cerr << "Insight SDK error: " << e.what() << std::endl;
    return 1;
}
