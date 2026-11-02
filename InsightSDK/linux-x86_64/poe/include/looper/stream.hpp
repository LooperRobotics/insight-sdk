// Copyright (c) 2026 Looper Robotics. All rights reserved.
// Licensed under the MIT License.

/**
 * @file stream.hpp
 * @brief LooperSDK C++ API: streaming.
 *
 * Header-only wrapper over stream.h. Two ways to receive data:
 *
 * Per-stream callbacks, for IMU and pose, or any stream on its own:
 * @code
 *   lp::Device dev("192.168.1.10");
 *   dev.start(lp::Stream::Imu, [](const lp::Frame &f) { use(f.imu()); });
 * @endcode
 *
 * A Pipeline, which groups the image streams into time-matched FrameSets:
 * @code
 *   lp::Pipeline pipe(dev);
 *   lp::Config cfg;
 *   cfg.enableStream(lp::Stream::Depth);
 *   cfg.enableStream(lp::Stream::Color);
 *   pipe.start(cfg);
 *   lp::FrameSet fs = pipe.waitForFrames(1000);
 *   if(fs) { use(fs.depth(), fs.color()); }
 * @endcode
 */

#ifndef LOOPER_STREAM_HPP
#define LOOPER_STREAM_HPP

#include "looper/looper.hpp"
#include "looper/stream.h"

#include <condition_variable>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace lp {

enum class Stream {
    IrLeft  = LP_STREAM_IR_LEFT,
    IrRight = LP_STREAM_IR_RIGHT,
    Depth   = LP_STREAM_DEPTH,
    Color   = LP_STREAM_COLOR,
    Imu     = LP_STREAM_IMU,
    Pose    = LP_STREAM_POSE
};

enum class Format {
    Unknown = LP_FORMAT_UNKNOWN,
    Gray8   = LP_FORMAT_GRAY8,
    Z16     = LP_FORMAT_Z16,
    Mjpeg   = LP_FORMAT_MJPEG,
    Nv12    = LP_FORMAT_NV12,
    Imu     = LP_FORMAT_IMU,
    Pose    = LP_FORMAT_POSE
};

enum class VioStatus {
    NotInited      = LP_VIO_NOT_INITED,
    Restarting     = LP_VIO_RESTARTING,
    Stopped        = LP_VIO_STOPPED,
    Tracking       = LP_VIO_TRACKING,
    TrackingStatic = LP_VIO_TRACKING_STATIC,
    TrackingLost   = LP_VIO_TRACKING_LOST,
    DataLost       = LP_VIO_DATA_LOST
};

typedef lp_imu_data     ImuData;
typedef lp_pose_data    PoseData;
typedef lp_intrinsics   Intrinsics;
typedef lp_stream_stats StreamStats;
typedef lp_extrinsic    Extrinsic;

inline const char *streamName(Stream s) { return lp_stream_name(static_cast<lp_stream>(s)); }
inline const char *formatName(Format f) { return lp_format_name(static_cast<lp_format>(f)); }

inline bool isTracking(const PoseData &p) {
    return p.status == LP_VIO_TRACKING || p.status == LP_VIO_TRACKING_STATIC;
}

/* A received frame. Cheap to copy: copies share one immutable buffer. An
 * empty Frame (no data) converts to false. */
class Frame {
public:
    Frame() {}

    /* Copies the data, so the Frame outlives the C callback it came from. */
    explicit Frame(const lp_frame *raw) : f_(lp_frame_clone(raw), lp_frame_release) {
        if(raw && !f_) {
            throw Error(LP_ERR_NOMEM, "could not copy a frame");
        }
    }

    explicit operator bool() const { return static_cast<bool>(f_); }

    Stream         stream() const { return static_cast<Stream>(get().stream); }
    Format         format() const { return static_cast<Format>(get().format); }
    uint32_t       width() const { return get().width; }
    uint32_t       height() const { return get().height; }
    uint32_t       stride() const { return get().stride; }
    const uint8_t *data() const { return get().data; }
    size_t         size() const { return get().size; }
    uint64_t       number() const { return get().number; }
    bool           rectified() const { return (get().flags & LP_FRAME_RECTIFIED) != 0; }
    bool           calibrationMode() const { return (get().flags & LP_FRAME_CALIBRATION) != 0; }
    /* IMU frames: true in the sensor's body axes, false in camera axes. */
    bool           imuRaw() const { return (get().flags & LP_FRAME_IMU_RAW) != 0; }

    /* Capture time on the camera's monotonic clock, shared by all its
     * streams: use it to match frames of one camera. */
    uint64_t timestamp() const { return get().timestamp_ns; }
    /* Capture time on the camera's wall clock: comparable across cameras
     * only when their clocks are synchronised. */
    uint64_t systemTimestamp() const { return get().system_timestamp_ns; }
    /* IR right only: the right eye's own capture time. */
    uint64_t rightTimestamp() const { return get().right_timestamp_ns; }

    /* Depth in millimetres at (x, y); Z16 frames only. */
    uint16_t depthAt(uint32_t x, uint32_t y) const {
        if(format() != Format::Z16 || x >= width() || y >= height()) {
            throw Error(LP_ERR_INVALID, "depthAt() outside a Z16 frame");
        }
        uint16_t v;
        std::memcpy(&v, data() + static_cast<size_t>(y) * stride() + x * 2u, sizeof(v));
        return v;
    }

    ImuData imu() const { return payload<ImuData>(Format::Imu); }
    PoseData pose() const { return payload<PoseData>(Format::Pose); }

    const lp_frame *raw() const { return f_.get(); }

private:
    const lp_frame &get() const {
        if(!f_) {
            throw Error(LP_ERR_INVALID, "empty frame");
        }
        return *f_;
    }

    template<typename T> T payload(Format expected) const {
        if(format() != expected || size() < sizeof(T)) {
            throw Error(LP_ERR_INVALID, std::string("frame is not ") + formatName(expected));
        }
        T out;
        std::memcpy(&out, data(), sizeof(out));
        return out;
    }

    std::shared_ptr<lp_frame> f_;
};

typedef std::function<void(const Frame &)> FrameCallback;

/* An open camera: one control session, any number of streams. */
class Device {
public:
    explicit Device(const std::string &ip, int timeoutMs = 2000, uint16_t port = LP_CONTROL_PORT)
        : dev_(nullptr, lp_device_close) {
        lp_device *raw = nullptr;
        lp_status  st  = lp_device_open_port(ip.c_str(), port, timeoutMs, &raw);
        if(st != LP_OK) {
            throw Error(st, lp_open_error());
        }
        dev_.reset(raw);
    }

    /* Open a camera from discovery. */
    explicit Device(const DeviceInfo &info, int timeoutMs = 2000) : Device(info.ip, timeoutMs) {}

    Device(Device &&) = default;
    Device &operator=(Device &&) = default;

    bool connected() const { return lp_device_is_connected(dev_.get()) != 0; }

    std::string infoJson() { return json(lp_device_get_info_json); }
    std::string calibrationJson() { return json(lp_device_get_calibration_json); }

    Intrinsics intrinsics(Stream s) {
        Intrinsics out;
        check(lp_device_get_intrinsics(dev_.get(), static_cast<lp_stream>(s), &out));
        return out;
    }

    std::vector<lp_extrinsic> extrinsics() {
        std::vector<lp_extrinsic> out(16);
        size_t                    n = 0;
        check(lp_device_get_extrinsics(dev_.get(), out.data(), out.size(), &n));
        if(n > out.size()) {
            out.resize(n);
            check(lp_device_get_extrinsics(dev_.get(), out.data(), out.size(), &n));
        }
        out.resize(n);
        return out;
    }

    /* Start a stream; callback runs on the stream's receive thread and must
     * return quickly. port 0 lets the system choose a free UDP port. */
    void start(Stream s, FrameCallback callback, uint16_t port = 0) {
        const size_t i = static_cast<size_t>(s);
        std::unique_ptr<FrameCallback> cb(new FrameCallback(std::move(callback)));
        check(lp_device_start_stream(dev_.get(), static_cast<lp_stream>(s), port, trampoline, cb.get()));
        callbacks_[i] = std::move(cb);
    }

    void stop(Stream s) {
        lp_status st = lp_device_stop_stream(dev_.get(), static_cast<lp_stream>(s));
        callbacks_[static_cast<size_t>(s)].reset();  // thread is joined by now
        if(st != LP_OK && st != LP_ERR_DISCONNECTED) {
            check(st);
        }
    }

    StreamStats stats(Stream s) {
        StreamStats out;
        check(lp_device_get_stream_stats(dev_.get(), static_cast<lp_stream>(s), &out));
        return out;
    }

    lp_device *raw() { return dev_.get(); }

private:
    static void trampoline(const lp_frame *frame, void *user) {
        try {
            (*static_cast<FrameCallback *>(user))(Frame(frame));
        } catch(...) {
            /* An exception must not unwind through the C receive thread. */
        }
    }

    void check(lp_status st) {
        if(st != LP_OK) {
            throw Error(st, lp_device_last_error(dev_.get()));
        }
    }

    std::string json(lp_status (*fn)(lp_device *, char *, size_t, size_t *)) {
        std::string buf(16384, '\0');
        size_t      n  = 0;
        lp_status   st = fn(dev_.get(), &buf[0], buf.size(), &n);
        if(st == LP_ERR_NOMEM && n > 0) {
            buf.assign(n + 1, '\0');
            st = fn(dev_.get(), &buf[0], buf.size(), &n);
        }
        check(st);
        buf.resize(n);
        return buf;
    }

    /* Declared before dev_ so they outlive it: lp_device_close() joins the
     * threads that call them. */
    std::unique_ptr<FrameCallback>                    callbacks_[LP_STREAM_COUNT];
    std::unique_ptr<lp_device, void (*)(lp_device *)> dev_;
};

/* Image frames of one instant. Absent streams return an empty Frame. */
class FrameSet {
public:
    explicit operator bool() const {
        for(const auto &f : frames_) {
            if(f) {
                return true;
            }
        }
        return false;
    }

    Frame get(Stream s) const { return frames_[static_cast<size_t>(s)]; }
    Frame irLeft() const { return get(Stream::IrLeft); }
    Frame irRight() const { return get(Stream::IrRight); }
    Frame depth() const { return get(Stream::Depth); }
    Frame color() const { return get(Stream::Color); }

    /* The stereo capture time the set was assembled around. */
    uint64_t timestamp() const { return timestamp_; }

private:
    friend class Pipeline;
    Frame    frames_[LP_STREAM_COUNT];
    uint64_t timestamp_ = 0;
};

/* Which image streams a Pipeline delivers. */
class Config {
public:
    /* IMU and pose are not part of FrameSets; use Device::start() for them. */
    void enableStream(Stream s) {
        if(s == Stream::Imu || s == Stream::Pose) {
            throw Error(LP_ERR_INVALID, std::string(streamName(s)) +
                                            " is not an image stream; use Device::start()");
        }
        enabled_[static_cast<size_t>(s)] = true;
    }
    void disableStream(Stream s) { enabled_[static_cast<size_t>(s)] = false; }
    void enableAllImageStreams() {
        for(Stream s : {Stream::IrLeft, Stream::IrRight, Stream::Depth, Stream::Color}) {
            enableStream(s);
        }
    }
    bool isEnabled(Stream s) const { return enabled_[static_cast<size_t>(s)]; }

    /* FrameSets kept for waitForFrames(); the oldest is dropped when full. */
    void setQueueSize(size_t n) { queueSize_ = n ? n : 1; }
    size_t queueSize() const { return queueSize_; }

private:
    bool   enabled_[LP_STREAM_COUNT] = {};
    size_t queueSize_ = 4;
};

/*
 * Groups the enabled image streams into FrameSets.
 *
 * IR left, IR right and depth come from one stereo capture and carry the
 * same timestamp; a set holds one frame of each with that timestamp. Depth
 * arrives a few hundred ms after its IR pair, so sets with depth enabled are
 * that much behind. Color has its own capture times; the color frame nearest
 * the stereo timestamp is attached. With only color enabled, every color
 * frame is a set.
 */
class Pipeline {
public:
    typedef std::function<void(const FrameSet &)> FrameSetCallback;

    explicit Pipeline(Device &dev) : dev_(dev) {}
    ~Pipeline() {
        try {
            stop();
        } catch(...) {
        }
    }

    Pipeline(const Pipeline &) = delete;
    Pipeline &operator=(const Pipeline &) = delete;

    /* Deliver sets through waitForFrames(). */
    void start(const Config &config) { start(config, FrameSetCallback()); }

    /* Deliver sets to callback instead, on a receive thread. */
    void start(const Config &config, FrameSetCallback callback) {
        stop();
        config_   = config;
        callback_ = std::move(callback);
        lastSet_  = 0;
        bool any  = false;
        try {
            for(size_t i = 0; i < LP_STREAM_COUNT; i++) {
                const Stream s = static_cast<Stream>(i);
                if(!config_.isEnabled(s)) {
                    continue;
                }
                any = true;
                dev_.start(s, [this](const Frame &f) { onFrame(f); });
                started_.push_back(s);
            }
        } catch(...) {
            stop();
            throw;
        }
        if(!any) {
            throw Error(LP_ERR_INVALID, "no stream enabled in the Config");
        }
    }

    void stop() {
        for(Stream s : started_) {
            dev_.stop(s);
        }
        started_.clear();
        std::lock_guard<std::mutex> lock(mutex_);
        for(auto &h : history_) {
            h.clear();
        }
        queue_.clear();
    }

    /* Next FrameSet, or an empty one after timeoutMs. */
    FrameSet waitForFrames(int timeoutMs = 1000) {
        std::unique_lock<std::mutex> lock(mutex_);
        if(!ready_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                            [this] { return !queue_.empty(); })) {
            return FrameSet();
        }
        FrameSet fs = queue_.front();
        queue_.pop_front();
        return fs;
    }

private:
    static constexpr size_t   kHistory = 32;       /* ~0.6 s of IR at 50 fps */
    static constexpr uint64_t kSameCaptureNs = 1000000;  /* 1 ms */

    static uint64_t distance(uint64_t a, uint64_t b) { return a > b ? a - b : b - a; }

    void onFrame(const Frame &f) {
        FrameSet out;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto &h = history_[static_cast<size_t>(f.stream())];
            h.push_back(f);
            if(h.size() > kHistory) {
                h.pop_front();
            }
            if(!assemble(&out)) {
                return;
            }
            if(!callback_) {
                queue_.push_back(out);
                if(queue_.size() > config_.queueSize()) {
                    queue_.pop_front();
                }
            }
        }
        if(callback_) {
            callback_(out);
        } else {
            ready_.notify_one();
        }
    }

    /* Frame of stream s captured at t, or nullptr. */
    const Frame *findSameCapture(Stream s, uint64_t t) const {
        for(const auto &f : history_[static_cast<size_t>(s)]) {
            if(distance(f.timestamp(), t) <= kSameCaptureNs) {
                return &f;
            }
        }
        return nullptr;
    }

    bool assemble(FrameSet *out) {
        std::vector<Stream> stereo;
        for(Stream s : {Stream::Depth, Stream::IrLeft, Stream::IrRight}) {
            if(config_.isEnabled(s)) {
                stereo.push_back(s);
            }
        }
        const bool color = config_.isEnabled(Stream::Color);
        auto      &colorHistory = history_[static_cast<size_t>(Stream::Color)];

        uint64_t t = 0;
        if(stereo.empty()) {
            if(colorHistory.empty() || colorHistory.back().timestamp() <= lastSet_) {
                return false;
            }
            t = colorHistory.back().timestamp();
        } else {
            /* Newest capture every stereo stream has; the first listed is the
             * slowest to arrive, so search from it. */
            const auto &lead = history_[static_cast<size_t>(stereo[0])];
            bool found = false;
            for(auto it = lead.rbegin(); it != lead.rend() && !found; ++it) {
                if(it->timestamp() <= lastSet_) {
                    break;
                }
                bool all = true;
                for(size_t i = 1; i < stereo.size() && all; i++) {
                    all = findSameCapture(stereo[i], it->timestamp()) != nullptr;
                }
                if(all) {
                    t     = it->timestamp();
                    found = true;
                }
            }
            if(!found) {
                return false;
            }
            /* Wait until color has caught up past t, so the nearest frame is
             * not one that is still on its way. */
            if(color && (colorHistory.empty() || colorHistory.back().timestamp() < t)) {
                return false;
            }
        }

        *out            = FrameSet();
        out->timestamp_ = t;
        for(Stream s : stereo) {
            out->frames_[static_cast<size_t>(s)] = *findSameCapture(s, t);
        }
        if(color) {
            const Frame *best = nullptr;
            for(const auto &f : colorHistory) {
                if(!best || distance(f.timestamp(), t) < distance(best->timestamp(), t)) {
                    best = &f;
                }
            }
            out->frames_[static_cast<size_t>(Stream::Color)] = *best;
        }
        lastSet_ = t;
        return true;
    }

    Device                 &dev_;
    Config                  config_;
    FrameSetCallback        callback_;
    std::vector<Stream>     started_;
    std::mutex              mutex_;
    std::condition_variable ready_;
    std::deque<Frame>       history_[LP_STREAM_COUNT];
    std::deque<FrameSet>    queue_;
    uint64_t                lastSet_ = 0;
};

}  // namespace lp

#endif /* LOOPER_STREAM_HPP */
