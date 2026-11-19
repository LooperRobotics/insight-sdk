#include "Insight9SDK.h"
#include "ExtensionUnitControl.hpp"
#include "UvcDevice.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <vector>

// ==================== Logging Macros ====================
// Always print errors and warnings
#define SDK_LOG_ERROR(fmt, ...)   fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__)
#define SDK_LOG_WARN(fmt, ...)    fprintf(stderr, "[WARN] " fmt "\n", ##__VA_ARGS__)

// Info and debug are controlled by DEBUG macro
#ifdef DEBUG
    #define SDK_LOG_INFO(fmt, ...)  printf("[INFO] " fmt "\n", ##__VA_ARGS__)
    #define SDK_LOG_DEBUG(fmt, ...) printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)
#else
    #define SDK_LOG_INFO(fmt, ...)  ((void)0)
    #define SDK_LOG_DEBUG(fmt, ...) ((void)0)
#endif

#define RECONNECT_BACKOFF_BASE_MS 1000
#define RECONNECT_BACKOFF_MAX_MS  30000
#define RECONNECT_MAX_ATTEMPTS    30

static constexpr uint32_t RS_META_CAPTURE_TIMING_ID = 0x80000001u;

static constexpr double ACCEL_SCALE_FACTOR   = -0.366459184;
static constexpr double GYRO_SCALE_FACTOR    = -610.464183381;
static constexpr double ACCEL_SCALE_LOOPERHUB = 0.0035913;
static constexpr double GYRO_SCALE_LOOPERHUB  = 0.00106526;

// ==================== HID Report Structures ====================
// Raw HID reports, same layout the Linux SDK reads from hidraw.
#pragma pack(push, 1)
struct imu_accel_report_t {
    uint8_t  report_id;
    uint8_t  sensor_state;
    uint64_t timestamp;
    int32_t  accel_x, accel_y, accel_z;
    uint32_t timestamp_l, timestamp_h;
    int16_t  custom_data3, custom_data4, custom_data5;
    int8_t   custom_data6, custom_data7;
};
struct imu_gyro_report_t {
    uint8_t  report_id;
    uint8_t  sensor_state;
    uint64_t timestamp;
    int32_t  gyro_x, gyro_y, gyro_z;
    uint32_t timestamp_l, timestamp_h;
    int16_t  custom_data3, custom_data4, custom_data5;
    int8_t   custom_data6, custom_data7;
};
struct vio_hid_payload {
    uint64_t timestamp;
    float px, py, pz;
    float qx, qy, qz, qw;
    uint8_t seq;
    uint8_t reserved[3];
};
#pragma pack(pop)

// ==================== Target Device VID/PID ====================
#define VENDOR_ID  0x3652
#define PRODUCT_ID 0x0b5c

// ==================== Camera Configuration ====================
#define UVC_NUM 2
#define LOGICAL_CAM_NUM 3
#define HID_NUM 2

#define RGB_CAM_ID    0
#define GRAY_CAM_ID   1
#define DEPTH_CAM_ID  2

#define RGB_UVC_ID       0
#define COMPOSITE_UVC_ID 1

#define DEPTH_STREAM_ID  0
#define GRAY_STREAM_ID   1

#define HID_IMU_ID 0
#define HID_VIO_ID 1

#define MAIN_WIDTH   1088
#define MAIN_HEIGHT  1920
#define MAIN_FORMAT  PixelFormat::YUYV
#define SUB_WIDTH    544
#define SUB_HEIGHT   640
#define SUB_FORMAT   PixelFormat::Y8I
#define DEPTH_WIDTH  544
#define DEPTH_HEIGHT 640
#define DEPTH_FORMAT PixelFormat::Z16

enum class FrameType {
    Unknown,
    Gray,
    Depth,
    Color
};

struct FrameInfo {
    FrameType type;
    uint64_t timestamp;
    int width;
    int height;
    PixelFormat format;
    uint32_t streamIndex;
};

static inline uint32_t rdLe32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline uint64_t rdLe64(const uint8_t* p) {
    return  (uint64_t)p[0]        | ((uint64_t)p[1] << 8)  |
            ((uint64_t)p[2] << 16) | ((uint64_t)p[3] << 24) |
            ((uint64_t)p[4] << 32) | ((uint64_t)p[5] << 40) |
            ((uint64_t)p[6] << 48) | ((uint64_t)p[7] << 56);
}

static uint64_t steadyNowUs() {
    return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

static void sortCapabilities(std::vector<DeviceCapability>& caps) {
    std::sort(caps.begin(), caps.end(), [](const DeviceCapability& a, const DeviceCapability& b) {
        if (a.format != b.format) {
            return static_cast<int>(a.format) < static_cast<int>(b.format);
        }
        const long long areaA = (long long)a.width * a.height;
        const long long areaB = (long long)b.width * b.height;
        if (areaA != areaB) return areaA < areaB;
        if (a.width != b.width) return a.width < b.width;
        return a.fps < b.fps;
    });
}

// Vendor metadata carried in the UVC payload header after PTS/SCR: a list of
// {id, size, ...} items. The capture-timing item holds the frame counter at
// +16 and a 64-bit microsecond timestamp at +20 (same layout as on Windows,
// where the driver wraps it in a 40-byte KS header first).
static bool parseRsVendorMetadata(const uint8_t* buf, size_t len,
                                  uint64_t* time_us_64, uint32_t* frame_counter)
{
    size_t off = 0;
    while (buf && off + 8 <= len) {
        uint32_t id   = rdLe32(buf + off);
        uint32_t size = rdLe32(buf + off + 4);
        if (size < 8 || off + size > len)
            break;
        if (id == RS_META_CAPTURE_TIMING_ID && size >= 28) {
            if (frame_counter) *frame_counter = rdLe32(buf + off + 16);
            if (time_us_64)    *time_us_64    = rdLe64(buf + off + 20);
            return true;
        }
        off += size;
    }
    return false;
}

// ==================== Extension Unit Helpers ====================
static int is_camera_params_valid(const camera_params *params) {
    // Validate resolution index. RGB uses 0-3 and grayscale uses 0-1; the SDK uses the more permissive 0-3 range.
    if (params->resolution > 3) {
        fprintf(stderr, "[XU][ERR] invalid resolution, expected 0-3, got %d\n", params->resolution);
        return 0;
    }
    // Validate frame-rate index.
    if (params->frame_rate > 5) {
        fprintf(stderr, "[XU][ERR] invalid frame rate, expected 0-5, got %d\n", params->frame_rate);
        return 0;
    }

    // exposure_time: 0.0 ~ 0.03
    if (params->exposure_time < 0.0f || params->exposure_time > 0.03f) {
        fprintf(stderr, "[XU][ERR] invalid exposure time, expected 0.0-0.03, got %f\n", params->exposure_time);
        return 0;
    }
    // exposure_gain: 1.0 ~ 16.0
    if (params->exposure_gain < 1.0f || params->exposure_gain > 16.0f) {
        fprintf(stderr, "[XU][ERR] invalid exposure gain, expected 1.0-16.0, got %f\n", params->exposure_gain);
        return 0;
    }
    // auto_exposure: 0 or 1
    if (params->auto_exposure > 1) {
        fprintf(stderr, "[XU][ERR] invalid auto exposure, expected 0 or 1, got %d\n", params->auto_exposure);
        return 0;
    }
    // brightness: 0.0 ~ 127.0
    if (params->brightness < 0.0f || params->brightness > 127.0f) {
        fprintf(stderr, "[XU][ERR] invalid brightness, expected 0.0-127.0, got %f\n", params->brightness);
        return 0;
    }
    // contrast: 0.0 ~ 1.9
    if (params->contrast < 0.0f || params->contrast > 1.9f) {
        fprintf(stderr, "[XU][ERR] invalid contrast, expected 0.0-1.9, got %f\n", params->contrast);
        return 0;
    }
    // gamma_dark: 1.0 ~ 4.0
    if (params->gamma_dark < 1.0f || params->gamma_dark > 4.0f) {
        fprintf(stderr, "[XU][ERR] invalid gamma dark, expected 1.0-4.0, got %f\n", params->gamma_dark);
        return 0;
    }
    // hue: 0.0 ~ 87.0
    if (params->hue < 0.0f || params->hue > 87.0f) {
        fprintf(stderr, "[XU][ERR] invalid hue, expected 0.0-87.0, got %f\n", params->hue);
        return 0;
    }
    // saturation: 0.0 ~ 1.999
    if (params->saturation < 0.0f || params->saturation > 1.999f) {
        fprintf(stderr, "[XU][ERR] invalid saturation, expected 0.0-1.999, got %f\n", params->saturation);
        return 0;
    }
    // sharpness: 1 ~ 255
    if (params->sharpness < 1) {
        fprintf(stderr, "[XU][ERR] invalid sharpness, expected 1-255, got %d\n", params->sharpness);
        return 0;
    }
    // auto_white_balance: 0 or 1
    if (params->auto_white_balance > 1) {
        fprintf(stderr, "[XU][ERR] invalid auto white balance, expected 0 or 1, got %d\n", params->auto_white_balance);
        return 0;
    }
    // white_balance: 1.0 ~ 3.0
    if (params->white_balance < 1.0f || params->white_balance > 3.0f) {
        fprintf(stderr, "[XU][ERR] invalid white balance, expected 1.0-3.0, got %f\n", params->white_balance);
        return 0;
    }
    // decimation: 1 ~ 255
    if (params->decimation < 1) {
        fprintf(stderr, "[XU][ERR] invalid decimation, expected 1-255, got %d\n", params->decimation);
        return 0;
    }

    return 1;
}

// ==================== UvcSource ====================
// One physical UVC function (the macOS counterpart of the Windows
// MFVideoSource): the RGB function with one stream, or the composite function
// with depth on stream 0 and gray on stream 1. Frames from all of its streams
// are funnelled into one queue that the slot's video worker drains.
class UvcSource {
public:
    ~UvcSource() { close(); }

    bool open(const std::string& devicePath, int deviceIndex, int width, int height, PixelFormat fmt, int fps)
    {
        close();
        deviceIndex_ = deviceIndex;
        if (!attach(devicePath)) return false;

        if (!addStream(0, fmt, width, height, fps, -1)) {
            SDK_LOG_ERROR("[UvcSource] Stream configure failed: %dx%d@%d fmt=%d", width, height, fps, (int)fmt);
            close();
            return false;
        }
        SDK_LOG_INFO("[UvcSource] Open success: %dx%d@%d, format=%d", width, height, fps, (int)fmt);
        return true;
    }

    bool openWithTwoStreams(const std::string& devicePath, int deviceIndex,
                            int width0, int height0, int width1, int height1,
                            PixelFormat fmt0, int fps0,
                            PixelFormat fmt1, int fps1)
    {
        close();
        deviceIndex_ = deviceIndex;
        if (!attach(devicePath)) return false;

        if (!addStream(0, fmt0, width0, height0, fps0, -1)) {
            SDK_LOG_ERROR("[UvcSource] Stream 0 configure failed: %dx%d@%d fmt=%d", width0, height0, fps0, (int)fmt0);
            close();
            return false;
        }
        if (!addStream(1, fmt1, width1, height1, fps1, streams_[0]->ifnum())) {
            SDK_LOG_ERROR("[UvcSource] Stream 1 configure failed: %dx%d@%d fmt=%d", width1, height1, fps1, (int)fmt1);
            close();
            return false;
        }
        SDK_LOG_INFO("[UvcSource] Open success: two streams (%dx%d@%d, %dx%d@%d)",
               width0, height0, fps0, width1, height1, fps1);
        return true;
    }

    void close() {
        stop();
        for (auto& s : streams_) s.reset();
        fn_ = nullptr;
        dev_.reset();
        std::lock_guard<std::mutex> lock(queueMutex_);
        queue_.clear();
        resetTimingState();
    }

    bool start() {
        if (!dev_ || running_) return false;
        resetTimingState();
        lastFrameMs_.store((int64_t)(steadyNowUs() / 1000));
        for (uint32_t i = 0; i < 2; ++i) {
            if (!streams_[i]) continue;
            if (!streams_[i]->start([this, i](insight9::UvcRawFrame& raw) { onFrame(i, raw); })) {
                SDK_LOG_ERROR("[UvcSource] Failed to start stream %u on UVC %d", i, deviceIndex_);
                for (auto& s : streams_) {
                    if (s) s->stop();
                }
                return false;
            }
        }
        running_ = true;
        return true;
    }

    void stop() {
        running_ = false;
        // Wake a worker blocked in readFrame(). The owner must join that worker
        // before deleting or replacing this source.
        queueCv_.notify_all();
        for (auto& s : streams_) {
            if (s) s->stop();
        }
        queueCv_.notify_all();
    }

    bool isRunning() const { return running_; }

    bool readFrame(FrameInfo& info, std::vector<uint8_t>& data, int timeoutMs) {
        std::unique_lock<std::mutex> lock(queueMutex_);
        queueCv_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                          [this] { return !queue_.empty() || !running_; });

        for (const auto& s : streams_) {
            if (s && s->failed() && running_) {
                SDK_LOG_ERROR("[UvcSource] UVC %d stream on if%u failed (device disconnected?)",
                              deviceIndex_, (unsigned)s->ifnum());
                running_ = false;
            }
        }
        // On macOS a bulk transfer that is in flight during an unplug does not
        // always complete with NO_DEVICE, so the streams alone may never notice.
        // Also trust the device flag (set by any thread that saw the unplug),
        // and treat a stream that has gone silent as broken.
        if (running_ && dev_ && !dev_->alive()) {
            SDK_LOG_ERROR("[UvcSource] UVC %d device disconnected", deviceIndex_);
            running_ = false;
        }
        const int64_t silentMs = (int64_t)(steadyNowUs() / 1000) - lastFrameMs_.load();
        if (running_ && silentMs > kStallTimeoutMs) {
            SDK_LOG_ERROR("[UvcSource] UVC %d: no frames for %lld ms, reopening",
                          deviceIndex_, (long long)silentMs);
            running_ = false;
        }
        if (queue_.empty()) return false;

        QueuedFrame f = std::move(queue_.front());
        queue_.pop_front();
        lock.unlock();

        const insight9::UvcStream& s = *streams_[f.streamIndex];
        info.streamIndex = f.streamIndex;
        info.width = s.width();
        info.height = s.height();
        info.format = s.format();
        info.timestamp = f.timestamp;
        switch (info.format) {
            case PixelFormat::Z16:   info.type = FrameType::Depth; break;
            case PixelFormat::Y8I:
            case PixelFormat::GREY:  info.type = FrameType::Gray; break;
            case PixelFormat::MJPEG:
            case PixelFormat::YUYV:
            case PixelFormat::YVYU:
            case PixelFormat::NV12:
            case PixelFormat::RGB8:  info.type = FrameType::Color; break;
            default:                 info.type = FrameType::Unknown; break;
        }
        data.swap(f.data);
        return true;
    }

    uint64_t getLastTimestamp(uint32_t streamIndex) const {
        if (streamIndex >= 2) return 0;
        return lastTimestamp_[streamIndex].load(std::memory_order_relaxed);
    }

private:
    struct QueuedFrame {
        uint32_t streamIndex = 0;
        uint64_t timestamp = 0;
        std::vector<uint8_t> data;
    };

    static constexpr size_t kMaxQueuedFrames = 6;
    // A started source that delivers nothing for this long is reopened.
    static constexpr int64_t kStallTimeoutMs = 5000;

    bool attach(const std::string& devicePath) {
        dev_ = insight9::UsbDevice::acquire(VENDOR_ID, PRODUCT_ID);
        if (!dev_) {
            SDK_LOG_ERROR("[UvcSource] Device %04x:%04x not available", VENDOR_ID, PRODUCT_ID);
            return false;
        }
        fn_ = dev_->findFunction(devicePath);
        if (!fn_) {
            SDK_LOG_ERROR("[UvcSource] UVC function not found: %s", devicePath.c_str());
            dev_.reset();
            return false;
        }
        devicePath_ = devicePath;
        return true;
    }

    // Bind stream slot `index` to the first VideoStreaming interface of this
    // function that offers the requested mode, skipping `excludeIfnum`.
    bool addStream(uint32_t index, PixelFormat fmt, int width, int height, int fps, int excludeIfnum) {
        for (const auto& vs : fn_->streams) {
            if ((int)vs.ifnum == excludeIfnum) continue;
            const insight9::UvcFormatDesc* f = nullptr;
            const insight9::UvcFrameDesc* fr = nullptr;
            uint32_t iv = 0;
            if (!vs.find(fmt, width, height, fps, &f, &fr, &iv)) continue;
            auto stream = std::make_unique<insight9::UvcStream>(dev_, fn_->bcdUVC, vs);
            if (!stream->configure(fmt, width, height, fps)) return false;
            streams_[index] = std::move(stream);
            return true;
        }

        std::string modes;
        for (const auto& vs : fn_->streams) {
            if ((int)vs.ifnum == excludeIfnum) continue;
            for (const auto& cap : vs.capabilities()) {
                if (cap.format != fmt) continue;
                modes += " " + std::to_string(cap.width) + "x" + std::to_string(cap.height) +
                         "@" + std::to_string(cap.fps);
            }
        }
        SDK_LOG_ERROR("[UvcSource] No stream offers %dx%d@%d fmt=%d; available:%s",
                      width, height, fps, (int)fmt, modes.empty() ? " (none)" : modes.c_str());
        return false;
    }

    // Runs on the libusb event thread.
    void onFrame(uint32_t index, insight9::UvcRawFrame& raw) {
        uint64_t ts = 0;
        uint64_t mdTimeUs64 = 0;
        uint32_t mdFrameCounter = 0;
        if (!raw.vendorMeta.empty() &&
            parseRsVendorMetadata(raw.vendorMeta.data(), raw.vendorMeta.size(), &mdTimeUs64, &mdFrameCounter)) {
            ts = mdTimeUs64;
        } else if (raw.hasPts) {
            // Firmware writes the low 32 bits of its microsecond clock into PTS.
            ts = unwrapPts(index, raw.pts);
        } else {
            ts = steadyNowUs();
        }
        lastTimestamp_[index].store(ts, std::memory_order_relaxed);
        lastFrameMs_.store((int64_t)(steadyNowUs() / 1000));

        {
            std::lock_guard<std::mutex> lock(queueMutex_);
            if (!running_) return;
            if (queue_.size() >= kMaxQueuedFrames) queue_.pop_front();   // consumer is behind
            QueuedFrame f;
            f.streamIndex = index;
            f.timestamp = ts;
            f.data = std::move(raw.data);
            queue_.push_back(std::move(f));
        }
        queueCv_.notify_one();
    }

    uint64_t unwrapPts(uint32_t index, uint32_t ts32) {
        if (ptsInit_[index] && ts32 < lastPts_[index] && (lastPts_[index] - ts32) > 0x80000000u) {
            ptsWrapHigh_[index] += 0x100000000ull;
        }
        lastPts_[index] = ts32;
        ptsInit_[index] = true;
        return ptsWrapHigh_[index] + (uint64_t)ts32;
    }

    void resetTimingState() {
        for (int i = 0; i < 2; ++i) {
            lastTimestamp_[i].store(0, std::memory_order_relaxed);
            lastPts_[i] = 0;
            ptsWrapHigh_[i] = 0;
            ptsInit_[i] = false;
        }
    }

    std::shared_ptr<insight9::UsbDevice> dev_;
    const insight9::UvcFunction* fn_ = nullptr;   // owned by dev_
    std::unique_ptr<insight9::UvcStream> streams_[2];
    std::string devicePath_;
    int deviceIndex_ = -1;

    std::atomic<bool> running_{false};
    std::mutex queueMutex_;
    std::condition_variable queueCv_;
    std::deque<QueuedFrame> queue_;

    std::atomic<uint64_t> lastTimestamp_[2] = {{0}, {0}};
    std::atomic<int64_t> lastFrameMs_{0};               // host time of the last frame
    uint32_t lastPts_[2] = {0, 0};
    uint64_t ptsWrapHigh_[2] = {0, 0};
    bool ptsInit_[2] = {false, false};
};

struct sdk_ctx_t {
    insight9_config_t config;
    // =====================================================
    // Physical UVC functions
    // videos[0] = RGB function
    // videos[1] = Composite function
    //               stream 0 = Depth
    //               stream 1 = Gray
    // =====================================================
    UvcSource* videos[UVC_NUM];
    std::string videoPaths[UVC_NUM];
    // =====================================================
    // Logical camera state
    // cam 0 = RGB
    // cam 1 = Gray
    // cam 2 = Depth
    // =====================================================
    std::atomic<bool> cam_running[LOGICAL_CAM_NUM];
    std::thread videoThreads[UVC_NUM];
    camera_params cachedInitialParams[LOGICAL_CAM_NUM];
    bool hasCachedInitialParams[LOGICAL_CAM_NUM] = {false, false, false};
    // =====================================================
    // HID interfaces: [0] = IMU, [1] = VIO
    std::string hidPaths[HID_NUM];
    std::mutex hidPathMutex;
    std::thread hidThreads[HID_NUM];
    viewer::ExtensionUnitControl* xu;
    // Serialize all access/recreation of the Extension Unit object.
    std::recursive_mutex xuMutex;
    // Serialize ownership changes of UvcSource. Never hold this while
    // waiting for a video worker to join, otherwise the worker could deadlock
    // while retiring a failed source.
    std::mutex videoLifecycleMutex;
    // Serialize user-triggered start/stop/reconfigure operations so two UI/control
    // calls can never rebuild the same physical UVC concurrently.
    std::recursive_mutex cameraControlMutex;
    // =====================================================
    std::atomic<bool> running;
    image_callback imgCb;
    void* imgUser;
    imu_callback imuCb;
    void* imuUser;
    vio_callback vioCb;
    void* vioUser;
    std::mutex imgMutex;
    std::mutex imuMutex;
    std::mutex vioMutex;
    bool initialized;
    DeviceCapabilities_t device_caps;
    bool singleNodeMode = false;
};

static sdk_ctx_t g_ctx;

// Returns a vector of exactly UVC_NUM entries indexed by slot:
//   [RGB_UVC_ID]       the standalone RGB function
//   [COMPOSITE_UVC_ID] the depth+gray composite (the function offering Z16)
// An empty string means that slot is not present on this unit (an Insight 3u
// publishes only the composite function).
static std::vector<std::string> findUvcDevices(uint16_t vid, uint16_t pid) {
    std::vector<std::string> paths(UVC_NUM);
    auto dev = insight9::UsbDevice::acquire(vid, pid);
    if (!dev) return paths;

    for (const auto& fn : dev->functions()) {
        if (fn.streams.empty()) continue;
        if (fn.offers(PixelFormat::Z16)) {
            if (paths[COMPOSITE_UVC_ID].empty()) paths[COMPOSITE_UVC_ID] = dev->functionPath(fn);
        } else if (paths[RGB_UVC_ID].empty()) {
            paths[RGB_UVC_ID] = dev->functionPath(fn);
        }
    }

    // Called on every reconnect attempt; only report changes.
    static std::mutex logMutex;
    static std::string lastLogged;
    const std::string summary = "RGB=" + (paths[RGB_UVC_ID].empty() ? std::string("(absent)") : paths[RGB_UVC_ID]) +
                                " Composite=" + (paths[COMPOSITE_UVC_ID].empty() ? std::string("(absent)") : paths[COMPOSITE_UVC_ID]);
    std::lock_guard<std::mutex> lock(logMutex);
    if (summary != lastLogged) {
        SDK_LOG_INFO("[SDK] UVC slots: %s", summary.c_str());
        lastLogged = summary;
    }
    return paths;
}

// True when this unit has no standalone RGB function - the composite function
// is the whole device.
static bool uvcSlotsAreSingleNode(const std::vector<std::string>& paths) {
    return paths[RGB_UVC_ID].empty() && !paths[COMPOSITE_UVC_ID].empty();
}

static bool reopenXUControlLocked(const char* reason) {
    // The XU sits on the VideoControl interface, so either function exposes
    // it. Prefer RGB, but fall back to the composite. Always re-enumerate: the
    // paths change when the device is re-plugged into another port.
    auto paths = findUvcDevices(VENDOR_ID, PRODUCT_ID);
    std::string path = !paths[RGB_UVC_ID].empty() ? paths[RGB_UVC_ID] : paths[COMPOSITE_UVC_ID];

    if (path.empty()) {
        SDK_LOG_ERROR("[XU][ERR] Cannot reopen XU: no UVC function available (%s)",
                      reason ? reason : "unknown");
        return false;
    }

    if (g_ctx.xu) {
        delete g_ctx.xu;
        g_ctx.xu = nullptr;
    }

    auto* xu = new (std::nothrow) viewer::ExtensionUnitControl();
    if (!xu) {
        SDK_LOG_ERROR("[XU][ERR] Allocation failed while reopening XU (%s)",
                      reason ? reason : "unknown");
        return false;
    }

    SDK_LOG_INFO("[XU] Reopening XU on UVC: %s (%s)", path.c_str(), reason ? reason : "unknown");

    if (!xu->open(path)) {
        SDK_LOG_ERROR("[XU][WARN] Reopen failed on path: %s", path.c_str());
        delete xu;
        return false;
    }

    g_ctx.xu = xu;
    SDK_LOG_INFO("[XU] Reopen success");
    return true;
}

static bool callXUWithRetry(const char* op,
                            const std::function<bool(viewer::ExtensionUnitControl&)>& fn) {
    std::lock_guard<std::recursive_mutex> lock(g_ctx.xuMutex);

    if ((!g_ctx.xu || !g_ctx.xu->isOpen()) && !reopenXUControlLocked(op)) {
        return false;
    }

    if (fn(*g_ctx.xu)) {
        return true;
    }

    SDK_LOG_ERROR("[XU][WARN] Operation '%s' failed; reopening XU and retrying once", op ? op : "unknown");

    if (!reopenXUControlLocked(op)) {
        return false;
    }

    return fn(*g_ctx.xu);
}

static int reconnect_backoff_ms(int attempt) {
    if (attempt < 1) attempt = 1;
    long delay = RECONNECT_BACKOFF_BASE_MS;
    for (int i = 1; i < attempt && delay < RECONNECT_BACKOFF_MAX_MS; ++i) delay <<= 1;
    if (delay > RECONNECT_BACKOFF_MAX_MS) delay = RECONNECT_BACKOFF_MAX_MS;
    return (int)delay;
}

static void interruptible_sleep_ms(int total_ms, const std::function<bool()>& keepWaiting = nullptr) {
    int slept = 0;
    while (slept < total_ms && g_ctx.running) {
        if (keepWaiting && !keepWaiting()) break;
        int chunk = (total_ms - slept) > 200 ? 200 : (total_ms - slept);
        std::this_thread::sleep_for(std::chrono::milliseconds(chunk));
        slept += chunk;
    }
}

static void reconnect_backoff_apply(const char* tag, int id, int* fails, const char* reason,
                                    const std::function<bool()>& keepWaiting = nullptr) {
    (*fails)++;
    int delay = reconnect_backoff_ms(*fails);
    if (RECONNECT_MAX_ATTEMPTS <= 0 || *fails <= RECONNECT_MAX_ATTEMPTS) {
        SDK_LOG_ERROR("[%s%d][ERR] %s, retry in %dms (attempt %d)", tag, id, reason, delay, *fails);
    } else if (*fails == RECONNECT_MAX_ATTEMPTS + 1) {
        SDK_LOG_ERROR("[%s%d][ERR] %s failed %d times, slowing to %ds retry",
                      tag, id, reason, RECONNECT_MAX_ATTEMPTS, RECONNECT_BACKOFF_MAX_MS / 1000);
    }
    interruptible_sleep_ms(delay, keepWaiting);
}

static void cacheInitialCameraParams(int cam_id) {
    camera_params params;
    if (insight9_receive_get_camera_params_for(cam_id, &params) == 0) {
        g_ctx.cachedInitialParams[cam_id] = params;
        g_ctx.hasCachedInitialParams[cam_id] = true;
        SDK_LOG_INFO("[SDK] Cached initial params for camera %d", cam_id);
    } else {
        SDK_LOG_WARN("[SDK] Failed to cache initial params for camera %d (device may not be ready yet)", cam_id);
    }
}

// Create and open (but do not start) the source for one UVC slot from the
// stored configuration.
// Does any VideoStreaming interface of this UVC function offer the mode?
static bool functionOffers(const std::string& path, const video_config_t& c) {
    auto dev = insight9::UsbDevice::acquire(VENDOR_ID, PRODUCT_ID);
    const insight9::UvcFunction* fn = dev ? dev->findFunction(path) : nullptr;
    if (!fn) return false;
    for (const auto& vs : fn->streams) {
        const insight9::UvcFormatDesc* f = nullptr;
        const insight9::UvcFrameDesc* fr = nullptr;
        uint32_t iv = 0;
        if (vs.find(c.pixel_format, c.width, c.height, c.fps, &f, &fr, &iv)) return true;
    }
    return false;
}

// Depth frame height that goes with a stereo frame of this height (same rule
// as the Linux SDK): YUYV/YVYU stack the two eyes, GREY stacks them plus one
// timestamp row, Y8I interleaves them so the height is unchanged.
static int depthHeightForGray(const video_config_t& gray) {
    switch (gray.pixel_format) {
        case PixelFormat::YUYV:
        case PixelFormat::YVYU: return gray.height / 2;
        case PixelFormat::GREY: return (gray.height - 1) / 2;
        default:                return gray.height;
    }
}

// Last depth mode the composite function was opened with successfully.
static video_config_t g_lastGoodDepth = {};
static bool g_hasLastGoodDepth = false;

// Pick the depth mode for opening the composite function. Unlike Windows,
// macOS negotiates each VideoStreaming interface on its own, so depth does not
// have to share the gray stream's size:
//   1. depth_config as configured;
//   2. otherwise the size derived from the gray mode (depthHeightForGray);
//   3. otherwise the depth mode that last opened successfully, so switching
//      the gray stream never takes the whole composite function down.
static video_config_t resolveDepthConfig(const std::string& path) {
    const video_config_t requested = g_ctx.config.depth_config;
    if (functionOffers(path, requested)) return requested;

    video_config_t derived = requested;
    derived.width = g_ctx.config.gray_config.width;
    derived.height = depthHeightForGray(g_ctx.config.gray_config);
    if (functionOffers(path, derived)) {
        SDK_LOG_WARN("[SDK] Depth %dx%d@%d is not supported; using %dx%d derived from the gray mode",
                     requested.width, requested.height, requested.fps, derived.width, derived.height);
        return derived;
    }

    if (g_hasLastGoodDepth && functionOffers(path, g_lastGoodDepth)) {
        SDK_LOG_WARN("[SDK] Depth %dx%d@%d is not supported; keeping the previous depth mode %dx%d@%d",
                     requested.width, requested.height, requested.fps,
                     g_lastGoodDepth.width, g_lastGoodDepth.height, g_lastGoodDepth.fps);
        return g_lastGoodDepth;
    }

    // Nothing better; opening reports the modes the device does offer.
    return requested;
}

static UvcSource* createVideoSource(int uvcId, const std::string& path) {
    UvcSource* src = new (std::nothrow) UvcSource();
    if (!src) return nullptr;

    bool opened = false;
    if (uvcId == RGB_UVC_ID) {
        opened = src->open(path, uvcId,
                           g_ctx.config.rgb_config.width, g_ctx.config.rgb_config.height,
                           g_ctx.config.rgb_config.pixel_format, g_ctx.config.rgb_config.fps);
    } else {
        const video_config_t gray = g_ctx.config.gray_config;
        const video_config_t depth = resolveDepthConfig(path);
        SDK_LOG_INFO("[SDK] Opening Composite UVC %d: Depth %dx%d@%d fmt=%d, Gray %dx%d@%d fmt=%d",
                     uvcId, depth.width, depth.height, depth.fps, (int)depth.pixel_format,
                     gray.width, gray.height, gray.fps, (int)gray.pixel_format);
        opened = src->openWithTwoStreams(path, uvcId,
                                         depth.width, depth.height, gray.width, gray.height,
                                         depth.pixel_format, depth.fps,
                                         gray.pixel_format, gray.fps);
        if (opened) {
            // Report the mode actually in use through get_camera_config().
            g_ctx.config.depth_config = depth;
            g_lastGoodDepth = depth;
            g_hasLastGoodDepth = true;
        }
    }

    if (!opened) {
        delete src;
        return nullptr;
    }
    return src;
}

static bool openVideoSourceForSlot(int uvcId) {
    if (uvcId < 0 || uvcId >= UVC_NUM) return false;
    auto uvcPaths = findUvcDevices(VENDOR_ID, PRODUCT_ID);
    g_ctx.singleNodeMode = uvcSlotsAreSingleNode(uvcPaths);

    // This is the path videoThreadFunc retries on, which is what lets a device
    // that was absent at init attach later.
    const std::string path = uvcPaths[uvcId];
    if (path.empty()) return false;

    cacheInitialCameraParams(uvcId == RGB_UVC_ID ? RGB_CAM_ID : GRAY_CAM_ID);

    UvcSource* src = createVideoSource(uvcId, path);
    if (!src) return false;

    if (!src->start()) {
        delete src;
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(g_ctx.videoLifecycleMutex);
        g_ctx.videoPaths[uvcId] = path;
        g_ctx.videos[uvcId] = src;
    }
    return true;
}

// Capabilities come straight from the UVC descriptors; nothing has to stream.
static bool probeDeviceCapabilities(const std::string& devicePath, int deviceIndex,
                                    DeviceCapabilities_t& caps) {
    auto dev = insight9::UsbDevice::acquire(VENDOR_ID, PRODUCT_ID);
    const insight9::UvcFunction* fn = dev ? dev->findFunction(devicePath) : nullptr;
    if (!fn || fn->streams.empty()) {
        SDK_LOG_ERROR("[Probe] UVC function not available: %s", devicePath.c_str());
        return false;
    }

    if (deviceIndex == RGB_UVC_ID) {
        const insight9::UvcStreamingInterface& vs = fn->streams.front();
        std::vector<DeviceCapability> allCaps = vs.capabilities();
        caps.rgb_default = vs.defaultCapability();
        sortCapabilities(allCaps);
        caps.rgb_capabilities = allCaps;
    } else {
        const insight9::UvcStreamingInterface* depthVs = nullptr;
        const insight9::UvcStreamingInterface* grayVs = nullptr;
        for (const auto& vs : fn->streams) {
            if (!depthVs && vs.offers(PixelFormat::Z16)) depthVs = &vs;
        }
        for (const auto& vs : fn->streams) {
            if (&vs != depthVs) { grayVs = &vs; break; }
        }

        if (depthVs) {
            std::vector<DeviceCapability> depthCaps = depthVs->capabilities();
            caps.depth_default = depthVs->defaultCapability();
            sortCapabilities(depthCaps);
            caps.depth_capabilities = depthCaps;
        }
        if (grayVs) {
            std::vector<DeviceCapability> grayCaps = grayVs->capabilities();
            caps.gray_default = grayVs->defaultCapability();
            sortCapabilities(grayCaps);
            caps.gray_capabilities = grayCaps;
        }
    }

    caps.initialized = true;
    return true;
}

// ==================== HID (IMU / VIO) ====================
// The IMU is the interface whose report descriptor uses the HID Sensor usage
// page (it is what Windows binds through its Sensor API); the VIO pose stream
// is the other one. If no descriptor says "sensor", fall back to the Linux
// convention: lowest interface number = IMU.
static bool pickHidInterface(const insight9::UsbDevice& dev, int role, insight9::HidInterface& out) {
    const auto& hids = dev.hidInterfaces();
    if (hids.empty()) return false;

    int imu = -1;
    for (size_t i = 0; i < hids.size(); ++i) {
        if (hids[i].isSensor) { imu = (int)i; break; }
    }
    if (imu < 0) imu = 0;

    if (role == HID_IMU_ID) {
        out = hids[imu];
        return true;
    }
    for (size_t i = 0; i < hids.size(); ++i) {
        if ((int)i != imu) {
            out = hids[i];
            return true;
        }
    }
    return false;
}

struct ImuState {
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;
    uint64_t lastAccelTs = 0;
    uint64_t lastGyroTs = 0;
    bool accelDirty = false;
    bool gyroDirty = false;
};

static void handleImuReport(const uint8_t* buf, int n, ImuState& s) {
    if (n <= 0) return;
    const uint8_t report_id = buf[0];

    if (report_id == 0x01 && n >= (int)sizeof(imu_accel_report_t)) {
        imu_accel_report_t accel;
        memcpy(&accel, buf, sizeof(accel));
        const uint64_t ts = ((uint64_t)accel.timestamp_h << 32) | (uint64_t)accel.timestamp_l;
        if (ts == s.lastAccelTs) return;
        s.lastAccelTs = ts;
        s.ax = (float)(accel.accel_x / ACCEL_SCALE_FACTOR * ACCEL_SCALE_LOOPERHUB);
        s.ay = (float)(accel.accel_y / ACCEL_SCALE_FACTOR * ACCEL_SCALE_LOOPERHUB);
        s.az = (float)(accel.accel_z / ACCEL_SCALE_FACTOR * ACCEL_SCALE_LOOPERHUB);
        s.accelDirty = true;
    } else if (report_id == 0x02 && n >= (int)sizeof(imu_gyro_report_t)) {
        imu_gyro_report_t gyro;
        memcpy(&gyro, buf, sizeof(gyro));
        const uint64_t ts = ((uint64_t)gyro.timestamp_h << 32) | (uint64_t)gyro.timestamp_l;
        if (ts == s.lastGyroTs) return;
        s.lastGyroTs = ts;
        s.gx = (float)(gyro.gyro_x / GYRO_SCALE_FACTOR * GYRO_SCALE_LOOPERHUB);
        s.gy = (float)(gyro.gyro_y / GYRO_SCALE_FACTOR * GYRO_SCALE_LOOPERHUB);
        s.gz = (float)(gyro.gyro_z / GYRO_SCALE_FACTOR * GYRO_SCALE_LOOPERHUB);
        s.gyroDirty = true;
    }

    if (s.accelDirty && s.gyroDirty) {
        imu_callback cb;
        void* user;
        {
            std::lock_guard<std::mutex> lock(g_ctx.imuMutex);
            cb = g_ctx.imuCb;
            user = g_ctx.imuUser;
        }
        if (cb) {
            cb(s.ax, s.ay, s.az, s.gx, s.gy, s.gz,
               (s.lastAccelTs > s.lastGyroTs) ? s.lastAccelTs : s.lastGyroTs, user);
        }
        s.accelDirty = false;
        s.gyroDirty = false;
    }
}

static void handleVioReport(const uint8_t* buf, int n, uint64_t& lastVioTs) {
    if (n < (int)sizeof(vio_hid_payload)) return;
    vio_hid_payload p;
    memcpy(&p, buf, sizeof(p));
    if (p.timestamp != 0 && p.timestamp == lastVioTs) return;
    lastVioTs = p.timestamp;

    vio_callback cb;
    void* user;
    {
        std::lock_guard<std::mutex> lock(g_ctx.vioMutex);
        cb = g_ctx.vioCb;
        user = g_ctx.vioUser;
    }
    if (cb) {
        cb(p.px, p.py, p.pz, p.qx, p.qy, p.qz, p.qw, p.timestamp, user);
    }
}

static void setHidPath(int role, const std::string& path) {
    std::lock_guard<std::mutex> lock(g_ctx.hidPathMutex);
    g_ctx.hidPaths[role] = path;
}

static void hidThreadFunc(int role) {
    const char* tag = (role == HID_IMU_ID) ? "IMU" : "VIO";
    int reconnectFails = 0;
    std::shared_ptr<insight9::UsbDevice> dev;
    std::unique_ptr<insight9::HidInterruptReader> reader;
    std::vector<uint8_t> report;
    ImuState imu;
    uint64_t lastVioTs = 0;

    while (g_ctx.running) {
        if (!reader) {
            auto candidate = insight9::UsbDevice::acquire(VENDOR_ID, PRODUCT_ID);
            insight9::HidInterface hif;
            if (!candidate || !pickHidInterface(*candidate, role, hif)) {
                reconnect_backoff_apply(tag, role, &reconnectFails,
                                        candidate ? "HID interface not available" : "device not found");
                continue;
            }
            auto r = std::make_unique<insight9::HidInterruptReader>(candidate, hif);
            if (!r->start()) {
                reconnect_backoff_apply(tag, role, &reconnectFails, "cannot start HID reads");
                continue;
            }
            dev = candidate;
            reader = std::move(r);
            reconnectFails = 0;
            imu = ImuState();
            lastVioTs = 0;

            setHidPath(role, dev->interfacePath(hif.ifnum));
            SDK_LOG_INFO("[HID] %s attached on %s (ep=0x%02x)", tag,
                         dev->interfacePath(hif.ifnum).c_str(), (unsigned)hif.inEndpoint);
        }

        if (!reader->read(report, 100)) {
            if (reader->failed() || !dev->alive()) {
                SDK_LOG_WARN("[HID%d][WARN] %s device lost, reconnecting...", role, tag);
                reader.reset();     // cancels and frees its transfers
                dev.reset();
                setHidPath(role, std::string());
            }
            continue;
        }

        if (role == HID_IMU_ID) {
            handleImuReport(report.data(), (int)report.size(), imu);
        } else {
            handleVioReport(report.data(), (int)report.size(), lastVioTs);
        }
    }
    reader.reset();
    SDK_LOG_INFO("[HID] %s thread exited", tag);
}

// ==================== Video workers ====================
static int streamToLogicalCamId(int uvcId, uint32_t streamIndex) {
    if (uvcId == RGB_UVC_ID && streamIndex == 0) {
        return RGB_CAM_ID;  // 0
    } else if (uvcId == COMPOSITE_UVC_ID) {
        if (streamIndex == DEPTH_STREAM_ID) {
            return DEPTH_CAM_ID;  // 2
        } else if (streamIndex == GRAY_STREAM_ID) {
            return GRAY_CAM_ID;   // 1
        }
    }
    return -1;
}

static void deliverFrame(int uvcId, std::vector<uint8_t>& data, const FrameInfo& info) {
    // Snapshot cb/user together so a concurrent register cannot pair them
    // wrongly or null the pointer between the check and the call.
    image_callback cb;
    void* user;
    {
        std::lock_guard<std::mutex> lock(g_ctx.imgMutex);
        cb = g_ctx.imgCb;
        user = g_ctx.imgUser;
    }
    if (!cb) return;

    int callbackCamId = streamToLogicalCamId(uvcId, info.streamIndex);
    if (callbackCamId < 0 || callbackCamId >= LOGICAL_CAM_NUM) {
        SDK_LOG_ERROR("[SDK] Invalid frame mapping: uvc=%d stream=%u", uvcId, info.streamIndex);
        return;
    }

    // Same codes as the Windows SDK so callback code is portable.
    unsigned int v4l2Fmt = 0;
    if (info.format == PixelFormat::MJPEG) v4l2Fmt = 0x47504A4D;        // 'MJPG'
    else if (info.format == PixelFormat::GREY) v4l2Fmt = 0x59455247;    // 'GREY'
    else if (info.format == PixelFormat::Z16) v4l2Fmt = 0x36315A;       // 'Z16 '
    else if (info.format == PixelFormat::Y8I) v4l2Fmt = 0x49385956;     // 'Y8I '
    else if (info.format == PixelFormat::YUYV) v4l2Fmt = 0x32595559;    // 'YUYV'
    else if (info.format == PixelFormat::YVYU) v4l2Fmt = 0x55595659;    // 'YVYU'
    else if (info.format == PixelFormat::RGB8) v4l2Fmt = 0x42475200;    // 'RGB8'
    else if (info.format == PixelFormat::NV12) v4l2Fmt = 0x3231564E;    // 'NV12'

    cb(callbackCamId, data.data(), data.size(), info.width, info.height,
       v4l2Fmt, info.timestamp, user);
}

static void videoThreadFunc(int uvcId) {
    int reconnect_fails = 0;

    auto stillWanted = [uvcId]() -> bool {
        if (!g_ctx.running) return false;
        if (uvcId == RGB_UVC_ID) return g_ctx.cam_running[RGB_CAM_ID];
        return g_ctx.cam_running[GRAY_CAM_ID] || g_ctx.cam_running[DEPTH_CAM_ID];
    };

    std::vector<uint8_t> data;
    while (stillWanted()) {
        UvcSource* src = nullptr;
        {
            std::lock_guard<std::mutex> lock(g_ctx.videoLifecycleMutex);
            src = g_ctx.videos[uvcId];
        }

        if (!src) {
            if (!openVideoSourceForSlot(uvcId)) {
                reconnect_backoff_apply("VIDEO", uvcId, &reconnect_fails, "open failed", stillWanted);
                continue;
            }
            reconnect_fails = 0;
            SDK_LOG_INFO("[SDK] Video device %d (re)connected", uvcId);

            {
                std::lock_guard<std::mutex> lock(g_ctx.videoLifecycleMutex);
                src = g_ctx.videos[uvcId];
            }

            // The XU is bound to the old USB device instance; rebind it on the
            // slot that carries it (RGB, or the composite on composite-only units).
            if (uvcId == RGB_UVC_ID || g_ctx.singleNodeMode) {
                std::lock_guard<std::recursive_mutex> lock(g_ctx.xuMutex);
                if (!reopenXUControlLocked("video reconnect")) {
                    SDK_LOG_INFO("[XU][WARN] Video recovered but XU is not available yet");
                }
            }
        }

        if (!src) {
            reconnect_backoff_apply("VIDEO", uvcId, &reconnect_fails, "source pointer unavailable", stillWanted);
            continue;
        }

        while (stillWanted() && src->isRunning()) {
            FrameInfo info{};
            if (!src->readFrame(info, data, 100)) continue;

            if (uvcId == RGB_UVC_ID) {
                deliverFrame(uvcId, data, info);
            } else if ((info.streamIndex == DEPTH_STREAM_ID && g_ctx.cam_running[DEPTH_CAM_ID]) ||
                       (info.streamIndex == GRAY_STREAM_ID && g_ctx.cam_running[GRAY_CAM_ID])) {
                deliverFrame(uvcId, data, info);
            }
        }

        // The worker owns the source only when it stopped unexpectedly while the
        // logical camera is still wanted. During a normal stop/reconfigure, the
        // controller clears cam_running[] first, joins this worker, and then
        // destroys g_ctx.videos[uvcId].
        if (stillWanted()) {
            std::lock_guard<std::mutex> lock(g_ctx.videoLifecycleMutex);
            if (g_ctx.videos[uvcId] == src) {
                SDK_LOG_WARN("[VIDEO%d][WARN] Stream stopped unexpectedly; retiring source and reconnecting", uvcId);
                g_ctx.videos[uvcId] = nullptr;
                delete src;
                src = nullptr;
            }
        }
    }

    SDK_LOG_INFO("[SDK] Video thread exited: UVC=%d", uvcId);
}

// ==================== SDK API ====================
static void resetContext() {
    // sdk_ctx_t contains std::thread/std::mutex/std::string; never memset it.
    g_ctx.config = {};
    g_ctx.running = false;
    g_ctx.initialized = false;
    g_ctx.singleNodeMode = false;
    g_ctx.xu = nullptr;
    g_ctx.device_caps = {};
    g_lastGoodDepth = {};
    g_hasLastGoodDepth = false;
    for (int i = 0; i < UVC_NUM; ++i) {
        g_ctx.videos[i] = nullptr;
        g_ctx.videoPaths[i].clear();
    }
    for (int i = 0; i < LOGICAL_CAM_NUM; ++i) {
        g_ctx.cam_running[i] = false;
        g_ctx.hasCachedInitialParams[i] = false;
    }
    for (int i = 0; i < HID_NUM; ++i) {
        setHidPath(i, std::string());
    }
    {
        std::lock_guard<std::mutex> lock(g_ctx.imgMutex);
        g_ctx.imgCb = nullptr;
        g_ctx.imgUser = nullptr;
    }
    {
        std::lock_guard<std::mutex> lock(g_ctx.imuMutex);
        g_ctx.imuCb = nullptr;
        g_ctx.imuUser = nullptr;
    }
    {
        std::lock_guard<std::mutex> lock(g_ctx.vioMutex);
        g_ctx.vioCb = nullptr;
        g_ctx.vioUser = nullptr;
    }
}

// Open the sources for every present slot and bind the XU. Missing slots are
// not fatal: videoThreadFunc's reconnect loop opens them if and when they appear.
static void openInitialDevices(const std::vector<std::string>& uvcPaths) {
    for (int i = 0; i < UVC_NUM; ++i) {
        g_ctx.videoPaths[i] = uvcPaths[i];
        if (uvcPaths[i].empty()) {
            g_ctx.videos[i] = nullptr;
            SDK_LOG_WARN("[SDK] UVC slot %d not present; will attach on reconnect", i);
            continue;
        }
        g_ctx.videos[i] = createVideoSource(i, uvcPaths[i]);
        if (!g_ctx.videos[i]) {
            SDK_LOG_WARN("[SDK] Failed to open physical UVC %d: %s; will retry", i, uvcPaths[i].c_str());
        }
    }

    std::lock_guard<std::recursive_mutex> lock(g_ctx.xuMutex);
    if (!reopenXUControlLocked("initialization")) {
        SDK_LOG_WARN("[SDK][WARN] XU control unavailable during initialization; video can still run");
    }
    if (!g_ctx.singleNodeMode) cacheInitialCameraParams(RGB_CAM_ID);
    cacheInitialCameraParams(GRAY_CAM_ID);
}

int insight9_receive_init(const insight9_config_t* config) {
    if (g_ctx.initialized) {
        SDK_LOG_ERROR("[SDK] Already initialized");
        return -1;
    }

    if (!config) {
        SDK_LOG_ERROR("[SDK] Config is NULL, using default");
        return insight9_receive_init_default();
    }

    if (config->rgb_config.width <= 0 || config->rgb_config.height <= 0 ||
        config->gray_config.width <= 0 || config->gray_config.height <= 0 ||
        config->depth_config.width <= 0 || config->depth_config.height <= 0) {
        SDK_LOG_ERROR("[SDK] Invalid resolution in config");
        return -1;
    }

    resetContext();
    g_ctx.config = *config;

    if (!insight9::usbRuntimeStart()) return -1;

    auto uvcPaths = findUvcDevices(VENDOR_ID, PRODUCT_ID);
    g_ctx.singleNodeMode = uvcSlotsAreSingleNode(uvcPaths);
    if (g_ctx.singleNodeMode) {
        SDK_LOG_INFO("[SDK] Composite-only device: no standalone RGB function");
    }

    openInitialDevices(uvcPaths);

    g_ctx.initialized = true;
    SDK_LOG_INFO("[SDK] Initialized successfully with config:");
    SDK_LOG_INFO("  RGB: %dx%d@%d, format=%d",
           config->rgb_config.width, config->rgb_config.height,
           config->rgb_config.fps, (int)config->rgb_config.pixel_format);
    SDK_LOG_INFO("  Gray: %dx%d@%d, format=%d",
           config->gray_config.width, config->gray_config.height,
           config->gray_config.fps, (int)config->gray_config.pixel_format);
    SDK_LOG_INFO("  Depth: %dx%d@%d, format=%d",
           config->depth_config.width, config->depth_config.height,
           config->depth_config.fps, (int)config->depth_config.pixel_format);

    return 0;
}

int insight9_receive_init_default() {
    if (g_ctx.initialized) {
        SDK_LOG_ERROR("[SDK] Already initialized");
        return -1;
    }

    resetContext();

    if (!insight9::usbRuntimeStart()) return -1;

    auto uvcPaths = findUvcDevices(VENDOR_ID, PRODUCT_ID);
    g_ctx.singleNodeMode = uvcSlotsAreSingleNode(uvcPaths);
    if (g_ctx.singleNodeMode) {
        SDK_LOG_INFO("[SDK] Composite-only device: no standalone RGB function");
    }

    SDK_LOG_INFO("[SDK] Probing device capabilities...");

    if (!uvcPaths[RGB_UVC_ID].empty()) {
        probeDeviceCapabilities(uvcPaths[RGB_UVC_ID], RGB_UVC_ID, g_ctx.device_caps);
    }
    if (!uvcPaths[COMPOSITE_UVC_ID].empty()) {
        probeDeviceCapabilities(uvcPaths[COMPOSITE_UVC_ID], COMPOSITE_UVC_ID, g_ctx.device_caps);
    }

    if (g_ctx.device_caps.rgb_default.valid) {
        g_ctx.config.rgb_config.width = g_ctx.device_caps.rgb_default.width;
        g_ctx.config.rgb_config.height = g_ctx.device_caps.rgb_default.height;
        g_ctx.config.rgb_config.fps = g_ctx.device_caps.rgb_default.fps;
        g_ctx.config.rgb_config.pixel_format = g_ctx.device_caps.rgb_default.format;
        SDK_LOG_INFO("[SDK] Using probed RGB config: %dx%d@%d",
               g_ctx.config.rgb_config.width,
               g_ctx.config.rgb_config.height,
               g_ctx.config.rgb_config.fps);
    } else {
        g_ctx.config.rgb_config.width = MAIN_WIDTH;
        g_ctx.config.rgb_config.height = MAIN_HEIGHT;
        g_ctx.config.rgb_config.fps = 30;
        g_ctx.config.rgb_config.pixel_format = MAIN_FORMAT;
        SDK_LOG_INFO("[SDK] Using fallback RGB config: %dx%d@%d",
               g_ctx.config.rgb_config.width,
               g_ctx.config.rgb_config.height,
               g_ctx.config.rgb_config.fps);
    }

    if (g_ctx.device_caps.gray_default.valid) {
        g_ctx.config.gray_config.width = g_ctx.device_caps.gray_default.width;
        g_ctx.config.gray_config.height = g_ctx.device_caps.gray_default.height;
        g_ctx.config.gray_config.fps = g_ctx.device_caps.gray_default.fps;
        g_ctx.config.gray_config.pixel_format = g_ctx.device_caps.gray_default.format;
        SDK_LOG_INFO("[SDK] Using probed Gray config: %dx%d@%d",
               g_ctx.config.gray_config.width,
               g_ctx.config.gray_config.height,
               g_ctx.config.gray_config.fps);
    } else {
        g_ctx.config.gray_config.width = SUB_WIDTH;
        g_ctx.config.gray_config.height = SUB_HEIGHT;
        g_ctx.config.gray_config.fps = 30;
        g_ctx.config.gray_config.pixel_format = SUB_FORMAT;
        SDK_LOG_INFO("[SDK] Using fallback Gray config: %dx%d@%d",
               g_ctx.config.gray_config.width,
               g_ctx.config.gray_config.height,
               g_ctx.config.gray_config.fps);
    }

    if (g_ctx.device_caps.depth_default.valid) {
        g_ctx.config.depth_config.width = g_ctx.device_caps.depth_default.width;
        g_ctx.config.depth_config.height = g_ctx.device_caps.depth_default.height;
        g_ctx.config.depth_config.fps = g_ctx.device_caps.depth_default.fps;
        g_ctx.config.depth_config.pixel_format = g_ctx.device_caps.depth_default.format;
        SDK_LOG_INFO("[SDK] Using probed Depth config: %dx%d@%d",
            g_ctx.config.depth_config.width,
            g_ctx.config.depth_config.height,
            g_ctx.config.depth_config.fps);
    } else if (g_ctx.device_caps.gray_default.valid) {
        g_ctx.config.depth_config.width = g_ctx.device_caps.gray_default.width;
        g_ctx.config.depth_config.height = g_ctx.device_caps.gray_default.height;
        g_ctx.config.depth_config.fps = g_ctx.device_caps.gray_default.fps;
        g_ctx.config.depth_config.pixel_format = PixelFormat::Z16;
        SDK_LOG_INFO("[SDK] Using Gray config for Depth fallback: %dx%d@%d",
            g_ctx.config.depth_config.width,
            g_ctx.config.depth_config.height,
            g_ctx.config.depth_config.fps);
    } else {
        g_ctx.config.depth_config.width = DEPTH_WIDTH;
        g_ctx.config.depth_config.height = DEPTH_HEIGHT;
        g_ctx.config.depth_config.fps = 30;
        g_ctx.config.depth_config.pixel_format = DEPTH_FORMAT;
        SDK_LOG_INFO("[SDK] Using hardcoded Depth fallback: %dx%d@30", DEPTH_WIDTH, DEPTH_HEIGHT);
    }

    // Depth and gray come from the same composite function; run them at the
    // same rate when the gray stream supports the depth rate.
    if (g_ctx.config.gray_config.fps != g_ctx.config.depth_config.fps) {
        for (const auto& cap : g_ctx.device_caps.gray_capabilities) {
            if (cap.valid && cap.format == g_ctx.config.gray_config.pixel_format &&
                cap.width == g_ctx.config.gray_config.width &&
                cap.height == g_ctx.config.gray_config.height &&
                cap.fps == g_ctx.config.depth_config.fps) {
                SDK_LOG_INFO("[SDK] Aligning Gray FPS %d -> %d to match Depth",
                       g_ctx.config.gray_config.fps, g_ctx.config.depth_config.fps);
                g_ctx.config.gray_config.fps = g_ctx.config.depth_config.fps;
                break;
            }
        }
    }

    openInitialDevices(uvcPaths);

    g_ctx.initialized = true;
    return 0;
}

int insight9_receive_start() {
    if (!g_ctx.initialized || g_ctx.running) return -1;
    g_ctx.running = true;
    g_ctx.cam_running[RGB_CAM_ID] = true;
    g_ctx.cam_running[GRAY_CAM_ID] = true;
    g_ctx.cam_running[DEPTH_CAM_ID] = true;

    for (int uvcId = 0; uvcId < UVC_NUM; ++uvcId) {
        // Start the worker for every slot, including ones with no source yet:
        // videoThreadFunc() opens missing sources with backoff, which is what
        // lets the SDK come up with the device absent and attach it later.
        if (g_ctx.videos[uvcId] && !g_ctx.videos[uvcId]->start()) {
            SDK_LOG_WARN("[SDK] Failed to start UVC %d; retiring source and retrying", uvcId);
            std::lock_guard<std::mutex> lock(g_ctx.videoLifecycleMutex);
            delete g_ctx.videos[uvcId];
            g_ctx.videos[uvcId] = nullptr;
        }

        g_ctx.videoThreads[uvcId] = std::thread(videoThreadFunc, uvcId);
    }

    g_ctx.hidThreads[HID_IMU_ID] = std::thread(hidThreadFunc, HID_IMU_ID);
    g_ctx.hidThreads[HID_VIO_ID] = std::thread(hidThreadFunc, HID_VIO_ID);

    return 0;
}

static int mapCompositeCamId(int cam_id);

// The worker may retire (delete) g_ctx.videos[uvcId] at any time when the
// device drops out, so the pointer is only dereferenced under the lifecycle lock.
static void stopVideoSource(int uvcId) {
    std::lock_guard<std::mutex> lock(g_ctx.videoLifecycleMutex);
    if (g_ctx.videos[uvcId]) g_ctx.videos[uvcId]->stop();
}

// Stop and join the worker of one physical UVC, then destroy its source.
// cam_running[] for that slot must already be cleared.
static void retireVideoSlot(int uvcId) {
    stopVideoSource(uvcId);
    if (g_ctx.videoThreads[uvcId].joinable()) {
        g_ctx.videoThreads[uvcId].join();
    }

    std::lock_guard<std::mutex> lock(g_ctx.videoLifecycleMutex);
    if (g_ctx.videos[uvcId]) {
        delete g_ctx.videos[uvcId];
        g_ctx.videos[uvcId] = nullptr;
    }
}

// Open, start and hand a fresh source for this slot to a new worker.
static int launchVideoSlot(int uvcId) {
    auto uvcPaths = findUvcDevices(VENDOR_ID, PRODUCT_ID);
    g_ctx.singleNodeMode = uvcSlotsAreSingleNode(uvcPaths);
    // An empty entry means this unit does not publish that function.
    if (uvcPaths[uvcId].empty()) return -1;
    g_ctx.videoPaths[uvcId] = uvcPaths[uvcId];

    UvcSource* src = createVideoSource(uvcId, uvcPaths[uvcId]);
    if (!src) return -1;
    if (!src->start()) {
        delete src;
        return -1;
    }

    {
        std::lock_guard<std::mutex> lock(g_ctx.videoLifecycleMutex);
        g_ctx.videos[uvcId] = src;
    }

    if (uvcId == COMPOSITE_UVC_ID) {
        g_ctx.cam_running[GRAY_CAM_ID] = true;
        g_ctx.cam_running[DEPTH_CAM_ID] = true;
    } else {
        g_ctx.cam_running[RGB_CAM_ID] = true;
    }

    g_ctx.videoThreads[uvcId] = std::thread(videoThreadFunc, uvcId);
    return 0;
}

int insight9_receive_start_camera(int cam_id) {
    // Before insight9_receive_start() the worker would exit at once and leave a
    // joinable std::thread that start() then overwrites (std::terminate).
    if (!g_ctx.initialized || !g_ctx.running || cam_id < 0 || cam_id >= LOGICAL_CAM_NUM) return -1;

    std::lock_guard<std::recursive_mutex> controlLock(g_ctx.cameraControlMutex);

    if (g_ctx.cam_running[cam_id]) return 0;

    const int uvcId = mapCompositeCamId(cam_id);
    if (uvcId < 0 || uvcId >= UVC_NUM) return -1;

    // Gray and Depth share one Composite worker. While the other logical stream
    // is on, that worker is alive (streaming, or reconnecting with backoff), so
    // just re-enable this stream on it. Joining it here would never return.
    if (uvcId == COMPOSITE_UVC_ID &&
        (g_ctx.cam_running[GRAY_CAM_ID] || g_ctx.cam_running[DEPTH_CAM_ID]) &&
        g_ctx.videoThreads[uvcId].joinable()) {
        g_ctx.cam_running[cam_id] = true;
        SDK_LOG_INFO("[SDK] Logical camera %d enabled on existing Composite worker", cam_id);
        return 0;
    }

    // Nothing wants this slot any more, so its worker has exited or is exiting.
    retireVideoSlot(uvcId);
    return launchVideoSlot(uvcId);
}

void insight9_receive_stop_camera(int cam_id) {
    std::lock_guard<std::recursive_mutex> controlLock(g_ctx.cameraControlMutex);
    if (cam_id < 0 || cam_id >= LOGICAL_CAM_NUM) return;

    SDK_LOG_INFO("[SDK] Stop logical camera %d", cam_id);

    if (cam_id == RGB_CAM_ID) {
        g_ctx.cam_running[RGB_CAM_ID] = false;
        retireVideoSlot(RGB_UVC_ID);
        return;
    }

    // Logical Gray and Depth share ONE physical Composite UVC function. Stopping
    // one logical stream keeps the other alive; the physical streams stop when
    // both are off.
    g_ctx.cam_running[cam_id] = false;

    if (!g_ctx.cam_running[GRAY_CAM_ID] && !g_ctx.cam_running[DEPTH_CAM_ID]) {
        retireVideoSlot(COMPOSITE_UVC_ID);
    }
}

static void stopCompositeForReconfigure() {
    g_ctx.cam_running[GRAY_CAM_ID] = false;
    g_ctx.cam_running[DEPTH_CAM_ID] = false;
    retireVideoSlot(COMPOSITE_UVC_ID);
}

int insight9_receive_get_camera_config(int cam_id, int* width, int* height, PixelFormat* format, int* fps) {
    if (!g_ctx.initialized || cam_id < 0 || cam_id >= LOGICAL_CAM_NUM) return -1;
    if (!width || !height || !format || !fps) return -1;

    const video_config_t* cur = (cam_id == RGB_CAM_ID) ? &g_ctx.config.rgb_config
                               : (cam_id == GRAY_CAM_ID) ? &g_ctx.config.gray_config
                                                          : &g_ctx.config.depth_config;
    *width  = cur->width;
    *height = cur->height;
    *format = cur->pixel_format;
    *fps    = cur->fps;
    return 0;
}

int insight9_receive_restart_camera(int cam_id) {
    std::lock_guard<std::recursive_mutex> controlLock(g_ctx.cameraControlMutex);
    // Same reason as insight9_receive_start_camera(): no workers before start().
    if (!g_ctx.initialized || !g_ctx.running || cam_id < 0 || cam_id >= LOGICAL_CAM_NUM) return -1;

    const int uvcId = mapCompositeCamId(cam_id);
    if (uvcId < 0 || uvcId >= UVC_NUM) return -1;

    SDK_LOG_INFO("[SDK] Restart logical camera %d (physical UVC %d)", cam_id, uvcId);

    if (uvcId == RGB_UVC_ID) {
        insight9_receive_stop_camera(RGB_CAM_ID);
    } else {
        // Gray and Depth share the composite function: always rebuild both.
        stopCompositeForReconfigure();
    }
    // Let the device settle after the streaming interface went back to idle.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    return launchVideoSlot(uvcId);
}

int insight9_receive_set_camera_fps(int cam_id, int fps) {
    if (!g_ctx.initialized || cam_id < 0 || cam_id >= LOGICAL_CAM_NUM || fps <= 0) return -1;
    if (cam_id == RGB_CAM_ID) {
        g_ctx.config.rgb_config.fps = fps;
        SDK_LOG_INFO("[SDK] Set RGB FPS to %d", fps);
    } else if (cam_id == GRAY_CAM_ID) {
        g_ctx.config.gray_config.fps = fps;
        SDK_LOG_INFO("[SDK] Set Gray FPS to %d", fps);
    } else {
        g_ctx.config.depth_config.fps = fps;
        SDK_LOG_INFO("[SDK] Set Depth FPS to %d", fps);
    }
    return 0;
}

int insight9_receive_set_camera_format(int cam_id, int width, int height, PixelFormat format) {
    if (!g_ctx.initialized) return -1;
    if (width <= 0 || height <= 0) return -1;

    if (cam_id == RGB_CAM_ID) {
        g_ctx.config.rgb_config.width = width;
        g_ctx.config.rgb_config.height = height;
        g_ctx.config.rgb_config.pixel_format = format;
    } else if (cam_id == GRAY_CAM_ID) {
        g_ctx.config.gray_config.width = width;
        g_ctx.config.gray_config.height = height;
        g_ctx.config.gray_config.pixel_format = format;
    } else if (cam_id == DEPTH_CAM_ID) {
        g_ctx.config.depth_config.width = width;
        g_ctx.config.depth_config.height = height;
        g_ctx.config.depth_config.pixel_format = format;
    } else {
        return -1;
    }

    SDK_LOG_INFO("[SDK] Set camera %d format to %dx%d", cam_id, width, height);
    return 0;
}

int insight9_receive_switch_camera_config(int cam_id, int width, int height, PixelFormat format, int fps) {
    std::lock_guard<std::recursive_mutex> controlLock(g_ctx.cameraControlMutex);
    if (!g_ctx.initialized || cam_id < 0 || cam_id >= LOGICAL_CAM_NUM) return -1;
    if (width <= 0 || height <= 0 || fps <= 0) return -1;

    // Nothing is streaming yet: store the config. init() has already opened
    // the source with the old one, so rebuild it; start() then streams it.
    if (!g_ctx.running) {
        if (insight9_receive_set_camera_format(cam_id, width, height, format) != 0) return -1;
        if (insight9_receive_set_camera_fps(cam_id, fps) != 0) return -1;

        const int uvcId = mapCompositeCamId(cam_id);
        std::lock_guard<std::mutex> lock(g_ctx.videoLifecycleMutex);
        if (g_ctx.videos[uvcId] && !g_ctx.videoPaths[uvcId].empty()) {
            delete g_ctx.videos[uvcId];
            g_ctx.videos[uvcId] = createVideoSource(uvcId, g_ctx.videoPaths[uvcId]);
            if (!g_ctx.videos[uvcId]) {
                SDK_LOG_ERROR("[SDK] Camera %d does not support %dx%d@%d fmt=%d",
                              cam_id, width, height, fps, (int)format);
                return -1;
            }
        }
        return 0;
    }

    SDK_LOG_INFO("[SDK] Switching camera %d to %dx%d@%d...", cam_id, width, height, fps);

    if (insight9_receive_set_camera_format(cam_id, width, height, format) != 0) {
        SDK_LOG_ERROR("[SDK] Failed to set camera %d format", cam_id);
        return -1;
    }
    if (insight9_receive_set_camera_fps(cam_id, fps) != 0) {
        SDK_LOG_ERROR("[SDK] Failed to set camera %d fps", cam_id);
        return -1;
    }

    const int ret = insight9_receive_restart_camera(cam_id);
    if (ret == 0) {
        SDK_LOG_INFO("[SDK] Camera %d switched to %dx%d@%d successfully", cam_id, width, height, fps);
    } else {
        SDK_LOG_ERROR("[SDK] Failed to restart camera %d after config switch", cam_id);
    }
    return ret;
}

int insight9_receive_switch_camera_fps(int cam_id, int fps) {
    if (!g_ctx.initialized || cam_id < 0 || cam_id >= LOGICAL_CAM_NUM) return -1;
    video_config_t* cur = (cam_id == RGB_CAM_ID) ? &g_ctx.config.rgb_config
                        : (cam_id == GRAY_CAM_ID) ? &g_ctx.config.gray_config
                                                    : &g_ctx.config.depth_config;
    return insight9_receive_switch_camera_config(cam_id, cur->width, cur->height, cur->pixel_format, fps);
}

int insight9_receive_switch_camera_format(int cam_id, int width, int height, PixelFormat format) {
    if (!g_ctx.initialized || cam_id < 0 || cam_id >= LOGICAL_CAM_NUM) return -1;
    video_config_t* cur = (cam_id == RGB_CAM_ID) ? &g_ctx.config.rgb_config
                        : (cam_id == GRAY_CAM_ID) ? &g_ctx.config.gray_config
                                                    : &g_ctx.config.depth_config;
    return insight9_receive_switch_camera_config(cam_id, width, height, format, cur->fps);
}

int insight9_receive_is_camera_running(int cam_id) {
    if (cam_id < 0 || cam_id >= LOGICAL_CAM_NUM) return 0;
    return g_ctx.cam_running[cam_id] ? 1 : 0;
}

int insight9_receive_has_camera(int cam_id) {
    if (cam_id < 0 || cam_id >= LOGICAL_CAM_NUM) return 0;
    // Gray and depth are two streams of the same composite function, so both
    // depend on the composite slot; only RGB has a function of its own.
    const int uvcId = (cam_id == RGB_CAM_ID) ? RGB_UVC_ID : COMPOSITE_UVC_ID;
    return g_ctx.videoPaths[uvcId].empty() ? 0 : 1;
}

int insight9_receive_camera_has_color(int cam_id) {
    if (cam_id < 0 || cam_id >= LOGICAL_CAM_NUM) return 0;
    if (!insight9_receive_has_camera(cam_id)) return 0;

    const std::vector<DeviceCapability>* caps =
        (cam_id == RGB_CAM_ID)  ? &g_ctx.device_caps.rgb_capabilities :
        (cam_id == GRAY_CAM_ID) ? &g_ctx.device_caps.gray_capabilities :
                                  &g_ctx.device_caps.depth_capabilities;

    // insight9_receive_init(config) does not probe capabilities - only
    // init_default() does - so probe on demand.
    if (caps->empty()) {
        const int uvcId = (cam_id == RGB_CAM_ID) ? RGB_UVC_ID : COMPOSITE_UVC_ID;
        probeDeviceCapabilities(g_ctx.videoPaths[uvcId], uvcId, g_ctx.device_caps);
    }

    for (const auto& cap : *caps) {
        if (!cap.valid) continue;
        if (cap.format == PixelFormat::YUYV || cap.format == PixelFormat::YVYU ||
            cap.format == PixelFormat::MJPEG || cap.format == PixelFormat::NV12 ||
            cap.format == PixelFormat::RGB8) {
            return 1;
        }
    }
    return 0;
}

void insight9_receive_stop() {
    std::lock_guard<std::recursive_mutex> controlLock(g_ctx.cameraControlMutex);
    if (!g_ctx.running) return;
    g_ctx.running = false;

    for (int i = 0; i < LOGICAL_CAM_NUM; ++i) g_ctx.cam_running[i] = false;
    for (int i = 0; i < UVC_NUM; ++i) stopVideoSource(i);
    for (int i = 0; i < UVC_NUM; ++i) {
        if (g_ctx.videoThreads[i].joinable()) g_ctx.videoThreads[i].join();
    }
    for (int i = 0; i < HID_NUM; ++i) {
        if (g_ctx.hidThreads[i].joinable()) g_ctx.hidThreads[i].join();
    }
}

void insight9_receive_cleanup() {
    std::lock_guard<std::recursive_mutex> controlLock(g_ctx.cameraControlMutex);
    insight9_receive_stop();

    {
        std::lock_guard<std::mutex> lock(g_ctx.videoLifecycleMutex);
        for (int i = 0; i < UVC_NUM; ++i) {
            delete g_ctx.videos[i];
            g_ctx.videos[i] = nullptr;
        }
    }

    {
        std::lock_guard<std::recursive_mutex> lock(g_ctx.xuMutex);
        delete g_ctx.xu;
        g_ctx.xu = nullptr;
    }
    g_ctx.initialized = false;
    g_ctx.running = false;

    for (int i = 0; i < LOGICAL_CAM_NUM; ++i) {
        g_ctx.cam_running[i] = false;
    }

    // Releases the USB device (handing it back to macOS) and stops libusb.
    insight9::usbRuntimeStop();
}

// ==================== The remaining API implementations ====================
static int mapCompositeCamId(int cam_id) {
    if (cam_id == RGB_CAM_ID) {
        return RGB_UVC_ID;
    }
    if (cam_id == GRAY_CAM_ID || cam_id == DEPTH_CAM_ID) {
        return COMPOSITE_UVC_ID;
    }
    return -1;
}

static int mapCamIdToStreamIndex(int cam_id) {
    if (cam_id == RGB_CAM_ID)   return 0;
    if (cam_id == DEPTH_CAM_ID) return DEPTH_STREAM_ID;  // 0
    if (cam_id == GRAY_CAM_ID)  return GRAY_STREAM_ID;   // 1
    return -1;
}

const char* insight9_receive_get_video_dev(int cam_id) {
    int mapped = mapCompositeCamId(cam_id);
    if (mapped < 0 || mapped >= UVC_NUM || g_ctx.videoPaths[mapped].empty()) return nullptr;
    return g_ctx.videoPaths[mapped].c_str();
}

const char* insight9_receive_get_metadata_dev(int cam_id) {
    // Metadata arrives inline in the payload headers of the video stream.
    return insight9_receive_get_video_dev(cam_id);
}

void insight9_receive_register_image_callback(image_callback cb, void *user) {
    std::lock_guard<std::mutex> lock(g_ctx.imgMutex);
    g_ctx.imgCb = cb;
    g_ctx.imgUser = user;
}

void insight9_receive_register_imu_callback(imu_callback cb, void *user) {
    std::lock_guard<std::mutex> lock(g_ctx.imuMutex);
    g_ctx.imuCb = cb;
    g_ctx.imuUser = user;
}

void insight9_receive_register_vio_callback(vio_callback cb, void *user) {
    std::lock_guard<std::mutex> lock(g_ctx.vioMutex);
    g_ctx.vioCb = cb;
    g_ctx.vioUser = user;
}

const DeviceCapabilities_t* insight9_receive_get_device_capabilities_ptr(void) {
    if (!g_ctx.initialized) {
        SDK_LOG_ERROR("[SDK] Not initialized");
        return nullptr;
    }
    return &g_ctx.device_caps;
}

static const std::vector<DeviceCapability>* capabilityList(int cam_id) {
    switch (cam_id) {
        case RGB_CAM_ID:   return &g_ctx.device_caps.rgb_capabilities;
        case GRAY_CAM_ID:  return &g_ctx.device_caps.gray_capabilities;
        case DEPTH_CAM_ID: return &g_ctx.device_caps.depth_capabilities;
        default:
            SDK_LOG_ERROR("[SDK] Invalid cam_id: %d (must be 0, 1, or 2)", cam_id);
            return nullptr;
    }
}

int insight9_receive_get_device_default_capability(int cam_id, DeviceCapability* cap) {
    if (!g_ctx.initialized) {
        SDK_LOG_ERROR("[SDK] Not initialized");
        return -1;
    }

    if (!cap) {
        SDK_LOG_ERROR("[SDK] Invalid cap parameter");
        return -1;
    }

    const DeviceCapability* target = nullptr;
    switch (cam_id) {
        case RGB_CAM_ID:   target = &g_ctx.device_caps.rgb_default; break;
        case GRAY_CAM_ID:  target = &g_ctx.device_caps.gray_default; break;
        case DEPTH_CAM_ID: target = &g_ctx.device_caps.depth_default; break;
        default:
            SDK_LOG_ERROR("[SDK] Invalid cam_id: %d (must be 0, 1, or 2)", cam_id);
            return -1;
    }

    if (!target->valid) {
        SDK_LOG_ERROR("[SDK] No default capability found for cam_id %d", cam_id);
        return -1;
    }

    *cap = *target;
    return 0;
}

int insight9_receive_get_device_capability_count(int cam_id, int* count) {
    if (!g_ctx.initialized) {
        SDK_LOG_ERROR("[SDK] Not initialized");
        return -1;
    }

    if (!count) {
        SDK_LOG_ERROR("[SDK] Invalid count parameter");
        return -1;
    }

    const std::vector<DeviceCapability>* vec = capabilityList(cam_id);
    if (!vec) return -1;

    *count = (int)vec->size();
    return 0;
}

int insight9_receive_get_device_capability_by_index(int cam_id, int index, DeviceCapability* cap) {
    if (!g_ctx.initialized) {
        SDK_LOG_ERROR("[SDK] Not initialized");
        return -1;
    }

    if (!cap) {
        SDK_LOG_ERROR("[SDK] Invalid cap parameter");
        return -1;
    }

    if (index < 0) {
        SDK_LOG_ERROR("[SDK] Invalid index: %d (must be >= 0)", index);
        return -1;
    }

    const std::vector<DeviceCapability>* vec = capabilityList(cam_id);
    if (!vec) return -1;

    if (index >= (int)vec->size()) {
        SDK_LOG_ERROR("[SDK] Index %d out of range (size: %zu)", index, vec->size());
        return -1;
    }

    *cap = (*vec)[index];
    return 0;
}

int insight9_receive_read_metadata_timestamp(int cam_id, uint64_t* timestamp) {
    if (!timestamp) return -1;

    int uvcId = mapCompositeCamId(cam_id);
    int streamIdx = mapCamIdToStreamIndex(cam_id);
    if (uvcId < 0 || uvcId >= UVC_NUM || streamIdx < 0) {
        return -1;
    }

    uint64_t ts = 0;
    {
        std::lock_guard<std::mutex> lock(g_ctx.videoLifecycleMutex);
        if (!g_ctx.videos[uvcId]) return -1;
        ts = g_ctx.videos[uvcId]->getLastTimestamp((uint32_t)streamIdx);
    }
    if (ts == 0) {
        return -1;
    }

    *timestamp = ts;
    return 0;
}

int insight9_receive_set_active_camera(int cam_id) {
    return callXUWithRetry("setActiveCamera", [cam_id](viewer::ExtensionUnitControl& xu) {
        return xu.setActiveCamera((uint8_t)cam_id);
    }) ? 0 : -1;
}

int insight9_receive_get_active_camera(int *cam_id) {
    if (!cam_id) return -1;
    uint8_t val = 0;
    if (!callXUWithRetry("getActiveCamera", [&val](viewer::ExtensionUnitControl& xu) {
            return xu.getActiveCamera(val);
        })) return -1;
    *cam_id = val;
    return 0;
}

int insight9_check_camera_params(const camera_params *params) {
    if (!params) return -1;
    return is_camera_params_valid(params) ? 0 : -1;
}

int insight9_receive_set_camera_params(const camera_params *params) {
    if (!params || !is_camera_params_valid(params)) return -1;
    viewer::camera_params xuParams;
    memcpy(&xuParams, params, sizeof(viewer::camera_params));
    return callXUWithRetry("writeCurrentCameraParams", [xuParams](viewer::ExtensionUnitControl& xu) {
        return xu.writeCurrentCameraParams(xuParams);
    }) ? 0 : -1;
}

int insight9_receive_get_camera_params(camera_params *params) {
    if (!params) return -1;
    viewer::camera_params xuParams;
    if (!callXUWithRetry("readCurrentCameraParams", [&xuParams](viewer::ExtensionUnitControl& xu) {
            return xu.readCurrentCameraParams(xuParams);
        })) return -1;
    memcpy(params, &xuParams, sizeof(camera_params));
    return 0;
}

int insight9_receive_set_camera_params_for(int cam_id, const camera_params *params) {
    if (!params || !is_camera_params_valid(params)) return -1;
    int mapped = mapCompositeCamId(cam_id);
    if (mapped < 0) return -1;
    viewer::camera_params xuParams;
    memcpy(&xuParams, params, sizeof(viewer::camera_params));
    return callXUWithRetry("writeCameraParams", [mapped, xuParams](viewer::ExtensionUnitControl& xu) {
        return xu.writeCameraParams((uint8_t)mapped, xuParams);
    }) ? 0 : -1;
}

int insight9_receive_get_camera_params_for(int cam_id, camera_params *params) {
    if (!params) return -1;
    int mapped = mapCompositeCamId(cam_id);
    if (mapped < 0) return -1;
    viewer::camera_params xuParams;
    if (!callXUWithRetry("readCameraParams", [mapped, &xuParams](viewer::ExtensionUnitControl& xu) {
            return xu.readCameraParams((uint8_t)mapped, xuParams);
        })) return -1;
    memcpy(params, &xuParams, sizeof(camera_params));
    return 0;
}

int insight9_receive_get_cached_initial_params(int cam_id, camera_params* params) {
    if (!params || cam_id < 0 || cam_id >= LOGICAL_CAM_NUM) return -1;
    if (!g_ctx.hasCachedInitialParams[cam_id]) return -1;
    *params = g_ctx.cachedInitialParams[cam_id];
    return 0;
}

int insight9_receive_reset_camera_params(int) {
   // Reading factory defaults from the device requires additional implementation. A simple approach is to read
   // and save the current values during initialization, then write them back when a reset is needed.
   SDK_LOG_ERROR("[XU][ERR] reset_camera_params not implemented, use set_camera_params with saved defaults");
   return -1;
}

void insight9_receive_print_camera_params(const camera_params *params) {
    if (!params) return;
    viewer::camera_params xu_params;
    memcpy(&xu_params, params, sizeof(viewer::camera_params));
    viewer::printParams(xu_params);
}

int insight9_receive_get_camera_calib(int cam_idx, camera_calib *calib) {
    if (!calib) return -1;
    if (cam_idx < 0 || cam_idx >= viewer::kCalibCamCount) return -1;

    viewer::camera_calib xu_calib;
    if (!callXUWithRetry("readCameraCalib", [cam_idx, &xu_calib](viewer::ExtensionUnitControl& xu) {
            return xu.readCameraCalib(static_cast<uint8_t>(cam_idx), xu_calib);
        })) return -1;
    memcpy(calib, &xu_calib, sizeof(camera_calib));
    return 0;
}

void insight9_receive_print_camera_calib(const camera_calib *calib) {
    if (!calib) return;
    viewer::camera_calib xu_calib;
    memcpy(&xu_calib, calib, sizeof(viewer::camera_calib));
    viewer::printCalib(xu_calib);
}

// ==================== Depth Alignment Functions ====================

// Rasterize one triangle into the aligned-depth output, interpolating the RGB-
// frame depth across it with a nearest-surface z-buffer. Vertices are given as
// projected RGB pixel coords (x,y, float) plus their RGB-frame depth z (mm).
static inline void align_raster_triangle(uint16_t *out, int W, int H,
                                         float x0, float y0, float z0,
                                         float x1, float y1, float z1,
                                         float x2, float y2, float z2) {
    int minx = (int)floorf(fminf(x0, fminf(x1, x2)));
    int maxx = (int)ceilf(fmaxf(x0, fmaxf(x1, x2)));
    int miny = (int)floorf(fminf(y0, fminf(y1, y2)));
    int maxy = (int)ceilf(fmaxf(y0, fmaxf(y1, y2)));
    if (minx < 0) minx = 0;
    if (miny < 0) miny = 0;
    if (maxx > W - 1) maxx = W - 1;
    if (maxy > H - 1) maxy = H - 1;
    if (minx > maxx || miny > maxy) return;

    float area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
    if (fabsf(area) < 1e-6f) return;          // degenerate
    float inv_area = 1.0f / area;

    for (int py = miny; py <= maxy; py++) {
        for (int px = minx; px <= maxx; px++) {
            float fx = px + 0.5f, fy = py + 0.5f;
            // Barycentric weights via edge cross-products (same winding as area).
            float w0 = (x1 - fx) * (y2 - fy) - (x2 - fx) * (y1 - fy);
            float w1 = (x2 - fx) * (y0 - fy) - (x0 - fx) * (y2 - fy);
            float w2 = (x0 - fx) * (y1 - fy) - (x1 - fx) * (y0 - fy);
            bool inside = (w0 >= 0 && w1 >= 0 && w2 >= 0) ||
                          (w0 <= 0 && w1 <= 0 && w2 <= 0);
            if (!inside) continue;

            float z = (w0 * z0 + w1 * z1 + w2 * z2) * inv_area;
            if (z <= 0.0f) continue;
            uint16_t zq = (z > 65535.0f) ? 65535 : (uint16_t)(z + 0.5f);
            uint16_t *dst = &out[(size_t)py * W + px];
            if (*dst == 0 || zq < *dst) *dst = zq;
        }
    }
}

int insight9_receive_align_depth_to_rgb(const uint16_t *depth,
                                        int depth_w, int depth_h,
                                        const camera_calib *left_calib,
                                        const camera_calib *rgb_calib,
                                        uint16_t *aligned_out,
                                        int rgb_w, int rgb_h) {
    if (!depth || !left_calib || !rgb_calib || !aligned_out ||
        depth_w <= 0 || depth_h <= 0 || rgb_w <= 0 || rgb_h <= 0) {
        return -1;
    }

    // Start with all-invalid (0) output.
    memset(aligned_out, 0, (size_t)rgb_w * rgb_h * sizeof(uint16_t));

    // Intrinsics (row-major k = [fx 0 cx; 0 fy cy; 0 0 1]). Streams are
    // rectified (p == k), so no distortion is applied.
    const double dfx = left_calib->intrinsics.k[0];
    const double dfy = left_calib->intrinsics.k[4];
    const double dcx = left_calib->intrinsics.k[2];
    const double dcy = left_calib->intrinsics.k[5];
    const double rfx = rgb_calib->intrinsics.k[0];
    const double rfy = rgb_calib->intrinsics.k[4];
    const double rcx = rgb_calib->intrinsics.k[2];
    const double rcy = rgb_calib->intrinsics.k[5];
    if (dfx == 0.0 || dfy == 0.0) return -1;

    // RGB extrinsic: parent=camera_camera_left, child=camera_camera_rgb. In ROS
    // convention (t, q) maps a point rgb->left: p_left = R*p_rgb + t. We need
    // left->rgb, i.e. the inverse: p_rgb = R^T * (p_left - t).
    const double *q = rgb_calib->extrinsics.rotation;     // x, y, z, w
    const double *t = rgb_calib->extrinsics.translation;  // meters
    const double qx = q[0], qy = q[1], qz = q[2], qw = q[3];
    const double R00 = 1 - 2*(qy*qy + qz*qz);
    const double R01 = 2*(qx*qy - qz*qw);
    const double R02 = 2*(qx*qz + qy*qw);
    const double R10 = 2*(qx*qy + qz*qw);
    const double R11 = 1 - 2*(qx*qx + qz*qz);
    const double R12 = 2*(qy*qz - qx*qw);
    const double R20 = 2*(qx*qz - qy*qw);
    const double R21 = 2*(qy*qz + qx*qw);
    const double R22 = 1 - 2*(qx*qx + qy*qy);
    const double tx = t[0] * 1000.0, ty = t[1] * 1000.0, tz = t[2] * 1000.0;

    // Pass 1: project every depth pixel to RGB pixel coords + RGB-frame depth.
    // The depth grid is treated as a mesh; these are its vertices.
    const size_t n = (size_t)depth_w * depth_h;
    std::vector<float>   VX(n), VY(n), VZ(n);
    std::vector<uint8_t> OK(n, 0);
    for (int v = 0; v < depth_h; v++) {
        const uint16_t *row = depth + (size_t)v * depth_w;
        for (int u = 0; u < depth_w; u++) {
            size_t i = (size_t)v * depth_w + u;
            uint16_t d = row[u];
            if (d == 0) continue;              // no measurement

            // Deproject to the LEFT frame (mm), then left -> rgb: R^T*(p - t).
            double z = (double)d;
            double x = (u - dcx) * z / dfx;
            double y = (v - dcy) * z / dfy;
            double ax = x - tx, ay = y - ty, az = z - tz;
            double xr = R00*ax + R10*ay + R20*az;
            double yr = R01*ax + R11*ay + R21*az;
            double zr = R02*ax + R12*ay + R22*az;
            if (zr <= 0.0) continue;           // behind the RGB camera

            VX[i] = (float)(rfx * xr / zr + rcx);
            VY[i] = (float)(rfy * yr / zr + rcy);
            VZ[i] = (float)zr;
            OK[i] = 1;
        }
    }

    // Pass 2: rasterize the mesh. Each 2x2 cell -> two triangles. Triangles that
    // straddle a depth discontinuity (an object edge) are dropped to avoid
    // "rubber sheet" stretching between foreground and background.
    const float EDGE_REL = 0.05f;              // max 5% depth jump within a triangle
    auto emit = [&](size_t a, size_t b, size_t c) {
        if (!OK[a] || !OK[b] || !OK[c]) return;
        float za = VZ[a], zb = VZ[b], zc = VZ[c];
        float zmin = fminf(za, fminf(zb, zc));
        float zmax = fmaxf(za, fmaxf(zb, zc));
        if (zmin <= 0.0f || (zmax - zmin) > EDGE_REL * zmin) return;
        align_raster_triangle(aligned_out, rgb_w, rgb_h,
                              VX[a], VY[a], za,
                              VX[b], VY[b], zb,
                              VX[c], VY[c], zc);
    };
    for (int v = 0; v + 1 < depth_h; v++) {
        for (int u = 0; u + 1 < depth_w; u++) {
            size_t i00 = (size_t)v * depth_w + u;
            size_t i10 = i00 + 1;
            size_t i01 = i00 + depth_w;
            size_t i11 = i01 + 1;
            emit(i00, i10, i11);
            emit(i00, i11, i01);
        }
    }
    return 0;
}

int insight9_receive_get_current_fps(int* fps) {
    if (!fps) return -1;
    // Same as the Windows SDK: the device-side FPS selector is not reliable yet.
    *fps = 30;
    return 0;
}

int insight9_receive_get_vio_status(int* status) {
    if (!status) return -1;
    uint8_t val = 0;
    if (!callXUWithRetry("readVioStatus", [&val](viewer::ExtensionUnitControl& xu) {
            return xu.readVioStatus(val);
        })) return -1;
    *status = val;
    return 0;
}

const char* insight9_receive_get_hardware_type() {
    static std::string result;
    camera_params params;
    params.hardware_model = 0xFF;
    if (insight9_receive_get_camera_params(&params) == 0) {
        const char* models[] = {"Insight 9", "Insight 7", "Insight 3"};
        if (params.hardware_model < 3) {
            result = models[params.hardware_model];
            return result.c_str();
        }
    }
    return "unknown";
}
