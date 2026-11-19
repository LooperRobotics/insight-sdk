#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <algorithm>

#include "Insight9SDK.h"
#include <opencv2/opencv.hpp>

using Clock = std::chrono::steady_clock;

static std::atomic<bool> keep_running{true};
static std::atomic<int64_t> last_image_time{0};
static std::atomic<int64_t> last_image_time_per_cam[3] = {{0}, {0}, {0}};
static std::mutex g_sdk_op_lock;

static void signal_handler(int) {
    keep_running = false;
}

static int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        Clock::now().time_since_epoch()).count();
}

struct cam_stats {
    uint64_t cb_count;
    uint64_t last_ts;
    size_t last_size;
    int width;
    int height;
    unsigned int format;
};

struct hid_stats {
    uint64_t cb_count;
    uint64_t last_ts;
};

struct imu_latest_sample {
    float ax, ay, az;
    float gx, gy, gz;
};

struct vio_latest_sample {
    float px, py, pz;
    float qx, qy, qz, qw;
};

static struct cam_stats g_stats[3];
static struct hid_stats g_imu_stats;
static struct hid_stats g_vio_stats;
static struct imu_latest_sample g_imu_latest;
static struct vio_latest_sample g_vio_latest;
static Clock::time_point g_img_last_print;
static Clock::time_point g_hid_last_print;
static std::mutex g_stats_lock;
static int g_img_stats_inited = 0;
static int g_hid_stats_inited = 0;
static int vio_status_raw = -1;

static int gray_fps_state = 0;
static const int GRAY_FPS_VALUES[] = {20,30,40,50,60};

// ==================== Display / depth-to-RGB alignment ====================
// Grid layout (single fixed window):
//   top-left = left eye, bottom-left = right eye,
//   top-right = raw depth, bottom-right = RGB with aligned depth overlaid.
enum { PANEL_LEFT = 0, PANEL_RIGHT = 1, PANEL_DEPTH = 2, PANEL_RGB = 3, PANEL_COUNT = 4 };
static const int CELL_W = 544;            // sensor panel width
static const int CELL_H = 640;            // sensor panel height (one stereo eye / valid depth rows)
static const int RGB_W  = 544;            // RGB shown at half of its 1088x1920 native size
static const int RGB_H  = 960;
static const double DISPLAY_SCALE = 0.6;  // shrink the composite so the window fits the screen

// Producer/consumer staging: the capture callback only copies raw bytes here so
// it returns immediately (never throttling the SDK's per-camera capture thread).
struct raw_frame {
    std::vector<uint8_t> data;
    int width = 0;
    int height = 0;
    unsigned int fmt = 0;
    bool has_new = false;
};
static std::mutex g_raw_lock;
static std::condition_variable g_raw_cv;
static raw_frame g_raw[3];

// Decoded panels + colorized aligned depth, written by worker threads and read
// by the display thread. g_panel_lock guards this handoff.
static std::mutex g_panel_lock;
static std::condition_variable g_panel_cv;   // signalled when a panel changes
static uint64_t g_panel_seq = 0;             // bumped under g_panel_lock per decoded frame
// Size of the latest decoded RGB frame; depth is only aligned when it matches
// the RGB calibration, otherwise the overlay could not be used anyway.
static std::atomic<int> g_rgb_w{0};
static std::atomic<int> g_rgb_h{0};
static cv::Mat g_panel[PANEL_COUNT];
static cv::Mat g_aligned_color;   // aligned depth, colorized (RGB-sized, BGR)
static cv::Mat g_aligned_mask;    // valid-pixel mask for the overlay (RGB-sized, 8U)

// Calibration for depth->RGB alignment, read once at startup.
static bool g_calib_ready = false;
static camera_calib g_left_calib;
static camera_calib g_rgb_calib;

static double get_elapsed_seconds(Clock::time_point start, Clock::time_point end) {
    return std::chrono::duration<double>(end - start).count();
}

static void reset_img_stats(void) {
    for (int i = 0; i < 3; i++) {
        g_stats[i].cb_count = 0;
    }
}

static void reset_hid_stats(void) {
    g_imu_stats.cb_count = 0;
    g_vio_stats.cb_count = 0;
}

static void ensure_img_stats_initialized_locked(void) {
    if (!g_img_stats_inited) {
        reset_img_stats();
        g_img_last_print = Clock::now();
        g_img_stats_inited = 1;
    }
}

static void ensure_hid_stats_initialized_locked(void) {
    if (!g_hid_stats_inited) {
        reset_hid_stats();
        g_hid_last_print = Clock::now();
        g_hid_stats_inited = 1;
    }
}

static const char* image_format_to_string(unsigned int format) {
    switch (format) {
        case 0x47504A4D: return "MJPEG";  // 'MJPG'
        case 0x59455247: return "GREY";   // 'GREY'
        case 0x36315A:   return "Z16";    // 'Z16 '
        case 0x32595559: return "YUYV";   // 'YUYV'
        case 0x55595659: return "YVYU";   // 'YVYU'
        case 0x49385956: return "Y8I";    // 'Y8I '
        case 0x42475200: return "RGB8";   // 'RGB8'
        case 0x3231564E: return "NV12";   // 'NV12'
        default: return "UNKNOWN";
    }
}

static const char* pixel_format_to_string(PixelFormat format) {
    switch (format) {
        case PixelFormat::MJPEG: return "MJPEG";
        case PixelFormat::GREY:  return "GREY";
        case PixelFormat::Z16:   return "Z16";
        case PixelFormat::RGB8:  return "RGB8";
        case PixelFormat::Y8I:   return "Y8I";
        case PixelFormat::YUYV:  return "YUYV";
        case PixelFormat::YVYU:  return "YVYU";
        case PixelFormat::NV12:  return "NV12";
        default: return "UNKNOWN";
    }
}


static const char* vio_status_to_string(VioStatus status) {
    switch (status) {
        case VioStatus::NOT_INITED:         return "NOT_INITED";
        case VioStatus::RESTARTING:         return "RESTARTING";
        case VioStatus::STOPPED:            return "STOPPED";
        case VioStatus::TRACKING:           return "TRACKING";
        case VioStatus::TRACKING_STATIC:    return "TRACKING_STATIC";
        case VioStatus::TRACKINGLOST:       return "TRACKINGLOST";
        case VioStatus::DATA_LOST:          return "DATA_LOST";
        default:                            return "UNKNOWN";
    }
}

static void maybe_print_img_stats_locked(void) {
    Clock::time_point now = Clock::now();
    double elapsed = get_elapsed_seconds(g_img_last_print, now);
    if (elapsed < 1.0) return;

    printf("\n========= Image Callbacks : 1/s =========\n");
    for (int i = 0; i < 3; i++) {
        const struct cam_stats* cs = &g_stats[i];
        printf("IMG[%d]: fps=%.1f ts=%llu size=%zu %dx%d format=%s\n",
               i,
               cs->cb_count / elapsed,
               (unsigned long long)cs->last_ts,
               cs->last_size,
               cs->width,
               cs->height,
               image_format_to_string(cs->format));
    }
    fflush(stdout);

    reset_img_stats();
    g_img_last_print = now;
}

static void maybe_print_hid_stats_locked(void) {
    Clock::time_point now = Clock::now();
    double elapsed = get_elapsed_seconds(g_hid_last_print, now);
    if (elapsed < 0.5) return;

    printf("\n========= HID Callbacks : 0.5/s =========\n");
    printf("IMU: hz=%.1f ax=%f ay=%f az=%f gx=%f gy=%f gz=%f ts=%llu\n",
           g_imu_stats.cb_count / elapsed,
           g_imu_latest.ax, g_imu_latest.ay, g_imu_latest.az,
           g_imu_latest.gx, g_imu_latest.gy, g_imu_latest.gz,
           (unsigned long long)g_imu_stats.last_ts);
    printf("VIO: hz=%.1f pos=(%f %f %f) ori=(%f %f %f %f) ts=%llu\n",
           g_vio_stats.cb_count / elapsed,
           g_vio_latest.px, g_vio_latest.py, g_vio_latest.pz,
           g_vio_latest.qx, g_vio_latest.qy, g_vio_latest.qz, g_vio_latest.qw,
           (unsigned long long)g_vio_stats.last_ts);
    fflush(stdout);

    if (insight9_receive_get_vio_status(&vio_status_raw) == 0) {
        VioStatus vio_status = static_cast<VioStatus>(vio_status_raw);
        printf("\nCurrent VIO status: %s (%d)\n\n", vio_status_to_string(vio_status), vio_status_raw);
    } else {
        printf("Failed to get VIO status\n");
    }

    reset_hid_stats();
    g_hid_last_print = now;
}

static void update_hid_stats(struct hid_stats* s, uint64_t ts) {
    s->cb_count++;
    s->last_ts = ts;
}

void print_device_capabilities() {
    printf("\n========================================\n");
    printf("    Device Capabilities Information\n");
    printf("========================================\n");

    const DeviceCapabilities_t* caps = insight9_receive_get_device_capabilities_ptr();
    if (!caps) {
        printf("Failed to get device capabilities (SDK not initialized?)\n");
        return;
    }

    const char* names[] = {"RGB", "Gray", "Depth"};
    const std::vector<DeviceCapability>* lists[] = {
        &caps->rgb_capabilities, &caps->gray_capabilities, &caps->depth_capabilities
    };
    const DeviceCapability* defaults[] = {&caps->rgb_default, &caps->gray_default, &caps->depth_default};

    for (int cam_id = 0; cam_id < 3; ++cam_id) {
        printf("\n[%s Camera - cam_id: %d]\n", names[cam_id], cam_id);
        printf("  Total capabilities: %zu\n", lists[cam_id]->size());
        if (defaults[cam_id]->valid) {
            printf("  Default: %dx%d@%d, format=%s\n",
                   defaults[cam_id]->width,
                   defaults[cam_id]->height,
                   defaults[cam_id]->fps,
                   pixel_format_to_string(defaults[cam_id]->format));
        }
        printf("  All supported formats:\n");
        for (size_t i = 0; i < lists[cam_id]->size(); ++i) {
            const auto& cap = (*lists[cam_id])[i];
            printf("    [%zu] %dx%d@%d, format=%s\n",
                   i, cap.width, cap.height, cap.fps,
                   pixel_format_to_string(cap.format));
        }
    }

    printf("\n========================================\n");
}

bool select_best_resolution(int cam_id, int target_width, int target_height,
                            int target_fps, DeviceCapability* selected) {
    if (!selected) return false;

    int count = 0;
    if (insight9_receive_get_device_capability_count(cam_id, &count) != 0 || count == 0) {
        return false;
    }

    int best_score = -1;
    DeviceCapability best_cap;
    best_cap.valid = false;

    for (int i = 0; i < count; ++i) {
        DeviceCapability cap;
        if (insight9_receive_get_device_capability_by_index(cam_id, i, &cap) != 0) {
            continue;
        }

        int score = abs(cap.width - target_width) +
                   abs(cap.height - target_height) +
                   abs(cap.fps - target_fps) * 2;

        if (best_score == -1 || score < best_score) {
            best_score = score;
            best_cap = cap;
        }
    }

    if (best_cap.valid) {
        *selected = best_cap;
        return true;
    }

    return false;
}

void my_image_cb(int cam_id, uint8_t* data, size_t size,
                 int width, int height, unsigned int format,
                 uint64_t timestamp, void* userdata) {
    if (cam_id < 0 || cam_id >= 3) return;
    last_image_time = now_ms();
    last_image_time_per_cam[cam_id] = now_ms();

    {
        std::lock_guard<std::mutex> lk(g_stats_lock);
        ensure_img_stats_initialized_locked();
        struct cam_stats* s = &g_stats[cam_id];
        s->cb_count++;
        s->last_ts = timestamp;
        s->last_size = size;
        s->width = width;
        s->height = height;
        s->format = format;
        maybe_print_img_stats_locked();
    }

    // Cheap hand-off to the per-camera worker: just copy the raw bytes.
    {
        std::lock_guard<std::mutex> lk(g_raw_lock);
        raw_frame &rf = g_raw[cam_id];
        rf.data.assign(data, data + size);
        rf.width = width;
        rf.height = height;
        rf.fmt = format;
        rf.has_new = true;
    }
    g_raw_cv.notify_all();
}

void my_imu_cb(float ax, float ay, float az,
               float gx, float gy, float gz,
               uint64_t timestamp, void* userdata) {
    std::lock_guard<std::mutex> lk(g_stats_lock);
    ensure_hid_stats_initialized_locked();
    update_hid_stats(&g_imu_stats, timestamp);
    g_imu_latest.ax = ax; g_imu_latest.ay = ay; g_imu_latest.az = az;
    g_imu_latest.gx = gx; g_imu_latest.gy = gy; g_imu_latest.gz = gz;
    maybe_print_hid_stats_locked();
}

void my_vio_cb(float px, float py, float pz,
               float qx, float qy, float qz, float qw,
               uint64_t timestamp, void* userdata) {
    std::lock_guard<std::mutex> lk(g_stats_lock);
    ensure_hid_stats_initialized_locked();
    update_hid_stats(&g_vio_stats, timestamp);
    g_vio_latest.px = px; g_vio_latest.py = py; g_vio_latest.pz = pz;
    g_vio_latest.qx = qx; g_vio_latest.qy = qy; g_vio_latest.qz = qz; g_vio_latest.qw = qw;
    maybe_print_hid_stats_locked();
}


// Decode one raw frame into BGR panel(s); for depth, also align it to RGB and
// build the colorized overlay. Runs on a per-camera worker thread.
static void process_raw_frame(int cam_id, const raw_frame& rf) {
    const uint8_t* data = rf.data.data();
    size_t size = rf.data.size();
    int w = rf.width, h = rf.height;

    if (rf.fmt == 0x47504A4D) {  // MJPEG
        // cam0 RGB. Reject truncated frames: require SOI (FFD8)...EOI (FFD9).
        // The firmware pads frames to a multiple of 8 bytes, so EOI may be
        // followed by a few filler bytes; find it and drop the padding.
        size_t end = size;
        while (end >= 4 && size - end < 64 &&
               !(data[end - 2] == 0xFF && data[end - 1] == 0xD9)) {
            --end;
        }
        bool valid_jpeg = end >= 4 && size - end < 64 &&
                          data[0] == 0xFF && data[1] == 0xD8 &&
                          data[end - 2] == 0xFF && data[end - 1] == 0xD9;
        if (!valid_jpeg) return;
        size = end;

        // Strip the firmware's non-standard 16-byte APP1 "TS__" segment after
        // the SOI (its bogus length otherwise desyncs libjpeg -> garbled color).
        std::vector<uint8_t> buf;
        if (size > 18 && data[2] == 0xFF && data[3] == 0xE1 &&
            memcmp(data + 6, "TS__", 4) == 0) {
            buf.reserve(size - 16);
            buf.insert(buf.end(), data, data + 2);
            buf.insert(buf.end(), data + 18, data + size);
        } else {
            buf.assign(data, data + size);
        }

        cv::Mat rgb = cv::imdecode(buf, cv::IMREAD_COLOR);
        if (rgb.empty()) return;
        std::lock_guard<std::mutex> lk(g_panel_lock);
        g_panel[PANEL_RGB] = rgb;
        g_rgb_w = rgb.cols;
        g_rgb_h = rgb.rows;
    } else if (rf.fmt == 0x59455247) {  // GREY
        // cam1 stereo grayscale: two eyes stacked vertically (final row = timestamps).
        if (w <= 0 || h <= 0 || size < (size_t)w * h) return;
        int eye_h = (h - 1) / 2;
        cv::Mat full(h, w, CV_8UC1, (void*)data);
        cv::Mat left_bgr, right_bgr;
        cv::cvtColor(full.rowRange(0, eye_h), left_bgr, cv::COLOR_GRAY2BGR);
        cv::cvtColor(full.rowRange(eye_h, 2 * eye_h), right_bgr, cv::COLOR_GRAY2BGR);
        std::lock_guard<std::mutex> lk(g_panel_lock);
        g_panel[PANEL_LEFT] = left_bgr;
        g_panel[PANEL_RIGHT] = right_bgr;
    } else if (rf.fmt == 0x36315A) {  // Z16
        // cam2 depth: 16-bit; last 2 rows are metadata.
        if (w <= 0 || h <= 0 || size < (size_t)w * h * 2) return;
        int valid_h = h - 2;

        // Raw depth colormap for the top-right panel.
        cv::Mat depth(valid_h, w, CV_16UC1, (void*)data);
        cv::Mat depth8, depth_bgr;
        cv::normalize(depth, depth8, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::applyColorMap(depth8, depth_bgr, cv::COLORMAP_JET);

        // Align the depth onto the RGB image plane and colorize it for overlay.
        cv::Mat acolor, amask;
        if (g_calib_ready &&
            g_rgb_w == (int)g_rgb_calib.intrinsics.width &&
            g_rgb_h == (int)g_rgb_calib.intrinsics.height) {
            int rgb_w = (int)g_rgb_calib.intrinsics.width;
            int rgb_h = (int)g_rgb_calib.intrinsics.height;
            if (rgb_w > 0 && rgb_h > 0) {
                std::vector<uint16_t> aligned((size_t)rgb_w * rgb_h);
                if (insight9_receive_align_depth_to_rgb(
                        (const uint16_t*)data, w, valid_h,
                        &g_left_calib, &g_rgb_calib,
                        aligned.data(), rgb_w, rgb_h) == 0) {
                    cv::Mat a16(rgb_h, rgb_w, CV_16UC1, aligned.data());
                    amask = a16 > 0;
                    double dmin = 0, dmax = 0;
                    cv::minMaxLoc(a16, &dmin, &dmax, nullptr, nullptr, amask);
                    cv::Mat a8;
                    double scale = (dmax > dmin) ? 255.0 / (dmax - dmin) : 1.0;
                    a16.convertTo(a8, CV_8UC1, scale, -dmin * scale);
                    cv::applyColorMap(a8, acolor, cv::COLORMAP_JET);
                    acolor.setTo(cv::Scalar(0, 0, 0), ~amask);
                }
            }
        }

        std::lock_guard<std::mutex> lk(g_panel_lock);
        g_panel[PANEL_DEPTH] = depth_bgr;
        g_aligned_color = acolor;   // empty if calib/align unavailable
        g_aligned_mask = amask;
    } else if (rf.fmt == 0x32595559 || rf.fmt == 0x55595659) { // YUYV / YVYU
        if (w <= 0 || h <= 0 || size < (size_t)w * h * 2) return;

        cv::Mat bgr;
        cv::Mat yuyv_mat(h, w, CV_8UC2, (void*)data);
        cv::cvtColor(yuyv_mat, bgr, rf.fmt == 0x32595559 ? cv::COLOR_YUV2BGR_YUYV : cv::COLOR_YUV2BGR_YVYU);

        std::lock_guard<std::mutex> lk(g_panel_lock);
        g_panel[PANEL_RGB] = bgr;
        g_rgb_w = bgr.cols;
        g_rgb_h = bgr.rows;
    } else if (rf.fmt == 0x3231564E) { // NV12
        if (w <= 0 || h <= 0 || size < (size_t)w * h * 3 / 2) return;

        cv::Mat bgr;
        cv::Mat nv12_mat(h * 3 / 2, w, CV_8UC1, (void*)data);
        cv::cvtColor(nv12_mat, bgr, cv::COLOR_YUV2BGR_NV12);

        std::lock_guard<std::mutex> lk(g_panel_lock);
        g_panel[PANEL_RGB] = bgr;
        g_rgb_w = bgr.cols;
        g_rgb_h = bgr.rows;
    } else if (rf.fmt == 0x49385956) { // Y8I
        if (w <= 0 || h <= 0 || size < (size_t)w * h * 2) return;

        cv::Mat interleaved(h, w, CV_8UC2, (void*)data);
        std::vector<cv::Mat> channels(2);
        cv::split(interleaved, channels);

        cv::Mat left_bgr, right_bgr;
        cv::cvtColor(channels[0], left_bgr, cv::COLOR_GRAY2BGR);
        cv::cvtColor(channels[1], right_bgr, cv::COLOR_GRAY2BGR);

        std::lock_guard<std::mutex> lk(g_panel_lock);
        g_panel[PANEL_LEFT] = left_bgr;
        g_panel[PANEL_RIGHT] = right_bgr;
    }
}

// One dedicated worker thread per camera: waits for its latest raw frame and
// decodes independently, so a slow stream never delays the others.
static void camera_worker(int cam_id) {
    while (keep_running) {
        raw_frame local;
        {
            std::unique_lock<std::mutex> lk(g_raw_lock);
            // Timed wait: the SIGINT handler cannot notify the condition variable.
            g_raw_cv.wait_for(lk, std::chrono::milliseconds(100), [cam_id] {
                return !keep_running || g_raw[cam_id].has_new;
            });
            if (!keep_running) break;
            if (!g_raw[cam_id].has_new) continue;
            std::swap(local, g_raw[cam_id]);
            g_raw[cam_id].has_new = false;
        }
        process_raw_frame(cam_id, local);
        {
            std::lock_guard<std::mutex> lk(g_panel_lock);
            ++g_panel_seq;
        }
        g_panel_cv.notify_one();
    }
}

void toggle_gray_camera_fps() {
    int64_t now = now_ms();
    int64_t grayLast = last_image_time_per_cam[1];
    constexpr int64_t kGrayStreamTimeoutMs = 3000;

    if (grayLast == 0 || (now - grayLast) > kGrayStreamTimeoutMs) {
        printf("[FPS] Gray camera has no active video stream, skip FPS switch\n");
        return;
    }

    int new_fps = GRAY_FPS_VALUES[gray_fps_state];
    printf("new_fps = %d | gray_fps_state = %d\n", new_fps, gray_fps_state);
    gray_fps_state = (gray_fps_state + 1) % 5;

    printf("\n========= Switching Gray Camera FPS =========\n");
    printf("New FPS: %d\n", new_fps);

    std::lock_guard<std::mutex> lk(g_sdk_op_lock);
    if (insight9_receive_switch_camera_fps(1, new_fps) == 0) {
        printf("Gray camera switched to %d FPS\n", new_fps);
    } else {
        printf("Failed to switch gray camera FPS!\n");
    }
}

static void read_calibration(void) {
    static const int calib_cams[] = {
        INSIGHT9_CALIB_CAM_LEFT, INSIGHT9_CALIB_CAM_RIGHT, INSIGHT9_CALIB_CAM_RGB
    };
    static const char *calib_names[] = {"left", "right", "rgb"};
    camera_calib calib[3];
    bool ok[3] = {false, false, false};
    for (int i = 0; i < 3; i++) {
        ok[i] = insight9_receive_get_camera_calib(calib_cams[i], &calib[i]) == 0;
        if (ok[i]) {
            printf("Calibration for %s camera:\n", calib_names[i]);
            insight9_receive_print_camera_calib(&calib[i]);
        } else {
            printf("Failed to get calibration for %s camera\n", calib_names[i]);
        }
    }

    // Keep the LEFT + RGB calibration used to align depth onto RGB.
    if (ok[0] && ok[2]) {
        g_left_calib = calib[0];
        g_rgb_calib = calib[2];
        g_calib_ready = true;
        printf("Depth->RGB alignment enabled (RGB %ux%u).\n",
               g_rgb_calib.intrinsics.width, g_rgb_calib.intrinsics.height);
    } else {
        printf("Calibration unavailable; depth->RGB overlay disabled.\n");
    }
}

static void extension_unit_demo(bool write_params) {
    int active_cam = -1;
    camera_params params = {0};

    printf("\n================ Extension Unit Control Test ================\n");
    printf("\n---------------- camera params control ----------------\n");

    if (insight9_receive_get_active_camera(&active_cam) == 0) {
        printf("\nCurrent active camera: %d\n", active_cam);
        if (insight9_receive_get_camera_params(&params) == 0) {
            printf("Camera parameters retrieved successfully\n");
            insight9_receive_print_camera_params(&params);
        } else {
            printf("Failed to get camera parameters\n");
        }
    } else {
        printf("Failed to get active camera\n");
    }

    if (insight9_receive_get_camera_params_for(1, &params) == 0) {
        printf("Camera parameters for cam 1 retrieved successfully\n");
        insight9_receive_print_camera_params(&params);
    } else {
        printf("Failed to get camera parameters for cam 1\n");
    }

    // Writing parameters changes the RGB sensor (manual exposure, gain, ...),
    // which can lower its frame rate, so it is opt-in.
    if (!write_params) {
        printf("Skipping camera parameter write (run with --set-params to test it)\n");
    } else {
        params.cam_id = 0;
        params.brightness = 80.0f;
        params.contrast = 1.2f;
        params.exposure_time = 0.015f;
        params.exposure_gain = 4.0f;
        params.auto_white_balance = 1;
        params.resolution = 0;
        params.frame_rate = 2;
        params.auto_exposure = 0;
        params.gamma_dark = 2.0f;
        params.hue = 40.0f;
        params.saturation = 1.0f;
        params.sharpness = 128;
        params.white_balance = 2.0f;
        params.decimation = 1;

        if (insight9_receive_set_camera_params_for(0, &params) == 0) {
            printf("Camera parameters set successfully\n");
            if (insight9_receive_get_camera_params_for(0, &params) == 0) {
                printf("Camera parameters for cam 0 after setting:\n");
                insight9_receive_print_camera_params(&params);
            } else {
                printf("Failed to get camera parameters for cam 0 after setting\n");
            }
        } else {
            printf("Failed to set camera parameters (invalid range or XU not available)\n");
        }
    }

    printf("\n---------------- get looper info ----------------\n");

    int fps = 0;
    if (insight9_receive_get_current_fps(&fps) == 0) {
        printf("\nCurrent FPS for gray camera: %d\n", fps);
    } else {
        printf("Failed to get current FPS for gray camera\n");
    }

    printf("Current connected hardware type: %s\n", insight9_receive_get_hardware_type());
}

static void print_metadata_timestamps(void) {
    for (int cam = 0; cam < 3; ++cam) {
        uint64_t ts = 0;
        if (insight9_receive_read_metadata_timestamp(cam, &ts) == 0) {
            printf("metadata timestamp (cam %d): %llu\n", cam, (unsigned long long)ts);
        } else {
            printf("metadata timestamp (cam %d): not available yet\n", cam);
        }
    }
}

static void usage(const char* prog) {
    printf("Usage: sudo %s [--no-gui] [--set-params] [--rgb WxH@FPS:FMT]\n", prog);
    printf("  --no-gui       print statistics only, do not open the OpenCV window\n");
    printf("  --set-params   also write demo camera parameters to the RGB camera\n");
    printf("  --no-xu-demo   skip the parameter/calibration reads before start (no depth overlay)\n");
    printf("  --rgb MODE     RGB mode instead of the device default, e.g. 640x480@30:mjpeg,\n");
    printf("                 1088x1920@30:yuyv (FMT: mjpeg, yuyv, nv12)\n");
}

// Parse "WxH@FPS:FMT", e.g. "640x480@30:mjpeg".
static bool parse_rgb_mode(const char* s, int* w, int* h, int* fps, PixelFormat* fmt) {
    char name[16] = {0};
    if (sscanf(s, "%dx%d@%d:%15s", w, h, fps, name) != 4) return false;
    if (strcmp(name, "mjpeg") == 0)      *fmt = PixelFormat::MJPEG;
    else if (strcmp(name, "yuyv") == 0)  *fmt = PixelFormat::YUYV;
    else if (strcmp(name, "nv12") == 0)  *fmt = PixelFormat::NV12;
    else return false;
    return *w > 0 && *h > 0 && *fps > 0;
}

int main(int argc, char** argv) {
    bool gui = true;
    bool write_params = false;
    bool xu_demo = true;
    const char* rgb_mode = NULL;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--no-gui") == 0) {
            gui = false;
        } else if (strcmp(argv[i], "--set-params") == 0) {
            write_params = true;
        } else if (strcmp(argv[i], "--no-xu-demo") == 0) {
            xu_demo = false;
        } else if (strcmp(argv[i], "--rgb") == 0 && i + 1 < argc) {
            rgb_mode = argv[++i];
        } else {
            usage(argv[0]);
            return strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0 ? 0 : 1;
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (insight9_receive_init_default() != 0) {
        fprintf(stderr, "SDK init failed\n");
        return -1;
    }

    print_device_capabilities();

    if (rgb_mode) {
        int w = 0, h = 0, fps = 0;
        PixelFormat fmt = PixelFormat::Unknown;
        if (!parse_rgb_mode(rgb_mode, &w, &h, &fps, &fmt)) {
            fprintf(stderr, "Invalid --rgb mode '%s'\n", rgb_mode);
            usage(argv[0]);
            insight9_receive_cleanup();
            return 1;
        }
        // Before start() this only stores the config; start() opens with it.
        if (insight9_receive_switch_camera_config(0, w, h, fmt, fps) != 0) {
            fprintf(stderr, "Cannot use RGB mode %s\n", rgb_mode);
        } else {
            printf("RGB mode: %dx%d@%d %s\n", w, h, fps, pixel_format_to_string(fmt));
        }
    }

    printf("\n========================================\n");
    printf("    Resolution Selection Demo\n");
    printf("========================================\n");

    DeviceCapability selected;
    if (select_best_resolution(0, 1920, 1080, 30, &selected)) {
        printf("Best match for 1920x1080@30 on RGB: %dx%d@%d, format=%s\n",
               selected.width, selected.height, selected.fps,
               pixel_format_to_string(selected.format));
    }

    if (select_best_resolution(1, 640, 480, 30, &selected)) {
        printf("Best match for 640x480@30 on Gray: %dx%d@%d, format=%s\n",
               selected.width, selected.height, selected.fps,
               pixel_format_to_string(selected.format));
    }

    if (select_best_resolution(2, 640, 480, 30, &selected)) {
        printf("Best match for 640x480@30 on Depth: %dx%d@%d, format=%s\n",
               selected.width, selected.height, selected.fps,
               pixel_format_to_string(selected.format));
    }
    printf("========================================\n\n");

    insight9_receive_register_image_callback(my_image_cb, NULL);
    insight9_receive_register_imu_callback(my_imu_cb, NULL);
    insight9_receive_register_vio_callback(my_vio_cb, NULL);

    if (insight9_receive_start() != 0) {
        fprintf(stderr, "SDK start failed\n");
        insight9_receive_cleanup();
        return -1;
    }

    // Parameter/calibration reads go after start(), as in the Windows example.
    // Doing them before the streams were started left the composite function
    // broken (depth at ~4x its frame rate, no gray frames). They cost a short
    // frame-rate dip in the first seconds instead.
    if (xu_demo) {
        extension_unit_demo(write_params);
        read_calibration();
    }

    print_metadata_timestamps();
    last_image_time = now_ms();

    std::thread fps_switch_thread([]() {
        while (keep_running) {
            for (int i = 0; i < 100 && keep_running; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (keep_running) {
                // toggle_gray_camera_fps();
                // insight9_receive_switch_camera_format(0, 720, 1200, PixelFormat::NV12);
            }
        }
    });

    printf("SDK started with 2 UVC functions\n");
    printf("  Camera 0: RGB\n");
    printf("  Camera 1: Composite stream (Gray frames)\n");
    printf("  Camera 2: Composite stream (Depth frames)\n");
    printf("SDK running, press %s to stop...\n", gui ? "q / Esc in the window or Ctrl+C" : "Ctrl+C");

    // One worker thread per stream (independent decoding + alignment).
    std::thread workers[3];
    for (int cam = 0; cam < 3; cam++) {
        workers[cam] = std::thread(camera_worker, cam);
    }

    if (gui) {
        // Fixed composite canvas layout, laid out directly at display size so
        // each panel is resized once (no full-size canvas + second resize).
        auto scaled = [](int x, int y, int w, int h) {
            return cv::Rect((int)(x * DISPLAY_SCALE), (int)(y * DISPLAY_SCALE),
                            (int)(w * DISPLAY_SCALE), (int)(h * DISPLAY_SCALE));
        };
        const cv::Rect roi_left  = scaled(0, 0, CELL_W, CELL_H);           // top-left
        const cv::Rect roi_right = scaled(0, CELL_H, CELL_W, CELL_H);      // bottom-left
        const cv::Rect roi_depth = scaled(CELL_W, 0, CELL_W, CELL_H);      // top-right
        const cv::Rect roi_rgb   = scaled(CELL_W, CELL_H, RGB_W, RGB_H);   // bottom-right
        const int canvas_w = (int)((CELL_W + std::max(CELL_W, RGB_W)) * DISPLAY_SCALE);
        const int canvas_h = (int)(std::max(CELL_H + CELL_H, CELL_H + RGB_H) * DISPLAY_SCALE);
        cv::Mat canvas = cv::Mat::zeros(canvas_h, canvas_w, CV_8UC3);

        // macOS: HighGUI windows must be created and pumped on the main thread.
        const char* win_name = "Insight9 - L/R / Depth / RGB+AlignedDepth";
        cv::namedWindow(win_name, cv::WINDOW_AUTOSIZE);

        // Scale preserving aspect ratio and letterbox, instead of stretching.
        auto blit = [](cv::Mat& canvas, const cv::Mat& panel, const cv::Rect& roi) {
            if (panel.empty()) return;
            cv::Mat dst = canvas(roi);
            double scale = std::min((double)dst.cols / panel.cols, (double)dst.rows / panel.rows);
            int newW = (int)(panel.cols * scale);
            int newH = (int)(panel.rows * scale);
            cv::Mat resized;
            cv::resize(panel, resized, cv::Size(newW, newH));
            dst.setTo(cv::Scalar(0, 0, 0));
            cv::Mat sub = dst(cv::Rect((dst.cols - newW) / 2, (dst.rows - newH) / 2, newW, newH));
            resized.copyTo(sub);
        };

        uint64_t shown_seq = 0;
        bool pending = false;                 // a newer frame is waiting to be drawn
        Clock::time_point last_draw = Clock::now();
        const auto min_draw_interval = std::chrono::milliseconds(15);   // <= ~60 redraws/s

        while (keep_running) {
            // Redraw when a worker has decoded a new frame, instead of on a
            // fixed timer that is not synchronised with the camera. The timeout
            // keeps the window responsive while no frames arrive.
            cv::Mat left, right, depth, rgb, acolor, amask;
            {
                std::unique_lock<std::mutex> lk(g_panel_lock);
                const bool updated = g_panel_cv.wait_for(lk, std::chrono::milliseconds(pending ? 5 : 30), [&] {
                    return g_panel_seq != shown_seq || !keep_running;
                });
                shown_seq = g_panel_seq;
                pending = pending || updated;
                left = g_panel[PANEL_LEFT];
                right = g_panel[PANEL_RIGHT];
                depth = g_panel[PANEL_DEPTH];
                rgb = g_panel[PANEL_RGB];
                acolor = g_aligned_color;
                amask = g_aligned_mask;
            }

            // Several streams update within a few ms of each other; draw them
            // together instead of once per decoded frame.
            if (pending && Clock::now() - last_draw >= min_draw_interval) {
                pending = false;
                last_draw = Clock::now();

                // Bottom-right = RGB with the aligned depth semi-transparently overlaid.
                cv::Mat rgb_view = rgb;
                if (!rgb.empty() && !acolor.empty() && acolor.size() == rgb.size() &&
                    !amask.empty() && amask.size() == rgb.size()) {
                    rgb_view = rgb.clone();
                    cv::Mat blend;
                    cv::addWeighted(rgb, 0.5, acolor, 0.5, 0, blend);
                    blend.copyTo(rgb_view, amask);
                }

                blit(canvas, left, roi_left);
                blit(canvas, right, roi_right);
                blit(canvas, depth, roi_depth);
                blit(canvas, rgb_view, roi_rgb);
                cv::imshow(win_name, canvas);
            }

            // waitKey only pumps window events here; the pacing comes from the
            // frame wait above.
            int key = cv::waitKey(1);
            if (key == 'q' || key == 27) {
                keep_running = false;
            }
        }
        cv::destroyAllWindows();
    } else {
        while (keep_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // Wake and join the worker threads.
    g_raw_cv.notify_all();
    for (int cam = 0; cam < 3; cam++) {
        if (workers[cam].joinable()) workers[cam].join();
    }
    if (fps_switch_thread.joinable()) fps_switch_thread.join();

    insight9_receive_stop();
    insight9_receive_cleanup();
    printf("Program exited.\n");
    return 0;
}
