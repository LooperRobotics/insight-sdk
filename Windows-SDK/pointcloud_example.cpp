// Insight9 point cloud example.
//
// Walks through the point cloud API end to end:
//   1. Depth (Z16) -> single-frame point cloud in the LEFT camera frame,
//      coloured from the LEFT eye or the RGB camera
//      (insight9_receive_depth_to_pointcloud).
//   2. VIO pose -> T_world_left -> the same cloud in the world frame
//      (insight9_pointcloud_pose_to_transform + params.transform).
//   3. World-frame clouds -> global map with keyframe selection and voxel
//      downsampling (insight9_pointcloud_map_*).
//   4. A live 3D preview, and the last frame / the map saved as PLY files.
//
// Usage: pointcloud_example [--no-gui] [--color gray|rgb|none] [--step N]
//                           [--max-depth M] [--no-filter] [--voxel V]
//                           [--no-map] [--body-to-left flu|calib|none]
//                           [--out PREFIX]  (default prefix: device model)
//
// Preview window: drag with the left mouse button to orbit, wheel or +/- to
// zoom, r = reset view, , and . = max depth -/+ 0.5 m, 1-4 = pixel step,
// m = toggle frame/map view, s = save PLY,
// d = save the raw depth frame as a 16-bit PNG,
// c = clear the map, q/ESC = quit. Ctrl+C also quits. PLY files are written
// on exit as well.
//
// This file is identical in Linux-SDK, MAC-SDK and Windows-SDK; the platform
// block below selects the header, the camera ids and the pixel-format codes.

#if defined(_WIN32)
#define _CRT_SECURE_NO_WARNINGS
#define NOMINMAX
#include <windows.h>
#else
#include <signal.h>
#endif

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <opencv2/opencv.hpp>

// ==================== Platform block ====================
#if defined(__linux__)
#include "Insight_9_receive.h"
#include <linux/videodev2.h>
// Linux camera ids: depth 0, gray 1, RGB 2.
static const int kCamDepth = INSIGHT9_CAM_DEPTH;
static const int kCamGray  = INSIGHT9_CAM_GRAY;
static const int kCamRgb   = INSIGHT9_CAM_RGB;
static const unsigned kFmtZ16   = V4L2_PIX_FMT_Z16;
static const unsigned kFmtGrey  = V4L2_PIX_FMT_GREY;
static const unsigned kFmtY8I   = V4L2_PIX_FMT_Y8I;
static const unsigned kFmtMjpeg = V4L2_PIX_FMT_MJPEG;
static const unsigned kFmtYuyv  = V4L2_PIX_FMT_YUYV;
static const unsigned kFmtYvyu  = V4L2_PIX_FMT_YVYU;
static const unsigned kFmtNv12  = V4L2_PIX_FMT_NV12;
#else
#include "Insight9SDK.h"
// macOS / Windows camera ids: RGB 0, gray 1, depth 2.
static const int kCamRgb   = 0;
static const int kCamGray  = 1;
static const int kCamDepth = 2;
static const unsigned kFmtZ16   = 0x36315A;    // "Z16"
static const unsigned kFmtGrey  = 0x59455247;  // "GREY"
static const unsigned kFmtY8I   = 0x49385956;  // "Y8I"
static const unsigned kFmtMjpeg = 0x47504A4D;  // "MJPG"
static const unsigned kFmtYuyv  = 0x32595559;  // "YUY2"
static const unsigned kFmtYvyu  = 0x55595659;  // "YVYU"
static const unsigned kFmtNv12  = 0x3231564E;  // "NV12"
#endif

// ==================== LEFT camera -> VIO body extrinsic ====================
// insight9_pointcloud_pose_to_transform() needs the fixed transform from the
// LEFT camera to the VIO body frame (parent = VIO body, child = LEFT camera,
// p_body = R * p_left + t). Getting it wrong is what smears the map: every
// frame lands rotated/offset, differently for every camera orientation.
//
// --body-to-left selects the source:
//   calib (default) The LEFT calibration's own extrinsic. On Insight9 its
//         parent is the IMU ("camera_camera_imu"), which is the VIO body:
//         the IMU has the optical axes (x right, y down, z forward), so the
//         rotation is identity and the offset a few centimetres.
//   flu   A body with x forward, y left, z up at the LEFT camera
//         (kBodyToLeftFlu below), for devices whose VIO body is FLU.
//   none  NULL: the VIO pose is taken as the LEFT camera pose itself.
//
// Check: in map view, turn the camera in place. With the right extrinsic the
// map stays sharp; with a wrong one it smears or the world spins with you.
static const camera_extrinsics kBodyToLeftFlu = {
    "body",                  // parent_frame_id
    "left",                  // child_frame_id
    {0.0, 0.0, 0.0},         // translation x, y, z (m)
    {-0.5, 0.5, -0.5, 0.5},  // quaternion x, y, z, w: optical -> FLU axes
};

// ==================== Options ====================
enum ColorMode { COLOR_GRAY, COLOR_RGB, COLOR_NONE };
enum BodyToLeft { BTL_FLU, BTL_CALIB, BTL_NONE };

struct Options {
    bool gui = true;
    ColorMode color = COLOR_GRAY;
    int pixel_step = 1;
    float max_depth = 8.0f;
    bool filter = true;
    float voxel = 0.05f;
    float map_max_depth = 5.0f;
    int pose_delay_ms = 0;
    bool use_map = true;
    BodyToLeft body_to_left = BTL_CALIB;
    std::string out_prefix;  // empty: named after the device model
};

static void usage(const char *prog) {
    printf("Usage: %s [options]\n", prog);
    printf("  --no-gui            console output only\n");
    printf("  --color gray|rgb|none\n");
    printf("                      point colour: LEFT eye (default, pixel-aligned with\n");
    printf("                      depth), RGB camera, or none (shade by depth)\n");
    printf("  --step N            use every Nth depth pixel (default 1)\n");
    printf("  --max-depth M       drop points farther than M metres (default 8.0)\n");
    printf("  --no-filter         keep flying pixels at depth edges\n");
    printf("  --voxel V           map voxel size in metres (default 0.05)\n");
    printf("  --map-max-depth M   only points within M metres go into the map\n");
    printf("                      (default 5.0; far stereo depth is noisy)\n");
    printf("  --pose-delay MS     look up the VIO pose MS milliseconds before the\n");
    printf("                      depth frame's timestamp (default 0)\n");
    printf("  --no-map            camera-frame clouds only, no VIO / global map\n");
    printf("  --body-to-left flu|calib|none\n");
    printf("                      LEFT camera -> VIO body extrinsic (default calib,\n");
    printf("                      see the top of pointcloud_example.cpp)\n");
    printf("  --out PREFIX        PLY file prefix (default: the device model,\n");
    printf("                      e.g. insight9 / insight7 / insight3)\n");
}

// Default output prefix from the device model: "Insight 3" -> "insight3".
// Returns "" when the model cannot be read yet.
static std::string model_prefix() {
    const char *hw = insight9_receive_get_hardware_type();
    if (!hw || strcmp(hw, "unknown") == 0) return std::string();
    std::string out;
    for (const char *c = hw; *c; c++) {
        if (isalnum((unsigned char)*c)) out += (char)tolower((unsigned char)*c);
    }
    return out;
}

static bool parse_args(int argc, char **argv, Options *o) {
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        const char *next = (i + 1 < argc) ? argv[i + 1] : NULL;
        if (strcmp(a, "--no-gui") == 0) {
            o->gui = false;
        } else if (strcmp(a, "--no-map") == 0) {
            o->use_map = false;
        } else if (strcmp(a, "--no-filter") == 0) {
            o->filter = false;
        } else if (strcmp(a, "--color") == 0 && next) {
            i++;
            if (strcmp(next, "gray") == 0) o->color = COLOR_GRAY;
            else if (strcmp(next, "rgb") == 0) o->color = COLOR_RGB;
            else if (strcmp(next, "none") == 0) o->color = COLOR_NONE;
            else return false;
        } else if (strcmp(a, "--body-to-left") == 0 && next) {
            i++;
            if (strcmp(next, "flu") == 0) o->body_to_left = BTL_FLU;
            else if (strcmp(next, "calib") == 0) o->body_to_left = BTL_CALIB;
            else if (strcmp(next, "none") == 0) o->body_to_left = BTL_NONE;
            else return false;
        } else if (strcmp(a, "--step") == 0 && next) {
            o->pixel_step = atoi(argv[++i]);
            if (o->pixel_step < 1) return false;
        } else if (strcmp(a, "--max-depth") == 0 && next) {
            o->max_depth = (float)atof(argv[++i]);
        } else if (strcmp(a, "--map-max-depth") == 0 && next) {
            o->map_max_depth = (float)atof(argv[++i]);
        } else if (strcmp(a, "--pose-delay") == 0 && next) {
            o->pose_delay_ms = atoi(argv[++i]);
        } else if (strcmp(a, "--voxel") == 0 && next) {
            o->voxel = (float)atof(argv[++i]);
        } else if (strcmp(a, "--out") == 0 && next) {
            o->out_prefix = argv[++i];
        } else {
            return false;
        }
    }
    return true;
}

// ==================== Capture hand-off ====================
// Callbacks run on SDK threads and their buffers are reused, so they only copy
// the latest frame into a slot; the main thread does all the work.
static std::atomic<bool> g_running{true};

static int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Frame {
    std::vector<uint8_t> data;
    int width = 0;
    int height = 0;
    unsigned fmt = 0;
    uint64_t ts = 0;        // device timestamp from the SDK
    int64_t host_ms = 0;    // when the callback delivered it
};

struct Slot {
    std::mutex lock;
    Frame frame;
    bool has_new = false;
};

static Slot g_depth_slot, g_rgb_slot, g_gray_slot;
static std::condition_variable g_depth_cv;
static std::atomic<bool> g_want_rgb{false}, g_want_gray{false};
static std::atomic<int> g_gray_frames{0};   // for the fps printout

// Recent VIO poses (last 2 s), so each depth frame can be paired with the
// pose from when it was captured rather than the newest one.
struct Pose {
    float p[3];
    float q[4];         // x, y, z, w
    uint64_t dev_ts;    // device timestamp from the SDK
    int64_t host_ms;    // when the callback delivered it
};
static std::mutex g_pose_lock;
static std::deque<Pose> g_poses;

static void put(Slot &s, const uint8_t *data, size_t size,
                int w, int h, unsigned fmt, uint64_t ts) {
    std::lock_guard<std::mutex> lk(s.lock);
    s.frame.data.assign(data, data + size);
    s.frame.width = w;
    s.frame.height = h;
    s.frame.fmt = fmt;
    s.frame.ts = ts;
    s.frame.host_ms = now_ms();
    s.has_new = true;
}

// Swap the slot's frame into `out` (keeps both buffers allocated).
static bool take(Slot &s, Frame &out) {
    std::lock_guard<std::mutex> lk(s.lock);
    if (!s.has_new) return false;
    std::swap(out, s.frame);
    s.has_new = false;
    return true;
}

static bool is_gray_format(unsigned fmt) {
    return fmt == kFmtGrey || fmt == kFmtY8I || fmt == kFmtYuyv || fmt == kFmtYvyu;
}

static void on_image(int cam_id, uint8_t *data, size_t size, int w, int h,
                     unsigned int fmt, uint64_t ts, void *) {
    if (!data || size == 0) return;
    if (cam_id == kCamDepth && fmt == kFmtZ16) {
        put(g_depth_slot, data, size, w, h, fmt, ts);
        g_depth_cv.notify_one();
    } else if (cam_id == kCamRgb && g_want_rgb) {
        put(g_rgb_slot, data, size, w, h, fmt, ts);
    } else if (cam_id == kCamGray) {
        g_gray_frames++;
        if (g_want_gray && is_gray_format(fmt)) put(g_gray_slot, data, size, w, h, fmt, ts);
    }
}

static void on_vio(float px, float py, float pz,
                   float qx, float qy, float qz, float qw,
                   uint64_t ts, void *) {
    Pose p = {{px, py, pz}, {qx, qy, qz, qw}, ts, now_ms()};
    std::lock_guard<std::mutex> lk(g_pose_lock);
    g_poses.push_back(p);
    while (!g_poses.empty() && g_poses.front().host_ms < p.host_ms - 2000) g_poses.pop_front();
}

// Pose at time `at`, interpolated between the two poses around it. `key`
// gives a pose's time on the same clock and in the same unit as `at`.
// Fails if `at` is older than the history or more than `max_ahead` past the
// newest pose (VIO stalled).
template <typename Key>
static bool pose_at(int64_t at, int64_t max_ahead, Key key, Pose *out) {
    std::lock_guard<std::mutex> lk(g_pose_lock);
    if (g_poses.empty() || at < key(g_poses.front())) return false;
    const Pose &last = g_poses.back();
    if (at >= key(last)) {
        if (at - key(last) > max_ahead) return false;
        *out = last;
        return true;
    }
    size_t i = 1;
    while (key(g_poses[i]) < at) i++;
    const Pose &a = g_poses[i - 1], &b = g_poses[i];
    const double span = (double)(key(b) - key(a));
    const float k = span > 0 ? (float)((at - key(a)) / span) : 1.0f;
    float dot = 0;
    for (int j = 0; j < 4; j++) dot += a.q[j] * b.q[j];
    const float sb = dot < 0 ? -1.0f : 1.0f;   // shortest path
    float n = 0;
    for (int j = 0; j < 3; j++) out->p[j] = a.p[j] + k * (b.p[j] - a.p[j]);
    for (int j = 0; j < 4; j++) {
        out->q[j] = a.q[j] + k * (sb * b.q[j] - a.q[j]);
        n += out->q[j] * out->q[j];
    }
    n = sqrtf(n);
    for (int j = 0; j < 4; j++) out->q[j] /= n;
    out->dev_ts = b.dev_ts;
    out->host_ms = b.host_ms;
    return true;
}

// Divisor that turns a VIO timestamp into the image timestamp unit (us):
// 1000 if VIO counts nanoseconds on the same device clock, 1 if microseconds,
// 0 if the two do not look like the same clock.
static int64_t vio_ts_divisor(uint64_t image_us) {
    uint64_t vio;
    {
        std::lock_guard<std::mutex> lk(g_pose_lock);
        if (g_poses.empty()) return 0;
        vio = g_poses.back().dev_ts;
    }
    const int64_t img = (int64_t)image_us;
    if (llabs((int64_t)(vio / 1000) - img) < 2000000) return 1000;
    if (llabs((int64_t)vio - img) < 2000000) return 1;
    return 0;
}

#if defined(_WIN32)
static BOOL WINAPI on_console_ctrl(DWORD type) {
    if (type == CTRL_C_EVENT || type == CTRL_BREAK_EVENT || type == CTRL_CLOSE_EVENT) {
        g_running = false;
        g_depth_cv.notify_all();
        return TRUE;
    }
    return FALSE;
}
#else
static void on_signal(int) {
    g_running = false;   // the main loop wakes up on its own within 30 ms
}
#endif

// ==================== Depth preparation ====================
// The calibration is for one resolution, while depth follows the stereo
// stream's resolution. Find the valid depth rows (a Z16 frame normally ends
// with 2 metadata rows) and scale the LEFT intrinsics to the depth size.
static bool depth_geometry(const camera_calib &calib, int w, int h,
                           int *valid_h, camera_calib *scaled) {
    const camera_intrinsics &in = calib.intrinsics;
    if (w <= 0 || h <= 2 || in.width == 0 || in.height == 0) return false;
    const float sx = (float)w / (float)in.width;
    const int expected_h = (int)lroundf((float)in.height * sx);
    *valid_h = (h == expected_h + 2) ? h - 2 : h;
    const float sy = (float)*valid_h / (float)in.height;

    *scaled = calib;
    float *k = scaled->intrinsics.k;
    k[0] *= sx; k[2] *= sx;   // fx, cx
    k[4] *= sy; k[5] *= sy;   // fy, cy
    scaled->intrinsics.width = (uint32_t)w;
    scaled->intrinsics.height = (uint32_t)*valid_h;
    return true;
}

// Stereo depth interpolates a trail of "flying pixels" between foreground and
// background at object edges. Drop pixels whose depth jumps away from a
// 4-neighbour, and isolated pixels whose neighbours are mostly invalid.
static void filter_flying_pixels(const uint16_t *src, int w, int h, std::vector<uint16_t> &dst) {
    dst.assign(src, src + (size_t)w * h);
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            const int d = src[y * w + x];
            if (d == 0) continue;
            const int thresh = 10 + d * 3 / 100;   // 10 mm + 3 %
            const int nb[4] = {
                x > 0     ? src[y * w + x - 1]   : 0,
                x < w - 1 ? src[y * w + x + 1]   : 0,
                y > 0     ? src[(y - 1) * w + x] : 0,
                y < h - 1 ? src[(y + 1) * w + x] : 0,
            };
            int invalid = 0;
            bool jump = false;
            for (int v : nb) {
                if (v == 0) { invalid++; continue; }
                if (abs(v - d) > thresh) { jump = true; break; }
            }
            if (jump || invalid >= 3) dst[y * w + x] = 0;
        }
    }
}

// When the device's depth/VIO pipeline is not running (still starting, or
// stuck until the device is replugged) the firmware sends a placeholder Z16
// frame that draws the text "Depth not enabled or not ready". Deprojected, it
// becomes floating text, so such frames are skipped. Signs:
//   - under 5 % of the pixels hold depth (the text covers about 2 %; real
//     depth that sparse is unusable anyway);
//   - the frame is bit-for-bit the previous one (real depth is noisy).
static bool is_placeholder_depth(const uint16_t *d, int w, int h,
                                 std::vector<uint16_t> &prev) {
    const size_t n = (size_t)w * h;
    const bool repeated = prev.size() == n && memcmp(prev.data(), d, n * 2) == 0;
    prev.assign(d, d + n);
    if (repeated) return true;
    size_t valid = 0;
    for (size_t i = 0; i < n; i++) valid += d[i] != 0;
    return valid < n * 5 / 100;
}

// ==================== Colour sources ====================
// Point the params at the LEFT eye of a gray-camera frame. The LEFT eye is
// pixel-aligned with depth (GRAY8 colour), so it needs no extrinsic.
static bool set_gray_source(const Frame &f, int depth_w, int depth_h,
                            insight9_pointcloud_params *pc) {
    if (f.width != depth_w || f.data.empty()) return false;
    int eye_h, px_stride;
    if (f.fmt == kFmtGrey) {          // eyes stacked, left on top, + 1 timestamp row
        eye_h = (f.height - 1) / 2;
        px_stride = 1;
    } else if (f.fmt == kFmtYuyv || f.fmt == kFmtYvyu) {
        eye_h = f.height / 2;         // eyes stacked; luma is every other byte
        px_stride = 2;
    } else if (f.fmt == kFmtY8I) {    // left/right interleaved per pixel
        eye_h = f.height;
        px_stride = 2;
    } else {
        return false;
    }
    if (eye_h < depth_h || f.data.size() < (size_t)f.width * f.height * px_stride) return false;
    pc->color_format = INSIGHT9_PC_COLOR_GRAY8;
    pc->color = f.data.data();
    pc->color_width = depth_w;
    pc->color_height = depth_h;
    pc->color_pixel_stride = px_stride;
    pc->color_row_stride = f.width * px_stride;
    return true;
}

// Decode an RGB-camera frame to BGR. MJPEG frames can carry a non-standard
// APP1 "TS__" segment and padding after the end marker; both are removed.
static bool decode_rgb(const Frame &f, cv::Mat &bgr) {
    const uint8_t *d = f.data.data();
    size_t size = f.data.size();
    int w = f.width, h = f.height;

    if (f.fmt == kFmtMjpeg) {
        if (size < 4 || d[0] != 0xFF || d[1] != 0xD8) return false;
        size_t eoi = 0;
        size_t scan_from = (size > 64) ? size - 64 : 2;
        for (size_t i = size - 1; i >= scan_from + 1; i--) {
            if (d[i - 1] == 0xFF && d[i] == 0xD9) { eoi = i + 1; break; }
        }
        if (eoi == 0) return false;   // truncated frame
        size = eoi;

        std::vector<uint8_t> buf;
        if (size > 18 && d[2] == 0xFF && d[3] == 0xE1 && memcmp(d + 6, "TS__", 4) == 0) {
            buf.reserve(size - 16);
            buf.insert(buf.end(), d, d + 2);
            buf.insert(buf.end(), d + 18, d + size);
        } else {
            buf.assign(d, d + size);
        }
        bgr = cv::imdecode(buf, cv::IMREAD_COLOR);
        return !bgr.empty();
    }
    if (f.fmt == kFmtYuyv) {
        if (w <= 0 || h <= 0 || size < (size_t)w * h * 2) return false;
        cv::Mat yuyv(h, w, CV_8UC2, (void *)d);
        cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV);
        return true;
    }
    if (f.fmt == kFmtNv12) {
        if (w <= 0 || h <= 0 || size < (size_t)w * h * 3 / 2) return false;
        cv::Mat nv12(h * 3 / 2, w, CV_8UC1, (void *)d);
        cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
        return true;
    }
    return false;
}

// RGB intrinsics scaled to the decoded image. Fails when the image does not
// have the calibration's aspect ratio (e.g. a rotated or cropped mode).
static bool scale_rgb_calib(const camera_calib &calib, int w, int h, camera_calib *out) {
    const camera_intrinsics &in = calib.intrinsics;
    if (in.width == 0 || in.height == 0 || w <= 0 || h <= 0) return false;
    const float sx = (float)w / (float)in.width;
    const float sy = (float)h / (float)in.height;
    if (fabsf(sx - sy) > 0.02f * sx) return false;
    *out = calib;
    float *k = out->intrinsics.k;
    k[0] *= sx; k[2] *= sx;
    k[4] *= sy; k[5] *= sy;
    out->intrinsics.width = (uint32_t)w;
    out->intrinsics.height = (uint32_t)h;
    return true;
}

// ==================== Global map ====================
// Copy the whole map. The first call only asks for the size; if another
// thread grows the map in between, map_get returns 1 and we try again.
static bool fetch_map(insight9_pointcloud_map *map, std::vector<insight9_point> &out) {
    for (int attempt = 0; attempt < 3; attempt++) {
        size_t count = 0;
        if (insight9_pointcloud_map_get(map, NULL, 0, &count) != 0) return false;
        out.resize(count);
        size_t got = 0;
        int r = insight9_pointcloud_map_get(map, out.data(), out.size(), &got);
        if (r < 0) return false;
        out.resize(got);
        if (r == 0) return true;
    }
    return true;   // truncated, but good enough for a preview
}

// The SDK map voxel-filters each keyframe on its own, so overlapping
// keyframes stack their depth noise. For display and export, merge the whole
// map into one point per voxel (mean position and colour) and drop voxels
// seen only once, which are mostly noise.
static void voxel_merge(const std::vector<insight9_point> &in, float voxel,
                        std::vector<insight9_point> &out) {
    out.clear();
    if (voxel <= 0.0f) { out = in; return; }
    struct Acc { float x, y, z; uint32_t r, g, b, n; };
    std::unordered_map<uint64_t, Acc> grid;
    grid.reserve(in.size() / 4 + 16);
    const float inv = 1.0f / voxel;
    for (const insight9_point &p : in) {
        const uint64_t key = ((uint64_t)((int64_t)floorf(p.x * inv) + (1 << 20)) & 0x1FFFFF) << 42 |
                             ((uint64_t)((int64_t)floorf(p.y * inv) + (1 << 20)) & 0x1FFFFF) << 21 |
                             ((uint64_t)((int64_t)floorf(p.z * inv) + (1 << 20)) & 0x1FFFFF);
        Acc &a = grid[key];
        a.x += p.x; a.y += p.y; a.z += p.z;
        a.r += (p.rgb >> 16) & 0xFF; a.g += (p.rgb >> 8) & 0xFF; a.b += p.rgb & 0xFF;
        a.n++;
    }
    out.reserve(grid.size());
    for (const auto &kv : grid) {
        const Acc &a = kv.second;
        if (a.n < 2) continue;
        insight9_point p;
        p.x = a.x / a.n; p.y = a.y / a.n; p.z = a.z / a.n;
        p.rgb = ((a.r / a.n) << 16) | ((a.g / a.n) << 8) | (a.b / a.n);
        out.push_back(p);
    }
}

// ==================== PLY export ====================
static bool save_ply(const std::string &path, const insight9_point *pts, size_t n) {
    FILE *f = fopen(path.c_str(), "wb");
    if (!f) {
        fprintf(stderr, "Cannot write %s\n", path.c_str());
        return false;
    }
    fprintf(f, "ply\nformat binary_little_endian 1.0\nelement vertex %zu\n"
               "property float x\nproperty float y\nproperty float z\n"
               "property uchar red\nproperty uchar green\nproperty uchar blue\n"
               "end_header\n", n);
    std::vector<uint8_t> buf(n * 15);
    for (size_t i = 0; i < n; i++) {
        uint8_t *b = &buf[i * 15];
        memcpy(b, &pts[i].x, 12);              // x, y, z
        b[12] = (uint8_t)(pts[i].rgb >> 16);   // 0x00RRGGBB
        b[13] = (uint8_t)(pts[i].rgb >> 8);
        b[14] = (uint8_t)(pts[i].rgb);
    }
    bool ok = fwrite(buf.data(), 1, buf.size(), f) == buf.size();
    ok = (fclose(f) == 0) && ok;
    printf("Saved %zu points to %s\n", n, path.c_str());
    return ok;
}

// ==================== Preview ====================
// Orbit camera around a point 1.5 m in front of the LEFT camera. With yaw =
// pitch = 0 it sits behind the sensor looking forward.
struct ViewState {
    float yaw = 0.0f, pitch = 15.0f;   // degrees
    float distance = 2.5f;             // metres from the orbit target
    int splat = 3;                     // point size in pixels
    bool dragging = false;
    int last_x = 0, last_y = 0;
};
static ViewState g_view;

static void on_mouse(int event, int x, int y, int flags, void *) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        g_view.dragging = true;
        g_view.last_x = x; g_view.last_y = y;
    } else if (event == cv::EVENT_LBUTTONUP) {
        g_view.dragging = false;
    } else if (event == cv::EVENT_MOUSEMOVE && g_view.dragging) {
        g_view.yaw += (x - g_view.last_x) * 0.4f;
        g_view.pitch = std::min(89.0f, std::max(-89.0f, g_view.pitch + (y - g_view.last_y) * 0.4f));
        g_view.last_x = x; g_view.last_y = y;
    } else if (event == cv::EVENT_MOUSEWHEEL) {
        float k = cv::getMouseWheelDelta(flags) > 0 ? 0.9f : 1.1f;
        g_view.distance = std::min(20.0f, std::max(0.3f, g_view.distance * k));
    }
}

// Render points with a z-buffer. T: pose of the points' frame relative to the
// LEFT camera frame used for the view (T_world_left of the current view), or
// NULL when the points are already in the LEFT camera frame.
static void render(const insight9_point *pts, size_t n, const float *T, cv::Mat &img) {
    const int W = 960, H = 720;
    const float f = 620.0f, cx = W * 0.5f, cy = H * 0.5f;   // ~60 deg vertical FOV

    // View basis in the LEFT optical frame (x right, y down, z forward).
    const float target[3] = {0.0f, 0.0f, 1.5f};
    const float yaw = g_view.yaw * 0.0174533f, pitch = g_view.pitch * 0.0174533f;
    const float fw[3] = {sinf(yaw) * cosf(pitch), sinf(pitch), cosf(yaw) * cosf(pitch)};
    const float eye[3] = {target[0] - g_view.distance * fw[0],
                          target[1] - g_view.distance * fw[1],
                          target[2] - g_view.distance * fw[2]};
    // right = normalize(down x forward), down' = forward x right
    float rt[3] = {fw[2], 0.0f, -fw[0]};
    float rn = sqrtf(rt[0] * rt[0] + rt[2] * rt[2]);
    rt[0] /= rn; rt[2] /= rn;
    const float dn[3] = {fw[1] * rt[2] - fw[2] * rt[1],
                         fw[2] * rt[0] - fw[0] * rt[2],
                         fw[0] * rt[1] - fw[1] * rt[0]};

    img.create(H, W, CV_8UC3);
    img.setTo(cv::Scalar(25, 20, 15));
    cv::Mat zbuf(H, W, CV_32F, cv::Scalar(1e9f));
    const int s = g_view.splat;

    for (size_t i = 0; i < n; i++) {
        float x = pts[i].x, y = pts[i].y, z = pts[i].z;
        if (T) {   // world -> LEFT: R^T * (p - t)
            float dx = x - T[3], dy = y - T[7], dz = z - T[11];
            x = T[0] * dx + T[4] * dy + T[8] * dz;
            y = T[1] * dx + T[5] * dy + T[9] * dz;
            z = T[2] * dx + T[6] * dy + T[10] * dz;
        }
        const float qx = x - eye[0], qy = y - eye[1], qz = z - eye[2];
        const float vx = qx * rt[0] + qy * rt[1] + qz * rt[2];
        const float vy = qx * dn[0] + qy * dn[1] + qz * dn[2];
        const float vz = qx * fw[0] + qy * fw[1] + qz * fw[2];
        if (vz < 0.05f) continue;
        const int u = (int)(f * vx / vz + cx) - s / 2;
        const int v = (int)(f * vy / vz + cy) - s / 2;
        if (u < 0 || v < 0 || u + s > W || v + s > H) continue;

        cv::Vec3b c;
        const uint32_t rgb = pts[i].rgb;
        if (rgb) {
            c = cv::Vec3b((uint8_t)rgb, (uint8_t)(rgb >> 8), (uint8_t)(rgb >> 16));
        } else {   // no colour: near = warm, far = cool
            float t = std::min(1.0f, std::max(0.0f, z / 4.0f));
            c = cv::Vec3b((uint8_t)(60 + 180 * t), (uint8_t)(200 - 120 * t), (uint8_t)(255 - 175 * t));
        }
        for (int dv = 0; dv < s; dv++) {
            float *zr = zbuf.ptr<float>(v + dv);
            cv::Vec3b *ir = img.ptr<cv::Vec3b>(v + dv);
            for (int du = 0; du < s; du++) {
                if (vz < zr[u + du]) { zr[u + du] = vz; ir[u + du] = c; }
            }
        }
    }
}

static const char *vio_status_name(int s) {
    switch (s) {
        case (int)VioStatus::NOT_INITED:      return "NOT_INITED";
        case (int)VioStatus::RESTARTING:      return "RESTARTING";
        case (int)VioStatus::STOPPED:         return "STOPPED";
        case (int)VioStatus::TRACKING:        return "TRACKING";
        case (int)VioStatus::TRACKING_STATIC: return "TRACKING_STATIC";
        case (int)VioStatus::TRACKINGLOST:    return "TRACKINGLOST";
        case (int)VioStatus::DATA_LOST:       return "DATA_LOST";
        default:                              return "UNKNOWN";
    }
}

// ==================== main ====================
int main(int argc, char **argv) {
    Options opt;
    if (!parse_args(argc, argv, &opt)) {
        usage(argv[0]);
        return 1;
    }

#if defined(_WIN32)
    SetConsoleCtrlHandler(on_console_ctrl, TRUE);
#else
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
#endif

    if (insight9_receive_init_default() != 0) {
        fprintf(stderr, "SDK init failed\n");
        return 1;
    }
    insight9_receive_register_image_callback(on_image, NULL);
    if (opt.use_map) insight9_receive_register_vio_callback(on_vio, NULL);
    if (insight9_receive_start() != 0) {
        fprintf(stderr, "SDK start failed\n");
        insight9_receive_cleanup();
        return 1;
    }

    // ---- Calibration ----
    // Read lazily, once real depth arrives, as Looper Viewer does. Every read
    // is a UVC control transfer, and control transfers sent while the streams
    // are still coming up can keep the device from starting depth (it then
    // keeps sending the "not ready" placeholder). Retried every 3 s on failure.
    camera_calib left_calib, rgb_calib;
    bool have_calib = false;
    int64_t calib_retry_at = 0;
    g_want_rgb = (opt.color == COLOR_RGB);
    g_want_gray = (opt.color == COLOR_GRAY);

    auto load_calib = [&]() -> bool {
        if (insight9_receive_get_camera_calib(INSIGHT9_CALIB_CAM_LEFT, &left_calib) != 0 ||
            left_calib.intrinsics.width == 0 || left_calib.intrinsics.height == 0 ||
            left_calib.intrinsics.k[0] <= 0.0f || left_calib.intrinsics.k[4] <= 0.0f) {
            printf("Cannot read the LEFT calibration yet; retrying in 3 s\n");
            return false;
        }
        if (opt.out_prefix.empty()) {
            opt.out_prefix = model_prefix();
            if (!opt.out_prefix.empty())
                printf("Device model: %s (PLY prefix '%s')\n",
                       insight9_receive_get_hardware_type(), opt.out_prefix.c_str());
        }
        const camera_extrinsics &e = left_calib.extrinsics;
        printf("LEFT calib: %ux%u fx=%.2f fy=%.2f cx=%.2f cy=%.2f\n",
               left_calib.intrinsics.width, left_calib.intrinsics.height,
               left_calib.intrinsics.k[0], left_calib.intrinsics.k[4],
               left_calib.intrinsics.k[2], left_calib.intrinsics.k[5]);
        printf("LEFT extrinsic: parent '%s' -> child '%s', t=(%.4f %.4f %.4f) q=(%.4f %.4f %.4f %.4f)\n",
               e.parent_frame_id, e.child_frame_id,
               e.translation[0], e.translation[1], e.translation[2],
               e.rotation[0], e.rotation[1], e.rotation[2], e.rotation[3]);

        if (opt.color == COLOR_RGB) {
            if (insight9_receive_get_camera_calib(INSIGHT9_CALIB_CAM_RGB, &rgb_calib) == 0 &&
                rgb_calib.intrinsics.width > 0 && rgb_calib.intrinsics.height > 0) {
                printf("RGB calib : %ux%u, extrinsic %s -> %s\n",
                       rgb_calib.intrinsics.width, rgb_calib.intrinsics.height,
                       rgb_calib.extrinsics.parent_frame_id, rgb_calib.extrinsics.child_frame_id);
            } else {
                printf("No RGB calibration: colouring from the LEFT eye instead.\n");
                opt.color = COLOR_GRAY;
                g_want_rgb = false;
                g_want_gray = true;
            }
        }
        return true;
    };

    const camera_extrinsics *body_to_left = NULL;
    if (opt.use_map) {
        static const char *names[] = {"flu", "calib", "none"};
        if (opt.body_to_left == BTL_FLU) body_to_left = &kBodyToLeftFlu;
        else if (opt.body_to_left == BTL_CALIB) body_to_left = &left_calib.extrinsics;
        printf("LEFT -> VIO body extrinsic: %s (--body-to-left flu|calib|none)\n",
               names[opt.body_to_left]);
    }

    // Output file prefix: --out, else the device model (read with the
    // calibration, once depth runs), else a generic fallback.
    auto out_prefix = [&]() -> std::string {
        if (opt.out_prefix.empty()) opt.out_prefix = model_prefix();
        return opt.out_prefix.empty() ? std::string("insight") : opt.out_prefix;
    };

    // ---- Point cloud parameters ----
    insight9_pointcloud_params pc_defaults;
    insight9_pointcloud_default_params(&pc_defaults);
    pc_defaults.pixel_step = opt.pixel_step;
    pc_defaults.min_depth = 0.05f;
    pc_defaults.max_depth = opt.max_depth;

    insight9_pointcloud_map *map = NULL;
    if (opt.use_map) {
        insight9_pointcloud_map_params mp;
        insight9_pointcloud_map_default_params(&mp);
        mp.voxel_size = opt.voxel;
        // A bit sparser than the defaults (3 cm / 1 deg). Overlapping keyframes
        // are averaged per voxel for display (voxel_merge), which also evens
        // out their depth noise.
        mp.keyframe_translation = 0.05f;
        mp.keyframe_rotation_deg = 2.0f;
        map = insight9_pointcloud_map_create(&mp);
        if (!map) {
            fprintf(stderr, "Cannot create the point cloud map\n");
            insight9_receive_stop();
            insight9_receive_cleanup();
            return 1;
        }
    }

    const char *win = "Insight9 point cloud";
    if (opt.gui) {
        cv::namedWindow(win, cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback(win, on_mouse);
        printf("Mouse drag = orbit, wheel or +/- = zoom, [ ] = point size, r = reset view,\n"
               ", . = max depth -/+ 0.5 m, 1-4 = pixel step, "
               "m = frame/map view, s = save PLY, d = save raw depth PNG, c = clear map, q/ESC = quit\n");
    } else {
        printf("Running without GUI; Ctrl+C to stop and save PLY files.\n");
    }

    Frame depth, rgb_raw, gray;
    std::vector<uint16_t> depth_filtered, depth_prev;
    bool depth_ready = false;            // false while placeholder frames arrive
    int64_t not_ready_since = now_ms();
    bool hinted_replug = false;
    camera_calib depth_calib;            // LEFT calib scaled to the depth size
    cv::Mat bgr;                         // latest decoded RGB image
    camera_calib rgb_scaled;             // RGB calib scaled to bgr's size
    bool rgb_usable = false;
    std::vector<insight9_point> frame_pts, map_pts, map_raw, map_in;
    uint64_t last_pose_ts = 0;
    const char *sync_mode = "none";
    double proc_ms = 0, render_ms = 0;   // summed per second, for the printout
    int renders = 0;
    int64_t last_render = 0;
    size_t frame_n = 0;
    bool frame_in_world = false;
    float T_world_left[16];
    float view_T[16];                    // last good pose, for the map view
    bool have_view_T = false;
    bool show_map = false;               // start with the single-frame view

    int vio_status = -1;
    bool tracking = false;
    int64_t vio_interval = 2000;         // ms; backs off to 10 s on failure
    int64_t last_status_poll = 0, last_map_fetch = 0, last_print = now_ms();
    int depth_frames = 0, keyframes = 0;
    size_t map_count = 0;                // map size, refreshed once per second
    bool warned_color = false, warned_rgb = false;
    int last_dw = 0, last_dh = 0;
    cv::Mat view;

    while (g_running) {
        int64_t t = now_ms();

        // Poll VIO status from the main thread: it is a USB control transfer
        // and should not run on a callback thread. Like Looper Viewer, keep
        // these rare (2 s), skip them until depth runs, and back off on errors:
        // the device can stall when control transfers hit busy streams.
        if (opt.use_map && have_calib && depth_ready && t - last_status_poll >= vio_interval) {
            last_status_poll = t;
            if (insight9_receive_get_vio_status(&vio_status) == 0) {
                tracking = vio_status == (int)VioStatus::TRACKING ||
                           vio_status == (int)VioStatus::TRACKING_STATIC;
                vio_interval = 2000;
            } else {
                tracking = false;
                vio_interval = std::min<int64_t>(vio_interval * 2, 10000);
            }
        }

        // ---- Wait for the next depth frame ----
        bool got_depth = false;
        {
            std::unique_lock<std::mutex> lk(g_depth_slot.lock);
            g_depth_cv.wait_for(lk, std::chrono::milliseconds(30),
                                [] { return g_depth_slot.has_new || !g_running; });
            if (g_depth_slot.has_new) {
                std::swap(depth, g_depth_slot.frame);
                g_depth_slot.has_new = false;
                got_depth = true;
                depth_frames++;
            }
        }

        bool usable = got_depth && depth.width > 0 && depth.height > 2 &&
                      depth.data.size() >= (size_t)depth.width * depth.height * 2;

        // ---- Skip the "not ready" placeholder ----
        // Checked on all but the last 2 rows (Z16 metadata), so it needs no
        // calibration and runs before anything is sent to the device.
        if (usable) {
            const bool placeholder = is_placeholder_depth(
                reinterpret_cast<const uint16_t *>(depth.data.data()),
                depth.width, depth.height - 2, depth_prev);
            if (placeholder == depth_ready) {
                depth_ready = !placeholder;
                printf(depth_ready ? "Depth ready.\n"
                                   : "Depth not ready yet (placeholder frames); waiting...\n");
                not_ready_since = t;
                hinted_replug = false;
            }
            if (placeholder && !hinted_replug && t - not_ready_since > 5000) {
                printf("The device has sent only placeholder depth for 5 s, so its depth/VIO\n"
                       "pipeline is not running. Close other apps using the camera, then\n"
                       "unplug and replug it and run again.\n");
                hinted_replug = true;
            }
            if (placeholder) {   // nothing to show or add to the map
                frame_n = 0;
                frame_in_world = false;
                usable = false;
            }
        }

        // ---- Calibration, once depth runs ----
        if (usable && !have_calib) {
            if (t >= calib_retry_at) {
                have_calib = load_calib();
                if (!have_calib) calib_retry_at = t + 3000;
            }
            usable = have_calib;
        }

        int dh = 0;
        if (usable) usable = depth_geometry(left_calib, depth.width, depth.height, &dh, &depth_calib);
        if (usable && (depth.width != last_dw || dh != last_dh)) {
            printf("Depth %dx%d (%d valid rows): LEFT intrinsics scaled to fx=%.2f fy=%.2f cx=%.2f cy=%.2f\n",
                   depth.width, depth.height, dh, depth_calib.intrinsics.k[0], depth_calib.intrinsics.k[4],
                   depth_calib.intrinsics.k[2], depth_calib.intrinsics.k[5]);
            last_dw = depth.width; last_dh = dh;
        }

        if (usable) {
            const auto proc_start = std::chrono::steady_clock::now();
            const int dw = depth.width;
            const uint16_t *dptr = reinterpret_cast<const uint16_t *>(depth.data.data());
            if (opt.filter) {
                filter_flying_pixels(dptr, dw, dh, depth_filtered);
                dptr = depth_filtered.data();
            }
            insight9_pointcloud_params pc = pc_defaults;

            // ---- Colour source ----
            if (opt.color == COLOR_GRAY) {
                take(g_gray_slot, gray);
                if (!gray.data.empty() && !set_gray_source(gray, dw, dh, &pc) && !warned_color) {
                    printf("Gray frame %dx%d does not match depth %dx%d; points stay uncoloured.\n",
                           gray.width, gray.height, dw, dh);
                    warned_color = true;
                }
            } else if (opt.color == COLOR_RGB) {
                if (take(g_rgb_slot, rgb_raw)) {
                    cv::Mat img;
                    if (decode_rgb(rgb_raw, img)) {
                        bgr = img;
                        rgb_usable = scale_rgb_calib(rgb_calib, bgr.cols, bgr.rows, &rgb_scaled);
                        if (!rgb_usable && !warned_rgb) {
                            printf("RGB image %dx%d does not match the RGB calibration %ux%u; "
                                   "points stay uncoloured.\n", bgr.cols, bgr.rows,
                                   rgb_calib.intrinsics.width, rgb_calib.intrinsics.height);
                            warned_rgb = true;
                        }
                    }
                }
                if (rgb_usable) {
                    // RGB and depth are separate streams: colours at edges can be
                    // off by the time between the two frames.
                    pc.color_format = INSIGHT9_PC_COLOR_BGR8;   // cv::Mat byte order
                    pc.color = bgr.data;
                    pc.color_width = bgr.cols;
                    pc.color_height = bgr.rows;
                    pc.color_row_stride = (int)bgr.step;
                    pc.rgb_calib = &rgb_scaled;
                    pc.keep_uncolored = 1;   // keep points outside the RGB view (rgb = 0)
                }
            }

            // ---- World frame from the VIO pose ----
            // Pair the depth frame with the VIO pose from its capture time.
            // Image timestamps are microseconds and VIO timestamps nanoseconds
            // of the same device clock; depth reaches the host ~100 ms after
            // the matching pose, so pairing by arrival time would be off. If
            // the clocks do not match, fall back to the host arrival time.
            // --pose-delay shifts the lookup further back.
            frame_in_world = false;
            Pose p;
            bool have_pose = false;
            if (opt.use_map && tracking) {
                const int64_t div = vio_ts_divisor(depth.ts);
                if (div > 0) {
                    have_pose = pose_at((int64_t)depth.ts - opt.pose_delay_ms * 1000, 100000,
                                        [div](const Pose &q) { return (int64_t)(q.dev_ts / div); }, &p);
                    sync_mode = "device clock";
                } else {
                    have_pose = pose_at(depth.host_ms - opt.pose_delay_ms, 100,
                                        [](const Pose &q) { return q.host_ms; }, &p);
                    sync_mode = "host arrival time";
                }
            }
            if (have_pose) {
                last_pose_ts = p.dev_ts;
                if (insight9_pointcloud_pose_to_transform(
                        p.p[0], p.p[1], p.p[2], p.q[0], p.q[1], p.q[2], p.q[3],
                        body_to_left, T_world_left) == 0) {
                    pc.transform = T_world_left;
                    frame_in_world = true;
                    memcpy(view_T, T_world_left, sizeof(view_T));
                    have_view_T = true;
                }
            }

            // ---- Depth -> point cloud ----
            size_t cap = insight9_pointcloud_max_points(dw, dh, pc.pixel_step);
            if (frame_pts.size() < cap) frame_pts.resize(cap);
            if (insight9_receive_depth_to_pointcloud(dptr, dw, dh, &depth_calib, &pc,
                                                     frame_pts.data(), frame_pts.size(),
                                                     &frame_n) == 0) {
                // ---- Global map ----
                // Only points within --map-max-depth of the camera: far stereo
                // depth is noisy and would blur the accumulated map.
                if (frame_in_world) {
                    const float r2 = opt.map_max_depth * opt.map_max_depth;
                    const float cx = T_world_left[3], cy = T_world_left[7], cz = T_world_left[11];
                    map_in.clear();
                    for (size_t i = 0; i < frame_n; i++) {
                        const insight9_point &q = frame_pts[i];
                        const float dx = q.x - cx, dy = q.y - cy, dz = q.z - cz;
                        if (dx * dx + dy * dy + dz * dz <= r2) map_in.push_back(q);
                    }
                    if (insight9_pointcloud_map_insert(map, map_in.data(), map_in.size(),
                                                       T_world_left) == 1) {
                        keyframes++;
                    }
                }
            } else {
                frame_n = 0;
                fprintf(stderr, "depth_to_pointcloud failed (depth %dx%d)\n", dw, dh);
            }
            proc_ms += std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - proc_start).count();
        }

        // ---- Console statistics, once per second ----
        if (t - last_print >= 1000) {
            size_t map_n = 0;
            if (map) insight9_pointcloud_map_get(map, NULL, 0, &map_n);
            map_count = map_n;
            const double secs = (t - last_print) / 1000.0;
            printf("  time per depth frame %.1f ms, per preview %.1f ms\n",
                   depth_frames ? proc_ms / depth_frames : 0.0, renders ? render_ms / renders : 0.0);
            proc_ms = render_ms = 0;
            renders = 0;
            printf("depth %.1f fps (%s), gray %.1f fps | frame %zu pts (%s) | map %zu pts, %d keyframes | VIO %s\n",
                   depth_frames / secs, depth_ready ? "ok" : "NOT READY",
                   g_gray_frames.exchange(0) / secs, frame_n,
                   frame_in_world ? "world" : "camera", map_n, keyframes,
                   opt.use_map ? vio_status_name(vio_status) : "off");
            if (opt.use_map) {
                printf("  pose sync: %s | depth ts %llu us, VIO pose after it %llu\n",
                       sync_mode, (unsigned long long)depth.ts, (unsigned long long)last_pose_ts);
            }
            fflush(stdout);
            depth_frames = 0;
            last_print = t;
        }

        // ---- Preview, at most 15 times per second ----
        // Drawing and the HighGUI event loop are slow on some hosts; every
        // depth frame is still processed and added to the map in between.
        if (opt.gui && now_ms() - last_render >= 66) {
            const auto render_start = std::chrono::steady_clock::now();
            last_render = now_ms();
            const bool map_view = show_map && map;
            if (map_view && t - last_map_fetch >= 500) {
                if (fetch_map(map, map_raw)) voxel_merge(map_raw, opt.voxel, map_pts);
                map_count = map_raw.size();
                last_map_fetch = t;
            }
            if (map_view) {
                render(map_pts.data(), map_pts.size(), have_view_T ? view_T : NULL, view);
            } else {
                render(frame_pts.data(), frame_n, frame_in_world ? T_world_left : NULL, view);
            }
            char line[200];
            snprintf(line, sizeof(line), "%s view (m) | frame %zu pts | map %zu pts | max %.1f m | step %d | VIO %s",
                     map_view ? "MAP" : "FRAME", frame_n, map_count, pc_defaults.max_depth,
                     pc_defaults.pixel_step,
                     opt.use_map ? vio_status_name(vio_status) : "off");
            cv::putText(view, line, cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
            if (!depth_ready) {
                cv::putText(view, "waiting for depth (device sends a 'not ready' placeholder)",
                            cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
            } else if (map_view && !frame_in_world) {
                cv::putText(view, "not tracking: map not updated", cv::Point(10, 50),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
            }
            cv::imshow(win, view);

            int key = cv::waitKey(1);
            render_ms += std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - render_start).count();
            renders++;
            if (key == 'q' || key == 27) {
                g_running = false;
            } else if (key == 'm' && map) {
                show_map = !show_map;
            } else if (key == 'r') {
                g_view = ViewState();
            } else if (key == '+' || key == '=') {
                g_view.distance = std::max(0.3f, g_view.distance * 0.9f);
            } else if (key == '-') {
                g_view.distance = std::min(20.0f, g_view.distance * 1.1f);
            } else if (key >= '1' && key <= '4') {
                pc_defaults.pixel_step = key - '0';
                printf("pixel step %d\n", pc_defaults.pixel_step);
            } else if (key == ',') {
                pc_defaults.max_depth = std::max(0.5f, pc_defaults.max_depth - 0.5f);
                printf("max depth %.1f m\n", pc_defaults.max_depth);
            } else if (key == '.') {
                pc_defaults.max_depth = std::min(20.0f, pc_defaults.max_depth + 0.5f);
                printf("max depth %.1f m\n", pc_defaults.max_depth);
            } else if (key == '[') {
                g_view.splat = std::max(1, g_view.splat - 1);
            } else if (key == ']') {
                g_view.splat = std::min(6, g_view.splat + 1);
            } else if (key == 'c' && map) {
                insight9_pointcloud_map_clear(map);
                map_pts.clear();
                map_count = 0;
                keyframes = 0;
                printf("Map cleared\n");
            } else if (key == 'd' && !depth.data.empty()) {
                // Raw depth as a 16-bit PNG (mm), for checking what the device sends.
                cv::Mat raw(depth.height, depth.width, CV_16UC1, depth.data.data());
                std::string path = out_prefix() + "_depth_raw.png";
                cv::imwrite(path, raw);
                printf("Saved raw depth %dx%d to %s\n", depth.width, depth.height, path.c_str());
            } else if (key == 's') {
                save_ply(out_prefix() + "_frame.ply", frame_pts.data(), frame_n);
                if (map && fetch_map(map, map_raw)) {
                    voxel_merge(map_raw, opt.voxel, map_pts);
                    save_ply(out_prefix() + "_map.ply", map_pts.data(), map_pts.size());
                }
            }
        }
    }

    printf("Stopping...\n");
    insight9_receive_stop();

    // Save what we have: the last frame (camera or world frame, see above)
    // and the global map (world frame).
    if (frame_n > 0) save_ply(out_prefix() + "_frame.ply", frame_pts.data(), frame_n);
    if (map && fetch_map(map, map_raw) && !map_raw.empty()) {
        voxel_merge(map_raw, opt.voxel, map_pts);
        save_ply(out_prefix() + "_map.ply", map_pts.data(), map_pts.size());
    }

    insight9_pointcloud_map_destroy(map);
    if (opt.gui) cv::destroyAllWindows();
    insight9_receive_cleanup();
    printf("Done.\n");
    return 0;
}
