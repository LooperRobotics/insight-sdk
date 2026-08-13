#include "Insight_9_receive.h"
#include "hid.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <signal.h>
#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <vector>

static volatile int keep_running = 1;
static pthread_mutex_t g_sdk_op_lock = PTHREAD_MUTEX_INITIALIZER;

void sigint_handler(int sig) {
    printf("\n[Signal] Received SIGINT, stopping...\n");
    keep_running = 0;
    insight9_receive_all_stop();
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
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
};

struct vio_latest_sample {
    float px;
    float py;
    float pz;
    float qx;
    float qy;
    float qz;
    float qw;
};

static struct cam_stats g_stats[3];
static struct hid_stats g_imu_stats;
static struct hid_stats g_vio_stats;
static struct imu_latest_sample g_imu_latest;
static struct vio_latest_sample g_vio_latest;
static struct timespec g_img_last_print;
static struct timespec g_hid_last_print;
static pthread_mutex_t g_stats_lock = PTHREAD_MUTEX_INITIALIZER;
static int g_img_stats_inited = 0;
static int g_hid_stats_inited = 0;
static int vio_status_raw = -1;

static int gray_fps_state = 0;
static const int GRAY_FPS_VALUES[] = {20, 30, 40, 50, 60};

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
static cv::Mat g_panel[PANEL_COUNT];
static cv::Mat g_aligned_color;   // aligned depth, colorized (RGB-sized, BGR)
static cv::Mat g_aligned_mask;    // valid-pixel mask for the overlay (RGB-sized, 8U)

// Calibration for depth->RGB alignment, read once at startup.
static bool g_calib_ready = false;
static camera_calib g_left_calib;
static camera_calib g_rgb_calib;

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
        clock_gettime(CLOCK_MONOTONIC, &g_img_last_print);
        g_img_stats_inited = 1;
    }
}

static void ensure_hid_stats_initialized_locked(void) {
    if (!g_hid_stats_inited) {
        reset_hid_stats();
        clock_gettime(CLOCK_MONOTONIC, &g_hid_last_print);
        g_hid_stats_inited = 1;
    }
}

static const char *image_format_to_string(unsigned int format) {
    switch (format) {
        case V4L2_PIX_FMT_MJPEG:
            return "MJPEG";
        case V4L2_PIX_FMT_YUYV:
            return "yuyv";
        case V4L2_PIX_FMT_GREY:
            return "GREY";
        case V4L2_PIX_FMT_Z16:
            return "Z16";
        case V4L2_PIX_FMT_Y8I:
            return "Y8I";
        default:
            return "UNKNOWN";
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
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double elapsed = (now.tv_sec - g_img_last_print.tv_sec) +
                     (now.tv_nsec - g_img_last_print.tv_nsec) / 1e9;
    if (elapsed < 1.0) {
        return;
    }

    printf("\n========= Image Callbacks : 1/s =========\n");
    for (int i = 0; i < 3; i++) {
        const struct cam_stats *cs = &g_stats[i];
        printf("IMG[%d]: fps=%.1f ts=%lu size=%zu %dx%d format=0x%x %s\n",
               i,
               cs->cb_count / elapsed,
               (unsigned long)cs->last_ts,
               cs->last_size,
               cs->width,
               cs->height,
               cs->format,
               image_format_to_string(cs->format));
    }
    fflush(stdout);

    reset_img_stats();
    g_img_last_print = now;
}

static void maybe_print_hid_stats_locked(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double elapsed = (now.tv_sec - g_hid_last_print.tv_sec) +
                     (now.tv_nsec - g_hid_last_print.tv_nsec) / 1e9;
    if (elapsed < 0.5) {
        return;
    }
    // printf("\n========= HID Callbacks : 0.5/s =========\n");

    // printf("IMU: hz=%.1f ax=%f ay=%f az=%f gx=%f gy=%f gz=%f ts=%lu\n",
    //        g_imu_stats.cb_count / elapsed,
    //        g_imu_latest.ax, g_imu_latest.ay, g_imu_latest.az,
    //        g_imu_latest.gx, g_imu_latest.gy, g_imu_latest.gz,
    //        (unsigned long)g_imu_stats.last_ts);

    // printf("VIO: hz=%.1f pos=(%f %f %f) ori=(%f %f %f %f) ts=%lu\n",
    //        g_vio_stats.cb_count / elapsed,
    //        g_vio_latest.px, g_vio_latest.py, g_vio_latest.pz,
    //        g_vio_latest.qx, g_vio_latest.qy, g_vio_latest.qz, g_vio_latest.qw,
    //        (unsigned long)g_vio_stats.last_ts);
    fflush(stdout);

    if (insight9_receive_get_vio_status(&vio_status_raw) == 0) {
        VioStatus vio_status = static_cast<VioStatus>(vio_status_raw);
        printf("Current VIO status: %s (%d)\n", vio_status_to_string(vio_status), vio_status_raw);
    } else {
        printf("Failed to get VIO status\n");
    }

    reset_hid_stats();
    g_hid_last_print = now;
}

static void update_hid_stats(struct hid_stats *s, uint64_t ts) {
    s->cb_count++;
    s->last_ts = ts;
}

void my_image_cb(int cam_id, uint8_t *data, size_t size, int w, int h,
                 unsigned int fmt, uint64_t ts, uint64_t ts_right, void *user) {
    if (cam_id < 0 || cam_id >= 3) return;

    pthread_mutex_lock(&g_stats_lock);
    ensure_img_stats_initialized_locked();

    struct cam_stats *s = &g_stats[cam_id];
    s->cb_count++;
    s->last_ts = ts;
    s->last_size = size;
    s->width = w;
    s->height = h;
    s->format = fmt;

    maybe_print_img_stats_locked();
    pthread_mutex_unlock(&g_stats_lock);

    // Cheap hand-off to the per-camera worker: just copy the raw bytes.
    {
        std::lock_guard<std::mutex> lk(g_raw_lock);
        raw_frame &rf = g_raw[cam_id];
        rf.data.assign(data, data + size);
        rf.width = w;
        rf.height = h;
        rf.fmt = fmt;
        rf.has_new = true;
    }
    g_raw_cv.notify_all();
}

void my_imu_cb(float ax, float ay, float az,
               float gx, float gy, float gz,
               uint64_t ts, void *user) {
    pthread_mutex_lock(&g_stats_lock);
    ensure_hid_stats_initialized_locked();
    update_hid_stats(&g_imu_stats, ts);
    g_imu_latest.ax = ax;
    g_imu_latest.ay = ay;
    g_imu_latest.az = az;
    g_imu_latest.gx = gx;
    g_imu_latest.gy = gy;
    g_imu_latest.gz = gz;
    maybe_print_hid_stats_locked();
    pthread_mutex_unlock(&g_stats_lock);
}

void my_vio_cb(float px, float py, float pz,
               float qx, float qy, float qz, float qw,
               uint64_t ts, void *user) {
    pthread_mutex_lock(&g_stats_lock);
    ensure_hid_stats_initialized_locked();
    update_hid_stats(&g_vio_stats, ts);
    g_vio_latest.px = px;
    g_vio_latest.py = py;
    g_vio_latest.pz = pz;
    g_vio_latest.qx = qx;
    g_vio_latest.qy = qy;
    g_vio_latest.qz = qz;
    g_vio_latest.qw = qw;
    maybe_print_hid_stats_locked();
    pthread_mutex_unlock(&g_stats_lock);
}

// Decode one raw frame into BGR panel(s); for depth, also align it to RGB and
// build the colorized overlay. Runs on a per-camera worker thread.
static void process_raw_frame(int cam_id, const raw_frame &rf) {
    const uint8_t *data = rf.data.data();
    size_t size = rf.data.size();
    int w = rf.width, h = rf.height;

    if (rf.fmt == V4L2_PIX_FMT_MJPEG) {
        // cam0 RGB. Reject truncated frames: require SOI (FFD8)...EOI (FFD9).
        bool valid_jpeg = size >= 4 &&
                          data[0] == 0xFF && data[1] == 0xD8 &&
                          data[size - 2] == 0xFF && data[size - 1] == 0xD9;
        if (!valid_jpeg) return;

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
        if (!rgb.empty()) {
            std::lock_guard<std::mutex> lk(g_panel_lock);
            g_panel[PANEL_RGB] = rgb;
        }
    } else if (rf.fmt == V4L2_PIX_FMT_GREY) {
        // cam1 stereo grayscale: two eyes stacked vertically (final row = timestamps).
        if (w > 0 && h > 0 && size >= (size_t)w * h) {
            int eye_h = (h - 1) / 2;
            cv::Mat full(h, w, CV_8UC1, (void *)data);
            cv::Mat left_bgr, right_bgr;
            cv::cvtColor(full.rowRange(0, eye_h), left_bgr, cv::COLOR_GRAY2BGR);
            cv::cvtColor(full.rowRange(eye_h, 2 * eye_h), right_bgr, cv::COLOR_GRAY2BGR);
            std::lock_guard<std::mutex> lk(g_panel_lock);
            g_panel[PANEL_LEFT] = left_bgr;
            g_panel[PANEL_RIGHT] = right_bgr;
        }
    } else if (rf.fmt == V4L2_PIX_FMT_Z16) {
        // cam2 depth: 16-bit; last 2 rows are metadata.
        if (w <= 0 || h <= 0 || size < (size_t)w * h * 2) return;
        int valid_h = h - 2;

        // Raw depth colormap for the top-right panel.
        cv::Mat depth(valid_h, w, CV_16UC1, (void *)data);
        cv::Mat depth8, depth_bgr;
        cv::normalize(depth, depth8, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::applyColorMap(depth8, depth_bgr, cv::COLORMAP_JET);

        // Align the depth onto the RGB image plane and colorize it for overlay.
        cv::Mat acolor, amask;
        if (g_calib_ready) {
            int rgb_w = (int)g_rgb_calib.intrinsics.width;
            int rgb_h = (int)g_rgb_calib.intrinsics.height;
            if (rgb_w > 0 && rgb_h > 0) {
                std::vector<uint16_t> aligned((size_t)rgb_w * rgb_h);
                if (insight9_receive_align_depth_to_rgb(
                        (const uint16_t *)data, w, valid_h,
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
    } else if (rf.fmt == V4L2_PIX_FMT_YUYV) {
        if (w <= 0 || h <= 0 || size < (size_t)w * h * 2) return;
        
        cv::Mat bgr;
        cv::Mat yuyv_mat(h, w, CV_8UC2, (void*)data);
        cv::cvtColor(yuyv_mat, bgr, cv::COLOR_YUV2BGR_YUYV);
        
        std::lock_guard<std::mutex> lk(g_panel_lock);
        g_panel[PANEL_RGB] = bgr;
    } else if (rf.fmt == V4L2_PIX_FMT_NV12) {
        if (w <= 0 || h <= 0 || size < (size_t)w * h * 3 / 2) return;
        
        cv::Mat bgr;
        cv::Mat nv12_mat(h * 3 / 2, w, CV_8UC1, (void*)data);
        cv::cvtColor(nv12_mat, bgr, cv::COLOR_YUV2BGR_NV12);
        
        std::lock_guard<std::mutex> lk(g_panel_lock);
        g_panel[PANEL_RGB] = bgr;
    }
}

// One dedicated worker thread per camera: waits for its latest raw frame and
// decodes independently, so a slow stream never delays the others.
static void camera_worker(int cam_id) {
    while (keep_running) {
        raw_frame local;
        {
            std::unique_lock<std::mutex> lk(g_raw_lock);
            g_raw_cv.wait(lk, [cam_id] {
                return !keep_running || g_raw[cam_id].has_new;
            });
            if (!keep_running) break;
            std::swap(local, g_raw[cam_id]);
            g_raw[cam_id].has_new = false;
        }
        process_raw_frame(cam_id, local);
    }
}

void* reconnect_worker(void* arg) {
    while (keep_running) {
        sleep(10);
        if (!keep_running) break;
        
        pthread_mutex_lock(&g_sdk_op_lock);
        if (insight9_receive_is_camera_running(0) == 0 &&
            insight9_receive_is_camera_running(1) == 0 &&
            insight9_receive_is_camera_running(2) == 0) {
            printf("[ReconnectWorker] All cameras stopped, attempting reconnect...\n");
            int fps = 0;
            if (insight9_receive_get_current_fps(&fps) == 0) {
                printf("Current FPS for gray camera: %d\n", fps);
            } else {
                printf("Failed to get current FPS for gray camera\n");
            }
            insight9_receive_all_stop();
            insight9_receive_cleanup();
           
            insight9_config_t config_reinit;
            config_reinit.rgb_config.width = 1088;
            config_reinit.rgb_config.height = 1920;
            config_reinit.rgb_config.fps = 30;
            config_reinit.rgb_config.pixel_format = V4L2_PIX_FMT_MJPEG;
            config_reinit.gray_config.width = 544;
            config_reinit.gray_config.height = 1281;
            config_reinit.gray_config.fps = fps;
            config_reinit.gray_config.pixel_format = V4L2_PIX_FMT_GREY;
            config_reinit.depth_config.width = 544;
            config_reinit.depth_config.height = 642;
            config_reinit.depth_config.fps = 30;
            config_reinit.depth_config.pixel_format = V4L2_PIX_FMT_Z16;
            
            if (insight9_receive_init(&config_reinit) != 0) {
                printf("[ReconnectWorker] SDK init failed\n");
            } else {
                insight9_receive_register_image_callback(my_image_cb, NULL);
                insight9_receive_register_imu_callback(my_imu_cb, NULL);
                insight9_receive_register_vio_callback(my_vio_cb, NULL);
                
                if (insight9_receive_start() != 0) {
                    printf("[ReconnectWorker] SDK start failed\n");
                    insight9_receive_cleanup();
                } else {
                    printf("[ReconnectWorker] SDK restarted successfully\n");
                }
            }
        }
        pthread_mutex_unlock(&g_sdk_op_lock);
    }
    insight9_receive_all_stop();
    insight9_receive_cleanup();
    return NULL;
}

void toggle_gray_camera_fps() {
    int new_fps = GRAY_FPS_VALUES[gray_fps_state];
    printf("new_fps = %d | gray_fps_state = %d\n", new_fps, gray_fps_state);
    gray_fps_state = (gray_fps_state + 1) % 5;

    printf("\n========= Switching Gray Camera FPS =========\n");
    printf("New FPS: %d\n", new_fps);

    pthread_mutex_lock(&g_sdk_op_lock);
    if (insight9_receive_switch_camera_fps(1, new_fps) == 0) {
        printf("Gray camera switched to %d FPS\n", new_fps);
    } else {
        printf("Failed to switch gray camera FPS!\n");
    }
    pthread_mutex_unlock(&g_sdk_op_lock);
}

void* fps_switch_worker(void* arg) {
    while (keep_running) {
        sleep(15);
        if (!keep_running) break;
        toggle_gray_camera_fps();
    }
    return NULL;
}

int main() {
    signal(SIGINT, sigint_handler);

    int active_cam = -1;
    camera_params params = {0};

    if (insight9_receive_init_default() != 0) {
        fprintf(stderr, "SDK init failed\n");
        return -1;
    }

    insight9_receive_register_image_callback(my_image_cb, NULL);
    insight9_receive_register_imu_callback(my_imu_cb, NULL);
    insight9_receive_register_vio_callback(my_vio_cb, NULL);

    if (insight9_receive_start() != 0) {
        fprintf(stderr, "SDK start failed\n");
        insight9_receive_cleanup();
        return -1;
    }

    printf("\n================ Extension Unit Control Test ================\n");
    printf("\n---------------- camera params control ----------------\n");

    // Example: Get current active camera and its parameters
    if (insight9_receive_get_active_camera(&active_cam) == 0) {
        printf("Current active camera: %d\n", active_cam);
        if(insight9_receive_get_camera_params(&params) == 0) {
            printf("Camera parameters retrieved successfully\n");
            insight9_receive_print_camera_params(&params);
        } else {
            printf("Failed to get camera parameters\n");
        }
    } else {
        printf("Failed to get active camera\n");
    }

    if(insight9_receive_get_camera_params_for(1, &params) == 0) {
        printf("Camera parameters for cam %d retrieved successfully\n", active_cam);
        insight9_receive_print_camera_params(&params);
    } else {
        printf("Failed to get camera parameters for cam %d\n", active_cam);
    }

    if (insight9_receive_get_active_camera(&active_cam) == 0) {
        printf("Current active camera: %d\n", active_cam);
    } else {
        printf("Failed to get active camera\n");
    }

    // Example: Adjust RGB camera parameters
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
        if(insight9_receive_get_camera_params_for(0, &params) == 0) {
            printf("Camera parameters for cam %d after setting:\n", 0);
            insight9_receive_print_camera_params(&params);
        } else {
            printf("Failed to get camera parameters for cam %d after setting\n", 0);
        }
    } else {
        printf("Failed to set camera parameters (invalid range or XU not available)\n");
    }
    
    printf("\n---------------- get looper info ----------------\n");
    
    int fps = 0;
    if (insight9_receive_get_current_fps(&fps) == 0) {
        printf("Current FPS for gray camera: %d\n", fps);
    } else {
        printf("Failed to get current FPS for gray camera\n");
    }

    // Example: Read intrinsics/extrinsics of the three cameras
    {
        static const int calib_cams[] = {
            INSIGHT9_CALIB_CAM_LEFT, INSIGHT9_CALIB_CAM_RIGHT, INSIGHT9_CALIB_CAM_RGB
        };
        static const char *calib_names[] = {"left", "right", "rgb"};
        camera_calib calib;
        for (int i = 0; i < 3; i++) {
            if (insight9_receive_get_camera_calib(calib_cams[i], &calib) == 0) {
                printf("Calibration for %s camera:\n", calib_names[i]);
                insight9_receive_print_camera_calib(&calib);
            } else {
                printf("Failed to get calibration for %s camera\n", calib_names[i]);
            }
        }

        // Cache the LEFT + RGB calibration used to align depth onto RGB.
        if (insight9_receive_get_camera_calib(INSIGHT9_CALIB_CAM_LEFT, &g_left_calib) == 0 &&
            insight9_receive_get_camera_calib(INSIGHT9_CALIB_CAM_RGB, &g_rgb_calib) == 0) {
            g_calib_ready = true;
            printf("Depth->RGB alignment enabled (RGB %ux%u).\n",
                   g_rgb_calib.intrinsics.width, g_rgb_calib.intrinsics.height);
        } else {
            printf("Calibration unavailable; depth->RGB overlay disabled.\n");
        }
    }

    printf("Current connected hardware type: %s\n", insight9_receive_get_hardware_type());
    
    pthread_t reconnect_thread;
    pthread_create(&reconnect_thread, NULL, reconnect_worker, NULL);
    pthread_t fps_switch_thread;
    pthread_create(&fps_switch_thread, NULL, fps_switch_worker, NULL);

    printf("\n=================================================\n\n");
    printf("SDK running with all 3 cameras\n");
    printf("A fixed grid window will appear: L / R / raw-depth / RGB+aligned-depth.\n");
    printf("Press 'q' or ESC in the window, or Ctrl+C, to stop...\n");

    // Fixed composite canvas layout.
    const cv::Rect roi_left (0,      0,      CELL_W, CELL_H);  // top-left
    const cv::Rect roi_right(0,      CELL_H, CELL_W, CELL_H);  // bottom-left
    const cv::Rect roi_depth(CELL_W, 0,      CELL_W, CELL_H);  // top-right
    const cv::Rect roi_rgb  (CELL_W, CELL_H, RGB_W,  RGB_H);   // bottom-right
    const int canvas_w = CELL_W + std::max(CELL_W, RGB_W);
    const int canvas_h = std::max(CELL_H + CELL_H, CELL_H + RGB_H);

    const char *win_name = "Insight9 - L/R / Depth / RGB+AlignedDepth";
    cv::namedWindow(win_name, cv::WINDOW_AUTOSIZE);

    auto blit = [](cv::Mat &canvas, const cv::Mat &panel, const cv::Rect &roi) {
        if (panel.empty()) return;
        cv::Mat dst = canvas(roi);
        cv::resize(panel, dst, dst.size());
    };

    // One worker thread per stream (independent decoding + alignment).
    std::thread workers[3];
    for (int cam = 0; cam < 3; cam++) {
        workers[cam] = std::thread(camera_worker, cam);
    }

    while (keep_running) {
        // Grab the latest panels/overlay under the lock (cheap shallow copies),
        // then do the heavy compositing unlocked so workers are never blocked.
        cv::Mat left, right, depth, rgb, acolor, amask;
        {
            std::lock_guard<std::mutex> lk(g_panel_lock);
            left   = g_panel[PANEL_LEFT];
            right  = g_panel[PANEL_RIGHT];
            depth  = g_panel[PANEL_DEPTH];
            rgb    = g_panel[PANEL_RGB];
            acolor = g_aligned_color;
            amask  = g_aligned_mask;
        }

        // Bottom-right = RGB with the aligned depth semi-transparently overlaid.
        cv::Mat rgb_view;
        if (!rgb.empty()) {
            rgb_view = rgb.clone();
            if (!acolor.empty() && acolor.size() == rgb.size() &&
                !amask.empty() && amask.size() == rgb.size()) {
                cv::Mat blend;
                cv::addWeighted(rgb, 0.5, acolor, 0.5, 0, blend);
                blend.copyTo(rgb_view, amask);
            }
        }

        cv::Mat canvas = cv::Mat::zeros(canvas_h, canvas_w, CV_8UC3);
        blit(canvas, left,     roi_left);
        blit(canvas, right,    roi_right);
        blit(canvas, depth,    roi_depth);
        blit(canvas, rgb_view, roi_rgb);

        cv::Mat shown;
        cv::resize(canvas, shown, cv::Size(), DISPLAY_SCALE, DISPLAY_SCALE);
        cv::imshow(win_name, shown);

        int key = cv::waitKey(30);
        if (key == 'q' || key == 27) {
            keep_running = 0;
        }
    }

    // Wake and join the worker threads.
    g_raw_cv.notify_all();
    for (int cam = 0; cam < 3; cam++) {
        if (workers[cam].joinable()) workers[cam].join();
    }
    cv::destroyAllWindows();

    pthread_join(reconnect_thread, NULL);
    pthread_join(fps_switch_thread, NULL);

    insight9_receive_all_stop();
    insight9_receive_cleanup();
    printf("Program exited.\n");
    return 0;
}