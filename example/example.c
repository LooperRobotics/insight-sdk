#include "Insight_9_receive.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <signal.h>
#include <linux/videodev2.h>

static volatile int keep_running = 1;

void sigint_handler(int sig) {
    keep_running = 0;
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
            return "RGB";
        case V4L2_PIX_FMT_GREY:
            return "GREY";
        case V4L2_PIX_FMT_Z16:
            return "Z16";
        default:
            return "UNKNOWN";
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
        printf("IMG[%d]: fps=%.1f ts=%lu size=%zu %dx%d format=%s\n",
               i,
               cs->cb_count / elapsed,
               (unsigned long)cs->last_ts,
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
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double elapsed = (now.tv_sec - g_hid_last_print.tv_sec) +
                     (now.tv_nsec - g_hid_last_print.tv_nsec) / 1e9;
    if (elapsed < 0.5) {
        return;
    }
    printf("\n========= HID Callbacks : 0.5/s =========\n");

    printf("IMU: hz=%.1f ax=%f ay=%f az=%f gx=%f gy=%f gz=%f ts=%lu\n",
           g_imu_stats.cb_count / elapsed,
           g_imu_latest.ax, g_imu_latest.ay, g_imu_latest.az,
           g_imu_latest.gx, g_imu_latest.gy, g_imu_latest.gz,
           (unsigned long)g_imu_stats.last_ts);

    printf("VIO: hz=%.1f pos=(%f %f %f) ori=(%f %f %f %f) ts=%lu\n",
           g_vio_stats.cb_count / elapsed,
           g_vio_latest.px, g_vio_latest.py, g_vio_latest.pz,
           g_vio_latest.qx, g_vio_latest.qy, g_vio_latest.qz, g_vio_latest.qw,
           (unsigned long)g_vio_stats.last_ts);
    fflush(stdout);

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

int main() {
    signal(SIGINT, sigint_handler);

    int active_cam = -1;
    camera_params params = {0};

    if (insight9_receive_init() != 0) {
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

    printf("Current connected hardware type: %s\n", insight9_receive_get_hardware_type().c_str());

    printf("SDK running, press Ctrl+C to stop...\n");
    while (keep_running) {
        sleep(1);
    }

    insight9_receive_stop();
    insight9_receive_cleanup();
    return 0;
}
