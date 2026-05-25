#include "Insight_9_receive.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <signal.h>

static volatile int keep_running = 1;

void sigint_handler(int sig) {
    keep_running = 0;
}

struct cam_stats {
    uint64_t cb_count;
    uint64_t unique_ts_count;
    uint64_t last_ts;
    uint64_t ts_delta_sum_us;
    size_t   min_size;
    size_t   max_size;
    int      zero_size_count;
};

struct hid_stats {
    uint64_t cb_count;
    uint64_t unique_ts_count;
    uint64_t last_ts;
    uint64_t ts_delta_sum_us;
};

static struct cam_stats g_stats[3];
static struct hid_stats g_imu_stats;
static struct hid_stats g_vio_stats;
static struct timespec g_last_print;
static pthread_mutex_t g_stats_lock = PTHREAD_MUTEX_INITIALIZER;
static int g_stats_inited = 0;

static void reset_stats(void) {
    for (int i = 0; i < 3; i++) {
        g_stats[i].cb_count = 0;
        g_stats[i].unique_ts_count = 0;
        g_stats[i].ts_delta_sum_us = 0;
        g_stats[i].min_size = (size_t)-1;
        g_stats[i].max_size = 0;
        g_stats[i].zero_size_count = 0;
    }
    g_imu_stats.cb_count = 0;
    g_imu_stats.unique_ts_count = 0;
    g_imu_stats.ts_delta_sum_us = 0;
    g_vio_stats.cb_count = 0;
    g_vio_stats.unique_ts_count = 0;
    g_vio_stats.ts_delta_sum_us = 0;
}

static void update_hid_stats(struct hid_stats *s, uint64_t ts) {
    s->cb_count++;
    if (ts != s->last_ts) {
        if (s->last_ts != 0 && ts > s->last_ts) {
            s->ts_delta_sum_us += (ts - s->last_ts);
        }
        s->unique_ts_count++;
        s->last_ts = ts;
    }
}

void my_image_cb(int cam_id, uint8_t *data, size_t size, int w, int h,
                 unsigned int fmt, uint64_t ts, uint64_t ts_right, void *user) {
    if (cam_id < 0 || cam_id >= 3) return;

    pthread_mutex_lock(&g_stats_lock);
    if (!g_stats_inited) {
        reset_stats();
        clock_gettime(CLOCK_MONOTONIC, &g_last_print);
        g_stats_inited = 1;
    }

    struct cam_stats *s = &g_stats[cam_id];
    s->cb_count++;
    if (ts != s->last_ts) {
        if (s->last_ts != 0 && ts > s->last_ts) {
            s->ts_delta_sum_us += (ts - s->last_ts);
        }
        s->unique_ts_count++;
        s->last_ts = ts;
    }
    if (size == 0) s->zero_size_count++;
    if (size < s->min_size) s->min_size = size;
    if (size > s->max_size) s->max_size = size;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double elapsed = (now.tv_sec - g_last_print.tv_sec) +
                     (now.tv_nsec - g_last_print.tv_nsec) / 1e9;
    if (elapsed >= 1.0) {
        printf("---- stats (window=%.2fs) ----\n", elapsed);
        const char *names[3] = {"RGB ", "GREY", "Z16 "};
        for (int i = 0; i < 3; i++) {
            struct cam_stats *cs = &g_stats[i];
            double cb_fps  = cs->cb_count / elapsed;
            double uts_fps = cs->unique_ts_count / elapsed;
            double avg_ts_dt_ms = (cs->unique_ts_count > 1)
                ? (cs->ts_delta_sum_us / (double)(cs->unique_ts_count - 1)) / 1000.0
                : 0.0;
            size_t min_sz = (cs->min_size == (size_t)-1) ? 0 : cs->min_size;
            printf("  cam%d %s: cb=%6.1ffps  uniqueTS=%5.1ffps  avgTSdt=%6.2fms  size[min=%zu max=%zu zero=%d]\n",
                   i, names[i], cb_fps, uts_fps, avg_ts_dt_ms,
                   min_sz, cs->max_size, cs->zero_size_count);
        }
        struct hid_stats *hs[2] = {&g_imu_stats, &g_vio_stats};
        const char *hnames[2] = {"IMU ", "VIO "};
        for (int i = 0; i < 2; i++) {
            double cb_fps  = hs[i]->cb_count / elapsed;
            double uts_fps = hs[i]->unique_ts_count / elapsed;
            double avg_ts_dt_ms = (hs[i]->unique_ts_count > 1)
                ? (hs[i]->ts_delta_sum_us / (double)(hs[i]->unique_ts_count - 1)) / 1000.0
                : 0.0;
            printf("  %s     : cb=%6.1ffps  uniqueTS=%5.1ffps  avgTSdt=%6.2fms\n",
                   hnames[i], cb_fps, uts_fps, avg_ts_dt_ms);
        }
        fflush(stdout);
        reset_stats();
        g_last_print = now;
    }
    pthread_mutex_unlock(&g_stats_lock);
}

void my_imu_cb(float ax, float ay, float az,
               float gx, float gy, float gz,
               uint64_t ts, void *user) {
    pthread_mutex_lock(&g_stats_lock);
    update_hid_stats(&g_imu_stats, ts);
    pthread_mutex_unlock(&g_stats_lock);
}

void my_vio_cb(float px, float py, float pz,
               float qx, float qy, float qz, float qw,
               uint64_t ts, void *user) {
    pthread_mutex_lock(&g_stats_lock);
    update_hid_stats(&g_vio_stats, ts);
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
    params.rotation = 1;

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

    printf("SDK running, press Ctrl+C to stop...\n");
    while (keep_running) {
        sleep(1);
    }

    insight9_receive_stop();
    insight9_receive_cleanup();
    return 0;
}