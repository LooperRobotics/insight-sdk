// example_win.c - Windows 平台示例程序，使用 Insight9SDK.lib
#include <windows.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <signal.h>

#include "Insight9SDK.h"

// 全局标志，用于响应 Ctrl+C
static volatile BOOL keep_running = TRUE;
static volatile LONGLONG last_image_time = 0;

BOOL WINAPI console_handler(DWORD dwCtrlType) {
    if (dwCtrlType == CTRL_C_EVENT || dwCtrlType == CTRL_BREAK_EVENT) {
        keep_running = FALSE;
        return TRUE;
    }
    return FALSE;
}

// 统计结构体
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
static LARGE_INTEGER g_img_last_print;
static LARGE_INTEGER g_hid_last_print;
static CRITICAL_SECTION g_stats_lock;
static int g_img_stats_inited = 0;
static int g_hid_stats_inited = 0;

static double get_elapsed_seconds(LARGE_INTEGER start, LARGE_INTEGER end) {
    LARGE_INTEGER freq;
    QueryPerformanceFrequency(&freq);
    return (double)(end.QuadPart - start.QuadPart) / (double)freq.QuadPart;
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
        QueryPerformanceCounter(&g_img_last_print);
        g_img_stats_inited = 1;
    }
}

static void ensure_hid_stats_initialized_locked(void) {
    if (!g_hid_stats_inited) {
        reset_hid_stats();
        QueryPerformanceCounter(&g_hid_last_print);
        g_hid_stats_inited = 1;
    }
}

static const char* image_format_to_string(unsigned int format) {
    switch (format) {
        case 0x47504A4D: return "MJPEG";  // 'MJPEG'
        case 0x59455247: return "GREY";   // 'GREY'
        case 0x36315A:   return "Z16";    // 'Z16 '
        default: return "UNKNOWN";
    }
}

static void maybe_print_img_stats_locked(void) {
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
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
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
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

    reset_hid_stats();
    g_hid_last_print = now;
}

static void update_hid_stats(struct hid_stats* s, uint64_t ts) {
    s->cb_count++;
    s->last_ts = ts;
}

void my_image_cb(int cam_id, uint8_t* data, size_t size,
                 int width, int height, unsigned int format,
                 uint64_t timestamp, uint64_t right_timestamp,
                 void* userdata) {
    if (cam_id < 0 || cam_id >= 3) return;
    InterlockedExchange64(&last_image_time, GetTickCount64());

    EnterCriticalSection(&g_stats_lock);
    ensure_img_stats_initialized_locked();
    struct cam_stats* s = &g_stats[cam_id];
    s->cb_count++;
    s->last_ts = timestamp;
    s->last_size = size;
    s->width = width;
    s->height = height;
    s->format = format;
    maybe_print_img_stats_locked();
    LeaveCriticalSection(&g_stats_lock);
}

void my_imu_cb(float ax, float ay, float az,
               float gx, float gy, float gz,
               uint64_t timestamp, void* userdata) {
    EnterCriticalSection(&g_stats_lock);
    ensure_hid_stats_initialized_locked();
    update_hid_stats(&g_imu_stats, timestamp);
    g_imu_latest.ax = ax; g_imu_latest.ay = ay; g_imu_latest.az = az;
    g_imu_latest.gx = gx; g_imu_latest.gy = gy; g_imu_latest.gz = gz;
    maybe_print_hid_stats_locked();
    LeaveCriticalSection(&g_stats_lock);
}

void my_vio_cb(float px, float py, float pz,
               float qx, float qy, float qz, float qw,
               uint64_t timestamp, void* userdata) {
    EnterCriticalSection(&g_stats_lock);
    ensure_hid_stats_initialized_locked();
    update_hid_stats(&g_vio_stats, timestamp);
    g_vio_latest.px = px; g_vio_latest.py = py; g_vio_latest.pz = pz;
    g_vio_latest.qx = qx; g_vio_latest.qy = qy; g_vio_latest.qz = qz; g_vio_latest.qw = qw;
    maybe_print_hid_stats_locked();
    LeaveCriticalSection(&g_stats_lock);
}

int main() {
    // 设置控制台 Ctrl+C 处理
    SetConsoleCtrlHandler(console_handler, TRUE);
    InitializeCriticalSection(&g_stats_lock);

    int active_cam = -1;
    camera_params params = {0};

    if (insight9_receive_init() != 0) {
        fprintf(stderr, "SDK init failed\n");
        DeleteCriticalSection(&g_stats_lock);
        return -1;
    }

    insight9_receive_register_image_callback(my_image_cb, NULL);
    insight9_receive_register_imu_callback(my_imu_cb, NULL);
    insight9_receive_register_vio_callback(my_vio_cb, NULL);

    if (insight9_receive_start() != 0) {
        fprintf(stderr, "SDK start failed\n");
        insight9_receive_cleanup();
        DeleteCriticalSection(&g_stats_lock);
        return -1;
    }

    // 获取当前活跃摄像头及其参数
    if (insight9_receive_get_active_camera(&active_cam) == 0) {
        printf("Current active camera: %d\n", active_cam);
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

    // 调整 RGB 摄像头参数
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
            printf("Camera parameters for cam %d after setting:\n", 0);
            insight9_receive_print_camera_params(&params);
        } else {
            printf("Failed to get camera parameters for cam %d after setting\n", 0);
        }
    } else {
        printf("Failed to set camera parameters (invalid range or XU not available)\n");
    }

    printf("Current connected hardware type: %s\n", insight9_receive_get_hardware_type());

    InterlockedExchange64(&last_image_time, GetTickCount64());
    printf("SDK running, press Ctrl+C to stop...\n");
    while (keep_running) {
        Sleep(3000);   // 每秒检查一次
        LONGLONG now = GetTickCount64();
        LONGLONG last = last_image_time;   // 直接读取（对齐，x64 下原子）
        if (last != 0 && (now - last) > 5000) {
            insight9_receive_stop();
            insight9_receive_cleanup();
                
            if (insight9_receive_init() != 0) {
                printf("[ReconnectWorker] SDK init failed\n");
            } else {
                // 正确：传递两个参数
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
    }

    insight9_receive_stop();
    insight9_receive_cleanup();
    DeleteCriticalSection(&g_stats_lock);
    return 0;
}