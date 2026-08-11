#ifndef INSIGHT9_HID_H
#define INSIGHT9_HID_H

#include "Insight_9_receive.h"
#include "UvcExtensionUnit.hpp"
#include <atomic>
#include <pthread.h>
#include <sys/time.h>

#define CAM_NUM 3
#define HID_NUM 2
#define MAX_PATH 1024
#define VENDOR_ID  0x8086
#define PRODUCT_ID 0x0b5c

struct buffer {
    void *start;
    size_t length;
};

struct uvc_frame_info {
    unsigned int width;
    unsigned int height;
    unsigned int intervals[8];
};

struct uvc_format_info {
    unsigned int fcc;
    int frames_num;
    struct uvc_frame_info *frames;
};

struct cam_ctx {
    int fd;
    struct buffer *buffers;
    int meta_fd;
    struct buffer *meta_buffers;
    int meta_buffer_count;
    int buffer_count;
    pthread_t tid;
    int cam_id;
    int width;
    int height;
    int fps;
    unsigned int format;
    int format_num;
    struct uvc_format_info *formats_info;
    uint64_t last_timestamp;
    pthread_mutex_t fd_lock = PTHREAD_MUTEX_INITIALIZER;
};

typedef struct {
    struct cam_ctx cams[CAM_NUM];
    char video_devs[CAM_NUM][MAX_PATH];
    char metadata_devs[CAM_NUM][MAX_PATH];
    char video_usb_paths[CAM_NUM][MAX_PATH];
    char metadata_usb_paths[CAM_NUM][MAX_PATH];
    char hid_devs[HID_NUM][MAX_PATH];
    char hid_usb_paths[HID_NUM][MAX_PATH];
    pthread_t hid_tids[HID_NUM];
    image_callback img_cb;
    void *img_userdata;
    imu_callback imu_cb;
    void *imu_userdata;
    vio_callback vio_cb;
    void *vio_userdata;
    std::atomic<bool> running;
    insight9_config_t config;
    std::atomic<bool> cam_running[CAM_NUM];
    pthread_t video_tids[CAM_NUM];
    struct timespec last_frame_time[CAM_NUM];
    bool first_frame_received[CAM_NUM];
    int initialized;
    viewer::UvcExtensionUnit *xu_control;
    std::atomic<bool> xu_ready;
} sdk_ctx_t;

extern sdk_ctx_t g_ctx;

int get_hid_usb_device_path(const char *hid_dev, char *usb_path, size_t usb_path_size);
int find_hid_devices_by_vid_pid(unsigned int target_vid, unsigned int target_pid,
                               char dev_paths[][MAX_PATH], int max_devs);
void refresh_hid_device_path(int idx);
void *hid_thread(void *arg);
int hid_init(void);

#endif // INSIGHT9_HID_H
