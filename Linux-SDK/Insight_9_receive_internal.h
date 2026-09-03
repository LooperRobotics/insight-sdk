#pragma once
#include "Insight_9_receive.h"
#include "UvcExtensionUnit.hpp"
#include <atomic>
#include <pthread.h>

typedef struct {
    // Cameras
    struct cam_ctx cams[CAM_NUM];
    char metadata_devs[CAM_NUM][MAX_PATH];
    char metadata_usb_paths[CAM_NUM][MAX_PATH];
    char video_devs[CAM_NUM][MAX_PATH];   // Dynamically resolved video device paths
    char video_usb_paths[CAM_NUM][MAX_PATH]; // Matching USB root paths, used for rediscovery after reconnect
    camera_params cachedInitialParams[CAM_NUM];
    bool hasCachedInitialParams[CAM_NUM] = {false, false, false};
    // HID devices (only two are used: 0=IMU, 1=VIO)
    char hid_devs[HID_NUM][MAX_PATH];           // Dynamically resolved hidraw device paths
    char hid_usb_paths[HID_NUM][MAX_PATH];   // Matching HID USB root paths, used for rediscovery after reconnect
    pthread_t hid_tids[HID_NUM];
    // Callbacks
    image_callback img_cb;
    void *img_userdata;
    imu_callback imu_cb;
    void *imu_userdata;
    vio_callback vio_cb;
    void *vio_userdata;
    // Running flag
    std::atomic<bool> running;
    insight9_config_t config;
    std::atomic<bool> cam_running[CAM_NUM];
    pthread_t video_tids[CAM_NUM];
    struct timespec last_frame_time[CAM_NUM];
    bool first_frame_received[CAM_NUM];
    // Initialization flag
    int initialized;
    viewer::UvcExtensionUnit *xu_control;
    char xu_dev_path[MAX_PATH];
    pthread_mutex_t xu_mutex;
    std::atomic<bool> xu_ready;
} sdk_ctx_t;

extern sdk_ctx_t g_ctx;