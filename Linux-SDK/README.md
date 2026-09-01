# Insight 9 SDK User Guide (Linux)

## 1. Overview

The Insight 9 Linux SDK provides a C/C++ interface for Looper Robotics Insight 9 devices. It captures three UVC image streams and two HID streams, exposes camera parameter control through the UVC extension unit, and delivers image/IMU/VIO data to the application through callbacks.

Current device matching uses:

| Field | Value |
|---|---|
| VID | `0x3652` |
| PID | `0x0b5c` |

## 2. Stream Model

The SDK manages three logical cameras:

| cam_id | Stream | Default format | Notes |
|---|---|---|---|
| 0 | Depth | `Z16` | Depth stream; final metadata rows should be excluded when processing 
| 1 | Gray  | `GREY` or `Y8I` | Stereo grayscale stream |
| 2 | RGB   | `MJPEG`, `YUYV` | Main RGB camera |depth |

The SDK also manages two HID streams:

| Stream | Callback |
|---|---|
| IMU | `imu_callback` |
| VIO pose | `vio_callback` |

Image callback timestamps are provided in microseconds. Metadata device timestamps can also be read explicitly through `insight9_receive_read_metadata_timestamp()`.

## 3. Features

- Automatic UVC and HID discovery by VID/PID.
- Default initialization or custom per-camera stream configuration.
- Callback delivery for RGB/gray/depth images, IMU samples, and VIO poses.
- Per-camera start, stop, restart, FPS switching, and format switching.
- Device capability enumeration for supported resolutions, frame rates, and formats.
- UVC extension unit camera parameter control.
- Factory calibration readout and depth-to-RGB alignment helper.
- Hot-plug recovery for video/HID streams.
- VIO status and hardware type query helpers.

## 4. Dependencies

- Linux with V4L2, HIDRAW, sysfs, and UVC extension unit support.
- CMake 3.10 or newer.
- GCC/G++ with C++17 support.
- pthreads.
- OpenCV development package, required by the included `example`.
- Runtime access to `/sys/class/video4linux`, `/sys/class/hidraw`, `/dev/video*`, and `/dev/hidraw*`.

Install common dependencies on Ubuntu/Debian:

```bash
sudo apt update
sudo apt install build-essential cmake libopencv-dev
```

If you do not run as root, make sure the user has access to the video and HID devices. Depending on your distribution, this usually means adding udev rules or adding the user to groups such as `video` and `plugdev`.

## 5. Build

Build the SDK and example:

```bash
cd Linux-SDK
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Run the example:

```bash
sudo ./build/example
```

Install the shared library and headers:

```bash
sudo cmake --install build
sudo ldconfig
```

The build produces:

- `build/libinsight9.so`
- `build/example`

## 6. API Reference

Include the SDK header:

```cpp
#include "Insight_9_receive.h"
```

### 6.1 Pixel Formats and Configuration

```cpp
enum class PixelFormat {
    Unknown,
    MJPEG,
    GREY,
    Z16,
    RGB8,
    Y8I,
    YUYV,
    YVYU,
    NV12
};

typedef struct {
    int width;
    int height;
    int fps;
    PixelFormat pixel_format;
} video_config_t;

typedef struct {
    video_config_t rgb_config;
    video_config_t gray_config;
    video_config_t depth_config;
} insight9_config_t;
```

Use `insight9_receive_init_default()` for the SDK defaults, or pass an `insight9_config_t` to `insight9_receive_init()`.

### 6.2 Camera Parameters

```cpp
typedef struct {
    uint8_t cam_id;
    uint8_t resolution;
    uint8_t frame_rate;
    float exposure_time;
    float exposure_gain;
    uint8_t auto_exposure;
    float brightness;
    float contrast;
    float gamma_dark;
    float hue;
    float saturation;
    uint8_t sharpness;
    uint8_t auto_white_balance;
    float white_balance;
    uint8_t decimation;
    uint8_t hardware_model;
} camera_params;
```

Parameter ranges:

| Parameter | Min | Max | Notes |
|---|---:|---:|---|
| `exposure_time` | 0.0 | 0.03 | Seconds |
| `exposure_gain` | 1.0 | 16.0 | Gain |
| `auto_exposure` | 0 | 1 | Boolean |
| `brightness` | 0.0 | 127.0 | Brightness |
| `contrast` | 0.0 | 1.9 | Contrast |
| `gamma_dark` | 1.0 | 4.0 | Gamma |
| `hue` | 0.0 | 87.0 | Hue |
| `saturation` | 0.0 | 1.999 | Saturation |
| `sharpness` | 1 | 255 | Sharpness |
| `auto_white_balance` | 0 | 1 | Boolean |
| `white_balance` | 1.0 | 3.0 | White balance |
| `decimation` | 1 | 255 | Decimation |
| `hardware_model` | 0 | 3 | Hardware selector |

Set functions validate values before sending them to the device.

### 6.3 Calibration

```cpp
typedef struct {
    camera_intrinsics intrinsics;
    camera_extrinsics extrinsics;
} camera_calib;

enum {
    INSIGHT9_CALIB_CAM_LEFT  = 0,
    INSIGHT9_CALIB_CAM_RIGHT = 1,
    INSIGHT9_CALIB_CAM_RGB   = 2,
};
```

Calibration data contains ROS-style camera intrinsics and one extrinsic transform. Use it with `insight9_receive_align_depth_to_rgb()` to produce a depth map registered to the RGB image plane.

### 6.4 Callbacks

```cpp
typedef void (*image_callback)(int cam_id, uint8_t *data, size_t size,
                               int width, int height, unsigned int format,
                               uint64_t timestamp, void *userdata);

typedef void (*imu_callback)(float ax, float ay, float az,
                             float gx, float gy, float gz,
                             uint64_t timestamp, void *userdata);

typedef void (*vio_callback)(float px, float py, float pz,
                             float qx, float qy, float qz, float qw,
                             uint64_t timestamp, void *userdata);
```

The image data pointer belongs to the SDK and may be reused after the callback returns. Copy the data inside the callback if it must be retained.

### 6.5 Initialization and Lifecycle

```cpp
int insight9_receive_init_default(void);
int insight9_receive_init(const insight9_config_t *config);
int insight9_receive_start(void);
void insight9_receive_stop(void);
void insight9_receive_cleanup(void);
```

Call order:

1. `insight9_receive_init_default()` or `insight9_receive_init(&config)`.
2. Register callbacks.
3. `insight9_receive_start()`.
4. `insight9_receive_stop()`.
5. `insight9_receive_cleanup()`.

### 6.6 Callback Registration

```cpp
void insight9_receive_register_image_callback(image_callback cb, void *userdata);
void insight9_receive_register_imu_callback(imu_callback cb, void *userdata);
void insight9_receive_register_vio_callback(vio_callback cb, void *userdata);
```

Register callbacks before starting capture.

### 6.7 Device and Stream Control

```cpp
const char *insight9_receive_get_video_dev(int cam_id);
const char *insight9_receive_get_metadata_dev(int cam_id);
int insight9_receive_read_metadata_timestamp(int cam_id, uint64_t *timestamp);

int insight9_receive_start_camera(int cam_id);
void insight9_receive_stop_camera(int cam_id);
int insight9_receive_restart_camera(int cam_id);
int insight9_receive_is_camera_running(int cam_id);

int insight9_receive_get_current_format(int cam_id, int *width, int *height, unsigned int *format);
int insight9_receive_get_camera_config(int cam_id, int *width, int *height, PixelFormat *format, int *fps);
int insight9_receive_set_camera_fps(int cam_id, int fps);
int insight9_receive_switch_camera_fps(int cam_id, int fps);
int insight9_receive_set_camera_format(int cam_id, int width, int height, PixelFormat format);
int insight9_receive_switch_camera_format(int cam_id, int width, int height, PixelFormat format);
int insight9_receive_switch_camera_config(int cam_id, int width, int height, PixelFormat format, int fps);
int insight9_receive_get_current_fps(int *fps);
```

`set_camera_fps()` and `set_camera_format()` update the stored target config. Use the corresponding `switch_*` functions to apply changes immediately by restarting the affected camera.

### 6.8 Device Capabilities

```cpp
struct DeviceCapability {
    int width;
    int height;
    int fps;
    PixelFormat format;
    bool valid;
};

int insight9_receive_get_device_capability_count(int cam_id, int *count);
int insight9_receive_get_device_capability_by_index(int cam_id, int index, DeviceCapability *cap);
```

Capability enumeration is available from C++ callers after SDK initialization.

### 6.9 Camera Parameter Control

```cpp
int insight9_receive_set_active_camera(int cam_id);
int insight9_receive_get_active_camera(int *cam_id);
int insight9_receive_set_camera_params(const camera_params *params);
int insight9_receive_get_camera_params(camera_params *params);
int insight9_receive_set_camera_params_for(int cam_id, const camera_params *params);
int insight9_receive_get_camera_params_for(int cam_id, camera_params *params);
int insight9_receive_reset_camera_params(int cam_id);
void insight9_receive_print_camera_params(const camera_params *params);
```

`set_camera_params()` and `get_camera_params()` use the active camera. Prefer `set_camera_params_for()` and `get_camera_params_for()` when the target camera is known.

### 6.10 Calibration, VIO, and Hardware Info

```cpp
int insight9_receive_get_camera_calib(int cam_idx, camera_calib *calib);
void insight9_receive_print_camera_calib(const camera_calib *calib);

int insight9_receive_align_depth_to_rgb(const uint16_t *depth,
                                        int depth_w, int depth_h,
                                        const camera_calib *left_calib,
                                        const camera_calib *rgb_calib,
                                        uint16_t *aligned_out,
                                        int rgb_w, int rgb_h);

int insight9_receive_get_vio_status(int *status);
const char *insight9_receive_get_hardware_type(void);
```

VIO status values:

| Value | Status |
|---:|---|
| 0 | `NOT_INITED` |
| 1 | `RESTARTING` |
| 2 | `STOPPED` |
| 3 | `TRACKING` |
| 4 | `TRACKING_STATIC` |
| 5 | `TRACKINGLOST` |
| 6 | `DATA_LOST` |

## 7. Minimal Example

```cpp
#include "Insight_9_receive.h"
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

static void image_cb(int cam_id, uint8_t *data, size_t size,
                     int width, int height, unsigned int format,
                     uint64_t timestamp, void *userdata) {
    printf("Image[%d] %dx%d size=%zu fmt=0x%x ts=%lu\n",
           cam_id, width, height, size, format, (unsigned long)timestamp);
}

static void imu_cb(float ax, float ay, float az,
                   float gx, float gy, float gz,
                   uint64_t timestamp, void *userdata) {
    printf("IMU ax=%f ay=%f az=%f gx=%f gy=%f gz=%f ts=%lu\n",
           ax, ay, az, gx, gy, gz, (unsigned long)timestamp);
}

static void vio_cb(float px, float py, float pz,
                   float qx, float qy, float qz, float qw,
                   uint64_t timestamp, void *userdata) {
    printf("VIO pos=(%f,%f,%f) quat=(%f,%f,%f,%f) ts=%lu\n",
           px, py, pz, qx, qy, qz, qw, (unsigned long)timestamp);
}

int main(void) {
    if (insight9_receive_init_default() != 0) {
        fprintf(stderr, "SDK init failed\n");
        return 1;
    }

    insight9_receive_register_image_callback(image_cb, NULL);
    insight9_receive_register_imu_callback(imu_cb, NULL);
    insight9_receive_register_vio_callback(vio_cb, NULL);

    if (insight9_receive_start() != 0) {
        fprintf(stderr, "SDK start failed\n");
        insight9_receive_cleanup();
        return 1;
    }

    sleep(10);

    insight9_receive_stop();
    insight9_receive_cleanup();
    return 0;
}
```

Manual compile example:

```bash
g++ -std=c++17 -o minimal minimal.cpp -I. -Lbuild -linsight9 -lpthread -Wl,-rpath=build
sudo ./minimal
```

## 8. Included Example

`example.cpp` demonstrates:

- SDK initialization and callback registration.
- Image/IMU/VIO statistics printing.
- Camera parameter read/write.
- Current FPS, VIO status, and hardware type queries.
- Calibration readout.
- Depth-to-RGB alignment and OpenCV visualization.
- Runtime FPS switching for the gray camera.

The OpenCV window layout is:

| Position | Content |
|---|---|
| Top-left | Left grayscale image |
| Bottom-left | Right grayscale image |
| Top-right | Raw depth |
| Bottom-right | RGB with aligned depth overlay |

Press `q`, `Esc`, or `Ctrl+C` to stop the example.

## 9. Notes and Troubleshooting

- The SDK expects matching UVC video/metadata node pairs and HID devices. If initialization fails, check VID/PID, cabling, permissions, and `dmesg`.
- Linux discovery currently selects video/metadata pairs from matching `/sys/class/video4linux` nodes.
- Callbacks run on SDK capture threads. Keep callback work short; hand data off to worker threads for decoding, disk I/O, or heavy processing.
- For `Z16` depth frames, ignore metadata rows before depth processing. The included example uses `height - 2` valid depth rows.
- If UVC extension unit setup fails, image capture can still work, but parameter, calibration, VIO status, and hardware queries may fail.
- If USB recovery becomes unreliable after repeated disconnect/reconnect or rapid restart cycles, replug the device or restart the host.

## 10. Changelog

- v1.2.0 (2026-09): Updated README for current APIs; documented default/custom initialization, per-camera switching, metadata timestamps, VIO status, device capabilities, and depth-to-RGB alignment.
- v1.1.0 (2026-06): Added CMake build support, improved device discovery, and fixed build/runtime robustness issues.
- v1.1.0 (2026-05): Added UVC extension unit camera parameter API with range validation.
- v1.0.0 (2026-03): Initial Linux SDK release with UVC image capture and HID IMU/VIO callbacks.
