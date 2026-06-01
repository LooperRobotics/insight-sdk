# Insight 9 SDK User Guide

## 1. Overview

The Insight 9 SDK is a dynamic library for capturing data from three UVC cameras (main MJPEG, left/right mono8) and two HID devices (IMU, VIO), and also provides control over camera parameters (exposure, gain, brightness, white balance, etc.) via the UVC extension unit.

The SDK automatically discovers devices with the specified VID/PID, selects appropriate devices after sorting by device number, and passes the captured image and sensor data to the application via callbacks. Camera parameter adjustment APIs allow real-time tuning of image quality.

## 2. Features

- **Automatic device discovery**: Finds UVC and HID devices based on VID=0x1d6b, PID=0x0104.

- **Intelligent selection**:  
  - UVC: Sorts all matching video nodes numerically and selects indices 0, 2, 4 as the three cameras (initialization fails if less than 6 devices exist).  
  - HID: Sorts matching hidraw nodes numerically, takes the first two; the smaller number is used for IMU, the larger for VIO.

- **Multi‑threaded capture**: Each device runs in its own thread without interference.

- **Hot‑plug support**: Automatically reconnects after device disconnection and resumes data streaming.

- **Callback mechanism**: Image, IMU, and VIO data are delivered to the application in real time.

- **Extension Unit (XU) parameter control**: Full control over camera parameters including exposure, gain, brightness, contrast, gamma, hue, saturation, sharpness, white balance, and more.

- **Simple API**: Only a few calls are needed to start/stop data acquisition and adjust camera settings.

## 3. Dependencies

- **OS**: Linux (requires sysfs, V4L2, HIDRAW, UVC extension unit support)

- **Build**: gcc/g++, make, pthread library

- **Runtime**: Access to `/sys/class/video4linux` and `/sys/class/hidraw`; read/write permissions for `/dev/video*` and `/dev/hidraw*`

## 4. Building and Installation

### 4.1 Build the shared library

Place `Insight_9_receive.cpp`, `Insight_9_receive.h`, `UvcExtensionUnit.cpp`, and `UvcExtensionUnit.hpp` in the same directory and execute:

```bash
g++ -c -fPIC Insight_9_receive.cpp -o Insight_9_receive.o
g++ -c -fPIC UvcExtensionUnit.cpp -o UvcExtensionUnit.o
g++ -shared -o libinsight9.so Insight_9_receive.o UvcExtensionUnit.o -lpthread
```

This generates libinsight9.so.

### 4.2 Install

Copy libinsight9.so and Insight_9_receive.h to system or project directories, e.g.:

```bash
sudo cp libinsight9.so /usr/local/lib/
sudo cp Insight_9_receive.h /usr/local/include/
sudo ldconfig
```

## 5. API Reference
### 5.1 Header
c
#include "Insight_9_receive.h"

### 5.2 Data Structures

camera_params
c
typedef struct {
    uint8_t cam_id;             // Camera ID (0: RGB, 1: Mono)
    uint8_t resolution;         // Resolution index (RGB: 0-3, Mono: 0-1) Not enabled!!!
    uint8_t frame_rate;         // Frame rate index (0-5: 90,60,30,20,15,10 fps) Not enabled!!!
    float exposure_time;        // Exposure time (0.0 ~ 0.03 seconds)
    float exposure_gain;        // Exposure gain (1.0 ~ 16.0)
    uint8_t auto_exposure;      // Auto Exposure (0 ~ 1)
    float brightness;           // Brightness (0.0 ~ 127.0)
    float contrast;             // Contrast (0.0 ~ 1.9)
    float gamma_dark;           // Gamma dark (1.0 ~ 4.0)
    float hue;                  // Hue (0.0 ~ 87.0)
    float saturation;           // Saturation (0.0 ~ 1.999)
    uint8_t sharpness;          // Sharpness (1 ~ 255)
    uint8_t auto_white_balance; // Auto white balance (0 or 1)
    float white_balance;        // White balance (1.0 ~ 3.0)
    uint8_t decimation;         // Decimation factor (1 ~ 255) Not enabled!!!
    uint8_t rotation;           // Rotation (1 ~ 255) Not enabled!!!
} camera_params;
Note: All fields are range-checked by the SDK before sending to the device. Invalid values will cause the set function to fail.

### 5.3 Callback Types

Image callback
c
typedef void (*image_callback)(int cam_id, uint8_t *data, size_t size,
                               int width, int height, unsigned int format,
                               uint64_t timestamp, uint64_t right_timestamp,
                               void *userdata);
cam_id: 0 = RGB (MJPEG), 1 = left mono (GREY), 2 = right mono (GREY), 3 = depth (Z16)

data: Image data (MJPEG, raw grey, or Z16)

size: Data size in bytes

width, height: Image dimensions

format: V4L2 pixel format (V4L2_PIX_FMT_MJPEG, V4L2_PIX_FMT_GREY, V4L2_PIX_FMT_Z16)

timestamp: Device timestamp (microseconds)

right_timestamp: Right image timestamp (for stereo cameras)

userdata: User pointer passed during registration

**IMU callback**
c
typedef void (*imu_callback)(float ax, float ay, float az,
                             float gx, float gy, float gz,
                             uint64_t timestamp, void *userdata);
ax, ay, az: Accelerometer values (physical units)

gx, gy, gz: Gyroscope values (physical units)

timestamp: Device timestamp

userdata: User pointer

**VIO callback**
c
typedef void (*vio_callback)(float px, float py, float pz,
                             float qx, float qy, float qz, float qw,
                             uint64_t timestamp, void *userdata);
px, py, pz: Position coordinates

qx, qy, qz, qw: Quaternion orientation

timestamp: Device timestamp

userdata: User pointer

### 5.4 SDK Control Functions

**Initialization and Cleanup**
c
int insight9_receive_init(void);
Initializes the SDK, scans for devices, and allocates resources.
Returns: 0 on success, -1 on failure (insufficient devices or permission issues).

c
int insight9_receive_start(void);
Starts all acquisition threads.
Returns: 0 on success, -1 if not initialized or already running.

c
void insight9_receive_stop(void);
Stops all acquisition threads (blocks until threads exit).

c
void insight9_receive_cleanup(void);
Releases all resources (must be called after stop).

**Utility**
c
const char *insight9_receive_get_video_dev(int cam_id);
Returns the video device path (e.g., /dev/video0) for the specified camera.

c
void insight9_receive_print_camera_params(const camera_params *params);
Prints the specified camera parameters to stdout.

### 5.5 Callback Registration
c
void insight9_receive_register_image_callback(image_callback cb, void *userdata);
void insight9_receive_register_imu_callback(imu_callback cb, void *userdata);
void insight9_receive_register_vio_callback(vio_callback cb, void *userdata);
Register callbacks before calling insight9_receive_start().

### 5.6 Extension Unit (Camera Parameter) API
c
int insight9_receive_set_active_camera(int cam_id);
int insight9_receive_get_active_camera(int *cam_id);
Set/get the currently active camera (0: RGB, 1: Mono). Parameter read/write operations apply to the active camera.

c
int insight9_receive_set_camera_params(const camera_params *params);
int insight9_receive_get_camera_params(camera_params *params);
Set/get parameters for the currently active camera.
Note: The cam_id field in params is ignored for these calls.

c
int insight9_receive_set_camera_params_for(int cam_id, const camera_params *params);
int insight9_receive_get_camera_params_for(int cam_id, camera_params *params);
Set/get parameters for a specific camera (0/1/2). The cam_id field in params is overridden by the cam_id argument.

c
int insight9_receive_reset_camera_params(int cam_id);
Reset camera parameters to their default values (currently not fully implemented; use set_camera_params_for with saved defaults).

All set functions perform range validation on every parameter. If any value is out of the allowed range, the function returns -1 and no command is sent to the device.

## 6. Usage Example
```c
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
```

Compile and run:

```bash
g++ -o example example.c -L. -linsight9 -lpthread -Wl,-rpath=.
export LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH
sudo ./example
```

## 7. Important Notes
Data lifecycle: The image data pointer points to an internal SDK buffer that will be reused after the callback returns. If you need to keep the data, copy it inside the callback.

Thread safety: Callbacks are invoked from acquisition threads; avoid blocking operations (e.g., file I/O, long computations) to prevent frame drops.

Device permissions: The program must have read/write access to /dev/video* and /dev/hidraw*. This often requires root privileges or membership in the video and plugdev groups.

Device count requirement: The SDK expects at least 6 UVC devices (to select indices 0, 2, 4) and 2 HID devices; otherwise insight9_receive_init() will return -1.

Hot‑plug: After device disconnection, the acquisition threads will automatically attempt to reconnect using the same device roles. Data flow will pause and resume when the device becomes available again.

Parameter validation: All set functions validate each parameter against the defined range. The SDK will reject out-of-range values and return an error without sending anything to the device. This prevents invalid configurations that could cause USB communication errors.

Extension unit availability: The extension unit is opened on the first camera (/dev/video0). If it cannot be opened (e.g., missing kernel support), camera parameter functions will return -1. The rest of the SDK (video capture) will still work.

Error handling: If insight9_receive_init() fails, it is safe to call insight9_receive_cleanup() (though the SDK already cleans up internally, calling it ensures proper resource release).

## 8. Parameter Range Reference
Parameter	        Min	    Max	    Notes
exposure_time (s)	0.0	    0.03	Linear mapping from UI value
exposure_gain	    1.0	    16.0	Linear mapping
auto_exposure	    0	    1	    Boolean
brightness	        0.0	    127.0	Linear mapping from -64..64
contrast	        0.0	    1.9	    Linear mapping from 0..100
gamma_dark	        1.0	    4.0	    Linear mapping from 100..500
hue	                0.0	    87.0	Linear mapping from -180..180
saturation	        0.0	    1.999	Linear mapping from 0..100
sharpness	        1	    255	    Linear mapping from 0..100
auto_white_balance	0	    1	    Boolean
white_balance	    1.0	    3.0	    Linear mapping from 2800..6500K
decimation	        1	    255	    Linear mapping from 1..8
rotation	        1	    255	    Linear mapping from -90..180

## 9. Frequently Asked Questions
Q: How can I verify that devices are correctly detected?
A: When the program runs, it prints the selected device nodes, e.g., Selected UVC device 0: /dev/video4. You can also call insight9_receive_get_video_dev() after init.

Q: The image data is MJPEG. How do I decode it?
A: You can use libjpeg, turbojpeg, or a hardware decoder to convert MJPEG to RGB/YUV. Mono (GREY) data is raw 8‑bit and can be saved as PGM or processed with OpenCV. Z16 is 16‑bit depth data (little-endian).

Q: The SDK returns -1 when setting parameters. What could be wrong?
A: Possible reasons:

The extension unit was not opened during init (check console for warning).

One or more parameter values are out of the allowed range (see table above).

The device is disconnected or the extension unit is busy.

The kernel UVC driver does not support the control.

Q: How do I save current parameters as defaults for reset?
A: Use insight9_receive_get_camera_params_for() to read the current parameters and store them. Later, call insight9_receive_set_camera_params_for() with the stored values to reset.

Q: I get “cannot find libinsight9.so” when running the example.
A: Ensure the library path is in LD_LIBRARY_PATH or use -Wl,-rpath during compilation, e.g.:


```bash
g++ -o example example.c -L. -linsight9 -Wl,-rpath=.
```

## 10. Changelog
v1.1 (2025-05): Added Extension Unit API for camera parameter control (exposure, gain, white balance, etc.). Range validation implemented. Updated to C++ for compatibility with UvcExtensionUnit class.

v1.0 (2025-03): Initial release – supports automatic device discovery and acquisition for three UVC cameras and two HID devices.

For further assistance or customisations, please contact the developer.