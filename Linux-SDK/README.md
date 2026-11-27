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
- Depth-to-point-cloud conversion (camera or world frame, optional RGB/gray colour) and a global point cloud map.
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
sudo ./build/example -l   # also save the run log to example_YYYYMMDD_HHMMSS.log in the current directory
```

With `-l`, everything the example and the SDK print to stdout/stderr is also written to the log file, while still showing on the console.

Install the shared library and headers:

```bash
sudo cmake --install build
sudo ldconfig
```

The build produces:

- `build/libinsight9.so`
- `build/example`
- `build/pointcloud_example`

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

### 6.11 Point Cloud

`Insight9PointCloud.cpp` turns Z16 depth frames into point clouds, either per frame or accumulated into a global map. Points are `insight9_point { float x, y, z; uint32_t rgb; }` (metres, `0x00RRGGBB`). The layout matches `pcl::PointXYZRGB` and a ROS `PointCloud2` with fields `x,y,z,rgb` at offsets 0/4/8/12, so an array can be copied straight into either.

```cpp
void   insight9_pointcloud_default_params(insight9_pointcloud_params *params);
size_t insight9_pointcloud_max_points(int depth_w, int depth_h, int pixel_step);
int    insight9_receive_depth_to_pointcloud(const uint16_t *depth, int depth_w, int depth_h,
                                            const camera_calib *left_calib,
                                            const insight9_pointcloud_params *params,
                                            insight9_point *out, size_t out_capacity,
                                            size_t *out_count);
int    insight9_pointcloud_pose_to_transform(float px, float py, float pz,
                                             float qx, float qy, float qz, float qw,
                                             const camera_extrinsics *body_to_left,
                                             float T_out[16]);

insight9_pointcloud_map *insight9_pointcloud_map_create(const insight9_pointcloud_map_params *params);
int  insight9_pointcloud_map_insert(insight9_pointcloud_map *map, const insight9_point *points,
                                    size_t count, const float T_world_left[16]);
int  insight9_pointcloud_map_get(insight9_pointcloud_map *map, insight9_point *out,
                                 size_t out_capacity, size_t *out_count);
void insight9_pointcloud_map_clear(insight9_pointcloud_map *map);
void insight9_pointcloud_map_destroy(insight9_pointcloud_map *map);
```

- **Single frame.** Depth is deprojected with the LEFT intrinsics. Defaults: every 2nd pixel, depth up to 2.0 m. Pass the valid depth rows only (`height - 2` for Z16).
- **Colour (optional).** `INSIGHT9_PC_COLOR_RGB8` / `BGR8` takes a decoded RGB-camera image plus `rgb_calib`; points that fall outside the image are dropped unless `keep_uncolored` is set. `INSIGHT9_PC_COLOR_GRAY8` takes the LEFT grayscale eye, which must be the depth size. Use `color_pixel_stride = 2` for Y8I, where the left eye is every other byte.
- **World frame.** Set `params.transform` to `T_world_left` from `insight9_pointcloud_pose_to_transform()`, built from the VIO pose and the fixed LEFT-camera-to-VIO-body extrinsic. With no transform, points stay in the LEFT camera optical frame.
- **Global map.** Keeps a frame only when the camera moved >= 3 cm or turned >= 1 degree. Each keyframe is downsampled to 5 cm voxels, and keyframes and points farther than 100 m from the current position are dropped. Map functions are thread-safe. Call `map_get` with `out = NULL` to query the size first.

```cpp
insight9_pointcloud_params pc;
insight9_pointcloud_default_params(&pc);
pc.color_format = INSIGHT9_PC_COLOR_BGR8;          // cv::imdecode output
pc.color = bgr.data; pc.color_width = bgr.cols; pc.color_height = bgr.rows;
pc.rgb_calib = &rgb_calib;

float T_world_left[16];
insight9_pointcloud_pose_to_transform(px, py, pz, qx, qy, qz, qw, &body_to_left, T_world_left);
pc.transform = T_world_left;

std::vector<insight9_point> pts(insight9_pointcloud_max_points(w, h - 2, pc.pixel_step));
size_t n = 0;
if (insight9_receive_depth_to_pointcloud(depth, w, h - 2, &left_calib, &pc,
                                         pts.data(), pts.size(), &n) == 0) {
    insight9_pointcloud_map_insert(map, pts.data(), n, T_world_left);
}
```

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

### 8.1 Point Cloud Example

`pointcloud_example.cpp` shows the point cloud API (section 6.11) end to end:

1. Each depth frame becomes a point cloud in the LEFT camera frame with `insight9_receive_depth_to_pointcloud()`. The LEFT intrinsics are scaled to the depth resolution first, because the calibration covers a single resolution and depth follows the stereo stream's resolution. Flying pixels at object edges are filtered out. While the device's depth pipeline is not running (right after it connects, or stuck until it is replugged), the depth stream delivers a placeholder frame that shows the text "Depth not enabled or not ready". The example detects such frames and skips them until real depth arrives. It also sends no control transfers until then: calibration is read on the first real depth frame and VIO status is polled every 2 s, as in Looper Viewer, because control transfers sent while the streams start up can keep depth from starting. Points are coloured from the LEFT eye (`INSIGHT9_PC_COLOR_GRAY8`, pixel-aligned with depth; GREY, Y8I and YUYV are handled) or from the RGB camera (`INSIGHT9_PC_COLOR_BGR8`).
2. While VIO is tracking, the latest VIO pose becomes `T_world_left` through `insight9_pointcloud_pose_to_transform()`, and passing it as `params.transform` puts the cloud in the world frame.
3. World-frame clouds go into a global map with `insight9_pointcloud_map_insert()`. The map is read back with `insight9_pointcloud_map_get()`, querying the size first.
4. A preview window draws the current frame (default) or the map, with a mouse-orbit view. The last frame and the map are saved as PLY files (`insight9_frame.ply`, `insight9_map.ply`), which open in MeshLab, CloudCompare or Open3D.

```
sudo ./build/pointcloud_example
sudo ./build/pointcloud_example --no-gui --out scan
```

| Option | Meaning |
|---|---|
| `--no-gui` | Console output only. PLY files are written on exit. |
| `--color gray\|rgb\|none` | Point colour: LEFT eye (default), RGB camera, or none (shaded by depth). |
| `--step N` | Use every Nth depth pixel (default 1; keys `1`-`4` change it while running). |
| `--max-depth M` | Drop points farther than M metres (default 8.0; `,` and `.` change it while running). |
| `--no-filter` | Keep flying pixels at depth edges. |
| `--voxel V` | Map voxel size in metres (default 0.05). |
| `--map-max-depth M` | Only points within M metres of the camera go into the map (default 5.0). |
| `--pose-delay MS` | Look up the VIO pose MS milliseconds before the depth frame's timestamp (default 0). |
| `--no-map` | Camera-frame clouds only; VIO and the map are not used. |
| `--body-to-left calib\|flu\|none` | Source of the LEFT camera to VIO body extrinsic (default `calib`, see below). |
| `--out PREFIX` | PLY file name prefix (default `insight9`). |

In the preview window, drag with the left mouse button to orbit and use the wheel or `+`/`-` to zoom. Keys: `[`/`]` change the point size, `r` resets the view, `m` switches between frame and map view, `s` saves PLY files, `d` saves the raw depth frame as a 16-bit PNG, `c` clears the map, `q`/`Esc` quits.

The map needs the fixed LEFT-camera-to-VIO-body extrinsic. The VIO body is the IMU, and the LEFT calibration's extrinsic is exactly IMU -> LEFT (`camera_camera_imu` -> `camera_camera_left`), so the default `--body-to-left calib` uses it. `flu` (x forward, y left, z up) and `none` (VIO pose = LEFT pose) are for other devices.

How the map stays sharp: each depth frame is paired with the VIO pose interpolated at the depth frame's capture time. Image timestamps are microseconds and VIO timestamps nanoseconds of the same device clock; depth reaches the host about 100 ms after the matching pose, so the newest pose would be off while the camera moves. `--pose-delay` shifts the lookup further if needed. Frames are added as keyframes every 5 cm or 2 degrees, and only points within `--map-max-depth` go in. For display and PLY export the map is merged into one averaged point per voxel, and voxels seen only once are dropped as noise. The preview redraws at most 15 times per second so that drawing never holds up mapping; the console prints the processing time per depth frame and per redraw.

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
