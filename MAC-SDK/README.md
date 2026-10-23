# Insight 9 SDK User Guide (macOS)

## 1. Overview

The Insight 9 macOS SDK provides the same C/C++ interface as the Windows SDK (`Insight9SDK.h`) for Looper Robotics Insight 9 devices on macOS. It captures RGB, grayscale and depth streams, reads IMU/VIO data, and exposes camera controls and calibration through the UVC extension unit.

| Field | Value |
|---|---|
| VID | `0x3652` |
| PID | `0x0b5c` |

### How it works on macOS

macOS has no V4L2, and AVFoundation can neither deliver the camera's custom formats (`Z16`, `Y8I`) nor reach the UVC extension unit. The SDK therefore drives the device itself over **libusb**:

| Function | Windows SDK | macOS SDK |
|---|---|---|
| Video | Media Foundation | User-space UVC over libusb (probe/commit, isochronous or bulk streaming, payload reassembly) |
| Camera params / calibration / VIO status | DirectShow KS extension unit | UVC class requests to the extension unit |
| IMU | Windows Sensor API | Raw HID reports over libusb (HID Sensor interface) |
| VIO pose | HIDAPI | Raw HID reports over libusb |
| Frame timestamp | MF capture metadata | UVC payload header (vendor metadata, else PTS) |

To use the UVC interfaces the SDK has to detach the macOS camera driver from the device, which detaches all of the device's system drivers. That has two consequences:

- **The program must run as root (`sudo`).** Without root, macOS refuses to release the device and the SDK logs `LIBUSB_ERROR_ACCESS`.
- While the SDK is running, other apps (FaceTime, Photo Booth, Zoom, ...) cannot use the camera. `insight9_receive_cleanup()` hands the device back to macOS.

## 2. Stream Model

Identical to Windows (note this differs from the Linux numbering):

| cam_id | Stream | Typical format | Notes |
|---|---|---|---|
| 0 (`INSIGHT9_CAM_RGB`) | RGB | `YUYV`, `MJPEG`, `NV12` | Standalone RGB UVC function |
| 1 (`INSIGHT9_CAM_GRAY`) | Gray | `Y8I` | Stereo grayscale, composite UVC function |
| 2 (`INSIGHT9_CAM_DEPTH`) | Depth | `Z16` | Composite UVC function; exclude the last 2 metadata rows |

The composite function (the one that offers `Z16`) carries depth and gray on two VideoStreaming interfaces. Units without a standalone RGB function (Insight 3u) work too; `insight9_receive_has_camera(INSIGHT9_CAM_RGB)` then returns 0.

HID streams: IMU → `imu_callback`, VIO pose → `vio_callback`.

## 3. Requirements

- macOS 12 or newer, Apple Silicon or Intel.
- Xcode Command Line Tools, [Homebrew](https://brew.sh), and the Homebrew packages `cmake`, `pkg-config`, `libusb` (1.0.25 or newer) and `opencv` (only needed for the example).

Install all of them with one script, run as your normal user (not with sudo):

```bash
cd MAC-SDK
bash install_deps.sh              # everything
bash install_deps.sh --cn-mirror  # same, using the Tsinghua (TUNA) Homebrew mirror (mainland China)
bash install_deps.sh --no-opencv  # library only, no OpenCV
bash install_deps.sh --build      # install, then build
```

The script skips anything that is already installed and can be rerun safely. It opens the macOS installer for the Command Line Tools if they are missing (click **Install**), installs Homebrew if needed (it asks for your login password), adds Homebrew to `~/.zprofile`, installs the packages and checks the versions.

Manual alternative:

```bash
xcode-select --install
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
brew install cmake pkg-config libusb opencv
```

## 4. Build

```bash
cd MAC-SDK
bash build.sh
```

`build.sh` uses Homebrew's OpenCV even when another OpenCV is installed, and recreates `build/` automatically if it was copied from another location. Do not copy `build/` between machines or folders.

Outputs:

- `build/libinsight9.a`: the SDK static library
- `build/example`: OpenCV demo (built only when OpenCV is found)

Options (passed through to CMake):

```bash
BUILD_TYPE=Debug bash build.sh                 # debug build
bash build.sh -DINSIGHT9_DEBUG_LOG=ON          # print [INFO]/[DEBUG] SDK logs
bash build.sh -DBUILD_SHARED_LIBS=ON           # build libinsight9.dylib instead
bash build.sh -DINSIGHT9_BUILD_EXAMPLE=OFF     # library only
```

Or with CMake directly:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$(brew --prefix)"
cmake --build build -j
```

## 5. Run the Example

Connect the camera directly (or through a powered USB 3 hub), then:

```bash
sudo ./build/example            # OpenCV window: L/R gray, depth, RGB + aligned depth
sudo ./build/example --no-gui   # console statistics only (e.g. over SSH)
sudo ./build/example --set-params  # also write demo camera parameters to the RGB camera
sudo ./build/example --rgb 1920x1080@30:mjpeg  # RGB mode instead of the device default
sudo ./build/example --no-xu-demo  # skip the parameter/calibration reads (no depth overlay)
```

By default the example only reads camera parameters. `--set-params` writes manual exposure/gain and other demo values to the RGB sensor, which persist on the device and can lower the RGB frame rate. The depth overlay on the RGB panel needs RGB at the calibrated resolution (1920x1080), e.g. `--rgb 1920x1080@30:mjpeg`.

Press `q`/`Esc` in the window or `Ctrl+C` to stop. On exit the camera is returned to macOS.

The SDK only prints errors and warnings. Build with `-DINSIGHT9_DEBUG_LOG=ON` to also see what it found on the device and how each stream was negotiated, for example (interface numbers, transfer type and formats depend on the unit and firmware):

```text
[INFO] [USB] Opened usb:1-2 (speed=4): 2 UVC function(s), 2 HID interface(s)
[INFO] [USB]   UVC function usb:1-2/if0 bcdUVC=1.10 streams=2 xu=1
[INFO] [USB]     VS if1 ep=0x82 bulk formats=[Z16 ]
[INFO] [USB]     VS if2 ep=0x84 bulk formats=[Y8I ,YUY2]
[INFO] [SDK] UVC slots: RGB=usb:1-2/if3 Composite=usb:1-2/if0
[INFO] [UVC] if1 committed fmt=1 frame=1 interval=333333 maxFrame=696320 maxPayload=696575 (len 34)
```

## 6. Use the SDK in Your Own Project

### CMake

```cmake
add_subdirectory(path/to/MAC-SDK insight9)
add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE Insight9SDK)
```

### Manual linking

```bash
clang++ -std=c++17 main.cpp -I MAC-SDK -L MAC-SDK/build -linsight9 \
    $(pkg-config --libs libusb-1.0) -o my_app
sudo ./my_app
```

### Minimal example

```cpp
#include "Insight9SDK.h"
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <thread>

static std::atomic<bool> keep_running{true};

static void image_cb(int cam_id, uint8_t *data, size_t size,
                     int width, int height, unsigned int format,
                     uint64_t timestamp, void *userdata) {
    printf("Image[%d] %dx%d size=%zu fmt=0x%x ts=%llu\n",
           cam_id, width, height, size, format, (unsigned long long)timestamp);
}

int main() {
    std::signal(SIGINT, [](int) { keep_running = false; });

    if (insight9_receive_init_default() != 0) {
        fprintf(stderr, "SDK init failed\n");
        return 1;
    }
    insight9_receive_register_image_callback(image_cb, nullptr);

    if (insight9_receive_start() != 0) {
        fprintf(stderr, "SDK start failed\n");
        insight9_receive_cleanup();
        return 1;
    }

    while (keep_running) std::this_thread::sleep_for(std::chrono::milliseconds(100));

    insight9_receive_stop();
    insight9_receive_cleanup();   // also hands the camera back to macOS
    return 0;
}
```

Call order: `init_default()`/`init(&config)` → register callbacks → `start()` → ... → `stop()` → `cleanup()`.

## 7. API Reference

The API is the same as the Windows SDK. See `Insight9SDK.h` for full documentation and `Windows-SDK/README` for the parameter tables. Summary:

| Area | Functions |
|---|---|
| Lifecycle | `insight9_receive_init_default`, `insight9_receive_init`, `insight9_receive_start`, `insight9_receive_stop`, `insight9_receive_cleanup` |
| Callbacks | `insight9_receive_register_image_callback`, `..._imu_callback`, `..._vio_callback` |
| Capabilities | `insight9_receive_get_device_capabilities_ptr`, `..._get_device_default_capability`, `..._get_device_capability_count`, `..._get_device_capability_by_index` |
| Stream control | `..._start_camera`, `..._stop_camera`, `..._restart_camera`, `..._is_camera_running`, `..._has_camera`, `..._camera_has_color`, `..._get_camera_config` |
| FPS / format | `..._set_camera_fps`, `..._switch_camera_fps`, `..._set_camera_format`, `..._switch_camera_format`, `..._switch_camera_config`, `..._get_current_fps` |
| Camera params (XU) | `..._set_active_camera`, `..._get_active_camera`, `..._set_camera_params[_for]`, `..._get_camera_params[_for]`, `..._get_cached_initial_params`, `insight9_check_camera_params`, `..._print_camera_params` |
| Calibration | `..._get_camera_calib`, `..._print_camera_calib`, `..._align_depth_to_rgb` |
| Device info | `..._get_video_dev`, `..._get_metadata_dev`, `..._read_metadata_timestamp`, `..._get_vio_status`, `..._get_hardware_type` |

### Differences from Windows

- `insight9_receive_get_video_dev()` returns a libusb path of the UVC function, e.g. `usb:2-1/if3` (bus 2, port 1, VideoControl interface 3), instead of a Windows device path.
- `insight9_receive_get_metadata_dev()` is implemented and returns the same path, because metadata arrives inline in the video payload headers.
- Image timestamps (µs) come from the device's vendor capture-timing metadata when present, otherwise from the 32-bit payload-header PTS unwrapped to 64 bits, otherwise from the host steady clock.
- Capabilities are read from the UVC descriptors, so `insight9_receive_init()` with a custom config no longer has to open a stream to probe them. The default capability is the first format's default frame at its default frame rate.
- `YVYU` frames are reported with the FourCC `0x55595659` (Windows reports 0).
- Depth and gray do **not** have to share a size (Windows forces the gray size onto depth). Depth is opened with `depth_config`. If the device does not offer that mode, the SDK derives the depth size from the gray mode, using the same rule as Linux: depth height = gray height / 2 for YUYV/YVYU, (gray height − 1) / 2 for GREY, and gray height for Y8I. If that is not offered either, it keeps the depth mode that last opened successfully, so switching the gray stream (e.g. to YUYV) never takes the composite device down. `insight9_receive_get_camera_config()` reports the depth mode actually in use.

### Callback threads

Image callbacks run on the SDK's per-UVC worker threads, and IMU/VIO callbacks on the HID threads. Keep callbacks short and copy the data if you need it after the callback returns, because the buffer is reused. Do not call OpenCV HighGUI (`imshow`, `waitKey`) from a callback; on macOS it must run on the main thread, as the example does.

### Call order constraints (device firmware)

- **Read calibration and camera parameters after `insight9_receive_start()`.** Calibration/parameter reads sent before the streams are started put the composite function into a bad state: depth arrives at about 4x its frame rate and gray delivers no frames, until the device is replugged. Expect a short frame-rate dip while a burst of such requests is served.
- **MJPEG frames may carry padding after the JPEG end marker (`FF D9`).** Trim it before handing the buffer to a strict decoder; see `process_raw_frame()` in `example.cpp`.

## 8. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `Cannot claim interface ...: LIBUSB_ERROR_ACCESS` and the sudo hint | Run with `sudo`. |
| `Cannot detach the macOS driver ...` | Run with `sudo`, and quit apps that are using the camera (FaceTime, Photo Booth, Zoom, browsers). |
| `UVC slots: RGB=(absent) Composite=(absent)` / device not found | Check the cable/port; `system_profiler SPUSBDataType` should list a device with Vendor ID `0x3652`. |
| Frames dropped / low FPS | Use a USB 3 port or powered hub; hub chains and USB 2 ports may not carry RGB + depth + gray at full rate. Lower the RGB resolution or switch it to `MJPEG`. |
| `CMake: OpenCV not found` | `bash install_deps.sh` (or `brew install opencv`); the library still builds without it. |
| `pkg-config: libusb-1.0 not found` | `bash install_deps.sh` (or `brew install libusb pkg-config`). |
| `Undefined symbols ... cv::cvtColor` at link time | Two OpenCV installations are mixed (e.g. a manual install in `/usr/local`). `build.sh` pins Homebrew's OpenCV; rebuild with `rm -rf build && bash build.sh`. |
| Depth at ~120 fps, no gray frames | Calibration/parameters were read before `insight9_receive_start()`; move those calls after it and replug the camera. |
| The camera stays unavailable to other apps after a crash | The SDK hands the device back in `insight9_receive_cleanup()`. After `kill -9` or a crash, unplug and replug the camera. |
| Parameter/calibration calls fail but video works | The extension unit did not answer; check firmware. The log shows which XU unit/GUID was probed. |

Set `-DINSIGHT9_DEBUG_LOG=ON` to see probe/commit results, the selected alternate settings and dropped frames.

## 9. Files

| File | Purpose |
|---|---|
| `Insight9SDK.h` | Public API (same as Windows) |
| `Insight9SDK.cpp` | SDK core: device slots, workers, reconnect, HID, API |
| `UvcDevice.hpp/.cpp` | libusb runtime, descriptor parsing, UVC streaming |
| `ExtensionUnitControl.hpp/.cpp` | UVC extension unit (camera params, calibration, VIO status) |
| `example.cpp` | OpenCV demo, ported from the Windows example |
| `install_deps.sh` | One-step installation of Xcode CLT, Homebrew and the build dependencies |
| `CMakeLists.txt`, `build.sh` | Build |

## 10. Changelog

- v1.0.0 (2026-10): Initial macOS SDK, ported from the Windows SDK API with a libusb user-space UVC/HID backend.
