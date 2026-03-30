# Insight 9 SDK User Guide

## 1. Overview
The Insight 9 SDK is a dynamic library for capturing data from three UVC cameras (main MJPEG, left/right mono8) and two HID devices (IMU, VIO). The SDK automatically discovers devices with the specified VID/PID, selects appropriate devices after sorting by device number, and passes the captured image and sensor data to the application via callbacks.

## 2. Features
- **Automatic device discovery**: Finds UVC and HID devices based on VID=0x1d6b, PID=0x0104.

- **Intelligent selection**:  
  - UVC: Sorts all matching video nodes numerically and selects indices 0, 2, 4 as the three cameras (initialization fails if less than 6 devices exist).  
  - HID: Sorts matching hidraw nodes numerically, takes the first two; the smaller number is used for IMU, the larger for VIO.

- **Multi‑threaded capture**: Each device runs in its own thread without interference.

- **Hot‑plug support**: Automatically reconnects after device disconnection and resumes data streaming.

- **Callback mechanism**: Image, IMU, and VIO data are delivered to the application in real time.

- **Simple API**: Only a few calls are needed to start/stop data acquisition.

## 3. Dependencies
- **OS**: Linux (requires sysfs, V4L2, HIDRAW support)

- **Build**: gcc, make, pthread library

- **Runtime**: Access to `/sys/class/video4linux` and `/sys/class/hidraw`; read/write permissions for `/dev/video*` and `/dev/hidraw*`

## 4. Building and Installation
### 4.1 Build the shared library
Place `Insight_9_receive.c` and `Insight_9_receive.h` in the same directory and execute:

bash
gcc -c -fPIC Insight_9_receive.c -o Insight_9_receive.o
gcc -shared -o libinsight9.so Insight_9_receive.o -lpthread
This generates libinsight9.so.

### 4.2 Install
Copy libinsight9.so and Insight_9_receive.h to system or project directories, e.g.:

bash
sudo cp libinsight9.so /usr/local/lib/
sudo cp Insight_9_receive.h /usr/local/include/
sudo ldconfig

### 5. API Reference
### 5.1 Header
c
#include "Insight_9_receive.h"

### 5.2 Callback Types
Image callback
c
typedef void (*image_callback)(int cam_id, uint8_t *data, size_t size,
                               int width, int height, unsigned int format,
                               uint64_t timestamp, void *userdata);
cam_id: 0 = main RGB, 1 = left mono, 2 = right mono

data: Image data (MJPEG or raw mono)

size: Data size in bytes

width, height: Image dimensions

format: V4L2 pixel format (V4L2_PIX_FMT_MJPEG or V4L2_PIX_FMT_GREY)

timestamp: Device‑extracted timestamp (microseconds)

userdata: User pointer passed during registration

IMU callback
c
typedef void (*imu_callback)(float ax, float ay, float az,
                             float gx, float gy, float gz,
                             uint32_t timestamp, void *userdata);
ax, ay, az: Accelerometer values (physical units)

gx, gy, gz: Gyroscope values (physical units)

timestamp: Device timestamp

VIO callback
c
typedef void (*vio_callback)(float px, float py, float pz,
                             float qx, float qy, float qz, float qw,
                             uint32_t seq, void *userdata);
px, py, pz: Position coordinates

qx, qy, qz, qw: Quaternion orientation

seq: Sequence number

### 5.3 SDK Control Functions
int insight9_receive_init(void)
Initializes the SDK, scans for devices, and allocates resources.
Returns: 0 on success, -1 on failure (insufficient devices or permission issues).

int insight9_receive_start(void)
Starts all acquisition threads.
Returns: 0 on success, -1 if not initialized or already running.

void insight9_receive_stop(void)
Stops all acquisition threads (blocks until threads exit).

void insight9_receive_cleanup(void)
Releases all resources (must be called after stop).

### 5.4 Callback Registration
void insight9_receive_register_image_callback(image_callback cb, void *userdata)
Registers the image callback. Must be called before insight9_receive_start().

void insight9_receive_register_imu_callback(imu_callback cb, void *userdata)
Registers the IMU callback.

void insight9_receive_register_vio_callback(vio_callback cb, void *userdata)
Registers the VIO callback.

## 6. Usage Example
c
#include "Insight_9_receive.h"
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

static volatile int keep_running = 1;

void sigint_handler(int sig) {
    keep_running = 0;
}

void image_callback(int cam_id, uint8_t *data, size_t size,
                    int width, int height, unsigned int format,
                    uint64_t timestamp, void *userdata) {
    printf("CAM%d: %zub, %dx%d, ts=%lu\n", cam_id, size, width, height, timestamp);
    // Copy data if needed for later use
}

void imu_callback(float ax, float ay, float az,
                  float gx, float gy, float gz,
                  uint32_t timestamp, void *userdata) {
    printf("IMU: acc=(%.2f,%.2f,%.2f) gyro=(%.2f,%.2f,%.2f) ts=%u\n",
           ax, ay, az, gx, gy, gz, timestamp);
}

void vio_callback(float px, float py, float pz,
                  float qx, float qy, float qz, float qw,
                  uint32_t seq, void *userdata) {
    printf("VIO: pos=(%.2f,%.2f,%.2f) quat=(%.2f,%.2f,%.2f,%.2f) seq=%u\n",
           px, py, pz, qx, qy, qz, qw, seq);
}

int main() {
    signal(SIGINT, sigint_handler);

    if (insight9_receive_init() != 0) {
        fprintf(stderr, "SDK init failed\n");
        return -1;
    }

    insight9_receive_register_image_callback(image_callback, NULL);
    insight9_receive_register_imu_callback(imu_callback, NULL);
    insight9_receive_register_vio_callback(vio_callback, NULL);

    if (insight9_receive_start() != 0) {
        fprintf(stderr, "SDK start failed\n");
        insight9_receive_cleanup();
        return -1;
    }

    printf("SDK running, press Ctrl+C to stop...\n");
    while (keep_running) {
        sleep(1);
    }

    insight9_receive_stop();
    insight9_receive_cleanup();
    return 0;
}
Compile the example:

bash
gcc -o example example.c -L. -linsight9 -lpthread
export LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH
./example

## 7. Important Notes
Data lifecycle: The image data pointer points to an internal SDK buffer that will be reused after the callback returns. If you need to keep the data, copy it inside the callback.

Thread safety: Callbacks are invoked from acquisition threads; avoid blocking operations (e.g., file I/O, long computations) to prevent frame drops.

Device permissions: The program must have read/write access to /dev/video* and /dev/hidraw*. This often requires root privileges or membership in the video and plugdev groups.

Device count requirement: The SDK expects at least 5 UVC devices (to select indices 0, 2, 4) and 2 HID devices; otherwise insight9_receive_init() will return -1.

Hot‑plug: After device disconnection, the acquisition threads will automatically attempt to reconnect. Data flow will pause and resume when the device becomes available again.

Error handling: If insight9_receive_init() fails, it is safe to call insight9_receive_cleanup() (though the SDK already cleans up internally, calling it ensures proper resource release).

## 8. Frequently Asked Questions
Q: How can I verify that devices are correctly detected?
A: When the program runs, it prints the selected device nodes, e.g., Selected UVC device 0: /dev/video4. For debugging, you can inspect internal variables like g_ctx.video_devs after insight9_receive_init() (not recommended for production).

Q: The image data is MJPEG. How do I decode it?
A: You can use libjpeg or a hardware decoder to convert MJPEG to RGB/YUV. The mono (GREY) data is raw 8‑bit pixel data and can be saved as PGM or processed with OpenCV.

Q: How can I adjust camera parameters (exposure, gain)?
A: The current SDK does not expose parameter tuning. If needed, you can extend the init_capture() function with VIDIOC_S_CTRL calls to set V4L2 controls.

Q: I get “cannot find libinsight9.so” when running the example.
A: Ensure the library path is in LD_LIBRARY_PATH or use -Wl,-rpath during compilation, e.g.:

bash
gcc -o example example.c -L. -linsight9 -Wl,-rpath=.

## 9. Changelog
v1.0 (2025‑03): Initial release – supports automatic device discovery and acquisition for three UVC cameras and two HID devices.

For further assistance or customisations, please contact the developer.