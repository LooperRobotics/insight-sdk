#ifndef INSIGHT_SDK_H
#define INSIGHT_SDK_H

#include <stdint.h>
#include <stddef.h>
#include <pthread.h>
#include <atomic>


// ==================== Target Device VID/PID ====================
#define VENDOR_ID  0x3652
#define PRODUCT_ID 0x0b5c

// ==================== Camera Configuration ====================
#define CAM_NUM 3
#define HID_NUM 2
#define MAIN_WIDTH   1088
#define MAIN_HEIGHT  1920
#define MAIN_FORMAT  V4L2_PIX_FMT_MJPEG
#define SUB_WIDTH    544
#define SUB_HEIGHT   1281
#define SUB_FORMAT   V4L2_PIX_FMT_GREY
#define DEPTH_WIDTH  544
#define DEPTH_HEIGHT 642
#define DEPTH_FORMAT V4L2_PIX_FMT_Z16
#define FRAME_RATE 30
#define BUFFER_COUNT 8
#define MAX_PATH 1024
#define METADATA_SIZE 258

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


#pragma pack(push, 1)
typedef struct {
    uint8_t cam_id;             // Camera ID, 0/1
    uint8_t resolution;         // Resolution index
    uint8_t frame_rate;         // Frame-rate index, 0-5
    float exposure_time;        // Exposure time in seconds, range 0.0~0.03
    float exposure_gain;        // Exposure gain, range 1.0~16.0
    uint8_t auto_exposure;      // Auto exposure, 0/1
    float brightness;           // Brightness, range 0.0~127.0
    float contrast;             // Contrast, range 0.0~1.9
    float gamma_dark;           // Dark gamma, range 1.0~4.0
    float hue;                  // Hue, range 0.0~87.0
    float saturation;           // Saturation, range 0.0~1.999
    uint8_t sharpness;          // Sharpness (1~255)
    uint8_t auto_white_balance; // Auto white balance, 0 or 1
    float white_balance;        // White balance, range 1.0~3.0
    uint8_t decimation;         // Decimation (1~255)
    uint8_t hardware_model;     // Hardware model
} camera_params;

/**
 * Camera intrinsics, fields aligned with ROS sensor_msgs/CameraInfo.
 */
typedef struct {
    uint32_t sec;               // Calibration timestamp (seconds)
    uint32_t nsec;              // Calibration timestamp (nanoseconds)

    char frame_id[32];          // Camera frame id

    uint32_t height;            // Calibration image height
    uint32_t width;             // Calibration image width

    char distortion_model[32];  // e.g. "plumb_bob", "equidistant"

    float d[4];                 // Distortion coefficients
    float k[9];                 // 3x3 intrinsic matrix, row-major
    float r[9];                 // 3x3 rectification matrix, row-major
    float p[12];                // 3x4 projection matrix, row-major

    uint32_t binning_x;
    uint32_t binning_y;

    uint32_t roi_x_offset;
    uint32_t roi_y_offset;
    uint32_t roi_height;
    uint32_t roi_width;

    uint8_t roi_do_rectify;
} camera_intrinsics;

/**
 * Camera extrinsics, one transform from the device /tf_static.
 */
typedef struct {
    char parent_frame_id[32];   // Reference frame
    char child_frame_id[32];    // This camera frame

    double translation[3];      // x, y, z (m)
    double rotation[4];         // Quaternion x, y, z, w
} camera_extrinsics;

/**
 * Full calibration of one camera: intrinsics + extrinsics.
 */
typedef struct {
    camera_intrinsics intrinsics;
    camera_extrinsics extrinsics;
} camera_calib;
#pragma pack(pop)

/* Camera index for insight9_receive_get_camera_calib(). */
enum {
    INSIGHT9_CALIB_CAM_LEFT  = 0,   /* Left grayscale  */
    INSIGHT9_CALIB_CAM_RIGHT = 1,   /* Right grayscale */
    INSIGHT9_CALIB_CAM_RGB   = 2,   /* RGB             */
};

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

struct DeviceCapability {
    int width;
    int height;
    int fps;
    PixelFormat format;
    bool valid;

    DeviceCapability() : width(0), height(0), fps(0), format(PixelFormat::Unknown), valid(false) {}
};

// VIO status enumeration, matching the device firmware
enum class VioStatus : uint8_t {
    NOT_INITED      = 0,
    RESTARTING      = 1,
    STOPPED         = 2,
    TRACKING        = 3,
    TRACKING_STATIC = 4,
    TRACKINGLOST    = 5,
    DATA_LOST       = 6,
    Unknown         = 0xFF
};
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Image data callback.
 * @param cam_id    Camera ID (0: main RGB, 1: grayscale, 2: depth).
 * @param data      Image data pointer (JPEG data for MJPEG, raw data for GREY and Z16).
 * @param size      Data size in bytes.
 * @param width     Image width.
 * @param height    Image height.
 * @param format    V4L2 pixel format, such as V4L2_PIX_FMT_MJPEG or V4L2_PIX_FMT_GREY.
 * @param timestamp Image timestamp in microseconds, provided by the device or system time.
 * @param userdata  User pointer passed when the callback is registered.
 */
typedef void (*image_callback)(int cam_id, uint8_t *data, size_t size,
                               int width, int height, unsigned int format,
                               uint64_t timestamp, void *userdata);

/**
 * @brief IMU data callback.
 * @param ax,ay,az  Raw accelerometer values.
 * @param gx,gy,gz  Raw gyroscope values.
 * @param timestamp Timestamp provided by the device.
 * @param userdata  User pointer.
 */
typedef void (*imu_callback)(float ax, float ay, float az,
                             float gx, float gy, float gz,
                             uint64_t timestamp, void *userdata);

/**
 * @brief VIO pose data callback.
 * @param px,py,pz  Position coordinates.
 * @param qx,qy,qz,qw Quaternion orientation.
 * @param seq       Sequence number.
 * @param userdata  User pointer.
 */
typedef void (*vio_callback)(float px, float py, float pz,
                             float qx, float qy, float qz, float qw,
                             uint64_t timestamp, void *userdata);

/**
 * @brief Initialize the SDK with custom configuration.
 * @param config Configuration structure containing resolution, fps, and pixel format for each camera.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_init(const insight9_config_t* config);

/**
 * @brief Initialize the SDK with default configuration.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_init_default(void);

/**
 * @brief Start all capture threads.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_start(void);

/**
 * @brief Get current video format for a specific camera device.
 * @param cam_id Camera ID (0..2).
 * @param width Output width.
 * @param height Output height.
 * @param format Output V4L2 pixel format.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_get_current_format(int cam_id, int *width, int *height, unsigned int *format);

/**
 * @brief Get the video device path for the specified camera.
 * @param cam_id Camera index (0..2).
 * @return Device path string, or NULL on failure.
 */
const char *insight9_receive_get_video_dev(int cam_id);

/**
 * @brief Get the metadata device path for the specified camera.
 * @param cam_id Camera index (0..2).
 * @return Device path string, or NULL on failure.
 */
const char *insight9_receive_get_metadata_dev(int cam_id);

/**
 * @brief Read metadata timestamp from the metadata device.
 * @param cam_id Camera index.
 * @param timestamp Output parameter for the timestamp.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_read_metadata_timestamp(int cam_id, uint64_t *timestamp);

/**
 * @brief Start a specific camera.
 * @param cam_id Camera ID (0: RGB, 1: Grayscale, 2: Depth).
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_start_camera(int cam_id);

/**
 * @brief Stop a specific camera.
 * @param cam_id Camera ID.
 */
void insight9_receive_stop_camera(int cam_id);

/**
 * @brief Get the current configuration (resolution, pixel format, and frame rate) of the specified camera.
 * @param cam_id Camera ID (0: Depth, 1: Grayscale, 2: RGB on Linux; may vary by platform).
 * @param width  Output pointer to receive the current width.
 * @param height Output pointer to receive the current height.
 * @param format Output pointer to receive the current pixel format.
 * @param fps    Output pointer to receive the current frame rate.
 * @return 0 on success, -1 on failure (e.g., invalid cam_id or NULL pointer).
 */
int insight9_receive_get_camera_config(int cam_id, int* width, int* height, PixelFormat* format, int* fps);

/**
 * @brief Restart a specific camera (stop and start again).
 * @param cam_id Camera ID.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_restart_camera(int cam_id);

/**
 * @brief Switch a specific camera to a new frame rate and apply it immediately.
 *        Convenience wrapper that updates the target fps (as
 *        insight9_receive_set_camera_fps() does) and then stops/reopens
 *        just that one camera (as insight9_receive_restart_camera() does)
 *        so the new value actually takes effect.
 * @param cam_id Camera ID.
 * @param fps    Desired frame rate.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_switch_camera_fps(int cam_id, int fps);

/**
 * @brief Set the resolution/format for a specific camera (stored in config, requires restart to take effect).
 * @param cam_id Camera ID.
 * @param width  Target width.
 * @param height Target height.
 * @param format Target pixel format.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_set_camera_format(int cam_id, int width, int height, PixelFormat format);

/**
 * @brief Immediately switch the specified camera to a new configuration (resolution, format, and frame rate).
 *        This function stops the camera, updates the stored configuration, and restarts the camera with the new settings.
 * @param cam_id Camera ID (0: Depth, 1: Grayscale, 2: RGB on Linux; may vary by platform).
 * @param width  Desired width.
 * @param height Desired height.
 * @param format Desired pixel format.
 * @param fps    Desired frame rate.
 * @return 0 on success, -1 on failure (e.g., invalid parameters, unable to set format/fps, or restart failure).
 */
int insight9_receive_switch_camera_config(int cam_id, int width, int height, PixelFormat format, int fps);

/**
 * @brief Switch a specific camera to a new resolution/format and apply it immediately.
 * @param cam_id Camera ID.
 * @param width  Target width.
 * @param height Target height.
 * @param format Target pixel format.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_switch_camera_format(int cam_id, int width, int height, PixelFormat format);

/**
 * @brief Check if a camera is currently running.
 * @param cam_id Camera ID.
 * @return 1 if running, 0 otherwise.
 */
int insight9_receive_is_camera_running(int cam_id);

/**
 * @brief Stop all capture threads.
 */
void insight9_receive_stop(void);

/**
 * @brief Release all resources. Must be called after stopping.
 */
void insight9_receive_cleanup(void);

/**
 * @brief Register the image callback.
 * @param cb       Callback function.
 * @param userdata User pointer passed through to the callback.
 */
void insight9_receive_register_image_callback(image_callback cb, void *userdata);

/**
 * @brief Register the IMU callback.
 */
void insight9_receive_register_imu_callback(imu_callback cb, void *userdata);

/**
 * @brief Register the VIO callback.
 */
void insight9_receive_register_vio_callback(vio_callback cb, void *userdata);

/**
 * @brief Set the frame rate for a specific camera (stored in config, requires restart).
 * @param cam_id Camera ID.
 * @param fps Desired frame rate.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_set_camera_fps(int cam_id, int fps);

/**
 * @brief Set the currently active camera.
 * @param cam_id Camera ID (0: RGB, 1: stereo grayscale).
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_set_active_camera(int cam_id);

/**
 * @brief Read the currently active camera.
 * @param cam_id Output parameter that receives the active camera ID.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_get_active_camera(int *cam_id);

/**
 * @brief Check if the provided camera parameters are valid.
 * @param params Pointer to camera_params structure to validate.
 * @return 0 if valid, -1 if invalid.
 */
int insight9_check_camera_params(const camera_params *params);

/**
 * @brief Set all parameters for the currently active camera.
 * @param params Parameters to set. Must include the cam_id field indicating the target camera.
 * @return 0 on success, -1 on failure.
 * @note params->cam_id is ignored; the currently active camera ID is used. Prefer set_camera_params_for to specify a camera ID.
 */
int insight9_receive_set_camera_params(const camera_params *params);

/**
 * @brief Read all parameters for the currently active camera.
 * @param params Output parameter that receives the read parameters.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_get_camera_params(camera_params *params);

/**
 * @brief Set all parameters for the specified camera.
 * @param cam_id Camera ID (0/1/2).
 * @param params Parameters to set. params->cam_id is ignored; cam_id is used.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_set_camera_params_for(int cam_id, const camera_params *params);

/**
 * @brief Read all parameters for the specified camera.
 * @param cam_id Camera ID.
 * @param params Output parameter that receives the read parameters.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_get_camera_params_for(int cam_id, camera_params *params);

/**
 * @brief Get the cached initial parameters for the specified camera.
 * @param cam_id Camera ID.
 * @param params Output parameter that receives the cached initial parameters.
 * @return 0 on success, -1 on failure (e.g., no cached parameters available).
 */
int insight9_receive_get_cached_initial_params(int cam_id, camera_params* params);

/**
 * @brief Restore the specified camera to its initial values.
 * @param cam_id Camera ID.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_reset_camera_params(int cam_id);

/**
 * @brief Print camera parameters to stdout
 * @param params Pointer to camera_params
 */
void insight9_receive_print_camera_params(const camera_params *params);

/**
 * @brief Read the intrinsics and extrinsics of the specified camera.
 * @param cam_idx Camera index: INSIGHT9_CALIB_CAM_LEFT / RIGHT / RGB.
 * @param calib   Output parameter that receives the calibration data.
 * @return 0 on success, -1 on failure (e.g. firmware without calibration support).
 */
int insight9_receive_get_camera_calib(int cam_idx, camera_calib *calib);

/**
 * @brief Print camera calibration data to stdout.
 * @param calib Pointer to camera_calib.
 */
void insight9_receive_print_camera_calib(const camera_calib *calib);

/**
 * @brief Align a depth image (registered to the LEFT camera) onto the RGB image
 *        plane, producing a depth map registered to (and the same size as) the
 *        RGB image.
 *
 * The depth stream is the left grayscale camera's depth map, so each depth pixel
 * is deprojected with the LEFT intrinsics into the left camera frame, transformed
 * into the RGB frame, then reprojected with the RGB intrinsics. The RGB camera's
 * extrinsic (parent=camera_camera_left, child=camera_camera_rgb) maps a point
 * rgb->left, so its inverse is used for left->rgb. Both LEFT and RGB streams are
 * rectified (p == k), so no lens distortion is applied. The result is a forward
 * warp with z-buffering (nearest surface wins); RGB pixels with no corresponding
 * depth sample remain 0 (invalid).
 *
 * @param depth       Input depth buffer (uint16, row-major), 1 unit = 1 mm.
 * @param depth_w     Depth width  (valid rows only; exclude the metadata rows).
 * @param depth_h     Depth height (valid rows only).
 * @param left_calib  LEFT camera calibration (its intrinsics deproject depth).
 * @param rgb_calib   RGB camera calibration (intrinsics + left->rgb extrinsic).
 * @param aligned_out Output buffer (uint16, row-major), size rgb_w*rgb_h. The
 *                    caller allocates it; the function clears it to 0 first.
 * @param rgb_w       Output (RGB) width.  Should match rgb_calib->intrinsics.width.
 * @param rgb_h       Output (RGB) height. Should match rgb_calib->intrinsics.height.
 * @return 0 on success, -1 on invalid arguments.
 */
int insight9_receive_align_depth_to_rgb(const uint16_t *depth,
                                        int depth_w, int depth_h,
                                        const camera_calib *left_calib,
                                        const camera_calib *rgb_calib,
                                        uint16_t *aligned_out,
                                        int rgb_w, int rgb_h);

/**
 * @brief Get the current frame rate.
 * @param fps Pointer to store the current frame rate.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_get_current_fps(int* fps);

/**
 * @brief Get the VIO status.
 * @param status Pointer to store the VIO status value.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_get_vio_status(int* status);

/**
 * @brief Get the hardware type/model as a string. This requires reading the current camera parameters to determine the hardware_model field, which is then mapped to a string.
 * @return Hardware type/model string, or "unknown" on failure.
 */
const char* insight9_receive_get_hardware_type(void);

int insight9_receive_switch_camera_fps(int cam_id, int fps);

#ifdef __cplusplus
int insight9_receive_get_device_capability_count(int cam_id, int* count);
int insight9_receive_get_device_capability_by_index(int cam_id, int index, DeviceCapability* cap);
#endif

#ifdef __cplusplus
}
#endif

#endif // INSIGHT_SDK_H
