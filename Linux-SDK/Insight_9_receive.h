#ifndef INSIGHT_SDK_H
#define INSIGHT_SDK_H

#include <stdint.h>
#include <stddef.h>
#include <string>

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
    NV12
};

typedef struct {
    int width;
    int height;
    int fps;
    unsigned int pixel_format;
} video_config_t;

typedef struct {
    video_config_t rgb_config;
    video_config_t gray_config;
    video_config_t depth_config;
} insight9_config_t;

// VIO status enumeration, matching the device firmware
enum class VioStatus : uint8_t {
    NOT_INITED          = 0,
    TRACKING            = 1,
    TRACKING_LOST       = 2,
    DATA_LOST           = 3,
    TRACKING_STATIC     = 4,
    RELOCALIZATION      = 5,
    MOVING_ENVIRONMENT  = 6,
    Unknown             = 0xFF
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
 * @param right_timestamp Right image timestamp in microseconds; valid only for the stereo grayscale camera.
 * @param userdata  User pointer passed when the callback is registered.
 */
typedef void (*image_callback)(int cam_id, uint8_t *data, size_t size,
                               int width, int height, unsigned int format,
                               uint64_t timestamp, uint64_t right_timestamp,
                               void *userdata);

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
 * @brief Get the video device path for the specified camera.
 * @param cam_id Camera index (0..2).
 * @return Device path string, or NULL on failure.
 */
const char *insight9_receive_get_video_dev(int cam_id);

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
 * @brief Restart a specific camera (stop and start again).
 * @param cam_id Camera ID.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_restart_camera(int cam_id);

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

#ifdef __cplusplus
}
#endif

#endif // INSIGHT_SDK_H
