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
#pragma pack(pop)

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
 * @brief Initialize the SDK by opening all devices without starting capture.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_init(void);

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
 * @brief Restore the specified camera to its initial values, such as factory defaults or previously read initial values.
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
 * @brief Get the hardware type/model as a string. This requires reading the current camera parameters to determine the hardware_model field, which is then mapped to a string.
 * @return Hardware type/model string, or "unknown" on failure.
 */
std::string insight9_receive_get_hardware_type();

#ifdef __cplusplus
}
#endif

#endif // INSIGHT_SDK_H
