#ifndef INSIGHT_SDK_H
#define INSIGHT_SDK_H

#include <stdint.h>
#include <stddef.h>
#include <string>

#pragma pack(push, 1)
typedef struct {
    uint8_t cam_id;             // Camera ID, 0/1
    uint8_t resolution;         // Resolution index, RGB:[0-3], grayscale:[0-1]
    uint8_t frame_rate;         // Frame-rate index, 0-5 map to 10/15/20/30/60/90 fps
    float exposure_time;        // Exposure time in seconds, range 0.0~0.03
    float exposure_gain;        // Exposure gain, range 1.0~16.0
    uint8_t auto_exposure;      // Auto exposure, 0 or 1
    float brightness;           // Brightness, range 0.0~127.0
    float contrast;             // Contrast, range 0.0~1.9
    float gamma_dark;           // Dark gamma, range 1.0~4.0
    float hue;                  // Hue, range 0.0~87.0
    float saturation;           // Saturation, range 0.0~1.999
    uint8_t sharpness;          // Sharpness, 1~255
    uint8_t auto_white_balance; // Auto white balance, 0 or 1
    float white_balance;        // White balance, range 1.0~3.0
    uint8_t decimation;         // Decimation, 1~255
    uint8_t hardware_model;     // Hardware model, 0..3
} camera_params;
#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*image_callback)(int cam_id, uint8_t *data, size_t size,
                               int width, int height, unsigned int format,
                               uint64_t timestamp, uint64_t right_timestamp,
                               void *userdata);

typedef void (*imu_callback)(float ax, float ay, float az,
                             float gx, float gy, float gz,
                             uint64_t timestamp, void *userdata);

typedef void (*vio_callback)(float px, float py, float pz,
                             float qx, float qy, float qz, float qw,
                             uint64_t timestamp, void *userdata);

int insight9_receive_init(void);
int insight9_receive_start(void);
const char *insight9_receive_get_video_dev(int cam_id);
const char *insight9_receive_get_metadata_dev(int cam_id);
int insight9_receive_read_metadata_timestamp(int cam_id, uint64_t *timestamp);
void insight9_receive_stop(void);
void insight9_receive_cleanup(void);
void insight9_receive_register_image_callback(image_callback cb, void *userdata);
void insight9_receive_register_imu_callback(imu_callback cb, void *userdata);
void insight9_receive_register_vio_callback(vio_callback cb, void *userdata);
int insight9_receive_set_active_camera(int cam_id);
int insight9_receive_get_active_camera(int *cam_id);
int insight9_receive_set_camera_params(const camera_params *params);
int insight9_receive_get_camera_params(camera_params *params);
int insight9_receive_set_camera_params_for(int cam_id, const camera_params *params);
int insight9_receive_get_camera_params_for(int cam_id, camera_params *params);
int insight9_receive_reset_camera_params(int cam_id);
void insight9_receive_print_camera_params(const camera_params *params);
const char* insight9_receive_get_hardware_type();

#ifdef __cplusplus
}
#endif

#endif // INSIGHT_SDK_H