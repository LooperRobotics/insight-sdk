#ifndef INSIGHT_SDK_H
#define INSIGHT_SDK_H

#include <stdint.h>
#include <stddef.h>

#pragma pack(push, 1)
typedef struct {
    uint8_t cam_id;             // 摄像头ID，0/1
    uint8_t resolution;         // 分辨率索引，RGB:[0-3], "3840x2160", "1088x1920", "1280x720", "640x480", 灰度:[0-1] "1088x1280", "544x640"
    uint8_t frame_rate;         // 帧率索引，0-5分别对应 10/15/20/30/60/90fps
    float exposure_time;        // 曝光时间，单位秒，范围0.0~0.03
    float exposure_gain;        // 曝光增益，范围1.0~16.0
    uint8_t auto_exposure;      // 自动曝光，0或1
    float brightness;           // 亮度，范围0.0~127.0
    float contrast;             // 对比度，范围0.0~1.9
    float gamma_dark;           // Gamma暗部，范围1.0~4.0
    float hue;                  // 色调，范围0.0~87.0
    float saturation;           // 饱和度，范围0.0~1.999
    uint8_t sharpness;          // 锐度，暂未使用
    uint8_t auto_white_balance; // 自动白平衡，0或1
    float white_balance;        // 白平衡，范围1.0~3.0
    uint8_t decimation;         // 采样率, 暂未使用
    uint8_t rotation;           // 旋转，暂未使用
} camera_params;
#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 图像数据回调函数
 * @param cam_id    摄像头ID (0: 主路RGB, 1: 灰度, 2: 深度)
 * @param data      图像数据指针（对于MJPEG为JPEG数据，对于GREY、Z16为原始数据）
 * @param size      数据大小（字节）
 * @param width     图像宽度
 * @param height    图像高度
 * @param format    V4L2像素格式（如 V4L2_PIX_FMT_MJPEG 或 V4L2_PIX_FMT_GREY）
 * @param timestamp 图像时间戳（微秒，由设备提供或系统时间）
 * @param right_timestamp 右图像时间戳（微秒，由设备提供或系统时间，仅在双目摄像头（灰度）时有效）
 * @param userdata  用户注册时传入的指针
 */
typedef void (*image_callback)(int cam_id, uint8_t *data, size_t size,
                               int width, int height, unsigned int format,
                               uint64_t timestamp, uint64_t right_timestamp,
                               void *userdata);

/**
 * @brief IMU数据回调函数
 * @param ax,ay,az  加速度计原始值
 * @param gx,gy,gz  陀螺仪原始值
 * @param timestamp 时间戳（由设备提供）
 * @param userdata  用户指针
 */
typedef void (*imu_callback)(float ax, float ay, float az,
                             float gx, float gy, float gz,
                             uint64_t timestamp, void *userdata);

/**
 * @brief VIO位置数据回调函数
 * @param px,py,pz  位置坐标
 * @param qx,qy,qz,qw 四元数姿态
 * @param seq       序列号
 * @param userdata  用户指针
 */
typedef void (*vio_callback)(float px, float py, float pz,
                             float qx, float qy, float qz, float qw,
                             uint64_t timestamp, void *userdata);

/**
 * @brief 初始化SDK（打开所有设备，但不启动采集）
 * @return 0成功，-1失败
 */
int insight9_receive_init(void);

/**
 * @brief 启动所有采集线程
 * @return 0成功，-1失败
 */
int insight9_receive_start(void);

/**
 * @brief 获取指定摄像头的 video 设备路径
 * @param cam_id 摄像头索引 (0..2)
 * @return 设备路径字符串，失败返回 NULL
 */
const char *insight9_receive_get_video_dev(int cam_id);

/**
 * @brief 停止所有采集线程
 */
void insight9_receive_stop(void);

/**
 * @brief 清理所有资源（必须在停止后调用）
 */
void insight9_receive_cleanup(void);

/**
 * @brief 注册图像回调
 * @param cb       回调函数
 * @param userdata 用户自定义指针（将透传给回调）
 */
void insight9_receive_register_image_callback(image_callback cb, void *userdata);

/**
 * @brief 注册IMU回调
 */
void insight9_receive_register_imu_callback(imu_callback cb, void *userdata);

/**
 * @brief 注册VIO回调
 */
void insight9_receive_register_vio_callback(vio_callback cb, void *userdata);

/**
 * @brief 设置当前激活的摄像头
 * @param cam_id 摄像头ID (0: RGB, 1: 双目灰度)
 * @return 0成功，-1失败
 */
int insight9_receive_set_active_camera(int cam_id);

/**
 * @brief 读取当前激活的摄像头
 * @param cam_id 输出参数，存放当前激活的摄像头ID
 * @return 0成功，-1失败
 */
int insight9_receive_get_active_camera(int *cam_id);

/**
 * @brief 设置当前激活摄像头的全部参数
 * @param params 要设置的参数（必须包含 cam_id 字段，指示目标摄像头）
 * @return 0成功，-1失败
 * @note params->cam_id 将被忽略，实际使用当前激活的摄像头ID进行设置。建议使用 set_camera_params_for 指定摄像头ID。
 */
int insight9_receive_set_camera_params(const camera_params *params);

/**
 * @brief 读取当前激活摄像头的全部参数
 * @param params 输出参数，存放读取到的参数
 * @return 0成功，-1失败
 */
int insight9_receive_get_camera_params(camera_params *params);

/**
 * @brief 设置指定摄像头的全部参数
 * @param cam_id 摄像头ID (0/1/2)
 * @param params 要设置的参数（忽略 params->cam_id，以 cam_id 为准）
 * @return 0成功，-1失败
 */
int insight9_receive_set_camera_params_for(int cam_id, const camera_params *params);

/**
 * @brief 读取指定摄像头的全部参数
 * @param cam_id 摄像头ID
 * @param params 输出参数，存放读取到的参数
 * @return 0成功，-1失败
 */
int insight9_receive_get_camera_params_for(int cam_id, camera_params *params);

/**
 * @brief 将指定摄像头恢复为初始值（设备出厂默认或上次读取的初始值）
 * @param cam_id 摄像头ID
 * @return 0成功，-1失败
 */
int insight9_receive_reset_camera_params(int cam_id);

/**
 * @brief Print camera parameters to stdout
 * @param params Pointer to camera_params
 */
void insight9_receive_print_camera_params(const camera_params *params);

#ifdef __cplusplus
}
#endif

#endif // INSIGHT_SDK_H