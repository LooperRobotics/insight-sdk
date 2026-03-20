#ifndef UVC_HID_SDK_H
#define UVC_HID_SDK_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 图像数据回调函数
 * @param cam_id    摄像头ID (0: 主路RGB, 1: 左路灰度, 2: 右路灰度)
 * @param data      图像数据指针（对于MJPEG为JPEG数据，对于GREY为原始灰度数据）
 * @param size      数据大小（字节）
 * @param width     图像宽度
 * @param height    图像高度
 * @param format    V4L2像素格式（如 V4L2_PIX_FMT_MJPEG 或 V4L2_PIX_FMT_GREY）
 * @param timestamp 图像时间戳（微秒，由设备提供或系统时间）
 * @param userdata  用户注册时传入的指针
 */
typedef void (*image_callback)(int cam_id, uint8_t *data, size_t size,
                               int width, int height, unsigned int format,
                               uint64_t timestamp, void *userdata);

/**
 * @brief IMU数据回调函数
 * @param ax,ay,az  加速度计原始值
 * @param gx,gy,gz  陀螺仪原始值
 * @param timestamp 时间戳（由设备提供）
 * @param userdata  用户指针
 */
typedef void (*imu_callback)(int16_t ax, int16_t ay, int16_t az,
                             int16_t gx, int16_t gy, int16_t gz,
                             uint32_t timestamp, void *userdata);

/**
 * @brief VIO位置数据回调函数
 * @param px,py,pz  位置坐标
 * @param qx,qy,qz,qw 四元数姿态
 * @param seq       序列号
 * @param userdata  用户指针
 */
typedef void (*vio_callback)(float px, float py, float pz,
                             float qx, float qy, float qz, float qw,
                             uint32_t seq, void *userdata);

/**
 * @brief 初始化SDK（打开所有设备，但不启动采集）
 * @return 0成功，-1失败
 */
int sdk_init(void);

/**
 * @brief 启动所有采集线程
 * @return 0成功，-1失败
 */
int sdk_start(void);

/**
 * @brief 停止所有采集线程
 */
void sdk_stop(void);

/**
 * @brief 清理所有资源（必须在停止后调用）
 */
void sdk_cleanup(void);

/**
 * @brief 注册图像回调
 * @param cb       回调函数
 * @param userdata 用户自定义指针（将透传给回调）
 */
void sdk_register_image_callback(image_callback cb, void *userdata);

/**
 * @brief 注册IMU回调
 */
void sdk_register_imu_callback(imu_callback cb, void *userdata);

/**
 * @brief 注册VIO回调
 */
void sdk_register_vio_callback(vio_callback cb, void *userdata);

#ifdef __cplusplus
}
#endif

#endif // UVC_HID_SDK_H