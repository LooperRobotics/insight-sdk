# Insight 9 SDK 使用说明

## 1. 概述
Insight 9 SDK 是一个用于采集三路 UVC 摄像头（主路 MJPEG，左右副路 Mono8）和两路 HID 设备（IMU、VIO）数据的动态库。SDK 自动识别指定 VID/PID 的设备节点，按设备号排序后选取合适的设备，并通过回调函数将图像和传感器数据传递给上层应用。

## 2. 功能特性
自动设备发现：根据 VID=0x1d6b、PID=0x0104 自动查找 UVC 和 HID 设备。

智能选择：

UVC：将所有匹配的 video 节点按数字排序，选择索引 0、2、4 作为三个摄像头（若不足 6 个则初始化失败）。

HID：将匹配的 hidraw 节点按数字排序，取前两个，数字小的作为 IMU，大的作为 VIO。

多线程采集：为每个设备创建独立线程，互不干扰。

热插拔支持：设备断开后自动重连，恢复数据流。

回调机制：图像、IMU、VIO 数据通过回调函数实时通知应用层。

简单易用：仅需少量 API 调用即可启动/停止采集。

## 3. 依赖
操作系统：Linux（支持 sysfs、V4L2、HIDRAW）

编译环境：gcc、make、pthread 库

运行时：/sys/class/video4linux 和 /sys/class/hidraw 可访问，设备节点 /dev/video*、/dev/hidraw* 有读写权限。

## 4. 编译与安装
### 4.1 编译动态库
将 Insight_9_receive.c 和 Insight_9_receive.h 放在同一目录，执行：

bash
gcc -c -fPIC Insight_9_receive.c -o Insight_9_receive.o
gcc -shared -o libinsight9.so Insight_9_receive.o -lpthread
生成 libinsight9.so。

### 4.2 安装
将 libinsight9.so 和 Insight_9_receive.h 复制到系统目录或项目目录，例如：

bash
sudo cp libinsight9.so /usr/local/lib/
sudo cp Insight_9_receive.h /usr/local/include/
sudo ldconfig

## 5. API 参考
### 5.1 头文件
c
#include "Insight_9_receive.h"
### 5.2 回调函数类型定义
图像回调
c
typedef void (*image_callback)(int cam_id, uint8_t *data, size_t size,
                               int width, int height, unsigned int format,
                               uint64_t timestamp, void *userdata);
cam_id: 0=主路RGB，1=左路灰度，2=右路灰度

data: 图像数据（MJPEG 或原始灰度）

size: 数据大小（字节）

width, height: 图像尺寸

format: V4L2 像素格式（V4L2_PIX_FMT_MJPEG 或 V4L2_PIX_FMT_GREY）

timestamp: 从设备提取的时间戳（微秒级）

userdata: 注册时传入的用户指针

IMU 回调
c
typedef void (*imu_callback)(int16_t ax, int16_t ay, int16_t az,
                             int16_t gx, int16_t gy, int16_t gz,
                             uint32_t timestamp, void *userdata);
ax,ay,az: 加速度计原始值

gx,gy,gz: 陀螺仪原始值

timestamp: 设备时间戳

VIO 回调
c
typedef void (*vio_callback)(float px, float py, float pz,
                             float qx, float qy, float qz, float qw,
                             uint32_t seq, void *userdata);
px,py,pz: 位置坐标

qx,qy,qz,qw: 四元数姿态

seq: 序列号

### 5.3 SDK 控制 API
int sdk_init(void);
初始化 SDK，扫描并选择设备，分配资源。

返回值：0 成功，-1 失败（设备不足或权限问题）。

int sdk_start(void);
启动所有采集线程。

返回值：0 成功，-1 失败（未初始化或已在运行）。

void sdk_stop(void);
停止所有采集线程（阻塞等待线程退出）。

void sdk_cleanup(void);
释放所有资源（必须在停止后调用）。

### 5.4 回调注册 API
void sdk_register_image_callback(image_callback cb, void *userdata);
注册图像回调。必须在 sdk_start() 之前调用。

void sdk_register_imu_callback(imu_callback cb, void *userdata);
注册 IMU 回调。

void sdk_register_vio_callback(vio_callback cb, void *userdata);
注册 VIO 回调。

## 6. 使用示例
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
    // 如需保存数据，请在此处拷贝 data 内容
}

void imu_callback(int16_t ax, int16_t ay, int16_t az,
                  int16_t gx, int16_t gy, int16_t gz,
                  uint32_t timestamp, void *userdata) {
    printf("IMU: acc=(%d,%d,%d) gyro=(%d,%d,%d) ts=%u\n",
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

    if (sdk_init() != 0) {
        fprintf(stderr, "SDK init failed\n");
        return -1;
    }

    sdk_register_image_callback(image_callback, NULL);
    sdk_register_imu_callback(imu_callback, NULL);
    sdk_register_vio_callback(vio_callback, NULL);

    if (sdk_start() != 0) {
        fprintf(stderr, "SDK start failed\n");
        sdk_cleanup();
        return -1;
    }

    printf("SDK running, press Ctrl+C to stop...\n");
    while (keep_running) {
        sleep(1);
    }

    sdk_stop();
    sdk_cleanup();
    return 0;
}
编译示例
bash
gcc -o example example.c -L. -luvchid -lpthread
export LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH
./example

## 7. 注意事项
数据生命周期：图像数据指针 data 指向 SDK 内部缓冲区，回调返回后该缓冲区会被复用。如需长期保存，必须在回调中拷贝数据。

线程安全：回调函数在采集线程中被调用，不应执行阻塞操作（如文件 I/O、长时间计算），否则可能丢帧。

设备权限：运行程序需具备访问 /dev/video* 和 /dev/hidraw* 的权限（通常需要 root 或加入 video、plugdev 组）。

设备数量要求：SDK 需要至少 5 个 UVC 设备（取 0、2、4 三个）和 2 个 HID 设备，否则 sdk_init 将返回 -1。

热插拔：设备断开后，采集线程会自动尝试重连，回调数据会暂时中断，待设备恢复后自动恢复。

错误处理：若 sdk_init 失败，应调用 sdk_cleanup 释放已分配的资源（即使初始化失败，内部也会清理，但外部调用可确保安全）。

## 8. 常见问题
Q: 如何查看设备是否被正确识别？
A: 运行程序时会打印选中的设备节点，如 Selected UVC device 0: /dev/video4。也可在 sdk_init 后查看 g_ctx.video_devs 等内部变量（仅调试用）。

Q: 回调中收到的图像数据是 MJPEG，如何解码？
A: 可使用 libjpeg 或硬件解码器将 MJPEG 转换为 RGB/YUV。灰度图（Mono8）是原始 8 位像素数据，可直接保存为 PGM 或使用 OpenCV 处理。

Q: 如何调整摄像头参数（如曝光、增益）？
A: 当前 SDK 未开放参数设置接口，如有需要可扩展 init_capture 函数，通过 VIDIOC_S_CTRL 设置 V4L2 控制项。

Q: 编译时提示找不到 libinsight9.so？
A: 确保库文件路径在 LD_LIBRARY_PATH 中，或使用 -Wl,-rpath 指定运行时路径，例如 gcc -o example example.c -L. -luvchid -Wl,-rpath=.。

## 9. 更新日志
v1.0 (2025-03): 初始版本，支持三路 UVC + 两路 HID 自动识别与采集。

如需更多帮助或定制功能，请联系开发者。