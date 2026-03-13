#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <unistd.h>
#include <stdint.h>
#include <pthread.h>
#include <signal.h>
#include <dirent.h>
#include <poll.h>          // 用于 poll

// ==================== 三路UVC配置 ====================
#define CAM_NUM 3

// 设备节点
const char *DEVICE_NAMES[CAM_NUM] = {
    "/dev/video4",  // 主路：MJPEG 1920x1080
    "/dev/video6",  // 左副路：Mono8 544x640
    "/dev/video8"   // 右副路：Mono8 544x640
};

#define IMU_DEVICE "/dev/hidraw0"
#define VIO_DEVICE "/dev/hidraw1"
#define INTRINSICS_DEVICE "/dev/hidraw2"

// 主路配置 (MJPEG)
#define MAIN_WIDTH  1920
#define MAIN_HEIGHT 1080
#define MAIN_FORMAT V4L2_PIX_FMT_MJPEG

// 副路配置 (Mono8)
#define SUB_WIDTH   544
#define SUB_HEIGHT  640
#define SUB_FORMAT  V4L2_PIX_FMT_GREY

#define VENDOR_ID  0x1d6b
#define PRODUCT_ID 0x0104

#define MAX_PATH 512
#define MAX_BUF 256
#define MAX_DEVICES 32

#define FRAME_RATE 30
#define BUFFER_COUNT 8
// #define SAVE_FRAMES 10  // 每个摄像头保存10帧

pthread_t uvc_tids[CAM_NUM];
pthread_t hid_tid;
pthread_t vio_tid;
pthread_t intrinsics_tid;

// ==================== 摄像头上下文 ====================
struct cam_ctx {
    int fd;
    struct buffer *buffers;
    int buffer_count;
    pthread_t tid;
    int cam_id;
    int width;
    int height;
    unsigned int format;
    int frame_count;  // 已保存帧数
    char save_prefix[32];  // 保存文件名前缀
};

struct buffer {
    void *start;
    size_t length;
};

struct __attribute__((packed)) imu_hid_report {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    uint32_t timestamp;
};

struct __attribute__((packed)) vio_hid_payload {
    uint32_t sec;
    uint32_t nsec;

    float px;
    float py;
    float pz;

    float qx;
    float qy;
    float qz;
    float qw;

    uint32_t seq;
};

struct __attribute__((packed)) intrinsics_hid_payload_t
{
    uint32_t sec;
    uint32_t nsec;

    char frame_id[32];

    uint32_t height;
    uint32_t width;

    char distortion_model[32];

    float d[4];
    float k[9];
    float r[9];
    float p[12];

    uint32_t binning_x;
    uint32_t binning_y;

    uint32_t roi_x_offset;
    uint32_t roi_y_offset;
    uint32_t roi_height;
    uint32_t roi_width;

    uint8_t roi_do_rectify;
};

struct __attribute__((packed)) camera_calib_hid_t
{
    struct intrinsics_hid_payload_t cam_left;
    struct intrinsics_hid_payload_t cam_right;
    struct intrinsics_hid_payload_t cam_rgb;
    uint8_t padding[45];
};

typedef struct {
    char video_dev[MAX_PATH];        // /dev/videoX
    char usb_path[MAX_PATH];          // USB设备路径
    char product[MAX_BUF];             // 产品名
    char manufacturer[MAX_BUF];        // 制造商
    char serial[MAX_BUF];              // 序列号
    unsigned int vid;                   // 供应商ID
    unsigned int pid;                   // 产品ID
} uvc_device_info_t;

struct cam_ctx cams[CAM_NUM];
volatile int g_running = 1;

// ==================== 信号处理 ====================
void sigint_handler(int sig) {
    printf("\n收到Ctrl+C，正在停止采集...\n");
    g_running = 0;
}

// ==================== 序列号查找工具函数 ====================
// 移除字符串末尾的换行符
void trim_newline(char *str) {
    size_t len = strlen(str);
    if (len > 0 && str[len-1] == '\n') {
        str[len-1] = '\0';
    }
}

// 读取sysfs文件内容
int read_sysfs_file(const char *path, char *buffer, size_t size) {
    FILE *fp = fopen(path, "r");
    if (!fp) return -1;

    if (fgets(buffer, size, fp)) {
        trim_newline(buffer);
        fclose(fp);
        return 0;
    }

    fclose(fp);
    return -1;
}

// 解析USB路径，获取VID/PID和序列号
int parse_usb_device_info(const char *usb_path, uvc_device_info_t *info) {
    char path[MAX_PATH];
    char buffer[MAX_BUF];

    // 读取VID
    snprintf(path, sizeof(path), "%s/idVendor", usb_path);
    if (read_sysfs_file(path, buffer, sizeof(buffer)) == 0) {
        sscanf(buffer, "%x", &info->vid);
    }

    // 读取PID
    snprintf(path, sizeof(path), "%s/idProduct", usb_path);
    if (read_sysfs_file(path, buffer, sizeof(buffer)) == 0) {
        sscanf(buffer, "%x", &info->pid);
    }

    // 读取序列号
    snprintf(path, sizeof(path), "%s/serial", usb_path);
    read_sysfs_file(path, info->serial, sizeof(info->serial));

    // 读取产品名
    snprintf(path, sizeof(path), "%s/product", usb_path);
    read_sysfs_file(path, info->product, sizeof(info->product));

    // 读取制造商
    snprintf(path, sizeof(path), "%s/manufacturer", usb_path);
    read_sysfs_file(path, info->manufacturer, sizeof(info->manufacturer));

    return 0;
}

// 通过video设备找到对应的USB路径
int find_usb_path_from_video(const char *video_sysfs, char *usb_path, size_t usb_path_size) {
    char path[MAX_PATH];
    char target[MAX_PATH];
    char *p;
    struct stat st;

    // 读取设备名称，获取完整的sysfs路径
    snprintf(path, sizeof(path), "%s/device", video_sysfs);
    if (realpath(path, target) == NULL) {
        return -1;
    }

    // 向上查找直到找到包含idVendor的USB设备目录
    strncpy(usb_path, target, usb_path_size - 1);
    usb_path[usb_path_size - 1] = '\0';

    while (strlen(usb_path) > 1) {
        snprintf(path, sizeof(path), "%s/idVendor", usb_path);
        if (access(path, F_OK) == 0) {
            // 找到了USB设备目录
            return 0;
        }

        // 向上移动一级目录
        p = strrchr(usb_path, '/');
        if (p) {
            *p = '\0';
        } else {
            break;
        }
    }

    return -1;
}

// 检查video设备是否为UVC设备
int is_uvc_device(const char *video_dev) {
    int fd;
    struct v4l2_capability cap;

    fd = open(video_dev, O_RDWR);
    if (fd < 0) {
        return 0;
    }

    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
        // 检查是否为视频捕获设备
        if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) {
            close(fd);
            return 1;
        }
    }

    close(fd);
    return 0;
}

// 获取设备驱动信息
void get_driver_info(const char *video_sysfs, char *driver, size_t driver_size) {
    char path[MAX_PATH];
    char target[MAX_PATH];

    snprintf(path, sizeof(path), "%s/device/driver", video_sysfs);
    if (realpath(path, target) != NULL) {
        char *p = strrchr(target, '/');
        if (p) {
            strncpy(driver, p + 1, driver_size - 1);
            driver[driver_size - 1] = '\0';
        }
    }
}

// 扫描所有UVC设备
int scan_uvc_devices(uvc_device_info_t *devices, int max_devices) {
    DIR *dir;
    struct dirent *entry;
    char video_sysfs[MAX_PATH];
    char usb_path[MAX_PATH];
    char driver[MAX_BUF];
    int count = 0;
    int found_target = 0;

    // 遍历video4linux设备
    dir = opendir("/sys/class/video4linux");
    if (!dir) {
        perror("Failed to open /sys/class/video4linux");
        return -1;
    }

    // printf("Scanning UVC devices...\n");
    // printf("========================================\n\n");

    while ((entry = readdir(dir)) != NULL && count < max_devices) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        // 构建video设备sysfs路径
        snprintf(video_sysfs, sizeof(video_sysfs), 
                "/sys/class/video4linux/%s", entry->d_name);

        // 构建设备节点路径
        snprintf(devices[count].video_dev, sizeof(devices[count].video_dev),
                "/dev/%s", entry->d_name);

        // 检查是否为UVC设备
        if (!is_uvc_device(devices[count].video_dev)) {
            continue;
        }

        // 获取驱动信息
        get_driver_info(video_sysfs, driver, sizeof(driver));

        // 查找对应的USB路径
        if (find_usb_path_from_video(video_sysfs, usb_path, sizeof(usb_path)) == 0) {
            strncpy(devices[count].usb_path, usb_path, sizeof(devices[count].usb_path) - 1);
            devices[count].usb_path[sizeof(devices[count].usb_path) - 1] = '\0';

            // 解析USB设备信息
            parse_usb_device_info(usb_path, &devices[count]);

            // printf("UVC Device %d:\n", count + 1);
            // printf("  Video Device: %s\n", devices[count].video_dev);
            // printf("  USB Path: %s\n", devices[count].usb_path);
            // printf("  Driver: %s\n", driver);

            if (devices[count].manufacturer[0]) {
                // printf("  Manufacturer: %s\n", devices[count].manufacturer);
            }
            if (devices[count].product[0]) {
                // printf("  Product: %s\n", devices[count].product);
            }
            // printf("  VID: 0x%04x\n", devices[count].vid);
            // printf("  PID: 0x%04x\n", devices[count].pid);

            if (devices[count].serial[0]) {
                // printf("  Serial Number: %s\n", devices[count].serial);
            } else {
                // printf("  Serial Number: Not available\n");
            }
            // printf("\n");

            // 检查是否为目标设备
            if (devices[count].vid == VENDOR_ID && devices[count].pid == PRODUCT_ID) {
                found_target = 1;
                // printf("*** Found target device! ***\n");
                // printf("  Serial Number: %s\n\n", devices[count].serial);
            }

            count++;
        }
    }

    closedir(dir);

    if (!found_target && count > 0) {
        // printf("Target device (VID=0x%04x, PID=0x%04x) not found in UVC devices.\n", 
        //        VENDOR_ID, PRODUCT_ID);
    } else if (count == 0) {
        // printf("No UVC devices found.\n");
    }

    return count;
}

// 只查找特定VID/PID的UVC设备
int find_target_uvc_device(uvc_device_info_t *device) {
    DIR *dir;
    struct dirent *entry;
    char video_sysfs[MAX_PATH];
    char usb_path[MAX_PATH];
    uvc_device_info_t temp_info;

    dir = opendir("/sys/class/video4linux");
    if (!dir) return -1;

    while ((entry = readdir(dir)) != NULL) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        snprintf(video_sysfs, sizeof(video_sysfs), 
                "/sys/class/video4linux/%s", entry->d_name);

        // 检查是否为UVC设备
        snprintf(device->video_dev, sizeof(device->video_dev), "/dev/%s", entry->d_name);
        if (!is_uvc_device(device->video_dev)) {
            continue;
        }

        // 查找USB路径
        if (find_usb_path_from_video(video_sysfs, usb_path, sizeof(usb_path)) == 0) {
            strncpy(device->usb_path, usb_path, sizeof(device->usb_path) - 1);
            parse_usb_device_info(usb_path, device);

            // 检查VID/PID
            if (device->vid == VENDOR_ID && device->pid == PRODUCT_ID) {
                closedir(dir);
                return 0;
            }
        }
    }

    closedir(dir);
    return -1;
}

void show_dev_info() {
    uvc_device_info_t devices[MAX_DEVICES];
    int count = scan_uvc_devices(devices, MAX_DEVICES);
    int found = 0;

    printf("========================================\n\n");
    printf("Target: VID=0x%04x, PID=0x%04x\n\n", VENDOR_ID, PRODUCT_ID);
    printf("========================================\n\n");
    for (int i = 0; i < count; i++) {
        if (devices[i].vid == VENDOR_ID && devices[i].pid == PRODUCT_ID) {
            if (!found) {
                printf("\nTarget devices found:\n");
                printf("========================================\n\n");
                found = 1;
            }
            printf("  Video Device: %s\n", devices[i].video_dev);
            printf("  Serial: %s\n", devices[i].serial[0] ? devices[i].serial : "N/A");
            printf("  Product: %s\n", devices[i].product);
            printf("  Serial Number: %s\n\n", devices[i].serial[0] ? devices[i].serial : "Not available");
        }
    }
    printf("========================================\n\n");
    if (!found) {
        printf("No target device found.\n");
    }
}

// ==================== V4L2工具函数 ====================

void print_camera_info(int fd) {
    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
        perror("VIDIOC_QUERYCAP failed");
        return;
    }
    
    printf("  驱动: %s\n", cap.driver);
    printf("  卡: %s\n", cap.card);
    printf("  总线: %s\n", cap.bus_info);
}

int set_framerate(int fd, int framerate) {
    struct v4l2_streamparm parm;
    memset(&parm, 0, sizeof(parm));
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    if (ioctl(fd, VIDIOC_G_PARM, &parm) < 0) {
        perror("VIDIOC_G_PARM failed");
        return -1;
    }
    
    if (!(parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)) {
        printf("  设备不支持设置帧率\n");
        return 0;
    }
    
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = framerate;
    
    if (ioctl(fd, VIDIOC_S_PARM, &parm) < 0) {
        perror("VIDIOC_S_PARM failed");
        return -1;
    }
    
    printf("  设置帧率: %d fps\n", framerate);
    return 0;
}

int init_capture(struct cam_ctx *ctx) {
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = ctx->width;
    fmt.fmt.pix.height = ctx->height;
    fmt.fmt.pix.pixelformat = ctx->format;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    
    printf("[CAM%d] 设置格式: %dx%d, format=0x%x\n", 
           ctx->cam_id, ctx->width, ctx->height, ctx->format);
    
    if (ioctl(ctx->fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("VIDIOC_S_FMT failed");
        return -1;
    }
    
    printf("[CAM%d] 设置后: %dx%d\n", ctx->cam_id, 
           fmt.fmt.pix.width, fmt.fmt.pix.height);
    
    // 设置帧率
    set_framerate(ctx->fd, FRAME_RATE);
    
    // 请求缓冲区
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    
    if (ioctl(ctx->fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("VIDIOC_REQBUFS failed");
        return -1;
    }
    
    if (req.count < 2) {
        printf("[CAM%d] 缓冲区不足: %d\n", ctx->cam_id, req.count);
        return -1;
    }
    
    ctx->buffers = calloc(req.count, sizeof(struct buffer));
    if (!ctx->buffers) {
        perror("calloc failed");
        return -1;
    }
    ctx->buffer_count = req.count;
    
    // 映射缓冲区
    for (int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        
        if (ioctl(ctx->fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("VIDIOC_QUERYBUF failed");
            return -1;
        }
        
        ctx->buffers[i].length = buf.length;
        ctx->buffers[i].start = mmap(NULL, buf.length,
                                     PROT_READ | PROT_WRITE,
                                     MAP_SHARED, ctx->fd, buf.m.offset);
        
        if (ctx->buffers[i].start == MAP_FAILED) {
            perror("mmap failed");
            return -1;
        }
        
        printf("[CAM%d] 缓冲区 %d: %p, %zu bytes\n", 
               ctx->cam_id, i, ctx->buffers[i].start, ctx->buffers[i].length);
        
        // 入队
        if (ioctl(ctx->fd, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF failed");
            return -1;
        }
    }
    
    return 0;
}

int start_capture(int fd) {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("VIDIOC_STREAMON failed");
        return -1;
    }
    return 0;
}

int stop_capture(int fd) {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
        perror("VIDIOC_STREAMOFF failed");
        return -1;
    }
    return 0;
}

// // ==================== 帧保存函数 ====================

/**
 * 保存MJPEG帧
 */
void save_mjpeg_frame(struct cam_ctx *ctx, void *data, size_t size, int seq) {
    char filename[256];
    snprintf(filename, sizeof(filename), "%s_frame_%03d.jpg", 
             ctx->save_prefix, seq);
    
    FILE *fp = fopen(filename, "wb");
    if (!fp) {
        perror("fopen failed");
        return;
    }
    
    fwrite(data, 1, size, fp);
    fclose(fp);
    printf("[CAM%d] 已保存: %s (%zu bytes)\n", ctx->cam_id, filename, size);
}

/**
 * 保存Mono8帧为RAW格式
 */
void save_mono8_frame(struct cam_ctx *ctx, void *data, size_t size, int seq) {
    char filename[256];
    snprintf(filename, sizeof(filename), "%s_frame_%03d.raw", 
             ctx->save_prefix, seq);
    
    FILE *fp = fopen(filename, "wb");
    if (!fp) {
        perror("fopen failed");
        return;
    }
    
    fwrite(data, 1, size, fp);
    fclose(fp);
    printf("[CAM%d] 已保存: %s (%zu bytes)\n", ctx->cam_id, filename, size);
    
    // 同时保存PGM格式方便查看
    snprintf(filename, sizeof(filename), "%s_frame_%03d.pgm", 
             ctx->save_prefix, seq);
    
    fp = fopen(filename, "wb");
    if (fp) {
        fprintf(fp, "P5\n%d %d\n255\n", ctx->width, ctx->height);
        fwrite(data, 1, size, fp);
        fclose(fp);
    }
}

// ==================== 采集线程 ====================
void *capture_thread(void *arg) {
    struct cam_ctx *ctx = (struct cam_ctx *)arg;
    unsigned long frame_count = 0;
    uint64_t RGB_timestamps = 0, BW_L_timestamps = 0, BW_R_timestamps = 0;
    int ret;

    printf("[CAM%d] 采集线程启动，持续接收...\n", ctx->cam_id);

    while (g_running) {
        // 如果文件描述符无效，尝试重新打开设备
        if (ctx->fd < 0) {
            printf("[CAM%d] 尝试重新打开设备 %s\n", ctx->cam_id, DEVICE_NAMES[ctx->cam_id]);
            ctx->fd = open(DEVICE_NAMES[ctx->cam_id], O_RDWR);
            if (ctx->fd < 0) {
                printf("[CAM%d] 打开失败: %s，等待重试...\n", ctx->cam_id, strerror(errno));
                sleep(1);
                continue;
            }
            // 重新初始化设备
            if (init_capture(ctx) < 0) {
                printf("[CAM%d] 初始化失败，关闭设备\n", ctx->cam_id);
                close(ctx->fd);
                ctx->fd = -1;
                sleep(1);
                continue;
            }
            if (start_capture(ctx->fd) < 0) {
                printf("[CAM%d] 启动流失败\n", ctx->cam_id);
                close(ctx->fd);
                ctx->fd = -1;
                sleep(1);
                continue;
            }
            printf("[CAM%d] 设备重新初始化成功\n", ctx->cam_id);
            frame_count = 0; // 重置帧计数
        }

        // 使用 select 等待数据
        fd_set fds;
        struct timeval tv;
        FD_ZERO(&fds);
        FD_SET(ctx->fd, &fds);
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        ret = select(ctx->fd + 1, &fds, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            perror("select failed");
            // 可能是 fd 无效，关闭后重试
            close(ctx->fd);
            ctx->fd = -1;
            continue;
        }
        if (ret == 0) continue;  // 超时，继续检查 g_running

        // 出队
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(ctx->fd, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == ENODEV) {
                printf("[CAM%d] 设备已移除，关闭并等待重新连接\n", ctx->cam_id);
                close(ctx->fd);
                ctx->fd = -1;
                // 清理缓冲区？实际上 close 后 buffers 需要重新映射
                // 这里简单将 buffers 标记为无效，init_capture 会重新分配
                if (ctx->buffers) {
                    for (int j = 0; j < ctx->buffer_count; j++) {
                        if (ctx->buffers[j].start) {
                            munmap(ctx->buffers[j].start, ctx->buffers[j].length);
                        }
                    }
                    free(ctx->buffers);
                    ctx->buffers = NULL;
                }
                ctx->buffer_count = 0;
                continue;
            } else {
                perror("VIDIOC_DQBUF failed");
                // 其他错误，继续循环
                continue;
            }
        }

        //根据格式保存帧
        if (ctx->format == V4L2_PIX_FMT_MJPEG) {
            uint8_t *p = ctx->buffers[buf.index].start;
            // if (p[0] != 0xFF || p[1] != 0xD8) {
            //     goto error;
            // }
            p += 2;
            if (p[0] != 0xFF) break;  // 非标记，应是图像数据开始
            uint8_t marker = p[1];
            if (marker == 0xE1) {     // APP1
                uint16_t len = (p[2] << 8) | p[3];
                // if (p + len + 2 > ctx->buffers[buf.index].start + buf.bytesused) break;
                if (len >= 4+8 && memcmp(p+4, "TS__", 4) == 0) {
                    memcpy(&RGB_timestamps, p+8, sizeof(RGB_timestamps));
                }
                printf("MJPEG time-stamp: %ld\n", RGB_timestamps);
            }
            save_mjpeg_frame(ctx, ctx->buffers[buf.index].start, 
                            buf.bytesused, ctx->frame_count);
        } else if (ctx->format == V4L2_PIX_FMT_GREY) {
            if(ctx->cam_id == 1) {
                memcpy(&BW_L_timestamps, ctx->buffers[buf.index].start + ctx->width * ctx->height, sizeof(BW_L_timestamps));
                printf("BW L time-stamp: %ld\n", BW_L_timestamps);
            } else if (ctx->cam_id == 2) {
                memcpy(&BW_R_timestamps, ctx->buffers[buf.index].start + ctx->width * ctx->height, sizeof(BW_R_timestamps));
                printf("BW R time-stamp: %ld\n", BW_R_timestamps);
            }
            save_mono8_frame(ctx, ctx->buffers[buf.index].start,
                            ctx->width * ctx->height, ctx->frame_count);
        }

        ctx->frame_count++;
        // 可选打印
        // if (frame_count % 30 == 0) {
        //    printf("[CAM%d] 收到帧 #%lu, 大小=%u bytes\n",
        //           ctx->cam_id, frame_count, buf.bytesused);
        //}
        frame_count++;

        // 重新入队
        if (ioctl(ctx->fd, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF failed");
            // 错误处理，比如关闭设备
            close(ctx->fd);
            ctx->fd = -1;
            continue;
        }
    }

    printf("[CAM%d] 采集线程退出，共接收 %lu 帧\n", ctx->cam_id, frame_count);
    return NULL;
}

void *hid_receive_thread(void *arg) {
    const char *device = IMU_DEVICE;
    struct imu_hid_report rpt;
    int fd = -1;
    struct pollfd pfd;
    int ret;
    
    printf("HID 接收线程启动，设备 %s\n", device);
    
    while (g_running) {
        // 如果设备未打开，尝试打开
        if (fd < 0) {
            fd = open(device, O_RDONLY | O_NONBLOCK);  // 非阻塞模式
            if (fd < 0) {
                if (errno == ENOENT) {
                    printf("等待 HID 设备 %s 出现...\n", device);
                    sleep(1);
                    continue;
                } else {
                    perror("open HID device");
                    sleep(1);
                    continue;
                }
            }
            printf("成功打开 HID 设备 %s\n", device);
        }
        
        pfd.fd = fd;
        pfd.events = POLLIN;
        // 使用 poll 等待数据，超时 500ms，便于检查 g_running
        ret = poll(&pfd, 1, 500);
        if (ret < 0) {
            if (errno == EINTR) continue;
            perror("poll failed");
            close(fd);
            fd = -1;
            continue;
        }
        if (ret == 0) continue;  // 超时，继续循环
      
        // 有数据可读
        int n = read(fd, &rpt, sizeof(rpt));
        if (n == sizeof(rpt)) {
           printf("IMU: ACC=%6d %6d %6d GYR=%6d %6d %6d TS=%u\n",
                  rpt.ax, rpt.ay, rpt.az,
                  rpt.gx, rpt.gy, rpt.gz,
                  rpt.timestamp);
        } else if (n < 0) {
            // 读取出错，可能是设备移除
            perror("read HID error");
            close(fd);
            fd = -1;
            sleep(1);
        } else {
            // 数据不完整，忽略
            printf("HID 收到不完整数据 %d 字节\n", n);
        }
    }
   
    if (fd >= 0) close(fd);
    printf("HID 接收线程退出\n");
    return NULL;
}

void *vio_receive_thread(void *arg)
{
    const char *device = "/dev/hidraw1";
    struct vio_hid_payload payload;

    int fd = -1;
    struct pollfd pfd;
    int ret;

    printf("VIO 接收线程启动，设备 %s\n", device);

    while (g_running) {

        if (fd < 0) {
            fd = open(device, O_RDONLY | O_NONBLOCK);
            if (fd < 0) {
                printf("等待 VIO 设备...\n");
                sleep(1);
                continue;
            }
            printf("成功打开 VIO 设备 %s\n", device);
        }

        pfd.fd = fd;
        pfd.events = POLLIN;

        ret = poll(&pfd, 1, 500);
        if (ret <= 0)
            continue;

        int n = read(fd, &payload, sizeof(payload));

        if (n == sizeof(payload)) {

 //           printf("VIO: POS=(%.4f %.4f %.4f) "
 //                  "Q=(%.4f %.4f %.4f %.4f) "
 //                  "SEQ=%u\n",
 //                  payload.px,
 //                  payload.py,
 //                  payload.pz,
 //                  payload.qx,
 //                  payload.qy,
 //                  payload.qz,
 //                  payload.qw,
 //                  payload.seq);

        } else if (n < 0) {
            perror("VIO read error");
            close(fd);
            fd = -1;
        }
    }

    if (fd >= 0) close(fd);

    printf("VIO 接收线程退出\n");
    return NULL;
}

// void *intrinsics_receive_thread(void *arg)
// {
//     const char *device = INTRINSICS_DEVICE;

//     struct camera_calib_hid_t calib;
//     int fd = -1;
//     struct pollfd pfd;

//     printf("Intrinsics 接收线程启动，设备 %s\n", device);

//     while (g_running)
//     {
//         if (fd < 0)
//         {
//             fd = open(device, O_RDONLY | O_NONBLOCK);

//             if (fd < 0)
//             {
//                 printf("等待 Intrinsics 设备...\n");
//                 sleep(1);
//                 continue;
//             }

//             printf("成功打开 Intrinsics 设备 %s\n", device);
//         }

//         pfd.fd = fd;
//         pfd.events = POLLIN;

//         int ret = poll(&pfd, 1, 500);

//         if (ret <= 0)
//             continue;

//         int n = read(fd, &calib, sizeof(calib));

//         // printf("sizeof camera_calib_hid_t : %lu | n : %u", sizeof(struct camera_calib_hid_t), n);
//         if (n == sizeof(calib))
//         {
//             // printf("\n=== Camera Intrinsics Received ===\n");

//             // printf("LEFT  : %dx%d\n",
//             //         calib.cam_left.width,
//             //         calib.cam_left.height);

//             // printf("RIGHT : %dx%d\n",
//             //         calib.cam_right.width,
//             //         calib.cam_right.height);

//             // printf("RGB   : %dx%d\n",
//             //         calib.cam_rgb.width,
//             //         calib.cam_rgb.height);

//             // printf("RGB fx=%.3f fy=%.3f cx=%.3f cy=%.3f\n",
//             //         calib.cam_rgb.k[0],
//             //         calib.cam_rgb.k[4],
//             //         calib.cam_rgb.k[2],
//             //         calib.cam_rgb.k[5]);
//         }
//         else if (n < 0)
//         {
//             perror("Intrinsics read error");
//             close(fd);
//             fd = -1;
//         }
//     }

//     if (fd >= 0)
//         close(fd);

//     printf("Intrinsics 接收线程退出\n");

//     return NULL;
// }

// ==================== 主函数 ====================

int main() {
    // 注册信号处理
    signal(SIGINT, sigint_handler);
    show_dev_info();
    printf("========================================\n");
    printf("三路UVC采集程序启动\n");
    printf("主路: %s - MJPEG %dx%d\n", DEVICE_NAMES[0], MAIN_WIDTH, MAIN_HEIGHT);
    printf("左路: %s - Mono8 %dx%d\n", DEVICE_NAMES[1], SUB_WIDTH, SUB_HEIGHT);
    printf("右路: %s - Mono8 %dx%d\n", DEVICE_NAMES[2], SUB_WIDTH, SUB_HEIGHT);
    printf("按 Ctrl+C 可提前退出\n");
    printf("========================================\n\n");
    
    // ========== 1. 初始化摄像头 ==========
    for (int i = 0; i < CAM_NUM; i++) {
        memset(&cams[i], 0, sizeof(struct cam_ctx));
        cams[i].cam_id = i;
        
        // 设置分辨率和格式
        if (i == 0) {  // 主路
            cams[i].width = MAIN_WIDTH;
            cams[i].height = MAIN_HEIGHT;
            cams[i].format = MAIN_FORMAT;
            snprintf(cams[i].save_prefix, sizeof(cams[i].save_prefix), 
                     "cam0_main");
        } else if (i == 1) {  // 左路
            cams[i].width = SUB_WIDTH;
            cams[i].height = SUB_HEIGHT;
            cams[i].format = SUB_FORMAT;
            snprintf(cams[i].save_prefix, sizeof(cams[i].save_prefix), 
                     "cam1_left");
        } else {  // 右路
            cams[i].width = SUB_WIDTH;
            cams[i].height = SUB_HEIGHT;
            cams[i].format = SUB_FORMAT;
            snprintf(cams[i].save_prefix, sizeof(cams[i].save_prefix), 
                     "cam2_right");
        }
        
        printf("[CAM%d] 打开设备 %s\n", i, DEVICE_NAMES[i]);
        cams[i].fd = open(DEVICE_NAMES[i], O_RDWR);
        if (cams[i].fd < 0) {
            printf("  打开失败: %s\n", strerror(errno));
            goto cleanup;
        }
        
        print_camera_info(cams[i].fd);
        
        if (init_capture(&cams[i]) < 0) {
            printf("[CAM%d] 初始化失败\n", i);
            goto cleanup;
        }
    }
    
    // ========== 2. 启动视频流 ==========
    for (int i = 0; i < CAM_NUM; i++) {
        printf("[CAM%d] 启动视频流\n", i);
        if (start_capture(cams[i].fd) < 0) {
            printf("[CAM%d] 启动失败\n", i);
            goto cleanup;
        }
        usleep(100000);  // 100ms延时
    }
    
    for (int i = 0; i < CAM_NUM; i++) {
        pthread_create(&uvc_tids[i], NULL, capture_thread, &cams[i]);
    }
    
    // 启动 HID 接收线程
    pthread_create(&hid_tid, NULL, hid_receive_thread, NULL);
    pthread_create(&vio_tid, NULL, vio_receive_thread, NULL);
    // pthread_create(&intrinsics_tid, NULL, intrinsics_receive_thread, NULL);
    
    // 等待所有线程退出
    for (int i = 0; i < CAM_NUM; i++) {
        pthread_join(uvc_tids[i], NULL);
    }
    pthread_join(hid_tid, NULL);
    pthread_join(vio_tid, NULL);
    
    // ... 清理资源（原有 cleanup 部分）...
    return 0;

cleanup:
    // ========== 5. 停止并清理 ==========
    printf("\n正在清理...\n");
    for (int i = 0; i < CAM_NUM; i++) {
        if (cams[i].fd > 0) {
            stop_capture(cams[i].fd);
            
            if (cams[i].buffers) {
                for (int j = 0; j < cams[i].buffer_count; j++) {
                    if (cams[i].buffers[j].start) {
                        munmap(cams[i].buffers[j].start, 
                               cams[i].buffers[j].length);
                    }
                }
                free(cams[i].buffers);
            }
            
            close(cams[i].fd);
        }
    }
    
    printf("程序退出\n");
    return 0;
}
