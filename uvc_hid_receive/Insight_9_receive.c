#include "Insight_9_receive.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <linux/videodev2.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <dirent.h>
#include <poll.h>
#include <assert.h>
#include <ctype.h>

// ==================== 目标设备 VID/PID ====================
#define VENDOR_ID  0x1d6b
#define PRODUCT_ID 0x0104

// ==================== 摄像头配置 ====================
#define CAM_NUM 3
#define MAIN_WIDTH  1920
#define MAIN_HEIGHT 1080
#define MAIN_FORMAT V4L2_PIX_FMT_MJPEG
#define SUB_WIDTH   544
#define SUB_HEIGHT  640
#define SUB_FORMAT  V4L2_PIX_FMT_GREY
#define FRAME_RATE 30
#define BUFFER_COUNT 8

// ==================== 数据结构 ====================
#define MAX_PATH 1024

struct buffer {
    void *start;
    size_t length;
};

struct cam_ctx {
    int fd;                         // 设备文件描述符
    struct buffer *buffers;         // mmap缓冲区数组
    int buffer_count;               // 缓冲区数量
    pthread_t tid;                  // 采集线程ID
    int cam_id;
    int width;
    int height;
    unsigned int format;
};

// SDK全局上下文
typedef struct {
    // 摄像头
    struct cam_ctx cams[CAM_NUM];
    char video_devs[CAM_NUM][MAX_PATH];   // 动态确定的 video 设备路径
    // HID设备（只使用两个：0=IMU, 1=VIO）
    char hid_devs[2][MAX_PATH];           // 动态确定的 hidraw 设备路径
    pthread_t hid_tids[2];
    // 回调
    image_callback img_cb;
    void *img_userdata;
    imu_callback imu_cb;
    void *imu_userdata;
    vio_callback vio_cb;
    void *vio_userdata;
    // 运行标志
    volatile int running;
    // 初始化标志
    int initialized;
} sdk_ctx_t;

static sdk_ctx_t g_ctx = {0};

// ==================== 工具函数：sysfs 读取、VID/PID 解析等 ====================
static void trim_newline(char *str) {
    size_t len = strlen(str);
    if (len > 0 && str[len-1] == '\n') str[len-1] = '\0';
}

static int read_sysfs_file(const char *path, char *buffer, size_t size) {
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

static int find_usb_path_from_video(const char *video_sysfs, char *usb_path, size_t usb_path_size) {
    char path[MAX_PATH];
    char target[MAX_PATH];
    char *p;
    snprintf(path, sizeof(path), "%s/device", video_sysfs);
    if (realpath(path, target) == NULL) return -1;
    strncpy(usb_path, target, usb_path_size - 1);
    usb_path[usb_path_size - 1] = '\0';
    while (strlen(usb_path) > 1) {
        snprintf(path, sizeof(path), "%s/idVendor", usb_path);
        if (access(path, F_OK) == 0) return 0;
        p = strrchr(usb_path, '/');
        if (p) *p = '\0'; else break;
    }
    return -1;
}

static int parse_usb_vid_pid(const char *usb_path, unsigned int *vid, unsigned int *pid) {
    char path[MAX_PATH];
    char buffer[64];
    *vid = 0;
    *pid = 0;
    snprintf(path, sizeof(path), "%s/idVendor", usb_path);
    if (read_sysfs_file(path, buffer, sizeof(buffer)) == 0)
        sscanf(buffer, "%x", vid);
    snprintf(path, sizeof(path), "%s/idProduct", usb_path);
    if (read_sysfs_file(path, buffer, sizeof(buffer)) == 0)
        sscanf(buffer, "%x", pid);
    return (*vid != 0 && *pid != 0) ? 0 : -1;
}

static int is_uvc_device(const char *video_dev) {
    int fd = open(video_dev, O_RDWR);
    if (fd < 0) return 0;
    struct v4l2_capability cap;
    int ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
    close(fd);
    if (ret == 0 && (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
        return 1;
    return 0;
}

static int get_device_number(const char *dev_path) {
    const char *p = strrchr(dev_path, '/');
    if (!p) p = dev_path;
    while (*p && !isdigit(*p)) p++;
    return atoi(p);
}

static int compare_device_numbers(const void *a, const void *b) {
    const char *sa = *(const char **)a;
    const char *sb = *(const char **)b;
    int na = get_device_number(sa);
    int nb = get_device_number(sb);
    return na - nb;
}

// 查找所有匹配 VID/PID 的 UVC 设备，返回找到的设备路径（/dev/videoX），按数字升序排列
static int find_uvc_devices_by_vid_pid(unsigned int target_vid, unsigned int target_pid,
                                       char dev_paths[][MAX_PATH], int max_devs) {
    DIR *dir = opendir("/sys/class/video4linux");
    if (!dir) return -1;
    struct dirent *entry;
    char video_sysfs[MAX_PATH];
    char usb_path[MAX_PATH];
    unsigned int vid, pid;
    int count = 0;

    while ((entry = readdir(dir)) != NULL && count < max_devs) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;

        snprintf(video_sysfs, sizeof(video_sysfs), "/sys/class/video4linux/%s", entry->d_name);
        snprintf(dev_paths[count], MAX_PATH, "/dev/%s", entry->d_name);

        // 检查是否为 UVC 捕获设备
        if (!is_uvc_device(dev_paths[count]))
            continue;

        if (find_usb_path_from_video(video_sysfs, usb_path, sizeof(usb_path)) == 0) {
            if (parse_usb_vid_pid(usb_path, &vid, &pid) == 0) {
                if (vid == target_vid && pid == target_pid) {
                    count++;
                }
            }
        }
    }
    closedir(dir);

    // 按设备号排序
    if (count > 1) {
        // 临时指针数组用于排序
        const char *ptrs[count];
        for (int i = 0; i < count; i++) ptrs[i] = dev_paths[i];
        qsort(ptrs, count, sizeof(char *), compare_device_numbers);
        // 重新排列
        char tmp[count][MAX_PATH];
        for (int i = 0; i < count; i++) strcpy(tmp[i], ptrs[i]);
        for (int i = 0; i < count; i++) strcpy(dev_paths[i], tmp[i]);
    }
    return count;
}

// 查找所有匹配 VID/PID 的 HID 设备，返回找到的设备路径（/dev/hidrawX），按数字升序排列
static int find_hid_devices_by_vid_pid(unsigned int target_vid, unsigned int target_pid,
                                       char dev_paths[][MAX_PATH], int max_devs) {
    DIR *dir = opendir("/sys/class/hidraw");
    if (!dir) return -1;
    struct dirent *entry;
    char hidraw_sysfs[MAX_PATH];
    char usb_path[MAX_PATH];
    unsigned int vid, pid;
    int count = 0;

    while ((entry = readdir(dir)) != NULL && count < max_devs) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;

        snprintf(hidraw_sysfs, sizeof(hidraw_sysfs), "/sys/class/hidraw/%s", entry->d_name);
        snprintf(dev_paths[count], MAX_PATH, "/dev/%s", entry->d_name);

        // 找到 hidraw 设备对应的 USB 路径
        char device_path[MAX_PATH];
        snprintf(device_path, sizeof(device_path), "%s/device", hidraw_sysfs);
        if (realpath(device_path, usb_path) == NULL) continue;

        // 向上查找 USB 根目录（包含 idVendor）
        char *p;
        while (1) {
            snprintf(device_path, sizeof(device_path), "%s/idVendor", usb_path);
            if (access(device_path, F_OK) == 0) break;
            p = strrchr(usb_path, '/');
            if (!p) break;
            *p = '\0';
        }
        if (access(device_path, F_OK) != 0) continue;

        if (parse_usb_vid_pid(usb_path, &vid, &pid) == 0) {
            if (vid == target_vid && pid == target_pid) {
                count++;
            }
        }
    }
    closedir(dir);

    if (count > 1) {
        const char *ptrs[count];
        for (int i = 0; i < count; i++) ptrs[i] = dev_paths[i];
        qsort(ptrs, count, sizeof(char *), compare_device_numbers);
        char tmp[count][MAX_PATH];
        for (int i = 0; i < count; i++) strcpy(tmp[i], ptrs[i]);
        for (int i = 0; i < count; i++) strcpy(dev_paths[i], tmp[i]);
    }
    return count;
}

// ==================== V4L2 操作函数 ====================
static void print_camera_info(int fd) {
    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
        perror("VIDIOC_QUERYCAP failed");
        return;
    }
    printf("  驱动: %s\n", cap.driver);
    printf("  卡: %s\n", cap.card);
    printf("  总线: %s\n", cap.bus_info);
}

static int set_framerate(int fd, int framerate) {
    struct v4l2_streamparm parm;
    memset(&parm, 0, sizeof(parm));
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_PARM, &parm) < 0) {
        perror("VIDIOC_G_PARM failed");
        return -1;
    }
    if (!(parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)) {
        // 设备不支持设置帧率
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

static int init_capture(struct cam_ctx *ctx) {
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

    set_framerate(ctx->fd, FRAME_RATE);

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
        fprintf(stderr, "Insufficient buffers\n");
        return -1;
    }

    ctx->buffers = calloc(req.count, sizeof(struct buffer));
    if (!ctx->buffers) {
        perror("calloc");
        return -1;
    }
    ctx->buffer_count = req.count;

    for (int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(ctx->fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("VIDIOC_QUERYBUF");
            return -1;
        }

        ctx->buffers[i].length = buf.length;
        ctx->buffers[i].start = mmap(NULL, buf.length,
                                     PROT_READ | PROT_WRITE,
                                     MAP_SHARED, ctx->fd, buf.m.offset);

        if (ctx->buffers[i].start == MAP_FAILED) {
            perror("mmap");
            return -1;
        }

        if (ioctl(ctx->fd, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF");
            return -1;
        }
    }

    return 0;
}

static int start_capture(int fd) {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("VIDIOC_STREAMON");
        return -1;
    }
    return 0;
}

static int stop_capture(int fd) {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
        perror("VIDIOC_STREAMOFF");
        return -1;
    }
    return 0;
}

// ==================== 摄像头采集线程 ====================
static void *capture_thread(void *arg) {
    struct cam_ctx *ctx = (struct cam_ctx *)arg;
    unsigned long frame_count = 0;
    const char *dev_path = g_ctx.video_devs[ctx->cam_id];

    printf("[CAM%d] 采集线程启动，设备 %s\n", ctx->cam_id, dev_path);

    while (g_ctx.running) {
        if (ctx->fd < 0) {
            printf("[CAM%d] 尝试重新打开设备 %s\n", ctx->cam_id, dev_path);
            ctx->fd = open(dev_path, O_RDWR);
            if (ctx->fd < 0) {
                printf("[CAM%d] 打开失败: %s，等待重试...\n", ctx->cam_id, strerror(errno));
                usleep(1000000);
                continue;
            }
            if (init_capture(ctx) < 0) {
                close(ctx->fd);
                ctx->fd = -1;
                usleep(1000000);
                continue;
            }
            if (start_capture(ctx->fd) < 0) {
                printf("[CAM%d] 启动流失败\n", ctx->cam_id);
                close(ctx->fd);
                ctx->fd = -1;
                usleep(1000000);
                continue;
            }
            printf("[CAM%d] 设备重新初始化成功\n", ctx->cam_id);
            frame_count = 0;
        }

        fd_set fds;
        struct timeval tv;
        FD_ZERO(&fds);
        FD_SET(ctx->fd, &fds);
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        int ret = select(ctx->fd + 1, &fds, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            perror("select");
            close(ctx->fd);
            ctx->fd = -1;
            continue;
        }
        if (ret == 0) continue;

        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(ctx->fd, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == ENODEV) {
                printf("[CAM%d] 设备已移除，关闭并等待重新连接\n", ctx->cam_id);
                close(ctx->fd);
                ctx->fd = -1;
                if (ctx->buffers) {
                    for (int i = 0; i < ctx->buffer_count; i++) {
                        if (ctx->buffers[i].start)
                            munmap(ctx->buffers[i].start, ctx->buffers[i].length);
                    }
                    free(ctx->buffers);
                    ctx->buffers = NULL;
                }
                ctx->buffer_count = 0;
                continue;
            }
            perror("VIDIOC_DQBUF");
            continue;
        }

        uint64_t timestamp = 0;
        if (ctx->format == V4L2_PIX_FMT_MJPEG) {
            uint8_t *p = ctx->buffers[buf.index].start;
            if (p[0] == 0xFF && p[1] == 0xD8) {
                p += 2;
                if (p[0] == 0xFF && p[1] == 0xE1) {
                    uint16_t len = (p[2] << 8) | p[3];
                    if (len >= 4+8 && memcmp(p+4, "TS__", 4) == 0) {
                        memcpy(&timestamp, p+8, sizeof(timestamp));
                    }
                }
            }
        } else if (ctx->format == V4L2_PIX_FMT_GREY) {
            if (buf.bytesused >= (ctx->width * ctx->height + 8)) {
                memcpy(&timestamp,
                       (uint8_t*)ctx->buffers[buf.index].start + ctx->width * ctx->height,
                       sizeof(timestamp));
            }
        }

        if (g_ctx.img_cb) {
            g_ctx.img_cb(ctx->cam_id,
                         ctx->buffers[buf.index].start,
                         buf.bytesused,
                         ctx->width, ctx->height,
                         ctx->format,
                         timestamp,
                         g_ctx.img_userdata);
        }

        frame_count++;

        if (ioctl(ctx->fd, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF");
            close(ctx->fd);
            ctx->fd = -1;
        }
    }

    return NULL;
}

// ==================== HID 设备读取 ====================
struct imu_hid_report {
    float ax, ay, az;
    float gx, gy, gz;
    uint32_t timestamp;
};

struct vio_hid_payload {
    uint32_t sec, nsec;
    float px, py, pz;
    float qx, qy, qz, qw;
    uint32_t seq;
};

static void *hid_thread(void *arg) {
    int idx = (int)(intptr_t)arg;
    const char *device = g_ctx.hid_devs[idx];
    int fd = -1;

    printf("HID 线程 %d 启动，设备 %s\n", idx, device);

    while (g_ctx.running) {
        if (fd < 0) {
            fd = open(device, O_RDONLY | O_NONBLOCK);
            if (fd < 0) {
                if (errno != ENOENT) perror("open HID");
                usleep(1000000);
                continue;
            }
            printf("成功打开 HID 设备 %s\n", device);
        }

        struct pollfd pfd = {.fd = fd, .events = POLLIN};
        int ret = poll(&pfd, 1, 500);
        if (ret < 0) {
            if (errno == EINTR) continue;
            perror("poll HID");
            close(fd);
            fd = -1;
            continue;
        }
        if (ret == 0) continue;

        if (idx == 0) {
            struct imu_hid_report rpt;
            int n = read(fd, &rpt, sizeof(rpt));
            if (n == sizeof(rpt) && g_ctx.imu_cb) {
                g_ctx.imu_cb(rpt.ax, rpt.ay, rpt.az,
                             rpt.gx, rpt.gy, rpt.gz,
                             rpt.timestamp,
                             g_ctx.imu_userdata);
            } else if (n < 0) {
                perror("read IMU");
                close(fd);
                fd = -1;
            }
        } else {
            struct vio_hid_payload payload;
            int n = read(fd, &payload, sizeof(payload));
            if (n == sizeof(payload) && g_ctx.vio_cb) {
                g_ctx.vio_cb(payload.px, payload.py, payload.pz,
                             payload.qx, payload.qy, payload.qz, payload.qw,
                             payload.seq,
                             g_ctx.vio_userdata);
            } else if (n < 0) {
                perror("read VIO");
                close(fd);
                fd = -1;
            }
        }
    }

    if (fd >= 0) close(fd);
    printf("HID 线程 %d 退出\n", idx);
    return NULL;
}

// ==================== SDK API 实现 ====================
int insight9_receive_init(void) {
    if (g_ctx.initialized) {
        fprintf(stderr, "Insight_9_receive already initialized\n");
        return -1;
    }

    memset(&g_ctx, 0, sizeof(g_ctx));

    // 查找 UVC 设备
    char uvc_list[10][MAX_PATH] = {{0}};
    int uvc_count = find_uvc_devices_by_vid_pid(VENDOR_ID, PRODUCT_ID, uvc_list, 10);
    if (uvc_count < 3) {
        fprintf(stderr, "Error: need at least 3 UVC devices with VID=0x%04x PID=0x%04x, found %d\n",
                VENDOR_ID, PRODUCT_ID, uvc_count);
        return -1;
    }
    // 隔一个选取：索引 0,2,4
    int selected_idx[] = {0, 2, 4};
    for (int i = 0; i < CAM_NUM; i++) {
        if (selected_idx[i] >= uvc_count) {
            fprintf(stderr, "Error: cannot select 3 devices by skipping one (need at least 6 devices)\n");
            return -1;
        }
        strcpy(g_ctx.video_devs[i], uvc_list[selected_idx[i]]);
        printf("Selected UVC device %d: %s\n", i, g_ctx.video_devs[i]);
    }

    // 查找 HID 设备
    char hid_list[10][MAX_PATH] = {{0}};
    int hid_count = find_hid_devices_by_vid_pid(VENDOR_ID, PRODUCT_ID, hid_list, 10);
    if (hid_count < 2) {
        fprintf(stderr, "Error: need at least 2 HID devices with VID=0x%04x PID=0x%04x, found %d\n",
                VENDOR_ID, PRODUCT_ID, hid_count);
        return -1;
    }
    // 取前两个，小的为 IMU，大的为 VIO（已排序）
    strcpy(g_ctx.hid_devs[0], hid_list[0]);
    strcpy(g_ctx.hid_devs[1], hid_list[1]);
    printf("Selected HID devices: IMU=%s, VIO=%s\n", g_ctx.hid_devs[0], g_ctx.hid_devs[1]);

    // 初始化摄像头上下文
    for (int i = 0; i < CAM_NUM; i++) {
        g_ctx.cams[i].cam_id = i;
        g_ctx.cams[i].fd = -1;
        if (i == 0) {
            g_ctx.cams[i].width = MAIN_WIDTH;
            g_ctx.cams[i].height = MAIN_HEIGHT;
            g_ctx.cams[i].format = MAIN_FORMAT;
        } else {
            g_ctx.cams[i].width = SUB_WIDTH;
            g_ctx.cams[i].height = SUB_HEIGHT;
            g_ctx.cams[i].format = SUB_FORMAT;
        }
    }

    g_ctx.initialized = 1;
    printf("Insight_9_receive initialized.\n");
    return 0;
}

int insight9_receive_start(void) {
    if (!g_ctx.initialized) {
        fprintf(stderr, "Insight_9_receive not initialized\n");
        return -1;
    }
    if (g_ctx.running) {
        fprintf(stderr, "Insight_9_receive already running\n");
        return -1;
    }

    g_ctx.running = 1;

    for (int i = 0; i < CAM_NUM; i++) {
        pthread_create(&g_ctx.cams[i].tid, NULL, capture_thread, &g_ctx.cams[i]);
    }
    for (int i = 0; i < 2; i++) {
        pthread_create(&g_ctx.hid_tids[i], NULL, hid_thread, (void*)(intptr_t)i);
    }

    printf("Insight_9_receive started.\n");
    return 0;
}

void insight9_receive_stop(void) {
    if (!g_ctx.running) return;
    g_ctx.running = 0;

    for (int i = 0; i < CAM_NUM; i++) {
        pthread_join(g_ctx.cams[i].tid, NULL);
    }
    for (int i = 0; i < 2; i++) {
        pthread_join(g_ctx.hid_tids[i], NULL);
    }
    printf("Insight_9_receive stopped.\n");
}

void insight9_receive_cleanup(void) {
    if (!g_ctx.initialized) return;

    if (g_ctx.running) {
        insight9_receive_stop();
    }

    for (int i = 0; i < CAM_NUM; i++) {
        struct cam_ctx *ctx = &g_ctx.cams[i];
        if (ctx->fd >= 0) {
            stop_capture(ctx->fd);
            if (ctx->buffers) {
                for (int j = 0; j < ctx->buffer_count; j++) {
                    if (ctx->buffers[j].start)
                        munmap(ctx->buffers[j].start, ctx->buffers[j].length);
                }
                free(ctx->buffers);
                ctx->buffers = NULL;
            }
            close(ctx->fd);
            ctx->fd = -1;
        }
    }

    g_ctx.initialized = 0;
    printf("Insight_9_receive cleaned up.\n");
}

void insight9_receive_register_image_callback(image_callback cb, void *userdata) {
    g_ctx.img_cb = cb;
    g_ctx.img_userdata = userdata;
}

void insight9_receive_register_imu_callback(imu_callback cb, void *userdata) {
    g_ctx.imu_cb = cb;
    g_ctx.imu_userdata = userdata;
}

void insight9_receive_register_vio_callback(vio_callback cb, void *userdata) {
    g_ctx.vio_cb = cb;
    g_ctx.vio_userdata = userdata;
}