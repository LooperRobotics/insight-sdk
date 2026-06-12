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
#include "UvcExtensionUnit.hpp"
#include <atomic>

#ifndef V4L2_META_FMT_UVC
#define V4L2_META_FMT_UVC v4l2_fourcc('U', 'V', 'C', 'H')
#endif

// ==================== Target Device VID/PID ====================
#define VENDOR_ID  0x1d6b
#define PRODUCT_ID 0x0104

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

// ==================== Data Structures ====================
#define MAX_PATH 1024

std::string hardware_model[50] = { "Insight 9", "Insight 7", "Insight 7p", "Insight 3u" };

struct buffer {
    void *start;
    size_t length;
};

struct cam_ctx {
    int fd;                         // Device file descriptor
    struct buffer *buffers;         // mmap buffer array
    int buffer_count;               // Buffer count
    pthread_t tid;                  // Capture thread ID
    int cam_id;
    int width;
    int height;
    unsigned int format;
    uint64_t last_timestamp;        // Last embedded timestamp delivered, used to drop duplicates
};

// SDK global context
typedef struct {
    // Cameras
    struct cam_ctx cams[CAM_NUM];
    char video_devs[CAM_NUM][MAX_PATH];   // Dynamically resolved video device paths
    char metadata_devs[CAM_NUM][MAX_PATH]; // Matching UVC metadata device paths
    char video_usb_paths[CAM_NUM][MAX_PATH]; // Matching USB root paths, used for rediscovery after reconnect
    char metadata_usb_paths[CAM_NUM][MAX_PATH]; // Matching metadata USB root paths
    // HID devices (only two are used: 0=IMU, 1=VIO)
    char hid_devs[HID_NUM][MAX_PATH];           // Dynamically resolved hidraw device paths
    char hid_usb_paths[HID_NUM][MAX_PATH];   // Matching HID USB root paths, used for rediscovery after reconnect
    pthread_t hid_tids[HID_NUM];
    // Callbacks
    image_callback img_cb;
    void *img_userdata;
    imu_callback imu_cb;
    void *imu_userdata;
    vio_callback vio_cb;
    void *vio_userdata;
    // Running flag
    volatile int running;
    // Initialization flag
    int initialized;
    viewer::UvcExtensionUnit *xu_control;
    std::atomic<bool> xu_ready;
} sdk_ctx_t;

static sdk_ctx_t g_ctx = {0};

// ==================== Utilities: sysfs Reads, VID/PID Parsing, etc. ====================
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

static int get_video_usb_device_path(const char *video_dev, char *usb_path, size_t usb_path_size) {
    const char *base = strrchr(video_dev, '/');
    if (base) base++;
    else base = video_dev;
    char video_sysfs[MAX_PATH];
    snprintf(video_sysfs, sizeof(video_sysfs), "/sys/class/video4linux/%s", base);
    return find_usb_path_from_video(video_sysfs, usb_path, usb_path_size);
}

static int get_hid_usb_device_path(const char *hid_dev, char *usb_path, size_t usb_path_size) {
    const char *base = strrchr(hid_dev, '/');
    if (base) base++;
    else base = hid_dev;
    char hid_sysfs[MAX_PATH];
    snprintf(hid_sysfs, sizeof(hid_sysfs), "/sys/class/hidraw/%s/device", base);

    char resolved[MAX_PATH];
    if (realpath(hid_sysfs, resolved) == NULL) {
        return -1;
    }

    char *p;
    while (strlen(resolved) > 1) {
        char path[MAX_PATH];
        snprintf(path, sizeof(path), "%s/idVendor", resolved);
        if (access(path, F_OK) == 0) {
            strncpy(usb_path, resolved, usb_path_size - 1);
            usb_path[usb_path_size - 1] = '\0';
            return 0;
        }
        p = strrchr(resolved, '/');
        if (!p) break;
        *p = '\0';
    }
    return -1;
}

static int find_video_device_by_usb_path(const char *usb_path, char dev_path[MAX_PATH]) {
    DIR *dir = opendir("/sys/class/video4linux");
    if (!dir) return -1;
    struct dirent *entry;
    char path[MAX_PATH];
    char candidate_usb[MAX_PATH];
    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_name[0] == '.') continue;
        snprintf(path, sizeof(path), "/dev/%s", entry->d_name);
        if (get_video_usb_device_path(path, candidate_usb, sizeof(candidate_usb)) != 0) continue;
        if (strcmp(candidate_usb, usb_path) == 0) {
            snprintf(dev_path, MAX_PATH, "/dev/%s", entry->d_name);
            closedir(dir);
            return 0;
        }
    }
    closedir(dir);
    return -1;
}

static int video_device_supports_format(const char *dev, int width, int height, unsigned int format) {
    int fd = open(dev, O_RDWR);
    if (fd < 0) return 0;
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = format;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    int ret = ioctl(fd, VIDIOC_TRY_FMT, &fmt);
    close(fd);
    if (ret < 0) return 0;
    return fmt.fmt.pix.width == width && fmt.fmt.pix.height == height && fmt.fmt.pix.pixelformat == format;
}

static int find_hid_device_by_usb_path(const char *usb_path, char dev_path[MAX_PATH]) {
    DIR *dir = opendir("/sys/class/hidraw");
    if (!dir) return -1;
    struct dirent *entry;
    char path[MAX_PATH];
    char candidate_usb[MAX_PATH];
    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_name[0] == '.') continue;
        snprintf(path, sizeof(path), "/dev/%s", entry->d_name);
        if (get_hid_usb_device_path(path, candidate_usb, sizeof(candidate_usb)) != 0) continue;
        if (strcmp(candidate_usb, usb_path) == 0) {
            snprintf(dev_path, MAX_PATH, "/dev/%s", entry->d_name);
            closedir(dir);
            return 0;
        }
    }
    closedir(dir);
    return -1;
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

// Find all UVC devices matching the VID/PID, return device paths (/dev/videoX) sorted by numeric suffix.
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

        // Check whether this is a UVC capture device.
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

    // Sort by device number.
    if (count > 1) {
        // Temporary pointer array for sorting.
        const char *ptrs[count];
        for (int i = 0; i < count; i++) ptrs[i] = dev_paths[i];
        qsort(ptrs, count, sizeof(char *), compare_device_numbers);
        // Reorder the device paths.
        char tmp[count][MAX_PATH];
        for (int i = 0; i < count; i++) strcpy(tmp[i], ptrs[i]);
        for (int i = 0; i < count; i++) strcpy(dev_paths[i], tmp[i]);
    }
    return count;
}

static void refresh_video_device_path(int cam_id) {
    printf("[CAM%d] Refreshing device path...\n", cam_id);
    if (cam_id < 0 || cam_id >= CAM_NUM) return;
    printf("[CAM%d] Current device path: %s\n", cam_id, g_ctx.video_devs[cam_id]);

    char uvc_list[10][MAX_PATH] = {{0}};
    int uvc_count = find_uvc_devices_by_vid_pid(VENDOR_ID, PRODUCT_ID, uvc_list, 10);
    if (uvc_count < CAM_NUM) {
        printf("[CAM%d] Found %d UVC devices, fewer than %d; skipping reconnect\n", cam_id, uvc_count, CAM_NUM);
        return;
    } else {
        printf("[CAM%d] Found %d UVC devices, trying to match...\n", cam_id, uvc_count);
        for (int i = 0; i < uvc_count; ++i) {
            printf("  [%d] %s\n", i, uvc_list[i]);
        }
    }

    // Prefer format-based matching and choose the first device that supports the current camera format.
    for (int i = 0; i < uvc_count; ++i) {
        if (video_device_supports_format(uvc_list[i], g_ctx.cams[cam_id].width,
                                         g_ctx.cams[cam_id].height,
                                         g_ctx.cams[cam_id].format)) {
            if (strcmp(uvc_list[i], g_ctx.video_devs[cam_id]) != 0) {
                printf("[CAM%d] Rematched device path: %s -> %s (format match)\n", cam_id,
                       g_ctx.video_devs[cam_id], uvc_list[i]);
                strcpy(g_ctx.video_devs[cam_id], uvc_list[i]);
                if (get_video_usb_device_path(g_ctx.video_devs[cam_id], g_ctx.video_usb_paths[cam_id], MAX_PATH) < 0) {
                    g_ctx.video_usb_paths[cam_id][0] = '\0';
                }
            }
            return;
        }
    }

    // Fallback: select by the predefined device index.
    int selected_idx[CAM_NUM] = {0, 2, 4};
    int index = (uvc_count >= 6 ? selected_idx[cam_id] : cam_id);
    if (index >= uvc_count) index = cam_id;
    if (index >= uvc_count) {
        printf("[CAM%d] Device index is out of range; cannot reconnect\n", cam_id);
        return;
    }

    if (strcmp(uvc_list[index], g_ctx.video_devs[cam_id]) != 0) {
        printf("[CAM%d] Rematched device path: %s -> %s (index fallback)\n", cam_id, 
               g_ctx.video_devs[cam_id], uvc_list[index]);
        strcpy(g_ctx.video_devs[cam_id], uvc_list[index]);
    }
    if (get_video_usb_device_path(g_ctx.video_devs[cam_id], g_ctx.video_usb_paths[cam_id], MAX_PATH) < 0) {
        g_ctx.video_usb_paths[cam_id][0] = '\0';
    }
}

// Find all HID devices matching the VID/PID, return device paths (/dev/hidrawX) sorted by numeric suffix.
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

        // Find the USB path corresponding to the hidraw device.
        char device_path[MAX_PATH];
        snprintf(device_path, sizeof(device_path), "%s/device", hidraw_sysfs);
        if (realpath(device_path, usb_path) == NULL) continue;

        // Walk upward to find the USB root directory that contains idVendor.
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

static void refresh_hid_device_path(int idx) {
    if (idx < 0 || idx >= HID_NUM) return;

    char hid_list[10][MAX_PATH] = {{0}};
    int hid_count = find_hid_devices_by_vid_pid(VENDOR_ID, PRODUCT_ID, hid_list, 10);
    if (hid_count < HID_NUM) {
        fprintf(stderr, "[HID%d][WARN] found %d HID devices (< %d), skip reconnect\n",
                idx, hid_count, HID_NUM);
        return;
    }

    // Try matching with the cached USB path; if it is stale, choose the device from the current list.
    if (strcmp(hid_list[idx], g_ctx.hid_devs[idx]) != 0) {
        printf("[HID%d] rematched %s -> %s\n",
               idx, g_ctx.hid_devs[idx], hid_list[idx]);
        strcpy(g_ctx.hid_devs[idx], hid_list[idx]);
    }
    if (get_hid_usb_device_path(g_ctx.hid_devs[idx], g_ctx.hid_usb_paths[idx], MAX_PATH) < 0) {
        g_ctx.hid_usb_paths[idx][0] = '\0';
    }
}

// ==================== V4L2 Operations ====================
static void print_camera_info(int fd) {
    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
        perror("VIDIOC_QUERYCAP failed");
        return;
    }
    printf("  Driver: %s\n", cap.driver);
    printf("  Card: %s\n", cap.card);
    printf("  Bus: %s\n", cap.bus_info);
}

static int set_framerate(int fd, int framerate) {
    struct v4l2_streamparm parm;
    memset(&parm, 0, sizeof(parm));
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_PARM, &parm) < 0) {
        fprintf(stderr, "[CAM][ERR] VIDIOC_G_PARM: %s\n", strerror(errno));
        return -1;
    }
    if (!(parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)) {
        // The device does not support setting the frame rate.
        return 0;
    }
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = framerate;
    if (ioctl(fd, VIDIOC_S_PARM, &parm) < 0) {
        fprintf(stderr, "[CAM][ERR] VIDIOC_S_PARM: %s\n", strerror(errno));
        return -1;
    }
    printf("[CAM] set framerate=%d fps\n", framerate);
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

    printf("[CAM%d] setting format %dx%d fmt=0x%x\n",
           ctx->cam_id, ctx->width, ctx->height, ctx->format);

    if (ioctl(ctx->fd, VIDIOC_S_FMT, &fmt) < 0) {
        fprintf(stderr, "[CAM%d][ERR] VIDIOC_S_FMT: %s\n", ctx->cam_id, strerror(errno));
        return -1;
    }

    ctx->width = fmt.fmt.pix.width;
    ctx->height = fmt.fmt.pix.height;
    ctx->format = fmt.fmt.pix.pixelformat;
    printf("[CAM%d] negotiated %dx%d fmt=0x%x\n",
           ctx->cam_id, ctx->width, ctx->height, ctx->format);
    
    int target_framerate = 30;
    if (ctx->format == V4L2_PIX_FMT_GREY) {
        target_framerate = 20;
    } else if (ctx->format == V4L2_PIX_FMT_Z16) {
        target_framerate = 15;
    }
    set_framerate(ctx->fd, target_framerate);

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(ctx->fd, VIDIOC_REQBUFS, &req) < 0) {
        fprintf(stderr, "[CAM%d][ERR] VIDIOC_REQBUFS: %s\n", ctx->cam_id, strerror(errno));
        return -1;
    }

    if (req.count < 2) {
        fprintf(stderr, "[CAM%d][ERR] insufficient buffers (%u)\n", ctx->cam_id, req.count);
        return -1;
    }

    ctx->buffers = (buffer*)calloc(req.count, sizeof(struct buffer));
    if (!ctx->buffers) {
        fprintf(stderr, "[CAM%d][ERR] calloc: %s\n", ctx->cam_id, strerror(errno));
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
            fprintf(stderr, "[CAM%d][ERR] VIDIOC_QUERYBUF: %s\n", ctx->cam_id, strerror(errno));
            return -1;
        }

        ctx->buffers[i].length = buf.length;
        ctx->buffers[i].start = mmap(NULL, buf.length,
                                     PROT_READ | PROT_WRITE,
                                     MAP_SHARED, ctx->fd, buf.m.offset);

        if (ctx->buffers[i].start == MAP_FAILED) {
            fprintf(stderr, "[CAM%d][ERR] mmap: %s\n", ctx->cam_id, strerror(errno));
            goto err_cleanup;
        }

        if (ioctl(ctx->fd, VIDIOC_QBUF, &buf) < 0) {
            fprintf(stderr, "[CAM%d][ERR] VIDIOC_QBUF: %s\n", ctx->cam_id, strerror(errno));
            goto err_cleanup;
        }
    }

    return 0;

err_cleanup:
    for (int j = 0; j < req.count; j++) {
        if (ctx->buffers[j].start)
            munmap(ctx->buffers[j].start, ctx->buffers[j].length);
    }
    free(ctx->buffers);
    ctx->buffers = NULL;
    ctx->buffer_count = 0;
    return -1;
}

static int start_capture(int fd) {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        fprintf(stderr, "[CAM][ERR] VIDIOC_STREAMON: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

static int stop_capture(int fd) {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
        fprintf(stderr, "[CAM][ERR] VIDIOC_STREAMOFF: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

// ==================== Extension Unit Helpers ====================
static int ensure_xu_available() {
    if (!g_ctx.xu_control) {
        // Try to recreate the extension unit.
        if (g_ctx.video_devs[0][0] == '\0') return -1;
        g_ctx.xu_control = new viewer::UvcExtensionUnit();
        if (!g_ctx.xu_control->open(g_ctx.video_devs[0])) {
            delete g_ctx.xu_control;
            g_ctx.xu_control = nullptr;
            g_ctx.xu_ready = false;
            return -1;
        }
        g_ctx.xu_ready = true;
    } else if (!g_ctx.xu_control->isOpen()) {
        // The extension unit exists but is closed; try to reopen it because the path may have changed.
        g_ctx.xu_control->close();
        if (!g_ctx.xu_control->open(g_ctx.video_devs[0])) {
            g_ctx.xu_ready = false;
            return -1;
        }
        g_ctx.xu_ready = true;
    }
    return 0;
}

static int is_camera_params_valid(const camera_params *params) {
    // Validate resolution index. RGB uses 0-3 and grayscale uses 0-1; the SDK uses the more permissive 0-3 range.
    if (params->resolution > 3) {
        fprintf(stderr, "[XU][ERR] invalid resolution index, expected 0-3, got %d\n", params->resolution);
        return 0;
    }
    // Validate frame-rate index.
    if (params->frame_rate > 5) {
        fprintf(stderr, "[XU][ERR] invalid frame rate index, expected 0-5, got %d\n", params->frame_rate);
        return 0;
    }

    // exposure_time: 0.0 ~ 0.03
    if (params->exposure_time < 0.0f || params->exposure_time > 0.03f) {
        fprintf(stderr, "[XU][ERR] invalid exposure time, expected 0.0-0.03, got %f\n", params->exposure_time);
        return 0;
    }
    // exposure_gain: 1.0 ~ 16.0
    if (params->exposure_gain < 1.0f || params->exposure_gain > 16.0f) {
        fprintf(stderr, "[XU][ERR] invalid exposure gain, expected 1.0-16.0, got %f\n", params->exposure_gain);
        return 0;
    }
    // auto_exposure: 0 or 1
    if (params->auto_exposure > 1) {
        fprintf(stderr, "[XU][ERR] invalid auto exposure, expected 0 or 1, got %d\n", params->auto_exposure);
        return 0;
    }
    // brightness: 0.0 ~ 127.0
    if (params->brightness < 0.0f || params->brightness > 127.0f) {
        fprintf(stderr, "[XU][ERR] invalid brightness, expected 0.0-127.0, got %f\n", params->brightness);
        return 0;
    }
    // contrast: 0.0 ~ 1.9
    if (params->contrast < 0.0f || params->contrast > 1.9f) {
        fprintf(stderr, "[XU][ERR] invalid contrast, expected 0.0-1.9, got %f\n", params->contrast);
        return 0;
    }
    // gamma_dark: 1.0 ~ 4.0
    if (params->gamma_dark < 1.0f || params->gamma_dark > 4.0f) {
        fprintf(stderr, "[XU][ERR] invalid gamma dark, expected 1.0-4.0, got %f\n", params->gamma_dark);
        return 0;
    }
    // hue: 0.0 ~ 87.0
    if (params->hue < 0.0f || params->hue > 87.0f) {
        fprintf(stderr, "[XU][ERR] invalid hue, expected 0.0-87.0, got %f\n", params->hue);
        return 0;
    }
    // saturation: 0.0 ~ 1.999
    if (params->saturation < 0.0f || params->saturation > 1.999f) {
        fprintf(stderr, "[XU][ERR] invalid saturation, expected 0.0-1.999, got %f\n", params->saturation);
        return 0;
    }
    // sharpness: 1 ~ 255
    if (params->sharpness < 1 || params->sharpness > 255) {
        fprintf(stderr, "[XU][ERR] invalid sharpness, expected 1-255, got %d\n", params->sharpness);
        return 0;
    }
    // auto_white_balance: 0 or 1
    if (params->auto_white_balance > 1) {
        fprintf(stderr, "[XU][ERR] invalid auto white balance, expected 0 or 1, got %d\n", params->auto_white_balance);
        return 0;
    }
    // white_balance: 1.0 ~ 3.0
    if (params->white_balance < 1.0f || params->white_balance > 3.0f) {
        fprintf(stderr, "[XU][ERR] invalid white balance, expected 1.0-3.0, got %f\n", params->white_balance);
        return 0;
    }
    // decimation: 1 ~ 255
    if (params->decimation < 1 || params->decimation > 255) {
        fprintf(stderr, "[XU][ERR] invalid decimation, expected 1-255, got %d\n", params->decimation);
        return 0;
    }

    return 1;
}

// ==================== Camera Capture Thread ====================
static void *capture_thread(void *arg) {
    struct cam_ctx *ctx = (struct cam_ctx *)arg;
    unsigned long frame_count = 0;

    printf("[CAM%d] capture thread started, dev=%s meta=%s\n",
           ctx->cam_id, g_ctx.video_devs[ctx->cam_id], g_ctx.metadata_devs[ctx->cam_id]);

    while (g_ctx.running) {
        struct pollfd fds;
        fds.fd = ctx->fd;
        fds.events = POLLIN;
        const char *dev_path = g_ctx.video_devs[ctx->cam_id];
        if (ctx->fd < 0) {
            printf("[CAM%d] device not open, opening %s\n", ctx->cam_id, dev_path);
            refresh_video_device_path(ctx->cam_id);
            dev_path = g_ctx.video_devs[ctx->cam_id];
            printf("[CAM%d] reopening %s\n", ctx->cam_id, dev_path);
            ctx->fd = open(dev_path, O_RDWR);
            if (ctx->fd < 0) {
                fprintf(stderr, "[CAM%d][ERR] open failed: %s, retry in 1s\n",
                        ctx->cam_id, strerror(errno));
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
                fprintf(stderr, "[CAM%d][ERR] failed to start stream\n", ctx->cam_id);
                close(ctx->fd);
                ctx->fd = -1;
                usleep(1000000);
                continue;
            }
            ctx->last_timestamp = 0;
            printf("[CAM%d] device reinitialized\n", ctx->cam_id);
            frame_count = 0;
        }

        int ret = poll(&fds, 1, 200);
        if (ret < 0) {
            if (errno == EINTR)
                continue;
            fprintf(stderr, "[CAM%d][ERR] poll: %s\n", ctx->cam_id, strerror(errno));
            close(ctx->fd);
            ctx->fd = -1;
            continue;
        } else if (ret == 0) {
            // Timed out; continue the loop.
            continue;
        }

        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(ctx->fd, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) {
                // poll reported data but DQBUF returned EAGAIN; this is rare, so retry directly.
                continue;
            }
            if (errno == ENODEV) {
                fprintf(stderr, "[CAM%d][WARN] device removed, waiting for reconnect\n", ctx->cam_id);
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
            fprintf(stderr, "[CAM%d][ERR] VIDIOC_DQBUF: %s\n", ctx->cam_id, strerror(errno));
            continue;
        }

        // Filter 1: skip buffers the driver flagged as errored (incomplete USB transfer).
        if (buf.flags & V4L2_BUF_FLAG_ERROR) {
            if (ioctl(ctx->fd, VIDIOC_QBUF, &buf) < 0) {
                fprintf(stderr, "[CAM%d][ERR] VIDIOC_QBUF: %s\n", ctx->cam_id, strerror(errno));
                close(ctx->fd);
                ctx->fd = -1;
            }
            continue;
        }

        uint64_t timestamp = 0;
        uint64_t right_timestamp = 0;
        if (ctx->format == V4L2_PIX_FMT_MJPEG) {
            uint8_t *p = (uint8_t*)ctx->buffers[buf.index].start;
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
            if (buf.bytesused >= (ctx->width * ctx->height)) {
                memcpy(&timestamp,
                       (uint8_t*)ctx->buffers[buf.index].start + ctx->width * (ctx->height - 1),
                       sizeof(timestamp));
                memcpy(&right_timestamp,
                       (uint8_t*)ctx->buffers[buf.index].start + ctx->width * (ctx->height - 1) + sizeof(timestamp),
                       sizeof(right_timestamp));
            }
        } else if (ctx->format == V4L2_PIX_FMT_Z16) {
            if (buf.bytesused >= (ctx->width * ctx->height)) {
                memcpy(&timestamp,
                       (uint8_t*)ctx->buffers[buf.index].start + ctx->width * (ctx->height - 2) * 2,
                       sizeof(timestamp));
            }
        }

        // Filter 2: drop buffers until matching UVC metadata provides a usable timestamp.
        if (timestamp == 0) {
            if (ioctl(ctx->fd, VIDIOC_QBUF, &buf) < 0) {
                fprintf(stderr, "[CAM%d][ERR] VIDIOC_QBUF: %s\n", ctx->cam_id, strerror(errno));
                close(ctx->fd);
                ctx->fd = -1;
            }
            continue;
        }

        // Filter 3: drop duplicates — the UVC driver can redeliver the same buffer
        // when the host polls faster than the sender produces new frames.
        if (timestamp == ctx->last_timestamp) {
            if (ioctl(ctx->fd, VIDIOC_QBUF, &buf) < 0) {
                fprintf(stderr, "[CAM%d][ERR] VIDIOC_QBUF: %s\n", ctx->cam_id, strerror(errno));
                close(ctx->fd);
                ctx->fd = -1;
            }
            continue;
        }
        ctx->last_timestamp = timestamp;

        if (g_ctx.img_cb) {
            g_ctx.img_cb(ctx->cam_id,
                         (uint8_t*)ctx->buffers[buf.index].start,
                         buf.bytesused,
                         ctx->width, ctx->height,
                         ctx->format,
                         timestamp,
                         right_timestamp,
                         g_ctx.img_userdata);
        }

        frame_count++;

        if (ioctl(ctx->fd, VIDIOC_QBUF, &buf) < 0) {
            fprintf(stderr, "[CAM%d][ERR] VIDIOC_QBUF: %s\n", ctx->cam_id, strerror(errno));
            close(ctx->fd);
            ctx->fd = -1;
        }
    }

    return NULL;
}

// ==================== HID Device Reading ====================
struct __attribute__((packed)) imu_hid_report {
    float ax, ay, az;
    float gx, gy, gz;
    uint64_t timestamp;
};

struct __attribute__((packed)) vio_hid_payload {
    uint64_t timestamp;
    float px, py, pz;
    float qx, qy, qz, qw;
    uint8_t seq;
    uint8_t reserved[3];
};

static void *hid_thread(void *arg) {
    int idx = (int)(intptr_t)arg;
    const char *device = g_ctx.hid_devs[idx];
    int fd = -1;
    uint64_t last_ts = 0;

    printf("[HID%d] thread started, dev=%s\n", idx, g_ctx.hid_devs[idx]);

    while (g_ctx.running) {
        const char *device = g_ctx.hid_devs[idx];
        if (fd < 0) {
            refresh_hid_device_path(idx);
            device = g_ctx.hid_devs[idx];
            fd = open(device, O_RDONLY | O_NONBLOCK);
            if (fd < 0) {
                if (errno != ENOENT)
                    fprintf(stderr, "[HID%d][ERR] open %s: %s\n", idx, device, strerror(errno));
                usleep(1000000);
                continue;
            }
            printf("[HID%d] opened %s\n", idx, device);
            last_ts = 0;
        }

        struct pollfd pfd = {.fd = fd, .events = POLLIN};
        int ret = poll(&pfd, 1, -1);
        if (ret < 0) {
            if (errno == EINTR) continue;
            fprintf(stderr, "[HID%d][ERR] poll: %s\n", idx, strerror(errno));
            close(fd);
            fd = -1;
            continue;
        }
        if (ret == 0) continue;

        if (idx == 0) {
            struct imu_hid_report rpt;
            int n = read(fd, &rpt, sizeof(rpt));
            if (n == sizeof(rpt)) {
                // Drop duplicates if the device redelivers the same sample.
                if (rpt.timestamp != 0 && rpt.timestamp == last_ts) continue;
                last_ts = rpt.timestamp;
                if (g_ctx.imu_cb) {
                    g_ctx.imu_cb(rpt.ax, rpt.ay, rpt.az,
                                 rpt.gx, rpt.gy, rpt.gz,
                                 rpt.timestamp,
                                 g_ctx.imu_userdata);
                }
            } else if (n < 0) {
                fprintf(stderr, "[HID%d][ERR] read IMU: %s\n", idx, strerror(errno));
                close(fd);
                fd = -1;
            }
        } else {
            struct vio_hid_payload payload;
            int n = read(fd, &payload, sizeof(payload));
            if (n == sizeof(payload)) {
                if (payload.timestamp != 0 && payload.timestamp == last_ts) continue;
                last_ts = payload.timestamp;
                if (g_ctx.vio_cb) {
                    g_ctx.vio_cb(payload.px, payload.py, payload.pz,
                                 payload.qx, payload.qy, payload.qz, payload.qw,
                                 payload.timestamp,
                                 g_ctx.vio_userdata);
                }
            } else if (n < 0) {
                fprintf(stderr, "[HID%d][ERR] read VIO: %s\n", idx, strerror(errno));
                close(fd);
                fd = -1;
            }
        }
    }

    if (fd >= 0) close(fd);
    printf("[HID%d] thread exited\n", idx);
    return NULL;
}

// ==================== SDK API Implementation ====================
int insight9_receive_init(void) {
    if (g_ctx.initialized) {
        fprintf(stderr, "[SDK][WARN] already initialized\n");
        return -1;
    }

    memset(&g_ctx, 0, sizeof(g_ctx));

    // Find UVC devices.
    char uvc_list[10][MAX_PATH] = {{0}};
    int uvc_count = find_uvc_devices_by_vid_pid(VENDOR_ID, PRODUCT_ID, uvc_list, 10);
    if (uvc_count < 6) {
        fprintf(stderr, "[SDK][ERR] need >= 6 UVC video/metadata nodes with VID=0x%04x PID=0x%04x, found %d\n",
                VENDOR_ID, PRODUCT_ID, uvc_count);
        return -1;
    }
    // Select video nodes 0/2/4 and their matching metadata nodes 1/3/5.
    int selected_idx[] = {0, 2, 4};
    for (int i = 0; i < CAM_NUM; i++) {
        if (selected_idx[i] >= uvc_count) {
            fprintf(stderr, "Error: cannot select 3 devices by skipping one (need at least 6 devices)\n");
            return -1;
        }
        strcpy(g_ctx.video_devs[i], uvc_list[selected_idx[i]]);
        g_ctx.video_usb_paths[i][0] = '\0';
        if (get_video_usb_device_path(g_ctx.video_devs[i], g_ctx.video_usb_paths[i], MAX_PATH) < 0) {
            g_ctx.video_usb_paths[i][0] = '\0';
        }
        printf("Selected UVC device %d: %s\n", i, g_ctx.video_devs[i]);
    }

    // Find HID devices.
    char hid_list[10][MAX_PATH] = {{0}};
    int hid_count = find_hid_devices_by_vid_pid(VENDOR_ID, PRODUCT_ID, hid_list, 10);
    if (hid_count < 2) {
        fprintf(stderr, "[SDK][ERR] need >= 2 HID devices with VID=0x%04x PID=0x%04x, found %d\n",
                VENDOR_ID, PRODUCT_ID, hid_count);
        return -1;
    }
    // Take the first two devices after sorting: the lower number is IMU and the higher number is VIO.
    strcpy(g_ctx.hid_devs[0], hid_list[0]);
    strcpy(g_ctx.hid_devs[1], hid_list[1]);
    g_ctx.hid_usb_paths[0][0] = '\0';
    g_ctx.hid_usb_paths[1][0] = '\0';
    if (get_hid_usb_device_path(g_ctx.hid_devs[0], g_ctx.hid_usb_paths[0], MAX_PATH) < 0) {
        g_ctx.hid_usb_paths[0][0] = '\0';
    }
    if (get_hid_usb_device_path(g_ctx.hid_devs[1], g_ctx.hid_usb_paths[1], MAX_PATH) < 0) {
        g_ctx.hid_usb_paths[1][0] = '\0';
    }
    printf("[SDK] selected HID: IMU=%s VIO=%s\n", g_ctx.hid_devs[0], g_ctx.hid_devs[1]);

    // Initialize camera contexts.
    for (int i = 0; i < CAM_NUM; i++) {
        g_ctx.cams[i].cam_id = i;
        g_ctx.cams[i].fd = -1;
        if (i == 0) {
            g_ctx.cams[i].width = MAIN_WIDTH;
            g_ctx.cams[i].height = MAIN_HEIGHT;
            g_ctx.cams[i].format = MAIN_FORMAT;
        } else if (i == 1) {
            g_ctx.cams[i].width = SUB_WIDTH;
            g_ctx.cams[i].height = SUB_HEIGHT;
            g_ctx.cams[i].format = SUB_FORMAT;
        } else if (i == 2) {
            g_ctx.cams[i].width = DEPTH_WIDTH;
            g_ctx.cams[i].height = DEPTH_HEIGHT;
            g_ctx.cams[i].format = DEPTH_FORMAT;
        }
    }

    if (g_ctx.video_devs[0][0] != '\0') {
        g_ctx.xu_control = new viewer::UvcExtensionUnit();
        if (!g_ctx.xu_control->open(g_ctx.video_devs[0])) {
            fprintf(stderr, "[XU][WARN] cannot open extension unit, camera params will be unavailable\n");
            delete g_ctx.xu_control;
            g_ctx.xu_control = nullptr;
            g_ctx.xu_ready = false;
        } else {
            g_ctx.xu_ready = true;
            printf("[XU] initialized, dev=%s\n", g_ctx.video_devs[0]);
        }
    } else {
        g_ctx.xu_control = nullptr;
        g_ctx.xu_ready = false;
    }

    g_ctx.initialized = 1;
    printf("[SDK] initialized\n");
    return 0;
}

int insight9_receive_start(void) {
    if (!g_ctx.initialized) {
        fprintf(stderr, "[SDK][ERR] not initialized\n");
        return -1;
    }
    if (g_ctx.running) {
        fprintf(stderr, "[SDK][WARN] already running\n");
        return -1;
    }

    g_ctx.running = 1;

    for (int i = 0; i < CAM_NUM; i++) {
        pthread_create(&g_ctx.cams[i].tid, NULL, capture_thread, &g_ctx.cams[i]);
    }
    for (int i = 0; i < 2; i++) {
        pthread_create(&g_ctx.hid_tids[i], NULL, hid_thread, (void*)(intptr_t)i);
    }

    printf("[SDK] started\n");
    return 0;
}

const char *insight9_receive_get_video_dev(int cam_id) {
    if (!g_ctx.initialized) {
        return NULL;
    }
    if (cam_id < 0 || cam_id >= CAM_NUM) {
        return NULL;
    }
    return g_ctx.video_devs[cam_id][0] ? g_ctx.video_devs[cam_id] : NULL;
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
    printf("[SDK] stopped\n");
}

void insight9_receive_cleanup(void) {
    if (!g_ctx.initialized) return;

    if (g_ctx.xu_control) {
        g_ctx.xu_control->close();
        delete g_ctx.xu_control;
        g_ctx.xu_control = nullptr;
    }

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
    printf("[SDK] cleaned up\n");
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

extern "C" {

int insight9_receive_set_active_camera(int cam_id) {
    if (ensure_xu_available() != 0) return -1;
    return g_ctx.xu_control->setActiveCamera(static_cast<uint8_t>(cam_id)) ? 0 : -1;
}

int insight9_receive_get_active_camera(int *cam_id) {
    if (!cam_id) return -1;
    if (ensure_xu_available() != 0) return -1;
    uint8_t val;
    if (!g_ctx.xu_control->getActiveCamera(val)) return -1;
    *cam_id = val;
    return 0;
}

int insight9_receive_set_camera_params(const camera_params *params) {
    if (!params || !is_camera_params_valid(params)) return -1;
    if (ensure_xu_available() != 0) return -1;
    viewer::camera_params xu_params;
    memcpy(&xu_params, params, sizeof(viewer::camera_params));
    return g_ctx.xu_control->writeCurrentCameraParams(xu_params) ? 0 : -1;
}

int insight9_receive_get_camera_params(camera_params *params) {
    if (!params) return -1;
    if (ensure_xu_available() != 0) return -1;
    viewer::camera_params xu_params;
    if (!g_ctx.xu_control->readCurrentCameraParams(xu_params)) return -1;
    memcpy(params, &xu_params, sizeof(camera_params));
    return 0;
}

int insight9_receive_set_camera_params_for(int cam_id, const camera_params *params) {
    if (!params || !is_camera_params_valid(params)) return -1;
    if (ensure_xu_available() != 0) return -1;
    viewer::camera_params xu_params;
    memcpy(&xu_params, params, sizeof(viewer::camera_params));
    return g_ctx.xu_control->writeCameraParams(static_cast<uint8_t>(cam_id), xu_params) ? 0 : -1;
}

int insight9_receive_get_camera_params_for(int cam_id, camera_params *params) {
    if (!params) return -1;
    if (ensure_xu_available() != 0) return -1;
    viewer::camera_params xu_params;
    if (!g_ctx.xu_control->readCameraParams(static_cast<uint8_t>(cam_id), xu_params)) return -1;
    memcpy(params, &xu_params, sizeof(camera_params));
    return 0;
}

int insight9_receive_reset_camera_params(int cam_id) {
    // Reading factory defaults from the device requires additional implementation. A simple approach is to read
    // and save the current values during initialization, then write them back when a reset is needed.
    fprintf(stderr, "[XU][ERR] reset_camera_params not implemented, use set_camera_params with saved defaults\n");
    return -1;
}

void insight9_receive_print_camera_params(const camera_params *params) {
    if (!params) return;
    viewer::camera_params xu_params;
    memcpy(&xu_params, params, sizeof(xu_params));
    viewer::printParams(xu_params);
}

std::string insight9_receive_get_hardware_type() {
    viewer::camera_params xu_params;
    if (!g_ctx.xu_control->readCurrentCameraParams(xu_params)) return "unknown";
    return hardware_model[xu_params.hardware_model];
}

} // extern "C"
