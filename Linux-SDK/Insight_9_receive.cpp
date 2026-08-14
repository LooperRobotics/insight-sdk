#include "Insight_9_receive.h"
#include "hid.h"
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
#include <math.h>
#include <vector>

// ==================== Reconnect Backoff / Retry Limits ====================
// Rationale: repeatedly opening / S_FMT-ing a device that is in a bad state
// (e.g. a composite UVC+NCM gadget that is warm-restarting) generates a USB
// transaction storm that can wedge the whole xhci controller. We therefore
// back off exponentially between reconnect attempts and cap the number of
// consecutive failures so a dead device cannot keep hammering the bus forever.
#define RECONNECT_BACKOFF_BASE_MS 1000   // first retry delay
#define RECONNECT_BACKOFF_MAX_MS  30000  // delay ceiling (exponential, capped)
// Max consecutive failed reconnect attempts before a thread stops actively
// reconnecting and idles at the ceiling interval (set to 0 to retry forever).
// After the limit is hit the thread no longer generates fast USB traffic, but
// still wakes periodically so a recovered device can be picked back up.
#define RECONNECT_MAX_ATTEMPTS    30



sdk_ctx_t g_ctx = {0};

// Compute the exponential backoff delay (ms) for the Nth consecutive failure
// (attempt counted from 1), capped at RECONNECT_BACKOFF_MAX_MS.
static int reconnect_backoff_ms(int attempt) {
    if (attempt < 1) attempt = 1;
    long delay = RECONNECT_BACKOFF_BASE_MS;
    for (int i = 1; i < attempt && delay < RECONNECT_BACKOFF_MAX_MS; ++i)
        delay <<= 1;
    if (delay > RECONNECT_BACKOFF_MAX_MS) delay = RECONNECT_BACKOFF_MAX_MS;
    return (int)delay;
}

// Sleep up to total_ms, but wake every 200ms to re-check g_ctx.running (and,
// if given, a per-camera cam_running flag) so a pending stop_camera() /
// pthread_join() is not blocked by a long backoff delay.
static void interruptible_sleep_ms(int total_ms, std::atomic<bool> *extra_stop = nullptr) {
    int slept = 0;
    while (slept < total_ms && g_ctx.running) {
        if (extra_stop && !extra_stop->load()) break;
        int chunk = (total_ms - slept) > 200 ? 200 : (total_ms - slept);
        usleep(chunk * 1000);
        slept += chunk;
    }
}

// Apply exponential backoff after a failed (re)connect attempt. Increments
// *fails and sleeps for the capped backoff delay. Logs every attempt up to
// RECONNECT_MAX_ATTEMPTS, then logs once to announce slow-retry mode and stays
// silent afterwards so a permanently-dead device cannot spam the log or hammer
// the USB bus. Reset *fails to 0 once the device reconnects successfully.
// extra_stop, when given, lets the sleep bail out early on a per-camera stop
// request instead of only reacting to the global g_ctx.running.
static void reconnect_backoff_apply(const char *tag, int id, int *fails, const char *reason,
                                     std::atomic<bool> *extra_stop = nullptr) {
    (*fails)++;
    int delay = reconnect_backoff_ms(*fails);
    if (RECONNECT_MAX_ATTEMPTS <= 0 || *fails <= RECONNECT_MAX_ATTEMPTS) {
        fprintf(stderr, "[%s%d][ERR] %s, retry in %dms (attempt %d)\n",
                tag, id, reason, delay, *fails);
    } else if (*fails == RECONNECT_MAX_ATTEMPTS + 1) {
        fprintf(stderr, "[%s%d][ERR] %s failed %d times, slowing to %ds retry to avoid USB storm\n",
                tag, id, reason, RECONNECT_MAX_ATTEMPTS, RECONNECT_BACKOFF_MAX_MS / 1000);
    }
    interruptible_sleep_ms(delay, extra_stop);
}

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

// Find all video4linux nodes matching the VID/PID, return device paths (/dev/videoX) sorted by numeric suffix.
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

        if (access(dev_paths[count], F_OK) != 0) {
            continue;
        }

        // Check whether this is a UVC capture device.
        if (!is_uvc_device(dev_paths[count]))
            continue;

        if (find_usb_path_from_video(video_sysfs, usb_path, sizeof(usb_path)) == 0) {
            if (parse_usb_vid_pid(usb_path, &vid, &pid) == 0) {
                if (pid == target_pid) {
                    count++;
                }
            }
        }
    }
    closedir(dir);

    if (count == 0) return 0;

    // Sort by device number.
    if (count > 1) {
        const char **ptrs = (const char **)malloc(count * sizeof(const char *));
        if (ptrs) {
            for (int i = 0; i < count; i++) ptrs[i] = dev_paths[i];
            qsort(ptrs, count, sizeof(char *), compare_device_numbers);
            
            char (*tmp)[MAX_PATH] = (char(*)[MAX_PATH])malloc(count * MAX_PATH);
            if (tmp) {
                for (int i = 0; i < count; i++) strcpy(tmp[i], ptrs[i]);
                for (int i = 0; i < count; i++) strcpy(dev_paths[i], tmp[i]);
                free(tmp);
            }
            free(ptrs);
        }
    }
    return count;
}

// 获取单个分辨率的帧率列表
int get_frame_intervals(int fd, uint32_t pixelformat, 
                        uint32_t width, uint32_t height,
                        unsigned int *intervals, int max_count) {
    struct v4l2_frmivalenum frmival;
    int count = 0;
    
    memset(&frmival, 0, sizeof(frmival));
    frmival.pixel_format = pixelformat;
    frmival.width = width;
    frmival.height = height;
    
    while (count < max_count && 
           ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival) == 0) {
        if (frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
            // 计算帧率（denominator/numerator）
            intervals[count] = frmival.discrete.denominator / 
                              frmival.discrete.numerator;
            count++;
        } else if (frmival.type == V4L2_FRMIVAL_TYPE_STEPWISE) {
            // 步进范围：取最大值
            intervals[count] = frmival.stepwise.max.denominator / 
                              frmival.stepwise.max.numerator;
            count++;
        } else if (frmival.type == V4L2_FRMIVAL_TYPE_CONTINUOUS) {
            // 连续范围，取常见值
            intervals[count++] = 30;  // 默认
            intervals[count++] = 25;
            intervals[count++] = 15;
        }
        frmival.index++;
    }
    
    return count;
}

static int get_all_formats_info(int fd, struct uvc_format_info **formats_out, int *format_num_out) {
    struct v4l2_fmtdesc fmtdesc;
    struct v4l2_frmsizeenum frmsize;
    struct uvc_format_info *formats = NULL;
    int format_count = 0;
    
    if (!formats_out || !format_num_out) {
        return -1;
    }
    
    *formats_out = NULL;
    *format_num_out = 0;
    
    memset(&fmtdesc, 0, sizeof(fmtdesc));
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    // 第一遍：统计格式数量
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
        format_count++;
        fmtdesc.index++;
    }
    
    if (format_count == 0) {
        fprintf(stderr, "[CAM] No formats found\n");
        return -1;
    }
    
    formats = (struct uvc_format_info*)calloc(format_count, sizeof(struct uvc_format_info));
    if (!formats) {
        fprintf(stderr, "[CAM] Failed to allocate formats memory\n");
        return -1;
    }
    
    // 第二遍：填充数据
    memset(&fmtdesc, 0, sizeof(fmtdesc));
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmtdesc.index = 0;
    
    int fmt_idx = 0;
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
        formats[fmt_idx].fcc = fmtdesc.pixelformat;
        formats[fmt_idx].frames_num = 0;
        formats[fmt_idx].frames = NULL;
        
        // 枚举该格式下的所有分辨率
        memset(&frmsize, 0, sizeof(frmsize));
        frmsize.pixel_format = fmtdesc.pixelformat;
        frmsize.index = 0;
        
        int frame_count = 0;
        struct uvc_frame_info *frames = NULL;
        
        while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0) {
            // 只处理离散分辨率
            if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
                frame_count++;
                frames = (struct uvc_frame_info*)realloc(frames, frame_count * sizeof(struct uvc_frame_info));
                if (!frames) {
                    fprintf(stderr, "[CAM] Failed to allocate frames memory\n");
                    // 清理已分配的内存
                    for (int i = 0; i < fmt_idx; i++) {
                        if (formats[i].frames) free(formats[i].frames);
                    }
                    free(formats);
                    return -1;
                }
                
                frames[frame_count - 1].width = frmsize.discrete.width;
                frames[frame_count - 1].height = frmsize.discrete.height;
                memset(frames[frame_count - 1].intervals, 0, 
                       sizeof(frames[frame_count - 1].intervals));
                
                // 获取该分辨率的帧率
                int interval_count = get_frame_intervals(
                    fd,
                    fmtdesc.pixelformat,
                    frmsize.discrete.width,
                    frmsize.discrete.height,
                    frames[frame_count - 1].intervals,
                    8  // 最多8个帧率
                );
                
                // 如果没获取到帧率，使用默认值
                if (interval_count == 0) {
                    frames[frame_count - 1].intervals[0] = 30;
                    interval_count = 1;
                }
            }
            frmsize.index++;
        }
        
        formats[fmt_idx].frames = frames;
        formats[fmt_idx].frames_num = frame_count;
        fmt_idx++;
        fmtdesc.index++;
    }
    
    *formats_out = formats;
    *format_num_out = format_count;
    return 0;
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
            int meta_index = get_device_number(uvc_list[i]) + 1;
            snprintf(g_ctx.metadata_devs[cam_id], MAX_PATH, "/dev/video%d", meta_index);
            if (get_video_usb_device_path(g_ctx.metadata_devs[cam_id], g_ctx.metadata_usb_paths[cam_id], MAX_PATH) < 0) {
                g_ctx.metadata_usb_paths[cam_id][0] = '\0';
            }
            return;
        }
    }

    // Fallback: select by the predefined device index.
    int selected_idx[CAM_NUM] = {0, 2, 4};
    int metadata_idx[CAM_NUM] = {1, 3, 5};
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
    if (uvc_count >= 6 && metadata_idx[cam_id] < uvc_count) {
        if (strcmp(uvc_list[metadata_idx[cam_id]], g_ctx.metadata_devs[cam_id]) != 0) {
            printf("[CAM%d][META] rematched %s -> %s\n",
                   cam_id, g_ctx.metadata_devs[cam_id], uvc_list[metadata_idx[cam_id]]);
            strcpy(g_ctx.metadata_devs[cam_id], uvc_list[metadata_idx[cam_id]]);
        }
        if (get_video_usb_device_path(g_ctx.metadata_devs[cam_id], g_ctx.metadata_usb_paths[cam_id], MAX_PATH) < 0) {
            g_ctx.metadata_usb_paths[cam_id][0] = '\0';
        }
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

// 获取摄像头所有格式信息并写入 cam_ctx
static int get_camera_formats_info(struct cam_ctx *ctx) {
    if (!ctx || ctx->fd < 0) {
        fprintf(stderr, "[CAM] Invalid context or file descriptor\n");
        return -1;
    }

    // 获取所有格式信息
    struct uvc_format_info *formats = NULL;
    int format_num = 0;
    
    if (get_all_formats_info(ctx->fd, &formats, &format_num) != 0) {
        fprintf(stderr, "[CAM%d] Failed to get formats info\n", ctx->cam_id);
        return -1;
    }
    
    ctx->formats_info = formats;
    ctx->format_num = format_num;
    
    printf("[CAM%d] Got %d formats info\n", ctx->cam_id, format_num);
    
    // 打印调试信息
    for (int i = 0; i < format_num; i++) {
        printf("  Format %d: 0x%08X (%c%c%c%c), frames=%d\n", 
               i,
               formats[i].fcc,
               (char)(formats[i].fcc & 0xFF),
               (char)((formats[i].fcc >> 8) & 0xFF),
               (char)((formats[i].fcc >> 16) & 0xFF),
               (char)((formats[i].fcc >> 24) & 0xFF),
               formats[i].frames_num);
        
        for (int j = 0; j < formats[i].frames_num; j++) {
            printf("    %dx%d: ", 
                   formats[i].frames[j].width,
                   formats[i].frames[j].height);
            for (int k = 0; k < 8; k++) {
                if (formats[i].frames[j].intervals[k] > 0) {
                    printf("%dfps ", formats[i].frames[j].intervals[k]);
                }
            }
            printf("\n");
        }
    }

    return 0;
}

// 获取设备默认格式
static int get_default_format(int fd, struct v4l2_format *fmt) {
    memset(fmt, 0, sizeof(*fmt));
    fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    // 先尝试获取当前设备正在使用的格式
    if (ioctl(fd, VIDIOC_G_FMT, fmt) == 0) {
        printf("[CAM] Got current format: %dx%d, pixelformat=0x%x\n",
               fmt->fmt.pix.width, fmt->fmt.pix.height, fmt->fmt.pix.pixelformat);
        return 0;
    }
    printf("[CAM] VIDIOC_G_FMT failed\n");
    return -1;
}

// 初始化捕获（从设备获取默认格式或使用指定格式）
static int init_capture(struct cam_ctx *ctx) {
    struct v4l2_format fmt;
    int use_default_format = 0;

    if (1) {
    // if (ctx->width == 0 || ctx->height == 0 || ctx->format == 0) {
        use_default_format = 1;
        printf("[CAM%d] Using default format from device\n", ctx->cam_id);
        if (get_default_format(ctx->fd, &fmt) < 0) {
            fprintf(stderr, "[CAM%d][ERR] Failed to get default format%d\n", ctx->cam_id, ctx->fd);
            return -1;
        }
        ctx->width = fmt.fmt.pix.width;
        ctx->height = fmt.fmt.pix.height;
        ctx->format = fmt.fmt.pix.pixelformat;
        printf("init capture %d %d %d\n", ctx->width, ctx->height, ctx->format);
    } else {
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = ctx->width;
        fmt.fmt.pix.height = ctx->height;
        fmt.fmt.pix.pixelformat = ctx->format;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
    }

    if (ctx->cam_id == 2) {
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        printf("set cam3 format mjpeg");
    }
    if (ioctl(ctx->fd, VIDIOC_S_FMT, &fmt) < 0) {
        fprintf(stderr, "[CAM%d][ERR] VIDIOC_S_FMT failed: %s\n", ctx->cam_id, strerror(errno));
        if (!use_default_format) {
            fprintf(stderr, "[CAM%d] Falling back to default format\n", ctx->cam_id);
            if (get_default_format(ctx->fd, &fmt) < 0) {
                return -1;
            }

            ctx->width = fmt.fmt.pix.width;
            ctx->height = fmt.fmt.pix.height;
            ctx->format = fmt.fmt.pix.pixelformat;
        } else {
            return -1;
        }
    }

    printf("[CAM%d] Format: %dx%d, FourCC: %c%c%c%c\n",
        ctx->cam_id, ctx->width, ctx->height, 
        ctx->format & 0xFF, (ctx->format >> 8) & 0xFF, 
        (ctx->format >> 16) & 0xFF, (ctx->format >> 24) & 0xFF);

    struct v4l2_format actual_fmt;
    memset(&actual_fmt, 0, sizeof(actual_fmt));
    actual_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(ctx->fd, VIDIOC_G_FMT, &actual_fmt) == 0) {
        ctx->width = actual_fmt.fmt.pix.width;
        ctx->height = actual_fmt.fmt.pix.height;
        ctx->format = actual_fmt.fmt.pix.pixelformat;
        printf("[CAM%d] negotiated %dx%d fmt=0x%x\n",
               ctx->cam_id, ctx->width, ctx->height, ctx->format);
    }

    set_framerate(ctx->fd, ctx->fps);
    int cam_id = ctx->cam_id;
    if (cam_id == 0) {
        set_framerate(ctx->fd, g_ctx.config.rgb_config.fps);
    } else if (cam_id == 1) {
        set_framerate(ctx->fd, g_ctx.config.gray_config.fps);
    } else if (cam_id == 2) {
        set_framerate(ctx->fd, g_ctx.config.depth_config.fps);
    }

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    int retries = 0;
    int ret;
    while (retries < 50) {
        ret = ioctl(ctx->fd, VIDIOC_REQBUFS, &req);
        if (ret == 0) {
            break;
        }

        if (errno == EBUSY) {
            retries++;
            fprintf(stderr, "[CAM%d][WARN] VIDIOC_REQBUFS忙，重试 %d...\n", ctx->cam_id, retries);
            usleep(100000); 
        } else {
            // 其他错误则直接退出
            fprintf(stderr, "[CAM%d][ERR] VIDIOC_REQBUFS失败: %s\n", ctx->cam_id, strerror(errno));
            return -1;
        }
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

// 获取摄像头当前格式
int insight9_receive_get_current_format(int cam_id, int *width, int *height, unsigned int *format) {
    if (!g_ctx.initialized) {
        fprintf(stderr, "[SDK][ERR] not initialized\n");
        return -1;
    }
    if (cam_id < 0 || cam_id >= CAM_NUM) {
        fprintf(stderr, "[SDK][ERR] invalid camera ID\n");
        return -1;
    }
    if (!width || !height || !format) {
        fprintf(stderr, "[SDK][ERR] NULL output parameters\n");
        return -1;
    }
    
    struct cam_ctx *ctx = &g_ctx.cams[cam_id];
    
    // 如果设备已打开，直接从设备查询
    if (ctx->fd >= 0) {
        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        
        if (ioctl(ctx->fd, VIDIOC_G_FMT, &fmt) == 0) {
            *width = fmt.fmt.pix.width;
            *height = fmt.fmt.pix.height;
            *format = fmt.fmt.pix.pixelformat;
            return 0;
        }
    }
    
    // 如果设备未打开，尝试打开并获取格式
    int fd = open(g_ctx.video_devs[cam_id], O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "[CAM%d][ERR] open failed: %s\n", cam_id, strerror(errno));
        return -1;
    }
    
    struct v4l2_format fmt;
    int ret = get_default_format(fd, &fmt);
    close(fd);
    
    if (ret == 0) {
        *width = fmt.fmt.pix.width;
        *height = fmt.fmt.pix.height;
        *format = fmt.fmt.pix.pixelformat;
        return 0;
    }
    
    // 如果获取失败，返回ctx中保存的值
    *width = ctx->width;
    *height = ctx->height;
    *format = ctx->format;
    return 0;
}

// 获取设备支持的格式列表
int insight9_receive_enum_formats(int cam_id, int index, struct v4l2_fmtdesc *desc) {
    if (!g_ctx.initialized) {
        fprintf(stderr, "[SDK][ERR] not initialized\n");
        return -1;
    }
    if (cam_id < 0 || cam_id >= CAM_NUM) {
        fprintf(stderr, "[SDK][ERR] invalid camera ID\n");
        return -1;
    }
    if (!desc) {
        fprintf(stderr, "[SDK][ERR] NULL descriptor\n");
        return -1;
    }
    
    int fd = open(g_ctx.video_devs[cam_id], O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "[CAM%d][ERR] open failed: %s\n", cam_id, strerror(errno));
        return -1;
    }
    
    memset(desc, 0, sizeof(*desc));
    desc->index = index;
    desc->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    int ret = ioctl(fd, VIDIOC_ENUM_FMT, desc);
    close(fd);
    
    return ret < 0 ? -1 : 0;
}

// 枚举帧大小
int insight9_receive_enum_frame_sizes(int cam_id, unsigned int pixelformat, int index, struct v4l2_frmsizeenum *frmsize) {
    if (!g_ctx.initialized) {
        fprintf(stderr, "[SDK][ERR] not initialized\n");
        return -1;
    }
    if (cam_id < 0 || cam_id >= CAM_NUM) {
        fprintf(stderr, "[SDK][ERR] invalid camera ID\n");
        return -1;
    }
    if (!frmsize) {
        fprintf(stderr, "[SDK][ERR] NULL frame size structure\n");
        return -1;
    }
    
    int fd = open(g_ctx.video_devs[cam_id], O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "[CAM%d][ERR] open failed: %s\n", cam_id, strerror(errno));
        return -1;
    }
    
    memset(frmsize, 0, sizeof(*frmsize));
    frmsize->index = index;
    frmsize->pixel_format = pixelformat;
    
    int ret = ioctl(fd, VIDIOC_ENUM_FRAMESIZES, frmsize);
    close(fd);
    
    return ret < 0 ? -1 : 0;
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

static void free_buffer_array(struct buffer **buffers, int *buffer_count) {
    if (!buffers || !*buffers) return;
    for (int i = 0; i < *buffer_count; i++) {
        if ((*buffers)[i].start) {
            munmap((*buffers)[i].start, (*buffers)[i].length);
            (*buffers)[i].start = NULL;
        }
    }
    free(*buffers);
    *buffers = NULL;
    *buffer_count = 0;
}

static uint32_t read_le32(const uint8_t *p) {
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static uint64_t parse_uvc_metadata_timestamp(const uint8_t *data, size_t size) {
    if (!data || size < 10) return 0;

    // Firmware writes the low 32 bits of ts_us into the UVC payload header:
    // bytes 2..5 are PTS, bytes 6..9 are the low SCR bytes.
    //
    // Linux UVC metadata nodes usually prepend struct uvc_meta_buf:
    //   u64 ns, u16 sof, u8 length, u8 flags, then the UVC payload header.
    // In that common layout, the device header "f8 8e ..." starts at data + 12.
    if (size >= 22 && data[10] >= 10 && data[10] <= size - 12) {
        const uint8_t *uvc_header = data + 12;
        uint32_t pts = read_le32(uvc_header + 2);
        if (pts != 0) return pts;
        uint32_t scr_low = read_le32(uvc_header + 6);
        if (scr_low != 0) return scr_low;
    }

    // Fallback for drivers that expose the UVC payload header directly.
    uint32_t pts = read_le32(data + 12);
    if (pts != 0) return pts;

    return 0;
}

static int init_metadata_capture(struct cam_ctx *ctx) {
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_META_CAPTURE;
    fmt.fmt.meta.dataformat = V4L2_META_FMT_UVC;
    fmt.fmt.meta.buffersize = METADATA_SIZE;

    if (ioctl(ctx->meta_fd, VIDIOC_S_FMT, &fmt) < 0) {
        fprintf(stderr, "[CAM%d][META][WARN] VIDIOC_S_FMT: %s\n", ctx->cam_id, strerror(errno));
    }

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_META_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(ctx->meta_fd, VIDIOC_REQBUFS, &req) < 0) {
        fprintf(stderr, "[CAM%d][META][ERR] VIDIOC_REQBUFS: %s\n", ctx->cam_id, strerror(errno));
        return -1;
    }

    if (req.count < 2) {
        fprintf(stderr, "[CAM%d][META][ERR] insufficient buffers (%u)\n", ctx->cam_id, req.count);
        return -1;
    }

    ctx->meta_buffers = (buffer*)calloc(req.count, sizeof(struct buffer));
    if (!ctx->meta_buffers) {
        fprintf(stderr, "[CAM%d][META][ERR] calloc: %s\n", ctx->cam_id, strerror(errno));
        return -1;
    }
    ctx->meta_buffer_count = req.count;

    for (int i = 0; i < ctx->meta_buffer_count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_META_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(ctx->meta_fd, VIDIOC_QUERYBUF, &buf) < 0) {
            fprintf(stderr, "[CAM%d][META][ERR] VIDIOC_QUERYBUF: %s\n", ctx->cam_id, strerror(errno));
            return -1;
        }

        ctx->meta_buffers[i].length = buf.length;
        ctx->meta_buffers[i].start = mmap(NULL, buf.length,
                                          PROT_READ | PROT_WRITE,
                                          MAP_SHARED, ctx->meta_fd, buf.m.offset);
        if (ctx->meta_buffers[i].start == MAP_FAILED) {
            fprintf(stderr, "[CAM%d][META][ERR] mmap: %s\n", ctx->cam_id, strerror(errno));
            return -1;
        }

        if (ioctl(ctx->meta_fd, VIDIOC_QBUF, &buf) < 0) {
            fprintf(stderr, "[CAM%d][META][ERR] VIDIOC_QBUF: %s\n", ctx->cam_id, strerror(errno));
            return -1;
        }
    }

    return 0;
}

static int start_metadata_capture(int fd) {
    int type = V4L2_BUF_TYPE_META_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        fprintf(stderr, "[META][ERR] VIDIOC_STREAMON: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

static int stop_metadata_capture(int fd) {
    int type = V4L2_BUF_TYPE_META_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
        fprintf(stderr, "[META][ERR] VIDIOC_STREAMOFF: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

static uint64_t read_latest_metadata_timestamp(struct cam_ctx *ctx) {
    if (ctx->meta_fd < 0 || !ctx->meta_buffers) return 0;

    uint64_t latest_ts = 0;
    while (1) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_META_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(ctx->meta_fd, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) break;
            if (errno == ENODEV) {
                fprintf(stderr, "[CAM%d][META][WARN] metadata device removed\n", ctx->cam_id);
                close(ctx->meta_fd);
                ctx->meta_fd = -1;
                free_buffer_array(&ctx->meta_buffers, &ctx->meta_buffer_count);
                break;
            }
            fprintf(stderr, "[CAM%d][META][ERR] VIDIOC_DQBUF: %s\n", ctx->cam_id, strerror(errno));
            break;
        }

        uint64_t ts = parse_uvc_metadata_timestamp(
            (uint8_t*)ctx->meta_buffers[buf.index].start,
            buf.bytesused);
        if (ts != 0) latest_ts = ts;

        if (ioctl(ctx->meta_fd, VIDIOC_QBUF, &buf) < 0) {
            fprintf(stderr, "[CAM%d][META][ERR] VIDIOC_QBUF: %s\n", ctx->cam_id, strerror(errno));
            close(ctx->meta_fd);
            ctx->meta_fd = -1;
            free_buffer_array(&ctx->meta_buffers, &ctx->meta_buffer_count);
            break;
        }
    }

    return latest_ts;
}

static void safe_cleanup_camera(struct cam_ctx *ctx) {
    if (!ctx) return;
    pthread_mutex_lock(&ctx->fd_lock);
    if (ctx->fd < 0) {
        pthread_mutex_unlock(&ctx->fd_lock);
        return;
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(ctx->fd, VIDIOC_STREAMOFF, &type);

    if (ctx->buffers) {
        for (int i = 0; i < ctx->buffer_count; i++) {
            if (ctx->buffers[i].start && ctx->buffers[i].start != MAP_FAILED) {
                munmap(ctx->buffers[i].start, ctx->buffers[i].length);
            }
        }
        free(ctx->buffers);
        ctx->buffers = NULL;
    }
    ctx->buffer_count = 0;

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = 0;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ioctl(ctx->fd, VIDIOC_REQBUFS, &req);

    close(ctx->fd);
    ctx->fd = -1;
    ctx->last_timestamp = 0;

    if (ctx->meta_fd >= 0) {
        close(ctx->meta_fd);
        ctx->meta_fd = -1;
    }
    free_buffer_array(&ctx->meta_buffers, &ctx->meta_buffer_count);

    pthread_mutex_unlock(&ctx->fd_lock);
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
    int reconnect_fails = 0;


    printf("[CAM%d] capture thread started, dev=%s meta=%s\n",
           ctx->cam_id, g_ctx.video_devs[ctx->cam_id], g_ctx.metadata_devs[ctx->cam_id]);

    while (g_ctx.running) {
        if (!g_ctx.cam_running[ctx->cam_id]) {
            break;
        }
        struct pollfd fds;
        fds.fd = ctx->fd;
        fds.events = POLLIN;
        const char *dev_path = g_ctx.video_devs[ctx->cam_id];
        if (ctx->fd < 0) {
            pthread_mutex_lock(&ctx->fd_lock);
            // Re-check inside the lock: stop_camera() may have run between
            // the check above and acquiring the lock, or another thread may
            // have already reopened it.
            if (!g_ctx.cam_running[ctx->cam_id]) {
                pthread_mutex_unlock(&ctx->fd_lock);
                break;
            }
            if (ctx->fd >= 0) {
                pthread_mutex_unlock(&ctx->fd_lock);
                continue;
            }
            printf("[CAM%d] device not open, opening %s\n", ctx->cam_id, dev_path);
            refresh_video_device_path(ctx->cam_id);
            dev_path = g_ctx.video_devs[ctx->cam_id];
            
            printf("[CAM%d][META] opening %s\n", ctx->cam_id, g_ctx.metadata_devs[ctx->cam_id]);
            ctx->meta_fd = open(g_ctx.metadata_devs[ctx->cam_id], O_RDWR | O_NONBLOCK);
            if (ctx->meta_fd < 0) {
                fprintf(stderr, "[CAM%d][META][ERR] open failed: %s\n", ctx->cam_id, strerror(errno));
                usleep(500000);
                continue;
            }
            if (init_metadata_capture(ctx) < 0 || start_metadata_capture(ctx->meta_fd) < 0) {
                fprintf(stderr, "[CAM%d][META][ERR] init/start failed\n", ctx->cam_id);
                close(ctx->meta_fd);
                ctx->meta_fd = -1;
                usleep(500000);
                continue;
            }

            printf("[CAM%d] reopening %s\n", ctx->cam_id, dev_path);
            ctx->fd = open(dev_path, O_RDWR);
            if (ctx->fd < 0) {
                fprintf(stderr, "[CAM%d][ERR] open failed: %s, retry in 1s\n",
                        ctx->cam_id, strerror(errno));
                stop_metadata_capture(ctx->meta_fd);
                close(ctx->meta_fd);
                ctx->meta_fd = -1;
                free_buffer_array(&ctx->meta_buffers, &ctx->meta_buffer_count);
                pthread_mutex_unlock(&ctx->fd_lock);
                reconnect_backoff_apply("CAM", ctx->cam_id, &reconnect_fails, "open failed",
                                         &g_ctx.cam_running[ctx->cam_id]);
                stop_metadata_capture(ctx->meta_fd);
                close(ctx->meta_fd);
                ctx->meta_fd = -1;
                free_buffer_array(&ctx->meta_buffers, &ctx->meta_buffer_count);
                continue;
            }
            if (init_capture(ctx) < 0) {
                close(ctx->fd);
                ctx->fd = -1;
                stop_metadata_capture(ctx->meta_fd);
                close(ctx->meta_fd);
                pthread_mutex_unlock(&ctx->fd_lock);
                reconnect_backoff_apply("CAM", ctx->cam_id, &reconnect_fails, "init_capture failed",
                                         &g_ctx.cam_running[ctx->cam_id]);
                ctx->meta_fd = -1;
                free_buffer_array(&ctx->meta_buffers, &ctx->meta_buffer_count);
                continue;
            }
            
            if (start_capture(ctx->fd) < 0) {
                stop_metadata_capture(ctx->meta_fd);
                close(ctx->meta_fd);
                ctx->meta_fd = -1;
                free_buffer_array(&ctx->meta_buffers, &ctx->meta_buffer_count);
                close(ctx->fd);
                ctx->fd = -1;
                pthread_mutex_unlock(&ctx->fd_lock);
                reconnect_backoff_apply("CAM", ctx->cam_id, &reconnect_fails, "failed to start stream",
                                         &g_ctx.cam_running[ctx->cam_id]);
                usleep(1000000);
                continue;
            }
            ctx->last_timestamp = 0;
            reconnect_fails = 0;   // reconnected successfully; reset backoff
            pthread_mutex_unlock(&ctx->fd_lock);
            printf("[CAM%d] device reinitialized\n", ctx->cam_id);
            frame_count = 0;
        }

        if (!g_ctx.running) break;
        int ret = poll(&fds, 1, 200);
        if (ret < 0) {
            if (errno == EINTR){
                if (!g_ctx.running) break;
                continue;
            }
            fprintf(stderr, "[CAM%d][ERR] poll: %s\n", ctx->cam_id, strerror(errno));
            close(ctx->fd);
            ctx->fd = -1;
            free_buffer_array(&ctx->buffers, &ctx->buffer_count);
            if (ctx->meta_fd >= 0) {
                close(ctx->meta_fd);
                ctx->meta_fd = -1;
            }
            free_buffer_array(&ctx->meta_buffers, &ctx->meta_buffer_count);
            safe_cleanup_camera(ctx);
            continue;
        } else if (ret == 0) {
            if (!g_ctx.cam_running[ctx->cam_id]) {
                break;
            }
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
            if (errno == EBUSY || errno == ENODEV || errno == EBADF) {
                fprintf(stderr, "[CAM%d][WARN] device removed, waiting for reconnect\n", ctx->cam_id);
                close(ctx->fd);
                ctx->fd = -1;
                free_buffer_array(&ctx->buffers, &ctx->buffer_count);
                if (ctx->meta_fd >= 0) {
                    close(ctx->meta_fd);
                    ctx->meta_fd = -1;
                }
                free_buffer_array(&ctx->meta_buffers, &ctx->meta_buffer_count);
                safe_cleanup_camera(ctx);
                reconnect_backoff_apply("CAM", ctx->cam_id, &reconnect_fails, "DQBUF reset",
                                         &g_ctx.cam_running[ctx->cam_id]);
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
                safe_cleanup_camera(ctx);
            }
            continue;
        }

        uint64_t timestamp = read_latest_metadata_timestamp(ctx);
        uint64_t right_timestamp = 0;
        if (ctx->meta_fd < 0) {
            if (ioctl(ctx->fd, VIDIOC_QBUF, &buf) < 0) {
                fprintf(stderr, "[CAM%d][ERR] VIDIOC_QBUF: %s\n", ctx->cam_id, strerror(errno));
            }
            close(ctx->fd);
            ctx->fd = -1;
            free_buffer_array(&ctx->buffers, &ctx->buffer_count);
            continue;
        }

        // Filter 2: drop buffers until matching UVC metadata provides a usable timestamp.
        if (timestamp == 0) {
            if (ioctl(ctx->fd, VIDIOC_QBUF, &buf) < 0) {
                fprintf(stderr, "[CAM%d][ERR] VIDIOC_QBUF: %s\n", ctx->cam_id, strerror(errno));
                close(ctx->fd);
                ctx->fd = -1;
                safe_cleanup_camera(ctx);
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
                safe_cleanup_camera(ctx);
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
                         g_ctx.img_userdata);
        }

        frame_count++;

        if (ioctl(ctx->fd, VIDIOC_QBUF, &buf) < 0) {
            fprintf(stderr, "[CAM%d][ERR] VIDIOC_QBUF: %s\n", ctx->cam_id, strerror(errno));
            close(ctx->fd);
            ctx->fd = -1;
            safe_cleanup_camera(ctx);
        }

        if (!g_ctx.cam_running[ctx->cam_id]) {
            break;
        }
    }

    // Same cleanup as the mid-loop error paths, just done once on exit;
    // routing through safe_cleanup_camera() keeps it lock-protected and
    // avoids duplicating the close/munmap/REQBUFS(0) sequence here.
    safe_cleanup_camera(ctx);

    printf("[CAM%d] capture thread exited\n", ctx->cam_id);
    return NULL;
}

int switch_camera_fps(int cam_id, int new_fps) {
    if (cam_id < 0 || cam_id >= CAM_NUM) return -1;
    struct cam_ctx *ctx = &g_ctx.cams[cam_id];

    printf("[SDK] Switching camera %d to %d FPS...\n", cam_id, new_fps);

    g_ctx.cam_running[cam_id] = false;
    if (ctx->tid) {
        pthread_join(ctx->tid, NULL);
        ctx->tid = 0;
    }

    if (cam_id == 0) {
        g_ctx.config.rgb_config.fps = new_fps;
    } else if (cam_id == 1) {
        g_ctx.config.gray_config.fps = new_fps;
    } else if (cam_id == 2) {
        g_ctx.config.depth_config.fps = new_fps;
    }

    g_ctx.cam_running[cam_id] = true;
    if (pthread_create(&ctx->tid, NULL, capture_thread, ctx) != 0) {
        fprintf(stderr, "[SDK][ERR] Failed to recreate capture thread for cam %d\n", cam_id);
        g_ctx.cam_running[cam_id] = false;
        return -1;
    }

    printf("[SDK] Camera %d switched to %d FPS successfully\n", cam_id, new_fps);
    return 0;
}

// ==================== SDK API Implementation ====================
int insight9_receive_init(const insight9_config_t* config) {
    if (g_ctx.initialized) {
        fprintf(stderr, "[SDK][WARN] already initialized\n");
        return -1;
    }

    if (!config) {
        fprintf(stderr, "[SDK][ERR] Config is NULL, using default\n");
        return insight9_receive_init_default();
    }

    if (config->rgb_config.width <= 0 || config->rgb_config.height <= 0 ||
        config->gray_config.width <= 0 || config->gray_config.height <= 0 ||
        config->depth_config.width <= 0 || config->depth_config.height <= 0) {
        fprintf(stderr, "[SDK] Invalid resolution in config\n");
        return -1;
    }

    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.running = false;
    for (int i = 0; i < CAM_NUM; ++i) {
        g_ctx.cam_running[i] = false;
        g_ctx.first_frame_received[i] = false;
        clock_gettime(CLOCK_MONOTONIC, &g_ctx.last_frame_time[i]);
    }
    g_ctx.config = *config;

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
    int metadata_idx[] = {1, 3, 5};
    for (int i = 0; i < CAM_NUM; i++) {
        if (selected_idx[i] >= uvc_count || metadata_idx[i] >= uvc_count) {
            fprintf(stderr, "[SDK][ERR] cannot select UVC video/metadata pair indexes %d/%d\n",
                    selected_idx[i], metadata_idx[i]);
            return -1;
        }
        strcpy(g_ctx.video_devs[i], uvc_list[selected_idx[i]]);
        strcpy(g_ctx.metadata_devs[i], uvc_list[metadata_idx[i]]);
        g_ctx.video_usb_paths[i][0] = '\0';
        g_ctx.metadata_usb_paths[i][0] = '\0';
        if (get_video_usb_device_path(g_ctx.video_devs[i], g_ctx.video_usb_paths[i], MAX_PATH) < 0) {
            g_ctx.video_usb_paths[i][0] = '\0';
        }
        if (get_video_usb_device_path(g_ctx.metadata_devs[i], g_ctx.metadata_usb_paths[i], MAX_PATH) < 0) {
            g_ctx.metadata_usb_paths[i][0] = '\0';
        }
        printf("[SDK] selected UVC[%d]=%s metadata=%s\n",
               i, g_ctx.video_devs[i], g_ctx.metadata_devs[i]);
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
        g_ctx.cams[i].meta_fd = -1;
        if (i == 0) {
            g_ctx.cams[i].width = config->depth_config.width;
            g_ctx.cams[i].height = config->depth_config.height;
            g_ctx.cams[i].format = config->depth_config.pixel_format;
        } else if (i == 1) {
            g_ctx.cams[i].width = config->gray_config.width;
            g_ctx.cams[i].height = config->gray_config.height;
            g_ctx.cams[i].format = config->gray_config.pixel_format;
        } else if (i == 2) {
            g_ctx.cams[i].width = config->rgb_config.width;
            g_ctx.cams[i].height = config->rgb_config.height;
            g_ctx.cams[i].format = config->rgb_config.pixel_format;
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

int insight9_receive_init_default(void) {
    if (g_ctx.initialized) {
        fprintf(stderr, "[SDK][WARN] already initialized\n");
        return -1;
    }

    memset(&g_ctx, 0, sizeof(g_ctx));
    for (int i = 0; i < CAM_NUM; ++i) {
        g_ctx.cam_running[i] = false;
        g_ctx.first_frame_received[i] = false;
        clock_gettime(CLOCK_MONOTONIC, &g_ctx.last_frame_time[i]);
    }

    g_ctx.config.rgb_config.fps = 30;
    g_ctx.config.gray_config.fps = 30;
    g_ctx.config.depth_config.fps = 30;

    char uvc_list[10][MAX_PATH] = {{0}};
    int uvc_count = find_uvc_devices_by_vid_pid(VENDOR_ID, PRODUCT_ID, uvc_list, 10);
    if (uvc_count < 6) {
        fprintf(stderr, "[SDK][ERR] need >= 6 UVC video/metadata nodes with VID=0x%04x PID=0x%04x, found %d\n",
                VENDOR_ID, PRODUCT_ID, uvc_count);
        return -1;
    }
    // Select video nodes 0/2/4 and their matching metadata nodes 1/3/5.
    int selected_idx[] = {0, 2, 4};
    int metadata_idx[] = {1, 3, 5};
    for (int i = 0; i < CAM_NUM; i++) {
        if (selected_idx[i] >= uvc_count) {
            fprintf(stderr, "Error: cannot select 3 devices by skipping one (need at least 6 devices)\n");
            return -1;
        }
        strcpy(g_ctx.video_devs[i], uvc_list[selected_idx[i]]);
        strcpy(g_ctx.metadata_devs[i], uvc_list[metadata_idx[i]]);
        g_ctx.video_usb_paths[i][0] = '\0';
        g_ctx.metadata_usb_paths[i][0] = '\0';
        if (get_video_usb_device_path(g_ctx.video_devs[i], g_ctx.video_usb_paths[i], MAX_PATH) < 0) {
            g_ctx.video_usb_paths[i][0] = '\0';
        }
        if (get_video_usb_device_path(g_ctx.metadata_devs[i], g_ctx.metadata_usb_paths[i], MAX_PATH) < 0) {
            g_ctx.metadata_usb_paths[i][0] = '\0';
        }
        printf("Selected UVC[%d]: video=%s metadata=%s\n", 
               i, g_ctx.video_devs[i], g_ctx.metadata_devs[i]);
    }

    // Initialize camera contexts.
    for (int i = 0; i < CAM_NUM; i++) {
        g_ctx.cams[i].cam_id = i;
        g_ctx.cams[i].fd = -1;
    }

    for (int i = 0; i < CAM_NUM; ++i) {
        if (g_ctx.video_devs[i][0] == '\0') continue;
        int fd = open(g_ctx.video_devs[i], O_RDWR);
        if (fd < 0) {
            fprintf(stderr, "[CAM%d][WARN] open for format enum failed: %s\n", i, strerror(errno));
            continue;
        }
        g_ctx.cams[i].fd = fd;
        if (get_camera_formats_info(&g_ctx.cams[i]) < 0) {
            fprintf(stderr, "[CAM%d][WARN] failed to get formats info during init\n", i);
        }
        close(fd);
        g_ctx.cams[i].fd = -1;
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

    g_ctx.running = true;

    for (int i = 0; i < CAM_NUM; i++) {
        g_ctx.cam_running[i] = true;
        g_ctx.first_frame_received[i] = false;
        clock_gettime(CLOCK_MONOTONIC, &g_ctx.last_frame_time[i]);
        pthread_create(&g_ctx.cams[i].tid, NULL, capture_thread, &g_ctx.cams[i]);
    }

    hid_init();

    printf("[SDK] started\n");
    return 0;
}

int insight9_receive_start_camera(int cam_id) {
    if (!g_ctx.initialized) return -1;
    if (cam_id < 0 || cam_id >= CAM_NUM) return -1;
    if (g_ctx.cam_running[cam_id]) return 0;

    struct cam_ctx *ctx = &g_ctx.cams[cam_id];
    
    if (ctx->fd < 0) {
        pthread_mutex_lock(&ctx->fd_lock);
        if (ctx->fd >= 0) {
            // Someone else (e.g. capture_thread's own reopen path) already
            // got there first; nothing left for us to do here.
            pthread_mutex_unlock(&ctx->fd_lock);
        } else {
            refresh_video_device_path(cam_id);
            const char *dev_path = g_ctx.video_devs[cam_id];
            ctx->fd = open(dev_path, O_RDWR);
            if (ctx->fd < 0) {
                fprintf(stderr, "[CAM%d][ERR] open failed: %s\n", cam_id, strerror(errno));
                pthread_mutex_unlock(&ctx->fd_lock);
                return -1;
            }
        }

        if (cam_id == 0) {
            ctx->width = g_ctx.config.depth_config.width;
            ctx->height = g_ctx.config.depth_config.height;
            ctx->format = g_ctx.config.depth_config.pixel_format;
        } else if (cam_id == 1) {
            ctx->width = g_ctx.config.gray_config.width;
            ctx->height = g_ctx.config.gray_config.height;
            ctx->format = g_ctx.config.gray_config.pixel_format;
        } else {
            ctx->width = g_ctx.config.rgb_config.width;
            ctx->height = g_ctx.config.rgb_config.height;
            ctx->format = g_ctx.config.rgb_config.pixel_format;
        }

        if (init_capture(ctx) < 0) {
            close(ctx->fd);
            ctx->fd = -1;
            pthread_mutex_unlock(&ctx->fd_lock);
            return -1;
        }

        if (start_capture(ctx->fd) < 0) {
            close(ctx->fd);
            ctx->fd = -1;
            pthread_mutex_unlock(&ctx->fd_lock);
            return -1;
        }

        ctx->last_timestamp = 0;
        pthread_mutex_unlock(&ctx->fd_lock);

        ctx->last_timestamp = 0;
    }

    g_ctx.cam_running[cam_id] = true;
    g_ctx.first_frame_received[cam_id] = false;
    clock_gettime(CLOCK_MONOTONIC, &g_ctx.last_frame_time[cam_id]);
    
    if (g_ctx.video_tids[cam_id]) {
        pthread_join(g_ctx.video_tids[cam_id], NULL);
    }
    pthread_create(&g_ctx.video_tids[cam_id], NULL, capture_thread, ctx);
    
    printf("[CAM%d] started\n", cam_id);
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

const char *insight9_receive_get_metadata_dev(int cam_id) {
    if (!g_ctx.initialized) {
        return NULL;
    }
    if (cam_id < 0 || cam_id >= CAM_NUM) {
        return NULL;
    }
    return g_ctx.metadata_devs[cam_id][0] ? g_ctx.metadata_devs[cam_id] : NULL;
}

int insight9_receive_read_metadata_timestamp(int cam_id, uint64_t *timestamp) {
    if (!g_ctx.initialized || !timestamp || cam_id < 0 || cam_id >= CAM_NUM) {
        return -1;
    }

    *timestamp = 0;

    struct cam_ctx tmp;
    memset(&tmp, 0, sizeof(tmp));
    tmp.cam_id = cam_id;
    tmp.meta_fd = open(g_ctx.metadata_devs[cam_id], O_RDWR | O_NONBLOCK);
    if (tmp.meta_fd < 0) {
        fprintf(stderr, "[CAM%d][META][ERR] open %s: %s\n",
                cam_id, g_ctx.metadata_devs[cam_id], strerror(errno));
        return -1;
    }

    if (init_metadata_capture(&tmp) < 0 || start_metadata_capture(tmp.meta_fd) < 0) {
        if (tmp.meta_buffers) {
            free_buffer_array(&tmp.meta_buffers, &tmp.meta_buffer_count);
        }
        if (tmp.meta_fd >= 0) close(tmp.meta_fd);
        return -1;
    }

    struct pollfd pfd = {.fd = tmp.meta_fd, .events = POLLIN};
    int ret = poll(&pfd, 1, 1000);
    if (ret > 0 && (pfd.revents & POLLIN)) {
        *timestamp = read_latest_metadata_timestamp(&tmp);
    }

    stop_metadata_capture(tmp.meta_fd);
    free_buffer_array(&tmp.meta_buffers, &tmp.meta_buffer_count);
    close(tmp.meta_fd);

    return *timestamp != 0 ? 0 : -1;
}

void insight9_receive_all_stop_camera(int cam_id) {
    if (cam_id < 0 || cam_id >= CAM_NUM) return;
    
    g_ctx.cam_running[cam_id] = false;

    usleep(200000);  // 200ms
    struct cam_ctx *ctx = &g_ctx.cams[cam_id];
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
        ctx->buffer_count = 0;
    }
    safe_cleanup_camera(ctx);
    
    if (g_ctx.video_tids[cam_id]) {
        pthread_join(g_ctx.video_tids[cam_id], NULL);
        g_ctx.video_tids[cam_id] = 0;
    }
    
    printf("[CAM%d] stopped\n", cam_id);
}

int insight9_receive_restart_camera(int cam_id) {
    if (!g_ctx.initialized) return -1;
    if (cam_id < 0 || cam_id >= CAM_NUM) return -1;
    
    insight9_receive_all_stop_camera(cam_id);
    usleep(100000);
    return insight9_receive_start_camera(cam_id);
}

int insight9_receive_switch_camera_fps(int cam_id, int fps) {
    if (!g_ctx.initialized) return -1;
    if (cam_id < 0 || cam_id >= CAM_NUM) return -1;
    if (fps <= 0) return -1;

    printf("[SDK] Switching camera %d to %d fps...\n", cam_id, fps);

    // 1. 停止相机（使用安全清理）
    insight9_receive_all_stop_camera(cam_id);
    
    // 2. 等待设备完全释放
    usleep(1000000);  // 200ms
    
    if (insight9_receive_set_camera_fps(cam_id, fps) != 0) {
        fprintf(stderr, "[SDK] Failed to set camera %d FPS to %d\n", cam_id, fps);
        return -1;
    }
    
    int ret = insight9_receive_restart_camera(cam_id);
    if (ret == 0) {
        printf("[SDK] Camera %d switched to %d FPS successfully\n", cam_id, fps);
    } else {
        fprintf(stderr, "[SDK] Failed to restart camera %d after FPS switch\n", cam_id);
    }
    
    return ret;
}

int insight9_receive_is_camera_running(int cam_id) {
    if (cam_id < 0 || cam_id >= CAM_NUM) return 0;
    return g_ctx.cam_running[cam_id] ? 1 : 0;
}

void insight9_receive_all_stop(void) {
    if (!g_ctx.running) return;
    g_ctx.running = false;

    printf("[SDK] Stopping all cameras...\n");

    for (int i = 0; i < CAM_NUM; i++) {
        g_ctx.cam_running[i] = false;
        struct cam_ctx *ctx = &g_ctx.cams[i];
        if (ctx->fd >= 0) {
            stop_capture(ctx->fd);
            close(ctx->fd);
            ctx->fd = -1;
        }
        if (ctx->meta_fd >= 0) {
            stop_metadata_capture(ctx->meta_fd);
            close(ctx->meta_fd);
            ctx->meta_fd = -1;
        }
        if (g_ctx.video_tids[i]) {
            pthread_join(g_ctx.video_tids[i], NULL);
            g_ctx.video_tids[i] = 0;
        }
        safe_cleanup_camera(&g_ctx.cams[i]);
    }

    for (int i = 0; i < HID_NUM; i++) {
        if (g_ctx.hid_tids[i]) {
            pthread_join(g_ctx.hid_tids[i], NULL);
            g_ctx.hid_tids[i] = 0;
        }
    }

    printf("[SDK] stopped\n");
}

void insight9_receive_cleanup(void) {
    if (!g_ctx.initialized) return;

    printf("[SDK] Cleaning up...\n");

    if (g_ctx.running) {
        insight9_receive_all_stop();
    }

    for (int i = 0; i < CAM_NUM; i++) {
        struct cam_ctx *ctx = &g_ctx.cams[i];
        if (ctx->fd >= 0) {
            stop_capture(ctx->fd);
            // free_buffer_array(&ctx->buffers, &ctx->buffer_count);
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
        if (ctx->meta_fd >= 0) {
            stop_metadata_capture(ctx->meta_fd);
            free_buffer_array(&ctx->meta_buffers, &ctx->meta_buffer_count);
            close(ctx->meta_fd);
            ctx->meta_fd = -1;
        }
    }

    if (g_ctx.xu_control) {
        g_ctx.xu_control->close();
        delete g_ctx.xu_control;
        g_ctx.xu_control = nullptr;
    }

    g_ctx.initialized = 0;
    printf("[SDK] cleaned up\n");
}

int insight9_receive_set_camera_fps(int cam_id, int fps) {
    if (!g_ctx.initialized) return -1;
    if (cam_id < 0 || cam_id >= CAM_NUM) return -1;
    if (fps <= 0) return -1;
    
    if (cam_id == 0) {
        g_ctx.config.rgb_config.fps = fps;
    } else if (cam_id == 1) {
        g_ctx.config.gray_config.fps = fps;
    } else {
        g_ctx.config.depth_config.fps = fps;
    }
    
    printf("[SDK] Set camera %d FPS to %d\n", cam_id, fps);
    return 0;
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
    memcpy(params, &xu_params, sizeof(viewer::camera_params));
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
    memcpy(params, &xu_params, sizeof(viewer::camera_params));
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
    memcpy(&xu_params, params, sizeof(viewer::camera_params));
    viewer::printParams(xu_params);
}

int insight9_receive_get_camera_calib(int cam_idx, camera_calib *calib) {
    static_assert(sizeof(camera_calib) == sizeof(viewer::camera_calib),
                  "camera_calib layout mismatch between C API and UVC payload");
    if (!calib || cam_idx < 0 || cam_idx >= viewer::kCalibCamCount) return -1;
    if (ensure_xu_available() != 0) return -1;
    viewer::camera_calib xu_calib;
    if (!g_ctx.xu_control->readCameraCalib(static_cast<uint8_t>(cam_idx), xu_calib)) return -1;
    memcpy(calib, &xu_calib, sizeof(camera_calib));
    return 0;
}

void insight9_receive_print_camera_calib(const camera_calib *calib) {
    if (!calib) return;
    viewer::camera_calib xu_calib;
    memcpy(&xu_calib, calib, sizeof(xu_calib));
    viewer::printCalib(xu_calib);
}

// Rasterize one triangle into the aligned-depth output, interpolating the RGB-
// frame depth across it with a nearest-surface z-buffer. Vertices are given as
// projected RGB pixel coords (x,y, float) plus their RGB-frame depth z (mm).
static inline void align_raster_triangle(uint16_t *out, int W, int H,
                                         float x0, float y0, float z0,
                                         float x1, float y1, float z1,
                                         float x2, float y2, float z2) {
    int minx = (int)floorf(fminf(x0, fminf(x1, x2)));
    int maxx = (int)ceilf (fmaxf(x0, fmaxf(x1, x2)));
    int miny = (int)floorf(fminf(y0, fminf(y1, y2)));
    int maxy = (int)ceilf (fmaxf(y0, fmaxf(y1, y2)));
    if (minx < 0) minx = 0;
    if (miny < 0) miny = 0;
    if (maxx > W - 1) maxx = W - 1;
    if (maxy > H - 1) maxy = H - 1;
    if (minx > maxx || miny > maxy) return;

    float area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
    if (fabsf(area) < 1e-6f) return;          // degenerate
    float inv_area = 1.0f / area;

    for (int py = miny; py <= maxy; py++) {
        for (int px = minx; px <= maxx; px++) {
            float fx = px + 0.5f, fy = py + 0.5f;
            // Barycentric weights via edge cross-products (same winding as area).
            float w0 = (x1 - fx) * (y2 - fy) - (x2 - fx) * (y1 - fy);
            float w1 = (x2 - fx) * (y0 - fy) - (x0 - fx) * (y2 - fy);
            float w2 = (x0 - fx) * (y1 - fy) - (x1 - fx) * (y0 - fy);
            bool inside = (w0 >= 0 && w1 >= 0 && w2 >= 0) ||
                          (w0 <= 0 && w1 <= 0 && w2 <= 0);
            if (!inside) continue;

            float z = (w0 * z0 + w1 * z1 + w2 * z2) * inv_area;
            if (z <= 0.0f) continue;
            uint16_t zq = (z > 65535.0f) ? 65535 : (uint16_t)(z + 0.5f);
            uint16_t *dst = &out[(size_t)py * W + px];
            if (*dst == 0 || zq < *dst) *dst = zq;
        }
    }
}

int insight9_receive_align_depth_to_rgb(const uint16_t *depth,
                                        int depth_w, int depth_h,
                                        const camera_calib *left_calib,
                                        const camera_calib *rgb_calib,
                                        uint16_t *aligned_out,
                                        int rgb_w, int rgb_h) {
    if (!depth || !left_calib || !rgb_calib || !aligned_out ||
        depth_w <= 0 || depth_h <= 0 || rgb_w <= 0 || rgb_h <= 0) {
        return -1;
    }

    // Start with all-invalid (0) output.
    memset(aligned_out, 0, (size_t)rgb_w * rgb_h * sizeof(uint16_t));

    // Intrinsics (row-major k = [fx 0 cx; 0 fy cy; 0 0 1]). Streams are
    // rectified (p == k), so no distortion is applied.
    const double dfx = left_calib->intrinsics.k[0];
    const double dfy = left_calib->intrinsics.k[4];
    const double dcx = left_calib->intrinsics.k[2];
    const double dcy = left_calib->intrinsics.k[5];
    const double rfx = rgb_calib->intrinsics.k[0];
    const double rfy = rgb_calib->intrinsics.k[4];
    const double rcx = rgb_calib->intrinsics.k[2];
    const double rcy = rgb_calib->intrinsics.k[5];
    if (dfx == 0.0 || dfy == 0.0) return -1;

    // RGB extrinsic: parent=camera_camera_left, child=camera_camera_rgb. In ROS
    // convention (t, q) maps a point rgb->left: p_left = R*p_rgb + t. We need
    // left->rgb, i.e. the inverse: p_rgb = R^T * (p_left - t).
    const double *q = rgb_calib->extrinsics.rotation;     // x, y, z, w
    const double *t = rgb_calib->extrinsics.translation;  // meters
    const double qx = q[0], qy = q[1], qz = q[2], qw = q[3];
    const double R00 = 1 - 2*(qy*qy + qz*qz);
    const double R01 = 2*(qx*qy - qz*qw);
    const double R02 = 2*(qx*qz + qy*qw);
    const double R10 = 2*(qx*qy + qz*qw);
    const double R11 = 1 - 2*(qx*qx + qz*qz);
    const double R12 = 2*(qy*qz - qx*qw);
    const double R20 = 2*(qx*qz - qy*qw);
    const double R21 = 2*(qy*qz + qx*qw);
    const double R22 = 1 - 2*(qx*qx + qy*qy);
    const double tx = t[0] * 1000.0, ty = t[1] * 1000.0, tz = t[2] * 1000.0;

    // Pass 1: project every depth pixel to RGB pixel coords + RGB-frame depth.
    // The depth grid is treated as a mesh; these are its vertices.
    const size_t n = (size_t)depth_w * depth_h;
    std::vector<float>   VX(n), VY(n), VZ(n);
    std::vector<uint8_t> OK(n, 0);
    for (int v = 0; v < depth_h; v++) {
        const uint16_t *row = depth + (size_t)v * depth_w;
        for (int u = 0; u < depth_w; u++) {
            size_t i = (size_t)v * depth_w + u;
            uint16_t d = row[u];
            if (d == 0) continue;              // no measurement

            // Deproject to the LEFT frame (mm), then left -> rgb: R^T*(p - t).
            double z = (double)d;
            double x = (u - dcx) * z / dfx;
            double y = (v - dcy) * z / dfy;
            double ax = x - tx, ay = y - ty, az = z - tz;
            double xr = R00*ax + R10*ay + R20*az;
            double yr = R01*ax + R11*ay + R21*az;
            double zr = R02*ax + R12*ay + R22*az;
            if (zr <= 0.0) continue;           // behind the RGB camera

            VX[i] = (float)(rfx * xr / zr + rcx);
            VY[i] = (float)(rfy * yr / zr + rcy);
            VZ[i] = (float)zr;
            OK[i] = 1;
        }
    }

    // Pass 2: rasterize the mesh. Each 2x2 cell -> two triangles. Triangles that
    // straddle a depth discontinuity (an object edge) are dropped to avoid
    // "rubber sheet" stretching between foreground and background.
    const float EDGE_REL = 0.05f;              // max 5% depth jump within a triangle
    auto emit = [&](size_t a, size_t b, size_t c) {
        if (!OK[a] || !OK[b] || !OK[c]) return;
        float za = VZ[a], zb = VZ[b], zc = VZ[c];
        float zmin = fminf(za, fminf(zb, zc));
        float zmax = fmaxf(za, fmaxf(zb, zc));
        if (zmin <= 0.0f || (zmax - zmin) > EDGE_REL * zmin) return;
        align_raster_triangle(aligned_out, rgb_w, rgb_h,
                              VX[a], VY[a], za,
                              VX[b], VY[b], zb,
                              VX[c], VY[c], zc);
    };
    for (int v = 0; v + 1 < depth_h; v++) {
        for (int u = 0; u + 1 < depth_w; u++) {
            size_t i00 = (size_t)v * depth_w + u;
            size_t i10 = i00 + 1;
            size_t i01 = i00 + depth_w;
            size_t i11 = i01 + 1;
            emit(i00, i10, i11);
            emit(i00, i11, i01);
        }
    }
    return 0;
}

int insight9_receive_get_current_fps(int* fps) {
    if (!fps) return -1;
    if (ensure_xu_available() != 0) return -1;
    uint8_t val;
    if (!g_ctx.xu_control->readCurrentFps(val)) return -1;
    const int validFps[] = {0, 20, 30, 40, 50};
    if (val >= 0 && val < (int)(sizeof(validFps)/sizeof(validFps[0]))) {
        *fps = validFps[val];
    } else {
        *fps = 0;
    }
    return 0;
}

int insight9_receive_get_vio_status(int* status) {
    if (!status) return -1;
    if (ensure_xu_available() != 0) return -1;
    uint8_t val;
    if (!g_ctx.xu_control->readVioStatus(val)) return -1;
    *status = val;
    return 0;
}

const char* insight9_receive_get_hardware_type(void) {
    static std::string result;
    viewer::camera_params xu_params;
    
    if (!g_ctx.xu_control || !g_ctx.xu_control->isOpen()) {
        return "unknown";
    }
    
    if (!g_ctx.xu_control->readCurrentCameraParams(xu_params)) {
        return "unknown";
    }
    
    uint8_t model = xu_params.hardware_model;
    const char* models[] = {"Insight 9", "Insight 7", "Insight 7p", "Insight 3u"};
    if (model < 4) {
        result = models[model];
        return result.c_str();
    }
    return "unknown";
}

} // extern "C"
