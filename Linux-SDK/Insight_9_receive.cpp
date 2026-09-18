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
#include <algorithm>
#include "Insight_9_receive_internal.h"

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

// ---- Depth camera (cam_id 0) gets a more conservative backoff ----
// cam_id 0 also owns the XU control node (see reopenXUControlLocked()), so
// every reconnect it does also re-opens the shared XU control channel under
// the single global xu_mutex. A flapping depth stream therefore doesn't just
// hammer its own video node - it starves every other XU caller (get/set
// params, FPS reads, VIO status polling) and can generate enough USB control
// traffic to disturb the other two streams sharing the same host controller.
// So: start with a longer delay, grow to a higher ceiling, and require the
// stream to actually stay up for a while before we trust it and reset the
// failure counter.
#define DEPTH_RECONNECT_BACKOFF_BASE_MS 3000    // first retry delay for cam0
#define DEPTH_RECONNECT_BACKOFF_MAX_MS  60000   // delay ceiling for cam0
#define DEPTH_MIN_STABLE_MS             3000    // must run this long before a reconnect counts as "recovered"
#define XU_REOPEN_MIN_INTERVAL_MS       5000    // don't reopen XU more often than this

sdk_ctx_t g_ctx{};
// TYPE_UNKNOWN must stay 0: cam_type[] is a zero-initialised global, and an
// unclassified camera must never match a node during hot-plug re-detection.
enum CameraType { TYPE_UNKNOWN = 0, TYPE_RGB, TYPE_LEFT, TYPE_DEPTH };
CameraType cam_type[CAM_NUM];

// ==================== Stream presence / late binding ====================
// Insight 9 publishes three VideoStreaming interfaces (depth / gray / RGB) and
// therefore six v4l2 nodes. Other units in the family publish fewer - an
// Insight 3u has no RGB function at all, so Linux shows four nodes and Windows
// shows a single composite device. The old code hard-coded video indexes 0/2/4
// with metadata siblings 1/3/5 and refused to start on anything else.
//
// cam_bound[i] now says whether logical camera i is currently attached to a
// real node. It starts false, is filled in by try_bind_cameras() at init, and
// can flip to true later: the capture threads call try_bind_cameras() on every
// reconnect attempt, so the SDK can initialise with no device attached at all
// and pick the streams up when they appear.
static bool cam_bound[CAM_NUM];
// Which stream families the node behind each camera advertises. A camera can
// legitimately offer more than one (Insight 3u's stereo node does Y8I and
// YUYV); switching between them is a normal set_camera_format + restart.
static int cam_families[CAM_NUM];
static pthread_mutex_t g_bind_lock = PTHREAD_MUTEX_INITIALIZER;

// Which config slot a logical camera draws from. Spelled out because the
// cam_id -> config mapping was inverted in several places.
typedef enum { PLACE_DEPTH, PLACE_GRAY, PLACE_RGB } video_place_t;

// Fixed logical ids used by the image callback: 0 = DEPTH, 1 = GRAY, 2 = RGB.
static int cam_id_for_type(CameraType t) {
    switch (t) {
        case TYPE_DEPTH: return 0;
        case TYPE_LEFT:  return 1;
        case TYPE_RGB:   return 2;
        default:         return -1;
    }
}

static CameraType cam_type_for_id(int cam_id) {
    switch (cam_id) {
        case 0:  return TYPE_DEPTH;
        case 1:  return TYPE_LEFT;
        case 2:  return TYPE_RGB;
        default: return TYPE_UNKNOWN;
    }
}

// Compute the exponential backoff delay (ms) for the Nth consecutive failure
// (attempt counted from 1), capped at max_ms, starting from base_ms.
static int reconnect_backoff_ms_ex(int attempt, int base_ms, int max_ms) {
    if (attempt < 1) attempt = 1;
    long delay = base_ms;
    for (int i = 1; i < attempt && delay < max_ms; ++i)
        delay <<= 1;
    if (delay > max_ms) delay = max_ms;
    return (int)delay;
}

// Back-compat wrapper using the default (non-depth) backoff range.
static int reconnect_backoff_ms(int attempt) {
    return reconnect_backoff_ms_ex(attempt, RECONNECT_BACKOFF_BASE_MS, RECONNECT_BACKOFF_MAX_MS);
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
static void reconnect_backoff_apply_ex(const char *tag, int id, int *fails, const char *reason,
                                        int base_ms, int max_ms,
                                        std::atomic<bool> *extra_stop = nullptr) {
    (*fails)++;
    int delay = reconnect_backoff_ms_ex(*fails, base_ms, max_ms);
    if (RECONNECT_MAX_ATTEMPTS <= 0 || *fails <= RECONNECT_MAX_ATTEMPTS) {
        fprintf(stderr, "[%s%d][ERR] %s, retry in %dms (attempt %d)\n",
                tag, id, reason, delay, *fails);
    } else if (*fails == RECONNECT_MAX_ATTEMPTS + 1) {
        fprintf(stderr, "[%s%d][ERR] %s failed %d times, slowing to %ds retry to avoid USB storm\n",
                tag, id, reason, RECONNECT_MAX_ATTEMPTS, max_ms / 1000);
    }
    interruptible_sleep_ms(delay, extra_stop);
}

// Back-compat wrapper using the default (non-depth) backoff range.
static void reconnect_backoff_apply(const char *tag, int id, int *fails, const char *reason,
                                     std::atomic<bool> *extra_stop = nullptr) {
    reconnect_backoff_apply_ex(tag, id, fails, reason,
                                RECONNECT_BACKOFF_BASE_MS, RECONNECT_BACKOFF_MAX_MS, extra_stop);
}

// cam_id 0 hosts the shared XU control node - give it the more conservative
// depth backoff range so a flapping depth stream doesn't keep re-opening XU
// (and hogging xu_mutex) at the same fast cadence as the other two streams.
static void reconnect_backoff_apply_for_cam(struct cam_ctx *ctx, int *fails, const char *reason) {
    if (ctx->cam_id == 0) {
        reconnect_backoff_apply_ex("CAM", ctx->cam_id, fails, reason,
                                    DEPTH_RECONNECT_BACKOFF_BASE_MS, DEPTH_RECONNECT_BACKOFF_MAX_MS,
                                    &g_ctx.cam_running[ctx->cam_id]);
    } else {
        reconnect_backoff_apply("CAM", ctx->cam_id, fails, reason,
                                 &g_ctx.cam_running[ctx->cam_id]);
    }
}

// Note: reopenXUControlThrottled() is defined further below, right after
// reopenXUControlLocked() (which it wraps), to avoid a forward declaration.

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

static const char* pixel_format_to_string(PixelFormat pf) {
    switch (pf) {
        case PixelFormat::MJPEG: return "MJPEG";
        case PixelFormat::GREY:  return "GREY";
        case PixelFormat::Z16:   return "Z16";
        case PixelFormat::YUYV:  return "YUYV";
        case PixelFormat::NV12:  return "NV12";
        case PixelFormat::Y8I:   return "Y8I";
        case PixelFormat::RGB8:  return "RGB8";
        default:                 return "Unknown";
    }
}

static unsigned int pixelFormatToFourcc(PixelFormat pf) {
    switch (pf) {
        case PixelFormat::MJPEG: return V4L2_PIX_FMT_MJPEG;
        case PixelFormat::GREY:  return V4L2_PIX_FMT_GREY;
        case PixelFormat::Z16:   return V4L2_PIX_FMT_Z16;
        case PixelFormat::YUYV:  return V4L2_PIX_FMT_YUYV;
        case PixelFormat::NV12:  return V4L2_PIX_FMT_NV12;
        case PixelFormat::Y8I:   return V4L2_PIX_FMT_Y8I;
        default:                 return 0;
    }
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

static void cache_initial_camera_params(int cam_id) {
    camera_params params;
    if (insight9_receive_get_camera_params_for(cam_id, &params) == 0) {
        g_ctx.cachedInitialParams[cam_id] = params;
        g_ctx.hasCachedInitialParams[cam_id] = 1;
        printf("[SDK] Cached initial params for camera %d\n", cam_id);
    } else {
        fprintf(stderr, "[SDK][WARN] Failed to cache initial params for camera %d\n", cam_id);
    }
}

static PixelFormat fourccToPixelFormat(unsigned int fcc) {
    switch (fcc) {
        case V4L2_PIX_FMT_MJPEG: return PixelFormat::MJPEG;
        case V4L2_PIX_FMT_GREY:  return PixelFormat::GREY;
        case V4L2_PIX_FMT_YUYV:  return PixelFormat::YUYV;
        case V4L2_PIX_FMT_NV12:  return PixelFormat::NV12;
#ifdef V4L2_PIX_FMT_Z16
        case V4L2_PIX_FMT_Z16:   return PixelFormat::Z16;
#endif
#ifdef V4L2_PIX_FMT_Y8I
        case V4L2_PIX_FMT_Y8I:   return PixelFormat::Y8I;
#endif
        default:
            return PixelFormat::Unknown;
    }
}

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
            intervals[count] = frmival.discrete.denominator / 
                              frmival.discrete.numerator;
            count++;
        } else if (frmival.type == V4L2_FRMIVAL_TYPE_STEPWISE) {
            intervals[count] = frmival.stepwise.max.denominator / 
                              frmival.stepwise.max.numerator;
            count++;
        } else if (frmival.type == V4L2_FRMIVAL_TYPE_CONTINUOUS) {
            intervals[count++] = 30;
            intervals[count++] = 25;
            intervals[count++] = 15;
        }
        frmival.index++;
    }
    
    return count;
}

static int get_default_format(int fd, struct v4l2_format *fmt) {
    memset(fmt, 0, sizeof(*fmt));
    fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    if (ioctl(fd, VIDIOC_G_FMT, fmt) == 0) {
        printf("[CAM] Got current format: %dx%d, pixelformat=0x%x\n",
               fmt->fmt.pix.width, fmt->fmt.pix.height, fmt->fmt.pix.pixelformat);
        return 0;
    }
    printf("[CAM] VIDIOC_G_FMT failed\n");
    return -1;
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
    
    memset(&fmtdesc, 0, sizeof(fmtdesc));
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmtdesc.index = 0;
    
    int fmt_idx = 0;
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
        formats[fmt_idx].fcc = fmtdesc.pixelformat;
        formats[fmt_idx].frames_num = 0;
        formats[fmt_idx].frames = NULL;
        
        memset(&frmsize, 0, sizeof(frmsize));
        frmsize.pixel_format = fmtdesc.pixelformat;
        frmsize.index = 0;
        
        int frame_count = 0;
        struct uvc_frame_info *frames = NULL;
        
        while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0) {
            if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
                frame_count++;
                frames = (struct uvc_frame_info*)realloc(frames, frame_count * sizeof(struct uvc_frame_info));
                if (!frames) {
                    fprintf(stderr, "[CAM] Failed to allocate frames memory\n");
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
                
                int interval_count = get_frame_intervals(
                    fd,
                    fmtdesc.pixelformat,
                    frmsize.discrete.width,
                    frmsize.discrete.height,
                    frames[frame_count - 1].intervals,
                    8
                );
                
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

// Probe one V4L2 node's capture formats and classify it as RGB / LEFT / DEPTH.
// Metadata-only nodes advertise no capture format and fall through to -1.
// Node kinds returned by probe_uvc_node().
#define NODE_VIDEO   0   // VideoStreaming node; *out holds its stream type
#define NODE_META    1   // its metadata sibling (no VIDEO_CAPTURE formats)
#define NODE_ERROR (-1)  // could not be opened / queried

// Stream families a node can advertise. A node is not required to carry
// exactly one: on Insight 3u the stereo node offers Y8I *and* YUYV (left/right
// stacked top-to-bottom), so the colour stream is an alternate format on the
// gray camera rather than a separate logical camera.
#define FAM_DEPTH 0x1   // Z16
#define FAM_GRAY  0x2   // Y8I / GREY
#define FAM_COLOR 0x4   // MJPEG / YUYV / NV12

// Probe one v4l2 node. find_uvc_devices_by_vid_pid() returns video *and*
// metadata nodes mixed together - VIDIOC_QUERYCAP reports the union of the
// whole device's capabilities, so a metadata node also advertises
// VIDEO_CAPTURE - and the only reliable way to tell them apart is that a
// metadata node enumerates zero VIDEO_CAPTURE formats.
//
// Classification is by priority, not by exclusive match. The old rule
// ("exactly one of MJPEG/Y8I/Z16 and nothing else") returned UNKNOWN for any
// node advertising a second format, which then silently disabled hot-plug
// re-detection for that camera.
static int probe_uvc_node(const char *dev, CameraType *out, int *families = NULL) {
    if (out) *out = TYPE_UNKNOWN;
    if (families) *families = 0;
    if (!dev || !dev[0]) return NODE_ERROR;
    int fd = open(dev, O_RDWR);
    if (fd < 0) return NODE_ERROR;

    bool has_mjpeg = false, has_y8i = false, has_z16 = false;
    bool has_grey = false, has_color = false;
    int nfmt = 0;
    struct v4l2_fmtdesc fmtdesc;
    memset(&fmtdesc, 0, sizeof(fmtdesc));
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmtdesc.index = 0;
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
        switch (fmtdesc.pixelformat) {
            case V4L2_PIX_FMT_MJPEG: has_mjpeg = true; break;
            case V4L2_PIX_FMT_GREY:  has_grey  = true; break;
            case V4L2_PIX_FMT_YUYV:
            case V4L2_PIX_FMT_NV12:  has_color = true; break;
#ifdef V4L2_PIX_FMT_Y8I
            case V4L2_PIX_FMT_Y8I:   has_y8i   = true; break;
#endif
#ifdef V4L2_PIX_FMT_Z16
            case V4L2_PIX_FMT_Z16:   has_z16   = true; break;
#endif
            default: break;
        }
        nfmt++;
        fmtdesc.index++;
    }
    close(fd);

    if (nfmt == 0) return NODE_META;   // no capture format -> metadata sibling

    int mask = 0;
    if (has_z16)                mask |= FAM_DEPTH;
    if (has_y8i || has_grey)    mask |= FAM_GRAY;
    if (has_mjpeg || has_color) mask |= FAM_COLOR;

    // Preferred identity when nothing else competes for this node.
    CameraType t = TYPE_UNKNOWN;
    if (mask & FAM_DEPTH)      t = TYPE_DEPTH;
    else if (mask & FAM_GRAY)  t = TYPE_LEFT;
    else if (mask & FAM_COLOR) t = TYPE_RGB;

    if (out) *out = t;
    if (families) *families = mask;
    return NODE_VIDEO;
}

// How many distinct stream families a node advertises.
static int family_count(int mask) {
    int n = 0;
    for (int b = 1; b <= FAM_COLOR; b <<= 1) if (mask & b) n++;
    return n;
}

// Which logical camera should claim a node with this family set, given which
// cameras are already taken. Depth beats gray beats colour, but a family whose
// camera is already claimed is skipped - that is what lets the stereo node be
// used as the colour source on a unit where something else is already gray.
static int cam_id_from_families(int mask, const bool *taken) {
    if ((mask & FAM_DEPTH) && !taken[0]) return 0;
    if ((mask & FAM_GRAY)  && !taken[1]) return 1;
    if ((mask & FAM_COLOR) && !taken[2]) return 2;
    return -1;
}

static void format_families_str(int mask, char *buf, size_t n) {
    snprintf(buf, n, "%s%s%s",
             (mask & FAM_DEPTH) ? "Z16 " : "",
             (mask & FAM_GRAY)  ? "Y8I/GREY " : "",
             (mask & FAM_COLOR) ? "MJPEG/YUYV " : "");
}

static const char *cam_type_name(CameraType t) {
    switch (t) {
        case TYPE_RGB:   return "RGB";
        case TYPE_LEFT:  return "GRAY";
        case TYPE_DEPTH: return "DEPTH";
        default:         return "UNKNOWN";
    }
}

// Reset every logical camera to "not attached".
static void unbind_all_cameras(void) {
    pthread_mutex_lock(&g_bind_lock);
    for (int i = 0; i < CAM_NUM; i++) {
        cam_bound[i] = false;
        cam_families[i] = 0;
        cam_type[i] = cam_type_for_id(i);   // what we look for, not what we have
        g_ctx.video_devs[i][0] = '\0';
        g_ctx.metadata_devs[i][0] = '\0';
        g_ctx.video_usb_paths[i][0] = '\0';
        g_ctx.metadata_usb_paths[i][0] = '\0';
    }
    pthread_mutex_unlock(&g_bind_lock);
}

// Attach any not-yet-attached logical camera to a matching node.
//
// Replaces the old fixed index selection (video 0/2/4, metadata 1/3/5), which
// assumed exactly three VideoStreaming interfaces in a fixed order and is what
// made a two-stream unit die with "cannot select 2/3 devices by skipping one".
// Here each video node's own formats decide which logical camera it is, and it
// is paired with the metadata node immediately after it - but only after
// confirming that node really is a metadata node, since on a device whose
// streams carry no metadata, node i+1 is the next camera's video node.
//
// Safe to call repeatedly from the capture threads. Returns how many cameras
// were newly attached by this call.
static int try_bind_cameras(void) {
    char uvc_list[10][MAX_PATH] = {{0}};
    int uvc_count = find_uvc_devices_by_vid_pid(VENDOR_ID, PRODUCT_ID, uvc_list, 10);
    if (uvc_count <= 0) return 0;

    int newly = 0;
    pthread_mutex_lock(&g_bind_lock);

    // Two passes, so a node that advertises one family only is never outbid by
    // a node that happens to advertise several.
    //   pass 0: single-family nodes claim their camera
    //   pass 1: multi-family nodes fill whatever is still open
    // On Insight 9 every node is single-family, so pass 0 does all the work and
    // the result is identical to before. On Insight 3u the stereo node offers
    // Y8I+YUYV: nothing else claims gray, so it lands on cam 1 and the colour
    // stream stays reachable there as an alternate format.
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < uvc_count; i++) {
            CameraType t = TYPE_UNKNOWN;
            int fams = 0;
            if (probe_uvc_node(uvc_list[i], &t, &fams) != NODE_VIDEO) continue;
            if (fams == 0) {
                if (pass == 0)
                    fprintf(stderr, "[SDK][WARN] %s: unrecognised stream type, skipped\n",
                            uvc_list[i]);
                continue;
            }

            const bool multi = family_count(fams) > 1;
            if ((pass == 0) == multi) continue;

            // Never steal a node another camera already owns.
            bool owned = false;
            for (int c = 0; c < CAM_NUM; c++) {
                if (cam_bound[c] && strcmp(uvc_list[i], g_ctx.video_devs[c]) == 0) {
                    owned = true;
                    break;
                }
            }
            if (owned) continue;

            int cid = cam_id_from_families(fams, cam_bound);
            if (cid < 0) continue;   // every camera this node could serve is taken

            snprintf(g_ctx.video_devs[cid], MAX_PATH, "%s", uvc_list[i]);
            cam_type[cid] = cam_type_for_id(cid);
            cam_families[cid] = fams;

            if (i + 1 < uvc_count && probe_uvc_node(uvc_list[i + 1], NULL) == NODE_META) {
                snprintf(g_ctx.metadata_devs[cid], MAX_PATH, "%s", uvc_list[i + 1]);
            } else {
                g_ctx.metadata_devs[cid][0] = '\0';
                fprintf(stderr, "[CAM%d][WARN] %s has no metadata node; "
                        "falling back to v4l2 buffer timestamps\n", cid, uvc_list[i]);
            }

            if (get_video_usb_device_path(g_ctx.video_devs[cid],
                                          g_ctx.video_usb_paths[cid], MAX_PATH) < 0) {
                g_ctx.video_usb_paths[cid][0] = '\0';
            }
            if (g_ctx.metadata_devs[cid][0] &&
                get_video_usb_device_path(g_ctx.metadata_devs[cid],
                                          g_ctx.metadata_usb_paths[cid], MAX_PATH) < 0) {
                g_ctx.metadata_usb_paths[cid][0] = '\0';
            }

            cam_bound[cid] = true;
            newly++;

            char fam_str[64];
            format_families_str(fams, fam_str, sizeof(fam_str));
            printf("[CAM%d] attached as %s video=%s metadata=%s offers=[%s]\n",
                   cid, cam_type_name(cam_type[cid]), g_ctx.video_devs[cid],
                   g_ctx.metadata_devs[cid][0] ? g_ctx.metadata_devs[cid] : "(none)",
                   fam_str);
        }
    }
    pthread_mutex_unlock(&g_bind_lock);
    return newly;
}

static int bound_camera_count(void) {
    int n = 0;
    for (int i = 0; i < CAM_NUM; i++) if (cam_bound[i]) n++;
    return n;
}

static void log_camera_inventory(void) {
    for (int i = 0; i < CAM_NUM; i++) {
        if (!cam_bound[i]) {
            printf("[CAM%d] type=%s not present yet; will attach on reconnect\n",
                   i, cam_type_name(cam_type_for_id(i)));
        }
    }
    printf("[SDK] %d/%d streams currently available\n", bound_camera_count(), CAM_NUM);
}

// Order in which to try nodes when binding the XU. The XU lives on the
// VideoControl interface, so any VideoStreaming node of the same USB function
// reaches it. Nodes whose capture fd is already open come first: those are
// known to be alive.
// Returns how many candidates were written to out[].
static int xu_candidate_order(int *out) {
    int n = 0;
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < CAM_NUM; i++) {
            if (!cam_bound[i] || !g_ctx.video_devs[i][0]) continue;
            const bool live = (g_ctx.cams[i].fd >= 0);
            if ((pass == 0) != live) continue;
            out[n++] = i;
        }
    }
    return n;
}

static void refresh_video_device_path(int cam_id) {
    // Not attached yet: nothing to refresh, try_bind_cameras() owns that case.
    if (!cam_bound[cam_id]) return;
    // Unclassified camera: never guess, just keep the path chosen at init.
    if (cam_type[cam_id] == TYPE_UNKNOWN) return;

    char uvc_list[10][MAX_PATH];
    int uvc_count = find_uvc_devices_by_vid_pid(VENDOR_ID, PRODUCT_ID, uvc_list, 10);
    if (uvc_count <= 0) {
        fprintf(stderr, "[CAM%d] No UVC devices found\n", cam_id);
        return;
    }

    // Match on "this node can serve my role", not "this node's preferred role
    // is mine": the stereo node's preferred role is gray even when it was
    // claimed for colour, and requiring exact equality would leave the camera
    // stuck on a stale path after a re-enumeration.
    const int want_fam = (cam_id == 0) ? FAM_DEPTH : (cam_id == 1) ? FAM_GRAY : FAM_COLOR;
    for (int i = 0; i < uvc_count; i++) {
        CameraType t = TYPE_UNKNOWN;
        int fams = 0;
        if (probe_uvc_node(uvc_list[i], &t, &fams) != NODE_VIDEO) continue;  // metadata node
        if (!(fams & want_fam)) continue;

        // Never steal a node another camera already owns; the enumeration lists
        // every node of the device, so an unguarded match would let two cameras
        // converge onto the same video/metadata pair.
        bool owned_by_other = false;
        for (int c = 0; c < CAM_NUM; c++) {
            if (c != cam_id && strcmp(uvc_list[i], g_ctx.video_devs[c]) == 0) {
                owned_by_other = true;
                break;
            }
        }
        if (owned_by_other) continue;

        if (strcmp(uvc_list[i], g_ctx.video_devs[cam_id]) != 0) {
            printf("[CAM%d] path moved %s -> %s\n",
                   cam_id, g_ctx.video_devs[cam_id], uvc_list[i]);
            snprintf(g_ctx.video_devs[cam_id], MAX_PATH, "%s", uvc_list[i]);
            // The metadata node is the sibling right after the video node -
            // but only if it really is a metadata node. On a variant whose
            // streams expose no metadata interface, node i+1 is the next
            // camera's video node, and copying it here would make two cameras
            // read each other's buffers.
            int meta_idx = i + 1;
            if (meta_idx < uvc_count && probe_uvc_node(uvc_list[meta_idx], NULL) == NODE_META)
                snprintf(g_ctx.metadata_devs[cam_id], MAX_PATH, "%s", uvc_list[meta_idx]);
            else
                g_ctx.metadata_devs[cam_id][0] = '\0';

            // Path refresh refreshes the path, nothing else. This used to open
            // the new node, read VIDIOC_G_FMT and overwrite the context's
            // geometry with whatever that node happened to be set to. Node
            // numbers shift on every USB re-enumeration, so a single "path
            // moved" replaced the resolution the user had selected with the
            // node's current one - and a camera whose path did not move kept
            // its selection, which is how the two composite streams ended up
            // with different geometry after a reconnect. Geometry belongs to
            // init_capture()'s S_FMT/G_FMT round-trip; only seed it here when
            // there is nothing configured yet.
            struct cam_ctx *rc = &g_ctx.cams[cam_id];
            if (rc->width == 0 || rc->height == 0 || rc->format == 0) {
                int fd2 = open(uvc_list[i], O_RDWR);
                if (fd2 >= 0) {
                    struct v4l2_format fmt;
                    if (get_default_format(fd2, &fmt) == 0) {
                        rc->width  = fmt.fmt.pix.width;
                        rc->height = fmt.fmt.pix.height;
                        rc->format = fmt.fmt.pix.pixelformat;
                    }
                    close(fd2);
                }
            } else {
                printf("[CAM%d] path moved, keeping configured geometry %dx%d fmt=0x%x\n",
                       cam_id, rc->width, rc->height, rc->format);
            }
        }
        return; // matched
    }

    // No match: keep the existing path rather than falling back to a fixed index.
    fprintf(stderr, "[CAM%d] Cannot find device for type %s, keeping old path: %s\n",
            cam_id, cam_type_name(cam_type[cam_id]), g_ctx.video_devs[cam_id]);
}

static bool reopenXUControlLocked(const char* reason) {
    // Try every attached node rather than only the first.
    //
    // The first attached node is camera 0, which on Linux is depth - and on
    // units where the depth stream can be switched off, that is exactly the
    // node that disappears. Binding the XU to it alone meant the XU went dead
    // and stayed dead whenever depth was disabled, even though cameras 1 and 2
    // were still present and expose the same VideoControl interface.
    //
    // A failure here is also not always "this node has no XU": the probe does
    // GET_INFO/GET_LEN control transfers, and those can time out (errno 110)
    // when the device's single control endpoint is busy with another node's
    // format negotiation. Moving on to the next candidate recovers from that
    // instead of concluding the selector is missing.
    int order[CAM_NUM];
    const int n = xu_candidate_order(order);
    if (n == 0) {
        fprintf(stderr, "[XU][ERR] Cannot reopen XU: no UVC video node attached (%s)\n",
                reason ? reason : "unknown");
        g_ctx.xu_dev_path[0] = '\0';
        g_ctx.xu_ready = false;
        return false;
    }

    if (g_ctx.xu_control) {
        g_ctx.xu_control->close();
        delete g_ctx.xu_control;
        g_ctx.xu_control = nullptr;
    }

    for (int k = 0; k < n; k++) {
        const char *dev_path = g_ctx.video_devs[order[k]];
        viewer::UvcExtensionUnit *xu = new viewer::UvcExtensionUnit();
        if (xu->open(dev_path)) {
            g_ctx.xu_control = xu;
            snprintf(g_ctx.xu_dev_path, MAX_PATH, "%s", dev_path);
            g_ctx.xu_ready = true;
            printf("[XU] bound to %s (%s)\n", dev_path, reason ? reason : "unknown");
            return true;
        }
        delete xu;
        fprintf(stderr, "[XU][WARN] cannot open XU on %s (%s)%s\n",
                dev_path, reason ? reason : "unknown",
                (k + 1 < n) ? ", trying next node" : "");
    }

    g_ctx.xu_dev_path[0] = '\0';
    g_ctx.xu_ready = false;
    return false;
}

// Throttle XU reopens that are triggered opportunistically on a successful
// video reconnect (capture_thread's "cam_id == 0" path below), as opposed to
// an actual failed XU query - those still get a real, un-throttled retry via
// callXUWithRetry(), since a caller is actively blocked waiting on the
// result. This just stops a flapping depth stream from re-opening the XU
// control node (and the ioctl round-trips that go with it) on every single
// reconnect. Must be called while holding g_ctx.xu_mutex.
static bool reopenXUControlThrottled(const char* reason) {
    static struct timespec s_last_reopen_ts = {0, 0};
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double since_ms = (now.tv_sec - s_last_reopen_ts.tv_sec) * 1000.0 +
                       (now.tv_nsec - s_last_reopen_ts.tv_nsec) / 1e6;
    if (s_last_reopen_ts.tv_sec != 0 && since_ms < XU_REOPEN_MIN_INTERVAL_MS) {
        fprintf(stderr, "[XU] Skipping reopen (%s): only %.0fms since last reopen (< %dms)\n",
                reason ? reason : "unknown", since_ms, XU_REOPEN_MIN_INTERVAL_MS);
        return false;
    }
    bool ok = reopenXUControlLocked(reason);
    if (ok) {
        clock_gettime(CLOCK_MONOTONIC, &s_last_reopen_ts);
    }
    return ok;
}

template<typename Func>
static bool callXUWithRetry(const char* op, Func fn) {
    pthread_mutex_lock(&g_ctx.xu_mutex);

    if (!g_ctx.xu_control || !g_ctx.xu_control->isOpen()) {
        if (!reopenXUControlLocked(op)) {
            pthread_mutex_unlock(&g_ctx.xu_mutex);
            return false;
        }
    }

    bool success = fn(*g_ctx.xu_control);
    if (success) {
        pthread_mutex_unlock(&g_ctx.xu_mutex);
        return true;
    }

    fprintf(stderr, "[XU][WARN] Operation '%s' failed; reopening XU and retrying once\n", op ? op : "unknown");
    if (!reopenXUControlLocked(op)) {
        pthread_mutex_unlock(&g_ctx.xu_mutex);
        return false;
    }

    success = fn(*g_ctx.xu_control);
    pthread_mutex_unlock(&g_ctx.xu_mutex);
    return success;
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
    // A zero denominator is not a valid frame interval. Some nodes accept it
    // and quietly reset to their default, others reject it with EIO - which is
    // where the "[CAM][ERR] VIDIOC_S_PARM: Input/output error" at startup came
    // from. Refuse it here instead of letting it reach the driver.
    if (framerate <= 0) {
        fprintf(stderr, "[CAM][WARN] ignoring set_framerate(%d): not a valid rate\n", framerate);
        return -1;
    }
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

static int get_camera_formats_info(struct cam_ctx *ctx) {
    if (!ctx || ctx->fd < 0) {
        fprintf(stderr, "[CAM] Invalid context or file descriptor\n");
        return -1;
    }

    struct uvc_format_info *formats = NULL;
    int format_num = 0;
    
    if (get_all_formats_info(ctx->fd, &formats, &format_num) != 0) {
        fprintf(stderr, "[CAM%d] Failed to get formats info\n", ctx->cam_id);
        return -1;
    }
    
    ctx->formats_info = formats;
    ctx->format_num = format_num;
    
    printf("[CAM%d] Got %d formats info\n", ctx->cam_id, format_num);
    
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

// Push the configured geometry for this logical camera into its context.
// Zero width/height/format is fine and meaningful: init_capture() then adopts
// whatever the device reports, which is what init_default() wants.
static void apply_config_to_cam(int cam_id) {
    if (cam_id < 0 || cam_id >= CAM_NUM) return;
    struct cam_ctx *c = &g_ctx.cams[cam_id];
    const video_config_t *sc = (cam_id == 0) ? &g_ctx.config.depth_config
                             : (cam_id == 1) ? &g_ctx.config.gray_config
                                             : &g_ctx.config.rgb_config;
    // Only push a config that actually says something. A zeroed entry used to
    // be copied over verbatim, which wiped a geometry the context had already
    // negotiated: on reconnect init_capture() then saw 0 and fell back to the
    // device default, silently discarding the resolution the user selected.
    if (sc->width > 0 && sc->height > 0 && sc->pixel_format != PixelFormat::Unknown) {
        c->width  = sc->width;
        c->height = sc->height;
        c->format = pixelFormatToFourcc(sc->pixel_format);
    } else {
        printf("[CAM%d] config empty, keeping current geometry %dx%d fmt=0x%x\n",
               cam_id, c->width, c->height, c->format);
    }
    // fps too. Nothing ever wrote cam_ctx::fps, so it stayed 0 from the
    // memset and init_capture() opened every stream with set_framerate(fd, 0).
    if (sc->fps > 0) c->fps = sc->fps;
}

// Consecutive VIDIOC_S_FMT failures per logical camera. Reset on any success.
#define SFMT_MAX_RETRY 5
static int sfmt_fails[CAM_NUM] = {0};

static int init_capture(struct cam_ctx *ctx) {
    struct v4l2_format fmt;
    int use_default_format = 0;

    if (ctx->width == 0 || ctx->height == 0 || ctx->format == 0) {
        use_default_format = 1;
        printf("[CAM%d] Using default format from device\n", ctx->cam_id);
        if (get_default_format(ctx->fd, &fmt) < 0) {
            fprintf(stderr, "[CAM%d][ERR] Failed to get default format\n", ctx->cam_id);
            return -1;
        }
        ctx->width = fmt.fmt.pix.width;
        ctx->height = fmt.fmt.pix.height;
        ctx->format = fmt.fmt.pix.pixelformat;
        printf("init capture (default) %d %d %d\n", ctx->width, ctx->height, ctx->format);
    } else {
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = ctx->width;
        fmt.fmt.pix.height = ctx->height;
        fmt.fmt.pix.pixelformat = ctx->format;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        printf("[CAM%d] Trying to set format: %dx%d, fourcc=0x%x\n",
               ctx->cam_id, ctx->width, ctx->height, ctx->format);
    }

    if (ioctl(ctx->fd, VIDIOC_S_FMT, &fmt) < 0) {
        fprintf(stderr, "[CAM%d][ERR] VIDIOC_S_FMT failed: %s\n", ctx->cam_id, strerror(errno));

        if (!use_default_format) {
            // Right after the composite re-enumerates, the depth node is often
            // opened before the firmware has finished configuring it and S_FMT
            // fails with EBUSY/EINVAL. Falling straight back to the device
            // default here is what made a reconnect silently revert the stream
            // to its factory geometry. Fail the open instead and let the
            // capture thread retry on its backoff; only give up and take the
            // default after the requested mode has failed this many times in a
            // row, so a genuinely unsupported mode still ends up streaming.
            if (sfmt_fails[ctx->cam_id] < SFMT_MAX_RETRY) {
                sfmt_fails[ctx->cam_id]++;
                fprintf(stderr, "[CAM%d][WARN] S_FMT %dx%d fourcc=0x%x failed (%d/%d), will retry\n",
                        ctx->cam_id, ctx->width, ctx->height, ctx->format,
                        sfmt_fails[ctx->cam_id], SFMT_MAX_RETRY);
                return -1;
            }
            fprintf(stderr, "[CAM%d][WARN] S_FMT %dx%d fourcc=0x%x failed %d times, "
                            "falling back to device default - requested mode is lost\n",
                    ctx->cam_id, ctx->width, ctx->height, ctx->format, SFMT_MAX_RETRY);
            sfmt_fails[ctx->cam_id] = 0;
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
    sfmt_fails[ctx->cam_id] = 0;

    struct v4l2_format actual_fmt;
    memset(&actual_fmt, 0, sizeof(actual_fmt));
    actual_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(ctx->fd, VIDIOC_G_FMT, &actual_fmt) == 0) {
        ctx->width = actual_fmt.fmt.pix.width;
        ctx->height = actual_fmt.fmt.pix.height;
        ctx->format = actual_fmt.fmt.pix.pixelformat;
        printf("[CAM%d] negotiated %dx%d fmt=0x%x\n",
               ctx->cam_id, ctx->width, ctx->height, ctx->format);
        video_config_t *sc = (ctx->cam_id == 0) ? &g_ctx.config.depth_config
                       : (ctx->cam_id == 1) ? &g_ctx.config.gray_config
                                            : &g_ctx.config.rgb_config;
        sc->width        = ctx->width;
        sc->height       = ctx->height;
        sc->pixel_format = fourccToPixelFormat(ctx->format);
    }

    // One rate, from this camera's own config. The old code called
    // set_framerate() twice - once with ctx->fps and again with a config fps
    // picked by an inverted cam_id mapping (cam 0 is depth, cam 2 is RGB) - so
    // every stream took two S_PARM round-trips and the first used a stale or
    // zero value.
    const video_place_t place = (ctx->cam_id == 0) ? PLACE_DEPTH
                              : (ctx->cam_id == 1) ? PLACE_GRAY
                                                   : PLACE_RGB;
    int want_fps = (place == PLACE_DEPTH) ? g_ctx.config.depth_config.fps
                 : (place == PLACE_GRAY)  ? g_ctx.config.gray_config.fps
                                          : g_ctx.config.rgb_config.fps;
    if (want_fps <= 0) want_fps = ctx->fps;
    if (want_fps > 0) {
        set_framerate(ctx->fd, want_fps);
        ctx->fps = want_fps;
        // Same reason as the geometry write-back above: keep config in sync so
        // the next reopen restores this rate instead of starting from 0.
        video_config_t *fc = (place == PLACE_DEPTH) ? &g_ctx.config.depth_config
                           : (place == PLACE_GRAY)  ? &g_ctx.config.gray_config
                                                    : &g_ctx.config.rgb_config;
        fc->fps = want_fps;
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
    
    *width = ctx->width;
    *height = ctx->height;
    *format = ctx->format;
    return 0;
}

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
static int is_camera_params_valid(const camera_params *params) {
    // Validate resolution index. RGB uses 0-3 and grayscale uses 0-1; the SDK uses the more permissive 0-3 range.
    if (params->resolution > 3) {
        fprintf(stderr, "[XU][ERR] invalid resolution, expected 0-3, got %d\n", params->resolution);
        return 0;
    }
    // Validate frame-rate index.
    if (params->frame_rate > 5) {
        fprintf(stderr, "[XU][ERR] invalid frame rate, expected 0-5, got %d\n", params->frame_rate);
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
    // Hysteresis for reconnect_fails: a bare "open+start succeeded" doesn't
    // reset the failure counter anymore. We only trust the connection (and
    // reset backoff to its base delay) once it has stayed up for at least
    // this camera's min-stable window - otherwise a flapping device keeps
    // bouncing reconnect_fails back to 0 and the exponential backoff never
    // actually grows.
    bool pending_fail_reset = false;
    struct timespec connect_ts = {0, 0};

    printf("[CAM%d] capture thread started, dev=%s meta=%s\n",
           ctx->cam_id,
           g_ctx.video_devs[ctx->cam_id][0] ? g_ctx.video_devs[ctx->cam_id] : "(pending)",
           g_ctx.metadata_devs[ctx->cam_id][0] ? g_ctx.metadata_devs[ctx->cam_id] : "(pending)");
    // Set once we have said this camera is absent, so a stream this hardware
    // simply does not have cannot spam the log forever.
    bool absent_logged = false;
    // The first successful open is startup, not a reconnect; see the XU rebind
    // check further down.
    bool first_open = true;

    while (g_ctx.running) {
        if (!g_ctx.cam_running[ctx->cam_id]) {
            break;
        }
        // If a previous reconnect is still "on probation", check whether it
        // has now been up long enough to be trusted; only then clear the
        // failure counter so the next real failure starts backing off from
        // the base delay again instead of resuming a long-since-decayed one.
        if (pending_fail_reset) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            double up_ms = (now.tv_sec - connect_ts.tv_sec) * 1000.0 +
                            (now.tv_nsec - connect_ts.tv_nsec) / 1e6;
            int min_stable_ms = (ctx->cam_id == 0) ? DEPTH_MIN_STABLE_MS : 500;
            if (up_ms >= min_stable_ms) {
                reconnect_fails = 0;
                pending_fail_reset = false;
            }
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
            // The camera may have no node yet: the SDK is allowed to
            // initialise with the device absent, so discovery happens here
            // rather than only at init.
            if (!cam_bound[ctx->cam_id]) {
                pthread_mutex_unlock(&ctx->fd_lock);
                try_bind_cameras();
                if (!cam_bound[ctx->cam_id]) {
                    if (!absent_logged) {
                        printf("[CAM%d] %s stream not present, waiting for it to appear\n",
                               ctx->cam_id, cam_type_name(cam_type_for_id(ctx->cam_id)));
                        absent_logged = true;
                    }
                    // Reuses the normal backoff, so a stream this unit simply
                    // does not have settles at the ceiling interval instead of
                    // rescanning the bus continuously.
                    reconnect_backoff_apply_for_cam(ctx, &reconnect_fails, "stream not present");
                    continue;
                }
                absent_logged = false;
                reconnect_fails = 0;
                // A newly attached camera has no negotiated format yet; let
                // init_capture() fall back to the device default unless the
                // caller configured one.
                apply_config_to_cam(ctx->cam_id);
                pthread_mutex_lock(&ctx->fd_lock);
                if (!g_ctx.cam_running[ctx->cam_id] || ctx->fd >= 0) {
                    pthread_mutex_unlock(&ctx->fd_lock);
                    continue;
                }
            }

            dev_path = g_ctx.video_devs[ctx->cam_id];
            printf("[CAM%d] device not open, opening %s\n", ctx->cam_id, dev_path);
            refresh_video_device_path(ctx->cam_id);
            dev_path = g_ctx.video_devs[ctx->cam_id];

            // Only if init did not already get them. This used to run on
            // every open, so all three capture threads fired an XU
            // transaction at startup while the other two nodes were in the
            // middle of S_FMT/S_PARM - and the device has one control
            // endpoint, so those transfers timed out.
            if (!g_ctx.hasCachedInitialParams[ctx->cam_id]) {
                cache_initial_camera_params(ctx->cam_id);
            }

            // Metadata is optional: a variant whose VideoStreaming interface
            // exposes no metadata node still streams video, it just cannot
            // supply device timestamps.
            ctx->meta_fd = -1;
            if (g_ctx.metadata_devs[ctx->cam_id][0]) {
                printf("[CAM%d][META] opening %s\n", ctx->cam_id, g_ctx.metadata_devs[ctx->cam_id]);
                ctx->meta_fd = open(g_ctx.metadata_devs[ctx->cam_id], O_RDWR | O_NONBLOCK);
                if (ctx->meta_fd < 0) {
                    fprintf(stderr, "[CAM%d][META][ERR] open failed: %s\n", ctx->cam_id, strerror(errno));
                    pthread_mutex_unlock(&ctx->fd_lock);
                    usleep(500000);
                    continue;
                }
                if (init_metadata_capture(ctx) < 0 || start_metadata_capture(ctx->meta_fd) < 0) {
                    fprintf(stderr, "[CAM%d][META][ERR] init/start failed\n", ctx->cam_id);
                    close(ctx->meta_fd);
                    ctx->meta_fd = -1;
                    pthread_mutex_unlock(&ctx->fd_lock);
                    usleep(500000);
                    continue;
                }
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
                pending_fail_reset = false;  // invalidate any stale probation timer
                reconnect_backoff_apply_for_cam(ctx, &reconnect_fails, "open failed");
                continue;
            }
            if (init_capture(ctx) < 0) {
                close(ctx->fd);
                ctx->fd = -1;
                stop_metadata_capture(ctx->meta_fd);
                close(ctx->meta_fd);
                pthread_mutex_unlock(&ctx->fd_lock);
                pending_fail_reset = false;  // invalidate any stale probation timer
                reconnect_backoff_apply_for_cam(ctx, &reconnect_fails, "init_capture failed");
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
                pending_fail_reset = false;  // invalidate any stale probation timer
                reconnect_backoff_apply_for_cam(ctx, &reconnect_fails, "failed to start stream");
                usleep(1000000);
                continue;
            }
            ctx->last_timestamp = 0;
            // Don't reset reconnect_fails yet - just mark this connection as
            // "on probation" and start the stability clock. It only counts
            // as recovered (and backoff resets to base) once it survives
            // min_stable_ms without failing again; see the check at the top
            // of the loop. This stops a flapping device from bouncing the
            // failure counter back to 0 on every brief, short-lived success.
            pending_fail_reset = true;
            clock_gettime(CLOCK_MONOTONIC, &connect_ts);
            pthread_mutex_unlock(&ctx->fd_lock);
            printf("[CAM%d] device reinitialized\n", ctx->cam_id);

            // Rebind the XU only when it is actually broken, or when it is
            // bound to the node that just came back (its handle belongs to the
            // old USB device instance and is now stale).
            //
            // This used to fire unconditionally for cam_id == 0 and was
            // labelled "RGB video reconnect", from when camera 0 was assumed
            // to be the RGB/XU-bearing node. On Linux camera 0 is depth, so it
            // fired on the very first open at startup and tore down a
            // perfectly good XU handle at the exact moment cameras 1 and 2
            // were negotiating formats. The reopen's control transfers then
            // timed out and open() wrongly reported the selector missing.
            if (!first_open) {
                pthread_mutex_lock(&g_ctx.xu_mutex);
                const bool xu_dead = !g_ctx.xu_ready || !g_ctx.xu_control ||
                                     !g_ctx.xu_control->isOpen();
                const bool xu_on_this_node =
                    g_ctx.xu_dev_path[0] && dev_path && dev_path[0] &&
                    strcmp(g_ctx.xu_dev_path, dev_path) == 0;
                if (xu_dead || xu_on_this_node) {
                    reopenXUControlThrottled("video reconnect");
                }
                pthread_mutex_unlock(&g_ctx.xu_mutex);
            }
            first_open = false;
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
            pending_fail_reset = false;  // invalidate any stale probation timer
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
                pending_fail_reset = false;  // invalidate any stale probation timer
                reconnect_backoff_apply_for_cam(ctx, &reconnect_fails, "DQBUF reset");
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
                pending_fail_reset = false;  // invalidate any stale probation timer
                safe_cleanup_camera(ctx);
            }
            continue;
        }

        const bool has_meta_node = g_ctx.metadata_devs[ctx->cam_id][0] != '\0';
        uint64_t timestamp;
        if (has_meta_node) {
            timestamp = read_latest_metadata_timestamp(ctx);
            if (ctx->meta_fd < 0) {
                // The metadata node this camera should have has gone away -
                // tear the stream down and let the reconnect path rebuild it.
                if (ioctl(ctx->fd, VIDIOC_QBUF, &buf) < 0) {
                    fprintf(stderr, "[CAM%d][ERR] VIDIOC_QBUF: %s\n", ctx->cam_id, strerror(errno));
                }
                close(ctx->fd);
                ctx->fd = -1;
                pending_fail_reset = false;  // invalidate any stale probation timer
                free_buffer_array(&ctx->buffers, &ctx->buffer_count);
                continue;
            }
        } else {
            // No metadata interface on this variant: use the driver's own
            // buffer timestamp so frames still reach the callback. These are
            // host-clock timestamps, not device timestamps, so anything doing
            // cross-sensor alignment must account for that.
            timestamp = (uint64_t)buf.timestamp.tv_sec * 1000000ull +
                        (uint64_t)buf.timestamp.tv_usec;
        }

        // Filter 2: drop buffers until matching UVC metadata provides a usable timestamp.
        if (timestamp == 0) {
            if (ioctl(ctx->fd, VIDIOC_QBUF, &buf) < 0) {
                fprintf(stderr, "[CAM%d][ERR] VIDIOC_QBUF: %s\n", ctx->cam_id, strerror(errno));
                close(ctx->fd);
                ctx->fd = -1;
                pending_fail_reset = false;  // invalidate any stale probation timer
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
                pending_fail_reset = false;  // invalidate any stale probation timer
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
            pending_fail_reset = false;  // invalidate any stale probation timer
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

// int switch_camera_fps(int cam_id, int new_fps) {
//     if (cam_id < 0 || cam_id >= CAM_NUM) return -1;
//     if (!cam_bound[cam_id]) {
//         fprintf(stderr, "[CAM%d][ERR] stream not attached\n", cam_id);
//         return -1;
//     }
//     struct cam_ctx *ctx = &g_ctx.cams[cam_id];

//     printf("[SDK] Switching camera %d to %d FPS...\n", cam_id, new_fps);

//     g_ctx.cam_running[cam_id] = false;
//     if (ctx->tid) {
//         pthread_join(ctx->tid, NULL);
//         ctx->tid = 0;
//     }

//     if (cam_id == 0) {
//         g_ctx.config.rgb_config.fps = new_fps;
//     } else if (cam_id == 1) {
//         g_ctx.config.gray_config.fps = new_fps;
//     } else if (cam_id == 2) {
//         g_ctx.config.depth_config.fps = new_fps;
//     }

//     g_ctx.cam_running[cam_id] = true;
//     if (pthread_create(&ctx->tid, NULL, capture_thread, ctx) != 0) {
//         fprintf(stderr, "[SDK][ERR] Failed to recreate capture thread for cam %d\n", cam_id);
//         g_ctx.cam_running[cam_id] = false;
//         return -1;
//     }

//     printf("[SDK] Camera %d switched to %d FPS successfully\n", cam_id, new_fps);
//     return 0;
// }

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

    // Attach whatever streams this unit publishes. Nothing here is fatal:
    // a device with fewer streams starts with fewer, and a device that is not
    // plugged in yet starts with none and attaches later from the capture
    // threads' reconnect path.
    unbind_all_cameras();
    try_bind_cameras();
    log_camera_inventory();

    // Find HID devices. Also non-fatal.
    char hid_list[10][MAX_PATH] = {{0}};
    int hid_count = find_hid_devices_by_vid_pid(VENDOR_ID, PRODUCT_ID, hid_list, 10);
    for (int i = 0; i < HID_NUM; i++) {
        g_ctx.hid_devs[i][0] = '\0';
        g_ctx.hid_usb_paths[i][0] = '\0';
    }
    if (hid_count < 2) {
        fprintf(stderr, "[SDK][WARN] expected 2 HID devices with VID=0x%04x PID=0x%04x, found %d; "
                "IMU/VIO will attach when they appear\n", VENDOR_ID, PRODUCT_ID, hid_count);
    }
    // Take the first two devices after sorting: the lower number is IMU and the higher number is VIO.
    for (int i = 0; i < HID_NUM && i < hid_count; i++) {
        snprintf(g_ctx.hid_devs[i], MAX_PATH, "%s", hid_list[i]);
        if (get_hid_usb_device_path(g_ctx.hid_devs[i], g_ctx.hid_usb_paths[i], MAX_PATH) < 0) {
            g_ctx.hid_usb_paths[i][0] = '\0';
        }
    }
    printf("[SDK] selected HID: IMU=%s VIO=%s\n",
           g_ctx.hid_devs[0][0] ? g_ctx.hid_devs[0] : "(pending)",
           g_ctx.hid_devs[1][0] ? g_ctx.hid_devs[1] : "(pending)");

    // Initialize camera contexts.
    for (int i = 0; i < CAM_NUM; i++) {
        g_ctx.cams[i].cam_id = i;
        g_ctx.cams[i].fd = -1;
        g_ctx.cams[i].meta_fd = -1;
        apply_config_to_cam(i);
    }

    pthread_mutex_init(&g_ctx.xu_mutex, NULL);
    g_ctx.xu_dev_path[0] = '\0';

    if (reopenXUControlLocked("initialization")) {
        for (int i = 0; i < CAM_NUM; i++) {
            if (cam_bound[i]) cache_initial_camera_params(i);
        }
    } else {
        fprintf(stderr, "[XU][WARN] extension unit unavailable for now; "
                "camera params will be read once a stream attaches\n");
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

    // Attach whatever streams this unit publishes; none of this is fatal.
    unbind_all_cameras();
    try_bind_cameras();
    log_camera_inventory();

    // Initialize camera contexts. apply_config_to_cam() carries the fps set
    // just above; width/height/format stay 0 here on purpose so init_capture()
    // adopts whatever the device reports.
    for (int i = 0; i < CAM_NUM; i++) {
        g_ctx.cams[i].cam_id = i;
        g_ctx.cams[i].fd = -1;
        apply_config_to_cam(i);
    }

    for (int i = 0; i < CAM_NUM; ++i) {
        if (!cam_bound[i] || g_ctx.video_devs[i][0] == '\0') continue;
        int fd = open(g_ctx.video_devs[i], O_RDWR);
        if (fd < 0) {
            fprintf(stderr, "[CAM%d][WARN] open for format enum failed: %s\n", i, strerror(errno));
            continue;
        }
        g_ctx.cams[i].fd = fd;
        if (get_camera_formats_info(&g_ctx.cams[i]) < 0) {
            fprintf(stderr, "[CAM%d][WARN] failed to get formats info during init\n", i);
        }

        // Record what the node is actually set to right now, into both the
        // context and g_ctx.config.
        //
        // Without this, init_default() left config width/height at 0 and
        // insight9_receive_get_camera_config() reported 0x0 until a capture
        // thread had negotiated - so a caller that wants to show "whatever the
        // device came up as" had nothing to read at init time, and
        // restore_target_geometry() had no target to restore on reconnect.
        struct v4l2_format cur;
        if (get_default_format(fd, &cur) == 0) {
            video_config_t *sc = (i == 0) ? &g_ctx.config.depth_config
                               : (i == 1) ? &g_ctx.config.gray_config
                                          : &g_ctx.config.rgb_config;
            sc->width        = cur.fmt.pix.width;
            sc->height       = cur.fmt.pix.height;
            sc->pixel_format = fourccToPixelFormat(cur.fmt.pix.pixelformat);

            struct v4l2_streamparm parm;
            memset(&parm, 0, sizeof(parm));
            parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (ioctl(fd, VIDIOC_G_PARM, &parm) == 0 &&
                parm.parm.capture.timeperframe.numerator > 0 &&
                parm.parm.capture.timeperframe.denominator > 0) {
                sc->fps = (int)(parm.parm.capture.timeperframe.denominator /
                                parm.parm.capture.timeperframe.numerator);
            }

            g_ctx.cams[i].width  = sc->width;
            g_ctx.cams[i].height = sc->height;
            g_ctx.cams[i].format = cur.fmt.pix.pixelformat;
            g_ctx.cams[i].fps    = sc->fps;

            printf("[CAM%d] device default %dx%d fmt=0x%x @%dfps\n",
                   i, sc->width, sc->height, cur.fmt.pix.pixelformat, sc->fps);
        }

        close(fd);
        g_ctx.cams[i].fd = -1;
    }

    pthread_mutex_init(&g_ctx.xu_mutex, NULL);
    g_ctx.xu_dev_path[0] = '\0';

    if (reopenXUControlLocked("initialization")) {
        for (int i = 0; i < CAM_NUM; i++) {
            if (cam_bound[i]) cache_initial_camera_params(i);
        }
    } else {
        fprintf(stderr, "[XU][WARN] extension unit unavailable for now; "
                "camera params will be read once a stream attaches\n");
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

    // Start a thread for every logical camera, attached or not: an unattached
    // one sits in the reconnect path and picks its stream up if it appears.
    for (int i = 0; i < CAM_NUM; i++) {
        g_ctx.cam_running[i] = true;
        g_ctx.first_frame_received[i] = false;
        clock_gettime(CLOCK_MONOTONIC, &g_ctx.last_frame_time[i]);
        // The handle goes in video_tids[] because that is what stop() and
        // stop_camera() join. Writing it to cams[i].tid instead left every
        // thread started here unjoined at shutdown.
        if (pthread_create(&g_ctx.video_tids[i], NULL, capture_thread, &g_ctx.cams[i]) != 0) {
            fprintf(stderr, "[CAM%d][ERR] failed to create capture thread\n", i);
            g_ctx.cam_running[i] = false;
            g_ctx.video_tids[i] = 0;
            continue;
        }
        g_ctx.cams[i].tid = g_ctx.video_tids[i];
    }

    pthread_mutex_lock(&g_ctx.xu_mutex);
    reopenXUControlLocked("camera start");
    pthread_mutex_unlock(&g_ctx.xu_mutex);

    hid_init();

    printf("[SDK] started\n");
    return 0;
}

int insight9_receive_start_camera(int cam_id) {
    if (!g_ctx.initialized) return -1;
    if (cam_id < 0 || cam_id >= CAM_NUM) return -1;
    if (g_ctx.cam_running[cam_id]) return 0;

    if (!cam_bound[cam_id]) {
        try_bind_cameras();
        if (!cam_bound[cam_id]) {
            fprintf(stderr, "[CAM%d][ERR] %s stream not present on this device\n",
                    cam_id, cam_type_name(cam_type_for_id(cam_id)));
            return -1;
        }
        apply_config_to_cam(cam_id);
    }

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
            ctx->format = pixelFormatToFourcc(g_ctx.config.depth_config.pixel_format);
        } else if (cam_id == 1) {
            ctx->width = g_ctx.config.gray_config.width;
            ctx->height = g_ctx.config.gray_config.height;
            ctx->format = pixelFormatToFourcc(g_ctx.config.gray_config.pixel_format);
        } else {
            ctx->width = g_ctx.config.rgb_config.width;
            ctx->height = g_ctx.config.rgb_config.height;
            ctx->format = pixelFormatToFourcc(g_ctx.config.rgb_config.pixel_format);
        }

        // A failure here used to return -1 with cam_running still false and no
        // capture thread created, so the stream was gone until the whole app
        // restarted - and a resolution switch that races a USB re-enumeration
        // hits exactly this path. Hand the stream over to the capture thread
        // instead; it owns the reconnect backoff and will reopen on its own.
        bool opened_ok = true;
        if (init_capture(ctx) < 0) {
            close(ctx->fd);
            ctx->fd = -1;
            opened_ok = false;
        } else if (start_capture(ctx->fd) < 0) {
            close(ctx->fd);
            ctx->fd = -1;
            opened_ok = false;
        }

        ctx->last_timestamp = 0;
        pthread_mutex_unlock(&ctx->fd_lock);

        if (!opened_ok) {
            fprintf(stderr, "[CAM%d][WARN] initial open failed, capture thread will retry\n",
                    cam_id);
        }
    }

    g_ctx.cam_running[cam_id] = true;
    g_ctx.first_frame_received[cam_id] = false;
    clock_gettime(CLOCK_MONOTONIC, &g_ctx.last_frame_time[cam_id]);
    
    if (g_ctx.video_tids[cam_id]) {
        pthread_join(g_ctx.video_tids[cam_id], NULL);
    }
    pthread_create(&g_ctx.video_tids[cam_id], NULL, capture_thread, ctx);
    ctx->tid = g_ctx.video_tids[cam_id];
    
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

void insight9_receive_stop_camera(int cam_id) {
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

int insight9_receive_get_camera_config(int cam_id, int* width, int* height, PixelFormat* format, int* fps) {
    if (!g_ctx.initialized || cam_id < 0 || cam_id >= CAM_NUM) return -1;
    if (!width || !height || !format || !fps) return -1;

    const video_config_t* cur = (cam_id == 0) ? &g_ctx.config.depth_config
                               : (cam_id == 1) ? &g_ctx.config.gray_config
                                               : &g_ctx.config.rgb_config;
    *width  = cur->width;
    *height = cur->height;
    *format = cur->pixel_format;
    *fps    = cur->fps;
    return 0;
}

int insight9_receive_restart_camera(int cam_id) {
    if (!g_ctx.initialized) return -1;
    if (cam_id < 0 || cam_id >= CAM_NUM) return -1;
    
    insight9_receive_stop_camera(cam_id);
    usleep(100000);
    return insight9_receive_start_camera(cam_id);
}

int insight9_receive_set_camera_fps(int cam_id, int fps) {
    if (!g_ctx.initialized) return -1;
    if (cam_id < 0 || cam_id >= CAM_NUM) return -1;
    if (fps <= 0) return -1;
    
    if (cam_id == 0) {
        g_ctx.config.depth_config.fps = fps;
    } else if (cam_id == 1) {
        g_ctx.config.gray_config.fps = fps;
    } else { // cam_id == 2
        g_ctx.config.rgb_config.fps = fps;
    }
    
    printf("[SDK] Set camera %d FPS to %d\n", cam_id, fps);
    return 0;
}

int insight9_receive_set_camera_format(int cam_id, int width, int height, PixelFormat format) {
    if (!g_ctx.initialized) return -1;
    if (cam_id < 0 || cam_id >= CAM_NUM) return -1;
    if (width <= 0 || height <= 0) return -1;

    if (cam_id == 0) {
        g_ctx.config.depth_config.width = width;
        g_ctx.config.depth_config.height = height;
        g_ctx.config.depth_config.pixel_format = format;
    } else if (cam_id == 1) {
        g_ctx.config.gray_config.width = width;
        g_ctx.config.gray_config.height = height;
        g_ctx.config.gray_config.pixel_format = format;
    } else { // cam_id == 2
        g_ctx.config.rgb_config.width = width;
        g_ctx.config.rgb_config.height = height;
        g_ctx.config.rgb_config.pixel_format = format;
    }

    printf("[SDK] Set camera %d format to %dx%d\n", cam_id, width, height);
    return 0;
}

int insight9_receive_switch_camera_config(int cam_id, int width, int height, PixelFormat format, int fps) {
    if (!g_ctx.initialized) return -1;
    if (cam_id < 0 || cam_id >= CAM_NUM) return -1;
    if (width <= 0 || height <= 0 || fps <= 0) return -1;

    printf("[SDK] Switching camera %d to %dx%d@%d...\n", cam_id, width, height, fps);

    insight9_receive_stop_camera(cam_id);
    usleep(1000000);

    if (insight9_receive_set_camera_format(cam_id, width, height, format) != 0) {
        fprintf(stderr, "[SDK] Failed to set camera %d format\n", cam_id);
        return -1;
    }
    if (insight9_receive_set_camera_fps(cam_id, fps) != 0) {
        fprintf(stderr, "[SDK] Failed to set camera %d fps\n", cam_id);
        return -1;
    }

    int ret = insight9_receive_restart_camera(cam_id);
    if (ret == 0) {
        printf("[SDK] Camera %d switched to %dx%d@%d successfully\n", cam_id, width, height, fps);
    } else {
        fprintf(stderr, "[SDK] Failed to restart camera %d after config switch\n", cam_id);
    }
    return ret;
}

int insight9_receive_switch_camera_fps(int cam_id, int fps) {
    if (!g_ctx.initialized || cam_id < 0 || cam_id >= CAM_NUM) return -1;
    // cam 0 is depth and cam 2 is RGB everywhere else in this file; these two
    // wrappers had the mapping inverted, so they read the current geometry out
    // of the wrong config and carried it into the switch.
    video_config_t* cur = (cam_id == 0) ? &g_ctx.config.depth_config
                        : (cam_id == 1) ? &g_ctx.config.gray_config
                                         : &g_ctx.config.rgb_config;
    return insight9_receive_switch_camera_config(cam_id, cur->width, cur->height, cur->pixel_format, fps);
}

int insight9_receive_switch_camera_format(int cam_id, int width, int height, PixelFormat format) {
    if (!g_ctx.initialized || cam_id < 0 || cam_id >= CAM_NUM) return -1;
    video_config_t* cur = (cam_id == 0) ? &g_ctx.config.depth_config
                        : (cam_id == 1) ? &g_ctx.config.gray_config
                                         : &g_ctx.config.rgb_config;
    return insight9_receive_switch_camera_config(cam_id, width, height, format, cur->fps);
}

int insight9_receive_is_camera_running(int cam_id) {
    if (cam_id < 0 || cam_id >= CAM_NUM) return 0;
    return g_ctx.cam_running[cam_id] ? 1 : 0;
}

// Is logical camera cam_id currently attached to a real node? Lets the
// application show "waiting for stream" instead of treating a variant that
// simply lacks a stream as a failure.
int insight9_receive_has_camera(int cam_id) {
    if (cam_id < 0 || cam_id >= CAM_NUM) return 0;
    return cam_bound[cam_id] ? 1 : 0;
}

// Does this camera's node offer a colour format as well? True for the Insight
// 3u stereo node, which streams Y8I or YUYV out of the same interface - so the
// caller knows a colour option exists on camera 1 without having to walk the
// capability list.
int insight9_receive_camera_has_color(int cam_id) {
    if (cam_id < 0 || cam_id >= CAM_NUM) return 0;
    return (cam_families[cam_id] & FAM_COLOR) ? 1 : 0;
}

void insight9_receive_stop(void) {
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
        insight9_receive_stop();
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

    pthread_mutex_lock(&g_ctx.xu_mutex);
    if (g_ctx.xu_control) {
        g_ctx.xu_control->close();
        delete g_ctx.xu_control;
        g_ctx.xu_control = nullptr;
    }
    pthread_mutex_unlock(&g_ctx.xu_mutex);
    pthread_mutex_destroy(&g_ctx.xu_mutex);

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

static bool buildCapabilityList(struct cam_ctx* ctx, std::vector<DeviceCapability>& out) {
    out.clear();
    if (!ctx || !ctx->formats_info) return false;

    for (int f = 0; f < ctx->format_num; ++f) {
        const struct uvc_format_info& fmt = ctx->formats_info[f];
        PixelFormat pf = fourccToPixelFormat(fmt.fcc);
        if (pf == PixelFormat::Unknown) continue;

        for (int r = 0; r < fmt.frames_num; ++r) {
            const struct uvc_frame_info& frame = fmt.frames[r];
            for (int iv = 0; iv < 8; ++iv) {
                unsigned int fps = frame.intervals[iv];
                if (fps == 0) break;
                DeviceCapability cap;
                cap.width = frame.width;
                cap.height = frame.height;
                cap.fps = (int)fps;
                cap.format = pf;
                cap.valid = true;
                out.push_back(cap);
            }
        }
    }

    std::sort(out.begin(), out.end(), [](const DeviceCapability& a, const DeviceCapability& b) {
        if (a.format != b.format) return (int)a.format < (int)b.format;
        long long areaA = (long long)a.width * a.height;
        long long areaB = (long long)b.width * b.height;
        if (areaA != areaB) return areaA < areaB;
        if (a.width != b.width) return a.width < b.width;
        return a.fps < b.fps;
    });
    return true;
}

int insight9_receive_get_device_capability_count(int cam_id, int* count) {
    if (!count || cam_id < 0 || cam_id >= CAM_NUM) return -1;
    std::vector<DeviceCapability> caps;
    if (!buildCapabilityList(&g_ctx.cams[cam_id], caps)) return -1;
    *count = (int)caps.size();
    return 0;
}

int insight9_receive_get_device_capability_by_index(int cam_id, int index, DeviceCapability* cap) {
    if (!cap || cam_id < 0 || cam_id >= CAM_NUM || index < 0) return -1;
    std::vector<DeviceCapability> caps;
    if (!buildCapabilityList(&g_ctx.cams[cam_id], caps)) return -1;
    if (index >= (int)caps.size()) return -1;
    *cap = caps[index];
    return 0;
}

extern "C" {

int insight9_receive_set_active_camera(int cam_id) {
    return callXUWithRetry("setActiveCamera", [cam_id](viewer::UvcExtensionUnit& xu) {
        return xu.setActiveCamera(static_cast<uint8_t>(cam_id));
    }) ? 0 : -1;
}

int insight9_receive_get_active_camera(int *cam_id) {
    if (!cam_id) return -1;
    uint8_t val = 0;
    if (!callXUWithRetry("getActiveCamera", [&val](viewer::UvcExtensionUnit& xu) {
            return xu.getActiveCamera(val);
        })) return -1;
    *cam_id = val;
    return 0;
}

int insight9_check_camera_params(const camera_params *params) {
    if (!params) return -1;
    return is_camera_params_valid(params) ? 0 : -1;
}

int insight9_receive_set_camera_params(const camera_params *params) {
    if (!params || !is_camera_params_valid(params)) return -1;
    viewer::camera_params xu_params;
    memcpy(&xu_params, params, sizeof(viewer::camera_params));
    return callXUWithRetry("writeCurrentCameraParams", [xu_params](viewer::UvcExtensionUnit& xu) {
        return xu.writeCurrentCameraParams(xu_params);
    }) ? 0 : -1;
}

int insight9_receive_get_camera_params(camera_params *params) {
    if (!params) return -1;
    viewer::camera_params xu_params;
    if (!callXUWithRetry("readCurrentCameraParams", [&xu_params](viewer::UvcExtensionUnit& xu) {
            return xu.readCurrentCameraParams(xu_params);
        })) return -1;
    memcpy(params, &xu_params, sizeof(viewer::camera_params));
    return 0;
}

int insight9_receive_set_camera_params_for(int cam_id, const camera_params *params) {
    if (!params || !is_camera_params_valid(params)) return -1;
    viewer::camera_params xu_params;
    memcpy(&xu_params, params, sizeof(viewer::camera_params));
    return callXUWithRetry("writeCameraParams", [cam_id, xu_params](viewer::UvcExtensionUnit& xu) {
        return xu.writeCameraParams(static_cast<uint8_t>(cam_id), xu_params);
    }) ? 0 : -1;
}

int insight9_receive_get_camera_params_for(int cam_id, camera_params *params) {
    if (!params) return -1;
    viewer::camera_params xu_params;
    if (!callXUWithRetry("readCameraParams", [cam_id, &xu_params](viewer::UvcExtensionUnit& xu) {
            return xu.readCameraParams(static_cast<uint8_t>(cam_id), xu_params);
        })) return -1;
    memcpy(params, &xu_params, sizeof(viewer::camera_params));
    return 0;
}

int insight9_receive_get_cached_initial_params(int cam_id, camera_params* params) {
    if (!params || cam_id < 0 || cam_id >= CAM_NUM) return -1;
    if (!g_ctx.hasCachedInitialParams[cam_id]) return -1;
    *params = g_ctx.cachedInitialParams[cam_id];
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
    viewer::camera_calib xu_calib;
    if (!callXUWithRetry("readCameraCalib", [cam_idx, &xu_calib](viewer::UvcExtensionUnit& xu) {
            return xu.readCameraCalib(static_cast<uint8_t>(cam_idx), xu_calib);
        })) return -1;
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
    // uint8_t val = 0;
    // if (!callXUWithRetry("readCurrentFps", [&val](viewer::UvcExtensionUnit& xu) {
    //         return xu.readCurrentFps(val);
    //     })) return -1;
    // const int validFps[] = {0, 20, 30, 40, 50};
    // if (val >= 0 && val < (int)(sizeof(validFps)/sizeof(validFps[0]))) {
    //     *fps = validFps[val];
    // } else {
    //     *fps = 0;
    // }
    *fps = 30;
    return 0;
}

int insight9_receive_get_vio_status(int* status) {
    if (!status) return -1;
    uint8_t val = 0;
    if (!callXUWithRetry("readVioStatus", [&val](viewer::UvcExtensionUnit& xu) {
            return xu.readVioStatus(val);
        })) return -1;
    *status = val;
    return 0;
}

const char* insight9_receive_get_hardware_type(void) {
    static std::string result;
    viewer::camera_params xu_params;
    bool ok = callXUWithRetry("getHardwareType", [&xu_params](viewer::UvcExtensionUnit& xu) {
        return xu.readCurrentCameraParams(xu_params);
    });
    if (!ok) return "unknown";
    uint8_t model = xu_params.hardware_model;
    const char* models[] = {"Insight 9", "Insight 7", "Insight 3"};
    if (model < 3) {
        result = models[model];
        return result.c_str();
    }
    return "unknown";
}

} // extern "C"