#include "hid.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <poll.h>
#include <assert.h>
#include <ctype.h>

#define GYRO_SCALE_FACTOR -610.464183381
#define ACCEL_SCALE_FACTOR -0.366459184

#define GYRO_SCALE_LOOPERHUB 0.00106526
#define ACCEL_SCALE_LOOPERHUB 0.0035913

#define HID_RECONNECT_BACKOFF_BASE_MS 500
#define HID_RECONNECT_BACKOFF_MAX_MS  10000

extern sdk_ctx_t g_ctx;

static void trim_newline(char *str) {
    size_t len = strlen(str);
    if (len > 0 && str[len - 1] == '\n')
        str[len - 1] = '\0';
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

static int get_device_number(const char *dev_path) {
    const char *p = strrchr(dev_path, '/');
    if (!p) p = dev_path;
    while (*p && !isdigit((unsigned char)*p)) p++;
    return atoi(p);
}

struct __attribute__((packed)) imu_gyro_report_t {
    uint8_t  report_id;
    uint8_t  sensor_state;
    uint64_t timestamp;
    int32_t  gyro_x;
    int32_t  gyro_y;
    int32_t  gyro_z;
    uint32_t timestamp_l;
    uint32_t timestamp_h;
    int16_t  custom_data3;
    int16_t  custom_data4;
    int16_t  custom_data5;
    int8_t   custom_data6;
    int8_t   custom_data7;
};

struct __attribute__((packed)) imu_accel_report_t {
    uint8_t  report_id;
    uint8_t  sensor_state;
    uint64_t timestamp;
    int32_t  accel_x;
    int32_t  accel_y;
    int32_t  accel_z;
    uint32_t timestamp_l;
    uint32_t timestamp_h;
    int16_t  custom_data3;
    int16_t  custom_data4;
    int16_t  custom_data5;
    int8_t   custom_data6;
    int8_t   custom_data7;
};


struct __attribute__((packed)) vio_hid_payload {
    uint64_t timestamp;
    float px, py, pz;
    float qx, qy, qz, qw;
    uint8_t seq;
    uint8_t reserved[3];
};

int get_hid_usb_device_path(const char *hid_dev, char *usb_path, size_t usb_path_size) {
    const char *base = strrchr(hid_dev, '/');
    if (base)
        base++;
    else
        base = hid_dev;

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

static_assert(sizeof(struct vio_hid_payload) == 40, "VIO payload must match the 40-byte HID report");

// ---------------------------------------------------------------------------
// HID role detection
//
// The device exposes two HID interfaces: the IMU (HID Sensor usage page 0x20,
// accel = report ID 1, gyro = report ID 2) and the VIO pose stream (vendor
// usage page 0xFF00, one 40-byte input report without a report ID). Firmware
// places them on USB interfaces 5 and 6, but that depends on how the gadget is
// composed, and hidrawN numbers depend on host enumeration order - so neither
// is used on its own. The report descriptor decides the role; the interface
// number is a cross-check and the fallback when the descriptor is unreadable.
// ---------------------------------------------------------------------------

#define INSIGHT_HID_IFACE_IMU          5
#define INSIGHT_HID_IFACE_VIO          6
#define HID_USAGE_PAGE_SENSOR          0x20
#define HID_USAGE_PAGE_VENDOR_VIO      0xFF00
#define ROS_MODE_PRODUCT_ID            0x0731
#define MAX_HID_CANDIDATES             16

static pthread_mutex_t g_hid_path_mutex = PTHREAD_MUTEX_INITIALIZER;

static const char *hid_role_name(int role) {
    switch (role) {
    case HID_ROLE_IMU: return "IMU";
    case HID_ROLE_VIO: return "VIO";
    default:           return "UNKNOWN";
    }
}

typedef struct {
    char path[MAX_PATH];       // /dev/hidrawN
    char usb_path[MAX_PATH];   // sysfs USB device directory (groups interfaces of one unit)
    int  iface;                // bInterfaceNumber, -1 if unknown
    int  desc_ok;              // report descriptor was read and parsed
    unsigned usage_page;       // first Usage Page in the descriptor
    unsigned input_bits;       // sum of all Input items (meaningful without report IDs)
    int  has_report_id;
    int  desc_role;            // role from the descriptor alone
    int  iface_role;           // role from the interface number alone
    int  role;                 // final role
} hid_candidate_t;

// Walks the short items of a HID report descriptor. Only collects what is
// needed to tell IMU from VIO; it is not a general-purpose parser.
static void parse_report_descriptor(const uint8_t *d, int len, hid_candidate_t *c) {
    unsigned report_size = 0, report_count = 0;
    int have_page = 0;
    c->usage_page = 0;
    c->input_bits = 0;
    c->has_report_id = 0;
    for (int i = 0; i < len;) {
        uint8_t b = d[i];
        if (b == 0xFE) {                    // long item: FE <size> <tag> <data...>
            if (i + 1 >= len) break;
            i += 3 + d[i + 1];
            continue;
        }
        int size = b & 0x03;
        if (size == 3) size = 4;
        if (i + 1 + size > len) break;
        unsigned val = 0;
        for (int k = 0; k < size; k++) val |= (unsigned)d[i + 1 + k] << (8 * k);
        switch (b & 0xFC) {
        case 0x04: if (!have_page) { c->usage_page = val; have_page = 1; } break; // Usage Page
        case 0x74: report_size = val; break;                                    // Report Size
        case 0x94: report_count = val; break;                                   // Report Count
        case 0x84: c->has_report_id = 1; break;                                 // Report ID
        case 0x80: c->input_bits += report_size * report_count; break;          // Input
        default: break;
        }
        i += 1 + size;
    }
}

static int read_report_descriptor(const char *hidraw_name, hid_candidate_t *c) {
    char path[MAX_PATH];
    uint8_t buf[4096];
    snprintf(path, sizeof(path), "/sys/class/hidraw/%s/device/report_descriptor", hidraw_name);
    int fd = open(path, O_RDONLY);
    if (fd < 0) return -1;
    int n = (int)read(fd, buf, sizeof(buf));
    close(fd);
    if (n <= 0) return -1;
    parse_report_descriptor(buf, n, c);
    return 0;
}

// /sys/class/hidraw/hidrawN/device resolves to .../<usb-if>/<hid-dev>; the
// parent directory is the USB interface and carries bInterfaceNumber.
static int read_interface_number(const char *hidraw_name) {
    char path[MAX_PATH], resolved[MAX_PATH], buf[16];
    snprintf(path, sizeof(path), "/sys/class/hidraw/%s/device", hidraw_name);
    if (realpath(path, resolved) == NULL) return -1;
    char *slash = strrchr(resolved, '/');
    if (!slash) return -1;
    *slash = '\0';
    snprintf(path, sizeof(path), "%s/bInterfaceNumber", resolved);
    if (read_sysfs_file(path, buf, sizeof(buf)) != 0) return -1;
    unsigned v = 0;
    if (sscanf(buf, "%x", &v) != 1) return -1;
    return (int)v;
}

static void classify_candidate(hid_candidate_t *c, int verbose) {
    c->desc_role = HID_ROLE_UNKNOWN;
    if (c->desc_ok) {
        if (c->usage_page == HID_USAGE_PAGE_SENSOR) {
            c->desc_role = HID_ROLE_IMU;
        } else if (c->usage_page == HID_USAGE_PAGE_VENDOR_VIO && !c->has_report_id &&
                   c->input_bits == 8 * sizeof(struct vio_hid_payload)) {
            c->desc_role = HID_ROLE_VIO;
        }
    }

    c->iface_role = HID_ROLE_UNKNOWN;
    if (c->iface == INSIGHT_HID_IFACE_IMU) c->iface_role = HID_ROLE_IMU;
    else if (c->iface == INSIGHT_HID_IFACE_VIO) c->iface_role = HID_ROLE_VIO;

    if (c->desc_role != HID_ROLE_UNKNOWN) {
        c->role = c->desc_role;
        if (verbose && c->iface_role != c->desc_role) {
            fprintf(stderr, "[SDK][WARN] %s: report descriptor says %s but USB interface %d "
                    "is expected to be %s; trusting the descriptor (firmware gadget layout changed?)\n",
                    c->path, hid_role_name(c->desc_role), c->iface, hid_role_name(c->iface_role));
        }
    } else if (!c->desc_ok && c->iface_role != HID_ROLE_UNKNOWN) {
        c->role = c->iface_role;
        if (verbose) {
            fprintf(stderr, "[SDK][WARN] %s: report descriptor unreadable, "
                    "assuming %s from USB interface %d\n",
                    c->path, hid_role_name(c->iface_role), c->iface);
        }
    } else {
        c->role = HID_ROLE_UNKNOWN;
        if (verbose) {
            fprintf(stderr, "[SDK][ERR] %s (USB interface %d): unrecognized HID interface "
                    "(usage page 0x%04x, %u input bits, report IDs %s); not used\n",
                    c->path, c->iface, c->usage_page, c->input_bits,
                    c->has_report_id ? "yes" : "no");
        }
    }
}

static int compare_candidates(const void *a, const void *b) {
    return get_device_number(((const hid_candidate_t *)a)->path) -
           get_device_number(((const hid_candidate_t *)b)->path);
}

// Lists every hidraw node that belongs to a USB device with the given VID/PID.
static int enumerate_hid_candidates(unsigned int target_vid, unsigned int target_pid,
                                    hid_candidate_t *out, int max_out, int verbose) {
    DIR *dir = opendir("/sys/class/hidraw");
    if (!dir) return 0;
    struct dirent *entry;
    int count = 0;
    while ((entry = readdir(dir)) != NULL && count < max_out) {
        if (entry->d_name[0] == '.') continue;
        hid_candidate_t *c = &out[count];
        memset(c, 0, sizeof(*c));
        snprintf(c->path, sizeof(c->path), "/dev/%s", entry->d_name);
        if (get_hid_usb_device_path(c->path, c->usb_path, sizeof(c->usb_path)) != 0) continue;
        unsigned int vid, pid;
        if (parse_usb_vid_pid(c->usb_path, &vid, &pid) != 0) continue;
        if (vid != target_vid || pid != target_pid) continue;
        c->iface = read_interface_number(entry->d_name);
        c->desc_ok = read_report_descriptor(entry->d_name, c) == 0;
        count++;
    }
    closedir(dir);
    qsort(out, count, sizeof(out[0]), compare_candidates);
    for (int i = 0; i < count; i++) classify_candidate(&out[i], verbose);
    return count;
}

// Picks the hidraw node for `role`. Prefers the unit at `prefer_usb` so that
// IMU and VIO stay on the same device when several are attached.
static int find_hid_by_role(int role, const char *prefer_usb,
                            char out_path[MAX_PATH], char out_usb[MAX_PATH], int verbose) {
    hid_candidate_t cands[MAX_HID_CANDIDATES];
    int n = enumerate_hid_candidates(VENDOR_ID, PRODUCT_ID, cands, MAX_HID_CANDIDATES, verbose);
    int pick = -1, matches = 0;
    for (int i = 0; i < n; i++) {
        if (cands[i].role != role) continue;
        matches++;
        if (pick < 0) pick = i;
        if (prefer_usb && prefer_usb[0] && strcmp(cands[i].usb_path, prefer_usb) == 0) {
            pick = i;
            break;
        }
    }
    if (pick < 0) return -1;
    if (verbose && matches > 1) {
        printf("[SDK] %d %s HID interfaces found (multiple units?), using %s\n",
               matches, hid_role_name(role), cands[pick].path);
    }
    snprintf(out_path, MAX_PATH, "%s", cands[pick].path);
    snprintf(out_usb, MAX_PATH, "%s", cands[pick].usb_path);
    return 0;
}

// ROS mode re-enumerates under another PID with only an NCM function and no
// HID at all, so this looks at the USB bus rather than hidraw.
static int ros_mode_device_present(void) {
    DIR *usb = opendir("/sys/bus/usb/devices");
    if (!usb) return 0;
    struct dirent *entry;
    int found = 0;
    char path[MAX_PATH];
    while (!found && (entry = readdir(usb)) != NULL) {
        if (entry->d_name[0] == '.') continue;
        snprintf(path, sizeof(path), "/sys/bus/usb/devices/%s", entry->d_name);
        unsigned int vid, pid;
        if (parse_usb_vid_pid(path, &vid, &pid) == 0 &&
            vid == VENDOR_ID && pid == ROS_MODE_PRODUCT_ID) {
            found = 1;
        }
    }
    closedir(usb);
    return found;
}

// Re-resolves the hidraw node for `role` (the thread index is the role). When
// the role is not present the path is cleared, so the thread never reopens a
// stale hidrawN that the kernel may since have handed to another interface.
void refresh_hid_device_path(int idx, int verbose) {
    if (idx < 0 || idx >= HID_NUM) return;

    char prefer[MAX_PATH];
    pthread_mutex_lock(&g_hid_path_mutex);
    snprintf(prefer, sizeof(prefer), "%s",
             g_ctx.hid_usb_paths[idx][0] ? g_ctx.hid_usb_paths[idx] : g_ctx.hid_usb_paths[1 - idx]);
    pthread_mutex_unlock(&g_hid_path_mutex);

    char path[MAX_PATH] = {0}, usb[MAX_PATH] = {0};
    int found = find_hid_by_role(idx, prefer, path, usb, verbose) == 0;

    pthread_mutex_lock(&g_hid_path_mutex);
    if (found) {
        if (strcmp(path, g_ctx.hid_devs[idx]) != 0) {
            printf("[HID%d] %s rematched %s -> %s\n", idx, hid_role_name(idx),
                   g_ctx.hid_devs[idx][0] ? g_ctx.hid_devs[idx] : "(none)", path);
            snprintf(g_ctx.hid_devs[idx], MAX_PATH, "%s", path);
        }
        snprintf(g_ctx.hid_usb_paths[idx], MAX_PATH, "%s", usb);
    } else {
        g_ctx.hid_devs[idx][0] = '\0';
    }
    pthread_mutex_unlock(&g_hid_path_mutex);
}

void hid_discover(int verbose) {
    pthread_mutex_lock(&g_hid_path_mutex);
    for (int i = 0; i < HID_NUM; i++) {
        g_ctx.hid_devs[i][0] = '\0';
        g_ctx.hid_usb_paths[i][0] = '\0';
    }
    pthread_mutex_unlock(&g_hid_path_mutex);

    // IMU first, then VIO pinned to the same unit.
    refresh_hid_device_path(HID_ROLE_IMU, verbose);
    refresh_hid_device_path(HID_ROLE_VIO, verbose);

    if (verbose && (!g_ctx.hid_devs[HID_ROLE_IMU][0] || !g_ctx.hid_devs[HID_ROLE_VIO][0])) {
        if (ros_mode_device_present()) {
            fprintf(stderr, "[SDK][WARN] device is in ROS mode (PID 0x%04x): it exposes no IMU/VIO HID; "
                    "switch it to UVC mode to receive IMU/VIO\n", ROS_MODE_PRODUCT_ID);
        } else {
            fprintf(stderr, "[SDK][WARN] IMU or VIO HID interface not found on VID=0x%04x PID=0x%04x; "
                    "it will attach when it appears\n", VENDOR_ID, PRODUCT_ID);
        }
    }
    printf("[SDK] selected HID: IMU=%s VIO=%s\n",
           g_ctx.hid_devs[HID_ROLE_IMU][0] ? g_ctx.hid_devs[HID_ROLE_IMU] : "(pending)",
           g_ctx.hid_devs[HID_ROLE_VIO][0] ? g_ctx.hid_devs[HID_ROLE_VIO] : "(pending)");
}

static int hid_reconnect_backoff_ms(int attempt) {
    if (attempt < 1) attempt = 1;
    long delay = HID_RECONNECT_BACKOFF_BASE_MS;
    for (int i = 1; i < attempt && delay < HID_RECONNECT_BACKOFF_MAX_MS; ++i) delay <<= 1;
    if (delay > HID_RECONNECT_BACKOFF_MAX_MS) delay = HID_RECONNECT_BACKOFF_MAX_MS;
    return (int)delay;
}

void *hid_thread(void *arg) {
    int idx = (int)(intptr_t)arg;
    const char *device = g_ctx.hid_devs[idx];
    int fd = -1;
    int reconnect_fails = 0;
    uint64_t accel_ts = 0;
    uint64_t gyro_ts = 0;
    uint64_t last_accel_ts = 0;
    uint64_t last_gyro_ts = 0;
    uint64_t last_imu_ts = 0;
    float last_ax = 0, last_ay = 0, last_az = 0;
    float last_gx = 0, last_gy = 0, last_gz = 0;
    int accel_dirty = 0;
    int gyro_dirty = 0;

    printf("[HID%d] %s thread started, dev=%s\n", idx, hid_role_name(idx),
           g_ctx.hid_devs[idx][0] ? g_ctx.hid_devs[idx] : "(pending)");
    while (!g_ctx.running) {usleep(100 * 1000);}
    while (g_ctx.running) {
        device = g_ctx.hid_devs[idx];
        if (fd < 0) {
            // Log classification problems on the first attempt and then
            // periodically, not on every backoff retry.
            refresh_hid_device_path(idx, reconnect_fails % 8 == 0);
            device = g_ctx.hid_devs[idx];

            if (device[0] == '\0') {
                int delay = hid_reconnect_backoff_ms(++reconnect_fails);
                fprintf(stderr, "[HID%d][WARN] device not enumerated yet, retry in %dms (attempt %d)\n",
                        idx, delay, reconnect_fails);
                usleep(delay * 1000);
                continue;
            }

            fd = open(device, O_RDONLY | O_NONBLOCK);
            if (fd < 0) {
                int delay = hid_reconnect_backoff_ms(++reconnect_fails);
                if (errno != ENOENT)
                    fprintf(stderr, "[HID%d][ERR] open %s: %s, retry in %dms (attempt %d)\n",
                            idx, device, strerror(errno), delay, reconnect_fails);
                usleep(delay * 1000);
                continue;
            }
            printf("[HID%d] opened %s as %s\n", idx, device, hid_role_name(idx));
            reconnect_fails = 0;
            last_accel_ts = 0;
            last_gyro_ts = 0;
            accel_dirty = 0;
            gyro_dirty = 0;
        }

        if (!g_ctx.running) break;

        struct pollfd pfd = {.fd = fd, .events = POLLIN};
        int ret = poll(&pfd, 1, 100);
        if (ret < 0) {
            if (errno == EINTR) {
                if (!g_ctx.running) break;
                continue;
            }
            fprintf(stderr, "[HID%d][ERR] poll: %s\n", idx, strerror(errno));
            close(fd);
            fd = -1;
            continue;
        }
        if (ret == 0) {
            static int cnt = 0;
            if (++cnt % 20 == 0)
                printf("[HID%d] poll timeout\n", idx);
            continue;
        }

        uint8_t buf[256];
        int n = read(fd, buf, sizeof(buf));
        if (n < 0) {
            fprintf(stderr, "[HID%d][ERR] read: %s\n", idx, strerror(errno));
            close(fd);
            fd = -1;
            continue;
        }

        if (n == 0) continue;

        uint8_t report_id = buf[0];

        if (idx == 0) {
            if (report_id == 0x01 && n >= (int)sizeof(struct imu_accel_report_t)) {
                struct imu_accel_report_t *accel = (struct imu_accel_report_t*)buf;
                accel_ts = ((uint64_t)(accel->timestamp_h & 0xffffffff) << 32) |
                                    (uint64_t)(accel->timestamp_l & 0xffffffff);
                if (accel_ts == last_accel_ts) {
                    continue;
                }
                last_accel_ts = accel_ts;
                last_ax = (float)accel->accel_x / ACCEL_SCALE_FACTOR * ACCEL_SCALE_LOOPERHUB;
                last_ay = (float)accel->accel_y / ACCEL_SCALE_FACTOR * ACCEL_SCALE_LOOPERHUB;
                last_az = (float)accel->accel_z / ACCEL_SCALE_FACTOR * ACCEL_SCALE_LOOPERHUB;
                accel_dirty = 1;
            } else if (report_id == 0x02 && n >= (int)sizeof(struct imu_gyro_report_t)) {
                struct imu_gyro_report_t *gyro = (struct imu_gyro_report_t*)buf;
                gyro_ts = ((uint64_t)(gyro->timestamp_h & 0xffffffff) << 32) | 
                                    (uint64_t)(gyro->timestamp_l & 0xffffffff);
                if (gyro_ts== last_gyro_ts) {
                    continue;
                }
                last_gyro_ts = gyro_ts;
                last_gx = (float)gyro->gyro_x / GYRO_SCALE_FACTOR * GYRO_SCALE_LOOPERHUB;
                last_gy = (float)gyro->gyro_y / GYRO_SCALE_FACTOR * GYRO_SCALE_LOOPERHUB;
                last_gz = (float)gyro->gyro_z / GYRO_SCALE_FACTOR * GYRO_SCALE_LOOPERHUB;
                gyro_dirty = 1;
            }

            if (accel_dirty && gyro_dirty) {
                if (g_ctx.imu_cb) {
                    uint64_t ts = (last_accel_ts > last_gyro_ts) ? last_accel_ts : last_gyro_ts;
                    g_ctx.imu_cb(last_ax, last_ay, last_az, last_gx, last_gy, last_gz, ts, g_ctx.imu_userdata);
                }
                accel_dirty = 0;
                gyro_dirty = 0;
            }
        } else {
            struct vio_hid_payload payload;
            if (n == (int)sizeof(payload)) {
                memcpy(&payload, buf, sizeof(payload));
                if (payload.timestamp != 0 && payload.timestamp == last_imu_ts) continue;
                last_imu_ts = payload.timestamp;
                if (g_ctx.vio_cb) {
                    g_ctx.vio_cb(payload.px, payload.py, payload.pz,
                                 payload.qx, payload.qy, payload.qz, payload.qw,
                                 payload.timestamp,
                                 g_ctx.vio_userdata);
                }
            }
        }
    }

    if (fd >= 0) close(fd);
    printf("[HID%d] thread exited\n", idx);
    return NULL;
}

int hid_init(void)
{
    // Re-resolve by role: the hidraw numbers may have changed since init().
    hid_discover(1);

    // Start both threads even when a role is still missing; the thread keeps
    // looking for it (with backoff) and attaches once it enumerates.
    for (int i = 0; i < HID_NUM; i++) {
        printf("[HID%d] starting %s thread for %s\n", i, hid_role_name(i),
               g_ctx.hid_devs[i][0] ? g_ctx.hid_devs[i] : "(pending)");
        pthread_create(&g_ctx.hid_tids[i], NULL, hid_thread, (void*)(intptr_t)i);
    }

    return 0;
}
