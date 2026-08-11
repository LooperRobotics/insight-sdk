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
    int32_t  custom_data1;
    int32_t  custom_data2;
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
    int32_t  custom_data1;
    int32_t  custom_data2;
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

static int compare_device_numbers(const void *a, const void *b) {
    const char *sa = *(const char **)a;
    const char *sb = *(const char **)b;
    int na = get_device_number(sa);
    int nb = get_device_number(sb);
    return na - nb;
}

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

int find_hid_devices_by_vid_pid(unsigned int target_vid, unsigned int target_pid,
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

        char device_path[MAX_PATH];
        snprintf(device_path, sizeof(device_path), "%s/device", hidraw_sysfs);
        if (realpath(device_path, usb_path) == NULL) continue;

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
            if (pid == target_pid) {
                count++;
            }
        }
    }
    closedir(dir);

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

void refresh_hid_device_path(int idx) {
    if (idx < 0 || idx >= HID_NUM) return;

    char hid_list[10][MAX_PATH] = {{0}};
    int hid_count = find_hid_devices_by_vid_pid(VENDOR_ID, PRODUCT_ID, hid_list, 10);
    if (hid_count < HID_NUM) {
        fprintf(stderr, "[HID%d][WARN] found %d HID devices (< %d), skip reconnect\n",
                idx, hid_count, HID_NUM);
        return;
    }

    if (strcmp(hid_list[idx], g_ctx.hid_devs[idx]) != 0) {
        printf("[HID%d] rematched %s -> %s\n",
               idx, g_ctx.hid_devs[idx], hid_list[idx]);
        strcpy(g_ctx.hid_devs[idx], hid_list[idx]);
    }
    if (get_hid_usb_device_path(g_ctx.hid_devs[idx], g_ctx.hid_usb_paths[idx], MAX_PATH) < 0) {
        g_ctx.hid_usb_paths[idx][0] = '\0';
    }
}

void *hid_thread(void *arg) {
    int idx = (int)(intptr_t)arg;
    const char *device = g_ctx.hid_devs[idx];
    int fd = -1;
    uint64_t last_accel_ts = 0;
    uint64_t last_gyro_ts = 0;
    uint64_t last_imu_ts = 0;
    float last_ax = 0, last_ay = 0, last_az = 0;
    float last_gx = 0, last_gy = 0, last_gz = 0;

    printf("[HID%d] thread started, dev=%s\n", idx, g_ctx.hid_devs[idx]);
    while (!g_ctx.running) {usleep(100 * 000);}
    while (g_ctx.running) {
        device = g_ctx.hid_devs[idx];
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
            last_accel_ts = 0;
            last_gyro_ts = 0;
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
                if (accel->timestamp == last_accel_ts) {
                    continue;
                }
                last_accel_ts = accel->timestamp;
                last_ax = (float)accel->accel_x / ACCEL_SCALE_FACTOR * ACCEL_SCALE_LOOPERHUB;
                last_ay = (float)accel->accel_y / ACCEL_SCALE_FACTOR * ACCEL_SCALE_LOOPERHUB;
                last_az = (float)accel->accel_z / ACCEL_SCALE_FACTOR * ACCEL_SCALE_LOOPERHUB;
                if (g_ctx.imu_cb) {
                    g_ctx.imu_cb(last_ax, last_ay, last_az,
                                 last_gx, last_gy, last_gz,
                                 accel->timestamp,
                                 g_ctx.imu_userdata);
                }
            } else if (report_id == 0x02 && n >= (int)sizeof(struct imu_gyro_report_t)) {
                struct imu_gyro_report_t *gyro = (struct imu_gyro_report_t*)buf;
                if (gyro->timestamp == last_gyro_ts) {
                    continue;
                }
                last_gyro_ts = gyro->timestamp;
                last_gx = (float)gyro->gyro_x / GYRO_SCALE_FACTOR * GYRO_SCALE_LOOPERHUB;
                last_gy = (float)gyro->gyro_y / GYRO_SCALE_FACTOR * GYRO_SCALE_LOOPERHUB;
                last_gz = (float)gyro->gyro_z / GYRO_SCALE_FACTOR * GYRO_SCALE_LOOPERHUB;
                if (g_ctx.imu_cb) {
                    g_ctx.imu_cb(last_ax, last_ay, last_az,
                                 last_gx, last_gy, last_gz,
                                 gyro->timestamp,
                                 g_ctx.imu_userdata);
                }
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

int hid_init(void)
{
    // Find HID devices.
    char hid_list[10][MAX_PATH] = {{0}};
    int hid_count = find_hid_devices_by_vid_pid(VENDOR_ID, PRODUCT_ID, hid_list, 10);

    g_ctx.hid_devs[0][0] = '\0';
    g_ctx.hid_devs[1][0] = '\0';

    if (hid_count >= 1) {
        strcpy(g_ctx.hid_devs[0], hid_list[0]);
        printf("[SDK] selected HID: IMU=%s\n", g_ctx.hid_devs[0]);
    } else {
        fprintf(stderr, "[SDK][WARN] No HID devices found\n");
    }

    if (hid_count >= 2) {
        strcpy(g_ctx.hid_devs[1], hid_list[1]);
        printf("[SDK] selected HID: VIO=%s\n", g_ctx.hid_devs[1]);
    } else {
        g_ctx.hid_devs[1][0] = '\0';
    }

    if (g_ctx.hid_devs[0][0] != '\0') {
        if (get_hid_usb_device_path(g_ctx.hid_devs[0], g_ctx.hid_usb_paths[0], MAX_PATH) < 0) {
            g_ctx.hid_usb_paths[0][0] = '\0';
        }
    }
    if (g_ctx.hid_devs[1][0] != '\0') {
        if (get_hid_usb_device_path(g_ctx.hid_devs[1], g_ctx.hid_usb_paths[1], MAX_PATH) < 0) {
            g_ctx.hid_usb_paths[1][0] = '\0';
        }
    }
    printf("[SDK] selected HID: IMU=%s VIO=%s\n", g_ctx.hid_devs[0], g_ctx.hid_devs[1]);

    for (int i = 0; i < HID_NUM; i++) {
        printf("[HID%d] starting thread for %s\n", i, g_ctx.hid_devs[i]);
        if (g_ctx.hid_devs[i][0] != '\0') {
            pthread_create(&g_ctx.hid_tids[i], NULL, hid_thread, (void*)(intptr_t)i);
        }
    }

    return 0;
}