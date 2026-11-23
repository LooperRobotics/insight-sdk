#ifndef INSIGHT9_HID_H
#define INSIGHT9_HID_H

#include "Insight_9_receive.h"
#include "UvcExtensionUnit.hpp"
#include <atomic>
#include <pthread.h>
#include <sys/time.h>
#include "Insight_9_receive_internal.h"

#define CAM_NUM 3
#define HID_NUM 2
#define MAX_PATH 1024

// HID thread index == role.
#define HID_ROLE_UNKNOWN (-1)
#define HID_ROLE_IMU     0
#define HID_ROLE_VIO     1

int get_hid_usb_device_path(const char *hid_dev, char *usb_path, size_t usb_path_size);
// Resolves g_ctx.hid_devs[role] by report descriptor (interface number as
// cross-check); clears it when that role is not present.
void refresh_hid_device_path(int idx, int verbose);
// Resolves both roles from scratch.
void hid_discover(int verbose);
void *hid_thread(void *arg);
int hid_init(void);

#endif // INSIGHT9_HID_H
