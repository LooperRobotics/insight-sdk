#ifndef INSIGHT9_HID_H
#define INSIGHT9_HID_H

#include "Insight_9_receive.h"
#include "UvcExtensionUnit.hpp"
#include <atomic>
#include <pthread.h>
#include <sys/time.h>

#define CAM_NUM 3
#define HID_NUM 2
#define MAX_PATH 1024

int get_hid_usb_device_path(const char *hid_dev, char *usb_path, size_t usb_path_size);
int find_hid_devices_by_vid_pid(unsigned int target_vid, unsigned int target_pid,
                               char dev_paths[][MAX_PATH], int max_devs);
void refresh_hid_device_path(int idx);
void *hid_thread(void *arg);
int hid_init(void);

#endif // INSIGHT9_HID_H
