#include "Insight_9_receive.h"
#include <stdio.h>
#include <unistd.h>

void my_image_cb(int cam_id, uint8_t *data, size_t size, int w, int h,
                 unsigned int fmt, uint64_t ts, uint64_t ts_right, void *user) {
    // if (cam_id == 1) {
        printf("Left ts=%llu, Right ts=%llu\n", ts, ts_right);
    // } else {
        printf("Image[%d]: %zub, %dx%d, ts=%lu\n", cam_id, size, w, h, ts);
    // }
}

void my_imu_cb(float ax, float ay, float az,
               float gx, float gy, float gz,
               uint32_t ts, void *user) {
    printf("IMU: ax=%f ay=%f az=%f gx=%f gy=%f gz=%f ts=%u\n",
           ax, ay, az, gx, gy, gz, ts);
}

void my_vio_cb(float px, float py, float pz,
               float qx, float qy, float qz, float qw,
               uint32_t seq, void *user) {
    printf("VIO: pos=(%f %f %f) ori=(%f %f %f %f) seq=%u\n", px, py, pz, qx, qy, qz, qw, seq);
}

int main() {
    insight9_receive_init();
    insight9_receive_register_image_callback(my_image_cb, NULL);
    insight9_receive_register_imu_callback(my_imu_cb, NULL);
    insight9_receive_register_vio_callback(my_vio_cb, NULL);

    if (insight9_receive_start() != 0) {
        fprintf(stderr, "start failed\n");
        return -1;
    }

    printf("Press Enter to stop...\n");
    getchar();

    insight9_receive_stop();
    insight9_receive_cleanup();
    return 0;
}
