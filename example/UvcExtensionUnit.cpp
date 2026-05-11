#include "UvcExtensionUnit.hpp"
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <cerrno>

namespace viewer {

// 包装 uvc_control_query（需要从内核头文件或自己实现）
static int uvc_control_query(int fd, __u8 unit, __u8 selector,
                             __u8 query, void *data, __u16 size)
{
    struct uvc_xu_control_query xq;
    memset(&xq, 0, sizeof(xq));
    xq.unit = unit;
    xq.selector = selector;
    xq.query = query;
    xq.size = size;
    xq.data = static_cast<__u8*>(data);
    return ioctl(fd, UVCIOC_CTRL_QUERY, &xq);
}

static int get_selector_len(int fd, __u8 unit, __u8 selector, __u16 *len)
{
    __u16 tmp = 0;
    if (uvc_control_query(fd, unit, selector, UVC_GET_LEN, &tmp, sizeof(tmp)) == 0) {
        *len = tmp;
        return 0;
    }
    return -1;
}

static int selector_exists(int fd, __u8 unit, __u8 selector)
{
    __u8 info = 0;
    if (uvc_control_query(fd, unit, selector, UVC_GET_INFO, &info, sizeof(info)) == 0) {
        if (info != 0) return 1;
    }
    __u16 len = 0;
    if (uvc_control_query(fd, unit, selector, UVC_GET_LEN, &len, sizeof(len)) == 0) {
        if (len > 0) return 1;
    }
    return 0;
}

UvcExtensionUnit::UvcExtensionUnit() = default;
UvcExtensionUnit::~UvcExtensionUnit() { close(); }

bool UvcExtensionUnit::open(const std::string& devicePath) {
    close();
    device_path_ = devicePath;
    fd_ = ::open(devicePath.c_str(), O_RDWR);
    if (fd_ < 0) {
        perror("open UVC device");
        return false;
    }

    // 检查必需的选择器是否存在
    if (!selector_exists(fd_, unitId_, 4) || !selector_exists(fd_, unitId_, 7)) {
        fprintf(stderr, "UVC extension unit %d missing selector 4 or 7\n", unitId_);
        close();
        return false;
    }
    return true;
}

bool UvcExtensionUnit::reopen() {
    if (isOpen()) return true;
    if (device_path_.empty()) return false;
    return open(device_path_);
}

void UvcExtensionUnit::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool UvcExtensionUnit::isOpen() const { return fd_ >= 0; }

bool UvcExtensionUnit::getActiveCamera(uint8_t& camId) const {
    if (!isOpen()) return false;
    uint8_t val = 0;
    if (uvc_control_query(fd_, unitId_, 7, UVC_GET_CUR, &val, sizeof(val)) == 0) {
        camId = val;
        return true;
    }
    return false;
}

bool UvcExtensionUnit::setActiveCamera(uint8_t camId) const {
    if (!isOpen()) return false;
    return uvc_control_query(fd_, unitId_, 7, UVC_SET_CUR, &camId, sizeof(camId)) == 0;
}

bool UvcExtensionUnit::readCurrentCameraParams(camera_params& params) const {
    if (!isOpen()) return false;
    memset(&params, 0, sizeof(params));
    return uvc_control_query(fd_, unitId_, 4, UVC_GET_CUR, &params, sizeof(params)) == 0;
}

bool UvcExtensionUnit::writeCurrentCameraParams(const camera_params& params) const {
    if (!isOpen()) return false;
    camera_params copy = params;
    return uvc_control_query(fd_, unitId_, 4, UVC_SET_CUR, &copy, sizeof(copy)) == 0;
}

bool UvcExtensionUnit::readCameraParams(uint8_t camId, camera_params& params) const {
    if (!isOpen()) return false;
    if (!setActiveCamera(camId)) return false;
    usleep(50000); // 等待切换稳定
    return readCurrentCameraParams(params);
}

bool UvcExtensionUnit::writeCameraParams(uint8_t camId, const camera_params& params) const {
    if (!isOpen()) return false;
    if (!setActiveCamera(camId)) return false;
    usleep(50000);
    return writeCurrentCameraParams(params);
}

void printParams(const camera_params& params) {
    std::printf(
        "[XU] cam=%u res=%u fps=%u exp_t=%.4f exp_g=%.4f bl=%u bright=%.4f contrast=%.4f "
        "gamma=%.4f hue=%.4f sat=%.4f sharp=%u awb=%u wb=%.4f dec=%u rot=%u\n",
        static_cast<unsigned>(params.cam_id),
        static_cast<unsigned>(params.resolution),
        static_cast<unsigned>(params.frame_rate),
        params.exposure_time,
        params.exposure_gain,
        static_cast<unsigned>(params.backlight_comp),
        params.brightness,
        params.contrast,
        params.gamma_dark,
        params.hue,
        params.saturation,
        static_cast<unsigned>(params.sharpness),
        static_cast<unsigned>(params.auto_white_balance),
        params.white_balance,
        static_cast<unsigned>(params.decimation),
        static_cast<unsigned>(params.rotation));
}

} // namespace viewer
