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

// Wrap uvc_control_query, which must come from kernel headers or a local implementation.
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

    // Check whether the required selectors exist.
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

bool UvcExtensionUnit::readCameraCalib(uint8_t camIdx, camera_calib& calib) const {
    if (!isOpen()) return false;
    if (camIdx >= kCalibCamCount) {
        fprintf(stderr, "[XU] invalid calib camera index %u (expected 0-%u)\n",
                static_cast<unsigned>(camIdx), kCalibCamCount - 1);
        return false;
    }
    const __u8 selector = kCameraCalibSelectorBase + camIdx; // 0x14/0x15/0x16

    // Older firmware does not expose the calibration selectors; verify the
    // reported length matches the chunk protocol so a mismatch fails loudly.
    __u16 len = 0;
    if (get_selector_len(fd_, unitId_, selector, &len) != 0) {
        fprintf(stderr, "[XU] calib selector 0x%02x not supported by device\n", selector);
        return false;
    }
    if (len != kCalibChunkSize) {
        fprintf(stderr, "[XU] calib selector 0x%02x length mismatch: device=%u, sdk chunk=%u\n",
                selector, static_cast<unsigned>(len), static_cast<unsigned>(kCalibChunkSize));
        return false;
    }

    constexpr size_t kTotal = sizeof(camera_calib);
    constexpr uint8_t kBlocks =
        static_cast<uint8_t>((kTotal + kCalibChunkData - 1) / kCalibChunkData);

    uint8_t assembled[kTotal];
    uint8_t chunk[kCalibChunkSize];
    for (uint8_t blk = 0; blk < kBlocks; ++blk) {
        // Select the block explicitly instead of relying on the device-side
        // auto-increment, so concurrent readers cannot desynchronize us.
        memset(chunk, 0, sizeof(chunk));
        chunk[0] = blk;
        if (uvc_control_query(fd_, unitId_, selector, UVC_SET_CUR, chunk, sizeof(chunk)) != 0) {
            fprintf(stderr, "[XU] SET_CUR calib block %u selector 0x%02x failed: %s\n",
                    static_cast<unsigned>(blk), selector, strerror(errno));
            return false;
        }
        if (uvc_control_query(fd_, unitId_, selector, UVC_GET_CUR, chunk, sizeof(chunk)) != 0) {
            fprintf(stderr, "[XU] GET_CUR calib block %u selector 0x%02x failed: %s\n",
                    static_cast<unsigned>(blk), selector, strerror(errno));
            return false;
        }
        if (chunk[0] != blk) {
            fprintf(stderr, "[XU] calib selector 0x%02x block out of sync: got %u, expected %u\n",
                    selector, static_cast<unsigned>(chunk[0]), static_cast<unsigned>(blk));
            return false;
        }
        if (chunk[1] != kBlocks) {
            fprintf(stderr, "[XU] calib selector 0x%02x total blocks mismatch: device=%u, sdk=%u"
                    " (payload layout differs)\n",
                    selector, static_cast<unsigned>(chunk[1]), static_cast<unsigned>(kBlocks));
            return false;
        }
        const size_t off = static_cast<size_t>(blk) * kCalibChunkData;
        const size_t n = (kTotal - off < kCalibChunkData) ? (kTotal - off) : kCalibChunkData;
        memcpy(assembled + off, chunk + kCalibChunkHdr, n);
    }
    memcpy(&calib, assembled, sizeof(calib));
    // Ensure the strings coming from the device are terminated.
    calib.intrinsics.frame_id[sizeof(calib.intrinsics.frame_id) - 1] = '\0';
    calib.intrinsics.distortion_model[sizeof(calib.intrinsics.distortion_model) - 1] = '\0';
    calib.extrinsics.parent_frame_id[sizeof(calib.extrinsics.parent_frame_id) - 1] = '\0';
    calib.extrinsics.child_frame_id[sizeof(calib.extrinsics.child_frame_id) - 1] = '\0';
    return true;
}

bool UvcExtensionUnit::readCameraParams(uint8_t camId, camera_params& params) const {
    if (!isOpen()) return false;
    if (!setActiveCamera(camId)) return false;
    usleep(50000); // Wait for the switch to stabilize.
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

void printCalib(const camera_calib& calib) {
    const camera_intrinsics& in = calib.intrinsics;
    const camera_extrinsics& ex = calib.extrinsics;
    std::printf("[XU] intrinsics frame_id=%s %ux%u model=%s\n",
                in.frame_id, in.width, in.height, in.distortion_model);
    std::printf("[XU]   d=[%.6f %.6f %.6f %.6f]\n", in.d[0], in.d[1], in.d[2], in.d[3]);
    std::printf("[XU]   k=[%.4f %.4f %.4f; %.4f %.4f %.4f; %.4f %.4f %.4f]\n",
                in.k[0], in.k[1], in.k[2], in.k[3], in.k[4], in.k[5], in.k[6], in.k[7], in.k[8]);
    std::printf("[XU]   r=[%.4f %.4f %.4f; %.4f %.4f %.4f; %.4f %.4f %.4f]\n",
                in.r[0], in.r[1], in.r[2], in.r[3], in.r[4], in.r[5], in.r[6], in.r[7], in.r[8]);
    std::printf("[XU]   p=[%.4f %.4f %.4f %.4f; %.4f %.4f %.4f %.4f; %.4f %.4f %.4f %.4f]\n",
                in.p[0], in.p[1], in.p[2], in.p[3], in.p[4], in.p[5],
                in.p[6], in.p[7], in.p[8], in.p[9], in.p[10], in.p[11]);
    std::printf("[XU] extrinsics %s -> %s t=[%.6f %.6f %.6f] q(xyzw)=[%.6f %.6f %.6f %.6f]\n",
                ex.parent_frame_id, ex.child_frame_id,
                ex.translation[0], ex.translation[1], ex.translation[2],
                ex.rotation[0], ex.rotation[1], ex.rotation[2], ex.rotation[3]);
}

} // namespace viewer
