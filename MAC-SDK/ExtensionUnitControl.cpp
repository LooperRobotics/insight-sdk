#include "ExtensionUnitControl.hpp"
#include "UvcDevice.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#define XU_LOG_WARN(fmt, ...) fprintf(stderr, "[WARN] " fmt "\n", ##__VA_ARGS__)
#ifdef DEBUG
    #define XU_LOG_INFO(fmt, ...) printf("[INFO] " fmt "\n", ##__VA_ARGS__)
#else
    #define XU_LOG_INFO(fmt, ...) ((void)0)
#endif

namespace viewer {
namespace {

// Extension unit GUIDs as they appear in the descriptor (little-endian
// Data1..Data3, same byte order as a Windows GUID in memory).
constexpr uint8_t kExtGuid_RealSense1[16] = {
    0xcb, 0x6c, 0x60, 0xc9, 0x4c, 0x59, 0x25, 0x4d,
    0xaf, 0x47, 0xcc, 0xc4, 0x96, 0x43, 0x59, 0x95
};   // {C9606CCB-594C-4D25-AF47-CCC496435995}

constexpr uint8_t kExtGuid_RealSense3[16] = {
    0x6e, 0x41, 0xec, 0xb8, 0xac, 0xa3, 0x80, 0x45,
    0x8d, 0x5c, 0x0b, 0xee, 0x15, 0x97, 0xe4, 0x3d
};   // {B8EC416E-A3AC-4580-8D5C-0BEE1597E43D}

constexpr const uint8_t* kCandidateExtGuids[] = {
    kExtGuid_RealSense1,
    kExtGuid_RealSense3,
};

constexpr uint16_t kVendorId = 0x3652;
constexpr uint16_t kProductId = 0x0b5c;

constexpr uint8_t kUvcSetCur = 0x01;
constexpr uint8_t kUvcGetCur = 0x81;

// Same as Linux uvcvideo: the firmware answers control requests from user
// space and can take well over a second while it is (re)starting streams.
constexpr unsigned kControlTimeoutMs = 5000;

}  // namespace

ExtensionUnitControl::ExtensionUnitControl() = default;

ExtensionUnitControl::~ExtensionUnitControl() {
    close();
}

bool ExtensionUnitControl::open(const std::string& devicePath) {
    close();

    auto dev = insight9::UsbDevice::acquire(kVendorId, kProductId);
    if (!dev) {
        XU_LOG_WARN("[XU] Device not available for: %s", devicePath.c_str());
        return false;
    }
    const insight9::UvcFunction* fn = dev->findFunction(devicePath);
    if (!fn) {
        XU_LOG_WARN("[XU] UVC function not found: %s", devicePath.c_str());
        return false;
    }
    if (fn->extensionUnits.empty()) {
        XU_LOG_WARN("[XU] UVC function %s has no extension unit", devicePath.c_str());
        return false;
    }

    // Probe order: known GUIDs first, then the unit id the Linux SDK uses,
    // then whatever else the function exposes.
    candidates_.clear();
    auto add = [this](const insight9::UvcExtensionUnitDesc& xu) {
        for (const auto& c : candidates_) {
            if (c.first == xu.unitId) return;
        }
        std::array<uint8_t, 16> guid;
        std::memcpy(guid.data(), xu.guid, 16);
        candidates_.emplace_back(xu.unitId, guid);
    };
    for (const uint8_t* known : kCandidateExtGuids) {
        for (const auto& xu : fn->extensionUnits) {
            if (std::memcmp(xu.guid, known, 16) == 0) add(xu);
        }
    }
    for (const auto& xu : fn->extensionUnits) {
        if (xu.unitId == kXuUnitId) add(xu);
    }
    for (const auto& xu : fn->extensionUnits) add(xu);

    dev_ = dev;
    vcIfnum_ = fn->vcIfnum;
    if (!probeExtensionUnit()) {
        XU_LOG_WARN("[XU] No extension unit answered on %s", devicePath.c_str());
        close();
        return false;
    }

    XU_LOG_INFO("[XU] Bound extension unit on %s (vc_if=%u, unit=%u, guid=%s)",
           devicePath.c_str(), static_cast<unsigned>(vcIfnum_),
           static_cast<unsigned>(unitId_), getActiveGuidString().c_str());
    return true;
}

bool ExtensionUnitControl::probeExtensionUnit() {
    for (const auto& c : candidates_) {
        unitId_ = c.first;
        std::memcpy(activeExtGuidBytes_, c.second.data(), sizeof(activeExtGuidBytes_));

        uint8_t camId = 0;
        if (query(kActiveCameraSelector, true, &camId, sizeof(camId))) {
            return true;
        }

        camera_params params{};
        if (query(kCameraParamsSelector, true, &params, sizeof(params))) {
            return true;
        }
    }

    unitId_ = 0;
    std::memset(activeExtGuidBytes_, 0, sizeof(activeExtGuidBytes_));
    return false;
}

std::string ExtensionUnitControl::getActiveGuidString() const {
    const uint8_t* g = activeExtGuidBytes_;
    char buf[64];
    std::snprintf(buf, sizeof(buf),
        "{%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X}",
        g[3], g[2], g[1], g[0], g[5], g[4], g[7], g[6],
        g[8], g[9], g[10], g[11], g[12], g[13], g[14], g[15]);
    return std::string(buf);
}

void ExtensionUnitControl::close() {
    dev_.reset();
    vcIfnum_ = 0;
    unitId_ = 0;
    std::memset(activeExtGuidBytes_, 0, sizeof(activeExtGuidBytes_));
}

bool ExtensionUnitControl::isOpen() const {
    return dev_ != nullptr && dev_->alive() && unitId_ != 0;
}

bool ExtensionUnitControl::getActiveCamera(uint8_t& camId) const {
    camId = 0;
    return query(kActiveCameraSelector, true, &camId, sizeof(camId));
}

bool ExtensionUnitControl::setActiveCamera(uint8_t camId) const {
    return query(kActiveCameraSelector, false, &camId, sizeof(camId));
}

bool ExtensionUnitControl::readCurrentCameraParams(camera_params& params) const {
    std::memset(&params, 0, sizeof(params));
    return query(kCameraParamsSelector, true, &params, sizeof(params));
}

bool ExtensionUnitControl::writeCurrentCameraParams(const camera_params& params) const {
    camera_params copy = params;
    return query(kCameraParamsSelector, false, &copy, sizeof(copy));
}

bool ExtensionUnitControl::readCameraCalib(uint8_t camIdx, camera_calib& calib) const {
    if (!isOpen()) return false;
    if (camIdx >= kCalibCamCount) {
        fprintf(stderr, "[XU] invalid calib camera index %u (expected 0-%u)\n",
                static_cast<unsigned>(camIdx), kCalibCamCount - 1);
        return false;
    }
    const uint8_t selector = kCameraCalibSelectorBase + camIdx; // 0x14/0x15/0x16

    constexpr size_t kTotal = sizeof(camera_calib);
    constexpr uint8_t kBlocks =
        static_cast<uint8_t>((kTotal + kCalibChunkData - 1) / kCalibChunkData);

    uint8_t assembled[kTotal];
    uint8_t chunk[kCalibChunkSize];
    for (uint8_t blk = 0; blk < kBlocks; ++blk) {
        memset(chunk, 0, sizeof(chunk));
        chunk[0] = blk;
        // SET_CUR to select the block
        if (!query(selector, false, chunk, sizeof(chunk))) {
            fprintf(stderr, "[XU] SET_CUR calib block %u selector 0x%02x failed\n",
                    static_cast<unsigned>(blk), selector);
            return false;
        }
        // GET_CUR to read the block
        if (!query(selector, true, chunk, sizeof(chunk))) {
            fprintf(stderr, "[XU] GET_CUR calib block %u selector 0x%02x failed\n",
                    static_cast<unsigned>(blk), selector);
            return false;
        }
        if (chunk[0] != blk) {
            fprintf(stderr, "[XU] calib selector 0x%02x block out of sync: got %u, expected %u\n",
                    selector, static_cast<unsigned>(chunk[0]), static_cast<unsigned>(blk));
            return false;
        }
        if (chunk[1] != kBlocks) {
            fprintf(stderr, "[XU] calib selector 0x%02x total blocks mismatch: device=%u, sdk=%u\n",
                    selector, static_cast<unsigned>(chunk[1]), static_cast<unsigned>(kBlocks));
            return false;
        }
        const size_t off = static_cast<size_t>(blk) * kCalibChunkData;
        const size_t n = (kTotal - off < kCalibChunkData) ? (kTotal - off) : kCalibChunkData;
        memcpy(assembled + off, chunk + kCalibChunkHdr, n);
    }
    memcpy(&calib, assembled, sizeof(calib));
    // Ensure strings are terminated
    calib.intrinsics.frame_id[sizeof(calib.intrinsics.frame_id) - 1] = '\0';
    calib.intrinsics.distortion_model[sizeof(calib.intrinsics.distortion_model) - 1] = '\0';
    calib.extrinsics.parent_frame_id[sizeof(calib.extrinsics.parent_frame_id) - 1] = '\0';
    calib.extrinsics.child_frame_id[sizeof(calib.extrinsics.child_frame_id) - 1] = '\0';
    return true;
}

bool ExtensionUnitControl::readCameraParams(uint8_t camId, camera_params& params) const {
    if (!setActiveCamera(camId)) {
        XU_LOG_WARN("[XU] Failed to set active camera to %u for reading params", static_cast<unsigned>(camId));
        return false;
    }
    XU_LOG_INFO("[XU] Set active camera to %u for reading params", static_cast<unsigned>(camId));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return readCurrentCameraParams(params);
}

bool ExtensionUnitControl::writeCameraParams(uint8_t camId, const camera_params& params) const {
    if (!setActiveCamera(camId)) {
        return false;
    }
    XU_LOG_INFO("[XU] Set active camera to %u for writing params", static_cast<unsigned>(camId));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return writeCurrentCameraParams(params);
}

bool ExtensionUnitControl::readVioStatus(uint8_t& val) const {
    return query(kVioManagerStatus, true, &val, sizeof(val));
}

bool ExtensionUnitControl::readCurrentFps(uint8_t& fpsIndex) const {
    return query(kCurrentFpsSelector, true, &fpsIndex, sizeof(fpsIndex));
}

bool ExtensionUnitControl::query(uint8_t selector, bool get, void* data, uint16_t size) const {
    if (!dev_ || unitId_ == 0) {
        XU_LOG_WARN("[XU] query failed: extension unit is not bound");
        return false;
    }

    // UVC class request to the extension unit: wValue = selector,
    // wIndex = unit id (high byte) | VideoControl interface (low byte).
    const int r = dev_->controlTransfer(get ? 0xA1 : 0x21, get ? kUvcGetCur : kUvcSetCur,
                                        static_cast<uint16_t>(selector << 8),
                                        static_cast<uint16_t>((unitId_ << 8) | vcIfnum_),
                                        static_cast<uint8_t*>(data), size, kControlTimeoutMs);
    if (r != static_cast<int>(size)) {
        XU_LOG_WARN("[XU] %s failed: selector=%u unit=%u size=%u result=%s",
               get ? "GET_CUR" : "SET_CUR", static_cast<unsigned>(selector),
               static_cast<unsigned>(unitId_), static_cast<unsigned>(size),
               r < 0 ? libusb_error_name(r) : "short transfer");
        return false;
    }
    return true;
}

void printParams(const camera_params& params) {
    const int fps = params.frame_rate < (sizeof(kFramerateMap) / sizeof(kFramerateMap[0]))
        ? kFramerateMap[params.frame_rate]
        : -1;

    printf("[XU] cam_id=%u, resolution=%u, framerate=%u (%d fps), exposure_time=%.4f, gain=%.4f, auto_exposure=%u\n"
            "brightness=%.4f, contrast=%.4f, gamma=%.4f, hue=%.4f, saturation=%.4f, sharpness=%u, awb=%u\n"
            "white_balance=%.4f, filter=%u, Hardware_model=%u\n",
                static_cast<unsigned>(params.cam_id),
                static_cast<unsigned>(params.resolution),
                static_cast<unsigned>(params.frame_rate),
                fps,
                params.exposure_time,
                params.exposure_gain,
                static_cast<unsigned>(params.auto_exposure),
                params.brightness,
                params.contrast,
                params.gamma_dark,
                params.hue,
                params.saturation,
                static_cast<unsigned>(params.sharpness),
                static_cast<unsigned>(params.auto_white_balance),
                params.white_balance,
                static_cast<unsigned>(params.decimation),
                static_cast<unsigned>(params.hardware_model));
}

void printCalib(const camera_calib& calib) {
    const auto& in = calib.intrinsics;
    const auto& ex = calib.extrinsics;
    printf("[XU] intrinsics frame_id=%s %ux%u model=%s\n",
           in.frame_id, in.width, in.height, in.distortion_model);
    printf("[XU]   d=[%.6f %.6f %.6f %.6f]\n", in.d[0], in.d[1], in.d[2], in.d[3]);
    printf("[XU]   k=[%.4f %.4f %.4f; %.4f %.4f %.4f; %.4f %.4f %.4f]\n",
           in.k[0], in.k[1], in.k[2], in.k[3], in.k[4], in.k[5], in.k[6], in.k[7], in.k[8]);
    printf("[XU]   r=[%.4f %.4f %.4f; %.4f %.4f %.4f; %.4f %.4f %.4f]\n",
           in.r[0], in.r[1], in.r[2], in.r[3], in.r[4], in.r[5], in.r[6], in.r[7], in.r[8]);
    printf("[XU]   p=[%.4f %.4f %.4f %.4f; %.4f %.4f %.4f %.4f; %.4f %.4f %.4f %.4f]\n",
           in.p[0], in.p[1], in.p[2], in.p[3], in.p[4], in.p[5],
           in.p[6], in.p[7], in.p[8], in.p[9], in.p[10], in.p[11]);
    printf("[XU] extrinsics %s -> %s t=[%.6f %.6f %.6f] q(xyzw)=[%.6f %.6f %.6f %.6f]\n",
           ex.parent_frame_id, ex.child_frame_id,
           ex.translation[0], ex.translation[1], ex.translation[2],
           ex.rotation[0], ex.rotation[1], ex.rotation[2], ex.rotation[3]);
}

}  // namespace viewer
