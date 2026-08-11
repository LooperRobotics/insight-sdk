#pragma once

#include <cstdint>
#include <string>

namespace viewer {

#pragma pack(push, 1)
struct camera_params {
    uint8_t cam_id;             // Camera ID, 0/1
    uint8_t resolution;         // Resolution index
    uint8_t frame_rate;         // Frame-rate index, 0-5
    float exposure_time;        // Exposure time in seconds, range 0.0~0.03
    float exposure_gain;        // Exposure gain, range 1.0~16.0
    uint8_t auto_exposure;      // Auto exposure, 0/1
    float brightness;           // Brightness, range 0.0~127.0
    float contrast;             // Contrast, range 0.0~1.9
    float gamma_dark;           // Dark gamma, range 1.0~4.0
    float hue;                  // Hue, range 0.0~87.0
    float saturation;           // Saturation, range 0.0~1.999
    uint8_t sharpness;          // Sharpness (1~255)
    uint8_t auto_white_balance; // Auto white balance, 0 or 1
    float white_balance;        // White balance, range 1.0~3.0
    uint8_t decimation;         // Decimation (1~255)
    uint8_t hardware_model;     // Hardware model
};

// Camera intrinsics payload, fields aligned with ROS sensor_msgs/CameraInfo.
// Must match intrinsics_hid_payload_tt on the device side (uvc_gadget.h).
struct camera_intrinsics {
    uint32_t sec;
    uint32_t nsec;

    char frame_id[32];

    uint32_t height;
    uint32_t width;

    char distortion_model[32];

    float d[4];
    float k[9];
    float r[9];
    float p[12];

    uint32_t binning_x;
    uint32_t binning_y;

    uint32_t roi_x_offset;
    uint32_t roi_y_offset;
    uint32_t roi_height;
    uint32_t roi_width;

    uint8_t roi_do_rectify;
};

// Camera extrinsics payload, one transform from /tf_static.
// Must match extrinsics_hid_payload_tt on the device side.
struct camera_extrinsics {
    char parent_frame_id[32];   // transform.header.frame_id (reference frame)
    char child_frame_id[32];    // transform.child_frame_id  (this camera frame)

    double translation[3];      // x, y, z (m)
    double rotation[4];         // quaternion x, y, z, w
};

// Full calibration of one camera: intrinsics + extrinsics, returned by a
// single GET_CUR on selector 0x14/0x15/0x16.
struct camera_calib {
    camera_intrinsics intrinsics;
    camera_extrinsics extrinsics;
};
#pragma pack(pop)

// Camera index for readCameraCalib(), mapped to selectors 0x14/0x15/0x16.
enum : uint8_t {
    kCalibCamLeft  = 0,   // 0x14 left grayscale
    kCalibCamRight = 1,   // 0x15 right grayscale
    kCalibCamRgb   = 2,   // 0x16 RGB
    kCalibCamCount = 3,
};

inline constexpr int kFramerateMap[] = {90, 60, 30, 20, 15, 10};
inline constexpr uint8_t kXuUnitId = 3;
inline constexpr uint8_t kCameraParamsSelector = 4;
inline constexpr uint8_t kActiveCameraSelector = 7;
inline constexpr uint8_t kCameraCalibSelectorBase = 0x14;
inline constexpr uint8_t kCurrentFpsSelector = 0x17;
inline constexpr uint8_t kVioManagerStatus = 0x18;

// The UVC gadget can only return 60 bytes per control request, so the
// calibration payload is transferred in chunks:
//   GET_CUR -> [0]=block index, [1]=total blocks, [2..57]=payload data
//   SET_CUR with byte 0 = block index selects the block to read next.
inline constexpr uint16_t kCalibChunkHdr = 2;
inline constexpr uint16_t kCalibChunkData = 56;
inline constexpr uint16_t kCalibChunkSize = kCalibChunkHdr + kCalibChunkData; // 58

class UvcExtensionUnit {
public:
    UvcExtensionUnit();
    ~UvcExtensionUnit();

    UvcExtensionUnit(const UvcExtensionUnit&) = delete;
    UvcExtensionUnit& operator=(const UvcExtensionUnit&) = delete;

    bool open(const std::string& devicePath);
    void close();
    bool isOpen() const;
    bool reopen();

    bool getActiveCamera(uint8_t& camId) const;
    bool setActiveCamera(uint8_t camId) const;
    bool readCurrentCameraParams(camera_params& params) const;
    bool writeCurrentCameraParams(const camera_params& params) const;
    bool readCameraParams(uint8_t camId, camera_params& params) const;
    bool writeCameraParams(uint8_t camId, const camera_params& params) const;
    bool readCurrentFps(uint8_t& fpsIndex) const;
    bool readCameraCalib(uint8_t camIdx, camera_calib& calib) const;
    bool readVioStatus(uint8_t& status) const;

private:
    int fd_ = -1;
    uint8_t unitId_ = kXuUnitId;
    std::string device_path_;
};

void printParams(const camera_params& params);
void printCalib(const camera_calib& calib);

}  // namespace viewer