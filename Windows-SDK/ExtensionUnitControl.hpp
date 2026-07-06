#pragma once

#include <cstdint>
#include <memory>
#include <string>

struct IBaseFilter;
struct IKsControl;

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

typedef struct {
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
} intrinsics_hid_payload_tt;

typedef struct {
    char parent_frame_id[32];
    char child_frame_id[32];
    double translation[3];
    double rotation[4];
} extrinsics_hid_payload_tt;

typedef struct {
    intrinsics_hid_payload_tt intrinsics;
    extrinsics_hid_payload_tt extrinsics;
} camera_calib_hid_payload_tt;
#pragma pack(pop)

inline constexpr int kFramerateMap[] = {90, 60, 30, 20, 15, 10};
inline constexpr uint8_t kXuUnitId = 3;
inline constexpr uint8_t kCameraParamsSelector = 4;
inline constexpr uint8_t kActiveCameraSelector = 7;
inline constexpr uint8_t kCurrentFpsSelector = 0x17;

class ExtensionUnitControl {
public:
    ExtensionUnitControl();
    ~ExtensionUnitControl();

    ExtensionUnitControl(const ExtensionUnitControl&) = delete;
    ExtensionUnitControl& operator=(const ExtensionUnitControl&) = delete;

    bool open(const std::string& devicePath);
    void close();
    bool isOpen() const;

    bool getActiveCamera(uint8_t& camId) const;
    bool setActiveCamera(uint8_t camId) const;
    bool readCurrentCameraParams(camera_params& params) const;
    bool writeCurrentCameraParams(const camera_params& params) const;
    bool readCameraParams(uint8_t camId, camera_params& params) const;
    bool writeCameraParams(uint8_t camId, const camera_params& params) const;
    bool readCurrentFps(uint8_t& fpsIndex) const;

private:
    bool bindFilterByDevicePath(const std::string& devicePath);
    bool resolveNodeId();
    bool query(uint8_t selector, unsigned long flags, void* data, unsigned long size) const;

    IBaseFilter* filter_ = nullptr;
    IKsControl* ksControl_ = nullptr;
    unsigned long nodeId_ = 0;
};

void printParams(const camera_params& params);

}  // namespace viewer
