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
#pragma pack(pop)

inline constexpr int kFramerateMap[] = {90, 60, 30, 20, 15, 10};
inline constexpr uint8_t kXuUnitId = 4;
inline constexpr uint8_t kCameraParamsSelector = 4;
inline constexpr uint8_t kActiveCameraSelector = 7;
inline constexpr uint8_t kIntrinsicsSelector = 0x14;
inline constexpr uint8_t kExtrinsicSelector = 0x15;

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

private:
    int fd_ = -1;
    uint8_t unitId_ = kXuUnitId;
    std::string device_path_;
};

void printParams(const camera_params& params);

}  // namespace viewer