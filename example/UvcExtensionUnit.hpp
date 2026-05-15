#pragma once

#include <cstdint>
#include <string>

namespace viewer {

#pragma pack(push, 1)
struct camera_params {
    uint8_t cam_id;         // 0 rgb,1 bw
    uint8_t resolution;     // rgb:0:"3840x2160", 1:"1088x1920", 2:"1280x720", 3:"640x480"; bw:0:"1088x1280", 1:"544x640"
    uint8_t frame_rate;     // 0: 90fps, 1: 60fps, 2: 30fps, 3: 20fps, 4: 15fps, 5: 10fps
    float exposure_time;  // 0.1~10000 ms, mapped to (0, 0.1)
    float exposure_gain;  //1~10000, mapped to [1, 16]
    uint8_t backlight_comp; //0~128, mapped to 1~255
    float brightness;     //-64~64, mapped to [-127, 127]
    float contrast;       //0~100, mapped to [0, 1.999]
    float gamma_dark;     //100~500, mapped to [1, 4]
    float hue;            //-180~180, mapped to [-90, 90]
    float saturation;     //0~100, mapped to [0, 1.999]
    uint8_t sharpness;      //0~100, mapped to 1~255
    uint8_t auto_white_balance; //0 or 1
    float white_balance;  //2800~6500, mapped to 1~255
    uint8_t decimation;     //1~8, mapped to 1~255
    uint8_t rotation;       //-90~180, mapped to 1~255
};
#pragma pack(pop)

inline constexpr int kFramerateMap[] = {90, 60, 30, 20, 15, 10};
inline constexpr uint8_t kXuUnitId = 4;
inline constexpr uint8_t kCameraParamsSelector = 4;
inline constexpr uint8_t kActiveCameraSelector = 7;

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
