#include "Insight9SDK.h"
#include <windows.h>
#include <dshow.h>
#include <ks.h>
#include <ksmedia.h>
#include <ksproxy.h>
#include <vidcap.h>
#include <hidapi.h>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <algorithm>
#include <cctype>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavdevice/avdevice.h>
}

#pragma comment(lib, "strmiids.lib")
#pragma comment(lib, "avcodec.lib")
#pragma comment(lib, "avformat.lib")
#pragma comment(lib, "avutil.lib")
#pragma comment(lib, "swscale.lib")
#pragma comment(lib, "ole32.lib")

#include "ExtensionUnitControl.hpp"

#define VENDOR_ID  0x1d6b
#define PRODUCT_ID 0x0104
#define CAM_NUM 3
#define HID_NUM 2

enum class PixelFormat {
    Unknown,
    MJPEG,
    GREY,
    Z16,
    RGB8,
    Y8I
};
#define MAIN_WIDTH   1088
#define MAIN_HEIGHT  1920
#define MAIN_FORMAT  PixelFormat::MJPEG
#define SUB_WIDTH    544
#define SUB_HEIGHT   1281
#define SUB_FORMAT   PixelFormat::GREY
#define DEPTH_WIDTH  544
#define DEPTH_HEIGHT 642
#define DEPTH_FORMAT PixelFormat::Z16

class FFmpegVideoSource;
class HidDevice;

struct sdk_ctx_t {
    FFmpegVideoSource* videos[CAM_NUM];
    HidDevice* hidDevs[HID_NUM];
    viewer::ExtensionUnitControl *xu;
    std::thread videoThreads[CAM_NUM];
    std::thread hidThreads[HID_NUM];
    std::atomic<bool> running;
    image_callback imgCb = nullptr;
    void* imgUser = nullptr;
    imu_callback imuCb = nullptr;
    void* imuUser = nullptr;
    vio_callback vioCb = nullptr;
    void* vioUser = nullptr;
    std::string videoPaths[CAM_NUM];
    std::string hidPaths[HID_NUM];
    std::mutex imgMutex;
    std::mutex imuMutex;
    std::mutex vioMutex;
    bool initialized = false;
    uint64_t last_img_timestamp[CAM_NUM] = {0};
} g_ctx;

std::string getDirectShowDeviceName(const std::string& devicePath) {
    ICreateDevEnum* pDevEnum = nullptr;
    IEnumMoniker* pEnum = nullptr;
    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, 
                                   IID_ICreateDevEnum, (void**)&pDevEnum);
    if (FAILED(hr)) return "";
    
    hr = pDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEnum, 0);
    if (FAILED(hr) || !pEnum) {
        pDevEnum->Release();
        return "";
    }
    
    IMoniker* pMoniker = nullptr;
    while (pEnum->Next(1, &pMoniker, NULL) == S_OK) {
        IPropertyBag* pPropBag = nullptr;
        hr = pMoniker->BindToStorage(0, 0, IID_IPropertyBag, (void**)&pPropBag);
        if (SUCCEEDED(hr)) {
            VARIANT var;
            VariantInit(&var);
            
            hr = pPropBag->Read(L"DevicePath", &var, 0);
            if (SUCCEEDED(hr) && var.vt == VT_BSTR) {
                std::wstring wpath = var.bstrVal;
                std::string path(wpath.begin(), wpath.end());
                
                if (path == devicePath) {
                    VariantClear(&var);
                    VariantInit(&var);
                    hr = pPropBag->Read(L"FriendlyName", &var, 0);
                    if (SUCCEEDED(hr) && var.vt == VT_BSTR) {
                        std::wstring wname = var.bstrVal;
                        std::string name(wname.begin(), wname.end());
                        VariantClear(&var);
                        pPropBag->Release();
                        pMoniker->Release();
                        pEnum->Release();
                        pDevEnum->Release();
                        return name;
                    }
                }
            }
            VariantClear(&var);
            pPropBag->Release();
        }
        pMoniker->Release();
    }
    
    pEnum->Release();
    pDevEnum->Release();
    return "";
}

static std::vector<std::string> findUvcDevices(uint16_t vid, uint16_t pid) {
    std::vector<std::string> paths;
    HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    ICreateDevEnum* pDevEnum = nullptr;
    IEnumMoniker* pEnum = nullptr;
    hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, IID_ICreateDevEnum, (void**)&pDevEnum);
    if (FAILED(hr)) return paths;
    hr = pDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEnum, 0);
    if (SUCCEEDED(hr) && pEnum) {
        IMoniker* pMoniker = nullptr;
        while (pEnum->Next(1, &pMoniker, NULL) == S_OK) {
            IPropertyBag* pPropBag = nullptr;
            hr = pMoniker->BindToStorage(0, 0, IID_IPropertyBag, (void**)&pPropBag);
            if (SUCCEEDED(hr)) {
                VARIANT var;
                VariantInit(&var);
                hr = pPropBag->Read(L"DevicePath", &var, 0);
                if (SUCCEEDED(hr) && var.vt == VT_BSTR) {
                    std::wstring wpath = var.bstrVal;
                    std::string path(wpath.begin(), wpath.end());

                    printf("[DEBUG] Found UVC device path: %s\n", path.c_str());

                    char vidStr[16], pidStr[16];
                    snprintf(vidStr, sizeof(vidStr), "vid_%04x", vid);
                    snprintf(pidStr, sizeof(pidStr), "pid_%04x", pid);
                    std::string lower = path;
                    for (char& c : lower) c = tolower(c);
                    
                    if (lower.find(vidStr) != std::string::npos && 
                        lower.find(pidStr) != std::string::npos) {
                        if (lower.find("mi_04") != std::string::npos ||  // RGB
                            lower.find("mi_06") != std::string::npos ||  // GREY
                            lower.find("mi_08") != std::string::npos) {  // Depth
                            paths.push_back(path);
                        }
                    }
                }
                VariantClear(&var);
                pPropBag->Release();
            }
            pMoniker->Release();
        }
        pEnum->Release();
    }
    pDevEnum->Release();
    if (SUCCEEDED(hr)) CoUninitialize();
    std::sort(paths.begin(), paths.end());
    return paths;
}

static std::vector<std::string> findHidDevices(uint16_t vid, uint16_t pid, int interfaceNum) {
    std::vector<std::string> paths;
    struct hid_device_info *devs, *cur;
    devs = hid_enumerate(vid, pid);
    for (cur = devs; cur; cur = cur->next) {
        if (cur->interface_number == interfaceNum) {
            paths.push_back(cur->path);
        }
    }
    hid_free_enumeration(devs);
    return paths;
}

class FFmpegVideoSource {
public:
    bool open(const std::string& devicePath, int deviceIndex, int width, int height, PixelFormat fmt, int fps) {
        close();
        devicePath_ = devicePath;
        deviceIndex_ = deviceIndex;
        width_ = width;
        height_ = height;
        format_ = fmt;
        fps_ = fps;
        return true;
    }
    void close() {
        if (fmtCtx_) avformat_close_input(&fmtCtx_);
        if (packet_) av_packet_free(&packet_);
        fmtCtx_ = nullptr;
        packet_ = nullptr;
        running_ = false;
    }
    bool start() {
        if (fmtCtx_) return true;
        avdevice_register_all();
        
        if (devicePath_.empty()) {
            fprintf(stderr, "[FFmpegVideoSource] Device path is empty\n");
            return false;
        }
        
        std::string deviceName = getDirectShowDeviceName(devicePath_);
        if (deviceName.empty()) {
            fprintf(stderr, "[FFmpegVideoSource] Cannot find DirectShow device name for path: %s\n", devicePath_.c_str());
            return false;
        }
        
        std::string inputName = "video=" + deviceName;
        fprintf(stderr, "[FFmpegVideoSource] Opening device: %s\n", inputName.c_str());
        
        AVDictionary* opts = nullptr;
        char sizeStr[32];
        char fpsStr[16];
        snprintf(sizeStr, sizeof(sizeStr), "%dx%d", width_, height_);
        snprintf(fpsStr, sizeof(fpsStr), "%d", fps_);
        av_dict_set(&opts, "video_size", sizeStr, 0);
        av_dict_set(&opts, "framerate", fpsStr, 0);
        av_dict_set(&opts, "rtbufsize", "50M", 0);
        
        const AVInputFormat* ifmt = av_find_input_format("dshow");
        fmtCtx_ = avformat_alloc_context();
        runningPtr_ = &running_;
        fmtCtx_->interrupt_callback.callback = interruptCallback;
        fmtCtx_->interrupt_callback.opaque = this;
        
        int ret = avformat_open_input(&fmtCtx_, inputName.c_str(), ifmt, &opts);
        av_dict_free(&opts);
        
        if (ret < 0) {
            char errbuf[256];
            av_strerror(ret, errbuf, sizeof(errbuf));
            fprintf(stderr, "[FFmpegVideoSource] Failed to open %s: %s\n", inputName.c_str(), errbuf);
            return false;
        }
        
        ret = avformat_find_stream_info(fmtCtx_, nullptr);
        if (ret < 0) {
            fprintf(stderr, "[FFmpegVideoSource] Failed to find stream info\n");
            return false;
        }
        
        for (unsigned i = 0; i < fmtCtx_->nb_streams; ++i) {
            if (fmtCtx_->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
                videoIdx_ = i;
                break;
            }
        }
        
        if (videoIdx_ == -1) {
            fprintf(stderr, "[FFmpegVideoSource] No video stream found\n");
            return false;
        }
        
        packet_ = av_packet_alloc();
        running_ = true;
        fprintf(stderr, "[FFmpegVideoSource] Started successfully\n");
        return true;
    }
    void stop() {
        running_ = false;
        if (fmtCtx_) avformat_close_input(&fmtCtx_);
        fmtCtx_ = nullptr;
        if (packet_) av_packet_free(&packet_);
        packet_ = nullptr;
    }
    bool pollFrame(uint8_t*& data, size_t& size, int& width, int& height, PixelFormat& fmt, uint64_t& ts, uint64_t& tsRight) {
        if (!fmtCtx_ || !running_) return false;
        int ret = av_read_frame(fmtCtx_, packet_);
        if (ret < 0) return false;
        if (packet_->stream_index != videoIdx_) {
            av_packet_unref(packet_);
            return false;
        }

        uint8_t* copy = new uint8_t[packet_->size];
        memcpy(copy, packet_->data, packet_->size);
        data = copy;
        size = packet_->size;
        width = width_;
        height = height_;
        fmt = format_;

        ts = 0;
        tsRight = 0;
        if (format_ == PixelFormat::MJPEG) {
            const uint8_t* mjpegData = data;
            const size_t mjpegSize = size;
            size_t pos = 0;
            while (pos + 4 < mjpegSize) {
                if (mjpegData[pos] == 0xFF && mjpegData[pos + 1] == 0xE1) {
                    const uint16_t len = (static_cast<uint16_t>(mjpegData[pos + 2]) << 8) | static_cast<uint16_t>(mjpegData[pos + 3]);
                    if (len >= 12 && pos + 4 + len <= mjpegSize &&
                        mjpegData[pos + 4] == 'T' && mjpegData[pos + 5] == 'S' &&
                        mjpegData[pos + 6] == '_' && mjpegData[pos + 7] == '_') {
                        memcpy(&ts, mjpegData + pos + 8, sizeof(uint64_t));
                    }
                    break;
                }
                ++pos;
            }
        } else if (format_ == PixelFormat::GREY && size >= static_cast<size_t>(width_) * (height_ - 1) + 8) {
            memcpy(&ts, data + width_ * (height_ - 1), 8);
        } else if (format_ == PixelFormat::Z16 && size >= static_cast<size_t>(width_) * (height_ - 2) * 2 + 8) {
            memcpy(&ts, data + width_ * (height_ - 2) * 2, 8);
        }

        if (format_ == PixelFormat::GREY && size >= static_cast<size_t>(width_) * (height_ - 1) + 16) {
            memcpy(&tsRight, data + width_ * (height_ - 1) + 8, 8);
        }

        av_packet_unref(packet_);
        return true;
    }
private:
    std::string devicePath_;
    int deviceIndex_ = -1;
    int width_ = 0, height_ = 0;
    PixelFormat format_ = PixelFormat::Unknown;
    int fps_ = 30;
    AVFormatContext* fmtCtx_ = nullptr;
    AVPacket* packet_ = nullptr;
    int videoIdx_ = -1;
    std::atomic<bool> running_{false};
    static int interruptCallback(void* ctx);
    std::atomic<bool>* runningPtr_ = nullptr;
};

class HidDevice {
public:
    bool open(const std::string& path) {
        close();
        dev_ = hid_open_path(path.c_str());
        if (dev_) hid_set_nonblocking(dev_, 1);
        return dev_ != nullptr;
    }
    void close() {
        if (dev_) hid_close(dev_);
        dev_ = nullptr;
    }
    bool read(uint8_t* buf, size_t& len) {
        if (!dev_) return false;
        int ret = hid_read(dev_, buf, (size_t)len);
        if (ret > 0) {
            len = ret;
            return true;
        }
        return false;
    }
private:
    hid_device* dev_ = nullptr;
};

int FFmpegVideoSource::interruptCallback(void* ctx) {
    FFmpegVideoSource* self = static_cast<FFmpegVideoSource*>(ctx);
    // 如果 running_ 被设为 false，返回 1 表示中断操作
    return (self->runningPtr_ && !(*self->runningPtr_)) ? 1 : 0;
}

static void videoThreadFunc(int camId) {
    auto* src = g_ctx.videos[camId];
    if (!src) return;

    uint64_t& last_ts = g_ctx.last_img_timestamp[camId];

    while (g_ctx.running) {
        uint8_t* data = nullptr;
        size_t size = 0;
        int w = 0, h = 0;
        PixelFormat fmt = PixelFormat::Unknown;
        uint64_t ts = 0, tsRight = 0;

        if (!src->pollFrame(data, size, w, h, fmt, ts, tsRight)) {
            if (!g_ctx.running) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        bool shouldCallCallback = true;
        if (data == nullptr || size == 0) shouldCallCallback = false;
        else if (ts == 0) shouldCallCallback = false;
        else if (ts == last_ts) shouldCallCallback = false;
        else last_ts = ts;

        uint8_t* callbackData = data;
        size_t callbackSize = size;
        bool customAllocated = false;

        if (shouldCallCallback && fmt == PixelFormat::MJPEG && size >= 4) {
            if (data[0] == 0xFF && data[1] == 0xD8 && size > 4) {
                size_t pos = 2;
                while (pos + 1 < size) {
                    if (data[pos] == 0xFF && data[pos+1] == 0xE1) {
                        if (pos + 3 < size) {
                            uint16_t segLen = (data[pos+2] << 8) | data[pos+3];
                            size_t appTotal = 2 + segLen;
                            if (pos + appTotal <= size) {
                                uint8_t* remaining = data + pos + appTotal;
                                size_t remainingSize = size - (pos + appTotal);
                                uint8_t* newBuf = new uint8_t[2 + remainingSize];
                                newBuf[0] = 0xFF;
                                newBuf[1] = 0xD8;
                                memcpy(newBuf + 2, remaining, remainingSize);
                                callbackData = newBuf;
                                callbackSize = 2 + remainingSize;
                                customAllocated = true;
                                break;
                            }
                        }
                        break;
                    }
                    pos++;
                }
            }
        }

        if (shouldCallCallback && g_ctx.imgCb) {
            unsigned int v4l2Fmt = 0;
            if (fmt == PixelFormat::MJPEG) v4l2Fmt = 0x47504A4D;
            else if (fmt == PixelFormat::GREY) v4l2Fmt = 0x59455247;
            else if (fmt == PixelFormat::Z16) v4l2Fmt = 0x36315A;
            else if (fmt == PixelFormat::Y8I) v4l2Fmt = 0x49385956;
            g_ctx.imgCb(camId, callbackData, callbackSize, w, h, v4l2Fmt, ts, tsRight, g_ctx.imgUser);
        }

        if (customAllocated) {
            delete[] callbackData;
        }
        delete[] data;
    }
}

static void hidThreadFunc(int idx) {
    auto* dev = g_ctx.hidDevs[idx];
    if (!dev) return;
    uint8_t buf[64];
    while (g_ctx.running) {
        size_t len = sizeof(buf);
        if (dev->read(buf, len)) {
            if (idx == 0 && g_ctx.imuCb) {
                if (len >= 32) {
                    float ax, ay, az, gx, gy, gz;
                    uint64_t ts;
                    memcpy(&ax, buf, 4);
                    memcpy(&ay, buf+4, 4);
                    memcpy(&az, buf+8, 4);
                    memcpy(&gx, buf+12, 4);
                    memcpy(&gy, buf+16, 4);
                    memcpy(&gz, buf+20, 4);
                    memcpy(&ts, buf+24, 8);
                    g_ctx.imuCb(ax, ay, az, gx, gy, gz, ts, g_ctx.imuUser);
                }
            } else if (idx == 1 && g_ctx.vioCb) {
                if (len >= 32) {
                    uint64_t ts; float px, py, pz, qx, qy, qz, qw;
                    memcpy(&ts, buf, 8);
                    memcpy(&px, buf+8, 4);
                    memcpy(&py, buf+12, 4);
                    memcpy(&pz, buf+16, 4);
                    memcpy(&qx, buf+20, 4);
                    memcpy(&qy, buf+24, 4);
                    memcpy(&qz, buf+28, 4);
                    memcpy(&qw, buf+32, 4);
                    g_ctx.vioCb(px, py, pz, qx, qy, qz, qw, ts, g_ctx.vioUser);
                }
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

int insight9_receive_init() {
    if (g_ctx.initialized) return -1;

    HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    if (FAILED(hr) && hr != S_FALSE) {
        fprintf(stderr, "[SDK] CoInitializeEx failed, hr=0x%08lx\n", hr);
    }

    memset(&g_ctx, 0, sizeof(g_ctx));
    avdevice_register_all();
    hid_init();

    auto uvcPaths = findUvcDevices(VENDOR_ID, PRODUCT_ID);
    if (uvcPaths.size() < 3) {
        fprintf(stderr, "[SDK] Need at least 3 UVC devices, found %zu\n", uvcPaths.size());
        return -1;
    }
    for (int i = 0; i < CAM_NUM; ++i) {
        g_ctx.videoPaths[i] = uvcPaths[i];
        g_ctx.videos[i] = new FFmpegVideoSource();
        int w = (i==0)? MAIN_WIDTH : (i==1)? SUB_WIDTH : DEPTH_WIDTH;
        int h = (i==0)? MAIN_HEIGHT : (i==1)? SUB_HEIGHT : DEPTH_HEIGHT;
        PixelFormat fmt = (i==0)? MAIN_FORMAT : (i==1)? SUB_FORMAT : DEPTH_FORMAT;
        if (!g_ctx.videos[i]->open(uvcPaths[i], i, w, h, fmt, 30)) {
            fprintf(stderr, "[SDK] Failed to open video device %d: %s\n", i, uvcPaths[i].c_str());
            return -1;
        }
    }
    auto imuPaths = findHidDevices(VENDOR_ID, PRODUCT_ID, 0);
    auto vioPaths = findHidDevices(VENDOR_ID, PRODUCT_ID, 1);
    if (imuPaths.empty() || vioPaths.empty()) {
        fprintf(stderr, "[SDK] IMU or VIO device not found\n");
        return -1;
    }
    g_ctx.hidPaths[0] = imuPaths[0];
    g_ctx.hidPaths[1] = vioPaths[0];
    g_ctx.hidDevs[0] = new HidDevice();
    g_ctx.hidDevs[1] = new HidDevice();
    if (!g_ctx.hidDevs[0]->open(g_ctx.hidPaths[0])) {
        fprintf(stderr, "[SDK] Failed to open IMU HID\n");
        return -1;
    }
    if (!g_ctx.hidDevs[1]->open(g_ctx.hidPaths[1])) {
        fprintf(stderr, "[SDK] Failed to open VIO HID\n");
        return -1;
    }

    g_ctx.xu = new viewer::ExtensionUnitControl();
    if (!g_ctx.xu->open(g_ctx.videoPaths[0])) {
        fprintf(stderr, "[SDK] Failed to open XU control\n");
        delete g_ctx.xu; g_ctx.xu = nullptr;
    }

    g_ctx.initialized = true;
    return 0;
}

int insight9_receive_start() {
    if (!g_ctx.initialized || g_ctx.running) return -1;
    g_ctx.running = true;
    for (int i = 0; i < CAM_NUM; ++i) {
        if (g_ctx.videos[i]) {
            g_ctx.videos[i]->start();
            g_ctx.videoThreads[i] = std::thread(videoThreadFunc, i);
        }
    }
    for (int i = 0; i < HID_NUM; ++i) {
        g_ctx.hidThreads[i] = std::thread(hidThreadFunc, i);
    }
    return 0;
}

void insight9_receive_stop() {
    if (!g_ctx.running) return;
    g_ctx.running = false;
     for (int i = 0; i < CAM_NUM; ++i) {
        if (g_ctx.videos[i]) g_ctx.videos[i]->stop();
    }
    for (int i = 0; i < CAM_NUM; ++i) {
        if (g_ctx.videoThreads[i].joinable()) g_ctx.videoThreads[i].join();
    }
    for (int i = 0; i < HID_NUM; ++i) {
        if (g_ctx.hidThreads[i].joinable()) g_ctx.hidThreads[i].join();
    }
}

void insight9_receive_cleanup() {
    insight9_receive_stop();
    for (int i = 0; i < CAM_NUM; ++i) {
        delete g_ctx.videos[i]; g_ctx.videos[i] = nullptr;
    }
    for (int i = 0; i < HID_NUM; ++i) {
        if (g_ctx.hidDevs[i]) g_ctx.hidDevs[i]->close();
        delete g_ctx.hidDevs[i];
    }
    delete g_ctx.xu;
    g_ctx.initialized = false;
    hid_exit();

    CoUninitialize();
}

const char* insight9_receive_get_video_dev(int cam_id) {
    return (cam_id >=0 && cam_id < CAM_NUM) ? g_ctx.videoPaths[cam_id].c_str() : nullptr;
}
const char* insight9_receive_get_metadata_dev(int) { return nullptr; }
int insight9_receive_read_metadata_timestamp(int, uint64_t*) { return -1; }
void insight9_receive_register_image_callback(image_callback cb, void *user) { g_ctx.imgCb = cb; g_ctx.imgUser = user; }
void insight9_receive_register_imu_callback(imu_callback cb, void *user) { g_ctx.imuCb = cb; g_ctx.imuUser = user; }
void insight9_receive_register_vio_callback(vio_callback cb, void *user) { g_ctx.vioCb = cb; g_ctx.vioUser = user; }

int insight9_receive_set_active_camera(int cam_id) {
    if (!g_ctx.xu) return -1;
    return g_ctx.xu->setActiveCamera((uint8_t)cam_id) ? 0 : -1;
}
int insight9_receive_get_active_camera(int *cam_id) {
    if (!g_ctx.xu || !cam_id) return -1;
    uint8_t val;
    if (!g_ctx.xu->getActiveCamera(val)) return -1;
    *cam_id = val;
    return 0;
}
int insight9_receive_set_camera_params(const camera_params *params) {
    if (!g_ctx.xu || !params) return -1;
    viewer::camera_params xuParams;
    memcpy(&xuParams, params, sizeof(viewer::camera_params));
    return g_ctx.xu->writeCurrentCameraParams(xuParams) ? 0 : -1;
}
int insight9_receive_get_camera_params(camera_params *params) {
    if (!g_ctx.xu || !params) return -1;
    viewer::camera_params xuParams;
    if (!g_ctx.xu->readCurrentCameraParams(xuParams)) return -1;
    memcpy(params, &xuParams, sizeof(camera_params));
    return 0;
}
int insight9_receive_set_camera_params_for(int cam_id, const camera_params *params) {
    if (!g_ctx.xu || !params) return -1;
    viewer::camera_params xuParams;
    memcpy(&xuParams, params, sizeof(viewer::camera_params));
    return g_ctx.xu->writeCameraParams((uint8_t)cam_id, xuParams) ? 0 : -1;
}
int insight9_receive_get_camera_params_for(int cam_id, camera_params *params) {
    if (!g_ctx.xu || !params) return -1;
    viewer::camera_params xuParams;
    if (!g_ctx.xu->readCameraParams((uint8_t)cam_id, xuParams)) return -1;
    memcpy(params, &xuParams, sizeof(camera_params));
    return 0;
}
int insight9_receive_reset_camera_params(int) { return -1; }
void insight9_receive_print_camera_params(const camera_params *p) {}
const char* insight9_receive_get_hardware_type() {
    static std::string result;
    camera_params params;
    params.hardware_model = 0xFF;
    if (insight9_receive_get_camera_params(&params) == 0) {
        const char* models[] = {"Insight 9", "Insight 7", "Insight 7p", "Insight 3u"};
        if (params.hardware_model < 4) {
            result = models[params.hardware_model];
            return result.c_str();
        }
    }
    return "unknown";
}
