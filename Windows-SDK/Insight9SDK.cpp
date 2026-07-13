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
#include <regex>
#include <map>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <vector>
#include <cfloat>

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

// ==================== Target Device VID/PID ====================
#define VENDOR_ID  0x1d6b
#define PRODUCT_ID 0x0104

// ==================== Camera Configuration ====================
#define CAM_NUM 3
#define HID_NUM 2
#define MAIN_WIDTH   1088
#define MAIN_HEIGHT  1920
#define MAIN_FORMAT  PixelFormat::MJPEG
#define SUB_WIDTH    544
#define SUB_HEIGHT   1281
#define SUB_FORMAT   PixelFormat::GREY
#define DEPTH_WIDTH  544
#define DEPTH_HEIGHT 642
#define DEPTH_FORMAT PixelFormat::Z16

// ==================== Reconnect Backoff / Retry Limits ====================
// Rationale: repeatedly opening / S_FMT-ing a device that is in a bad state
// (e.g. a composite UVC+NCM gadget that is warm-restarting) generates a USB
// transaction storm that can wedge the whole xhci controller. We therefore
// back off exponentially between reconnect attempts and cap the number of
// consecutive failures so a dead device cannot keep hammering the bus forever.
#define RECONNECT_BACKOFF_BASE_MS 1000   // first retry delay
#define RECONNECT_BACKOFF_MAX_MS  30000  // delay ceiling (exponential, capped)
// Max consecutive failed reconnect attempts before a thread stops actively
// reconnecting and idles at the ceiling interval (set to 0 to retry forever).
// After the limit is hit the thread no longer generates fast USB traffic, but
// still wakes periodically so a recovered device can be picked back up.
#define RECONNECT_MAX_ATTEMPTS    30

static bool g_com_initialized_by_sdk = false;

#ifndef NOMINMAX
#define NOMINMAX
#endif

class FFmpegVideoSource;
class HidDevice;

struct sdk_ctx_t {
    insight9_config_t config;
    FFmpegVideoSource* videos[CAM_NUM];
    HidDevice* hidDevs[HID_NUM];
    viewer::ExtensionUnitControl *xu;
    std::thread videoThreads[CAM_NUM];
    std::thread hidThreads[HID_NUM];
    std::atomic<bool> running;
    std::atomic<bool> cam_running[CAM_NUM];
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

                    char vidStr[16], pidStr[16];
                    snprintf(vidStr, sizeof(vidStr), "vid_%04x", vid);
                    snprintf(pidStr, sizeof(pidStr), "pid_%04x", pid);
                    std::string lower = path;
                    for (char& c : lower) c = tolower(c);
                    
                    if (lower.find(vidStr) != std::string::npos && 
                        lower.find(pidStr) != std::string::npos) {
                        
                        std::regex mi_pattern(R"(mi_(\d+))", std::regex::icase);
                        std::smatch match;
                        if (std::regex_search(lower, match, mi_pattern)) {
                            paths.push_back(path);
                        }
                        else {
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
    std::sort(paths.begin(), paths.end(), [](const std::string& a, const std::string& b) {
        std::regex pattern(R"(mi_(\d+))", std::regex::icase);
        std::smatch match_a, match_b;
        
        int num_a = 999, num_b = 999;
        if (std::regex_search(a, match_a, pattern)) {
            num_a = std::stoi(match_a[1].str());
        }
        if (std::regex_search(b, match_b, pattern)) {
            num_b = std::stoi(match_b[1].str());
        }
        return num_a < num_b;
    });
    
    return paths;
}

static std::vector<std::string> findHidDevices(uint16_t vid, uint16_t pid) {
    std::vector<std::string> paths;
    struct hid_device_info *devs, *cur;
    devs = hid_enumerate(vid, pid);
    
    std::map<int, std::string> interfaceMap;
    for (cur = devs; cur; cur = cur->next) {
        if (cur->interface_number >= 0) {
            printf("[HID] Interface %d: %s\n", cur->interface_number, cur->path);
            interfaceMap[cur->interface_number] = cur->path;
        }
    }
    hid_free_enumeration(devs);
    
    for (std::map<int, std::string>::iterator it = interfaceMap.begin(); 
         it != interfaceMap.end(); ++it) {
        paths.push_back(it->second);
    }
    
    return paths;
}

void refreshVideoDevicePath(int camId) {
    auto uvcPaths = findUvcDevices(VENDOR_ID, PRODUCT_ID);
    if (camId < (int)uvcPaths.size()) {
        g_ctx.videoPaths[camId] = uvcPaths[camId];
        printf("[SDK] Camera %d device path refreshed: %s\n", camId, uvcPaths[camId].c_str());
    }
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
            // fprintf(stderr, "[FFmpegVideoSource] Cannot find DirectShow device name for path: %s\n", devicePath_.c_str());
            return false;
        }
        
        std::string inputName = "video=" + deviceName;
        fprintf(stderr, "[FFmpegVideoSource] Opening device: %s\n", inputName.c_str());
        
        AVDictionary* opts = nullptr;
        char sizeStr[32];
        char fpsStr[16];
        snprintf(sizeStr, sizeof(sizeStr), "%dx%d", width_, height_);
        snprintf(fpsStr, sizeof(fpsStr), "%d", fps_);
        printf("[FFmpegVideoSource] Video option %dx%d %dfps\n", width_, height_, fps_);
        av_dict_set(&opts, "video_size", sizeStr, 0);
        av_dict_set(&opts, "framerate", fpsStr, 0);
        av_dict_set(&opts, "rtbufsize", "100M", 0);
        
        const AVInputFormat* ifmt = av_find_input_format("dshow");
        fmtCtx_ = avformat_alloc_context();
        runningPtr_ = &running_;
        fmtCtx_->interrupt_callback.callback = interruptCallback;
        fmtCtx_->interrupt_callback.opaque = this;
        if (deviceIndex_ >= 0)
        {
            char idx[8];
            sprintf(idx,"%d",deviceIndex_);
            av_dict_set(&opts,"video_device_number",idx,0);
        }
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
        if (ret < 0) {
            if (ret == AVERROR(EAGAIN)) {
                return false;
            }
            return false;
        }
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
    return (self->runningPtr_ && !(*self->runningPtr_)) ? 1 : 0;
}

static void videoThreadFunc(int camId) {
    auto* src = g_ctx.videos[camId];
    if (!src) return;

    uint64_t& last_ts = g_ctx.last_img_timestamp[camId];
    
    bool need_reconnect = false;
    auto last_reconnect_attempt = std::chrono::steady_clock::now();
    auto last_success_time = std::chrono::steady_clock::now();
    bool first_frame_received = false;

    while (g_ctx.cam_running[camId]) {
        uint8_t* data = nullptr;
        size_t size = 0;
        int w = 0, h = 0;
        PixelFormat fmt = PixelFormat::Unknown;
        uint64_t ts = 0, tsRight = 0;

        if (!src->pollFrame(data, size, w, h, fmt, ts, tsRight)) {
            if (!g_ctx.cam_running[camId]) break;
            
            auto now = std::chrono::steady_clock::now();
            if (first_frame_received && (now - last_success_time > std::chrono::seconds(3))) {
                need_reconnect = true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if (!first_frame_received) {
            first_frame_received = true;
            printf("[SDK] Camera %d received first frame\n", camId);
        }
        last_success_time = std::chrono::steady_clock::now();

        uint8_t* callbackData = data;
        size_t callbackSize = size;
        bool customAllocated = false;
        uint64_t finalTs = ts;
        uint64_t finalTsRight = tsRight;

        if (fmt == PixelFormat::MJPEG && size >= 4 && data[0] == 0xFF && data[1] == 0xD8) {
            size_t pos = 2;
            bool foundApp1 = false;
            
            while (pos + 3 < size) {
                if (data[pos] == 0xFF && data[pos+1] == 0xE1) {
                    uint16_t segLen = (data[pos+2] << 8) | data[pos+3];
                    size_t appTotal = 2 + segLen;
                    
                    if (pos + appTotal <= size) {
                        if (appTotal >= 2 + 4 + sizeof(uint64_t) &&
                            data[pos + 4] == 'T' && data[pos + 5] == 'S' &&
                            data[pos + 6] == '_' && data[pos + 7] == '_') {
                            
                            size_t remainingSize = size - (pos + appTotal);
                            uint8_t* newBuf = new uint8_t[2 + remainingSize];
                            newBuf[0] = 0xFF;
                            newBuf[1] = 0xD8;
                            if (remainingSize > 0) {
                                memcpy(newBuf + 2, data + pos + appTotal, remainingSize);
                            }
                            
                            callbackData = newBuf;
                            callbackSize = 2 + remainingSize;
                            customAllocated = true;
                            foundApp1 = true;
                            
                            break;
                        }
                    }
                    pos += appTotal;
                } else {
                    pos++;
                }
            }
        }

        bool shouldCallCallback = true;
        if (callbackData == nullptr || callbackSize == 0) shouldCallCallback = false;
        else if (finalTs == 0) shouldCallCallback = false;
        else if (finalTs == last_ts) shouldCallCallback = false;
        else last_ts = finalTs;

        if (shouldCallCallback && g_ctx.imgCb) {
            unsigned int v4l2Fmt = 0;
            if (fmt == PixelFormat::MJPEG) v4l2Fmt = 0x47504A4D;
            else if (fmt == PixelFormat::GREY) v4l2Fmt = 0x59455247;
            else if (fmt == PixelFormat::Z16) v4l2Fmt = 0x36315A;
            else if (fmt == PixelFormat::Y8I) v4l2Fmt = 0x49385956;
            
            g_ctx.imgCb(camId, callbackData, callbackSize, w, h, v4l2Fmt, 
                       finalTs, finalTsRight, g_ctx.imgUser);
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

int insight9_receive_init(const insight9_config_t* config) {
    if (g_ctx.initialized) {
        fprintf(stderr, "[SDK] Already initialized\n");
        return -1;
    }

    if (!config) {
        fprintf(stderr, "[SDK] Config is NULL, using default\n");
        return insight9_receive_init_default();
    }

    if (config->rgb_config.width <= 0 || config->rgb_config.height <= 0 ||
        config->gray_config.width <= 0 || config->gray_config.height <= 0 ||
        config->depth_config.width <= 0 || config->depth_config.height <= 0) {
        fprintf(stderr, "[SDK] Invalid resolution in config\n");
        return -1;
    }

    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.running = false;
    for (int i = 0; i < CAM_NUM; ++i) {
        g_ctx.cam_running[i] = false;
    }
    g_ctx.config = *config;

    avdevice_register_all();
    hid_init();

    auto uvcPaths = findUvcDevices(VENDOR_ID, PRODUCT_ID);
    if (uvcPaths.size() < 3) {
        fprintf(stderr, "[SDK] Need at least 3 UVC devices, found %zu\n", uvcPaths.size());
        return -1;
    }
    
    HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    if (FAILED(hr) && hr != S_FALSE) {
        fprintf(stderr, "[SDK] CoInitializeEx failed, hr=0x%08lx\n", hr);
        CoUninitialize();
        return -1;
    }
    g_com_initialized_by_sdk = SUCCEEDED(hr) || hr == S_FALSE;
    fprintf(stderr, "[SDK] COM initialized by SDK (hr=0x%08lx)\n", hr);

    for (int i = 0; i < CAM_NUM; ++i) {
        g_ctx.videoPaths[i] = uvcPaths[i];
        g_ctx.videos[i] = new FFmpegVideoSource();
        int w = (i==0)? config->rgb_config.width : (i==1)? config->gray_config.width : config->depth_config.width;
        int h = (i==0)? config->rgb_config.height : (i==1)? config->gray_config.height : config->depth_config.height;
        PixelFormat fmt = (i==0)? config->rgb_config.pixel_format : (i==1)? config->gray_config.pixel_format : config->depth_config.pixel_format;
        int fps = (i==0)? config->rgb_config.fps : (i==1)? config->gray_config.fps : config->depth_config.fps;
        if (!g_ctx.videos[i]->open(uvcPaths[i], i, w, h, fmt, fps)) {
            fprintf(stderr, "[SDK] Failed to open video device %d: %s\n", i, uvcPaths[i].c_str());
            return -1;
        }
    }
    auto hidPaths = findHidDevices(VENDOR_ID, PRODUCT_ID);
    if (hidPaths.size() < 2) {
        fprintf(stderr, "[SDK] IMU or VIO device not found (found %zu HID devices)\n", hidPaths.size());
        return -1;
    }
    g_ctx.hidPaths[0] = hidPaths[0];
    g_ctx.hidPaths[1] = hidPaths[1];
    printf("[HID] IMU device (interface %d): %s\n", 0, g_ctx.hidPaths[0].c_str());
    printf("[HID] VIO device (interface %d): %s\n", 1, g_ctx.hidPaths[1].c_str());
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
    printf("[SDK] Initialized successfully with config:\n");
    printf("  RGB: %dx%d@%d, fourcc=0x%08x\n", 
           config->rgb_config.width, config->rgb_config.height,
           config->rgb_config.fps, config->rgb_config.pixel_format);
    printf("  Gray: %dx%d@%d, fourcc=0x%08x\n",
           config->gray_config.width, config->gray_config.height,
           config->gray_config.fps, config->gray_config.pixel_format);
    printf("  Depth: %dx%d@%d, fourcc=0x%08x\n",
           config->depth_config.width, config->depth_config.height,
           config->depth_config.fps, config->depth_config.pixel_format);
    
    return 0;
}

int insight9_receive_init_default() {
    if (g_ctx.initialized) {
        fprintf(stderr, "[SDK] Already initialized\n");
        return -1;
    }

    HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    if (FAILED(hr) && hr != S_FALSE) {
        fprintf(stderr, "[SDK] CoInitializeEx failed, hr=0x%08lx\n", hr);
        return -1;
    }
    g_com_initialized_by_sdk = SUCCEEDED(hr) || hr == S_FALSE;
    fprintf(stderr, "[SDK] COM initialized by SDK (hr=0x%08lx)\n", hr);

    memset(&g_ctx, 0, sizeof(g_ctx));
    for (int i = 0; i < CAM_NUM; ++i) {
        g_ctx.cam_running[i] = false;
    }

    g_ctx.config.rgb_config.width = MAIN_WIDTH;
    g_ctx.config.rgb_config.height = MAIN_HEIGHT;
    g_ctx.config.rgb_config.fps = 30;
    g_ctx.config.rgb_config.pixel_format = MAIN_FORMAT;
    
    g_ctx.config.gray_config.width = SUB_WIDTH;
    g_ctx.config.gray_config.height = SUB_HEIGHT;
    g_ctx.config.gray_config.fps = 20;
    g_ctx.config.gray_config.pixel_format = SUB_FORMAT;
    
    g_ctx.config.depth_config.width = DEPTH_WIDTH;
    g_ctx.config.depth_config.height = DEPTH_HEIGHT;
    g_ctx.config.depth_config.fps = 15;
    g_ctx.config.depth_config.pixel_format = DEPTH_FORMAT;

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
    auto hidPaths = findHidDevices(VENDOR_ID, PRODUCT_ID);
    if (hidPaths.size() < 2) {
        fprintf(stderr, "[SDK] IMU or VIO device not found (found %zu HID devices)\n", hidPaths.size());
        return -1;
    }
    g_ctx.hidPaths[0] = hidPaths[0];
    g_ctx.hidPaths[1] = hidPaths[1];
    printf("[HID] IMU device (interface %d): %s\n", 0, g_ctx.hidPaths[0].c_str());
    printf("[HID] VIO device (interface %d): %s\n", 1, g_ctx.hidPaths[1].c_str());
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
        insight9_receive_start_camera(i);
    }
    
    for (int i = 0; i < HID_NUM; ++i) {
        g_ctx.hidThreads[i] = std::thread(hidThreadFunc, i);
    }
    return 0;
}

const char* insight9_receive_get_video_dev(int cam_id) {
    return (cam_id >=0 && cam_id < CAM_NUM) ? g_ctx.videoPaths[cam_id].c_str() : nullptr;
}

int insight9_receive_start_camera(int cam_id) {
    if (!g_ctx.initialized) return -1;
    if (cam_id < 0 || cam_id >= CAM_NUM) return -1;
    if (g_ctx.cam_running[cam_id]) return 0;
    
    if (!g_ctx.videos[cam_id]) {
        auto uvcPaths = findUvcDevices(VENDOR_ID, PRODUCT_ID);
        if (cam_id >= (int)uvcPaths.size()) return -1;
        
        g_ctx.videoPaths[cam_id] = uvcPaths[cam_id];
        g_ctx.videos[cam_id] = new FFmpegVideoSource();
        
        int w = (cam_id==0)? g_ctx.config.rgb_config.width : 
                (cam_id==1)? g_ctx.config.gray_config.width : 
                            g_ctx.config.depth_config.width;
        int h = (cam_id==0)? g_ctx.config.rgb_config.height : 
                (cam_id==1)? g_ctx.config.gray_config.height : 
                            g_ctx.config.depth_config.height;
        PixelFormat fmt = (cam_id==0)? g_ctx.config.rgb_config.pixel_format : 
                           (cam_id==1)? g_ctx.config.gray_config.pixel_format : 
                                        g_ctx.config.depth_config.pixel_format;
        int fps = (cam_id==0)? g_ctx.config.rgb_config.fps : 
                   (cam_id==1)? g_ctx.config.gray_config.fps : 
                                g_ctx.config.depth_config.fps;
        
        if (!g_ctx.videos[cam_id]->open(uvcPaths[cam_id], cam_id, w, h, fmt, fps)) {
            delete g_ctx.videos[cam_id];
            g_ctx.videos[cam_id] = nullptr;
            return -1;
        }
    }
    
    g_ctx.cam_running[cam_id] = true;
    g_ctx.videos[cam_id]->start();
    
    if (g_ctx.videoThreads[cam_id].joinable()) {
        g_ctx.videoThreads[cam_id].join();
    }
    g_ctx.videoThreads[cam_id] = std::thread(videoThreadFunc, cam_id);
    
    return 0;
}

void insight9_receive_stop_camera(int cam_id) {
    if (cam_id < 0 || cam_id >= CAM_NUM) return;
    
    g_ctx.cam_running[cam_id] = false;
    
    if (g_ctx.videos[cam_id]) {
        g_ctx.videos[cam_id]->stop();
    }
    
    if (g_ctx.videoThreads[cam_id].joinable()) {
        g_ctx.videoThreads[cam_id].join();
    }
}

int insight9_receive_restart_camera(int cam_id) {
    if (!g_ctx.initialized) return -1;
    if (cam_id < 0 || cam_id >= CAM_NUM) return -1;
    
    insight9_receive_stop_camera(cam_id);
    
    delete g_ctx.videos[cam_id];
    g_ctx.videos[cam_id] = nullptr;
    
    auto uvcPaths = findUvcDevices(VENDOR_ID, PRODUCT_ID);
    if (cam_id >= (int)uvcPaths.size()) {
        fprintf(stderr, "[SDK] Camera %d not found during restart\n", cam_id);
        return -1;
    }

    int w = (cam_id==0)? g_ctx.config.rgb_config.width : 
            (cam_id==1)? g_ctx.config.gray_config.width : 
                        g_ctx.config.depth_config.width;
    int h = (cam_id==0)? g_ctx.config.rgb_config.height : 
            (cam_id==1)? g_ctx.config.gray_config.height : 
                        g_ctx.config.depth_config.height;
    PixelFormat fmt = (cam_id==0)? g_ctx.config.rgb_config.pixel_format : 
                       (cam_id==1)? g_ctx.config.gray_config.pixel_format : 
                                    g_ctx.config.depth_config.pixel_format;
    int fps = (cam_id==0)? g_ctx.config.rgb_config.fps : 
               (cam_id==1)? g_ctx.config.gray_config.fps : 
                            g_ctx.config.depth_config.fps;
    
    g_ctx.videos[cam_id] = new FFmpegVideoSource();
    if (!g_ctx.videos[cam_id]->open(uvcPaths[cam_id], cam_id, w, h, fmt, fps)) {
        delete g_ctx.videos[cam_id];
        g_ctx.videos[cam_id] = nullptr;
        return -1;
    }
    
    return insight9_receive_start_camera(cam_id);
}

int insight9_receive_is_camera_running(int cam_id) {
    if (cam_id < 0 || cam_id >= CAM_NUM) return 0;
    return g_ctx.cam_running[cam_id] ? 1 : 0;
}

void insight9_receive_stop() {
    if (!g_ctx.running) return;
    g_ctx.running = false;
    
    for (int i = 0; i < CAM_NUM; ++i) {
        g_ctx.cam_running[i] = false;
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

    fprintf(stderr, "[SDK] Cleaning up resources...\n");
    for (int i = 0; i < CAM_NUM; ++i) {
        if (g_ctx.videos[i]) {
            g_ctx.videos[i]->stop();
            delete g_ctx.videos[i];
            g_ctx.videos[i] = nullptr;
        }
    }
    for (int i = 0; i < HID_NUM; ++i) {
        if (g_ctx.hidDevs[i]) {
            g_ctx.hidDevs[i]->close();
            delete g_ctx.hidDevs[i];
            g_ctx.hidDevs[i] = nullptr;
        }
    }
    if (g_ctx.xu) {
        delete g_ctx.xu;
        g_ctx.xu = nullptr;
    }
    
    g_ctx.initialized = false;
    g_ctx.running = false;
    for (int i = 0; i < CAM_NUM; ++i) {
        g_ctx.cam_running[i] = false;
    }
    
    hid_exit();
    
    if (g_com_initialized_by_sdk) {
        fprintf(stderr, "[SDK] Uninitializing COM...\n");
        CoUninitialize();
        g_com_initialized_by_sdk = false;
    }
    
    fprintf(stderr, "[SDK] Cleanup complete\n");
}

int insight9_receive_set_camera_fps(int cam_id, int fps) {
    if (!g_ctx.initialized || cam_id < 0 || cam_id >= CAM_NUM) return -1;
    
    if (cam_id == 0) {
        g_ctx.config.rgb_config.fps = fps;
        printf("[SDK] Set RGB FPS to %d\n", fps);
    } else if (cam_id == 1) {
        g_ctx.config.gray_config.fps = fps;
        printf("[SDK] Set Gray FPS to %d\n", fps);
    } else if (cam_id == 2) {
        g_ctx.config.depth_config.fps = fps;
        printf("[SDK] Set Depth FPS to %d\n", fps);
    }
    return 0;
}

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


int insight9_receive_reset_camera_params(int) { 
   // Reading factory defaults from the device requires additional implementation. A simple approach is to read
   // and save the current values during initialization, then write them back when a reset is needed.
   fprintf(stderr, "[XU][ERR] reset_camera_params not implemented, use set_camera_params with saved defaults\n");
   return -1;
}

void insight9_receive_print_camera_params(const camera_params *params) {
    if (!params) return;
    viewer::camera_params xu_params;
    memcpy(&xu_params, params, sizeof(viewer::camera_params));
    viewer::printParams(xu_params);
}

int insight9_receive_get_camera_calib(int cam_idx, camera_calib *calib) {
    if (!g_ctx.xu || !calib) return -1;
    if (cam_idx < 0 || cam_idx >= viewer::kCalibCamCount) return -1;
    
    viewer::camera_calib xu_calib;
    if (!g_ctx.xu->readCameraCalib(static_cast<uint8_t>(cam_idx), xu_calib)) {
        return -1;
    }
    memcpy(calib, &xu_calib, sizeof(camera_calib));
    return 0;
}

void insight9_receive_print_camera_calib(const camera_calib *calib) {
    if (!calib) return;
    viewer::camera_calib xu_calib;
    memcpy(&xu_calib, calib, sizeof(viewer::camera_calib));
    viewer::printCalib(xu_calib);
}

int insight9_receive_get_current_fps(int* fps) {
    if (!g_ctx.xu || !fps) return -1;
    uint8_t val;
    if (!g_ctx.xu->readCurrentFps(val)) return -1;
    const int validFps[] = {0, 20, 30, 40, 50};
    if (val >= 0 && val < (int)(sizeof(validFps)/sizeof(validFps[0]))) {
        *fps = validFps[val];
    } else {
        *fps = 0;
    }
    return 0;
}

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
