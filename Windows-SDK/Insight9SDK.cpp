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
#include <functional>
#include <new>
#include <cctype>
#include <cmath>
#include <vector>
#include <cfloat>
#include <dshow.h>
#include <sensorsapi.h>
#include <sensors.h>
#include <propvarutil.h>

// ==================== Media Foundation ====================
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <mferror.h>
#include <propsys.h>
#include <initguid.h>

#pragma comment(lib, "strmiids.lib")
#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "oleaut32.lib")
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mfreadwrite.lib")
#pragma comment(lib, "mfuuid.lib")
#pragma comment(lib, "propsys.lib")

// ==================== Logging Macros ====================
// Always print errors and warnings
#define SDK_LOG_ERROR(fmt, ...)   fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__)
#define SDK_LOG_WARN(fmt, ...)    fprintf(stderr, "[WARN] " fmt "\n", ##__VA_ARGS__)

// Info and debug are controlled by DEBUG macro
#ifdef DEBUG
    #define SDK_LOG_INFO(fmt, ...)  printf("[INFO] " fmt "\n", ##__VA_ARGS__)
    #define SDK_LOG_DEBUG(fmt, ...) printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)
#else
    #define SDK_LOG_INFO(fmt, ...)  ((void)0)
    #define SDK_LOG_DEBUG(fmt, ...) ((void)0)
#endif

// ==================== Define The Missing Media Foundation GUID ====================
extern "C" {
// Y800 - 8-bit grayscale
DEFINE_GUID(MEDIASUBTYPE_Y800, 
    0x30303859, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);

// Z16 - 16-bit depth
DEFINE_GUID(MEDIASUBTYPE_Z16, 
    0x2036315A, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);

DEFINE_GUID(MEDIASUBTYPE_YUY2, 
    0x32595559, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);

DEFINE_GUID(MEDIASUBTYPE_MJPG, 
    0x47504A4D, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);

DEFINE_GUID(MEDIASUBTYPE_RGB24, 
    0x42475200, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);
}
// Y8I - RealSense Interleaved Infrared (FOURCC: 'Y8I ')
DEFINE_GUID(MEDIASUBTYPE_Y8I, 
    0x20493859, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);

#define MFVideoFormat_Y800 MEDIASUBTYPE_Y800
#define MFVideoFormat_Z16 MEDIASUBTYPE_Z16
#define MFVideoFormat_MJPG MEDIASUBTYPE_MJPG
#define MFVideoFormat_YUY2 MEDIASUBTYPE_YUY2
#define MFVideoFormat_RGB24 MEDIASUBTYPE_RGB24
#define MFVideoFormat_Y8I MEDIASUBTYPE_Y8I

#include "ExtensionUnitControl.hpp"

// ==================== Metadata extraction GUIDs ====================
// These are not always publicly defined in older SDKs
#define RS_META_CAPTURE_TIMING_OFF 12

DEFINE_GUID(MFSampleExtension_CaptureMetadata,
    0x2EBE23A8, 0xFAF5, 0x444A, 0xA6, 0xA2, 0xEB, 0x81, 0x08, 0x80, 0xAB, 0x5D);

DEFINE_GUID(MF_CAPTURE_METADATA_FRAME_RAWSTREAM,
    0x26CBE9DE, 0x5BC3, 0x4EDC, 0xB9, 0x94, 0x0F, 0x54, 0x4D, 0x74, 0x49, 0x3D);

static const GUID VENDOR_META_ITEM0 = // {9252077B-2680-49B9-AE02-B19075973B70}
    { 0x9252077B, 0x2680, 0x49B9, { 0xAE, 0x02, 0xB1, 0x90, 0x75, 0x97, 0x3B, 0x70 } };
static const GUID VENDOR_META_ITEM1 = // {F9F88A87-E1DD-441E-95CB-42E21A64F1D9}
    { 0xF9F88A87, 0xE1DD, 0x441E, { 0x95, 0xCB, 0x42, 0xE2, 0x1A, 0x64, 0xF1, 0xD9 } };

// ==================== Sensor GUIDs ====================
static const PROPERTYKEY SENSOR_DATA_TYPE_DEV_TS_LOW = {
    { 0xB14C764F, 0x07CF, 0x41E8, { 0x9D, 0x82, 0xEB, 0xE3, 0xD0, 0x77, 0x6A, 0x6F } },
    7
};
static const PROPERTYKEY SENSOR_DATA_TYPE_DEV_TS_HIGH = {
    { 0xB14C764F, 0x07CF, 0x41E8, { 0x9D, 0x82, 0xEB, 0xE3, 0xD0, 0x77, 0x6A, 0x6F } },
    8
};

// ==================== UVC and MS metadata structures ====================
#pragma pack(push, 1)
struct KsItemHeader {
    uint32_t MetadataId;
    uint32_t Size;
};
struct KsUvcTimestamp {
    uint32_t PresentationTimeStamp;
    uint32_t SourceClockReference;
    uint16_t SofCounter;
    uint16_t Reserved;
};
struct MsMetadataHeader {
    KsItemHeader   header;
    KsUvcTimestamp startOfFrame;
    KsUvcTimestamp endOfFrame;
};
#pragma pack(pop)

#define RECONNECT_BACKOFF_BASE_MS 1000
#define RECONNECT_BACKOFF_MAX_MS  30000
#define RECONNECT_MAX_ATTEMPTS    30

static constexpr size_t MS_HEADER_SIZE = 40;
static constexpr size_t UVC_HEADER_SIZE = 12;

static constexpr uint32_t RS_META_CAPTURE_TIMING_ID = 0x80000001u;

static constexpr double ACCEL_SCALE_FACTOR   = -0.366459184;
static constexpr double GYRO_SCALE_FACTOR    = -610.464183381;
static constexpr double ACCEL_SCALE_LOOPERHUB = 0.0035913;
static constexpr double GYRO_SCALE_LOOPERHUB  = 0.00106526;

static std::mutex g_imuStateMutex;
static float g_lastAx = 0, g_lastAy = 0, g_lastAz = 0;
static float g_lastGx = 0, g_lastGy = 0, g_lastGz = 0;
static uint64_t g_lastAccelTs = 0, g_lastGyroTs = 0;
static bool g_accelDirty = false;
static bool g_gyroDirty = false;

// ==================== HID Report Structures ====================
#pragma pack(push, 1)
struct imu_accel_report_t {
    uint8_t  report_id;
    uint8_t  sensor_state;
    uint64_t timestamp;
    int32_t  accel_x, accel_y, accel_z;
    int32_t  custom_data1, custom_data2;
    int16_t  custom_data3, custom_data4, custom_data5;
    int8_t   custom_data6, custom_data7;
};
struct imu_gyro_report_t {
    uint8_t  report_id;
    uint8_t  sensor_state;
    uint64_t timestamp;
    int32_t  gyro_x, gyro_y, gyro_z;
    int32_t  custom_data1, custom_data2;
    int16_t  custom_data3, custom_data4, custom_data5;
    int8_t   custom_data6, custom_data7;
};
struct vio_hid_payload {
    uint64_t timestamp;
    float px, py, pz;
    float qx, qy, qz, qw;
    uint8_t seq;
    uint8_t reserved[3];
};
#pragma pack(pop)

// ==================== Target Device VID/PID ====================
#define VENDOR_ID  0x3652
#define PRODUCT_ID 0x0b5c

// ==================== Camera Configuration ====================
#define UVC_NUM 2
#define LOGICAL_CAM_NUM 3
#define HID_NUM 2

#define RGB_CAM_ID    0
#define GRAY_CAM_ID   1
#define DEPTH_CAM_ID  2

#define RGB_UVC_ID       0
#define COMPOSITE_UVC_ID 1

#define DEPTH_STREAM_ID  0
#define GRAY_STREAM_ID   1

#define MAIN_WIDTH   1088
#define MAIN_HEIGHT  1920
#define MAIN_FORMAT  PixelFormat::YUYV
#define SUB_WIDTH    544
#define SUB_HEIGHT   640
#define SUB_FORMAT   PixelFormat::Y8I
#define DEPTH_WIDTH  544
#define DEPTH_HEIGHT 640
#define DEPTH_FORMAT PixelFormat::Z16

#ifndef NOMINMAX
#define NOMINMAX
#endif

class HidDevice;

enum class FrameType {
    Unknown,
    Gray,
    Depth,
    Color
};

struct FrameInfo {
    FrameType type;
    uint64_t timestamp;
    uint64_t right_timestamp;
    int width;
    int height;
    PixelFormat format;
    DWORD streamIndex;
};

static inline uint32_t rdLe32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline uint64_t rdLe64(const uint8_t* p) {
    return  (uint64_t)p[0]        | ((uint64_t)p[1] << 8)  |
            ((uint64_t)p[2] << 16) | ((uint64_t)p[3] << 24) |
            ((uint64_t)p[4] << 32) | ((uint64_t)p[5] << 40) |
            ((uint64_t)p[6] << 48) | ((uint64_t)p[7] << 56);
}

static void sortCapabilities(std::vector<DeviceCapability>& caps) {
    std::sort(caps.begin(), caps.end(), [](const DeviceCapability& a, const DeviceCapability& b) {
        if (a.format != b.format) {
            return static_cast<int>(a.format) < static_cast<int>(b.format);
        }
        const long long areaA = (long long)a.width * a.height;
        const long long areaB = (long long)b.width * b.height;
        if (areaA != areaB) return areaA < areaB;
        if (a.width != b.width) return a.width < b.width;
        return a.fps < b.fps;
    });
}

static bool parseRsVendorMetadata(const uint8_t* buf, size_t len,
                                  uint64_t* time_us_64, uint32_t* frame_counter)
{
    if (!buf || len < MS_HEADER_SIZE + 8)
        return false;

    size_t off = MS_HEADER_SIZE;
    while (off + 8 <= len) {
        uint32_t id   = rdLe32(buf + off);
        uint32_t size = rdLe32(buf + off + 4);
        if (size < 8 || off + size > len)
            break;
        if (id == RS_META_CAPTURE_TIMING_ID && size >= 28 && off + 20 + 8 <= len) {
            if (frame_counter) *frame_counter = rdLe32(buf + off + 16);
            if (time_us_64)    *time_us_64    = rdLe64(buf + off + 20);
            return true;
        }
        off += size;
    }

    if (len >= MS_HEADER_SIZE + 8) {
        if (time_us_64)    *time_us_64    = rdLe64(buf + 8);
        if (frame_counter) *frame_counter = 0;
        return true;
    }
    return false;
}

static bool joinWithTimeout(std::thread& t, DWORD timeoutMs, const char* label) {
    if (!t.joinable()) return true;
    HANDLE h = t.native_handle();
    if (WaitForSingleObject(h, timeoutMs) == WAIT_OBJECT_0) {
        t.join();
        return true;
    }
    SDK_LOG_INFO(
            "[SDK][WARN] Thread '%s' did not exit within %lu ms "
            "(likely stuck in ReadSample/blocking I/O). Detaching instead of waiting forever.\n",
            label, timeoutMs);
    t.detach();
    return false;
}

class MFVideoSource {
public:
    MFVideoSource() {
        HRESULT hr = MFStartup(MF_VERSION);
        if (FAILED(hr)) {
            SDK_LOG_ERROR("[MFVideoSource] MFStartup failed: 0x%08lx\n", hr);
        }
    }
    
    ~MFVideoSource() {
        close();
        MFShutdown();
    }

    bool open(const std::string& devicePath, int deviceIndex, int width, int height, PixelFormat fmt, int fps)
    {
        close();

        devicePath_ = devicePath;
        deviceIndex_ = deviceIndex;
        width_ = width;
        height_ = height;
        format_ = fmt;
        fps_ = fps;

        if (!createMediaSource(devicePath))
            return false;

        if (!configureSourceReader())
            return false;
        //------------------------------------------------
        // RGB Camera
        //------------------------------------------------
        if (deviceIndex_ == 0)
        {
            if (!configureStream(0, fmt, width, height, fps)) {
                SDK_LOG_ERROR("RGB stream configure failed.\n");
                return false;
            }
        }
        //------------------------------------------------
        // Depth Camera
        //------------------------------------------------
        else
        {
            if (!configureStream(0, fmt, width, height, fps)) {
                SDK_LOG_ERROR("Depth stream configure failed.\n");
                return false;
            }

            PixelFormat grayFmt = PixelFormat::Y8I;
            if (!configureStream(1, grayFmt, width, height, fps)) {
                grayFmt = PixelFormat::GREY;
                if (!configureStream(1, grayFmt, width, height, fps)) {
                    SDK_LOG_ERROR("IR stream configure failed.\n");
                    return false;
                }
            }
        }

        printf("[MFVideoSource] Open success: %dx%d@%d, format=%d\n", width, height, fps, (int)fmt);

        return true;
    }

    bool openWithTwoStreams(const std::string& devicePath, int deviceIndex, 
                            int width, int height, 
                            PixelFormat fmt0, int fps0,
                            PixelFormat fmt1, int fps1)
    {
        close();

        devicePath_ = devicePath;
        deviceIndex_ = deviceIndex;
        width_ = width;
        height_ = height;
        format_ = fmt0;
        fps_ = fps0;

        if (!createMediaSource(devicePath))
            return false;

        if (!configureSourceReader())
            return false;

        if (!configureStream(0, fmt0, width, height, fps0)) {
            SDK_LOG_ERROR("Stream 0 configure failed.\n");
            return false;
        }

        if (!configureStream(1, fmt1, width, height, fps1)) {
            SDK_LOG_ERROR("Stream 1 configure failed.\n");
            return false;
        }

        printf("[MFVideoSource] Open success: two streams (%dx%d@%d, %dx%d@%d)\n", 
            width, height, fps0, width, height, fps1);
        return true;
    }
    
    void close() {
        stop();
        
        if (sourceReader_) {
            sourceReader_->Release();
            sourceReader_ = nullptr;
        }
        if (mediaSource_) {
            mediaSource_->Shutdown();
            mediaSource_->Release();
            mediaSource_ = nullptr;
        }
    }
    
    bool start() {
        if (!sourceReader_ || running_) return false;
        running_ = true;
        return true;
    }
    
    void stop() {
        running_ = false;
    }
    
    bool isRunning() const { return running_; }

    bool readFrame(DWORD requestedStream, uint8_t*& data, size_t& size, FrameInfo& info, DWORD* outActualStream = nullptr) {
        data = nullptr;
        size = 0;

        memset(&info, 0, sizeof(info));

        if (requestedStream != MF_SOURCE_READER_ANY_STREAM) {
            if (requestedStream >= 2) return false;
            if (!streamEnabled_[requestedStream]) return false;
        }
        if (!sourceReader_) return false;

        std::lock_guard<std::mutex> lock(readerMutex_);
        IMFSample* sample = nullptr;
        DWORD actualStream = MF_SOURCE_READER_ANY_STREAM;
        DWORD flags = 0;
        LONGLONG ts = 0;

        HRESULT hr = sourceReader_->ReadSample(requestedStream, 0, &actualStream, &flags, &ts, &sample);
        if (FAILED(hr)) {
            if (hr == MF_E_VIDEO_RECORDING_DEVICE_INVALIDATED) {
                SDK_LOG_ERROR("[MFVideoSource] Device disconnected (stream=%lu)\n", requestedStream);
                running_ = false;
                return false;
            }
            // SDK_LOG_DEBUG("[MFVideoSource] ReadSample(stream=%lu) FAILED hr=0x%08lx flags=0x%08lx actual=%lu\n",
            //     requestedStream, hr, flags, actualStream);
            if (sample) sample->Release();
            return false;
        }
        if (outActualStream) *outActualStream = actualStream;

        if (actualStream < 2 && !streamEnabled_[actualStream]) {
            if (sample) sample->Release();
            return false;
        }

        if (flags & MF_SOURCE_READERF_ERROR) {
            SDK_LOG_ERROR("[MFVideoSource] Stream %lu ERROR flags=0x%08lx\n", requestedStream, flags);
            if (sample) sample->Release();
            return false;
        }

        if (flags & MF_SOURCE_READERF_ENDOFSTREAM) {
            SDK_LOG_ERROR("[MFVideoSource] Stream %lu END_OF_STREAM\n", requestedStream);
            if (sample) sample->Release();
            return false;
        }

        if (flags & MF_SOURCE_READERF_STREAMTICK) {
            SDK_LOG_ERROR("[MFVideoSource] Stream %lu STREAMTICK ts=%lld sample=%p\n", requestedStream, ts, sample);
        }

        if (!sample) {
            SDK_LOG_ERROR("[MFVideoSource] Stream %lu no sample, flags=0x%08lx\n", requestedStream, flags);
            return false;
        }
        
        uint64_t mdTimeUs64 = 0;
        uint32_t mdFrameCounter = 0;
        bool     mdValid = false;

        {
            IMFAttributes* pSampleAttrs = nullptr;
            if (SUCCEEDED(sample->QueryInterface(IID_PPV_ARGS(&pSampleAttrs)))) {
                IUnknown* spUnknown = nullptr;
                if (SUCCEEDED(pSampleAttrs->GetUnknown(MFSampleExtension_CaptureMetadata,
                                                       IID_PPV_ARGS(&spUnknown)))) {
                    IMFAttributes* spMetadata = nullptr;
                    if (SUCCEEDED(spUnknown->QueryInterface(IID_PPV_ARGS(&spMetadata)))) {
                        IUnknown* spInner = nullptr;
                        if (SUCCEEDED(spMetadata->GetUnknown(VENDOR_META_ITEM0,
                                                             IID_PPV_ARGS(&spInner)))) {
                            IMFMediaBuffer* spInnerBuf = nullptr;
                            if (SUCCEEDED(spInner->QueryInterface(IID_PPV_ARGS(&spInnerBuf)))) {
                                BYTE* pBytes = nullptr;
                                DWORD metaLen = 0;
                                if (SUCCEEDED(spInnerBuf->Lock(&pBytes, nullptr, &metaLen))) {
                                    mdValid = parseRsVendorMetadata(pBytes, metaLen, &mdTimeUs64, &mdFrameCounter);
                                    spInnerBuf->Unlock();
                                }
                                spInnerBuf->Release();
                            }
                            spInner->Release();
                        }
                        spMetadata->Release();
                    }
                    spUnknown->Release();
                }
                pSampleAttrs->Release();
            }
        }

#if defined(INSIGHT9_MD_DEBUG)
SDK_LOG_INFO("[MD] stream=%lu valid=%d counter=%u time_us64=%llu\n",
       requestedStream, (int)mdValid, mdFrameCounter, (unsigned long long)mdTimeUs64);
#endif

        IMFMediaBuffer* buffer = nullptr;
        hr = sample->ConvertToContiguousBuffer(&buffer);
        if (FAILED(hr) || !buffer) {
            SDK_LOG_ERROR("[MFVideoSource] Stream %lu ConvertToContiguousBuffer failed hr=0x%08lx\n",
                requestedStream, hr);
            sample->Release();
            return false;
        }

        BYTE* ptr = nullptr;

        DWORD maxLen = 0;
        DWORD curLen = 0;

        hr = buffer->Lock(&ptr, &maxLen, &curLen);

        if (FAILED(hr) || !ptr || curLen == 0) {
            SDK_LOG_ERROR("[MFVideoSource] Stream %lu Buffer Lock failed hr=0x%08lx curLen=%lu\n",
                requestedStream, hr, curLen);

            buffer->Release();
            sample->Release();

            return false;
        }

        data = new (std::nothrow) uint8_t[curLen];

        if (!data) {
            buffer->Unlock();
            buffer->Release();
            sample->Release();

            return false;
        }
        memcpy(data, ptr, curLen);
        size = curLen;

        buffer->Unlock();
        buffer->Release();
        sample->Release();

        info.streamIndex = actualStream;
        info.width = streamWidth_[actualStream];
        info.height = streamHeight_[actualStream];
        info.format = streamFormat_[actualStream];

        switch (streamFormat_[actualStream]) {
            case PixelFormat::MJPEG: info.type = FrameType::Color; break;
            case PixelFormat::Z16:   info.type = FrameType::Depth; break;
            case PixelFormat::Y8I:   info.type = FrameType::Gray; break;
            case PixelFormat::GREY:  info.type = FrameType::Gray; break;
            case PixelFormat::YUYV:  info.type = FrameType::Color; break;
            case PixelFormat::NV12:  info.type = FrameType::Color; break;
            default:                 info.type = FrameType::Unknown; break;
        }

        if (mdValid) {
            info.timestamp = mdTimeUs64;
        } else {
            info.timestamp = (uint64_t)(ts / 10);
        }

        if (lastTimestamp_[actualStream] != 0 && lastTimestamp_[actualStream] == info.timestamp) {
            SDK_LOG_ERROR("[MFVideoSource] Stream %lu duplicate timestamp=%llu\n", requestedStream, info.timestamp);
            delete[] data;
            data = nullptr;
            size = 0;
            return false;
        }
        lastTimestamp_[actualStream] = info.timestamp;

        return true;
    }

    bool getDefaultCapability(int streamIndex, DeviceCapability& cap) {
        if (!sourceReader_) return false;

        IMFMediaType* type = nullptr;
        HRESULT hr = sourceReader_->GetNativeMediaType(streamIndex, 0, &type);
        if (FAILED(hr) || !type) {
            return false;
        }

        GUID subtype;
        UINT32 w = 0, h = 0;
        UINT32 num = 0, den = 1;

        type->GetGUID(MF_MT_SUBTYPE, &subtype);
        MFGetAttributeSize(type, MF_MT_FRAME_SIZE, &w, &h);
        MFGetAttributeRatio(type, MF_MT_FRAME_RATE, &num, &den);

        PixelFormat fmt = guidToPixelFormat(subtype);
        if (fmt == PixelFormat::Unknown) {
            type->Release();
            return false;
        }

        cap.width = w;
        cap.height = h;
        cap.fps = den ? num / den : 0;
        cap.format = fmt;
        cap.valid = true;

        type->Release();
        return true;
    }

    bool getAllCapabilities(int streamIndex, std::vector<DeviceCapability>& caps) {
        if (!sourceReader_) return false;

        caps.clear();
        DWORD index = 0;

        while (true) {
            IMFMediaType* type = nullptr;
            HRESULT hr = sourceReader_->GetNativeMediaType(streamIndex, index, &type);
            if (FAILED(hr) || !type) break;

            GUID subtype;
            UINT32 w = 0, h = 0;
            UINT32 num = 0, den = 1;

            type->GetGUID(MF_MT_SUBTYPE, &subtype);
            MFGetAttributeSize(type, MF_MT_FRAME_SIZE, &w, &h);
            MFGetAttributeRatio(type, MF_MT_FRAME_RATE, &num, &den);

            PixelFormat fmt = guidToPixelFormat(subtype);
            if (fmt != PixelFormat::Unknown) {
                DeviceCapability cap;
                cap.width = w;
                cap.height = h;
                cap.fps = den ? num / den : 0;
                cap.format = fmt;
                cap.valid = true;
                caps.push_back(cap);
            }

            type->Release();
            ++index;
        }

        return !caps.empty();
    }

    bool MFVideoSource::tryReadMetadata(IMFSample* sample, uint8_t outBuffer[UVC_HEADER_SIZE]) {
        if (!sample) return false;

        HRESULT hr = S_OK;
        IMFAttributes* pSampleAttrs = nullptr;
        IUnknown* spUnknown = nullptr;
        IMFAttributes* spMetadata = nullptr;
        IMFMediaBuffer* spBuffer = nullptr;
        BYTE* pData = nullptr;
        DWORD curLen = 0;
        bool success = false;

        do {
            hr = sample->QueryInterface(IID_PPV_ARGS(&pSampleAttrs));
            if (FAILED(hr)) break;

            hr = pSampleAttrs->GetUnknown(MFSampleExtension_CaptureMetadata, IID_PPV_ARGS(&spUnknown));
            if (FAILED(hr) || !spUnknown) break;

            hr = spUnknown->QueryInterface(IID_PPV_ARGS(&spMetadata));
            if (FAILED(hr) || !spMetadata) break;

            hr = spMetadata->GetUnknown(MF_CAPTURE_METADATA_FRAME_RAWSTREAM, IID_PPV_ARGS(&spBuffer));
            if (FAILED(hr) || !spBuffer) break;

            hr = spBuffer->Lock(&pData, nullptr, &curLen);
            if (FAILED(hr) || !pData || curLen < sizeof(MsMetadataHeader)) break;

            auto* ms = reinterpret_cast<const MsMetadataHeader*>(pData);

            uint32_t pts = ms->startOfFrame.PresentationTimeStamp;

            const uint8_t* vendor = pData + sizeof(MsMetadataHeader);
            size_t vendorLen = curLen - sizeof(MsMetadataHeader);

            if (vendorLen >= 24) {
                uint32_t id   = read_le32(vendor + 0);
                if (id == 0x80000001) {
                    uint32_t frameCounter    = read_le32(vendor + 16);
                    uint64_t sensorTimestamp = read_le32(vendor + 20);
                }
            }

            success = true;
        } while (false);

        if (pData) spBuffer->Unlock();
        if (spBuffer) spBuffer->Release();
        if (spMetadata) spMetadata->Release();
        if (spUnknown) spUnknown->Release();
        if (pSampleAttrs) pSampleAttrs->Release();

        return success;
    }

    uint64_t getLastTimestamp(DWORD streamIndex) const {
        if (streamIndex >= 2) return 0;
        return lastTimestamp_[streamIndex].load(std::memory_order_relaxed);
    }

private:
    PixelFormat guidToPixelFormat(const GUID& guid) {
        if (IsEqualGUID(guid, MFVideoFormat_MJPG)) return PixelFormat::MJPEG;
        if (IsEqualGUID(guid, MEDIASUBTYPE_Z16)) return PixelFormat::Z16;
        if (IsEqualGUID(guid, MEDIASUBTYPE_Y800)) return PixelFormat::GREY;
        if (IsEqualGUID(guid, MFVideoFormat_Y8I)) return PixelFormat::Y8I;
        if (IsEqualGUID(guid, MFVideoFormat_YUY2)) return PixelFormat::YUYV;
        if (IsEqualGUID(guid, MFVideoFormat_RGB24)) return PixelFormat::RGB8;
        if (IsEqualGUID(guid, MFVideoFormat_NV12)) return PixelFormat::NV12;
        return PixelFormat::Unknown;
    }

    bool configureStream(DWORD streamIndex, PixelFormat fmt, int width, int height, int fps)
    {
        if (!sourceReader_) return false;

        GUID targetSubtype = GUID_NULL;

        switch(fmt)
        {
            case PixelFormat::MJPEG:
                targetSubtype = MFVideoFormat_MJPG;
                break;
            case PixelFormat::Z16:
                targetSubtype = MEDIASUBTYPE_Z16;
                break;
            case PixelFormat::Y8I:
                targetSubtype = MFVideoFormat_Y8I;
                break;
            case PixelFormat::GREY:
                targetSubtype = MEDIASUBTYPE_Y800;
                break;
            case PixelFormat::YUYV:
                targetSubtype = MFVideoFormat_YUY2;
                break;
            case PixelFormat::NV12:
                targetSubtype = MFVideoFormat_NV12;
                break;
            default:
                return false;
        }

        IMFMediaType* bestType = nullptr;
        DWORD index = 0;

        while (true)
        {
            IMFMediaType* type = nullptr;

            HRESULT hr = sourceReader_->GetNativeMediaType(streamIndex, index, &type);
            if (FAILED(hr)) break;

            GUID subtype;

            UINT32 w = 0;
            UINT32 h = 0;
            UINT32 num = 0;
            UINT32 den = 1;

            type->GetGUID(MF_MT_SUBTYPE, &subtype);

            MFGetAttributeSize(type, MF_MT_FRAME_SIZE, &w, &h);

            MFGetAttributeRatio(type, MF_MT_FRAME_RATE, &num, &den);

            char fourcc[5] = {0};

            memcpy(fourcc, &subtype.Data1, 4);
            SDK_LOG_DEBUG("[Stream%u] [%02u] %ux%u %u/%u %s\n", streamIndex, index, w, h, num, den, fourcc);

            double rate = den ? (double)num / den : 0.0;
            if (IsEqualGUID(subtype, targetSubtype) && w == (UINT32)width && h == (UINT32)height && fabs(rate - fps) < 0.1) {
                bestType = type;
                break;
            }

            type->Release();
            ++index;
        }

        if(bestType==nullptr) {
            SDK_LOG_ERROR("Cannot find media type for stream %u\n", streamIndex);
            return false;
        }

        HRESULT hr = sourceReader_->SetCurrentMediaType(streamIndex, nullptr, bestType);

        bestType->Release();

        if(FAILED(hr)) {
            SDK_LOG_ERROR("SetCurrentMediaType stream %u failed 0x%08lx\n", streamIndex, hr);
            return false;
        }

        sourceReader_->Flush(streamIndex);

        streamEnabled_[streamIndex]=true;
        streamFormat_[streamIndex]=fmt;
        streamWidth_[streamIndex]=width;
        streamHeight_[streamIndex]=height;

        return true;
    }

    std::string getSymbolicLink(const std::string& devicePath) {
        std::string upper = devicePath;
        for (char& c : upper) c = toupper(c);
        
        size_t vidPos = upper.find("VID_");
        if (vidPos == std::string::npos) return "";
        
        size_t pidPos = upper.find("PID_");
        if (pidPos == std::string::npos) return "";
        
        std::string vid = upper.substr(vidPos, 13);
        std::string pid = upper.substr(pidPos, 13);
        
        size_t miPos = upper.find("MI_");
        std::string mi = "";
        if (miPos != std::string::npos) {
            mi = upper.substr(miPos, 5);
        }
        
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
        std::string result;
        
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
                    
                    std::string pathUpper = path;
                    for (char& c : pathUpper) c = toupper(c);
                    
                    if (pathUpper.find(vid) != std::string::npos &&
                        pathUpper.find(pid) != std::string::npos &&
                        (mi.empty() || pathUpper.find(mi) != std::string::npos)) {
                        
                        result = path; 
                        
                        VariantClear(&var);
                        pPropBag->Release();
                        pMoniker->Release();
                        pEnum->Release();
                        pDevEnum->Release();
                        printf("[MFVideoSource] Found symbolic link: %s\n", result.c_str());
                        return result;
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

    bool probeStreamCount() {
        IMFPresentationDescriptor* pd = nullptr;
        HRESULT hr = mediaSource_->CreatePresentationDescriptor(&pd);
        if (FAILED(hr) || !pd) {
            SDK_LOG_ERROR("[Probe] CreatePresentationDescriptor failed: 0x%08lx\n", hr);
            return false;
        }

        DWORD streamCount = 0;
        pd->GetStreamDescriptorCount(&streamCount);
        streamCount_ = streamCount;
        printf("[Probe] Total stream count on this media source: %lu\n", streamCount);

        for (DWORD i = 0; i < streamCount; ++i) {
            BOOL selected = FALSE;
            IMFStreamDescriptor* sd = nullptr;
            if (SUCCEEDED(pd->GetStreamDescriptorByIndex(i, &selected, &sd)) && sd) {
                DWORD streamId = 0;
                sd->GetStreamIdentifier(&streamId);
                SDK_LOG_DEBUG("[Probe] stream[%lu] id=%lu selected=%d\n", i, streamId, selected);

                IMFMediaTypeHandler* handler = nullptr;
                if (SUCCEEDED(sd->GetMediaTypeHandler(&handler)) && handler) {
                    GUID majorType = GUID_NULL;
                    handler->GetMajorType(&majorType);
                    const char* kindStr = "Unknown";
                    if (majorType == MFMediaType_Video) kindStr = "Video";
                    else if (majorType == MFMediaType_Metadata) kindStr = "Metadata";
                    printf("[Probe]   stream[%lu] id=%lu selected=%d majorType=%s\n",
                        i, streamId, selected, kindStr);

                    DWORD typeCount = 0;
                    handler->GetMediaTypeCount(&typeCount);
                    for (DWORD t = 0; t < typeCount; ++t) {
                        IMFMediaType* mt = nullptr;
                        if (SUCCEEDED(handler->GetMediaTypeByIndex(t, &mt)) && mt) {
                            GUID subtype; UINT32 w=0,h=0,num=0,den=1;
                            mt->GetGUID(MF_MT_SUBTYPE, &subtype);
                            MFGetAttributeSize(mt, MF_MT_FRAME_SIZE, &w, &h);
                            MFGetAttributeRatio(mt, MF_MT_FRAME_RATE, &num, &den);
                            char fourcc[5] = {0};
                            memcpy(fourcc, &subtype.Data1, 4);
                            SDK_LOG_DEBUG("    [type %lu] %ux%u %u/%u subtype=%s\n", t, w, h, num, den, fourcc);
                            mt->Release();
                        }
                    }
                    handler->Release();
                }
                sd->Release();
            }
        }
        pd->Release();
        return streamCount > 1;
    }

    bool createMediaSource(const std::string& devicePath) {
        std::string symbolicLink = getSymbolicLink(devicePath);
        if (symbolicLink.empty()) {
            SDK_LOG_ERROR("[MFVideoSource] Failed to get symbolic link\n");
            return false;
        }
        
        std::wstring wlink(symbolicLink.begin(), symbolicLink.end());
        
        IMFAttributes* pAttributes = nullptr;
        HRESULT hr = MFCreateAttributes(&pAttributes, 2);
        if (FAILED(hr)) {
            SDK_LOG_ERROR("[MFVideoSource] MFCreateAttributes failed: 0x%08lx\n", hr);
            return false;
        }
        
        pAttributes->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, 
                             MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
        pAttributes->SetString(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK,
                               wlink.c_str());
        
        IMFMediaSource* pSource = nullptr;
        hr = MFCreateDeviceSource(pAttributes, &pSource);
        pAttributes->Release();
        
        if (FAILED(hr)) {
            SDK_LOG_ERROR("[MFVideoSource] MFCreateDeviceSource failed: 0x%08lx\n", hr);
            return false;
        }
        
        mediaSource_ = pSource;
        probeStreamCount();
        return true;
    }
    
    bool configureSourceReader() {
        IMFAttributes* attr = nullptr;

        MFCreateAttributes(&attr,2);

        attr->SetUINT32(MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, TRUE);
        // attr->SetUINT32(MF_LOW_LATENCY, TRUE);
        // attr->SetUINT32(MF_READWRITE_DISABLE_CONVERTERS, TRUE);
        attr->SetUINT32(MF_SOURCE_READER_ENABLE_ADVANCED_VIDEO_PROCESSING, FALSE);
        // attr->SetUINT32(MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING, FALSE);

        HRESULT hr = MFCreateSourceReaderFromMediaSource(mediaSource_, attr, &sourceReader_);

        if(attr) attr->Release();

        if(FAILED(hr)) {
            SDK_LOG_ERROR("[MFVideoSource] MFCreateSourceReaderFromMediaSource failed: 0x%08lX\n", hr);
            return false;
        }
        //------------------------------------------
        // disable all stream
        //------------------------------------------
        sourceReader_->SetStreamSelection(MF_SOURCE_READER_ALL_STREAMS, FALSE);
        sourceReader_->SetStreamSelection(MF_SOURCE_READER_FIRST_VIDEO_STREAM, TRUE);
        //------------------------------------------
        // RGB camera
        //------------------------------------------
        if(deviceIndex_==0)
        {
            sourceReader_->SetStreamSelection(0,TRUE);
        }
        //------------------------------------------
        // Depth camera
        //------------------------------------------
        else
        {
            sourceReader_->SetStreamSelection(0,TRUE);
            sourceReader_->SetStreamSelection(1,TRUE);
        }

        BOOL sel=FALSE;

        sourceReader_->GetStreamSelection(0,&sel);
        sourceReader_->GetStreamSelection(1,&sel);

        DWORD streamCount = 0;
        for (DWORD i = 0; i < streamCount_; ++i) {
            IMFMediaType* pType = nullptr;
            HRESULT hr = sourceReader_->GetCurrentMediaType(i, &pType);
            if (FAILED(hr)) {
                hr = sourceReader_->GetNativeMediaType(i, 0, &pType);
            }
            if (SUCCEEDED(hr) && pType) {
                GUID majorType;
                if (SUCCEEDED(pType->GetGUID(MF_MT_MAJOR_TYPE, &majorType)) &&
                    majorType == MFMediaType_Metadata) {
                    sourceReader_->SetStreamSelection(i, TRUE);
                    printf("[MFVideoSource] Enabled metadata stream %lu\n", i);
                    pType->Release();
                    break;
                }
                pType->Release();
            }
        }

        return true;
    }
    
    bool findVideoStream() {
        IMFMediaType* pType = nullptr;
        HRESULT hr = sourceReader_->GetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, &pType);
        if (FAILED(hr) || !pType) {
            SDK_LOG_ERROR("[MFVideoSource] GetCurrentMediaType failed: 0x%08lx\n", hr);
            return false;
        }
        
        GUID majorType;
        pType->GetGUID(MF_MT_MAJOR_TYPE, &majorType);
        pType->Release();
        
        if (majorType != MFMediaType_Video) {
            SDK_LOG_INFO("[MFVideoSource] Not a video stream\n");
            return false;
        }
        
        return true;
    }

    static uint32_t read_le32(const uint8_t* p) {
        return ((uint32_t)p[0]) | ((uint32_t)p[1] << 8) |
            ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    }

    IMFMediaSource* mediaSource_ = nullptr;
    IMFSourceReader* sourceReader_ = nullptr;
    
    int width_ = 0;
    int height_ = 0;
    PixelFormat format_ = PixelFormat::Unknown;
    int fps_ = 30;
    std::string devicePath_;
    int deviceIndex_ = -1;
    
    std::atomic<bool> running_{false};
    std::mutex readerMutex_;
    std::atomic<uint64_t> lastTimestamp_[2] = {0, 0};
    uint32_t lastMeta32_[2]   = {0, 0};
    uint64_t metaWrapHigh_[2] = {0, 0};
    bool     metaInit_[2]     = {false, false};

    uint64_t unwrapMetaTimestamp(DWORD streamIndex, uint32_t ts32) {
        if (metaInit_[streamIndex] && ts32 < lastMeta32_[streamIndex] &&
            (lastMeta32_[streamIndex] - ts32) > 0x80000000u) {
            metaWrapHigh_[streamIndex] += 0x100000000ull;
        }
        lastMeta32_[streamIndex] = ts32;
        metaInit_[streamIndex] = true;
        return metaWrapHigh_[streamIndex] + (uint64_t)ts32;
    }

    DWORD streamCount_ = 0;
    bool streamEnabled_[2] = {false, false};
    PixelFormat streamFormat_[2] = {PixelFormat::Unknown, PixelFormat::Unknown};
    int streamWidth_[2] = {0,0};
    int streamHeight_[2] = {0,0};
};

struct sdk_ctx_t {
    insight9_config_t config;
    // =====================================================
    // Physical UVC devices
    // videos[0] = MI_03 RGB
    // videos[1] = MI_00 Composite
    //               stream 0 = Depth
    //               stream 1 = Gray
    // =====================================================
    MFVideoSource* videos[UVC_NUM];
    std::string videoPaths[UVC_NUM];
    // =====================================================
    // Logical camera state
    // cam 0 = RGB
    // cam 1 = Gray
    // cam 2 = Depth
    // =====================================================
    std::atomic<bool> cam_running[LOGICAL_CAM_NUM];
    std::thread videoThreads[UVC_NUM];
    bool videoThreadStuck[UVC_NUM] = {false, false};
    bool hidThreadStuck[HID_NUM] = {false, false};
    // =====================================================
    HidDevice* hidDevs[HID_NUM];
    std::string hidPaths[HID_NUM];
    std::thread hidThreads[HID_NUM];
    viewer::ExtensionUnitControl* xu;
    // Serialize all access/recreation of the KS Extension Unit object.
    std::recursive_mutex xuMutex;
    // =====================================================
    std::atomic<bool> running;
    image_callback imgCb;
    void* imgUser;
    imu_callback imuCb;
    void* imuUser;
    vio_callback vioCb;
    void* vioUser;
    std::mutex imgMutex;
    std::mutex imuMutex;
    std::mutex vioMutex;
    bool initialized;
    uint64_t last_img_timestamp[LOGICAL_CAM_NUM];
    DeviceCapabilities_t device_caps;
    bool singleNodeMode = false;
};

static sdk_ctx_t g_ctx;

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
                
                std::string pathUpper = path;
                for (char& c : pathUpper) c = toupper(c);
                std::string devicePathUpper = devicePath;
                for (char& c : devicePathUpper) c = toupper(c);
                
                size_t pos1 = pathUpper.find("VID_");
                size_t pos2 = devicePathUpper.find("VID_");
                if (pos1 != std::string::npos && pos2 != std::string::npos) {
                    std::string pathId = pathUpper.substr(pos1);
                    std::string deviceId = devicePathUpper.substr(pos2);
                    
                    size_t end1 = pathId.find_first_of("#\\");
                    size_t end2 = deviceId.find_first_of("#\\");
                    if (end1 != std::string::npos && end2 != std::string::npos) {
                        pathId = pathId.substr(0, end1);
                        deviceId = deviceId.substr(0, end2);
                    }
                    
                    if (pathId == deviceId) {
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
                            SDK_LOG_INFO("[SDK] Matched device: %s -> FriendlyName: %s\n", 
                                   devicePath.c_str(), name.c_str());
                            return name;
                        }
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
    std::vector<std::string> rgbPaths;
    std::vector<std::string> grayDepthPaths;
    
    ICreateDevEnum* pDevEnum = nullptr;
    IEnumMoniker* pEnum = nullptr;
    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, 
                                   IID_ICreateDevEnum, (void**)&pDevEnum);
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

                    VARIANT nameVar;
                    VariantInit(&nameVar);
                    std::string friendlyName;
                    if (SUCCEEDED(pPropBag->Read(L"FriendlyName", &nameVar, 0)) && nameVar.vt == VT_BSTR) {
                        std::wstring wname = nameVar.bstrVal;
                        friendlyName = std::string(wname.begin(), wname.end());
                        VariantClear(&nameVar);
                    }

                    std::string upper = path;
                    for (char& c : upper) c = toupper(c);
                    
                    char vidStr[16], pidStr[16];
                    snprintf(vidStr, sizeof(vidStr), "VID_%04X", vid);
                    snprintf(pidStr, sizeof(pidStr), "PID_%04X", pid);
                    
                    if (upper.find(vidStr) != std::string::npos && 
                        upper.find(pidStr) != std::string::npos) {
                        
                        if (upper.find("MI_03") != std::string::npos) {
                            rgbPaths.push_back(path);
                            printf("[SDK] Found RGB device (MI_03): %s\n", path.c_str());
                            printf("[SDK]   FriendlyName: %s\n", friendlyName.c_str());
                        } else if (upper.find("MI_00") != std::string::npos) {
                            grayDepthPaths.push_back(path);
                            printf("[SDK] Found Composite device (MI_00): %s\n", path.c_str());
                            printf("[SDK]   FriendlyName: %s\n", friendlyName.c_str());
                        }
                        else if (upper.find("MI_04") != std::string::npos) {
                            rgbPaths.push_back(path);
                        } else if (upper.find("MI_06") != std::string::npos || 
                                   upper.find("MI_08") != std::string::npos) {
                            grayDepthPaths.push_back(path);
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
    
    std::sort(rgbPaths.begin(), rgbPaths.end());
    std::sort(grayDepthPaths.begin(), grayDepthPaths.end());
    
    paths.insert(paths.end(), rgbPaths.begin(), rgbPaths.end());
    paths.insert(paths.end(), grayDepthPaths.begin(), grayDepthPaths.end());

    printf("[SDK] Found %zu UVC devices:\n", paths.size());
    for (size_t i = 0; i < paths.size(); i++) {
        printf("[SDK]   [%zu] %s\n", i, paths[i].c_str());
    }

    return paths;
}

static bool reopenXUControlLocked(const char* reason) {
    std::string path = g_ctx.videoPaths[RGB_UVC_ID];
    if (path.empty()) {
        auto paths = findUvcDevices(VENDOR_ID, PRODUCT_ID);
        if (!paths.empty()) {
            path = paths[RGB_UVC_ID];
            g_ctx.videoPaths[RGB_UVC_ID] = path;
        }
    }

    if (path.empty()) {
        SDK_LOG_ERROR("[XU][ERR] Cannot reopen XU: RGB UVC path unavailable (%s)\n",
                reason ? reason : "unknown");
        return false;
    }

    if (g_ctx.xu) {
        delete g_ctx.xu;
        g_ctx.xu = nullptr;
    }

    auto* xu = new (std::nothrow) viewer::ExtensionUnitControl();
    if (!xu) {
        SDK_LOG_ERROR("[XU][ERR] Allocation failed while reopening XU (%s)\n",
                reason ? reason : "unknown");
        return false;
    }

    SDK_LOG_INFO("[XU] Reopening XU on RGB UVC: %s (%s)\n",
           path.c_str(), reason ? reason : "unknown");

    if (!xu->open(path)) {
        SDK_LOG_ERROR("[XU][WARN] Reopen failed on path: %s\n", path.c_str());
        delete xu;
        return false;
    }

    g_ctx.xu = xu;
    printf("[XU] Reopen success\n");
    return true;
}

static bool callXUWithRetry(const char* op,
                            const std::function<bool(viewer::ExtensionUnitControl&)>& fn) {
    std::lock_guard<std::recursive_mutex> lock(g_ctx.xuMutex);

    if (!g_ctx.xu && !reopenXUControlLocked(op)) {
        return false;
    }

    if (fn(*g_ctx.xu)) {
        return true;
    }

    SDK_LOG_ERROR("[XU][WARN] Operation '%s' failed; reopening XU and retrying once\n", op ? op : "unknown");

    if (!reopenXUControlLocked(op)) {
        return false;
    }

    return fn(*g_ctx.xu);
}

static int reconnect_backoff_ms(int attempt) {
    if (attempt < 1) attempt = 1;
    long delay = RECONNECT_BACKOFF_BASE_MS;
    for (int i = 1; i < attempt && delay < RECONNECT_BACKOFF_MAX_MS; ++i) delay <<= 1;
    if (delay > RECONNECT_BACKOFF_MAX_MS) delay = RECONNECT_BACKOFF_MAX_MS;
    return (int)delay;
}

static void interruptible_sleep_ms(int total_ms, std::atomic<bool>* extraStop = nullptr) {
    int slept = 0;
    while (slept < total_ms && g_ctx.running) {
        if (extraStop && !extraStop->load()) break;
        int chunk = (total_ms - slept) > 200 ? 200 : (total_ms - slept);
        std::this_thread::sleep_for(std::chrono::milliseconds(chunk));
        slept += chunk;
    }
}

static void reconnect_backoff_apply(const char* tag, int id, int* fails, const char* reason,
                                     std::atomic<bool>* extraStop = nullptr) {
    (*fails)++;
    int delay = reconnect_backoff_ms(*fails);
    if (RECONNECT_MAX_ATTEMPTS <= 0 || *fails <= RECONNECT_MAX_ATTEMPTS) {
        SDK_LOG_ERROR("[%s%d][ERR] %s, retry in %dms (attempt %d)\n", tag, id, reason, delay, *fails);
    } else if (*fails == RECONNECT_MAX_ATTEMPTS + 1) {
        SDK_LOG_ERROR("[%s%d][ERR] %s failed %d times, slowing to %ds retry\n",
                      tag, id, reason, RECONNECT_MAX_ATTEMPTS, RECONNECT_BACKOFF_MAX_MS / 1000);
    }
    interruptible_sleep_ms(delay, extraStop);
}

static bool openVideoSourceForSlot(int uvcId) {
    auto uvcPaths = findUvcDevices(VENDOR_ID, PRODUCT_ID);
    std::string path;
    if (uvcId == RGB_UVC_ID) {
        if (uvcPaths.empty()) return false;
        path = uvcPaths[0];
    } else {
        if (uvcPaths.size() < 2 && !g_ctx.singleNodeMode) return false;
        path = g_ctx.singleNodeMode ? uvcPaths[0] : uvcPaths[COMPOSITE_UVC_ID];
    }

    MFVideoSource* src = new MFVideoSource();
    bool opened = false;
    if (uvcId == RGB_UVC_ID) {
        opened = src->open(path, uvcId,
                    g_ctx.config.rgb_config.width, g_ctx.config.rgb_config.height,
                    g_ctx.config.rgb_config.pixel_format, g_ctx.config.rgb_config.fps);
    } else {
        opened = src->openWithTwoStreams(path, uvcId,
                    g_ctx.config.depth_config.width, g_ctx.config.depth_config.height,
                    g_ctx.config.depth_config.pixel_format, g_ctx.config.depth_config.fps,
                    g_ctx.config.gray_config.pixel_format, g_ctx.config.gray_config.fps);
    }

    if (!opened) {
        delete src;
        return false;
    }
    
    if (!src->start()) {          // ← 补上这一步，之前漏掉了
        delete src;
        return false;
    }

    g_ctx.videoPaths[uvcId] = path;
    g_ctx.videos[uvcId] = src;
    return true;
}

static bool probeDeviceCapabilities(const std::string& devicePath, int deviceIndex, 
                                     DeviceCapabilities_t& caps) {
    MFVideoSource* probeSource = new MFVideoSource();
    
    bool opened = false;
    
    if (deviceIndex == 0) {
        opened = probeSource->open(devicePath, deviceIndex, 1088, 1920, PixelFormat::YUYV, 30);
        if (!opened) opened = probeSource->open(devicePath, deviceIndex, 1088, 1920, PixelFormat::MJPEG, 30);
        if (!opened) opened = probeSource->open(devicePath, deviceIndex, 1088, 1920, PixelFormat::NV12, 30);
    } else {
        opened = probeSource->openWithTwoStreams(devicePath, deviceIndex, 
                                                 544, 640, 
                                                 PixelFormat::Z16, 30,
                                                 PixelFormat::Y8I, 30);
    }
    
    if (!opened) {
        SDK_LOG_ERROR("[Probe] Failed to open device\n");
        delete probeSource;
        return false;
    }
    
    if (deviceIndex == 0) {
        std::vector<DeviceCapability> allCaps;
        if (probeSource->getAllCapabilities(0, allCaps)) {
            if (!allCaps.empty()) {
                caps.rgb_default = allCaps[0];
            }
            sortCapabilities(allCaps);
            caps.rgb_capabilities = allCaps;
        }
    } else {
        std::vector<DeviceCapability> depthCaps;
        if (probeSource->getAllCapabilities(0, depthCaps)) {
            if (!depthCaps.empty()) {
                caps.depth_default = depthCaps[0];
            }
            sortCapabilities(depthCaps);
            caps.depth_capabilities = depthCaps;
        }

        std::vector<DeviceCapability> grayCaps;
        if (probeSource->getAllCapabilities(1, grayCaps)) {
            if (!grayCaps.empty()) {
                caps.gray_default = grayCaps[0];
            }
            sortCapabilities(grayCaps);
            caps.gray_capabilities = grayCaps;
        }
    }
    
    caps.initialized = true;
    probeSource->close();
    delete probeSource;
    return true;
}

static std::vector<std::string> findHidDevices(uint16_t vid, uint16_t pid) {
    std::vector<std::string> paths;
    struct hid_device_info *devs, *cur;
    devs = hid_enumerate(vid, pid);
    
    std::map<int, std::string> interfaceMap;
    for (cur = devs; cur; cur = cur->next) {
        if (cur->interface_number >= 0) {
            SDK_LOG_DEBUG("[HID] Interface %d: %s\n", cur->interface_number, cur->path);
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

class HidDevice {
public:
    bool open(const std::string& path) {
        close();
        path_ = path;
        dev_ = hid_open_path(path.c_str());
        if (dev_) {
            hid_set_nonblocking(dev_, 1);
        }
        return dev_ != nullptr;
    }
    void close() {
        if (dev_) {
            hid_close(dev_);
            dev_ = nullptr;
        }
    }
    // Returns true when a report is available.
    // disconnected is set only when hidapi reports a real read error (ret < 0);
    // ret == 0 is just the normal non-blocking "no data yet" case.
    bool read(uint8_t* buf, size_t& len, bool* disconnected = nullptr) {
        if (disconnected) *disconnected = false;
        if (!dev_) {
            if (disconnected) *disconnected = true;
            return false;
        }
        int ret = hid_read(dev_, buf, (size_t)len);
        if (ret > 0) {
            len = static_cast<size_t>(ret);
            return true;
        }
        if (ret < 0 && disconnected) {
            *disconnected = true;
        }
        return false;
    }
    const std::string& path() const { return path_; }
private:
    hid_device* dev_ = nullptr;
    std::string path_;
};

class ImuSensorEvents : public ISensorEvents {
public:
    ImuSensorEvents(bool isAccel, IPortableDeviceKeyCollection* pAllKeys)
        : isAccel_(isAccel), pAllKeys_(pAllKeys) {
        if (pAllKeys_) pAllKeys_->AddRef();
    }
    ~ImuSensorEvents() {
        if (pAllKeys_) pAllKeys_->Release();
    }

    // IUnknown
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppv) override {
        if (riid == IID_IUnknown || riid == IID_ISensorEvents) {
            *ppv = static_cast<ISensorEvents*>(this);
            AddRef();
            return S_OK;
        }
        *ppv = nullptr;
        return E_NOINTERFACE;
    }
    ULONG STDMETHODCALLTYPE AddRef() override { return InterlockedIncrement(&ref_); }
    ULONG STDMETHODCALLTYPE Release() override {
        ULONG r = InterlockedDecrement(&ref_);
        if (r == 0) delete this;
        return r;
    }

    // ISensorEvents
    HRESULT STDMETHODCALLTYPE OnEvent(ISensor*, REFGUID, IPortableDeviceValues*) override { return S_OK; }
    HRESULT STDMETHODCALLTYPE OnLeave(REFSENSOR_ID sensorID) override { return S_OK; }
    HRESULT STDMETHODCALLTYPE OnStateChanged(ISensor* pSensor, SensorState state) override {
        SDK_LOG_INFO("[IMU] sensor state changed: %d (isAccel=%d)\n", (int)state, isAccel_);
        return S_OK;
    }

    HRESULT STDMETHODCALLTYPE OnDataUpdated(ISensor* pSensor, ISensorDataReport* pReport) override {
        if (!pReport) return S_OK;

        PROPVARIANT var;
        PropVariantInit(&var);

        uint32_t tsLow = 0, tsHigh = 0;
        bool haveDevTs = false;
        if (SUCCEEDED(pReport->GetSensorValue(SENSOR_DATA_TYPE_DEV_TS_LOW, &var))) {
            PropVariantToUInt32(var, &tsLow);
            PropVariantClear(&var);
            haveDevTs = true;
        }
        if (SUCCEEDED(pReport->GetSensorValue(SENSOR_DATA_TYPE_DEV_TS_HIGH, &var))) {
            PropVariantToUInt32(var, &tsHigh);
            PropVariantClear(&var);
        }

        uint64_t ts_device = 0;
        if (haveDevTs) {
            uint32_t tick32 = ((tsHigh & 0xFFFF) << 16) | (tsLow & 0xFFFF);
            ts_device = unwrapImuDeviceTimestamp(isAccel_, tick32);
        }

        std::lock_guard<std::mutex> lk(g_imuStateMutex);

        if (isAccel_) {
            double ax = 0, ay = 0, az = 0;
            if (SUCCEEDED(pReport->GetSensorValue(SENSOR_DATA_TYPE_ACCELERATION_X_G, &var))) {
                PropVariantToDouble(var, &ax); PropVariantClear(&var);
            }
            if (SUCCEEDED(pReport->GetSensorValue(SENSOR_DATA_TYPE_ACCELERATION_Y_G, &var))) {
                PropVariantToDouble(var, &ay); PropVariantClear(&var);
            }
            if (SUCCEEDED(pReport->GetSensorValue(SENSOR_DATA_TYPE_ACCELERATION_Z_G, &var))) {
                PropVariantToDouble(var, &az); PropVariantClear(&var);
            }
            constexpr double G_TO_MS2 = 9.80665;
            // std::lock_guard<std::mutex> lk(g_imuStateMutex);
            g_lastAx = (float)(ax * G_TO_MS2);
            g_lastAy = (float)(ay * G_TO_MS2);
            g_lastAz = (float)(az * G_TO_MS2);
            g_lastAccelTs = ts_device;
            g_accelDirty = true;
        } else {
            double gx = 0, gy = 0, gz = 0;
            if (SUCCEEDED(pReport->GetSensorValue(SENSOR_DATA_TYPE_ANGULAR_VELOCITY_X_DEGREES_PER_SECOND, &var))) {
                PropVariantToDouble(var, &gx); PropVariantClear(&var);
            }
            if (SUCCEEDED(pReport->GetSensorValue(SENSOR_DATA_TYPE_ANGULAR_VELOCITY_Y_DEGREES_PER_SECOND, &var))) {
                PropVariantToDouble(var, &gy); PropVariantClear(&var);
            }
            if (SUCCEEDED(pReport->GetSensorValue(SENSOR_DATA_TYPE_ANGULAR_VELOCITY_Z_DEGREES_PER_SECOND, &var))) {
                PropVariantToDouble(var, &gz); PropVariantClear(&var);
            }
            constexpr double DEG_TO_RAD = 3.14159265358979 / 180.0;
            // std::lock_guard<std::mutex> lk(g_imuStateMutex);
            g_lastGx = (float)(gx * DEG_TO_RAD);
            g_lastGy = (float)(gy * DEG_TO_RAD);
            g_lastGz = (float)(gz * DEG_TO_RAD);
            g_lastGyroTs = ts_device;
            g_gyroDirty = true;
        }

        if (g_accelDirty && g_gyroDirty) {
            if (g_ctx.imuCb) {
                g_ctx.imuCb(g_lastAx, g_lastAy, g_lastAz,
                            g_lastGx, g_lastGy, g_lastGz,
                            (g_lastAccelTs > g_lastGyroTs) ? g_lastAccelTs : g_lastGyroTs,
                            g_ctx.imuUser);
            }
            g_accelDirty = false;
            g_gyroDirty = false;
        }
        return S_OK;
    }

private:
    bool isAccel_;
    LONG ref_ = 1;
    IPortableDeviceKeyCollection* pAllKeys_ = nullptr;

    static uint64_t unwrapImuDeviceTimestamp(bool isAccel, uint32_t tick32) {
        static std::mutex s_mtx;
        static uint32_t s_lastTick[2] = {0, 0};
        static uint64_t s_highPart[2] = {0, 0};
        static bool     s_inited[2]   = {false, false};

        int idx = isAccel ? 0 : 1;
        std::lock_guard<std::mutex> lk(s_mtx);
        if (!s_inited[idx]) {
            s_inited[idx] = true;
            s_lastTick[idx] = tick32;
            return tick32;
        }
        if (tick32 < s_lastTick[idx]) {
            s_highPart[idx] += (1ULL << 32);
        }
        s_lastTick[idx] = tick32;
        return s_highPart[idx] + tick32;
    }
};

// ============ IMU Thread：STA Message Pump ============
static void imuSensorThreadFunc() {
    // The Windows Sensor API objects are tied to the current USB device instance.
    // After a USB disconnect/re-enumeration, keeping the old ISensorManager/ISensor
    // objects alive can leave us with a permanently silent stream.  Therefore this
    // thread deliberately rebuilds the entire Sensor API binding when the sensors
    // disappear or GetState() fails.
    HRESULT hrCo = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
    if (FAILED(hrCo) && hrCo != S_FALSE) {
        SDK_LOG_ERROR("[IMU] CoInitializeEx(STA) failed hr=0x%08lx\n", hrCo);
        return;
    }

    auto cleanupBinding = [](ISensor*& pAccel, ISensor*& pGyro,
                             ImuSensorEvents*& pAccelEvents,
                             ImuSensorEvents*& pGyroEvents,
                             ISensorManager*& pManager) {
        if (pAccel) {
            pAccel->SetEventSink(nullptr);
            pAccel->Release();
            pAccel = nullptr;
        }
        if (pGyro) {
            pGyro->SetEventSink(nullptr);
            pGyro->Release();
            pGyro = nullptr;
        }
        if (pAccelEvents) {
            pAccelEvents->Release();
            pAccelEvents = nullptr;
        }
        if (pGyroEvents) {
            pGyroEvents->Release();
            pGyroEvents = nullptr;
        }
        if (pManager) {
            pManager->Release();
            pManager = nullptr;
        }
    };

    while (g_ctx.running) {
        ISensorManager* pManager = nullptr;
        ISensor* pAccel = nullptr;
        ISensor* pGyro = nullptr;
        ImuSensorEvents* pAccelEvents = nullptr;
        ImuSensorEvents* pGyroEvents = nullptr;

        HRESULT hr = CoCreateInstance(CLSID_SensorManager, nullptr, CLSCTX_INPROC_SERVER,
                                       IID_PPV_ARGS(&pManager));
        if (FAILED(hr) || !pManager) {
            SDK_LOG_ERROR("[IMU][WARN] SensorManager unavailable hr=0x%08lx, retrying\n", hr);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }

        auto bindOne = [&](REFSENSOR_TYPE_ID typeId, bool isAccel,
                           ISensor** outSensor, ImuSensorEvents** outEvents) -> bool {
            if (!outSensor || !outEvents) return false;
            *outSensor = nullptr;
            *outEvents = nullptr;

            ISensorCollection* pColl = nullptr;
            HRESULT h = pManager->GetSensorsByType(typeId, &pColl);
            if (FAILED(h) || !pColl) return false;

            ULONG count = 0;
            pColl->GetCount(&count);
            if (count == 0) {
                pColl->Release();
                return false;
            }

            ISensor* pSensor = nullptr;
            h = pColl->GetAt(0, &pSensor);
            pColl->Release();
            if (FAILED(h) || !pSensor) return false;

            IPortableDeviceKeyCollection* pAllKeys = nullptr;
            pSensor->GetSupportedDataFields(&pAllKeys);

            auto* events = new (std::nothrow) ImuSensorEvents(isAccel, pAllKeys);
            if (pAllKeys) pAllKeys->Release();
            if (!events) {
                pSensor->Release();
                return false;
            }

            h = pSensor->SetEventSink(events);
            if (FAILED(h)) {
                events->Release();
                pSensor->Release();
                return false;
            }

            *outSensor = pSensor;
            *outEvents = events;
            return true;
        };

        const bool hasAccel = bindOne(SENSOR_TYPE_ACCELEROMETER_3D, true,
                                      &pAccel, &pAccelEvents);
        const bool hasGyro = bindOne(SENSOR_TYPE_GYROMETER_3D, false,
                                     &pGyro, &pGyroEvents);

        if (!hasAccel && !hasGyro) {
            SDK_LOG_ERROR("[IMU][WARN] No accelerometer/gyrometer currently available, retrying...\n");
            cleanupBinding(pAccel, pGyro, pAccelEvents, pGyroEvents, pManager);
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            continue;
        }

        SDK_LOG_DEBUG("[IMU] Sensor API bound/rebound: accel=%d gyro=%d\n", hasAccel, hasGyro);

        bool lost = false;
        ULONGLONG lastHealthCheck = GetTickCount64();
        MSG msg;

        while (g_ctx.running && !lost) {
            DWORD ret = MsgWaitForMultipleObjects(0, nullptr, FALSE, 100, QS_ALLINPUT);
            if (!g_ctx.running) break;

            while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
                TranslateMessage(&msg);
                DispatchMessage(&msg);
            }

            const ULONGLONG now = GetTickCount64();
            if (now - lastHealthCheck < 500) continue;
            lastHealthCheck = now;

            auto sensorHealthy = [](ISensor* sensor, const char* name) -> bool {
                if (!sensor) return true;
                SensorState state = SENSOR_STATE_NOT_AVAILABLE;
                HRESULT h = sensor->GetState(&state);
                if (FAILED(h)) {
                    SDK_LOG_ERROR("[IMU][WARN] %s GetState failed hr=0x%08lx\n", name, h);
                    return false;
                }
                if (state == SENSOR_STATE_NOT_AVAILABLE ||
                    state == SENSOR_STATE_NO_DATA) {
                    SDK_LOG_ERROR("[IMU][WARN] %s state=%d, rebinding\n",
                            name, static_cast<int>(state));
                    return false;
                }
                return true;
            };

            // A failed GetState is much more reliable than waiting for callbacks to
            // arrive: after USB re-enumeration the old sensor can remain registered
            // while silently producing no data.
            if (!sensorHealthy(pAccel, "accel") ||
                !sensorHealthy(pGyro, "gyro")) {
                lost = true;
            }
        }

        cleanupBinding(pAccel, pGyro, pAccelEvents, pGyroEvents, pManager);

        if (!g_ctx.running) break;
        if (lost) {
            SDK_LOG_ERROR("[IMU][WARN] Sensor binding lost; rebuilding SensorManager/ISensor objects\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
    }

    CoUninitialize();
    SDK_LOG_INFO("[IMU] sensor thread exited\n");
}

static int streamToLogicalCamId(int uvcId, DWORD streamIndex) {
    if (uvcId == RGB_UVC_ID && streamIndex == 0) {
        return RGB_CAM_ID;  // 0
    } else if (uvcId == COMPOSITE_UVC_ID) {
        if (streamIndex == DEPTH_STREAM_ID) {
            return DEPTH_CAM_ID;  // 2
        } else if (streamIndex == GRAY_STREAM_ID) {
            return GRAY_CAM_ID;   // 1
        }
    }
    return -1;
}

static void deliverFrame(int uvcId, uint8_t* data, size_t size, const FrameInfo& info) {
    if (!g_ctx.imgCb) return;
    
    int callbackCamId = streamToLogicalCamId(uvcId, info.streamIndex);
    if (callbackCamId < 0 || callbackCamId >= LOGICAL_CAM_NUM) {
        SDK_LOG_ERROR("[SDK] Invalid frame mapping: uvc=%d stream=%lu\n", uvcId, info.streamIndex);
        return;
    }

    unsigned int v4l2Fmt = 0;
    if (info.format == PixelFormat::MJPEG) v4l2Fmt = 0x47504A4D;  // 'MJPG'
    else if (info.format == PixelFormat::GREY) v4l2Fmt = 0x59455247;  // 'GREY'
    else if (info.format == PixelFormat::Z16) v4l2Fmt = 0x36315A;  // 'Z16 '
    else if (info.format == PixelFormat::Y8I) v4l2Fmt = 0x49385956;  // 'Y8I '
    else if (info.format == PixelFormat::YUYV) v4l2Fmt = 0x32595559;  // 'YUYV'
    else if (info.format == PixelFormat::RGB8) v4l2Fmt = 0x42475200;  // 'RGB8'
    else if (info.format == PixelFormat::NV12) v4l2Fmt = 0x3231564E;   // 'NV12'

    g_ctx.imgCb(callbackCamId, data, size, info.width, info.height, 
                v4l2Fmt, info.timestamp, g_ctx.imgUser);
}

static void videoThreadFunc(int uvcId) {
    HRESULT hrCo = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    if (FAILED(hrCo) && hrCo != S_FALSE) return;

    int reconnect_fails = 0;

    auto stillWanted = [&]() -> bool {
        if (!g_ctx.running) return false;
        if (uvcId == RGB_UVC_ID) return g_ctx.cam_running[RGB_CAM_ID];
        return g_ctx.cam_running[GRAY_CAM_ID] || g_ctx.cam_running[DEPTH_CAM_ID];
    };

    while (stillWanted()) {
        // ---- 设备没打开 / 上次断开了：尝试（重新）打开 ----
        if (!g_ctx.videos[uvcId]) {
            if (!openVideoSourceForSlot(uvcId)) {
                reconnect_backoff_apply("VIDEO", uvcId, &reconnect_fails, "open failed");
                continue;
            }
            reconnect_fails = 0;
            SDK_LOG_INFO("[SDK] Video device %d (re)connected\n", uvcId);

            // The KS/XU handle belongs to the old USB device instance.
            // Rebind it after RGB UVC is recreated.
            if (uvcId == RGB_UVC_ID) {
                std::lock_guard<std::recursive_mutex> lock(g_ctx.xuMutex);
                if (!reopenXUControlLocked("RGB video reconnect")) {
                    SDK_LOG_INFO("[XU][WARN] RGB video recovered but XU is not available yet\n");
                }
            }
        }

        // ---- 正常读帧循环（跟原来一样，只是把"检测到设备断开"的处理方式从
        //      "线程退出"改成"清理掉 source、break 回外层重连循环"）----
        MFVideoSource* src = g_ctx.videos[uvcId];
        while (stillWanted() && src->isRunning()) {
            uint8_t* data = nullptr;
            size_t size = 0;
            FrameInfo info{};

            if (uvcId == RGB_UVC_ID) {
                if (src->readFrame(0, data, size, info)) {
                    deliverFrame(uvcId, data, size, info);
                    delete[] data;
                } else if (!src->isRunning()) {
                    break;   // readFrame 内部检测到设备已失效（running_=false），跳出内层循环去重连
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            } else {
                DWORD actualStream = MF_SOURCE_READER_ANY_STREAM;
                if (src->readFrame(MF_SOURCE_READER_ANY_STREAM, data, size, info, &actualStream)) {
                    if ((actualStream == DEPTH_STREAM_ID && g_ctx.cam_running[DEPTH_CAM_ID]) ||
                        (actualStream == GRAY_STREAM_ID && g_ctx.cam_running[GRAY_CAM_ID])) {
                        deliverFrame(uvcId, data, size, info);
                    }
                    delete[] data;
                } else if (!src->isRunning()) {
                    break;
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
        }

        // ---- 走到这里说明内层循环退出了：要么是用户主动停止（stillWanted()==false，
        //      外层 while 也会退出），要么是设备掉线（src->isRunning()==false，需要清掉重连）----
        if (g_ctx.videos[uvcId]) {
            delete g_ctx.videos[uvcId];
            g_ctx.videos[uvcId] = nullptr;
        }
    }

    CoUninitialize();
    SDK_LOG_INFO("[SDK] Video thread exited: UVC=%d\n", uvcId);
}

static bool hidPathStillPresent(const std::string& path) {
    if (path.empty()) return false;
    auto paths = findHidDevices(VENDOR_ID, PRODUCT_ID);
    for (const auto& p : paths) {
        if (p == path) return true;
    }
    return false;
}

static bool reopenHidDeviceForThread(int idx, int& failCount) {
    if (idx < 0 || idx >= HID_NUM) return false;

    // Always enumerate again.  This is important because the path can change after
    // USB device-instance re-enumeration even when the interface number stays the same.
    auto paths = findHidDevices(VENDOR_ID, PRODUCT_ID);
    if (paths.empty()) {
        ++failCount;
        int delay = reconnect_backoff_ms(failCount);
        SDK_LOG_ERROR("[HID%d][ERR] No HID device found, retry in %dms (attempt %d)",
                      idx, delay, failCount);
        interruptible_sleep_ms(delay);
        return false;
    }

    std::string path;

    // Prefer the old path when it still exists; otherwise fall back to a stable
    // enumeration slot.  For this device there is normally one HID interface used
    // by the VIO payload.
    for (const auto& p : paths) {
        if (p == g_ctx.hidPaths[idx]) {
            path = p;
            break;
        }
    }
    if (path.empty()) {
        const size_t wanted = static_cast<size_t>((std::max)(idx, 0));
        path = paths[(std::min)(wanted, paths.size() - 1)];
    }

    if (!g_ctx.hidDevs[idx]) {
        g_ctx.hidDevs[idx] = new (std::nothrow) HidDevice();
        if (!g_ctx.hidDevs[idx]) return false;
    }

    g_ctx.hidDevs[idx]->close();
    if (!g_ctx.hidDevs[idx]->open(path)) {
        ++failCount;
        int delay = reconnect_backoff_ms(failCount);
        SDK_LOG_ERROR("[HID%d][ERR] Failed to open %s, retry in %dms (attempt %d)",
                      idx, path.c_str(), delay, failCount);
        interruptible_sleep_ms(delay);
        return false;
    }

    g_ctx.hidPaths[idx] = path;
    failCount = 0;
    SDK_LOG_INFO("[HID%d] Reconnected: %s", idx, path.c_str());
    return true;
}

static void hidThreadFunc(int idx) {
    if (idx < 0 || idx >= HID_NUM) return;

    constexpr ULONGLONG HID_NO_DATA_TIMEOUT_MS = 4000;
    constexpr ULONGLONG HID_ENUM_CHECK_MS = 500;

    int reconnectFails = 0;
    uint64_t last_vio_ts = 0;
    ULONGLONG lastDataTick = 0;
    ULONGLONG lastEnumCheck = 0;
    uint8_t buf[64];

    while (g_ctx.running) {
        if (!g_ctx.hidDevs[idx] || g_ctx.hidPaths[idx].empty()) {
            if (!reopenHidDeviceForThread(idx, reconnectFails)) continue;
            lastDataTick = GetTickCount64();
            lastEnumCheck = lastDataTick;
            last_vio_ts = 0;
        }

        const ULONGLONG now = GetTickCount64();

        // Do not rely solely on hid_read() returning an error.  Some HID backends keep
        // a stale handle alive after the USB instance disappears.  Periodically check
        // whether that exact path is still enumerated by Windows.
        if (now - lastEnumCheck >= HID_ENUM_CHECK_MS) {
            lastEnumCheck = now;
            if (!hidPathStillPresent(g_ctx.hidPaths[idx])) {
                SDK_LOG_WARN("[HID%d][WARN] HID path disappeared from enumeration, reopening...", idx);
                g_ctx.hidDevs[idx]->close();
                g_ctx.hidPaths[idx].clear();
                last_vio_ts = 0;
                lastDataTick = now;
                continue;
            }
        }

        bool disconnected = false;
        size_t len = sizeof(buf);
        HidDevice* dev = g_ctx.hidDevs[idx];
        if (!dev || !dev->read(buf, len, &disconnected)) {
            if (disconnected) {
                SDK_LOG_WARN("[HID%d][WARN] hid_read reports device error, reopening...", idx);
                if (dev) dev->close();
                g_ctx.hidPaths[idx].clear();
                last_vio_ts = 0;
                continue;
            }

            if (lastDataTick == 0) lastDataTick = now;
            if (now - lastDataTick > HID_NO_DATA_TIMEOUT_MS) {
                SDK_LOG_WARN("[HID%d][WARN] HID has produced no data for %llums, reopening...",
                             idx,
                             static_cast<unsigned long long>(now - lastDataTick));
                if (dev) dev->close();
                g_ctx.hidPaths[idx].clear();
                last_vio_ts = 0;
                continue;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        lastDataTick = now;
        if (len >= sizeof(vio_hid_payload)) {
            auto* p = reinterpret_cast<vio_hid_payload*>(buf);
            if (p->timestamp != 0 && p->timestamp == last_vio_ts) continue;
            last_vio_ts = p->timestamp;
            if (g_ctx.vioCb) {
                g_ctx.vioCb(p->px, p->py, p->pz, p->qx, p->qy, p->qz, p->qw,
                            p->timestamp, g_ctx.vioUser);
            }
        }
    }
}

// ==================== SDK API ====================
int insight9_receive_init(const insight9_config_t* config) {
    if (g_ctx.initialized) {
        SDK_LOG_ERROR("[SDK] Already initialized");
        return -1;
    }

    if (!config) {
        SDK_LOG_ERROR("[SDK] Config is NULL, using default");
        return insight9_receive_init_default();
    }

    if (config->rgb_config.width <= 0 || config->rgb_config.height <= 0 ||
        config->gray_config.width <= 0 || config->gray_config.height <= 0 ||
        config->depth_config.width <= 0 || config->depth_config.height <= 0) {
        SDK_LOG_ERROR("[SDK] Invalid resolution in config");
        return -1;
    }

    // sdk_ctx_t contains std::thread/std::mutex/std::string; never memset it.
    g_ctx.config = {};
    g_ctx.running = false;
    g_ctx.initialized = false;
    g_ctx.singleNodeMode = false;
    g_ctx.xu = nullptr;
    g_ctx.device_caps = {};
    for (int i = 0; i < UVC_NUM; ++i) {
        g_ctx.videos[i] = nullptr;
        g_ctx.videoPaths[i].clear();
        g_ctx.videoThreadStuck[i] = false;
    }
    for (int i = 0; i < LOGICAL_CAM_NUM; ++i) {
        g_ctx.cam_running[i] = false;
        g_ctx.last_img_timestamp[i] = 0;
    }
    for (int i = 0; i < HID_NUM; ++i) {
        g_ctx.hidDevs[i] = nullptr;
        g_ctx.hidPaths[i].clear();
        g_ctx.hidThreadStuck[i] = false;
    }
    g_ctx.imgCb = nullptr;
    g_ctx.imgUser = nullptr;
    g_ctx.imuCb = nullptr;
    g_ctx.imuUser = nullptr;
    g_ctx.vioCb = nullptr;
    g_ctx.vioUser = nullptr;
    g_ctx.running = false;
    for (int i = 0; i < UVC_NUM; ++i) {
        g_ctx.videos[i] = nullptr;
    }
    for (int i = 0; i < LOGICAL_CAM_NUM; ++i) {
        g_ctx.cam_running[i] = false;
        g_ctx.last_img_timestamp[i] = 0;
    }
    g_ctx.config = *config;

    hid_init();

    auto uvcPaths = findUvcDevices(VENDOR_ID, PRODUCT_ID);
    if (uvcPaths.size() < 2) {
        SDK_LOG_ERROR("[SDK] Need at least 2 UVC devices, found %zu", uvcPaths.size());
        return -1;
    }
    
    HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    if (FAILED(hr) && hr != S_FALSE) {
        SDK_LOG_ERROR("[SDK] CoInitializeEx failed, hr=0x%08lx", hr);
        CoUninitialize();
        return -1;
    }

    for (int i = 0; i < UVC_NUM; ++i) {
        g_ctx.videoPaths[i] = uvcPaths[i];
        g_ctx.videos[i] = new MFVideoSource();
        bool opened = false;
        if (i == RGB_UVC_ID) {
            opened = g_ctx.videos[i]->open(uvcPaths[i], i,
                        config->rgb_config.width, config->rgb_config.height,
                        config->rgb_config.pixel_format, config->rgb_config.fps);
        } else if (i == COMPOSITE_UVC_ID) {
            int depthW = config->depth_config.width, depthH = config->depth_config.height;
            int grayW  = config->gray_config.width,  grayH  = config->gray_config.height;
            if (depthW != grayW || depthH != grayH) {
                SDK_LOG_WARN("[SDK] Warning: composite streams size mismatch, using Gray size for both.");
                depthW = grayW; depthH = grayH;
            }
            opened = g_ctx.videos[i]->openWithTwoStreams(uvcPaths[i], i,
                        depthW, depthH, config->depth_config.pixel_format, config->depth_config.fps,
                        config->gray_config.pixel_format, config->gray_config.fps);
        }
        if (!opened) {
            SDK_LOG_ERROR("[SDK] Failed to open video device %d: %s", i, uvcPaths[i].c_str());
            return -1;
        }
    }
    
    auto hidPaths = findHidDevices(VENDOR_ID, PRODUCT_ID);
    if (hidPaths.size() < 1) {
        SDK_LOG_ERROR("[SDK] VIO device not found (found %zu HID devices)", hidPaths.size());
        return -1;
    }
    g_ctx.hidPaths[0] = hidPaths[0];
    // g_ctx.hidPaths[1] = hidPaths[1];
    // SDK_LOG_INFO("[HID] IMU device (interface %d): %s", 0, g_ctx.hidPaths[0].c_str());
    SDK_LOG_INFO("[HID] VIO device (interface %d): %s", 0, g_ctx.hidPaths[0].c_str());
    g_ctx.hidDevs[0] = new HidDevice();
    if (!g_ctx.hidDevs[0]->open(g_ctx.hidPaths[0])) {
        SDK_LOG_ERROR("[SDK] Failed to open VIO HID");
        return -1;
    }
    // if (!g_ctx.hidDevs[1]->open(g_ctx.hidPaths[1])) {
    //     SDK_LOG_ERROR("[SDK] Failed to open VIO HID");
    //     return -1;
    // }

    {
        std::lock_guard<std::recursive_mutex> lock(g_ctx.xuMutex);
        if (!reopenXUControlLocked("initialization")) {
            SDK_LOG_WARN("[SDK][WARN] XU control unavailable during initialization; video can still run");
        }
    }

    g_ctx.initialized = true;
    SDK_LOG_INFO("[SDK] Initialized successfully with config:");
    SDK_LOG_INFO("  RGB: %dx%d@%d, fourcc=0x%08x", 
           config->rgb_config.width, config->rgb_config.height,
           config->rgb_config.fps, config->rgb_config.pixel_format);
    SDK_LOG_INFO("  Gray: %dx%d@%d, fourcc=0x%08x",
           config->gray_config.width, config->gray_config.height,
           config->gray_config.fps, config->gray_config.pixel_format);
    SDK_LOG_INFO("  Depth: %dx%d@%d, fourcc=0x%08x",
           config->depth_config.width, config->depth_config.height,
           config->depth_config.fps, config->depth_config.pixel_format);
    
    return 0;
}

int insight9_receive_init_default() {
    if (g_ctx.initialized) {
        SDK_LOG_ERROR("[SDK] Already initialized");
        return -1;
    }

    HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    if (FAILED(hr) && hr != S_FALSE) {
        SDK_LOG_ERROR("[SDK] CoInitializeEx failed, hr=0x%08lx", hr);
        CoUninitialize();
        return -1;
    }

    // sdk_ctx_t contains std::thread/std::mutex/std::string; never memset it.
    g_ctx.config = {};
    g_ctx.running = false;
    g_ctx.initialized = false;
    g_ctx.singleNodeMode = false;
    g_ctx.xu = nullptr;
    g_ctx.device_caps = {};
    for (int i = 0; i < UVC_NUM; ++i) {
        g_ctx.videos[i] = nullptr;
        g_ctx.videoPaths[i].clear();
        g_ctx.videoThreadStuck[i] = false;
    }
    for (int i = 0; i < LOGICAL_CAM_NUM; ++i) {
        g_ctx.cam_running[i] = false;
        g_ctx.last_img_timestamp[i] = 0;
    }
    for (int i = 0; i < HID_NUM; ++i) {
        g_ctx.hidDevs[i] = nullptr;
        g_ctx.hidPaths[i].clear();
        g_ctx.hidThreadStuck[i] = false;
    }
    g_ctx.imgCb = nullptr;
    g_ctx.imgUser = nullptr;
    g_ctx.imuCb = nullptr;
    g_ctx.imuUser = nullptr;
    g_ctx.vioCb = nullptr;
    g_ctx.vioUser = nullptr;
    g_ctx.running = false;
    
    for (int i = 0; i < UVC_NUM; ++i) {
        g_ctx.videos[i] = nullptr;
    }
    for (int i = 0; i < LOGICAL_CAM_NUM; ++i) {
        g_ctx.cam_running[i] = false;
        g_ctx.last_img_timestamp[i] = 0;
    }

    hid_init();

    auto uvcPaths = findUvcDevices(VENDOR_ID, PRODUCT_ID);
    if (uvcPaths.size() < 2) {
        SDK_LOG_ERROR("[SDK] Need at least 2 UVC devices, found %zu", uvcPaths.size());
        return -1;
    }

    SDK_LOG_INFO("[SDK] Probing device capabilities...");
    
    if (uvcPaths.size() > 0) {
        probeDeviceCapabilities(uvcPaths[0], 0, g_ctx.device_caps);
    }
    
    if (uvcPaths.size() > 1) {
        probeDeviceCapabilities(uvcPaths[1], 1, g_ctx.device_caps);
    }
    
    if (g_ctx.device_caps.rgb_default.valid) {
        g_ctx.config.rgb_config.width = g_ctx.device_caps.rgb_default.width;
        g_ctx.config.rgb_config.height = g_ctx.device_caps.rgb_default.height;
        g_ctx.config.rgb_config.fps = g_ctx.device_caps.rgb_default.fps;
        g_ctx.config.rgb_config.pixel_format = g_ctx.device_caps.rgb_default.format;
        printf("[SDK] Using probed RGB config: %dx%d@%d\n", 
               g_ctx.config.rgb_config.width, 
               g_ctx.config.rgb_config.height,
               g_ctx.config.rgb_config.fps);
    } else {
        g_ctx.config.rgb_config.width = MAIN_WIDTH;
        g_ctx.config.rgb_config.height = MAIN_HEIGHT;
        g_ctx.config.rgb_config.fps = 30;
        g_ctx.config.rgb_config.pixel_format = MAIN_FORMAT;
        printf("[SDK] Using fallback RGB config: %dx%d@%d\n", 
               g_ctx.config.rgb_config.width, 
               g_ctx.config.rgb_config.height,
               g_ctx.config.rgb_config.fps);
    }
    
    if (g_ctx.device_caps.gray_default.valid) {
        g_ctx.config.gray_config.width = g_ctx.device_caps.gray_default.width;
        g_ctx.config.gray_config.height = g_ctx.device_caps.gray_default.height;
        g_ctx.config.gray_config.fps = g_ctx.device_caps.gray_default.fps;
        g_ctx.config.gray_config.pixel_format = g_ctx.device_caps.gray_default.format;
        printf("[SDK] Using probed Gray config: %dx%d@%d\n", 
               g_ctx.config.gray_config.width, 
               g_ctx.config.gray_config.height,
               g_ctx.config.gray_config.fps);
    } else {
        g_ctx.config.gray_config.width = SUB_WIDTH;
        g_ctx.config.gray_config.height = SUB_HEIGHT;
        g_ctx.config.gray_config.fps = 30;
        g_ctx.config.gray_config.pixel_format = SUB_FORMAT;
        printf("[SDK] Using fallback Gray config: %dx%d@%d\n", 
               g_ctx.config.gray_config.width, 
               g_ctx.config.gray_config.height,
               g_ctx.config.gray_config.fps);
    }
    
    if (g_ctx.device_caps.depth_default.valid) {
        g_ctx.config.depth_config.width = g_ctx.device_caps.depth_default.width;
        g_ctx.config.depth_config.height = g_ctx.device_caps.depth_default.height;
        g_ctx.config.depth_config.fps = g_ctx.device_caps.depth_default.fps;
        g_ctx.config.depth_config.pixel_format = g_ctx.device_caps.depth_default.format;
        printf("[SDK] Using probed Depth config: %dx%d@%d\n", 
            g_ctx.config.depth_config.width, 
            g_ctx.config.depth_config.height,
            g_ctx.config.depth_config.fps);
    } else {
        if (g_ctx.device_caps.gray_default.valid) {
            g_ctx.config.depth_config.width = g_ctx.device_caps.gray_default.width;
            g_ctx.config.depth_config.height = g_ctx.device_caps.gray_default.height;
            g_ctx.config.depth_config.fps = g_ctx.device_caps.gray_default.fps;
            g_ctx.config.depth_config.pixel_format = PixelFormat::Z16;
            printf("[SDK] Using Gray config for Depth fallback: %dx%d@%d\n", 
                g_ctx.config.depth_config.width, 
                g_ctx.config.depth_config.height,
                g_ctx.config.depth_config.fps);
        } else {
            g_ctx.config.depth_config.width = 424;
            g_ctx.config.depth_config.height = 240;
            g_ctx.config.depth_config.fps = 30;
            g_ctx.config.depth_config.pixel_format = PixelFormat::Z16;
            printf("[SDK] Using hardcoded Depth fallback: 424x240@30\n");
        }
    }
    
     for (int i = 0; i < UVC_NUM; ++i) {
        g_ctx.videoPaths[i] = uvcPaths[i];
        g_ctx.videos[i] = new MFVideoSource();
        
        bool opened = false;
        if (i == RGB_UVC_ID) {
            int w = g_ctx.config.rgb_config.width;
            int h = g_ctx.config.rgb_config.height;
            int fps = g_ctx.config.rgb_config.fps;
            PixelFormat fmt = g_ctx.config.rgb_config.pixel_format;

            opened = g_ctx.videos[i]->open(uvcPaths[i], i, w, h, fmt, fps);
        } else if (i == COMPOSITE_UVC_ID) {
            int depthW = g_ctx.config.depth_config.width;
            int depthH = g_ctx.config.depth_config.height;
            int depthFps = g_ctx.config.depth_config.fps;
            PixelFormat depthFmt = g_ctx.config.depth_config.pixel_format;
            int grayW = g_ctx.config.gray_config.width;
            int grayH = g_ctx.config.gray_config.height;
            int grayFps = g_ctx.config.gray_config.fps;
            PixelFormat grayFmt = g_ctx.config.gray_config.pixel_format;
            if (depthW != grayW || depthH != grayH) {
                SDK_LOG_WARN("[SDK] Warning: Composite streams have different configured sizes. Using Gray resolution for both because the current UVC device requires one shared media type size.");

                depthW = grayW;
                depthH = grayH;
            }

            SDK_LOG_INFO("[SDK] Opening Composite UVC %d:"
                "        Stream0 Depth: %dx%d@%d fmt=%d"
                "        Stream1 Gray : %dx%d@%d fmt=%d",
                i, depthW, depthH, depthFps, (int)depthFmt,
                grayW, grayH, grayFps, (int)grayFmt
            );

            opened = g_ctx.videos[i]->openWithTwoStreams(uvcPaths[i], i,
                    depthW, depthH, depthFmt,
                    depthFps, grayFmt, grayFps
                );
        }
        if (!opened) {
            SDK_LOG_ERROR("[SDK] Failed to open physical UVC %d: %s", i, uvcPaths[i].c_str());
            delete g_ctx.videos[i];
            g_ctx.videos[i] = nullptr;
            return -1;
        }
    }

    auto hidPaths = findHidDevices(VENDOR_ID, PRODUCT_ID);
    if (hidPaths.size() < 1) {
        SDK_LOG_ERROR("[SDK] VIO device not found (found %zu HID devices)", hidPaths.size());
        return -1;
    }
    g_ctx.hidPaths[0] = hidPaths[0];
    // g_ctx.hidPaths[1] = hidPaths[1];
    // SDK_LOG_INFO("[HID] IMU device (interface %d): %s", 0, g_ctx.hidPaths[0].c_str());
    SDK_LOG_INFO("[HID] VIO device (interface %d): %s", 0, g_ctx.hidPaths[0].c_str());
    g_ctx.hidDevs[0] = new HidDevice();
    if (!g_ctx.hidDevs[0]->open(g_ctx.hidPaths[0])) {
        SDK_LOG_ERROR("[SDK] Failed to open VIO HID");
        return -1;
    }
    // if (!g_ctx.hidDevs[1]->open(g_ctx.hidPaths[1])) {
    //     SDK_LOG_ERROR("[SDK] Failed to open VIO HID");
    //     return -1;
    // }

    {
        std::lock_guard<std::recursive_mutex> lock(g_ctx.xuMutex);
        if (!reopenXUControlLocked("initialization")) {
            SDK_LOG_WARN("[SDK][WARN] XU control unavailable during initialization; video can still run");
        }
    }

    g_ctx.initialized = true;
    return 0;
}

int insight9_receive_start() {
    if (!g_ctx.initialized || g_ctx.running) return -1;
    g_ctx.running = true;
    g_ctx.cam_running[RGB_CAM_ID] = true;
    g_ctx.cam_running[GRAY_CAM_ID] = true;
    g_ctx.cam_running[DEPTH_CAM_ID] = true;
    
    for (int uvcId = 0; uvcId < UVC_NUM; ++uvcId) {
        if (!g_ctx.videos[uvcId]) {
            continue;
        }

        if (!g_ctx.videos[uvcId]->start()) {
            SDK_LOG_ERROR("[SDK] Failed to start UVC %d", uvcId);
            g_ctx.running = false;
            return -1;
        }

        g_ctx.videoThreads[uvcId] = std::thread(videoThreadFunc, uvcId);
    }
    
    g_ctx.hidThreads[0] = std::thread(imuSensorThreadFunc);
    g_ctx.hidThreads[1] = std::thread(hidThreadFunc, 0);

    return 0;
}

static int mapCompositeCamId(int cam_id);

int insight9_receive_start_camera(int cam_id) {
    if (!g_ctx.initialized) return -1;
    int mapped = mapCompositeCamId(cam_id);
    if (mapped < 0 || mapped >= UVC_NUM) return -1;
    if (g_ctx.cam_running[mapped]) return 0;
    
    if (!g_ctx.videos[mapped]) {
        auto uvcPaths = findUvcDevices(VENDOR_ID, PRODUCT_ID);
        if (mapped >= (int)uvcPaths.size()) return -1;
        
        g_ctx.videoPaths[mapped] = uvcPaths[mapped];
        g_ctx.videos[mapped] = new MFVideoSource();
        
        if (cam_id == 0) {
            int w = g_ctx.config.rgb_config.width;
            int h = g_ctx.config.rgb_config.height;
            PixelFormat fmt = g_ctx.config.rgb_config.pixel_format;
            int fps = g_ctx.config.rgb_config.fps;
            
            if (!g_ctx.videos[mapped]->open(uvcPaths[mapped], mapped, w, h, fmt, fps)) {
                delete g_ctx.videos[mapped];
                g_ctx.videos[mapped] = nullptr;
                return -1;
            }
        } else {
            int w = g_ctx.config.gray_config.width;
            int h = g_ctx.config.gray_config.height;
            PixelFormat depthFmt = g_ctx.config.depth_config.pixel_format;
            PixelFormat grayFmt = g_ctx.config.gray_config.pixel_format;
            int fps = g_ctx.config.gray_config.fps;
            
            if (!g_ctx.videos[mapped]->openWithTwoStreams(uvcPaths[mapped], mapped, w, h, depthFmt, fps, grayFmt, fps)) {
                delete g_ctx.videos[mapped];
                g_ctx.videos[mapped] = nullptr;
                return -1;
            }
        }
    }
    
    g_ctx.cam_running[mapped] = true;
    g_ctx.videos[mapped]->start();
    
    joinWithTimeout(g_ctx.videoThreads[mapped], 1000, "video");
    g_ctx.videoThreads[mapped] = std::thread(videoThreadFunc, mapped);
    
    return 0;
}

void insight9_receive_stop_camera(int cam_id) {
    if (cam_id < 0 || cam_id >= LOGICAL_CAM_NUM) {
        return;
    }
    SDK_LOG_INFO("[SDK] Stop logical camera %d", cam_id);

    if (cam_id == RGB_CAM_ID) {
        g_ctx.cam_running[RGB_CAM_ID] = false;
        if (g_ctx.videos[RGB_UVC_ID]) {
            g_ctx.videos[RGB_UVC_ID]->stop();
        }
        if (g_ctx.videoThreads[RGB_UVC_ID].joinable()) {
            g_ctx.videoThreads[RGB_UVC_ID].join();
        }
        return;
    }

    if (cam_id == GRAY_CAM_ID) {
        g_ctx.cam_running[GRAY_CAM_ID] = false;
    } else if (cam_id == DEPTH_CAM_ID) {
        g_ctx.cam_running[DEPTH_CAM_ID] = false;
    }

    if (!g_ctx.cam_running[GRAY_CAM_ID] && !g_ctx.cam_running[DEPTH_CAM_ID]) {
        if (g_ctx.videos[COMPOSITE_UVC_ID]) g_ctx.videos[COMPOSITE_UVC_ID]->stop();
        g_ctx.videoThreadStuck[COMPOSITE_UVC_ID] =
            !joinWithTimeout(g_ctx.videoThreads[COMPOSITE_UVC_ID], 1000, "video[Composite]");

        if (g_ctx.videoThreadStuck[COMPOSITE_UVC_ID]) {
            SDK_LOG_WARN("[SDK][WARN] Composite video source leaked intentionally");
        } else {
            delete g_ctx.videos[COMPOSITE_UVC_ID];
        }
        g_ctx.videos[COMPOSITE_UVC_ID] = nullptr;
        g_ctx.videoThreadStuck[COMPOSITE_UVC_ID] = false;

        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        g_ctx.videos[COMPOSITE_UVC_ID] = new MFVideoSource();
    }
}

int insight9_receive_restart_camera(int cam_id) {
    if (!g_ctx.initialized) {
        return -1;
    }

    if (cam_id < 0 || cam_id >= LOGICAL_CAM_NUM) {
        return -1;
    }

    int uvcId = mapCompositeCamId(cam_id);

    if (uvcId < 0 || uvcId >= UVC_NUM) {
        return -1;
    }

    SDK_LOG_INFO("[SDK] Restart logical camera %d (physical UVC %d)", cam_id, uvcId);

    if (cam_id == RGB_CAM_ID) {
        insight9_receive_stop_camera(RGB_CAM_ID);

        if (g_ctx.videoThreadStuck[RGB_UVC_ID]) {
            SDK_LOG_WARN("[SDK][WARN] RGB video source leaked intentionally "
                         "(stuck thread may still reference it)");
        } else {
            delete g_ctx.videos[RGB_UVC_ID];
        }

        g_ctx.videos[RGB_UVC_ID] = nullptr;
        g_ctx.videoThreadStuck[RGB_UVC_ID] = false;

        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        g_ctx.videos[RGB_UVC_ID] = new MFVideoSource();

        if (!g_ctx.videos[RGB_UVC_ID]->open(g_ctx.videoPaths[RGB_UVC_ID],
                    RGB_UVC_ID,
                    g_ctx.config.rgb_config.width,
                    g_ctx.config.rgb_config.height,
                    g_ctx.config.rgb_config.pixel_format,
                    g_ctx.config.rgb_config.fps)) {
            delete g_ctx.videos[RGB_UVC_ID];
            g_ctx.videos[RGB_UVC_ID] = nullptr;
            return -1;
        }

        g_ctx.cam_running[RGB_CAM_ID] = true;
        g_ctx.videos[RGB_UVC_ID]->start();
        g_ctx.videoThreads[RGB_UVC_ID] = std::thread(videoThreadFunc, RGB_UVC_ID);

        return 0;
    }

    g_ctx.cam_running[GRAY_CAM_ID] = false;
    g_ctx.cam_running[DEPTH_CAM_ID] = false;

    if (g_ctx.videos[COMPOSITE_UVC_ID]) {
        g_ctx.videos[COMPOSITE_UVC_ID]->stop();
    }

    if (g_ctx.videoThreads[COMPOSITE_UVC_ID].joinable()) {
        g_ctx.videoThreads[COMPOSITE_UVC_ID].join();
    }

    delete g_ctx.videos[COMPOSITE_UVC_ID];
    g_ctx.videos[COMPOSITE_UVC_ID] = nullptr;

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    g_ctx.videos[COMPOSITE_UVC_ID] = new MFVideoSource();

    int w = g_ctx.config.gray_config.width;
    int h = g_ctx.config.gray_config.height;
    int fps = g_ctx.config.gray_config.fps;

    PixelFormat depthFmt = g_ctx.config.depth_config.pixel_format;
    PixelFormat grayFmt = g_ctx.config.gray_config.pixel_format;

    if (!g_ctx.videos[COMPOSITE_UVC_ID]->openWithTwoStreams(g_ctx.videoPaths[COMPOSITE_UVC_ID],
                COMPOSITE_UVC_ID, w, h, depthFmt, fps, grayFmt, fps)) {
        delete g_ctx.videos[COMPOSITE_UVC_ID];
        g_ctx.videos[COMPOSITE_UVC_ID] = nullptr;

        return -1;
    }

    g_ctx.cam_running[GRAY_CAM_ID] = true;
    g_ctx.cam_running[DEPTH_CAM_ID] = true;
    g_ctx.videos[COMPOSITE_UVC_ID]->start();

    g_ctx.videoThreads[COMPOSITE_UVC_ID] = std::thread(videoThreadFunc, COMPOSITE_UVC_ID);

    return 0;
}

int insight9_receive_switch_camera_fps(int cam_id, int fps) {
    if (!g_ctx.initialized || cam_id < 0 || cam_id >= UVC_NUM || fps <= 0) return -1;

    SDK_LOG_INFO("[SDK] Switching camera %d to %d FPS...", cam_id, fps);

    insight9_receive_stop_camera(cam_id);
    Sleep(3000);

    if (insight9_receive_set_camera_fps(cam_id, fps) != 0) {
        SDK_LOG_ERROR("[SDK] Failed to set camera %d FPS to %d", cam_id, fps);
        return -1;
    }

    int ret = insight9_receive_restart_camera(cam_id);
    if (ret == 0) {
        SDK_LOG_INFO("[SDK] Camera %d switched to %d FPS successfully", cam_id, fps);
    } else {
        SDK_LOG_ERROR("[SDK] Failed to restart camera %d after FPS switch", cam_id);
    }
    return ret;
}

int insight9_receive_is_camera_running(int cam_id) {
    int mapped = mapCompositeCamId(cam_id);
    if (mapped < 0 || mapped >= UVC_NUM) return 0;
    return g_ctx.cam_running[mapped] ? 1 : 0;
}

void insight9_receive_stop() {
    if (!g_ctx.running) return;
    g_ctx.running = false;

    for (int i = 0; i < LOGICAL_CAM_NUM; ++i) g_ctx.cam_running[i] = false;
    for (int i = 0; i < UVC_NUM; ++i) {
        if (g_ctx.videos[i]) g_ctx.videos[i]->stop();
    }
    for (int i = 0; i < UVC_NUM; ++i) {
        char label[32];
        snprintf(label, sizeof(label), "video[%d]", i);
        g_ctx.videoThreadStuck[i] = !joinWithTimeout(g_ctx.videoThreads[i], 1000, label);
    }
    for (int i = 0; i < HID_NUM; ++i) {
        char label[32];
        snprintf(label, sizeof(label), "hid[%d]", i);
        g_ctx.hidThreadStuck[i] = !joinWithTimeout(g_ctx.hidThreads[i], 1000, label);
    }
}

void insight9_receive_cleanup() {
    insight9_receive_stop();

    for (int i = 0; i < UVC_NUM; ++i) {
        if (g_ctx.videoThreadStuck[i]) {
            SDK_LOG_WARN("[SDK][WARN] Video source leaked intentionally "
                         "(stuck thread may still reference it)");
        } else {
            delete g_ctx.videos[i];
        }
        g_ctx.videos[i] = nullptr;
    }

    for (int i = 0; i < HID_NUM; ++i) {
        if (g_ctx.hidDevs[i]) {
            if (g_ctx.hidThreadStuck[i]) {
                SDK_LOG_WARN("[SDK][WARN] HID device %d leaked intentionally "
                             "(stuck thread may still reference it)", i);
            } else {
                g_ctx.hidDevs[i]->close();
                delete g_ctx.hidDevs[i];
            }
            g_ctx.hidDevs[i] = nullptr;
        }
    }

    {
        std::lock_guard<std::recursive_mutex> lock(g_ctx.xuMutex);
        delete g_ctx.xu;
        g_ctx.xu = nullptr;
    }
    g_ctx.initialized = false;
    g_ctx.running = false;

    for (int i = 0; i < LOGICAL_CAM_NUM; ++i) {
        g_ctx.cam_running[i] = false;
    }

    hid_exit();

    CoUninitialize();
}

// ==================== The remaining API implementations ====================
static int mapCompositeCamId(int cam_id) {
    if (cam_id == RGB_CAM_ID) {
        return RGB_UVC_ID;
    }
    if (cam_id == GRAY_CAM_ID || cam_id == DEPTH_CAM_ID) {
        return COMPOSITE_UVC_ID;
    }
    return -1;
}

static int mapCamIdToStreamIndex(int cam_id) {
    if (cam_id == RGB_CAM_ID)   return 0;
    if (cam_id == DEPTH_CAM_ID) return DEPTH_STREAM_ID;  // 0
    if (cam_id == GRAY_CAM_ID)  return GRAY_STREAM_ID;   // 1
    return -1;
}

const char* insight9_receive_get_video_dev(int cam_id) {
    int mapped = mapCompositeCamId(cam_id);
    return (mapped >= 0 && mapped < UVC_NUM) ? g_ctx.videoPaths[mapped].c_str() : nullptr;
}

int insight9_receive_set_camera_fps(int cam_id, int fps) {
    if (!g_ctx.initialized || cam_id < 0 || cam_id > 2) return -1;
    if (cam_id == 0) {
        g_ctx.config.rgb_config.fps = fps;
        SDK_LOG_INFO("[SDK] Set RGB FPS to %d", fps);
    } else {
        g_ctx.config.gray_config.fps = fps;
        SDK_LOG_INFO("[SDK] Set Gray/Depth composite FPS to %d", fps);
    }
    return 0;
}

void insight9_receive_register_image_callback(image_callback cb, void *user) { 
    g_ctx.imgCb = cb; 
    g_ctx.imgUser = user; 
}

void insight9_receive_register_imu_callback(imu_callback cb, void *user) { 
    g_ctx.imuCb = cb; 
    g_ctx.imuUser = user; 
}

void insight9_receive_register_vio_callback(vio_callback cb, void *user) { 
    g_ctx.vioCb = cb; 
    g_ctx.vioUser = user; 
}

const DeviceCapabilities_t* insight9_receive_get_device_capabilities_ptr(void) {
    if (!g_ctx.initialized) {
        SDK_LOG_ERROR("[SDK] Not initialized");
        return nullptr;
    }
    return &g_ctx.device_caps;
}

int insight9_receive_get_device_default_capability(int cam_id, DeviceCapability* cap) {
    if (!g_ctx.initialized) {
        SDK_LOG_ERROR("[SDK] Not initialized");
        return -1;
    }
    
    if (!cap) {
        SDK_LOG_ERROR("[SDK] Invalid cap parameter");
        return -1;
    }
    
    const DeviceCapability* target = nullptr;
    switch (cam_id) {
        case 0: target = &g_ctx.device_caps.rgb_default; break;
        case 1: target = &g_ctx.device_caps.gray_default; break;
        case 2: target = &g_ctx.device_caps.depth_default; break;
        default:
            SDK_LOG_ERROR("[SDK] Invalid cam_id: %d (must be 0, 1, or 2)", cam_id);
            return -1;
    }
    
    if (!target->valid) {
        SDK_LOG_ERROR("[SDK] No default capability found for cam_id %d", cam_id);
        return -1;
    }
    
    memcpy(cap, target, sizeof(DeviceCapability));
    return 0;
}

int insight9_receive_get_device_capability_count(int cam_id, int* count) {
    if (!g_ctx.initialized) {
        SDK_LOG_ERROR("[SDK] Not initialized");
        return -1;
    }
    
    if (!count) {
        SDK_LOG_ERROR("[SDK] Invalid count parameter");
        return -1;
    }
    
    const std::vector<DeviceCapability>* vec = nullptr;
    switch (cam_id) {
        case 0: vec = &g_ctx.device_caps.rgb_capabilities; break;
        case 1: vec = &g_ctx.device_caps.gray_capabilities; break;
        case 2: vec = &g_ctx.device_caps.depth_capabilities; break;
        default:
            SDK_LOG_ERROR("[SDK] Invalid cam_id: %d (must be 0, 1, or 2)", cam_id);
            return -1;
    }
    
    *count = (int)vec->size();
    return 0;
}

int insight9_receive_get_device_capability_by_index(int cam_id, int index, DeviceCapability* cap) {
    if (!g_ctx.initialized) {
        SDK_LOG_ERROR("[SDK] Not initialized");
        return -1;
    }
    
    if (!cap) {
        SDK_LOG_ERROR("[SDK] Invalid cap parameter");
        return -1;
    }
    
    if (index < 0) {
        SDK_LOG_ERROR("[SDK] Invalid index: %d (must be >= 0)", index);
        return -1;
    }
    
    const std::vector<DeviceCapability>* vec = nullptr;
    switch (cam_id) {
        case 0: vec = &g_ctx.device_caps.rgb_capabilities; break;
        case 1: vec = &g_ctx.device_caps.gray_capabilities; break;
        case 2: vec = &g_ctx.device_caps.depth_capabilities; break;
        default:
            SDK_LOG_ERROR("[SDK] Invalid cam_id: %d (must be 0, 1, or 2)", cam_id);
            return -1;
    }
    
    if (index >= (int)vec->size()) {
        SDK_LOG_ERROR("[SDK] Index %d out of range (size: %zu)", index, vec->size());
        return -1;
    }
    
    memcpy(cap, &(*vec)[index], sizeof(DeviceCapability));
    return 0;
}

int insight9_receive_read_metadata_timestamp(int cam_id, uint64_t* timestamp) {
    if (!timestamp) return -1;

    int uvcId = mapCompositeCamId(cam_id);
    int streamIdx = mapCamIdToStreamIndex(cam_id);
    if (uvcId < 0 || uvcId >= UVC_NUM || streamIdx < 0 || !g_ctx.videos[uvcId]) {
        return -1;
    }

    uint64_t ts = g_ctx.videos[uvcId]->getLastTimestamp(streamIdx);
    if (ts == 0) {
        return -1;
    }

    *timestamp = ts;
    return 0;
}

int insight9_receive_set_active_camera(int cam_id) {
    return callXUWithRetry("setActiveCamera", [cam_id](viewer::ExtensionUnitControl& xu) {
        return xu.setActiveCamera((uint8_t)cam_id);
    }) ? 0 : -1;
}

int insight9_receive_get_active_camera(int *cam_id) {
    if (!cam_id) return -1;
    uint8_t val = 0;
    if (!callXUWithRetry("getActiveCamera", [&val](viewer::ExtensionUnitControl& xu) {
            return xu.getActiveCamera(val);
        })) return -1;
    *cam_id = val;
    return 0;
}

int insight9_receive_set_camera_params(const camera_params *params) {
    if (!params) return -1;
    viewer::camera_params xuParams;
    memcpy(&xuParams, params, sizeof(viewer::camera_params));
    return callXUWithRetry("writeCurrentCameraParams", [xuParams](viewer::ExtensionUnitControl& xu) {
        return xu.writeCurrentCameraParams(xuParams);
    }) ? 0 : -1;
}

int insight9_receive_get_camera_params(camera_params *params) {
    if (!params) return -1;
    viewer::camera_params xuParams;
    if (!callXUWithRetry("readCurrentCameraParams", [&xuParams](viewer::ExtensionUnitControl& xu) {
            return xu.readCurrentCameraParams(xuParams);
        })) return -1;
    memcpy(params, &xuParams, sizeof(camera_params));
    return 0;
}

int insight9_receive_set_camera_params_for(int cam_id, const camera_params *params) {
    if (!params) return -1;
    int mapped = mapCompositeCamId(cam_id);
    if (mapped < 0) return -1;
    viewer::camera_params xuParams;
    memcpy(&xuParams, params, sizeof(viewer::camera_params));
    return callXUWithRetry("writeCameraParams", [mapped, xuParams](viewer::ExtensionUnitControl& xu) {
        return xu.writeCameraParams((uint8_t)mapped, xuParams);
    }) ? 0 : -1;
}

int insight9_receive_get_camera_params_for(int cam_id, camera_params *params) {
    if (!params) return -1;
    int mapped = mapCompositeCamId(cam_id);
    if (mapped < 0) return -1;
    viewer::camera_params xuParams;
    if (!callXUWithRetry("readCameraParams", [mapped, &xuParams](viewer::ExtensionUnitControl& xu) {
            return xu.readCameraParams((uint8_t)mapped, xuParams);
        })) return -1;
    memcpy(params, &xuParams, sizeof(camera_params));
    return 0;
}

int insight9_receive_reset_camera_params(int) { 
   // Reading factory defaults from the device requires additional implementation. A simple approach is to read
   // and save the current values during initialization, then write them back when a reset is needed.
   SDK_LOG_ERROR("[XU][ERR] reset_camera_params not implemented, use set_camera_params with saved defaults");
   return -1;
}

void insight9_receive_print_camera_params(const camera_params *params) {
    if (!params) return;
    viewer::camera_params xu_params;
    memcpy(&xu_params, params, sizeof(viewer::camera_params));
    viewer::printParams(xu_params);
}

int insight9_receive_get_camera_calib(int cam_idx, camera_calib *calib) {
    if (!calib) return -1;
    if (cam_idx < 0 || cam_idx >= viewer::kCalibCamCount) return -1;

    viewer::camera_calib xu_calib;
    if (!callXUWithRetry("readCameraCalib", [cam_idx, &xu_calib](viewer::ExtensionUnitControl& xu) {
            return xu.readCameraCalib(static_cast<uint8_t>(cam_idx), xu_calib);
        })) return -1;
    memcpy(calib, &xu_calib, sizeof(camera_calib));
    return 0;
}

void insight9_receive_print_camera_calib(const camera_calib *calib) {
    if (!calib) return;
    viewer::camera_calib xu_calib;
    memcpy(&xu_calib, calib, sizeof(viewer::camera_calib));
    viewer::printCalib(xu_calib);
}

// ==================== Depth Alignment Functions ====================

// Rasterize one triangle into the aligned-depth output, interpolating the RGB-
// frame depth across it with a nearest-surface z-buffer. Vertices are given as
// projected RGB pixel coords (x,y, float) plus their RGB-frame depth z (mm).
static inline void align_raster_triangle(uint16_t *out, int W, int H,
                                         float x0, float y0, float z0,
                                         float x1, float y1, float z1,
                                         float x2, float y2, float z2) {
    int minx = (int)floorf(fminf(x0, fminf(x1, x2)));
    int maxx = (int)ceilf(fmaxf(x0, fmaxf(x1, x2)));
    int miny = (int)floorf(fminf(y0, fminf(y1, y2)));
    int maxy = (int)ceilf(fmaxf(y0, fmaxf(y1, y2)));
    if (minx < 0) minx = 0;
    if (miny < 0) miny = 0;
    if (maxx > W - 1) maxx = W - 1;
    if (maxy > H - 1) maxy = H - 1;
    if (minx > maxx || miny > maxy) return;

    float area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
    if (fabsf(area) < 1e-6f) return;          // degenerate
    float inv_area = 1.0f / area;

    for (int py = miny; py <= maxy; py++) {
        for (int px = minx; px <= maxx; px++) {
            float fx = px + 0.5f, fy = py + 0.5f;
            // Barycentric weights via edge cross-products (same winding as area).
            float w0 = (x1 - fx) * (y2 - fy) - (x2 - fx) * (y1 - fy);
            float w1 = (x2 - fx) * (y0 - fy) - (x0 - fx) * (y2 - fy);
            float w2 = (x0 - fx) * (y1 - fy) - (x1 - fx) * (y0 - fy);
            bool inside = (w0 >= 0 && w1 >= 0 && w2 >= 0) ||
                          (w0 <= 0 && w1 <= 0 && w2 <= 0);
            if (!inside) continue;

            float z = (w0 * z0 + w1 * z1 + w2 * z2) * inv_area;
            if (z <= 0.0f) continue;
            uint16_t zq = (z > 65535.0f) ? 65535 : (uint16_t)(z + 0.5f);
            uint16_t *dst = &out[(size_t)py * W + px];
            if (*dst == 0 || zq < *dst) *dst = zq;
        }
    }
}

int insight9_receive_align_depth_to_rgb(const uint16_t *depth,
                                        int depth_w, int depth_h,
                                        const camera_calib *left_calib,
                                        const camera_calib *rgb_calib,
                                        uint16_t *aligned_out,
                                        int rgb_w, int rgb_h) {
    if (!depth || !left_calib || !rgb_calib || !aligned_out ||
        depth_w <= 0 || depth_h <= 0 || rgb_w <= 0 || rgb_h <= 0) {
        return -1;
    }

    // Start with all-invalid (0) output.
    memset(aligned_out, 0, (size_t)rgb_w * rgb_h * sizeof(uint16_t));

    // Intrinsics (row-major k = [fx 0 cx; 0 fy cy; 0 0 1]). Streams are
    // rectified (p == k), so no distortion is applied.
    const double dfx = left_calib->intrinsics.k[0];
    const double dfy = left_calib->intrinsics.k[4];
    const double dcx = left_calib->intrinsics.k[2];
    const double dcy = left_calib->intrinsics.k[5];
    const double rfx = rgb_calib->intrinsics.k[0];
    const double rfy = rgb_calib->intrinsics.k[4];
    const double rcx = rgb_calib->intrinsics.k[2];
    const double rcy = rgb_calib->intrinsics.k[5];
    if (dfx == 0.0 || dfy == 0.0) return -1;

    // RGB extrinsic: parent=camera_camera_left, child=camera_camera_rgb. In ROS
    // convention (t, q) maps a point rgb->left: p_left = R*p_rgb + t. We need
    // left->rgb, i.e. the inverse: p_rgb = R^T * (p_left - t).
    const double *q = rgb_calib->extrinsics.rotation;     // x, y, z, w
    const double *t = rgb_calib->extrinsics.translation;  // meters
    const double qx = q[0], qy = q[1], qz = q[2], qw = q[3];
    const double R00 = 1 - 2*(qy*qy + qz*qz);
    const double R01 = 2*(qx*qy - qz*qw);
    const double R02 = 2*(qx*qz + qy*qw);
    const double R10 = 2*(qx*qy + qz*qw);
    const double R11 = 1 - 2*(qx*qx + qz*qz);
    const double R12 = 2*(qy*qz - qx*qw);
    const double R20 = 2*(qx*qz - qy*qw);
    const double R21 = 2*(qy*qz + qx*qw);
    const double R22 = 1 - 2*(qx*qx + qy*qy);
    const double tx = t[0] * 1000.0, ty = t[1] * 1000.0, tz = t[2] * 1000.0;

    // Pass 1: project every depth pixel to RGB pixel coords + RGB-frame depth.
    // The depth grid is treated as a mesh; these are its vertices.
    const size_t n = (size_t)depth_w * depth_h;
    std::vector<float>   VX(n), VY(n), VZ(n);
    std::vector<uint8_t> OK(n, 0);
    for (int v = 0; v < depth_h; v++) {
        const uint16_t *row = depth + (size_t)v * depth_w;
        for (int u = 0; u < depth_w; u++) {
            size_t i = (size_t)v * depth_w + u;
            uint16_t d = row[u];
            if (d == 0) continue;              // no measurement

            // Deproject to the LEFT frame (mm), then left -> rgb: R^T*(p - t).
            double z = (double)d;
            double x = (u - dcx) * z / dfx;
            double y = (v - dcy) * z / dfy;
            double ax = x - tx, ay = y - ty, az = z - tz;
            double xr = R00*ax + R10*ay + R20*az;
            double yr = R01*ax + R11*ay + R21*az;
            double zr = R02*ax + R12*ay + R22*az;
            if (zr <= 0.0) continue;           // behind the RGB camera

            VX[i] = (float)(rfx * xr / zr + rcx);
            VY[i] = (float)(rfy * yr / zr + rcy);
            VZ[i] = (float)zr;
            OK[i] = 1;
        }
    }

    // Pass 2: rasterize the mesh. Each 2x2 cell -> two triangles. Triangles that
    // straddle a depth discontinuity (an object edge) are dropped to avoid
    // "rubber sheet" stretching between foreground and background.
    const float EDGE_REL = 0.05f;              // max 5% depth jump within a triangle
    auto emit = [&](size_t a, size_t b, size_t c) {
        if (!OK[a] || !OK[b] || !OK[c]) return;
        float za = VZ[a], zb = VZ[b], zc = VZ[c];
        float zmin = fminf(za, fminf(zb, zc));
        float zmax = fmaxf(za, fmaxf(zb, zc));
        if (zmin <= 0.0f || (zmax - zmin) > EDGE_REL * zmin) return;
        align_raster_triangle(aligned_out, rgb_w, rgb_h,
                              VX[a], VY[a], za,
                              VX[b], VY[b], zb,
                              VX[c], VY[c], zc);
    };
    for (int v = 0; v + 1 < depth_h; v++) {
        for (int u = 0; u + 1 < depth_w; u++) {
            size_t i00 = (size_t)v * depth_w + u;
            size_t i10 = i00 + 1;
            size_t i01 = i00 + depth_w;
            size_t i11 = i01 + 1;
            emit(i00, i10, i11);
            emit(i00, i11, i01);
        }
    }
    return 0;
}

int insight9_receive_get_current_fps(int* fps) {
    if (!fps) return -1;
    uint8_t val = 0;
    if (!callXUWithRetry("readCurrentFps", [&val](viewer::ExtensionUnitControl& xu) {
            return xu.readCurrentFps(val);
        })) return -1;
    const int validFps[] = {0, 20, 30, 40, 50};
    if (val < (uint8_t)(sizeof(validFps) / sizeof(validFps[0]))) {
        *fps = validFps[val];
    } else {
        *fps = 0;
    }
    return 0;
}

int insight9_receive_get_vio_status(int* status) {
    if (!status) return -1;
    uint8_t val = 0;
    if (!callXUWithRetry("readVioStatus", [&val](viewer::ExtensionUnitControl& xu) {
            return xu.readVioStatus(val);
        })) return -1;
    *status = val;
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