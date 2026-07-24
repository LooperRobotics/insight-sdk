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
#include <dshow.h>
#include "MetadataReader.hpp"

// ==================== Media Foundation 头文件 ====================
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

// ==================== 定义缺失的 Media Foundation GUID ====================
extern "C" {
// Y800 - 8-bit grayscale
DEFINE_GUID(MEDIASUBTYPE_Y800, 
    0x30303859, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);

// Z16 - 16-bit depth
DEFINE_GUID(MEDIASUBTYPE_Z16, 
    0x2036315A, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);

// 其他可能需要的 GUID
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

// ==================== Target Device VID/PID ====================
#define VENDOR_ID  0x8086
#define PRODUCT_ID 0x0b5c

// ==================== Camera Configuration ====================
#define CAM_NUM 2
#define HID_NUM 2
#define MAIN_WIDTH   848
#define MAIN_HEIGHT  480
#define MAIN_FORMAT  PixelFormat::YUYV
#define SUB_WIDTH    848
#define SUB_HEIGHT   480
#define SUB_FORMAT   PixelFormat::Z16
#define DEPTH_WIDTH  848
#define DEPTH_HEIGHT 480
#define DEPTH_FORMAT PixelFormat::Z16

#ifndef NOMINMAX
#define NOMINMAX
#endif

class HidDevice;

// ==================== MFVideoSource 内联实现 ====================
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

class MFVideoSource {
public:
    MFVideoSource() {
        HRESULT hr = MFStartup(MF_VERSION);
        if (FAILED(hr)) {
            printf("[MFVideoSource] MFStartup failed: 0x%08lx\n", hr);
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
                printf("RGB stream configure failed.\n");
                return false;
            }
        }
        //------------------------------------------------
        // Depth Camera
        //------------------------------------------------
        else
        {
            if (!configureStream(0, fmt, width, height, fps)) {
                printf("Depth stream configure failed.\n");
                return false;
            }

            // 灰度/IR 流 (stream 1) - 使用 Y8I 格式，同样的分辨率和帧率
            // 注意：这里我们使用 Y8I 作为灰度的格式
            PixelFormat grayFmt = PixelFormat::Y8I;
            if (!configureStream(1, grayFmt, width, height, fps)) {
                // 如果 Y8I 不支持，尝试 GREY
                grayFmt = PixelFormat::GREY;
                if (!configureStream(1, grayFmt, width, height, fps)) {
                    printf("IR stream configure failed.\n");
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

        // 配置流 0
        if (!configureStream(0, fmt0, width, height, fps0)) {
            printf("Stream 0 configure failed.\n");
            return false;
        }

        // 配置流 1
        if (!configureStream(1, fmt1, width, height, fps1)) {
            printf("Stream 1 configure failed.\n");
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

    bool readFrame(DWORD streamIndex, uint8_t*& data, size_t& size, FrameInfo& info) {
        data = nullptr;
        size = 0;

        if (!running_) return false;

        if (!streamEnabled_[streamIndex]) return false;

        IMFSample* sample = nullptr;

        DWORD actualStream = 0;
        DWORD flags = 0;

        LONGLONG ts = 0;

        HRESULT hr =
            sourceReader_->ReadSample(streamIndex, 0, &actualStream, &flags, &ts, &sample);

        if (FAILED(hr)) return false;

        //---------------------------------------
        // 没数据
        //---------------------------------------

        if (flags & (MF_SOURCE_READERF_STREAMTICK | MF_SOURCE_READERF_ENDOFSTREAM)) {
            if(sample) sample->Release();
            return false;
        }

        if(sample==nullptr) return false;

        IMFMediaBuffer* buffer=nullptr;

        hr=sample->ConvertToContiguousBuffer(&buffer);

        if(FAILED(hr)) {
            sample->Release();
            return false;
        }

        BYTE* ptr=nullptr;

        DWORD maxLen=0;
        DWORD curLen=0;

        hr=buffer->Lock(&ptr, &maxLen, &curLen);

        if(FAILED(hr)) {
            buffer->Release();
            sample->Release();
            return false;
        }

        data=new uint8_t[curLen];
        memcpy(data, ptr, curLen);

        size=curLen;

        buffer->Unlock();
        buffer->Release();
        sample->Release();

        //---------------------------------------
        // timestamp 去重
        //---------------------------------------

        if(lastTimestamp_[actualStream]==(uint64_t)ts) {
            delete[] data;
            data=nullptr;
            return false;
        }

        lastTimestamp_[actualStream]=(uint64_t)ts;

        //---------------------------------------
        // FrameInfo
        //---------------------------------------

        info.timestamp=(uint64_t)ts;

        info.right_timestamp=0;

        info.streamIndex=actualStream;

        info.width=streamWidth_[actualStream];

        info.height=streamHeight_[actualStream];

        info.format=streamFormat_[actualStream];

        switch(streamFormat_[actualStream])
        {
            case PixelFormat::MJPEG:
                info.type=FrameType::Color;
                break;
            case PixelFormat::Z16:
                info.type=FrameType::Depth;
                break;
            case PixelFormat::Y8I:
                info.type=FrameType::Gray;
                break;
            default:
                info.type=FrameType::Unknown;
                break;
        }

        return true;
    }

    bool getDefaultCapability(int streamIndex, DeviceCapability& cap) {
        if (!sourceReader_) return false;

        // 获取第一个可用的媒体类型作为默认
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

        // 转换 GUID 到 PixelFormat
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

    // 新增：获取设备支持的所有能力
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

private:
    PixelFormat guidToPixelFormat(const GUID& guid) {
        if (IsEqualGUID(guid, MFVideoFormat_MJPG)) return PixelFormat::MJPEG;
        if (IsEqualGUID(guid, MEDIASUBTYPE_Z16)) return PixelFormat::Z16;
        if (IsEqualGUID(guid, MEDIASUBTYPE_Y800)) return PixelFormat::GREY;
        if (IsEqualGUID(guid, MFVideoFormat_Y8I)) return PixelFormat::Y8I;
        if (IsEqualGUID(guid, MFVideoFormat_YUY2)) return PixelFormat::YUYV;
        if (IsEqualGUID(guid, MFVideoFormat_RGB24)) return PixelFormat::RGB8;
        return PixelFormat::Unknown;
    }

    bool configureStream(DWORD streamIndex, PixelFormat fmt, int width, int height, int fps)
    {
        if (!sourceReader_)
            return false;

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
            // printf("[Stream%u] [%02u] %ux%u %u/%u %s\n", streamIndex, index, w, h, num, den, fourcc);

            double rate = den ? (double)num / den : 0.0;
            if (IsEqualGUID(subtype, targetSubtype) && w == (UINT32)width && h == (UINT32)height && fabs(rate - fps) < 0.1) {
                bestType = type;
                break;
            }

            type->Release();
            ++index;
        }

        if(bestType==nullptr) {
            printf(
                "Cannot find media type for stream %u\n",
                streamIndex);

            return false;
        }

        HRESULT hr = sourceReader_->SetCurrentMediaType(streamIndex, nullptr, bestType);

        bestType->Release();

        if(FAILED(hr)) {
            printf(
                "SetCurrentMediaType stream %u failed 0x%08lx\n",
                streamIndex,
                hr);

            return false;
        }

        sourceReader_->Flush(streamIndex);

        streamEnabled_[streamIndex]=true;
        streamFormat_[streamIndex]=fmt;
        streamWidth_[streamIndex]=width;
        streamHeight_[streamIndex]=height;

        // printf("Stream %u configured.\n", streamIndex);

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
                        
                        // ======  这里是修改的核心：DevicePath 就是 MF 所需的 SymbolicLink ======
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
            printf("[Probe] CreatePresentationDescriptor failed: 0x%08lx\n", hr);
            return false;
        }

        DWORD streamCount = 0;
        pd->GetStreamDescriptorCount(&streamCount);
        printf("[Probe] Total stream count on this media source: %lu\n", streamCount);

        for (DWORD i = 0; i < streamCount; ++i) {
            BOOL selected = FALSE;
            IMFStreamDescriptor* sd = nullptr;
            if (SUCCEEDED(pd->GetStreamDescriptorByIndex(i, &selected, &sd)) && sd) {
                DWORD streamId = 0;
                sd->GetStreamIdentifier(&streamId);
                // printf("[Probe] stream[%lu] id=%lu selected=%d\n", i, streamId, selected);

                IMFMediaTypeHandler* handler = nullptr;
                if (SUCCEEDED(sd->GetMediaTypeHandler(&handler)) && handler) {
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
                            // printf("    [type %lu] %ux%u %u/%u subtype=%s\n", t, w, h, num, den, fourcc);
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
            printf("[MFVideoSource] Failed to get symbolic link\n");
            return false;
        }
        
        std::wstring wlink(symbolicLink.begin(), symbolicLink.end());
        
        IMFAttributes* pAttributes = nullptr;
        HRESULT hr = MFCreateAttributes(&pAttributes, 2);
        if (FAILED(hr)) {
            printf("[MFVideoSource] MFCreateAttributes failed: 0x%08lx\n", hr);
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
            printf("[MFVideoSource] MFCreateDeviceSource failed: 0x%08lx\n", hr);
            return false;
        }
        
        mediaSource_ = pSource;
        probeStreamCount();
        return true;
    }
    
    bool configureSourceReader()
    {
        IMFAttributes* attr = nullptr;

        MFCreateAttributes(&attr,2);

        attr->SetUINT32(
            MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS,
            TRUE);

        attr->SetUINT32(
            MF_SOURCE_READER_ENABLE_ADVANCED_VIDEO_PROCESSING,
            FALSE);

        HRESULT hr =
            MFCreateSourceReaderFromMediaSource(
                mediaSource_,
                attr,
                &sourceReader_);

        if(attr)
            attr->Release();

        if(FAILED(hr))
        {
            printf("CreateSourceReader failed.\n");
            return false;
        }

        //------------------------------------------
        // disable all stream
        //------------------------------------------

        sourceReader_->SetStreamSelection(
            MF_SOURCE_READER_ALL_STREAMS,
            FALSE);

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
        // printf("stream0=%d\n",sel);

        sourceReader_->GetStreamSelection(1,&sel);
        // printf("stream1=%d\n",sel);

        return true;
    }
    
    bool findVideoStream() {
        IMFMediaType* pType = nullptr;
        HRESULT hr = sourceReader_->GetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, &pType);
        if (FAILED(hr) || !pType) {
            printf("[MFVideoSource] GetCurrentMediaType failed: 0x%08lx\n", hr);
            return false;
        }
        
        GUID majorType;
        pType->GetGUID(MF_MT_MAJOR_TYPE, &majorType);
        pType->Release();
        
        if (majorType != MFMediaType_Video) {
            printf("[MFVideoSource] Not a video stream\n");
            return false;
        }
        
        return true;
    }
    
    void parseFrameMetadata(uint8_t* data, size_t& size, FrameInfo& info) {
        if (!data || size == 0) return;
        size_t origSize = size;
        
        if (format_ == PixelFormat::GREY) {
            size_t expectedGrey = static_cast<size_t>(width_) * height_;
            if (origSize >= expectedGrey + 9) {
                size_t metaOffset = expectedGrey;
                memcpy(&info.timestamp, data + metaOffset, 8);
                uint8_t frameType = data[metaOffset + 8];
                info.type = (frameType == 0) ? FrameType::Gray : FrameType::Depth;
                if (info.type == FrameType::Depth) {
                    info.format = PixelFormat::Z16;
                }
                if (origSize >= expectedGrey + 17) {
                    memcpy(&info.right_timestamp, data + metaOffset + 9, 8);
                }
                size = expectedGrey;
            } else if (origSize == expectedGrey) {
                info.type = FrameType::Gray;
            }
        } else if (format_ == PixelFormat::Z16) {
            size_t expectedDepth = static_cast<size_t>(width_) * height_ * 2;
            size_t expectedGrey = static_cast<size_t>(width_) * height_;
            if (origSize >= expectedDepth + 9) {
                size_t metaOffset = expectedDepth;
                memcpy(&info.timestamp, data + metaOffset, 8);
                uint8_t frameType = data[metaOffset + 8];
                info.type = (frameType == 0) ? FrameType::Gray : FrameType::Depth;
                if (info.type == FrameType::Gray) {
                    info.format = PixelFormat::GREY;
                    size = expectedGrey;
                } else {
                    size = expectedDepth;
                }
                if (origSize >= expectedDepth + 17) {
                    memcpy(&info.right_timestamp, data + metaOffset + 9, 8);
                }
            } else if (origSize == expectedDepth) {
                info.type = FrameType::Depth;
            } else if (origSize == expectedGrey) {
                info.type = FrameType::Gray;
                info.format = PixelFormat::GREY;
            } else if (origSize >= expectedGrey + 9) {
                size_t metaOffset = expectedGrey;
                memcpy(&info.timestamp, data + metaOffset, 8);
                uint8_t frameType = data[metaOffset + 8];
                info.type = (frameType == 0) ? FrameType::Gray : FrameType::Depth;
                if (info.type == FrameType::Depth) {
                    info.format = PixelFormat::Z16;
                } else {
                    info.format = PixelFormat::GREY;
                }
                size = expectedGrey;
                if (origSize >= expectedGrey + 17) {
                    memcpy(&info.right_timestamp, data + metaOffset + 9, 8);
                }
            }
        } else if (format_ == PixelFormat::MJPEG) {
            parseMJPEGMetadata(data, size, info);
        }
    }
    
    void parseMJPEGMetadata(uint8_t* data, size_t size, FrameInfo& info) {
        size_t pos = 2;
        while (pos + 4 < size) {
            if (data[pos] == 0xFF && data[pos+1] == 0xE1) {
                uint16_t len = (data[pos+2] << 8) | data[pos+3];
                if (len >= 12 && pos + 4 + len <= size) {
                    if (data[pos+4] == 'F' && data[pos+5] == 'T' &&
                        data[pos+6] == '_' && data[pos+7] == '_') {
                        info.type = static_cast<FrameType>(data[pos+8]);
                        memcpy(&info.timestamp, data + pos + 9, 8);
                        break;
                    }
                }
                break;
            }
            pos++;
        }
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
    uint64_t lastTimestamp_[2] = {0,0};
    DWORD streamCount_ = 0;
    bool streamEnabled_[2] = {false,false};

    PixelFormat streamFormat_[2] = {
        PixelFormat::Unknown,
        PixelFormat::Unknown
    };

    int streamWidth_[2] = {0,0};

    int streamHeight_[2] = {0,0};
};

// ==================== 全局上下文 ====================
struct sdk_ctx_t {
    insight9_config_t config;
    MFVideoSource* videos[CAM_NUM];
    HidDevice* hidDevs[HID_NUM];
    viewer::ExtensionUnitControl *xu;
    MetadataReader* metadataReaders[CAM_NUM];
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
    uint64_t last_img_timestamp[3] = {0};
    DeviceCapabilities_t device_caps;
} g_ctx;

// ==================== 辅助函数 ====================
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
                            printf("[SDK] Matched device: %s -> FriendlyName: %s\n", 
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

static bool probeDeviceCapabilities(const std::string& devicePath, int deviceIndex, 
                                     DeviceCapabilities_t& caps) {
    MFVideoSource* probeSource = new MFVideoSource();
    
    bool opened = false;
    
    if (deviceIndex == 0) {
        // RGB: 尝试 640x480 YUYV 30fps
        opened = probeSource->open(devicePath, deviceIndex, 640, 480, PixelFormat::YUYV, 30);
        if (!opened) {
            opened = probeSource->open(devicePath, deviceIndex, 424, 240, PixelFormat::YUYV, 90);
        }
    } else {
        // Depth: 先尝试用 Z16 打开（流0是深度）
        opened = probeSource->open(devicePath, deviceIndex, 640, 480, PixelFormat::Z16, 30);
        if (!opened) {
            opened = probeSource->open(devicePath, deviceIndex, 424, 240, PixelFormat::Z16, 90);
        }
        if (!opened) {
            opened = probeSource->open(devicePath, deviceIndex, 256, 144, PixelFormat::Z16, 90);
        }
    }
    
    if (!opened) {
        printf("[Probe] Failed to open device\n");
        delete probeSource;
        return false;
    }
    
    // 获取 RGB 设备的所有能力
    if (deviceIndex == 0) {
        std::vector<DeviceCapability> allCaps;
        if (probeSource->getAllCapabilities(0, allCaps)) {
            caps.rgb_capabilities = allCaps;
            if (!allCaps.empty()) {
                caps.rgb_default = allCaps[0];
            }
        }
    } 
    // 获取 Depth 设备的能力（包含灰度和深度流）
    else {
        // 流 0: 深度 (Z16)
        std::vector<DeviceCapability> depthCaps;
        if (probeSource->getAllCapabilities(0, depthCaps)) {
            caps.depth_capabilities = depthCaps;
            if (!depthCaps.empty()) {
                caps.depth_default = depthCaps[0];
            }
        }
        
        // 流 1: 灰度 (Y8I 或 GREY)
        std::vector<DeviceCapability> grayCaps;
        if (probeSource->getAllCapabilities(1, grayCaps)) {
            caps.gray_capabilities = grayCaps;
            if (!grayCaps.empty()) {
                caps.gray_default = grayCaps[0];
            }
        }
    }
    
    caps.initialized = true;
    probeSource->close();
    delete probeSource;
    return true;
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

// 辅助函数：分发帧
static void deliverFrame(int camId, uint8_t* data, size_t size, const FrameInfo& info) {
    if (!g_ctx.imgCb) return;

    unsigned int v4l2Fmt = 0;
    if (info.format == PixelFormat::MJPEG) v4l2Fmt = 0x47504A4D;  // 'MJPG'
    else if (info.format == PixelFormat::GREY) v4l2Fmt = 0x59455247;  // 'GREY'
    else if (info.format == PixelFormat::Z16) v4l2Fmt = 0x36315A;  // 'Z16 '
    else if (info.format == PixelFormat::Y8I) v4l2Fmt = 0x49385956;  // 'Y8I '
    else if (info.format == PixelFormat::YUYV) v4l2Fmt = 0x32595559;  // 'YUYV'
    else if (info.format == PixelFormat::RGB8) v4l2Fmt = 0x42475200;  // 'RGB8'
    
    int callbackCamId = camId;
    if (camId == 1)
    {
        callbackCamId = (info.streamIndex == 0) ? 1 : 2;
    }
    g_ctx.imgCb(callbackCamId, data, size, info.width, info.height, 
                v4l2Fmt, info.timestamp, info.right_timestamp, g_ctx.imgUser);
}

// 视频线程入口
static void videoThreadFunc(int camId) {
    auto* src = g_ctx.videos[camId];
    if (!src) return;
    
    while (g_ctx.cam_running[camId] && src->isRunning()) {
        uint8_t* data = nullptr;
        size_t size = 0;
        FrameInfo info;
        
        if (camId == 0)
        {
            // RGB只有一路
            if (src->readFrame(0, data, size, info))
            {
                deliverFrame(0, data, size, info);
                delete[] data;
            }
        }
        else
        {
            // Depth
            if (src->readFrame(0, data, size, info))
            {
                deliverFrame(1, data, size, info);
                delete[] data;
            }

            data = nullptr;
            size = 0;

            // IR
            if (src->readFrame(1, data, size, info))
            {
                deliverFrame(2, data, size, info);
                delete[] data;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// HID 线程入口
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

// ==================== SDK API 实现 ====================
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

    hid_init();

    auto uvcPaths = findUvcDevices(VENDOR_ID, PRODUCT_ID);
    if (uvcPaths.size() < 2) {
        fprintf(stderr, "[SDK] Need at least 2 UVC devices, found %zu\n", uvcPaths.size());
        return -1;
    }
    
    HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    if (FAILED(hr) && hr != S_FALSE) {
        fprintf(stderr, "[SDK] CoInitializeEx failed, hr=0x%08lx\n", hr);
        CoUninitialize();
        return -1;
    }

    for (int i = 0; i < CAM_NUM; ++i) {
        g_ctx.videoPaths[i] = uvcPaths[i];
        g_ctx.videos[i] = new MFVideoSource();
        int w = (i==0)? config->rgb_config.width : config->gray_config.width;
        int h = (i==0)? config->rgb_config.height : config->gray_config.height;
        PixelFormat fmt = (i==0)? config->rgb_config.pixel_format : config->gray_config.pixel_format;
        int fps = (i==0)? config->rgb_config.fps : config->gray_config.fps;
        if (!g_ctx.videos[i]->open(uvcPaths[i], i, w, h, fmt, fps)) {
            fprintf(stderr, "[SDK] Failed to open video device %d: %s\n", i, uvcPaths[i].c_str());
            return -1;
        }
        g_ctx.metadataReaders[i] = new MetadataReader();
        if (!g_ctx.metadataReaders[i]->open(uvcPaths[i])) {
            fprintf(stderr, "[SDK] Failed to open metadata reader for camera %d\n", i);
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
        delete g_ctx.xu;
        g_ctx.xu = nullptr;
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
        CoUninitialize();
        return -1;
    }

    memset(&g_ctx, 0, sizeof(g_ctx));
    for (int i = 0; i < CAM_NUM; ++i) {
        g_ctx.cam_running[i] = false;
    }

    // g_ctx.config.rgb_config.width = MAIN_WIDTH;
    // g_ctx.config.rgb_config.height = MAIN_HEIGHT;
    // g_ctx.config.rgb_config.fps = 30;
    // g_ctx.config.rgb_config.pixel_format = MAIN_FORMAT;
    
    // g_ctx.config.gray_config.width = SUB_WIDTH;
    // g_ctx.config.gray_config.height = SUB_HEIGHT;
    // g_ctx.config.gray_config.fps = 30;
    // g_ctx.config.gray_config.pixel_format = SUB_FORMAT;
    
    // g_ctx.config.depth_config.width = DEPTH_WIDTH;
    // g_ctx.config.depth_config.height = DEPTH_HEIGHT;
    // g_ctx.config.depth_config.fps = 15;
    // g_ctx.config.depth_config.pixel_format = DEPTH_FORMAT;

    hid_init();

    auto uvcPaths = findUvcDevices(VENDOR_ID, PRODUCT_ID);
    if (uvcPaths.size() < 2) {
        fprintf(stderr, "[SDK] Need at least 2 UVC devices, found %zu\n", uvcPaths.size());
        return -1;
    }

    printf("[SDK] Probing device capabilities...\n");
    
    // 探测 RGB 设备
    if (uvcPaths.size() > 0) {
        probeDeviceCapabilities(uvcPaths[0], 0, g_ctx.device_caps);
    }
    
    // 探测 Depth/Gray 设备
    if (uvcPaths.size() > 1) {
        probeDeviceCapabilities(uvcPaths[1], 1, g_ctx.device_caps);
    }
    
    // ====== 使用探测到的默认参数配置 ======
    // RGB 配置 - 使用探测到的默认值，如果没有则使用备选值
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
        // 备选配置
        g_ctx.config.rgb_config.width = MAIN_WIDTH;
        g_ctx.config.rgb_config.height = MAIN_HEIGHT;
        g_ctx.config.rgb_config.fps = 30;
        g_ctx.config.rgb_config.pixel_format = MAIN_FORMAT;
        printf("[SDK] Using fallback RGB config: %dx%d@%d\n", 
               g_ctx.config.rgb_config.width, 
               g_ctx.config.rgb_config.height,
               g_ctx.config.rgb_config.fps);
    }
    
    // Gray 配置
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
    
    // Depth 配置
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
        // 尝试使用 Gray 的配置作为 Depth 的 fallback
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
    
     for (int i = 0; i < CAM_NUM; ++i) {
        g_ctx.videoPaths[i] = uvcPaths[i];
        g_ctx.videos[i] = new MFVideoSource();
        
        // int w, h, fps;
        // PixelFormat fmt;
        if (i == 0) {
            int w = g_ctx.config.rgb_config.width;
            int h = g_ctx.config.rgb_config.height;
            PixelFormat fmt = g_ctx.config.rgb_config.pixel_format;
            int fps = g_ctx.config.rgb_config.fps;

            if (!g_ctx.videos[i]->open(uvcPaths[i], i, w, h, fmt, fps)) {
                fprintf(stderr, "[SDK] Failed to open RGB device: %s\n", uvcPaths[i].c_str());
                return -1;
            }
        } else {
            int w = g_ctx.config.gray_config.width;
            int h = g_ctx.config.gray_config.height;
            PixelFormat depthFmt = g_ctx.config.depth_config.pixel_format;
            PixelFormat grayFmt = g_ctx.config.gray_config.pixel_format;
            int fps = g_ctx.config.gray_config.fps;

            if (!g_ctx.videos[i]->openWithTwoStreams(uvcPaths[i], i, w, h, depthFmt, fps, grayFmt, fps)) {
                fprintf(stderr, "[SDK] Failed to open Depth/IR device: %s\n", uvcPaths[i].c_str());
                return -1;
            }
        }
        
        // if (!g_ctx.videos[i]->open(uvcPaths[i], i, w, h, fmt, fps)) {
        //     fprintf(stderr, "[SDK] Failed to open video device %d: %s\n", i, uvcPaths[i].c_str());
        //     return -1;
        // }
        
        g_ctx.metadataReaders[i] = new MetadataReader();
        if (!g_ctx.metadataReaders[i]->open(uvcPaths[i])) {
            fprintf(stderr, "[SDK] Failed to open metadata reader for camera %d\n", i);
        }
    }

    g_ctx.xu = new viewer::ExtensionUnitControl();
    if (!g_ctx.xu->open(g_ctx.videoPaths[0])) {
        fprintf(stderr, "[SDK] Failed to open XU control\n");
        delete g_ctx.xu;
        g_ctx.xu = nullptr;
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

static int mapCompositeCamId(int cam_id);

int insight9_receive_start_camera(int cam_id) {
    if (!g_ctx.initialized) return -1;
    int mapped = mapCompositeCamId(cam_id);
    if (mapped < 0 || mapped >= CAM_NUM) return -1;
    if (g_ctx.cam_running[mapped]) return 0;
    
    if (!g_ctx.videos[mapped]) {
        auto uvcPaths = findUvcDevices(VENDOR_ID, PRODUCT_ID);
        if (mapped >= (int)uvcPaths.size()) return -1;
        
        g_ctx.videoPaths[mapped] = uvcPaths[mapped];
        g_ctx.videos[mapped] = new MFVideoSource();
        
        // int w, h;
        // PixelFormat fmt;
        // int fps;
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
        
        // if (!g_ctx.videos[mapped]->open(uvcPaths[mapped], mapped, w, h, fmt, fps)) {
        //     delete g_ctx.videos[mapped];
        //     g_ctx.videos[mapped] = nullptr;
        //     return -1;
        // }
    }
    
    g_ctx.cam_running[mapped] = true;
    g_ctx.videos[mapped]->start();
    
    if (g_ctx.videoThreads[mapped].joinable()) {
        g_ctx.videoThreads[mapped].join();
    }
    g_ctx.videoThreads[mapped] = std::thread(videoThreadFunc, mapped);
    
    return 0;
}

void insight9_receive_stop_camera(int cam_id) {
    int mapped = mapCompositeCamId(cam_id);
    if (mapped < 0 || mapped >= CAM_NUM) return;
    
    g_ctx.cam_running[mapped] = false;
    
    if (g_ctx.videos[mapped]) {
        g_ctx.videos[mapped]->stop();
    }
    
    if (g_ctx.videoThreads[mapped].joinable()) {
        g_ctx.videoThreads[mapped].join();
    }
}

int insight9_receive_restart_camera(int cam_id) {
    if (!g_ctx.initialized) return -1;
    int mapped = mapCompositeCamId(cam_id);
    if (mapped < 0 || mapped >= CAM_NUM) return -1;
    
    printf("[SDK_HACK] Hard restarting camera %d (mapped to %d)...\n", cam_id, mapped);
    
    // 1. 彻底停止并销毁旧的视频源和线程
    insight9_receive_stop_camera(cam_id);
    
    if (g_ctx.videos[cam_id]) {
        delete g_ctx.videos[cam_id];
        g_ctx.videos[cam_id] = nullptr;
    }
    
    // 强制等待 300ms 让系统底层驱动释放设备句柄
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    
    auto uvcPaths = findUvcDevices(VENDOR_ID, PRODUCT_ID);
    if (mapped >= (int)uvcPaths.size()) {
        fprintf(stderr, "[SDK] Camera %d not found during restart\n", cam_id);
        return -1;
    }

    g_ctx.videos[mapped] = new MFVideoSource();

if (mapped == 0) {
    // RGB 摄像头
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
    // Depth 摄像头 - 两个流
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
    
    // 【核心 Hack】：如果是灰度/深度相机(cam_id=1)，在流开启前，利用 XU 强制下发一次对应的帧率索引。
    if (cam_id == 1 && g_ctx.xu) {
        printf("[SDK_HACK] Pre-setting Hardware Framerate via XU before stream start...\n");
        viewer::camera_params xuParams;
        if (g_ctx.xu->readCameraParams(1, xuParams)) {
            // 根据映射，20fps 对应 index=3, 30fps 对应 index=2 (参考 kFramerateMap[] = {90, 60, 30, 20, 15, 10})
            xuParams.frame_rate = (g_ctx.config.gray_config.fps == 30) ? 2 : 3; 
            g_ctx.xu->writeCameraParams(1, xuParams);
            
            // 给硬件 XU 写入留出稳定的时间
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
        }
    }
    
    // 3. 重新启动线程
    g_ctx.cam_running[mapped] = true;
    g_ctx.videos[mapped]->start();
    
    if (g_ctx.videoThreads[mapped].joinable()) {
        g_ctx.videoThreads[mapped].join();
    }
    g_ctx.videoThreads[mapped] = std::thread(videoThreadFunc, mapped);
    
    return 0;
}

int insight9_receive_is_camera_running(int cam_id) {
    int mapped = mapCompositeCamId(cam_id);
    if (mapped < 0 || mapped >= CAM_NUM) return 0;
    return g_ctx.cam_running[mapped] ? 1 : 0;
}

void insight9_receive_all_stop() {
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
    insight9_receive_all_stop();
    for (int i = 0; i < CAM_NUM; ++i) {
        delete g_ctx.videos[i];
        g_ctx.videos[i] = nullptr;
        delete g_ctx.metadataReaders[i];
        g_ctx.metadataReaders[i] = nullptr;
    }
    for (int i = 0; i < HID_NUM; ++i) {
        if (g_ctx.hidDevs[i]) g_ctx.hidDevs[i]->close();
        delete g_ctx.hidDevs[i];
    }
    delete g_ctx.xu;
    g_ctx.initialized = false;
    g_ctx.running = false;
    for (int i = 0; i < CAM_NUM; ++i) {
        g_ctx.cam_running[i] = false;
    }
    hid_exit();
    CoUninitialize();
}

// ==================== 其余 API 实现 ====================
static int mapCompositeCamId(int cam_id) {
    if (cam_id == 0) return 0;
    if (cam_id == 1 || cam_id == 2) return 1;
    return -1;
}

const char* insight9_receive_get_video_dev(int cam_id) {
    int mapped = mapCompositeCamId(cam_id);
    return (mapped >= 0 && mapped < CAM_NUM) ? g_ctx.videoPaths[mapped].c_str() : nullptr;
}

int insight9_receive_set_camera_fps(int cam_id, int fps) {
    if (!g_ctx.initialized || cam_id < 0 || cam_id > 2) return -1;
    if (cam_id == 0) {
        g_ctx.config.rgb_config.fps = fps;
        printf("[SDK] Set RGB FPS to %d\n", fps);
    } else {
        g_ctx.config.gray_config.fps = fps;
        printf("[SDK] Set Gray/Depth composite FPS to %d\n", fps);
    }
    return 0;
}

const char* insight9_receive_get_metadata_dev(int cam_id) {
    int mapped = mapCompositeCamId(cam_id);
    return (mapped >= 0 && mapped < CAM_NUM) ? g_ctx.videoPaths[mapped].c_str() : nullptr;
}

int insight9_receive_read_metadata_timestamp(int cam_id, uint64_t* timestamp) {
    if (!g_ctx.initialized || cam_id < 0 || cam_id > 2 || !timestamp) {
        return -1;
    }
    
    int mapped = mapCompositeCamId(cam_id);
    if (mapped < 0 || mapped >= CAM_NUM) return -1;
    auto* metaReader = g_ctx.metadataReaders[mapped];
    if (!metaReader || !metaReader->isOpen()) {
        return -1;
    }
    
    *timestamp = metaReader->readLatestTimestamp();
    return (*timestamp != 0) ? 0 : -1;
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
    int mapped = mapCompositeCamId(cam_id);
    if (mapped < 0) return -1;
    viewer::camera_params xuParams;
    memcpy(&xuParams, params, sizeof(viewer::camera_params));
    return g_ctx.xu->writeCameraParams((uint8_t)mapped, xuParams) ? 0 : -1;
}

int insight9_receive_get_camera_params_for(int cam_id, camera_params *params) {
    if (!g_ctx.xu || !params) return -1;
    int mapped = mapCompositeCamId(cam_id);
    if (mapped < 0) return -1;
    viewer::camera_params xuParams;
    if (!g_ctx.xu->readCameraParams((uint8_t)mapped, xuParams)) return -1;
    memcpy(params, &xuParams, sizeof(camera_params));
    return 0;
}

int insight9_receive_reset_camera_params(int) { return -1; }

void insight9_receive_print_camera_params(const camera_params *params) {
    if (!params) return;
    viewer::camera_params xu_params;
    memcpy(&xu_params, params, sizeof(viewer::camera_params));
    viewer::printParams(xu_params);
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

const DeviceCapabilities_t* insight9_receive_get_device_capabilities_ptr(void) {
    if (!g_ctx.initialized) {
        fprintf(stderr, "[SDK] Not initialized\n");
        return nullptr;
    }
    return &g_ctx.device_caps;
}

// 获取指定设备的默认能力
int insight9_receive_get_device_default_capability(int cam_id, DeviceCapability* cap) {
    if (!g_ctx.initialized) {
        fprintf(stderr, "[SDK] Not initialized\n");
        return -1;
    }
    
    if (!cap) {
        fprintf(stderr, "[SDK] Invalid cap parameter\n");
        return -1;
    }
    
    const DeviceCapability* target = nullptr;
    switch (cam_id) {
        case 0: target = &g_ctx.device_caps.rgb_default; break;
        case 1: target = &g_ctx.device_caps.gray_default; break;
        case 2: target = &g_ctx.device_caps.depth_default; break;
        default:
            fprintf(stderr, "[SDK] Invalid cam_id: %d (must be 0, 1, or 2)\n", cam_id);
            return -1;
    }
    
    if (!target->valid) {
        fprintf(stderr, "[SDK] No default capability found for cam_id %d\n", cam_id);
        return -1;
    }
    
    memcpy(cap, target, sizeof(DeviceCapability));
    return 0;
}

int insight9_receive_get_device_capability_count(int cam_id, int* count) {
    if (!g_ctx.initialized) {
        fprintf(stderr, "[SDK] Not initialized\n");
        return -1;
    }
    
    if (!count) {
        fprintf(stderr, "[SDK] Invalid count parameter\n");
        return -1;
    }
    
    const std::vector<DeviceCapability>* vec = nullptr;
    switch (cam_id) {
        case 0: vec = &g_ctx.device_caps.rgb_capabilities; break;
        case 1: vec = &g_ctx.device_caps.gray_capabilities; break;
        case 2: vec = &g_ctx.device_caps.depth_capabilities; break;
        default:
            fprintf(stderr, "[SDK] Invalid cam_id: %d (must be 0, 1, or 2)\n", cam_id);
            return -1;
    }
    
    *count = (int)vec->size();
    return 0;
}

int insight9_receive_get_device_capability_by_index(int cam_id, int index, DeviceCapability* cap) {
    if (!g_ctx.initialized) {
        fprintf(stderr, "[SDK] Not initialized\n");
        return -1;
    }
    
    if (!cap) {
        fprintf(stderr, "[SDK] Invalid cap parameter\n");
        return -1;
    }
    
    if (index < 0) {
        fprintf(stderr, "[SDK] Invalid index: %d (must be >= 0)\n", index);
        return -1;
    }
    
    const std::vector<DeviceCapability>* vec = nullptr;
    switch (cam_id) {
        case 0: vec = &g_ctx.device_caps.rgb_capabilities; break;
        case 1: vec = &g_ctx.device_caps.gray_capabilities; break;
        case 2: vec = &g_ctx.device_caps.depth_capabilities; break;
        default:
            fprintf(stderr, "[SDK] Invalid cam_id: %d (must be 0, 1, or 2)\n", cam_id);
            return -1;
    }
    
    if (index >= (int)vec->size()) {
        fprintf(stderr, "[SDK] Index %d out of range (size: %zu)\n", index, vec->size());
        return -1;
    }
    
    memcpy(cap, &(*vec)[index], sizeof(DeviceCapability));
    return 0;
}