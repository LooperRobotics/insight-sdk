#include "ExtensionUnitControl.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <cwctype>
#include <string>
#include <vector>

#include <windows.h>
#include <dshow.h>
#include <ks.h>
#include <ksmedia.h>
#include <ksproxy.h>
#include <vidcap.h>

namespace viewer {
namespace {

constexpr GUID kExtGuid_RealSense1 = {
    0xc9606ccb,
    0x594c,
    0x4d25,
    {0xaf, 0x47, 0xcc, 0xc4, 0x96, 0x43, 0x59, 0x95}
};

constexpr GUID kExtGuid_RealSense3 = {
    0xb8ec416e,
    0xa3ac,
    0x4580,
    {0x8d, 0x5c, 0x0b, 0xee, 0x15, 0x97, 0xe4, 0x3d}
};

constexpr const GUID* kCandidateExtGuids[] = {
    &kExtGuid_RealSense1,
    &kExtGuid_RealSense3,
};

constexpr GUID kIidIKsTopologyInfo = {
    0x720d4ac0,
    0x7533,
    0x11d0,
    {0xa5, 0xd6, 0x28, 0xdb, 0x04, 0xc1, 0x00, 0x00}
};

constexpr GUID kKsNodeTypeDevSpecific = {
    0x941c7ac0,
    0xc559,
    0x11d0,
    {0x8a, 0x2b, 0x00, 0xa0, 0xc9, 0x25, 0x5a, 0xc1}
};

std::wstring utf8ToWide(const std::string& input) {
    if (input.empty()) {
        return {};
    }

    const int size = MultiByteToWideChar(CP_UTF8, 0, input.c_str(), -1, nullptr, 0);
    if (size <= 0) {
        return {};
    }

    std::wstring output(static_cast<size_t>(size), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, input.c_str(), -1, &output[0], size);
    if (!output.empty() && output.back() == L'\0') {
        output.pop_back();
    }
    return output;
}

std::wstring toLower(std::wstring value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](wchar_t ch) { return static_cast<wchar_t>(std::towlower(ch)); });
    return value;
}

void safeRelease(IUnknown*& ptr) {
    if (ptr) {
        ptr->Release();
        ptr = nullptr;
    }
}

}  // namespace

ExtensionUnitControl::ExtensionUnitControl() = default;

ExtensionUnitControl::~ExtensionUnitControl() {
    close();
}

bool ExtensionUnitControl::open(const std::string& devicePath) {
    close();

    if (!bindFilterByDevicePath(devicePath)) {
        return false;
    }

    if (FAILED(filter_->QueryInterface(IID_IKsControl, reinterpret_cast<void**>(&ksControl_)))) {
        printf("[XU] IKsControl is not available for device: %s\n", devicePath.c_str());
        close();
        return false;
    }

    if (!resolveNodeId()) {
        printf("[XU] Failed to resolve extension-unit node for device: %s\n", devicePath.c_str());
        close();
        return false;
    }

    if (!probeExtensionGuid()) {
        printf("[XU] No known extension GUID matched on node=%lu for device: %s\n",
               nodeId_, devicePath.c_str());
        close();
        return false;
    }

    printf("[XU] Bound extension unit GUID to device path: %s (node=%lu, unit=%u, guid=%s)\n",
                devicePath.c_str(), nodeId_, static_cast<unsigned>(kXuUnitId),
                getActiveGuidString().c_str());
    return true;
}

bool ExtensionUnitControl::probeExtensionGuid() {
    for (const GUID* candidate : kCandidateExtGuids) {
        std::memcpy(activeExtGuidBytes_, candidate, sizeof(activeExtGuidBytes_));

        uint8_t camId = 0;
        if (query(kActiveCameraSelector, KSPROPERTY_TYPE_GET, &camId, sizeof(camId))) {
            return true;
        }

        camera_params params{};
        if (query(kCameraParamsSelector, KSPROPERTY_TYPE_GET, &params, sizeof(params))) {
            return true;
        }
    }

    std::memset(activeExtGuidBytes_, 0, sizeof(activeExtGuidBytes_));
    return false;
}

std::string ExtensionUnitControl::getActiveGuidString() const {
    GUID g{};
    std::memcpy(&g, activeExtGuidBytes_, sizeof(g));
    char buf[64];
    std::snprintf(buf, sizeof(buf),
        "{%08lX-%04X-%04X-%02X%02X-%02X%02X%02X%02X%02X%02X}",
        g.Data1, g.Data2, g.Data3,
        g.Data4[0], g.Data4[1], g.Data4[2], g.Data4[3],
        g.Data4[4], g.Data4[5], g.Data4[6], g.Data4[7]);
    return std::string(buf);
}

void ExtensionUnitControl::close() {
    IUnknown* ksUnknown = reinterpret_cast<IUnknown*>(ksControl_);
    IUnknown* filterUnknown = reinterpret_cast<IUnknown*>(filter_);
    safeRelease(ksUnknown);
    safeRelease(filterUnknown);
    ksControl_ = nullptr;
    filter_ = nullptr;
    nodeId_ = 0;
    std::memset(activeExtGuidBytes_, 0, sizeof(activeExtGuidBytes_));
}

bool ExtensionUnitControl::isOpen() const {
    return ksControl_ != nullptr;
}

bool ExtensionUnitControl::getActiveCamera(uint8_t& camId) const {
    camId = 0;
    return query(kActiveCameraSelector, KSPROPERTY_TYPE_GET, &camId, sizeof(camId));
}

bool ExtensionUnitControl::setActiveCamera(uint8_t camId) const {
    return query(kActiveCameraSelector, KSPROPERTY_TYPE_SET, &camId, sizeof(camId));
}

bool ExtensionUnitControl::readCurrentCameraParams(camera_params& params) const {
    std::memset(&params, 0, sizeof(params));
    return query(kCameraParamsSelector, KSPROPERTY_TYPE_GET, &params, sizeof(params));
}

bool ExtensionUnitControl::writeCurrentCameraParams(const camera_params& params) const {
    camera_params copy = params;
    return query(kCameraParamsSelector, KSPROPERTY_TYPE_SET, &copy, sizeof(copy));
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
        if (!query(selector, KSPROPERTY_TYPE_SET, chunk, sizeof(chunk))) {
            fprintf(stderr, "[XU] SET_CUR calib block %u selector 0x%02x failed\n",
                    static_cast<unsigned>(blk), selector);
            return false;
        }
        // GET_CUR to read the block
        if (!query(selector, KSPROPERTY_TYPE_GET, chunk, sizeof(chunk))) {
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
        printf("[XU] Failed to set active camera to %u for reading params\n", static_cast<unsigned>(camId));
        return false;
    }
    printf("[XU] Set active camera to %u for reading params\n", static_cast<unsigned>(camId));
    ::Sleep(50);
    return readCurrentCameraParams(params);
}

bool ExtensionUnitControl::writeCameraParams(uint8_t camId, const camera_params& params) const {
    if (!setActiveCamera(camId)) {
        return false;
    }
    printf("[XU] Set active camera to %u for writing params\n", static_cast<unsigned>(camId));
    ::Sleep(50);
    return writeCurrentCameraParams(params);
}

bool ExtensionUnitControl::readVioStatus(uint8_t& val) const {
    return query(kVioManagerStatus, KSPROPERTY_TYPE_GET, &val, sizeof(val));
}

bool ExtensionUnitControl::readCurrentFps(uint8_t& fpsIndex) const {
    return query(kCurrentFpsSelector, KSPROPERTY_TYPE_GET, &fpsIndex, sizeof(fpsIndex));
}

bool ExtensionUnitControl::bindFilterByDevicePath(const std::string& devicePath) {
    ICreateDevEnum* devEnum = nullptr;
    IEnumMoniker* enumMoniker = nullptr;

    const HRESULT createHr = CoCreateInstance(CLSID_SystemDeviceEnum, nullptr, CLSCTX_INPROC_SERVER,
                                              IID_ICreateDevEnum, reinterpret_cast<void**>(&devEnum));
    if (FAILED(createHr) || !devEnum) {
        printf("[XU] Failed to create system device enumerator, hr=0x%08lx\n",
                    static_cast<unsigned long>(createHr));
        return false;
    }

    const HRESULT enumHr = devEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &enumMoniker, 0);
    if (FAILED(enumHr) || !enumMoniker) {
        printf("[XU] Failed to enumerate video devices, hr=0x%08lx\n",
                    static_cast<unsigned long>(enumHr));
        devEnum->Release();
        return false;
    }

    const std::wstring expected = toLower(utf8ToWide(devicePath));
    IMoniker* moniker = nullptr;

    while (enumMoniker->Next(1, &moniker, nullptr) == S_OK) {
        IPropertyBag* propertyBag = nullptr;
        HRESULT bagHr = moniker->BindToStorage(nullptr, nullptr, IID_IPropertyBag,
                                               reinterpret_cast<void**>(&propertyBag));
        if (SUCCEEDED(bagHr) && propertyBag) {
            VARIANT value;
            VariantInit(&value);
            const HRESULT readHr = propertyBag->Read(L"DevicePath", &value, nullptr);
            if (SUCCEEDED(readHr) && value.vt == VT_BSTR) {
                const std::wstring current = toLower(value.bstrVal ? value.bstrVal : L"");
                if (current == expected) {
                    IBaseFilter* filter = nullptr;
                    const HRESULT bindHr = moniker->BindToObject(nullptr, nullptr, IID_IBaseFilter,
                                                                 reinterpret_cast<void**>(&filter));
                    VariantClear(&value);
                    propertyBag->Release();
                    moniker->Release();
                    enumMoniker->Release();
                    devEnum->Release();
                    if (SUCCEEDED(bindHr) && filter) {
                        filter_ = filter;
                        return true;
                    }

                    printf("[XU] Failed to bind DirectShow filter, hr=0x%08lx\n",
                                static_cast<unsigned long>(bindHr));
                    return false;
                }
            }
            VariantClear(&value);
            propertyBag->Release();
        }
        moniker->Release();
    }

    enumMoniker->Release();
    devEnum->Release();
    // printf("[XU] DevicePath not found in DirectShow enumeration: %s\n", devicePath.c_str());
    return false;
}

bool ExtensionUnitControl::resolveNodeId() {
    IKsTopologyInfo* topology = nullptr;
    const HRESULT hr = filter_->QueryInterface(kIidIKsTopologyInfo, reinterpret_cast<void**>(&topology));
    if (FAILED(hr) || !topology) {
        printf("[XU] IKsTopologyInfo is not available, hr=0x%08lx\n",
                    static_cast<unsigned long>(hr));
        return false;
    }

    DWORD nodeCount = 0;
    const HRESULT countHr = topology->get_NumNodes(&nodeCount);
    if (FAILED(countHr)) {
        printf("[XU] Failed to query topology node count, hr=0x%08lx\n",
                    static_cast<unsigned long>(countHr));
        topology->Release();
        return false;
    }

    bool found = false;
    for (DWORD i = 0; i < nodeCount; ++i) {
        GUID nodeType = GUID_NULL;
        if (FAILED(topology->get_NodeType(i, &nodeType))) {
            continue;
        }
        if (SUCCEEDED(topology->get_NodeType(i, &nodeType))) {
            // 打印所有节点类型
            printf("[XU] Node %lu: GUID={%08lx-%04x-%04x-%02x%02x-%02x%02x%02x%02x%02x%02x}\n",
                   i, nodeType.Data1, nodeType.Data2, nodeType.Data3,
                   nodeType.Data4[0], nodeType.Data4[1], nodeType.Data4[2], nodeType.Data4[3],
                   nodeType.Data4[4], nodeType.Data4[5], nodeType.Data4[6], nodeType.Data4[7]);
            
            if (InlineIsEqualGUID(nodeType, kKsNodeTypeDevSpecific)) {
                nodeId_ = i;
                found = true;
                break;
            }
        }
        // Most UVC extension units show up as a device-specific topology node.
        if (InlineIsEqualGUID(nodeType, kKsNodeTypeDevSpecific)) {
            nodeId_ = i;
            found = true;
            break;
        }
    }

    topology->Release();
    return found;
}

bool ExtensionUnitControl::query(uint8_t selector, unsigned long flags, void* data, unsigned long size) const {
    if (!ksControl_) {
        printf("[XU] query failed: ksControl_ is null\n");
        return false;
    }

    KSP_NODE property = {};
    std::memcpy(&property.Property.Set, activeExtGuidBytes_, sizeof(activeExtGuidBytes_));
    property.Property.Id = selector;
    property.Property.Flags = flags | KSPROPERTY_TYPE_TOPOLOGY;
    property.NodeId = nodeId_;

    ULONG bytesReturned = 0;
    const HRESULT hr = ksControl_->KsProperty(reinterpret_cast<PKSPROPERTY>(&property), sizeof(property),
                                              data, size, &bytesReturned);
    if (FAILED(hr)) {
        printf("[XU] KsProperty failed: selector=%u flags=0x%lx node=%lu hr=0x%08lx\n",
                    static_cast<unsigned>(selector), flags, nodeId_, static_cast<unsigned long>(hr));
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
