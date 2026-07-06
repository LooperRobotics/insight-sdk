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

constexpr GUID kUvcExtensionGuid = {
    0x04030201,
    0x0605,
    0x0807,
    {0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10}
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

    printf("[XU] Bound extension unit GUID to device path: %s (node=%lu, unit=%u)\n",
                devicePath.c_str(), nodeId_, static_cast<unsigned>(kXuUnitId));
    return true;
}

void ExtensionUnitControl::close() {
    IUnknown* ksUnknown = reinterpret_cast<IUnknown*>(ksControl_);
    IUnknown* filterUnknown = reinterpret_cast<IUnknown*>(filter_);
    safeRelease(ksUnknown);
    safeRelease(filterUnknown);
    ksControl_ = nullptr;
    filter_ = nullptr;
    nodeId_ = 0;
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
    printParams(params);
    return query(kCameraParamsSelector, KSPROPERTY_TYPE_GET, &params, sizeof(params));
}

bool ExtensionUnitControl::writeCurrentCameraParams(const camera_params& params) const {
    camera_params copy = params;
    return query(kCameraParamsSelector, KSPROPERTY_TYPE_SET, &copy, sizeof(copy));
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
    printf("[XU] DevicePath not found in DirectShow enumeration: %s\n", devicePath.c_str());
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
        return false;
    }

    KSP_NODE property = {};
    property.Property.Set = kUvcExtensionGuid;
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

    printf("[XU] cam_id=%u, resolution=%u, framerate=%u (%d fps), exposure_time=%.4f, gain=%.4f, backlight=%u\n"
            "brightness=%.4f, contrast=%.4f, gamma=%.4f, hue=%.4f, saturation=%.4f, sharpness=%u, awb=%u\n"
            "white_balance=%.4f, filter=%u, Hardware_model=%u\n",
                static_cast<unsigned>(params.cam_id),
                static_cast<unsigned>(params.resolution),
                static_cast<unsigned>(params.frame_rate),
                fps,
                params.exposure_time,
                params.exposure_gain,
                static_cast<unsigned>(params.backlight_comp),
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

}  // namespace viewer
