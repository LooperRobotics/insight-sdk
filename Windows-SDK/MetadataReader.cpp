#include "MetadataReader.hpp"
#include <dshow.h>
#include <ks.h>
#include <ksmedia.h>
#include <ksproxy.h>
#include <vidcap.h>
#include <cstdio>
#include <cstring>

#pragma comment(lib, "strmiids.lib")

#ifdef DEFINE_GUID
#undef DEFINE_GUID
#endif
#define DEFINE_GUID(name, l, w1, w2, b1, b2, b3, b4, b5, b6, b7, b8) \
    extern "C" const GUID name = { l, w1, w2, { b1, b2, b3, b4, b5, b6, b7, b8 } }

DEFINE_GUID(CLSID_ExtensionUnit, 
    0x04030201, 0x0605, 0x0807, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10);

#ifndef KSNODETYPE_DEV_SPECIFIC
DEFINE_GUID(KSNODETYPE_DEV_SPECIFIC, 
    0x941c7ac0, 0xc559, 0x11d0, 0x8a, 0x2b, 0x00, 0xa0, 0xc9, 0x25, 0x5a, 0xc1);
#endif

#undef DEFINE_GUID
#define DEFINE_GUID(name, l, w1, w2, b1, b2, b3, b4, b5, b6, b7, b8)

constexpr uint8_t kMetadataTimestampSelector = 0x08;
constexpr uint8_t kMetadataSizeSelector = 0x09;

MetadataReader::MetadataReader() = default;
MetadataReader::~MetadataReader() { close(); }

bool MetadataReader::open(const std::string& devicePath) {
    close();
    devicePath_ = devicePath;

    if (!bindFilterByDevicePath(devicePath)) {
        return false;
    }

    if (FAILED(filter_->QueryInterface(__uuidof(IKsControl), reinterpret_cast<void**>(&ksControl_)))) {
        printf("[MetadataReader] IKsControl not available\n");
        close();
        return false;
    }

    if (!resolveNodeId()) {
        printf("[MetadataReader] Failed to resolve extension unit node\n");
        close();
        return false;
    }

    printf("[MetadataReader] Opened for device: %s, node=%lu\n", 
           devicePath.c_str(), nodeId_);
    return true;
}

void MetadataReader::close() {
    if (ksControl_) {
        ksControl_->Release();
        ksControl_ = nullptr;
    }
    if (filter_) {
        filter_->Release();
        filter_ = nullptr;
    }
    nodeId_ = 0;
    lastTimestamp_ = 0;
}

bool MetadataReader::isOpen() const {
    return ksControl_ != nullptr;
}

uint64_t MetadataReader::readLatestTimestamp() {
    if (!isOpen()) return 0;

    uint64_t timestamp = 0;
    
    if (queryControl(kMetadataTimestampSelector, KSPROPERTY_TYPE_GET, &timestamp, sizeof(timestamp))) {
        if (timestamp == lastTimestamp_) {
            return 0;
        }
        lastTimestamp_ = timestamp;
        return timestamp;
    }

    uint32_t metadataSize = 0;
    if (queryControl(kMetadataSizeSelector, KSPROPERTY_TYPE_GET, &metadataSize, sizeof(metadataSize))) {
        printf("[MetadataReader] Metadata size: %u\n", metadataSize);
    }

    return 0;
}

bool MetadataReader::bindFilterByDevicePath(const std::string& devicePath) {
    ICreateDevEnum* devEnum = nullptr;
    IEnumMoniker* enumMoniker = nullptr;

    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, nullptr, CLSCTX_INPROC_SERVER,
                                  __uuidof(ICreateDevEnum), reinterpret_cast<void**>(&devEnum));
    if (FAILED(hr) || !devEnum) {
        return false;
    }

    hr = devEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &enumMoniker, 0);
    if (FAILED(hr) || !enumMoniker) {
        devEnum->Release();
        return false;
    }

    int wideLen = MultiByteToWideChar(CP_UTF8, 0, devicePath.c_str(), -1, nullptr, 0);
    if (wideLen <= 0) {
        enumMoniker->Release();
        devEnum->Release();
        return false;
    }
    std::wstring targetPath(wideLen, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, devicePath.c_str(), -1, &targetPath[0], wideLen);

    IMoniker* moniker = nullptr;
    while (enumMoniker->Next(1, &moniker, nullptr) == S_OK) {
        IPropertyBag* propertyBag = nullptr;
        hr = moniker->BindToStorage(nullptr, nullptr, __uuidof(IPropertyBag),
                                    reinterpret_cast<void**>(&propertyBag));
        if (SUCCEEDED(hr) && propertyBag) {
            VARIANT value;
            VariantInit(&value);
            hr = propertyBag->Read(L"DevicePath", &value, nullptr);
            if (SUCCEEDED(hr) && value.vt == VT_BSTR) {
                std::wstring currentPath = value.bstrVal;
                if (currentPath == targetPath) {
                    IBaseFilter* filter = nullptr;
                    hr = moniker->BindToObject(nullptr, nullptr, __uuidof(IBaseFilter),
                                               reinterpret_cast<void**>(&filter));
                    VariantClear(&value);
                    propertyBag->Release();
                    moniker->Release();
                    enumMoniker->Release();
                    devEnum->Release();
                    if (SUCCEEDED(hr) && filter) {
                        filter_ = filter;
                        return true;
                    }
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
    return false;
}

bool MetadataReader::resolveNodeId() {
    if (!filter_) return false;

    IKsTopologyInfo* topology = nullptr;
    HRESULT hr = filter_->QueryInterface(__uuidof(IKsTopologyInfo), reinterpret_cast<void**>(&topology));
    if (FAILED(hr) || !topology) {
        return false;
    }

    DWORD nodeCount = 0;
    hr = topology->get_NumNodes(&nodeCount);
    if (FAILED(hr)) {
        topology->Release();
        return false;
    }

    for (DWORD i = 0; i < nodeCount; ++i) {
        GUID nodeType;
        if (SUCCEEDED(topology->get_NodeType(i, &nodeType))) {
            if (IsEqualGUID(nodeType, KSNODETYPE_DEV_SPECIFIC)) {
                nodeId_ = i;
                topology->Release();
                return true;
            }
        }
    }

    topology->Release();
    return false;
}

bool MetadataReader::queryControl(uint8_t selector, unsigned long flags, 
                                  void* data, unsigned long size) const {
    if (!ksControl_) return false;

    KSP_NODE property = {};
    property.Property.Set = CLSID_ExtensionUnit;
    property.Property.Id = selector;
    property.Property.Flags = flags | KSPROPERTY_TYPE_TOPOLOGY;
    property.NodeId = nodeId_;

    ULONG bytesReturned = 0;
    HRESULT hr = ksControl_->KsProperty(reinterpret_cast<PKSPROPERTY>(&property), sizeof(property),
                                        data, size, &bytesReturned);
    return SUCCEEDED(hr);
}

bool MetadataReader::getNodeId(unsigned long* nodeId) const {
    if (!nodeId) return false;
    *nodeId = nodeId_;
    return nodeId_ != 0;
}