#pragma once

#include <windows.h>
#include <dshow.h>
#include <ks.h>
#include <ksmedia.h>
#include <ksproxy.h>
#include <vidcap.h>
#include <cstdint>
#include <string>

class MetadataReader {
public:
    MetadataReader();
    ~MetadataReader();

    bool open(const std::string& devicePath);
    void close();
    bool isOpen() const;
    
    // 读取最新的metadata时间戳
    uint64_t readLatestTimestamp();
    
    // 获取节点ID和扩展单元信息
    bool getNodeId(unsigned long* nodeId) const;

private:
    bool bindFilterByDevicePath(const std::string& devicePath);
    bool resolveNodeId();
    bool queryControl(uint8_t selector, unsigned long flags, void* data, unsigned long size) const;

    IBaseFilter* filter_ = nullptr;
    IKsControl* ksControl_ = nullptr;
    unsigned long nodeId_ = 0;
    std::string devicePath_;
    
    // 缓存上一次的时间戳，用于去重
    uint64_t lastTimestamp_ = 0;
};