#pragma once

// User-space UVC / HID access over libusb for macOS.
//
// macOS has no V4L2, and AVFoundation neither exposes the Insight camera's
// custom formats (Z16, Y8I) nor the UVC extension unit. The SDK therefore
// takes the device over from the system camera driver with libusb and speaks
// UVC itself: descriptor parsing, probe/commit, isochronous or bulk streaming,
// payload reassembly and extension-unit control requests. The IMU/VIO HID
// interfaces are read through the same handle, because taking over the UVC
// interfaces on macOS detaches the kernel drivers of the whole device.

#include "Insight9SDK.h"

#include <libusb.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace insight9 {

struct UvcFrameDesc {
    uint8_t frameIndex = 0;
    int width = 0;
    int height = 0;
    uint32_t maxFrameSize = 0;
    uint32_t defaultInterval = 0;       // 100 ns units
    std::vector<uint32_t> intervals;    // 100 ns units
};

struct UvcFormatDesc {
    uint8_t formatIndex = 0;
    uint8_t defaultFrameIndex = 0;
    PixelFormat format = PixelFormat::Unknown;
    std::string fourcc;
    std::vector<UvcFrameDesc> frames;
};

struct UvcIsoAlt {
    uint8_t altSetting = 0;
    int packetBytes = 0;                // bytes per (micro)frame interval
};

struct UvcStreamingInterface {
    uint8_t ifnum = 0;
    uint8_t endpoint = 0;
    bool bulk = false;
    std::vector<UvcIsoAlt> isoAlts;     // sorted by packetBytes, ascending
    std::vector<UvcFormatDesc> formats;

    bool offers(PixelFormat fmt) const;
    // Find the format/frame/interval for a request. fps is matched within 0.5.
    bool find(PixelFormat fmt, int width, int height, int fps,
              const UvcFormatDesc** format, const UvcFrameDesc** frame,
              uint32_t* interval) const;
    // Every format/frame/interval combination, in descriptor order.
    std::vector<DeviceCapability> capabilities() const;
    // The first format's default frame at its default interval.
    DeviceCapability defaultCapability() const;
};

struct UvcExtensionUnitDesc {
    uint8_t unitId = 0;
    uint8_t guid[16] = {};
};

// One UVC function: a VideoControl interface plus its VideoStreaming interfaces.
struct UvcFunction {
    uint8_t vcIfnum = 0;
    uint16_t bcdUVC = 0x0100;
    std::vector<UvcStreamingInterface> streams;
    std::vector<UvcExtensionUnitDesc> extensionUnits;

    bool offers(PixelFormat fmt) const;
};

struct HidInterface {
    uint8_t ifnum = 0;
    uint8_t inEndpoint = 0;
    uint16_t maxPacket = 0;
    uint16_t reportDescLength = 0;
    bool isSensor = false;              // report descriptor uses the HID Sensor usage page
    uint8_t bInterval = 0;              // interrupt endpoint polling interval (descriptor value)
    uint16_t inputReportSize = 0;       // largest input report incl. report id, 0 = unknown
};

// libusb context + event thread. Start once before any UsbDevice is used,
// stop after the last one has been released.
bool usbRuntimeStart();
void usbRuntimeStop();

class UsbDevice {
public:
    ~UsbDevice();
    UsbDevice(const UsbDevice&) = delete;
    UsbDevice& operator=(const UsbDevice&) = delete;

    // Return the open device with this VID/PID, opening and claiming it on
    // first use or after the previous instance was marked dead.
    static std::shared_ptr<UsbDevice> acquire(uint16_t vid, uint16_t pid);

    libusb_device_handle* handle() const { return handle_; }
    libusb_speed speed() const { return speed_; }
    const std::vector<UvcFunction>& functions() const { return functions_; }
    const std::vector<HidInterface>& hidInterfaces() const { return hids_; }

    // Stable identifier of one UVC function, e.g. "usb:20-1.2/if0".
    std::string functionPath(const UvcFunction& fn) const;
    std::string interfacePath(uint8_t ifnum) const;
    const UvcFunction* findFunction(const std::string& path) const;

    bool alive() const { return alive_.load(); }
    void markDead() { alive_.store(false); }

    // libusb_control_transfer that also marks the device dead on NO_DEVICE.
    int controlTransfer(uint8_t requestType, uint8_t request, uint16_t value,
                        uint16_t index, uint8_t* data, uint16_t length,
                        unsigned timeoutMs = 1000);

private:
    UsbDevice() = default;
    bool open(libusb_device* dev);
    void close();
    void parseConfig(const libusb_config_descriptor* cfg);
    bool claimInterface(uint8_t ifnum);
    void detectSensorHid(HidInterface& hid);
    int isoPacketBytes(const libusb_endpoint_descriptor& ep) const;

    libusb_device_handle* handle_ = nullptr;
    libusb_speed speed_ = LIBUSB_SPEED_UNKNOWN;
    std::string basePath_;
    std::vector<UvcFunction> functions_;
    std::vector<HidInterface> hids_;
    std::vector<uint8_t> claimed_;
    bool detachedKernelDriver_ = false;
    uint8_t detachedIfnum_ = 0;
    std::atomic<bool> alive_{true};
};

// Raw frame reassembled from UVC payloads, plus the header of its first payload.
struct UvcRawFrame {
    std::vector<uint8_t> data;
    bool hasPts = false;
    uint32_t pts = 0;
    std::vector<uint8_t> vendorMeta;    // payload-header bytes after PTS/SCR
};

// One VideoStreaming interface: negotiation plus transfer management.
// The sink runs on the libusb event thread and must return quickly.
class UvcStream {
public:
    using Sink = std::function<void(UvcRawFrame&)>;

    UvcStream(std::shared_ptr<UsbDevice> dev, uint16_t bcdUVC,
              const UvcStreamingInterface& vs);
    ~UvcStream();
    UvcStream(const UvcStream&) = delete;
    UvcStream& operator=(const UvcStream&) = delete;

    // Validate the request against the descriptors. Does not touch the device.
    bool configure(PixelFormat fmt, int width, int height, int fps);
    // Probe/commit, select the alternate setting and submit transfers.
    bool start(Sink sink);
    void stop();

    bool failed() const { return failed_.load(); }
    PixelFormat format() const { return fmt_; }
    int width() const { return width_; }
    int height() const { return height_; }
    uint8_t ifnum() const { return vs_.ifnum; }

private:
    static void LIBUSB_CALL onTransfer(libusb_transfer* transfer);
    void handleTransfer(libusb_transfer* transfer);
    void processPayload(const uint8_t* p, int len);
    void finishFrame();
    bool negotiate();
    void freeTransfers();

    std::shared_ptr<UsbDevice> dev_;
    uint16_t bcdUVC_;
    UvcStreamingInterface vs_;

    PixelFormat fmt_ = PixelFormat::Unknown;
    int width_ = 0;
    int height_ = 0;
    uint8_t formatIndex_ = 0;
    uint8_t frameIndex_ = 0;
    uint32_t interval_ = 0;
    size_t expectedSize_ = 0;           // 0 = variable size (MJPEG)
    uint32_t maxFrameSize_ = 0;
    uint32_t maxPayload_ = 0;

    Sink sink_;
    std::vector<libusb_transfer*> transfers_;
    std::mutex inFlightMutex_;
    std::condition_variable inFlightCv_;
    int inFlight_ = 0;
    std::atomic<bool> streaming_{false};
    std::atomic<bool> failed_{false};
    int consecutiveErrors_ = 0;
    int totalErrors_ = 0;
    int droppedFrames_ = 0;
    int deliveredFrames_ = 0;

    // Frame assembly state; touched only by the libusb event thread.
    UvcRawFrame cur_;
    bool haveFid_ = false;
    uint8_t lastFid_ = 0;
    bool frameError_ = false;
    bool headerSeen_ = false;
};

// Continuous reader for one HID interrupt IN endpoint. Keeps several transfers
// queued so the endpoint is always being polled, the way the Linux (hidraw) and
// Windows HID drivers do; a single synchronous read leaves gaps in which the
// device drops reports (VIO arrived at 50 Hz instead of 100 Hz).
// Reports are queued here and handed out by read() on the caller's thread.
class HidInterruptReader {
public:
    HidInterruptReader(std::shared_ptr<UsbDevice> dev, const HidInterface& hid);
    ~HidInterruptReader();
    HidInterruptReader(const HidInterruptReader&) = delete;
    HidInterruptReader& operator=(const HidInterruptReader&) = delete;

    bool start();
    void stop();
    // Wait up to timeoutMs for the next report. Returns false on timeout or failure.
    bool read(std::vector<uint8_t>& report, int timeoutMs);
    bool failed() const { return failed_.load(); }

private:
    static void LIBUSB_CALL onTransfer(libusb_transfer* transfer);
    void handleTransfer(libusb_transfer* transfer);

    std::shared_ptr<UsbDevice> dev_;
    HidInterface hid_;
    std::vector<libusb_transfer*> transfers_;
    std::atomic<bool> running_{false};
    std::atomic<bool> failed_{false};
    int consecutiveErrors_ = 0;          // event thread only

    std::mutex mutex_;                   // guards queue_ and inFlight_
    std::condition_variable cv_;
    std::deque<std::vector<uint8_t>> queue_;
    int inFlight_ = 0;
};

}  // namespace insight9
