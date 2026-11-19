#include "UvcDevice.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

#define USB_LOG_ERROR(fmt, ...) fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__)
#define USB_LOG_WARN(fmt, ...)  fprintf(stderr, "[WARN] " fmt "\n", ##__VA_ARGS__)
#ifdef DEBUG
    #define USB_LOG_INFO(fmt, ...)  printf("[INFO] " fmt "\n", ##__VA_ARGS__)
    #define USB_LOG_DEBUG(fmt, ...) printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)
#else
    #define USB_LOG_INFO(fmt, ...)  ((void)0)
    #define USB_LOG_DEBUG(fmt, ...) ((void)0)
#endif

namespace insight9 {
namespace {

// ==================== USB / UVC constants ====================
constexpr uint8_t kClassVideo = 0x0E;
constexpr uint8_t kSubclassVideoControl = 0x01;
constexpr uint8_t kSubclassVideoStreaming = 0x02;
constexpr uint8_t kClassHid = 0x03;

constexpr uint8_t kDescCsInterface = 0x24;
constexpr uint8_t kDescHid = 0x21;
constexpr uint8_t kDescHidReport = 0x22;

constexpr uint8_t kVcHeader = 0x01;
constexpr uint8_t kVcExtensionUnit = 0x06;

constexpr uint8_t kVsInputHeader = 0x01;
constexpr uint8_t kVsFormatUncompressed = 0x04;
constexpr uint8_t kVsFrameUncompressed = 0x05;
constexpr uint8_t kVsFormatMjpeg = 0x06;
constexpr uint8_t kVsFrameMjpeg = 0x07;
constexpr uint8_t kVsFormatFrameBased = 0x10;
constexpr uint8_t kVsFrameFrameBased = 0x11;

constexpr uint8_t kUvcSetCur = 0x01;
constexpr uint8_t kUvcGetCur = 0x81;
constexpr uint8_t kVsProbeControl = 0x01;
constexpr uint8_t kVsCommitControl = 0x02;

constexpr uint8_t kReqTypeClassIfOut = 0x21;   // host->device, class, interface
constexpr uint8_t kReqTypeClassIfIn  = 0xA1;   // device->host, class, interface
constexpr uint8_t kReqTypeStdIfIn    = 0x81;   // device->host, standard, interface

constexpr uint8_t kHidUsagePageSensor = 0x20;

// Payload header bmHeaderInfo bits.
constexpr uint8_t kPhFid = 0x01;
constexpr uint8_t kPhEof = 0x02;
constexpr uint8_t kPhPts = 0x04;
constexpr uint8_t kPhScr = 0x08;
constexpr uint8_t kPhErr = 0x40;

// Same as Linux uvcvideo (UVC_CTRL_STREAMING_TIMEOUT / UVC_CTRL_CONTROL_TIMEOUT):
// the firmware handles control requests in user space and can be slow to
// answer, especially while it is starting a stream.
constexpr unsigned kStreamingCtrlTimeoutMs = 5000;

constexpr int kIsoTransfers = 8;
constexpr int kIsoPacketsPerTransfer = 32;
constexpr int kBulkTransfers = 8;
constexpr int kMaxConsecutiveErrors = 200;

// HID interrupt reads kept queued per endpoint, and how many unread reports
// to buffer (about 1 s of IMU data) before dropping the oldest.
constexpr int kHidTransfers = 8;
constexpr size_t kHidQueueLimit = 512;

inline uint16_t le16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
inline uint32_t le32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
inline void put16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
inline void put32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

int intervalToFps(uint32_t interval) {
    return interval ? (int)std::lround(10000000.0 / interval) : 0;
}

// Map a UVC format GUID (FourCC in the first four bytes) to a PixelFormat.
PixelFormat formatFromGuid(const uint8_t* guid, std::string& fourcc) {
    char fc[5] = {(char)guid[0], (char)guid[1], (char)guid[2], (char)guid[3], 0};
    fourcc.assign(fc, 4);
    if (!memcmp(fc, "YUY2", 4) || !memcmp(fc, "YUYV", 4)) return PixelFormat::YUYV;
    if (!memcmp(fc, "YVYU", 4)) return PixelFormat::YVYU;
    if (!memcmp(fc, "NV12", 4)) return PixelFormat::NV12;
    if (!memcmp(fc, "Y8I ", 4)) return PixelFormat::Y8I;
    if (!memcmp(fc, "Z16 ", 4)) return PixelFormat::Z16;
    if (!memcmp(fc, "Y800", 4) || !memcmp(fc, "GREY", 4) || !memcmp(fc, "Y8  ", 4)) return PixelFormat::GREY;
    if (guid[0] == 0x00 && guid[1] == 'R' && guid[2] == 'G' && guid[3] == 'B') return PixelFormat::RGB8;
    if (le32(guid) == 0xe436eb7dU) return PixelFormat::RGB8;   // MEDIASUBTYPE_RGB24
    return PixelFormat::Unknown;
}

size_t expectedFrameSize(PixelFormat fmt, int w, int h) {
    const size_t px = (size_t)w * (size_t)h;
    switch (fmt) {
        case PixelFormat::GREY:  return px;
        case PixelFormat::NV12:  return px * 3 / 2;
        case PixelFormat::RGB8:  return px * 3;
        case PixelFormat::YUYV:
        case PixelFormat::YVYU:
        case PixelFormat::Y8I:
        case PixelFormat::Z16:   return px * 2;
        default:                 return 0;   // MJPEG: variable
    }
}

// Walk a block of class-specific descriptors: fn(type, subtype, desc, len).
template <typename Fn>
void forEachDescriptor(const unsigned char* extra, int length, Fn fn) {
    int off = 0;
    while (extra && off + 2 <= length) {
        const uint8_t len = extra[off];
        if (len < 2 || off + len > length) break;
        const uint8_t type = extra[off + 1];
        const uint8_t subtype = len >= 3 ? extra[off + 2] : 0;
        fn(type, subtype, extra + off, (int)len);
        off += len;
    }
}

void parseVcDescriptors(const unsigned char* extra, int length,
                        UvcFunction& fn, std::vector<uint8_t>& collection) {
    forEachDescriptor(extra, length, [&](uint8_t type, uint8_t subtype, const uint8_t* d, int len) {
        if (type != kDescCsInterface) return;
        if (subtype == kVcHeader && len >= 12) {
            fn.bcdUVC = le16(d + 3);
            const int count = d[11];
            for (int i = 0; i < count && 12 + i < len; ++i) collection.push_back(d[12 + i]);
        } else if (subtype == kVcExtensionUnit && len >= 20) {
            UvcExtensionUnitDesc xu;
            xu.unitId = d[3];
            memcpy(xu.guid, d + 4, 16);
            fn.extensionUnits.push_back(xu);
        }
    });
}

void parseVsDescriptors(const unsigned char* extra, int length, UvcStreamingInterface& vs) {
    forEachDescriptor(extra, length, [&](uint8_t type, uint8_t subtype, const uint8_t* d, int len) {
        if (type != kDescCsInterface) return;
        switch (subtype) {
            case kVsInputHeader:
                if (len >= 7) vs.endpoint = d[6];
                break;
            case kVsFormatUncompressed:
            case kVsFormatFrameBased:
                if (len >= 23) {
                    UvcFormatDesc f;
                    f.formatIndex = d[3];
                    f.format = formatFromGuid(d + 5, f.fourcc);
                    f.defaultFrameIndex = d[22];
                    vs.formats.push_back(f);
                }
                break;
            case kVsFormatMjpeg:
                if (len >= 7) {
                    UvcFormatDesc f;
                    f.formatIndex = d[3];
                    f.format = PixelFormat::MJPEG;
                    f.fourcc = "MJPG";
                    f.defaultFrameIndex = d[6];
                    vs.formats.push_back(f);
                }
                break;
            case kVsFrameUncompressed:
            case kVsFrameMjpeg:
            case kVsFrameFrameBased: {
                if (vs.formats.empty() || len < 26) break;
                UvcFrameDesc fr;
                fr.frameIndex = d[3];
                fr.width = le16(d + 5);
                fr.height = le16(d + 7);
                uint8_t intervalType;
                if (subtype == kVsFrameFrameBased) {
                    fr.defaultInterval = le32(d + 17);
                    intervalType = d[21];
                } else {
                    fr.maxFrameSize = le32(d + 17);
                    fr.defaultInterval = le32(d + 21);
                    intervalType = d[25];
                }
                const uint8_t* iv = d + 26;
                if (intervalType == 0) {
                    // Continuous range: min, max, step. Offer min, default and max.
                    if (len >= 38) {
                        fr.intervals.push_back(le32(iv));
                        fr.intervals.push_back(fr.defaultInterval);
                        fr.intervals.push_back(le32(iv + 4));
                    }
                } else {
                    for (int i = 0; i < intervalType && 26 + 4 * (i + 1) <= len; ++i) {
                        fr.intervals.push_back(le32(iv + 4 * i));
                    }
                }
                if (fr.intervals.empty() && fr.defaultInterval) fr.intervals.push_back(fr.defaultInterval);
                vs.formats.back().frames.push_back(fr);
                break;
            }
            default:
                break;
        }
    });
}

// Size in bytes of the largest input report, including the report id byte when
// the descriptor uses report ids (what Linux usbhid sizes its interrupt URB to).
uint16_t maxInputReportSize(const uint8_t* d, int len) {
    struct Globals {
        uint32_t reportSize = 0;
        uint32_t reportCount = 0;
        uint8_t reportId = 0;
    };
    Globals g;
    std::vector<Globals> stack;
    uint32_t bits[256] = {};
    bool usesIds = false;

    int i = 0;
    while (i < len) {
        const uint8_t prefix = d[i];
        if (prefix == 0xFE) {                       // long item
            if (i + 1 >= len) break;
            i += 3 + d[i + 1];
            continue;
        }
        int size = prefix & 0x03;
        if (size == 3) size = 4;
        if (i + 1 + size > len) break;
        uint32_t u = 0;
        for (int b = 0; b < size; ++b) u |= (uint32_t)d[i + 1 + b] << (8 * b);
        const uint8_t type = (prefix >> 2) & 0x03;
        const uint8_t tag = (prefix >> 4) & 0x0F;

        if (type == 0 && tag == 0x8) {              // main: Input
            bits[g.reportId] += g.reportSize * g.reportCount;
        } else if (type == 1) {                     // global
            switch (tag) {
                case 0x7: g.reportSize = u; break;
                case 0x8: g.reportId = (uint8_t)u; usesIds = true; break;
                case 0x9: g.reportCount = u; break;
                case 0xA: stack.push_back(g); break;
                case 0xB: if (!stack.empty()) { g = stack.back(); stack.pop_back(); } break;
                default: break;
            }
        }
        i += 1 + size;
    }

    uint32_t largest = 0;
    for (uint32_t b : bits) largest = (std::max)(largest, (b + 7) / 8);
    if (largest == 0) return 0;
    return (uint16_t)(largest + (usesIds ? 1 : 0));
}

// Is the HID Sensor usage page (0x20) used anywhere in this report descriptor?
bool reportDescriptorIsSensor(const uint8_t* d, int len) {
    int i = 0;
    while (i < len) {
        const uint8_t prefix = d[i];
        if (prefix == 0xFE) {                       // long item
            if (i + 1 >= len) break;
            i += 3 + d[i + 1];
            continue;
        }
        int size = prefix & 0x03;
        if (size == 3) size = 4;
        if (i + 1 + size > len) break;
        if ((prefix & 0xFC) == 0x04) {              // global: Usage Page
            uint32_t page = 0;
            for (int b = 0; b < size; ++b) page |= (uint32_t)d[i + 1 + b] << (8 * b);
            if (page == kHidUsagePageSensor) return true;
        }
        i += 1 + size;
    }
    return false;
}

// ==================== Runtime: context, event thread, registry ====================
std::mutex g_runtimeMutex;
libusb_context* g_usb = nullptr;
std::thread g_eventThread;
std::atomic<bool> g_eventRun{false};

std::mutex g_registryMutex;
std::shared_ptr<UsbDevice> g_current;

void eventLoop() {
    while (g_eventRun.load()) {
        timeval tv{0, 100000};
        const int r = libusb_handle_events_timeout_completed(g_usb, &tv, nullptr);
        if (r != 0 && r != LIBUSB_ERROR_INTERRUPTED && r != LIBUSB_ERROR_TIMEOUT) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

}  // namespace

bool usbRuntimeStart() {
    std::lock_guard<std::mutex> lock(g_runtimeMutex);
    if (g_usb) return true;
    const int r = libusb_init(&g_usb);
    if (r != 0) {
        USB_LOG_ERROR("[USB] libusb_init failed: %s", libusb_error_name(r));
        g_usb = nullptr;
        return false;
    }
    g_eventRun.store(true);
    g_eventThread = std::thread(eventLoop);
    return true;
}

void usbRuntimeStop() {
    std::lock_guard<std::mutex> lock(g_runtimeMutex);
    if (!g_usb) return;
    {
        std::lock_guard<std::mutex> reg(g_registryMutex);
        g_current.reset();
    }
    // Device handles must be closed while the event thread is still running,
    // so the release above has to happen first.
    g_eventRun.store(false);
    if (g_eventThread.joinable()) g_eventThread.join();
    libusb_exit(g_usb);
    g_usb = nullptr;
}

// ==================== Descriptor helpers ====================
bool UvcStreamingInterface::offers(PixelFormat fmt) const {
    for (const auto& f : formats) {
        if (f.format == fmt && !f.frames.empty()) return true;
    }
    return false;
}

bool UvcStreamingInterface::find(PixelFormat fmt, int width, int height, int fps,
                                 const UvcFormatDesc** format, const UvcFrameDesc** frame,
                                 uint32_t* interval) const {
    for (const auto& f : formats) {
        if (f.format != fmt) continue;
        for (const auto& fr : f.frames) {
            if (fr.width != width || fr.height != height) continue;
            for (uint32_t iv : fr.intervals) {
                if (!iv || std::fabs(10000000.0 / iv - fps) >= 0.5) continue;
                *format = &f;
                *frame = &fr;
                *interval = iv;
                return true;
            }
        }
    }
    return false;
}

std::vector<DeviceCapability> UvcStreamingInterface::capabilities() const {
    std::vector<DeviceCapability> caps;
    for (const auto& f : formats) {
        if (f.format == PixelFormat::Unknown) continue;
        for (const auto& fr : f.frames) {
            std::vector<int> seen;
            for (uint32_t iv : fr.intervals) {
                const int fps = intervalToFps(iv);
                if (fps <= 0 || std::find(seen.begin(), seen.end(), fps) != seen.end()) continue;
                seen.push_back(fps);
                DeviceCapability cap;
                cap.width = fr.width;
                cap.height = fr.height;
                cap.fps = fps;
                cap.format = f.format;
                cap.valid = true;
                caps.push_back(cap);
            }
        }
    }
    return caps;
}

DeviceCapability UvcStreamingInterface::defaultCapability() const {
    DeviceCapability cap;
    for (const auto& f : formats) {
        if (f.format == PixelFormat::Unknown || f.frames.empty()) continue;
        const UvcFrameDesc* fr = &f.frames.front();
        for (const auto& candidate : f.frames) {
            if (candidate.frameIndex == f.defaultFrameIndex) { fr = &candidate; break; }
        }
        // Some firmware advertises a dwDefaultFrameInterval that is not in the
        // frame's own interval list (e.g. default 15 fps, list 20..60 fps).
        // Only offer a rate the device accepts: the listed one closest to the
        // default, preferring the higher frame rate on a tie.
        uint32_t iv = fr->defaultInterval;
        if (std::find(fr->intervals.begin(), fr->intervals.end(), iv) == fr->intervals.end() &&
            !fr->intervals.empty()) {
            uint32_t best = fr->intervals.front();
            for (uint32_t candidate : fr->intervals) {
                const int64_t d = std::llabs((int64_t)candidate - (int64_t)fr->defaultInterval);
                const int64_t bestD = std::llabs((int64_t)best - (int64_t)fr->defaultInterval);
                if (d < bestD || (d == bestD && candidate < best)) best = candidate;
            }
            iv = best;
        }
        cap.width = fr->width;
        cap.height = fr->height;
        cap.fps = intervalToFps(iv);
        cap.format = f.format;
        cap.valid = cap.fps > 0;
        break;
    }
    return cap;
}

bool UvcFunction::offers(PixelFormat fmt) const {
    for (const auto& vs : streams) {
        if (vs.offers(fmt)) return true;
    }
    return false;
}

// ==================== UsbDevice ====================
std::shared_ptr<UsbDevice> UsbDevice::acquire(uint16_t vid, uint16_t pid) {
    std::lock_guard<std::mutex> lock(g_registryMutex);
    if (!g_usb) return nullptr;
    if (g_current && g_current->alive()) return g_current;
    // A dead instance stays alive for whoever still holds it; we just stop
    // handing it out.
    g_current.reset();

    libusb_device** list = nullptr;
    const ssize_t n = libusb_get_device_list(g_usb, &list);
    if (n < 0) return nullptr;
    for (ssize_t i = 0; i < n; ++i) {
        libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(list[i], &desc) != 0) continue;
        if (desc.idVendor != vid || desc.idProduct != pid) continue;
        std::shared_ptr<UsbDevice> dev(new UsbDevice());
        if (dev->open(list[i])) {
            g_current = dev;
            break;
        }
    }
    libusb_free_device_list(list, 1);
    return g_current;
}

UsbDevice::~UsbDevice() {
    close();
}

bool UsbDevice::open(libusb_device* dev) {
    int r = libusb_open(dev, &handle_);
    if (r != 0) {
        USB_LOG_ERROR("[USB] libusb_open failed: %s", libusb_error_name(r));
        handle_ = nullptr;
        return false;
    }

    speed_ = (libusb_speed)libusb_get_device_speed(dev);
    uint8_t ports[8];
    const int nports = libusb_get_port_numbers(dev, ports, (int)sizeof(ports));
    char buf[64];
    int pos = snprintf(buf, sizeof(buf), "usb:%u-", (unsigned)libusb_get_bus_number(dev));
    for (int i = 0; i < nports && pos < (int)sizeof(buf); ++i) {
        pos += snprintf(buf + pos, sizeof(buf) - pos, i ? ".%u" : "%u", (unsigned)ports[i]);
    }
    basePath_ = buf;

    libusb_config_descriptor* cfg = nullptr;
    r = libusb_get_active_config_descriptor(dev, &cfg);
    if (r != 0) r = libusb_get_config_descriptor(dev, 0, &cfg);
    if (r != 0 || !cfg) {
        USB_LOG_ERROR("[USB] Cannot read configuration descriptor: %s", libusb_error_name(r));
        close();
        return false;
    }
    parseConfig(cfg);
    libusb_free_config_descriptor(cfg);

    // Claim everything the SDK uses up front: the device then belongs to this
    // process for the whole session and streams only switch alt settings.
    bool anyStream = false;
    for (const auto& fn : functions_) {
        claimInterface(fn.vcIfnum);
        for (const auto& vs : fn.streams) {
            if (claimInterface(vs.ifnum)) anyStream = true;
        }
    }
    if (!anyStream) {
        USB_LOG_ERROR("[USB] No VideoStreaming interface could be claimed on %s", basePath_.c_str());
        close();
        return false;
    }

    std::vector<HidInterface> usable;
    for (auto& hid : hids_) {
        if (!claimInterface(hid.ifnum)) continue;
        detectSensorHid(hid);
        usable.push_back(hid);
    }
    hids_.swap(usable);

#ifdef DEBUG
    USB_LOG_INFO("[USB] Opened %s (speed=%d): %zu UVC function(s), %zu HID interface(s)",
                 basePath_.c_str(), (int)speed_, functions_.size(), hids_.size());
    for (const auto& fn : functions_) {
        USB_LOG_INFO("[USB]   UVC function %s bcdUVC=%x.%02x streams=%zu xu=%zu",
                     functionPath(fn).c_str(), fn.bcdUVC >> 8, fn.bcdUVC & 0xff,
                     fn.streams.size(), fn.extensionUnits.size());
        for (const auto& vs : fn.streams) {
            std::string fmts;
            for (const auto& f : vs.formats) fmts += (fmts.empty() ? "" : ",") + f.fourcc;
            USB_LOG_INFO("[USB]     VS if%u ep=0x%02x %s formats=[%s]", (unsigned)vs.ifnum,
                         (unsigned)vs.endpoint, vs.bulk ? "bulk" : "iso", fmts.c_str());
        }
    }
#endif
    return true;
}

void UsbDevice::close() {
    if (!handle_) return;
    for (uint8_t ifnum : claimed_) {
        libusb_release_interface(handle_, ifnum);
    }
    claimed_.clear();
    if (detachedKernelDriver_ && alive()) {
        // Hand the device back to macOS (on Darwin this re-enumerates it).
        libusb_attach_kernel_driver(handle_, detachedIfnum_);
    }
    detachedKernelDriver_ = false;
    libusb_close(handle_);
    handle_ = nullptr;
}

bool UsbDevice::claimInterface(uint8_t ifnum) {
    int r = libusb_claim_interface(handle_, ifnum);
    if (r == LIBUSB_ERROR_ACCESS || r == LIBUSB_ERROR_BUSY) {
        // macOS: the system UVC/HID drivers own the interfaces. Detaching them
        // captures the whole device and needs root (sudo). Do not gate this on
        // libusb_kernel_driver_active(): it reports 0 for interfaces held by the
        // user-space camera driver (UVCAssistant), which still blocks the claim.
        if (!detachedKernelDriver_) {
            const int d = libusb_detach_kernel_driver(handle_, ifnum);
            if (d == 0) {
                detachedKernelDriver_ = true;
                detachedIfnum_ = ifnum;
                r = libusb_claim_interface(handle_, ifnum);
            } else {
                USB_LOG_ERROR("[USB] Cannot detach the macOS driver from interface %u: %s",
                              (unsigned)ifnum, libusb_error_name(d));
            }
        } else if (detachedKernelDriver_) {
            r = libusb_claim_interface(handle_, ifnum);
        }
    }
    if (r != 0) {
        USB_LOG_ERROR("[USB] Cannot claim interface %u: %s", (unsigned)ifnum, libusb_error_name(r));
        if (r == LIBUSB_ERROR_ACCESS || r == LIBUSB_ERROR_NOT_SUPPORTED || r == LIBUSB_ERROR_BUSY) {
            USB_LOG_ERROR("[USB] On macOS the system camera driver owns the device. "
                          "Run the program with sudo so the SDK can take it over, "
                          "and close any app that is using the camera.");
        }
        return false;
    }
    claimed_.push_back(ifnum);
    return true;
}

void UsbDevice::detectSensorHid(HidInterface& hid) {
    const uint16_t len = hid.reportDescLength ? hid.reportDescLength : 1024;
    std::vector<uint8_t> desc(len);
    const int r = controlTransfer(kReqTypeStdIfIn, LIBUSB_REQUEST_GET_DESCRIPTOR,
                                  (uint16_t)(kDescHidReport << 8), hid.ifnum, desc.data(), len);
    hid.isSensor = r > 0 && reportDescriptorIsSensor(desc.data(), r);
    hid.inputReportSize = r > 0 ? maxInputReportSize(desc.data(), r) : 0;
}

int UsbDevice::isoPacketBytes(const libusb_endpoint_descriptor& ep) const {
    if (speed_ >= LIBUSB_SPEED_SUPER) {
        libusb_ss_endpoint_companion_descriptor* comp = nullptr;
        if (libusb_get_ss_endpoint_companion_descriptor(g_usb, &ep, &comp) == 0 && comp) {
            const int bytes = comp->wBytesPerInterval;
            libusb_free_ss_endpoint_companion_descriptor(comp);
            if (bytes > 0) return bytes;
        }
    }
    const uint16_t w = ep.wMaxPacketSize;
    return (w & 0x07FF) * (((w >> 11) & 0x3) + 1);
}

void UsbDevice::parseConfig(const libusb_config_descriptor* cfg) {
    functions_.clear();
    hids_.clear();
    std::vector<std::vector<uint8_t>> collections;
    std::vector<UvcStreamingInterface> streams;

    for (int i = 0; i < cfg->bNumInterfaces; ++i) {
        const libusb_interface& itf = cfg->interface[i];
        if (itf.num_altsetting < 1) continue;
        const libusb_interface_descriptor& a0 = itf.altsetting[0];

        if (a0.bInterfaceClass == kClassVideo && a0.bInterfaceSubClass == kSubclassVideoControl) {
            UvcFunction fn;
            fn.vcIfnum = a0.bInterfaceNumber;
            std::vector<uint8_t> collection;
            parseVcDescriptors(a0.extra, a0.extra_length, fn, collection);
            functions_.push_back(fn);
            collections.push_back(collection);
        } else if (a0.bInterfaceClass == kClassVideo && a0.bInterfaceSubClass == kSubclassVideoStreaming) {
            UvcStreamingInterface vs;
            vs.ifnum = a0.bInterfaceNumber;
            // Class-specific descriptors can end up attached to the endpoint
            // (bulk interfaces) rather than the interface, so read both.
            parseVsDescriptors(a0.extra, a0.extra_length, vs);
            for (int e = 0; e < a0.bNumEndpoints; ++e) {
                parseVsDescriptors(a0.endpoint[e].extra, a0.endpoint[e].extra_length, vs);
            }
            for (int a = 0; a < itf.num_altsetting; ++a) {
                const libusb_interface_descriptor& alt = itf.altsetting[a];
                for (int e = 0; e < alt.bNumEndpoints; ++e) {
                    const libusb_endpoint_descriptor& ep = alt.endpoint[e];
                    if (!(ep.bEndpointAddress & LIBUSB_ENDPOINT_IN)) continue;
                    if (vs.endpoint && ep.bEndpointAddress != vs.endpoint) continue;
                    const int type = ep.bmAttributes & 0x03;
                    if (type == LIBUSB_TRANSFER_TYPE_BULK) {
                        vs.bulk = true;
                        vs.endpoint = ep.bEndpointAddress;
                    } else if (type == LIBUSB_TRANSFER_TYPE_ISOCHRONOUS) {
                        vs.endpoint = ep.bEndpointAddress;
                        vs.isoAlts.push_back({alt.bAlternateSetting, isoPacketBytes(ep)});
                    }
                }
            }
            std::sort(vs.isoAlts.begin(), vs.isoAlts.end(),
                      [](const UvcIsoAlt& a, const UvcIsoAlt& b) { return a.packetBytes < b.packetBytes; });
            streams.push_back(vs);
        } else if (a0.bInterfaceClass == kClassHid) {
            HidInterface hid;
            hid.ifnum = a0.bInterfaceNumber;
            forEachDescriptor(a0.extra, a0.extra_length, [&](uint8_t type, uint8_t, const uint8_t* d, int len) {
                if (type == kDescHid && len >= 9 && d[6] == kDescHidReport) hid.reportDescLength = le16(d + 7);
            });
            for (int e = 0; e < a0.bNumEndpoints; ++e) {
                const libusb_endpoint_descriptor& ep = a0.endpoint[e];
                if ((ep.bEndpointAddress & LIBUSB_ENDPOINT_IN) &&
                    (ep.bmAttributes & 0x03) == LIBUSB_TRANSFER_TYPE_INTERRUPT) {
                    hid.inEndpoint = ep.bEndpointAddress;
                    hid.maxPacket = ep.wMaxPacketSize & 0x07FF;
                    hid.bInterval = ep.bInterval;
                    break;
                }
            }
            if (hid.inEndpoint) hids_.push_back(hid);
        }
    }

    // Attach each VideoStreaming interface to the VideoControl interface whose
    // header lists it, or else to the closest preceding one.
    for (auto& vs : streams) {
        UvcFunction* owner = nullptr;
        for (size_t f = 0; f < functions_.size() && !owner; ++f) {
            const auto& c = collections[f];
            if (std::find(c.begin(), c.end(), vs.ifnum) != c.end()) owner = &functions_[f];
        }
        if (!owner) {
            for (auto& fn : functions_) {
                if (fn.vcIfnum < vs.ifnum && (!owner || fn.vcIfnum > owner->vcIfnum)) owner = &fn;
            }
        }
        if (owner) owner->streams.push_back(vs);
    }

    std::sort(hids_.begin(), hids_.end(),
              [](const HidInterface& a, const HidInterface& b) { return a.ifnum < b.ifnum; });
}

std::string UsbDevice::functionPath(const UvcFunction& fn) const {
    return interfacePath(fn.vcIfnum);
}

std::string UsbDevice::interfacePath(uint8_t ifnum) const {
    return basePath_ + "/if" + std::to_string((unsigned)ifnum);
}

const UvcFunction* UsbDevice::findFunction(const std::string& path) const {
    for (const auto& fn : functions_) {
        if (functionPath(fn) == path) return &fn;
    }
    return nullptr;
}

int UsbDevice::controlTransfer(uint8_t requestType, uint8_t request, uint16_t value,
                               uint16_t index, uint8_t* data, uint16_t length,
                               unsigned timeoutMs) {
    if (!handle_ || !alive()) return LIBUSB_ERROR_NO_DEVICE;
    const int r = libusb_control_transfer(handle_, requestType, request, value, index,
                                          data, length, timeoutMs);
    if (r == LIBUSB_ERROR_NO_DEVICE) markDead();
    return r;
}

// ==================== UvcStream ====================
UvcStream::UvcStream(std::shared_ptr<UsbDevice> dev, uint16_t bcdUVC,
                     const UvcStreamingInterface& vs)
    : dev_(std::move(dev)), bcdUVC_(bcdUVC), vs_(vs) {}

UvcStream::~UvcStream() {
    stop();
}

bool UvcStream::configure(PixelFormat fmt, int width, int height, int fps) {
    const UvcFormatDesc* format = nullptr;
    const UvcFrameDesc* frame = nullptr;
    uint32_t interval = 0;
    if (!vs_.find(fmt, width, height, fps, &format, &frame, &interval)) {
        std::string modes;
        for (const auto& cap : vs_.capabilities()) {
            if (cap.format != fmt) continue;
            modes += " " + std::to_string(cap.width) + "x" + std::to_string(cap.height) +
                     "@" + std::to_string(cap.fps);
        }
        USB_LOG_ERROR("[UVC] if%u offers no %dx%d@%d for format %d; available:%s",
                      (unsigned)vs_.ifnum, width, height, fps, (int)fmt,
                      modes.empty() ? " (none)" : modes.c_str());
        return false;
    }
    fmt_ = fmt;
    width_ = width;
    height_ = height;
    formatIndex_ = format->formatIndex;
    frameIndex_ = frame->frameIndex;
    interval_ = interval;
    expectedSize_ = expectedFrameSize(fmt, width, height);
    maxFrameSize_ = frame->maxFrameSize;
    return true;
}

bool UvcStream::negotiate() {
    // Probe/commit control length depends on the UVC version, but some
    // devices disagree with their own bcdUVC, so PROBE falls back to the
    // others. COMMIT is different: on bulk-mode firmware it starts the stream,
    // so it is sent exactly once. Re-sending it after a timeout (the device may
    // well have acted on it) started extra producers on the device: one stream
    // ran at a multiple of its frame rate while its sibling stayed silent.
    const uint16_t primary = bcdUVC_ >= 0x0150 ? 48 : (bcdUVC_ >= 0x0110 ? 34 : 26);
    const uint16_t lengths[] = {primary, 34, 26, 48};

    uint8_t ctrl[48] = {};
    uint16_t len = 0;
    for (uint16_t candidate : lengths) {
        memset(ctrl, 0, sizeof(ctrl));
        put16(ctrl, 1);                         // bmHint: keep dwFrameInterval
        ctrl[2] = formatIndex_;
        ctrl[3] = frameIndex_;
        put32(ctrl + 4, interval_);

        int r = dev_->controlTransfer(kReqTypeClassIfOut, kUvcSetCur, kVsProbeControl << 8,
                                      vs_.ifnum, ctrl, candidate, kStreamingCtrlTimeoutMs);
        if (r != candidate) {
            USB_LOG_DEBUG("[UVC] if%u SET_CUR PROBE (len %u) failed: %s", (unsigned)vs_.ifnum,
                          candidate, r < 0 ? libusb_error_name(r) : "short transfer");
            if (r == LIBUSB_ERROR_NO_DEVICE) return false;
            continue;
        }
        r = dev_->controlTransfer(kReqTypeClassIfIn, kUvcGetCur, kVsProbeControl << 8,
                                  vs_.ifnum, ctrl, candidate, kStreamingCtrlTimeoutMs);
        if (r < 26) {
            USB_LOG_DEBUG("[UVC] if%u GET_CUR PROBE (len %u) failed: %s", (unsigned)vs_.ifnum,
                          candidate, r < 0 ? libusb_error_name(r) : "short transfer");
            if (r == LIBUSB_ERROR_NO_DEVICE) return false;
            continue;
        }
        len = candidate;
        break;
    }
    if (!len) {
        USB_LOG_ERROR("[UVC] if%u probe failed", (unsigned)vs_.ifnum);
        return false;
    }
    if (ctrl[2] != formatIndex_ || ctrl[3] != frameIndex_) {
        USB_LOG_WARN("[UVC] if%u probe changed format/frame %u/%u -> %u/%u",
                     (unsigned)vs_.ifnum, formatIndex_, frameIndex_, ctrl[2], ctrl[3]);
    }

    const int r = dev_->controlTransfer(kReqTypeClassIfOut, kUvcSetCur, kVsCommitControl << 8,
                                        vs_.ifnum, ctrl, len, kStreamingCtrlTimeoutMs);
    if (r != len) {
        USB_LOG_ERROR("[UVC] if%u COMMIT failed: %s", (unsigned)vs_.ifnum,
                      r < 0 ? libusb_error_name(r) : "short transfer");
        return false;
    }

    const uint32_t frameSize = le32(ctrl + 18);
    maxPayload_ = le32(ctrl + 22);
    if (frameSize) maxFrameSize_ = frameSize;
    if (!maxFrameSize_) maxFrameSize_ = (uint32_t)(expectedSize_ ? expectedSize_ : (size_t)width_ * height_ * 2);
    USB_LOG_INFO("[UVC] if%u committed fmt=%u frame=%u interval=%u maxFrame=%u maxPayload=%u (len %u)",
                 (unsigned)vs_.ifnum, ctrl[2], ctrl[3], le32(ctrl + 4), maxFrameSize_, maxPayload_, len);
    return true;
}

bool UvcStream::start(Sink sink) {
    if (streaming_.load()) return false;
    if (formatIndex_ == 0 || !dev_ || !dev_->alive()) return false;
    sink_ = std::move(sink);
    failed_.store(false);
    consecutiveErrors_ = 0;
    totalErrors_ = 0;
    droppedFrames_ = 0;
    deliveredFrames_ = 0;
    cur_ = UvcRawFrame();
    haveFid_ = false;
    frameError_ = false;
    headerSeen_ = false;

    if (!negotiate()) return false;

    libusb_device_handle* h = dev_->handle();
    int transferCount = 0;
    int bufferLength = 0;
    int packets = 0;
    int packetBytes = 0;

    if (vs_.bulk) {
        bufferLength = (int)(maxPayload_ ? maxPayload_ : maxFrameSize_);
        transferCount = kBulkTransfers;
    } else {
        if (vs_.isoAlts.empty()) {
            USB_LOG_ERROR("[UVC] if%u has no isochronous alternate setting", (unsigned)vs_.ifnum);
            return false;
        }
        // Smallest alt setting that carries one payload per interval; the
        // largest one if the device did not say how big a payload is.
        const UvcIsoAlt* alt = &vs_.isoAlts.back();
        for (const auto& a : vs_.isoAlts) {
            if (maxPayload_ && (uint32_t)a.packetBytes >= maxPayload_) { alt = &a; break; }
        }
        const int r = libusb_set_interface_alt_setting(h, vs_.ifnum, alt->altSetting);
        if (r != 0) {
            USB_LOG_ERROR("[UVC] if%u set alt %u failed: %s", (unsigned)vs_.ifnum,
                          (unsigned)alt->altSetting, libusb_error_name(r));
            if (r == LIBUSB_ERROR_NO_DEVICE) dev_->markDead();
            return false;
        }
        packets = kIsoPacketsPerTransfer;
        packetBytes = alt->packetBytes;
        bufferLength = packets * packetBytes;
        transferCount = kIsoTransfers;
    }
    if (bufferLength <= 0) return false;

    cur_.data.reserve(maxFrameSize_);
    streaming_.store(true);
    for (int i = 0; i < transferCount; ++i) {
        libusb_transfer* t = libusb_alloc_transfer(packets);
        unsigned char* buf = static_cast<unsigned char*>(malloc((size_t)bufferLength));
        if (!t || !buf) {
            if (t) libusb_free_transfer(t);
            free(buf);
            break;
        }
        if (vs_.bulk) {
            libusb_fill_bulk_transfer(t, h, vs_.endpoint, buf, bufferLength, onTransfer, this, 0);
        } else {
            libusb_fill_iso_transfer(t, h, vs_.endpoint, buf, bufferLength, packets, onTransfer, this, 0);
            libusb_set_iso_packet_lengths(t, (unsigned)packetBytes);
        }
        transfers_.push_back(t);
        {
            std::lock_guard<std::mutex> lock(inFlightMutex_);
            ++inFlight_;
        }
        const int r = libusb_submit_transfer(t);
        if (r != 0) {
            std::lock_guard<std::mutex> lock(inFlightMutex_);
            --inFlight_;
            USB_LOG_ERROR("[UVC] if%u submit failed: %s", (unsigned)vs_.ifnum, libusb_error_name(r));
            if (r == LIBUSB_ERROR_NO_DEVICE) dev_->markDead();
            break;
        }
    }

    bool ok;
    {
        std::lock_guard<std::mutex> lock(inFlightMutex_);
        ok = inFlight_ > 0;
    }
    if (!ok) {
        stop();
        return false;
    }
    return true;
}

void UvcStream::stop() {
    const bool wasStreaming = streaming_.exchange(false);
    for (libusb_transfer* t : transfers_) libusb_cancel_transfer(t);

    {
        std::unique_lock<std::mutex> lock(inFlightMutex_);
        if (!inFlightCv_.wait_for(lock, std::chrono::seconds(3), [this] { return inFlight_ == 0; })) {
            // Freeing a transfer libusb still owns would be use-after-free;
            // leak them instead (only reachable if the event thread is wedged).
            USB_LOG_ERROR("[UVC] if%u: %d transfer(s) did not complete; leaking them",
                          (unsigned)vs_.ifnum, inFlight_);
            transfers_.clear();
            return;
        }
    }
    freeTransfers();

    if (wasStreaming && dev_ && dev_->alive()) {
        libusb_device_handle* h = dev_->handle();
        if (vs_.bulk) {
            // A bulk stream is stopped by clearing the endpoint halt (as Linux uvcvideo does).
            libusb_clear_halt(h, vs_.endpoint);
        } else {
            libusb_set_interface_alt_setting(h, vs_.ifnum, 0);
        }
    }
}

void UvcStream::freeTransfers() {
    for (libusb_transfer* t : transfers_) {
        free(t->buffer);
        libusb_free_transfer(t);
    }
    transfers_.clear();
}

void LIBUSB_CALL UvcStream::onTransfer(libusb_transfer* transfer) {
    static_cast<UvcStream*>(transfer->user_data)->handleTransfer(transfer);
}

void UvcStream::handleTransfer(libusb_transfer* t) {
    switch (t->status) {
        case LIBUSB_TRANSFER_COMPLETED:
            if (t->type == LIBUSB_TRANSFER_TYPE_ISOCHRONOUS) {
                for (int i = 0; i < t->num_iso_packets; ++i) {
                    const libusb_iso_packet_descriptor& pkt = t->iso_packet_desc[i];
                    if (pkt.status != LIBUSB_TRANSFER_COMPLETED) {
                        frameError_ = true;
                        continue;
                    }
                    if (pkt.actual_length > 0) {
                        processPayload(libusb_get_iso_packet_buffer_simple(t, (unsigned)i),
                                       (int)pkt.actual_length);
                    }
                }
            } else if (t->actual_length > 0) {
                processPayload(t->buffer, t->actual_length);
            }
            consecutiveErrors_ = 0;
            break;
        case LIBUSB_TRANSFER_NO_DEVICE:
            failed_.store(true);
            dev_->markDead();
            break;
        case LIBUSB_TRANSFER_CANCELLED:
            break;
        default:
            // ERROR / STALL / OVERFLOW / TIMED_OUT: drop the frame, keep going,
            // and give up only if the stream never recovers.
            frameError_ = true;
            ++totalErrors_;
            if (totalErrors_ <= 5 || totalErrors_ % 100 == 0) {
                USB_LOG_DEBUG("[UVC] if%u transfer error status=%d actual=%d/%d (total %d)",
                              (unsigned)vs_.ifnum, (int)t->status, t->actual_length, t->length, totalErrors_);
            }
            if (++consecutiveErrors_ >= kMaxConsecutiveErrors) {
                USB_LOG_ERROR("[UVC] if%u: %d consecutive transfer errors (status %d)",
                              (unsigned)vs_.ifnum, consecutiveErrors_, (int)t->status);
                failed_.store(true);
            }
            break;
    }

    const bool resubmit = streaming_.load() && !failed_.load() &&
                          t->status != LIBUSB_TRANSFER_CANCELLED &&
                          t->status != LIBUSB_TRANSFER_NO_DEVICE;
    if (resubmit) {
        const int r = libusb_submit_transfer(t);
        if (r == 0) return;
        if (r == LIBUSB_ERROR_NO_DEVICE) dev_->markDead();
        failed_.store(true);
    }
    // Last access to this object from the event thread for this transfer.
    std::lock_guard<std::mutex> lock(inFlightMutex_);
    --inFlight_;
    inFlightCv_.notify_all();
}

void UvcStream::processPayload(const uint8_t* p, int len) {
    if (len < 2) return;
    const uint8_t hlen = p[0];
    if (hlen < 2 || hlen > len) return;
    const uint8_t info = p[1];
    const uint8_t fid = info & kPhFid;

    // A FID toggle without EOF still ends the previous frame.
    if (haveFid_ && fid != lastFid_ && (!cur_.data.empty() || headerSeen_)) finishFrame();
    haveFid_ = true;
    lastFid_ = fid;

    if (info & kPhErr) frameError_ = true;

    if (!headerSeen_) {
        headerSeen_ = true;
        int off = 2;
        if ((info & kPhPts) && off + 4 <= hlen) {
            cur_.hasPts = true;
            cur_.pts = le32(p + off);
            off += 4;
        }
        if (info & kPhScr) off += 6;
        if (off < hlen) cur_.vendorMeta.assign(p + off, p + hlen);
    }

    const int dataLen = len - hlen;
    if (dataLen > 0) {
        const size_t limit = (size_t)maxFrameSize_ * 2 + 4096;
        if (cur_.data.size() + (size_t)dataLen > limit) {
            frameError_ = true;
        } else {
            cur_.data.insert(cur_.data.end(), p + hlen, p + len);
        }
    }

    if (info & kPhEof) finishFrame();
}

void UvcStream::finishFrame() {
    const size_t size = cur_.data.size();
    const bool complete = expectedSize_ ? size >= expectedSize_ : size > 2;
    if (!frameError_ && complete && sink_) {
        ++deliveredFrames_;
        sink_(cur_);
    } else if (size) {
        ++droppedFrames_;
        if (droppedFrames_ <= 10 || droppedFrames_ % 100 == 0) {
            USB_LOG_DEBUG("[UVC] if%u dropped frame: %zu/%zu bytes error=%d (dropped %d, delivered %d)",
                          (unsigned)vs_.ifnum, size, expectedSize_, (int)frameError_,
                          droppedFrames_, deliveredFrames_);
        }
    }
    // The sink may have taken the buffer; start the next frame from scratch.
    cur_.data.clear();
    if (cur_.data.capacity() < maxFrameSize_) cur_.data.reserve(maxFrameSize_);
    cur_.hasPts = false;
    cur_.pts = 0;
    cur_.vendorMeta.clear();
    frameError_ = false;
    headerSeen_ = false;
}

// ==================== HidInterruptReader ====================
HidInterruptReader::HidInterruptReader(std::shared_ptr<UsbDevice> dev, const HidInterface& hid)
    : dev_(std::move(dev)), hid_(hid) {}

HidInterruptReader::~HidInterruptReader() {
    stop();
}

bool HidInterruptReader::start() {
    if (running_.load() || !dev_ || !dev_->alive()) return false;
    failed_.store(false);
    consecutiveErrors_ = 0;

    // Ask for one input report per transfer, as Linux usbhid does. A longer
    // request is wrong when the report is a whole number of packets (the
    // gadget sets wMaxPacketSize = report length): no short packet ends the
    // transfer, so the next report is appended and lost - this halved the VIO
    // rate. Never ask for less than one packet, so a report the descriptor
    // parse underestimated does not overflow; up to one packet, a report
    // always ends its transfer anyway.
    const int mps = hid_.maxPacket ? hid_.maxPacket : 64;
    const int length = (std::max)((int)hid_.inputReportSize, mps);
    USB_LOG_INFO("[HID] if%u ep=0x%02x wMaxPacketSize=%d bInterval=%u inputReport=%u -> %d byte reads",
                 (unsigned)hid_.ifnum, (unsigned)hid_.inEndpoint, mps, (unsigned)hid_.bInterval,
                 (unsigned)hid_.inputReportSize, length);

    running_.store(true);
    for (int i = 0; i < kHidTransfers; ++i) {
        libusb_transfer* t = libusb_alloc_transfer(0);
        unsigned char* buf = static_cast<unsigned char*>(malloc((size_t)length));
        if (!t || !buf) {
            if (t) libusb_free_transfer(t);
            free(buf);
            break;
        }
        libusb_fill_interrupt_transfer(t, dev_->handle(), hid_.inEndpoint, buf, length,
                                       onTransfer, this, 0);
        transfers_.push_back(t);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ++inFlight_;
        }
        const int r = libusb_submit_transfer(t);
        if (r != 0) {
            std::lock_guard<std::mutex> lock(mutex_);
            --inFlight_;
            USB_LOG_ERROR("[HID] if%u submit failed: %s", (unsigned)hid_.ifnum, libusb_error_name(r));
            if (r == LIBUSB_ERROR_NO_DEVICE) dev_->markDead();
            break;
        }
    }

    bool ok;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ok = inFlight_ > 0;
    }
    if (!ok) {
        stop();
        return false;
    }
    return true;
}

void HidInterruptReader::stop() {
    running_.store(false);
    for (libusb_transfer* t : transfers_) libusb_cancel_transfer(t);

    std::unique_lock<std::mutex> lock(mutex_);
    cv_.notify_all();
    if (!cv_.wait_for(lock, std::chrono::seconds(3), [this] { return inFlight_ == 0; })) {
        // Freeing a transfer libusb still owns would be use-after-free.
        USB_LOG_ERROR("[HID] if%u: %d transfer(s) did not complete; leaking them",
                      (unsigned)hid_.ifnum, inFlight_);
        transfers_.clear();
        return;
    }
    for (libusb_transfer* t : transfers_) {
        free(t->buffer);
        libusb_free_transfer(t);
    }
    transfers_.clear();
    queue_.clear();
}

bool HidInterruptReader::read(std::vector<uint8_t>& report, int timeoutMs) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                 [this] { return !queue_.empty() || failed_.load() || !running_.load(); });
    if (queue_.empty()) return false;
    report.swap(queue_.front());
    queue_.pop_front();
    return true;
}

void LIBUSB_CALL HidInterruptReader::onTransfer(libusb_transfer* transfer) {
    static_cast<HidInterruptReader*>(transfer->user_data)->handleTransfer(transfer);
}

void HidInterruptReader::handleTransfer(libusb_transfer* t) {
    switch (t->status) {
        case LIBUSB_TRANSFER_COMPLETED:
            consecutiveErrors_ = 0;
            if (t->actual_length > 0) {
                std::lock_guard<std::mutex> lock(mutex_);
                // The consumer is far behind: keep the newest reports.
                if (queue_.size() >= kHidQueueLimit) queue_.pop_front();
                queue_.emplace_back(t->buffer, t->buffer + t->actual_length);
                cv_.notify_one();
            }
            break;
        case LIBUSB_TRANSFER_NO_DEVICE:
            failed_.store(true);
            dev_->markDead();
            break;
        case LIBUSB_TRANSFER_CANCELLED:
            break;
        default:
            // STALL / ERROR / OVERFLOW: retry, and let the owner reconnect if
            // the endpoint never recovers.
            if (++consecutiveErrors_ >= kMaxConsecutiveErrors) {
                USB_LOG_ERROR("[HID] if%u: %d consecutive transfer errors (status %d)",
                              (unsigned)hid_.ifnum, consecutiveErrors_, (int)t->status);
                failed_.store(true);
            }
            break;
    }

    if (running_.load() && !failed_.load() &&
        t->status != LIBUSB_TRANSFER_CANCELLED && t->status != LIBUSB_TRANSFER_NO_DEVICE) {
        const int r = libusb_submit_transfer(t);
        if (r == 0) return;
        if (r == LIBUSB_ERROR_NO_DEVICE) dev_->markDead();
        failed_.store(true);
    }
    // Last access to this object from the event thread for this transfer.
    std::lock_guard<std::mutex> lock(mutex_);
    --inFlight_;
    cv_.notify_all();
}

}  // namespace insight9
