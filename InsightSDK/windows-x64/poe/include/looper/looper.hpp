// Copyright (c) 2026 Looper Robotics. All rights reserved.
// Licensed under the MIT License.

/**
 * @file looper.hpp
 * @brief LooperSDK C++ API.
 *
 * Header-only wrapper over looper.h adding RAII and exceptions.
 *
 * @code
 *   lp::Context ctx;
 *   for (const auto &d : ctx.discover()) { ... }
 *   ctx.forceIp(mac, lp::IpConfig::Static("192.168.1.50"));
 * @endcode
 */

#ifndef LOOPER_HPP
#define LOOPER_HPP

#include "looper/looper.h"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace lp {

class Error : public std::runtime_error {
public:
    Error(lp_status status, const std::string &detail)
        : std::runtime_error(detail.empty() ? lp_status_string(status)
                                            : detail + " (" + lp_status_string(status) + ")"),
          status_(status) {}

    lp_status status() const { return status_; }

private:
    lp_status status_;
};

enum class IpMode { Dhcp = LP_IPMODE_DHCP, Static = LP_IPMODE_STATIC, LinkLocal = LP_IPMODE_LINKLOCAL };

enum class TimeSync { No = LP_TIMESYNC_NO, Yes = LP_TIMESYNC_YES, Unknown = LP_TIMESYNC_UNKNOWN };

struct DeviceInfo {
    std::string model;
    std::string serial;
    std::string fwVersion;
    std::string mac;
    std::string ip;
    std::string netmask;
    std::string gateway;
    IpMode      ipMode = IpMode::Dhcp;
    TimeSync    timeSync = TimeSync::Unknown;
    std::string ntpServer;        // empty when the clock is not synchronised
    int         ntpOffsetUs = 0;  // last correction applied

    std::string ipModeString() const {
        switch(ipMode) {
        case IpMode::Dhcp:      return "dhcp";
        case IpMode::Static:    return "static";
        case IpMode::LinkLocal: return "link-local";
        }
        return "unknown";
    }

    /* Worth checking before comparing timestamps across cameras: an unsynced
       clock makes its timestamps incomparable with everyone else's. */
    std::string timeSyncString() const {
        switch(timeSync) {
        case TimeSync::No:      return "no";
        case TimeSync::Yes:     return "yes";
        case TimeSync::Unknown: return "unknown";
        }
        return "unknown";
    }
};

class IpConfig {
public:
    /* Persistent static address, kept across reboots. */
    static IpConfig Static(const std::string &ip,
                           const std::string &netmask = "255.255.255.0",
                           const std::string &gateway = "") {
        return IpConfig(LP_IPMODE_STATIC, /*persist=*/1, ip, netmask, gateway);
    }

    /* Temporary static address, reverts on the next reboot. */
    static IpConfig Temporary(const std::string &ip,
                              const std::string &netmask = "255.255.255.0",
                              const std::string &gateway = "") {
        return IpConfig(LP_IPMODE_STATIC, /*persist=*/0, ip, netmask, gateway);
    }

    /* Return the camera to DHCP. Always persistent. */
    static IpConfig Dhcp() { return IpConfig(LP_IPMODE_DHCP, 1, "", "", ""); }

    const lp_ip_config *raw() const { return &cfg_; }

private:
    IpConfig(int mode, int persist, const std::string &ip,
             const std::string &netmask, const std::string &gateway) {
        cfg_ = lp_ip_config{};
        cfg_.mode    = mode;
        cfg_.persist = persist;
        copy(cfg_.ip, sizeof(cfg_.ip), ip);
        copy(cfg_.netmask, sizeof(cfg_.netmask), netmask);
        copy(cfg_.gateway, sizeof(cfg_.gateway), gateway);
    }

    static void copy(char *dst, size_t cap, const std::string &src) {
        const size_t n = src.size() < cap - 1 ? src.size() : cap - 1;
        src.copy(dst, n);
        dst[n] = '\0';
    }

    lp_ip_config cfg_{};
};

class Context {
public:
    /* bindAddress selects which local interface to broadcast from; leave empty
     * to use the default route. Matters on a host with several NICs. */
    explicit Context(const std::string &bindAddress = "")
        : ctx_(lp_context_create(bindAddress.empty() ? nullptr : bindAddress.c_str()),
               lp_context_destroy) {
        if(!ctx_) {
            throw Error(LP_ERR_SOCKET, "could not create the discovery socket");
        }
    }

    std::vector<DeviceInfo> discover(int timeoutMs = 1000, size_t maxDevices = 64) {
        std::vector<lp_device_info> raw(maxDevices);
        size_t    found  = 0;
        lp_status status = lp_context_discover(ctx_.get(), raw.data(), raw.size(), &found, timeoutMs);
        if(status != LP_OK) {
            throw Error(status, lp_context_last_error(ctx_.get()));
        }

        std::vector<DeviceInfo> out;
        out.reserve(found);
        for(size_t i = 0; i < found; i++) {
            DeviceInfo d;
            d.model     = raw[i].model;
            d.serial    = raw[i].serial;
            d.fwVersion = raw[i].fw_version;
            d.mac       = raw[i].mac;
            d.ip        = raw[i].ip;
            d.netmask   = raw[i].netmask;
            d.gateway   = raw[i].gateway;
            d.ipMode    = static_cast<IpMode>(raw[i].ip_mode);
            d.timeSync  = static_cast<TimeSync>(raw[i].time_sync);
            d.ntpServer = raw[i].ntp_server;
            d.ntpOffsetUs = raw[i].ntp_offset_us;
            out.push_back(std::move(d));
        }
        return out;
    }

    /* Apply an address configuration to the camera with this MAC. Works even
     * when the camera's current address is not routable from this host. */
    void forceIp(const std::string &mac, const IpConfig &config, int timeoutMs = 3000) {
        lp_status status = lp_context_force_ip(ctx_.get(), mac.c_str(), config.raw(), timeoutMs);
        if(status != LP_OK) {
            throw Error(status, lp_context_last_error(ctx_.get()));
        }
    }

private:
    std::unique_ptr<lp_context, void (*)(lp_context *)> ctx_;
};

/*
 * Fast DDS helpers for a host that subscribes to the cameras.
 *
 * Fast DDS announces one locator per local interface, and a camera sends a
 * copy of every sample to each one. On a host with several interfaces this
 * multiplies the bandwidth each camera has to send. Restricting DDS to the
 * interface that reaches the cameras avoids the duplicates.
 */

/* The local address that would carry traffic to peerIp. Sends nothing. */
inline std::string localAddressFor(const std::string &peerIp) {
    char      buf[16];
    lp_status st = lp_local_address_for(peerIp.c_str(), buf, sizeof(buf));
    if(st != LP_OK) {
        throw Error(st, "no route to " + peerIp);
    }
    return buf;
}

/* The distinct local addresses reaching these cameras: what to whitelist.
 * Cameras with no route are skipped rather than failing the whole call, so a
 * stale entry from an earlier discovery cannot block the rest. */
inline std::vector<std::string> localAddressesFor(const std::vector<DeviceInfo> &devices) {
    std::vector<std::string> out;
    for(const auto &d : devices) {
        char buf[16];
        if(lp_local_address_for(d.ip.c_str(), buf, sizeof(buf)) != LP_OK) {
            continue;
        }
        if(std::find(out.begin(), out.end(), buf) == out.end()) {
            out.emplace_back(buf);
        }
    }
    return out;
}

inline std::string ddsProfileXml(const std::vector<std::string> &addresses) {
    std::vector<const char *> raw;
    raw.reserve(addresses.size());
    for(const auto &a : addresses) {
        raw.push_back(a.c_str());
    }

    std::string buf(8192, '\0');
    size_t      written = 0;
    lp_status   st = lp_dds_profile_render(raw.data(), raw.size(),
                                           &buf[0], buf.size(), &written);
    if(st != LP_OK) {
        throw Error(st, "could not render the Fast DDS profile");
    }
    buf.resize(written);
    return buf;
}

/* Write the profile where Fast DDS can be pointed at it:
 *   export FASTRTPS_DEFAULT_PROFILES_FILE=<path> */
inline void writeDdsProfile(const std::string &path,
                            const std::vector<std::string> &addresses) {
    std::vector<const char *> raw;
    raw.reserve(addresses.size());
    for(const auto &a : addresses) {
        raw.push_back(a.c_str());
    }
    lp_status st = lp_dds_profile_write(path.c_str(), raw.data(), raw.size());
    if(st != LP_OK) {
        throw Error(st, "could not write " + path);
    }
}

}  // namespace lp

#endif /* LOOPER_HPP */
