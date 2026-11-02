// Copyright (c) 2026 Looper Robotics. All rights reserved.
// Licensed under the MIT License.

/**
 * @file looper.h
 * @brief LooperSDK C API.
 *
 * Discovery and network configuration for Looper network cameras. Streaming
 * (images, depth, IMU, VIO pose) is declared in looper/stream.h.
 *
 * This C header is the stable ABI of the SDK and the one language bindings
 * should target. looper.hpp provides a header-only C++ wrapper on top of it.
 *
 * Typical use:
 * @code
 *   lp_context *ctx = lp_context_create(NULL);
 *   lp_device_info devices[16];
 *   size_t found = 0;
 *   lp_context_discover(ctx, devices, 16, &found, 1000);
 *   lp_context_destroy(ctx);
 * @endcode
 *
 * Discovery and Force IP use UDP broadcast and address a camera by its MAC,
 * so they work even when the camera's current IP address is not routable
 * from the host (wrong address, wrong subnet).
 */

#ifndef LOOPER_H
#define LOOPER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SDK version. The build reads it from here (poe/CMakeLists.txt). */
#define LP_VERSION_MAJOR 1
#define LP_VERSION_MINOR 0
#define LP_VERSION_PATCH 0
#define LP_VERSION_STRING "1.0.0"

/* Symbols of the shared library. LP_STATIC is for builds that link the SDK
 * statically from its sources on Windows; applications using the packages
 * link the shared library and leave it undefined. */
#if defined(_WIN32) && !defined(LP_STATIC)
#  ifdef LP_BUILDING
#    define LP_API __declspec(dllexport)
#  else
#    define LP_API __declspec(dllimport)
#  endif
#elif defined(__GNUC__) && __GNUC__ >= 4
#  define LP_API __attribute__((visibility("default")))
#else
#  define LP_API
#endif

/** Status codes returned by the API. */
typedef enum {
    LP_OK              =  0,  /**< Success. */
    LP_ERR_INVALID     = -1,  /**< Invalid argument. */
    LP_ERR_SOCKET      = -2,  /**< Socket could not be created or configured. */
    LP_ERR_SEND        = -3,  /**< Sending the request failed. */
    LP_ERR_TIMEOUT     = -4,  /**< No reply within the timeout. */
    LP_ERR_DEVICE      = -5,  /**< The camera replied with a failure. */
    LP_ERR_NOMEM       = -6,  /**< Out of memory, or output buffer too small. */
    LP_ERR_IO          = -7,  /**< A file could not be written. */
    LP_ERR_BUSY        = -8,  /**< Another host holds the camera's stream session. */
    LP_ERR_UNAVAILABLE = -9,  /**< Not available on this camera or in its current mode. */
    LP_ERR_PROTOCOL    = -10, /**< Malformed or unexpected reply. */
    LP_ERR_DISCONNECTED = -11 /**< The control session to the camera was lost. */
} lp_status;

/** Clock synchronisation state reported by a camera. */
typedef enum {
    LP_TIMESYNC_NO      = 0,  /**< Not synchronised: timestamps from this camera
                                   do not share a time base with other cameras. */
    LP_TIMESYNC_YES     = 1,  /**< Synchronised. */
    LP_TIMESYNC_UNKNOWN = 2   /**< Disabled, or state not available. */
} lp_time_sync;

/** IP address assignment mode. */
typedef enum {
    LP_IPMODE_DHCP      = 0,  /**< Address obtained via DHCP. */
    LP_IPMODE_STATIC    = 1,  /**< Static address. */
    LP_IPMODE_LINKLOCAL = 2   /**< Reported only: DHCP failed and the static
                                   address was already in use on the network. */
} lp_ip_mode;

#define LP_STR_LEN 32

/** One discovered camera. Addresses are dotted-quad strings. */
typedef struct {
    char    model[LP_STR_LEN];       /**< Product model. */
    char    serial[LP_STR_LEN];      /**< Serial number. */
    char    fw_version[LP_STR_LEN];  /**< Firmware version. */
    char    mac[18];                 /**< "xx:xx:xx:xx:xx:xx" */
    char    ip[16];                  /**< Current IPv4 address. */
    char    netmask[16];             /**< Current netmask. */
    char    gateway[16];             /**< Current gateway. */
    int     ip_mode;                 /**< ::lp_ip_mode currently in effect. */
    int     time_sync;               /**< ::lp_time_sync */
    char    ntp_server[LP_STR_LEN];  /**< NTP server in use, "" if none. */
    int     ntp_offset_us;           /**< Last clock correction, microseconds. */
} lp_device_info;

/**
 * Address configuration to apply to a camera.
 *
 * Set @c mode to ::LP_IPMODE_DHCP to return the camera to DHCP; @c ip,
 * @c netmask and @c gateway are then ignored.
 *
 * @c persist = 1 keeps the setting across reboots.
 * @c persist = 0 applies it until the next reboot only, which is useful for
 * temporary access. DHCP mode must be persistent.
 */
typedef struct {
    int  mode;          /**< ::lp_ip_mode, DHCP or STATIC. */
    int  persist;       /**< 1 = persistent, 0 = until next reboot. */
    char ip[16];
    char netmask[16];
    char gateway[16];   /**< May be empty. */
} lp_ip_config;

/** Opaque context handle. */
typedef struct lp_context lp_context;

/** Version of the library in use, "MAJOR.MINOR.PATCH". Compare with
 *  LP_VERSION_STRING to catch a header / library mismatch. */
LP_API const char *lp_version(void);

/** Human readable text for a status code. Never NULL. */
LP_API const char *lp_status_string(lp_status status);

/** Detail about the most recent failure on this context, or "" if none. */
LP_API const char *lp_context_last_error(const lp_context *ctx);

/**
 * Create a context.
 *
 * @param bind_addr Local IPv4 address selecting the interface to broadcast
 *                  from, or NULL to use the default route. Recommended on
 *                  hosts with more than one network interface.
 * @return A new context, or NULL on failure.
 */
LP_API lp_context *lp_context_create(const char *bind_addr);

/** Destroy a context. Accepts NULL. */
LP_API void        lp_context_destroy(lp_context *ctx);

/**
 * Broadcast a discovery request and collect replies.
 *
 * Always waits for the full timeout, since the number of cameras on the
 * network is not known in advance.
 *
 * @param ctx         Context.
 * @param devices     Output array.
 * @param max_devices Capacity of @p devices.
 * @param found       Receives the number of entries written.
 * @param timeout_ms  How long to collect replies.
 */
LP_API lp_status lp_context_discover(lp_context *ctx, lp_device_info *devices,
                              size_t max_devices, size_t *found, int timeout_ms);

/**
 * Apply an address configuration to the camera with the given MAC.
 *
 * The camera acknowledges before applying the change, so ::LP_OK means the
 * request was accepted. After moving a camera to another subnet, run
 * discovery again to confirm the new address.
 *
 * @param ctx        Context.
 * @param mac        Target MAC, "xx:xx:xx:xx:xx:xx".
 * @param config     Configuration to apply.
 * @param timeout_ms How long to wait for the acknowledgement.
 */
LP_API lp_status lp_context_force_ip(lp_context *ctx, const char *mac,
                              const lp_ip_config *config, int timeout_ms);

/**
 * @name Fast DDS helpers
 *
 * The cameras publish data over ROS 2. Fast DDS announces one locator per
 * local network interface, and a camera sends a copy of every sample to each
 * announced locator. On a host with several interfaces (for example Ethernet,
 * Wi-Fi and a VPN) this multiplies the bandwidth each camera has to send.
 * These helpers generate a Fast DDS profile that restricts DDS to the
 * interface(s) that actually reach the cameras.
 * @{
 */

/**
 * The local address the host would use to reach @p peer_ip, i.e. the address
 * to whitelist. Sends no traffic.
 *
 * @param peer_ip Camera IPv4 address.
 * @param out     Output buffer, at least 16 bytes.
 * @param outlen  Size of @p out.
 * @return ::LP_ERR_SOCKET when there is no route to @p peer_ip.
 */
LP_API lp_status lp_local_address_for(const char *peer_ip, char *out, size_t outlen);

/**
 * Render a Fast DDS profile restricting DDS to the given local addresses.
 *
 * @param addresses Dotted-quad IPv4 addresses. Usually one; more when the
 *                  cameras are on several network segments.
 * @param count     Number of addresses.
 * @param out       Output buffer.
 * @param outlen    Size of @p out.
 * @param written   Receives the length written; may be NULL.
 * @return ::LP_ERR_NOMEM if @p outlen is too small.
 */
LP_API lp_status lp_dds_profile_render(const char *const *addresses, size_t count,
                                char *out, size_t outlen, size_t *written);

/**
 * Same as lp_dds_profile_render(), written to a file. Point Fast DDS at it by
 * exporting FASTRTPS_DEFAULT_PROFILES_FILE before starting the subscriber.
 */
LP_API lp_status lp_dds_profile_write(const char *path,
                               const char *const *addresses, size_t count);

/**
 * Socket buffer size requested by the generated profile. The Linux kernel
 * silently caps it at net.core.rmem_max; compare against that limit to warn
 * about an undersized setting.
 */
LP_API size_t lp_dds_socket_buffer_bytes(void);

/** @} */

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* LOOPER_H */
