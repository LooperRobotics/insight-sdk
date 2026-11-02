// Copyright (c) 2026 Looper Robotics. All rights reserved.
// Licensed under the MIT License.

/**
 * @file stream.h
 * @brief LooperSDK C API: streaming.
 *
 * Opens a camera found by lp_context_discover() and receives its image, depth,
 * IMU and VIO pose streams over Ethernet, without ROS.
 *
 * @code
 *   lp_device *dev = NULL;
 *   if(lp_device_open("192.168.1.10", 2000, &dev) == LP_OK) {
 *       lp_device_start_stream(dev, LP_STREAM_DEPTH, 0, on_frame, NULL);
 *       ...
 *       lp_device_close(dev);
 *   }
 * @endcode
 *
 * Transport: a TCP control session on port 8090 and one RTP/UDP flow per
 * stream, sent by the camera to a port this host chooses. A lost packet loses
 * its whole frame; nothing is retransmitted, so a frame is either complete or
 * not delivered. The camera accepts one host session at a time.
 *
 * Streaming needs the camera to run its Ethernet streaming firmware mode.
 * lp_device_open() fails with ::LP_ERR_SOCKET otherwise.
 *
 * Threads: each started stream has its own receive thread, and the frame
 * callback runs on it. Return quickly; a slow callback makes the socket buffer
 * overflow and frames drop. Do not call lp_device_stop_stream() or
 * lp_device_close() from inside a callback.
 */

#ifndef LOOPER_STREAM_H
#define LOOPER_STREAM_H

#include "looper/looper.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Default TCP control port of the camera. */
#define LP_CONTROL_PORT 8090

/** Streams a camera can send. */
typedef enum {
    LP_STREAM_IR_LEFT  = 0,  /**< Left infrared camera, 8-bit mono. */
    LP_STREAM_IR_RIGHT = 1,  /**< Right infrared camera, 8-bit mono. */
    LP_STREAM_DEPTH    = 2,  /**< Depth, uint16 millimetres, aligned to IR left. */
    LP_STREAM_COLOR    = 3,  /**< Colour camera, MJPEG (or NV12, see device config). */
    LP_STREAM_IMU      = 4,  /**< One ::lp_imu_data per frame, ~400 Hz. */
    LP_STREAM_POSE     = 5,  /**< One ::lp_pose_data per frame, VIO, ~100 Hz. */
    LP_STREAM_COUNT    = 6
} lp_stream;

/** Pixel / payload format of a frame. */
typedef enum {
    LP_FORMAT_UNKNOWN = 0,
    LP_FORMAT_GRAY8   = 1,  /**< 8-bit mono, @c stride bytes per row. */
    LP_FORMAT_Z16     = 2,  /**< uint16 depth in millimetres, 0 = no depth. */
    LP_FORMAT_MJPEG   = 3,  /**< One JPEG image. */
    LP_FORMAT_NV12    = 4,  /**< Y plane then interleaved UV, @c width bytes per row. */
    LP_FORMAT_IMU     = 5,  /**< ::lp_imu_data */
    LP_FORMAT_POSE    = 6   /**< ::lp_pose_data */
} lp_format;

/** Frame flags. */
#define LP_FRAME_RECTIFIED   0x1u  /**< IR image is stereo-rectified. */
#define LP_FRAME_CALIBRATION 0x2u  /**< Camera is in calibration mode (IR is then raw). */
#define LP_FRAME_IMU_RAW     0x4u  /**< IMU in the sensor's body axes (calibration mode);
                                        otherwise rectified left camera optical axes. */

/** Payload of an ::LP_STREAM_IMU frame, in the rectified left camera's
 *  optical axes (x right, y down, z forward), or the IMU's own body axes when
 *  the frame has ::LP_FRAME_IMU_RAW. */
typedef struct {
    double acc[3];  /**< m/s^2 */
    double gyr[3];  /**< rad/s */
} lp_imu_data;

/** VIO tracking state carried in ::lp_pose_data. */
typedef enum {
    LP_VIO_NOT_INITED      = 0,
    LP_VIO_RESTARTING      = 1,
    LP_VIO_STOPPED         = 2,
    LP_VIO_TRACKING        = 3,  /**< Pose valid. */
    LP_VIO_TRACKING_STATIC = 4,  /**< Pose valid, camera at rest. */
    LP_VIO_TRACKING_LOST   = 5,
    LP_VIO_DATA_LOST       = 6
} lp_vio_status;

/** Payload of an ::LP_STREAM_POSE frame. Only ::LP_VIO_TRACKING and
 *  ::LP_VIO_TRACKING_STATIC carry a pose; in other states the camera sends a
 *  status-only frame (identity pose) at 10 Hz. */
typedef struct {
    double   position[3];     /**< x, y, z in metres, world frame. */
    double   orientation[4];  /**< Quaternion x, y, z, w. */
    int32_t  status;          /**< ::lp_vio_status */
    uint32_t reserved;
} lp_pose_data;

/**
 * One received frame.
 *
 * The frame passed to a callback is only valid during that call. Use
 * lp_frame_clone() to keep it.
 *
 * Timestamps: @c timestamp_ns is the capture time on the camera's monotonic
 * clock, which is shared by all streams of one camera; use it to match frames
 * across streams. @c system_timestamp_ns is the same instant on the camera's
 * wall clock, comparable across cameras and with the host only when the
 * clocks are synchronised (see lp_device_info::time_sync).
 */
typedef struct {
    lp_stream      stream;
    lp_format      format;
    uint32_t       width;         /**< 0 for IMU / pose. */
    uint32_t       height;
    uint32_t       stride;        /**< Bytes per row; 0 if not a raw image. */
    uint32_t       flags;         /**< LP_FRAME_* */
    const uint8_t *data;
    size_t         size;          /**< Bytes at @c data. */
    uint64_t       number;        /**< Per-stream counter, restarts at 0 per session. */
    uint64_t       timestamp_ns;         /**< Capture time, camera monotonic clock. */
    uint64_t       system_timestamp_ns;  /**< Capture time, camera wall clock. */
    uint64_t       right_timestamp_ns;   /**< IR right only: right-eye capture time. */
    uint64_t       reserved[4];
} lp_frame;

/** Called on the stream's receive thread for every complete frame. */
typedef void (*lp_frame_callback)(const lp_frame *frame, void *user);

/** Counters for one stream since it was started. */
typedef struct {
    uint64_t frames;         /**< Complete frames delivered. */
    uint64_t dropped;        /**< Frames lost to missing packets. */
    uint64_t bytes;          /**< UDP payload bytes received. */
    size_t   socket_buffer;  /**< Receive buffer the kernel granted. Below
                                  ~16 MB, image streams drop frames: raise
                                  net.core.rmem_max. */
    uint16_t port;           /**< Local UDP port in use. */
    uint16_t source_port;    /**< Camera UDP port the stream leaves from. The
                                  SDK sends it a small datagram every second,
                                  so a stateful firewall on this host (Windows
                                  Defender, ufw) admits the stream without a
                                  rule. 0: the camera does not report it, and
                                  the firewall must allow the stream. */
} lp_stream_stats;

/** Camera intrinsics of one stream, from the camera's calibration. */
typedef struct {
    uint32_t width;
    uint32_t height;
    double   k[9];   /**< 3x3 camera matrix, row-major. */
    double   d[8];   /**< Distortion coefficients; unused entries are 0. */
    uint32_t d_count;
    double   r[9];   /**< Rectification rotation, row-major. */
    double   p[12];  /**< 3x4 projection matrix after rectification, row-major. */
    char     distortion_model[LP_STR_LEN];
} lp_intrinsics;

/** One rigid transform from the calibration: @c child expressed in
 *  @c parent, as a ROS tf_static entry. */
typedef struct {
    char   parent[64];
    char   child[64];
    double t[3];  /**< Translation, metres. */
    double q[4];  /**< Rotation quaternion x, y, z, w. */
} lp_extrinsic;

/** Opaque handle for an open camera. */
typedef struct lp_device lp_device;

/** "ir_left", "depth", ... Never NULL. */
LP_API const char *lp_stream_name(lp_stream stream);

/** "gray8", "z16", ... Never NULL. */
LP_API const char *lp_format_name(lp_format format);

/**
 * Connect to a camera and open its control session.
 *
 * A background thread then sends a heartbeat every second; the camera stops
 * every stream if it hears nothing for 3 s.
 *
 * @param ip         Camera IPv4 address, e.g. lp_device_info::ip.
 * @param timeout_ms Connect and per-request timeout.
 * @param out        Receives the handle.
 * @return ::LP_ERR_BUSY if another host holds the camera's session,
 *         ::LP_ERR_SOCKET if the camera is unreachable or not in streaming
 *         mode. On failure lp_open_error() describes it.
 */
LP_API lp_status lp_device_open(const char *ip, int timeout_ms, lp_device **out);

/** Same as lp_device_open() with an explicit control port. */
LP_API lp_status lp_device_open_port(const char *ip, uint16_t port, int timeout_ms, lp_device **out);

/** Detail about the last lp_device_open() failure on this thread. */
LP_API const char *lp_open_error(void);

/** Stop every stream, end the session and free the handle. Accepts NULL. */
LP_API void lp_device_close(lp_device *dev);

/** Detail about the most recent failure on this device, or "" if none. */
LP_API const char *lp_device_last_error(const lp_device *dev);

/** 1 while the control session is alive, 0 after it was lost (cable pulled,
 *  camera rebooted). A lost session cannot be resumed: close and reopen. */
LP_API int lp_device_is_connected(const lp_device *dev);

/**
 * Camera description as JSON: product, MTU, stereo geometry and, per stream,
 * whether it is available in the current mode.
 *
 * @param written Receives the JSON length, excluding the NUL; may be NULL.
 * @return ::LP_ERR_NOMEM if @p cap is too small; @p written then still
 *         receives the size needed.
 */
LP_API lp_status lp_device_get_info_json(lp_device *dev, char *out, size_t cap, size_t *written);

/**
 * Full calibration as JSON: intrinsics of ir_left, ir_right and color, and
 * the extrinsics list (parent, child, t, q_xyzw). Same buffer rules as
 * lp_device_get_info_json().
 */
LP_API lp_status lp_device_get_calibration_json(lp_device *dev, char *out, size_t cap, size_t *written);

/**
 * Intrinsics of one image stream, parsed from the calibration.
 *
 * Depth is aligned to the rectified left IR image, so ::LP_STREAM_DEPTH
 * returns ir_left's entry; use its @c p matrix for the rectified focal length
 * and principal point.
 *
 * @return ::LP_ERR_UNAVAILABLE for IMU / pose, or when the camera has no
 *         calibration for the stream.
 */
LP_API lp_status lp_device_get_intrinsics(lp_device *dev, lp_stream stream, lp_intrinsics *out);

/**
 * The extrinsics in the calibration (camera, depth, IMU and colour frames).
 *
 * @param out   Output array.
 * @param max   Capacity of @p out.
 * @param count Receives how many the calibration has; more than @p max
 *              means some were left out.
 */
LP_API lp_status lp_device_get_extrinsics(lp_device *dev, lp_extrinsic *out, size_t max, size_t *count);

/**
 * Start receiving a stream.
 *
 * @param port     Local UDP port, or 0 to let the system choose a free one
 *                 (recommended; it lets several cameras stream to one host).
 * @param callback Called for every complete frame, on the stream's thread.
 * @return ::LP_ERR_UNAVAILABLE if the camera does not produce this stream in
 *         its current mode, ::LP_ERR_INVALID if it is already started.
 */
LP_API lp_status lp_device_start_stream(lp_device *dev, lp_stream stream, uint16_t port,
                                 lp_frame_callback callback, void *user);

/** Stop a stream and join its thread. ::LP_OK if it was not running. */
LP_API lp_status lp_device_stop_stream(lp_device *dev, lp_stream stream);

/** Counters of a started stream; ::LP_ERR_INVALID if it is not running. */
LP_API lp_status lp_device_get_stream_stats(lp_device *dev, lp_stream stream, lp_stream_stats *out);

/** Copy a frame, data included, so it outlives the callback. NULL on OOM. */
LP_API lp_frame *lp_frame_clone(const lp_frame *frame);

/** Free a frame from lp_frame_clone(). Accepts NULL. */
LP_API void lp_frame_release(lp_frame *frame);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* LOOPER_STREAM_H */
