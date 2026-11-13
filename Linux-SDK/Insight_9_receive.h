#ifndef INSIGHT_SDK_H
#define INSIGHT_SDK_H

#include <stdint.h>
#include <stddef.h>
#include <pthread.h>
#include <atomic>


// ==================== Target Device VID/PID ====================
#define VENDOR_ID  0x3652
#define PRODUCT_ID 0x0b5c

// ==================== Camera Configuration ====================
#define CAM_NUM 3
#define HID_NUM 2
#define MAIN_WIDTH   1088
#define MAIN_HEIGHT  1920
#define MAIN_FORMAT  V4L2_PIX_FMT_MJPEG
#define SUB_WIDTH    544
#define SUB_HEIGHT   1281
#define SUB_FORMAT   V4L2_PIX_FMT_GREY
#define DEPTH_WIDTH  544
#define DEPTH_HEIGHT 642
#define DEPTH_FORMAT V4L2_PIX_FMT_Z16
#define FRAME_RATE 30
#define BUFFER_COUNT 8
#define MAX_PATH 1024
#define METADATA_SIZE 258

struct buffer {
    void *start;
    size_t length;
};

struct uvc_frame_info {
    unsigned int width;
    unsigned int height;
    unsigned int intervals[8];
};

struct uvc_format_info {
    unsigned int fcc;
    int frames_num;
    struct uvc_frame_info *frames;
};

struct cam_ctx {
    int fd;
    struct buffer *buffers;
    int meta_fd;
    struct buffer *meta_buffers;
    int meta_buffer_count;
    int buffer_count;
    pthread_t tid;
    int cam_id;
    int width;
    int height;
    int fps;
    unsigned int format;
    int format_num;
    struct uvc_format_info *formats_info;
    uint64_t last_timestamp;
    pthread_mutex_t fd_lock = PTHREAD_MUTEX_INITIALIZER;
};


#pragma pack(push, 1)
typedef struct {
    uint8_t cam_id;             // Camera ID, 0/1
    uint8_t resolution;         // Resolution index
    uint8_t frame_rate;         // Frame-rate index, 0-5
    float exposure_time;        // Exposure time in seconds, range 0.0~0.03
    float exposure_gain;        // Exposure gain, range 1.0~16.0
    uint8_t auto_exposure;      // Auto exposure, 0/1
    float brightness;           // Brightness, range 0.0~127.0
    float contrast;             // Contrast, range 0.0~1.9
    float gamma_dark;           // Dark gamma, range 1.0~4.0
    float hue;                  // Hue, range 0.0~87.0
    float saturation;           // Saturation, range 0.0~1.999
    uint8_t sharpness;          // Sharpness (1~255)
    uint8_t auto_white_balance; // Auto white balance, 0 or 1
    float white_balance;        // White balance, range 1.0~3.0
    uint8_t decimation;         // Decimation (1~255)
    uint8_t hardware_model;     // Hardware model
} camera_params;

/**
 * Camera intrinsics, fields aligned with ROS sensor_msgs/CameraInfo.
 */
typedef struct {
    uint32_t sec;               // Calibration timestamp (seconds)
    uint32_t nsec;              // Calibration timestamp (nanoseconds)

    char frame_id[32];          // Camera frame id

    uint32_t height;            // Calibration image height
    uint32_t width;             // Calibration image width

    char distortion_model[32];  // e.g. "plumb_bob", "equidistant"

    float d[4];                 // Distortion coefficients
    float k[9];                 // 3x3 intrinsic matrix, row-major
    float r[9];                 // 3x3 rectification matrix, row-major
    float p[12];                // 3x4 projection matrix, row-major

    uint32_t binning_x;
    uint32_t binning_y;

    uint32_t roi_x_offset;
    uint32_t roi_y_offset;
    uint32_t roi_height;
    uint32_t roi_width;

    uint8_t roi_do_rectify;
} camera_intrinsics;

/**
 * Camera extrinsics, one transform from the device /tf_static.
 */
typedef struct {
    char parent_frame_id[32];   // Reference frame
    char child_frame_id[32];    // This camera frame

    double translation[3];      // x, y, z (m)
    double rotation[4];         // Quaternion x, y, z, w
} camera_extrinsics;

/**
 * Full calibration of one camera: intrinsics + extrinsics.
 */
typedef struct {
    camera_intrinsics intrinsics;
    camera_extrinsics extrinsics;
} camera_calib;
#pragma pack(pop)

/*
 * Logical camera ids used by image_callback and every cam_id argument below.
 *
 * NOTE: this numbering is the Linux one and is NOT the same as the Windows
 * SDK's (Insight9SDK.h), where RGB is 0 and depth is 2. Code built for both
 * platforms should use these names rather than literals.
 */
enum {
    INSIGHT9_CAM_DEPTH = 0,   /* Z16 depth                                  */
    INSIGHT9_CAM_GRAY  = 1,   /* Stereo grayscale (Y8I), or YUYV/YVYU colour
                               * on units whose stereo interface also offers
                               * a colour format - see
                               * insight9_receive_camera_has_color()         */
    INSIGHT9_CAM_RGB   = 2,   /* Dedicated RGB camera, if the unit has one   */
};

/* Camera index for insight9_receive_get_camera_calib(). */
enum {
    INSIGHT9_CALIB_CAM_LEFT  = 0,   /* Left grayscale  */
    INSIGHT9_CALIB_CAM_RIGHT = 1,   /* Right grayscale */
    INSIGHT9_CALIB_CAM_RGB   = 2,   /* RGB             */
};

enum class PixelFormat {
    Unknown,
    MJPEG,
    GREY,
    Z16,
    RGB8,
    Y8I,
    YUYV,
    YVYU,
    NV12
};

typedef struct {
    int width;
    int height;
    int fps;
    PixelFormat pixel_format;
} video_config_t;

typedef struct {
    video_config_t rgb_config;
    video_config_t gray_config;
    video_config_t depth_config;
} insight9_config_t;

struct DeviceCapability {
    int width;
    int height;
    int fps;
    PixelFormat format;
    bool valid;

    DeviceCapability() : width(0), height(0), fps(0), format(PixelFormat::Unknown), valid(false) {}
};

// VIO status enumeration, matching the device firmware
enum class VioStatus : uint8_t {
    NOT_INITED      = 0,
    RESTARTING      = 1,
    STOPPED         = 2,
    TRACKING        = 3,
    TRACKING_STATIC = 4,
    TRACKINGLOST    = 5,
    DATA_LOST       = 6,
    Unknown         = 0xFF
};
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Image data callback.
 * @param cam_id    Camera ID: 0 depth, 1 grayscale/colour, 2 RGB
 *                  (INSIGHT9_CAM_*). Note the Windows SDK numbers these
 *                  differently.
 * @param data      Image data pointer (JPEG data for MJPEG, raw data for GREY and Z16).
 * @param size      Data size in bytes.
 * @param width     Image width.
 * @param height    Image height.
 * @param format    V4L2 pixel format, such as V4L2_PIX_FMT_MJPEG or V4L2_PIX_FMT_GREY.
 * @param timestamp Image timestamp in microseconds, provided by the device or system time.
 * @param userdata  User pointer passed when the callback is registered.
 */
typedef void (*image_callback)(int cam_id, uint8_t *data, size_t size,
                               int width, int height, unsigned int format,
                               uint64_t timestamp, void *userdata);

/**
 * @brief IMU data callback.
 * @param ax,ay,az  Raw accelerometer values.
 * @param gx,gy,gz  Raw gyroscope values.
 * @param timestamp Timestamp provided by the device.
 * @param userdata  User pointer.
 */
typedef void (*imu_callback)(float ax, float ay, float az,
                             float gx, float gy, float gz,
                             uint64_t timestamp, void *userdata);

/**
 * @brief VIO pose data callback.
 * @param px,py,pz  Position coordinates.
 * @param qx,qy,qz,qw Quaternion orientation.
 * @param seq       Sequence number.
 * @param userdata  User pointer.
 */
typedef void (*vio_callback)(float px, float py, float pz,
                             float qx, float qy, float qz, float qw,
                             uint64_t timestamp, void *userdata);

/**
 * @brief Initialize the SDK with custom configuration.
 * @param config Configuration structure containing resolution, fps, and pixel format for each camera.
 * @return 0 on success, -1 on failure.
 *
 * @note Succeeds even when the device is absent or publishes fewer streams
 *       than CAM_NUM. Streams that are missing are attached later by the
 *       capture threads' reconnect path, so a missing camera is no longer an
 *       initialization failure. Use insight9_receive_has_camera() to find out
 *       what is actually available.
 */
int insight9_receive_init(const insight9_config_t* config);

/**
 * @brief Initialize the SDK with default configuration.
 * @return 0 on success, -1 on failure.
 *
 * @note Same tolerance for missing streams as insight9_receive_init().
 *       Per-camera geometry is taken from whatever format the device currently
 *       reports (VIDIOC_G_FMT), so the initial state is device-determined.
 */
int insight9_receive_init_default(void);

/**
 * @brief Start all capture threads.
 * @return 0 on success, -1 on failure.
 *
 * @note A thread is started for every logical camera, including ones with no
 *       device node yet; those sit in the reconnect backoff and begin
 *       delivering frames if and when their stream appears.
 */
int insight9_receive_start(void);

/**
 * @brief Get current video format for a specific camera device.
 * @param cam_id Camera ID (0..2).
 * @param width Output width.
 * @param height Output height.
 * @param format Output V4L2 pixel format.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_get_current_format(int cam_id, int *width, int *height, unsigned int *format);

/**
 * @brief Get the video device path for the specified camera.
 * @param cam_id Camera index (0..2).
 * @return Device path string, or NULL on failure.
 */
const char *insight9_receive_get_video_dev(int cam_id);

/**
 * @brief Get the metadata device path for the specified camera.
 * @param cam_id Camera index (0..2).
 * @return Device path string, or NULL on failure.
 */
const char *insight9_receive_get_metadata_dev(int cam_id);

/**
 * @brief Read metadata timestamp from the metadata device.
 * @param cam_id Camera index.
 * @param timestamp Output parameter for the timestamp.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_read_metadata_timestamp(int cam_id, uint64_t *timestamp);

/**
 * @brief Start a specific camera.
 * @param cam_id Camera ID (INSIGHT9_CAM_*).
 * @return 0 on success, -1 on failure. Fails if the stream is not present on
 *         this hardware; a re-enumeration is attempted first.
 */
int insight9_receive_start_camera(int cam_id);

/**
 * @brief Stop a specific camera.
 * @param cam_id Camera ID.
 */
void insight9_receive_stop_camera(int cam_id);

/**
 * @brief Get the current configuration (resolution, pixel format, and frame rate) of the specified camera.
 * @param cam_id Camera ID (0: Depth, 1: Grayscale, 2: RGB on Linux; may vary by platform).
 * @param width  Output pointer to receive the current width.
 * @param height Output pointer to receive the current height.
 * @param format Output pointer to receive the current pixel format.
 * @param fps    Output pointer to receive the current frame rate.
 * @return 0 on success, -1 on failure (e.g., invalid cam_id or NULL pointer).
 */
int insight9_receive_get_camera_config(int cam_id, int* width, int* height, PixelFormat* format, int* fps);

/**
 * @brief Restart a specific camera (stop and start again).
 * @param cam_id Camera ID.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_restart_camera(int cam_id);

/**
 * @brief Switch a specific camera to a new frame rate and apply it immediately.
 *        Convenience wrapper that updates the target fps (as
 *        insight9_receive_set_camera_fps() does) and then stops/reopens
 *        just that one camera (as insight9_receive_restart_camera() does)
 *        so the new value actually takes effect.
 * @param cam_id Camera ID.
 * @param fps    Desired frame rate.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_switch_camera_fps(int cam_id, int fps);

/**
 * @brief Set the resolution/format for a specific camera (stored in config, requires restart to take effect).
 * @param cam_id Camera ID.
 * @param width  Target width.
 * @param height Target height.
 * @param format Target pixel format.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_set_camera_format(int cam_id, int width, int height, PixelFormat format);

/**
 * @brief Immediately switch the specified camera to a new configuration (resolution, format, and frame rate).
 *        This function stops the camera, updates the stored configuration, and restarts the camera with the new settings.
 * @param cam_id Camera ID (0: Depth, 1: Grayscale, 2: RGB on Linux; may vary by platform).
 * @param width  Desired width.
 * @param height Desired height.
 * @param format Desired pixel format.
 * @param fps    Desired frame rate.
 * @return 0 on success, -1 on failure (e.g., invalid parameters, unable to set format/fps, or restart failure).
 */
int insight9_receive_switch_camera_config(int cam_id, int width, int height, PixelFormat format, int fps);

/**
 * @brief Switch a specific camera to a new resolution/format and apply it immediately.
 * @param cam_id Camera ID.
 * @param width  Target width.
 * @param height Target height.
 * @param format Target pixel format.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_switch_camera_format(int cam_id, int width, int height, PixelFormat format);

/**
 * @brief Check if a camera is currently running.
 * @param cam_id Camera ID.
 * @return 1 if running, 0 otherwise.
 */
int insight9_receive_is_camera_running(int cam_id);

/**
 * @brief Check whether this camera is attached to a real device node.
 *
 * Not every unit publishes all CAM_NUM streams: an Insight 3u has no separate
 * RGB interface, so INSIGHT9_CAM_RGB never attaches on it. This also returns 0
 * for a stream that exists on the hardware but has not been enumerated yet
 * (device not plugged in at init, or mid-reconnect), so callers that want to
 * distinguish "this model has no such camera" from "not connected yet" should
 * also look at insight9_receive_get_hardware_type().
 *
 * @param cam_id Camera ID (INSIGHT9_CAM_*).
 * @return 1 if attached, 0 otherwise.
 */
int insight9_receive_has_camera(int cam_id);

/**
 * @brief Check whether this camera's interface also offers a colour format.
 *
 * On some units the stereo interface advertises YUYV/YVYU in addition to Y8I,
 * so INSIGHT9_CAM_GRAY can stream either grayscale or colour - one at a time,
 * since it is one interface. Switching is a normal
 * insight9_receive_switch_camera_config() call with the colour format.
 *
 * Note the frame layout differs per format and the caller must account for it:
 * Y8I interleaves left/right per byte (mono height == frame height), while
 * YUYV/YVYU stack them top-to-bottom (mono height == frame height / 2).
 *
 * @param cam_id Camera ID (INSIGHT9_CAM_*).
 * @return 1 if a colour format is available on this camera, 0 otherwise
 *         (including when the camera is not attached).
 */
int insight9_receive_camera_has_color(int cam_id);

/**
 * @brief Stop all capture threads.
 */
void insight9_receive_stop(void);

/**
 * @brief Release all resources. Must be called after stopping.
 */
void insight9_receive_cleanup(void);

/**
 * @brief Register the image callback.
 * @param cb       Callback function.
 * @param userdata User pointer passed through to the callback.
 */
void insight9_receive_register_image_callback(image_callback cb, void *userdata);

/**
 * @brief Register the IMU callback.
 */
void insight9_receive_register_imu_callback(imu_callback cb, void *userdata);

/**
 * @brief Register the VIO callback.
 */
void insight9_receive_register_vio_callback(vio_callback cb, void *userdata);

/**
 * @brief Set the frame rate for a specific camera (stored in config, requires restart).
 * @param cam_id Camera ID.
 * @param fps Desired frame rate.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_set_camera_fps(int cam_id, int fps);

/**
 * @brief Set the currently active camera.
 * @param cam_id Camera ID (0: RGB, 1: stereo grayscale).
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_set_active_camera(int cam_id);

/**
 * @brief Read the currently active camera.
 * @param cam_id Output parameter that receives the active camera ID.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_get_active_camera(int *cam_id);

/**
 * @brief Check if the provided camera parameters are valid.
 * @param params Pointer to camera_params structure to validate.
 * @return 0 if valid, -1 if invalid.
 */
int insight9_check_camera_params(const camera_params *params);

/**
 * @brief Set all parameters for the currently active camera.
 * @param params Parameters to set. Must include the cam_id field indicating the target camera.
 * @return 0 on success, -1 on failure.
 * @note params->cam_id is ignored; the currently active camera ID is used. Prefer set_camera_params_for to specify a camera ID.
 */
int insight9_receive_set_camera_params(const camera_params *params);

/**
 * @brief Read all parameters for the currently active camera.
 * @param params Output parameter that receives the read parameters.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_get_camera_params(camera_params *params);

/**
 * @brief Set all parameters for the specified camera.
 * @param cam_id Camera ID (0/1/2).
 * @param params Parameters to set. params->cam_id is ignored; cam_id is used.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_set_camera_params_for(int cam_id, const camera_params *params);

/**
 * @brief Read all parameters for the specified camera.
 * @param cam_id Camera ID.
 * @param params Output parameter that receives the read parameters.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_get_camera_params_for(int cam_id, camera_params *params);

/**
 * @brief Get the cached initial parameters for the specified camera.
 * @param cam_id Camera ID.
 * @param params Output parameter that receives the cached initial parameters.
 * @return 0 on success, -1 on failure (e.g., no cached parameters available).
 */
int insight9_receive_get_cached_initial_params(int cam_id, camera_params* params);

/**
 * @brief Restore the specified camera to its initial values.
 * @param cam_id Camera ID.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_reset_camera_params(int cam_id);

/**
 * @brief Print camera parameters to stdout
 * @param params Pointer to camera_params
 */
void insight9_receive_print_camera_params(const camera_params *params);

/**
 * @brief Read the intrinsics and extrinsics of the specified camera.
 * @param cam_idx Camera index: INSIGHT9_CALIB_CAM_LEFT / RIGHT / RGB.
 * @param calib   Output parameter that receives the calibration data.
 * @return 0 on success, -1 on failure (e.g. firmware without calibration support).
 */
int insight9_receive_get_camera_calib(int cam_idx, camera_calib *calib);

/**
 * @brief Print camera calibration data to stdout.
 * @param calib Pointer to camera_calib.
 */
void insight9_receive_print_camera_calib(const camera_calib *calib);

/**
 * @brief Align a depth image (registered to the LEFT camera) onto the RGB image
 *        plane, producing a depth map registered to (and the same size as) the
 *        RGB image.
 *
 * The depth stream is the left grayscale camera's depth map, so each depth pixel
 * is deprojected with the LEFT intrinsics into the left camera frame, transformed
 * into the RGB frame, then reprojected with the RGB intrinsics. The RGB camera's
 * extrinsic (parent=camera_camera_left, child=camera_camera_rgb) maps a point
 * rgb->left, so its inverse is used for left->rgb. Both LEFT and RGB streams are
 * rectified (p == k), so no lens distortion is applied. The result is a forward
 * warp with z-buffering (nearest surface wins); RGB pixels with no corresponding
 * depth sample remain 0 (invalid).
 *
 * @param depth       Input depth buffer (uint16, row-major), 1 unit = 1 mm.
 * @param depth_w     Depth width  (valid rows only; exclude the metadata rows).
 * @param depth_h     Depth height (valid rows only).
 * @param left_calib  LEFT camera calibration (its intrinsics deproject depth).
 * @param rgb_calib   RGB camera calibration (intrinsics + left->rgb extrinsic).
 * @param aligned_out Output buffer (uint16, row-major), size rgb_w*rgb_h. The
 *                    caller allocates it; the function clears it to 0 first.
 * @param rgb_w       Output (RGB) width.  Should match rgb_calib->intrinsics.width.
 * @param rgb_h       Output (RGB) height. Should match rgb_calib->intrinsics.height.
 * @return 0 on success, -1 on invalid arguments.
 */
int insight9_receive_align_depth_to_rgb(const uint16_t *depth,
                                        int depth_w, int depth_h,
                                        const camera_calib *left_calib,
                                        const camera_calib *rgb_calib,
                                        uint16_t *aligned_out,
                                        int rgb_w, int rgb_h);

/**
 * @brief Get the current frame rate.
 * @param fps Pointer to store the current frame rate.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_get_current_fps(int* fps);

/**
 * @brief Get the VIO status.
 * @param status Pointer to store the VIO status value.
 * @return 0 on success, -1 on failure.
 */
int insight9_receive_get_vio_status(int* status);

/**
 * @brief Get the hardware type/model as a string. This requires reading the current camera parameters to determine the hardware_model field, which is then mapped to a string.
 * @return Hardware type/model string, or "unknown" on failure.
 */
const char* insight9_receive_get_hardware_type(void);

#ifdef __cplusplus
int insight9_receive_get_device_capability_count(int cam_id, int* count);
int insight9_receive_get_device_capability_by_index(int cam_id, int index, DeviceCapability* cap);
#endif

/* ==================== Point cloud ==================== */

/**
 * One point of a point cloud.
 *
 * 16 bytes, laid out like pcl::PointXYZRGB and like a ROS PointCloud2 with
 * fields x/y/z/rgb (FLOAT32) at offsets 0/4/8/12 and point_step 16, so an
 * array of these can be memcpy'd into either directly.
 */
typedef struct {
    float    x, y, z;   /* metres                                           */
    uint32_t rgb;       /* 0x00RRGGBB; 0 when the point has no colour       */
} insight9_point;

/* Colour source for insight9_receive_depth_to_pointcloud(). */
enum {
    INSIGHT9_PC_COLOR_NONE  = 0,  /* geometry only, rgb = 0                   */
    INSIGHT9_PC_COLOR_GRAY8 = 1,  /* 8-bit image pixel-aligned with the depth
                                   * map, i.e. the LEFT grayscale eye         */
    INSIGHT9_PC_COLOR_RGB8  = 2,  /* decoded RGB-camera image, bytes R,G,B    */
    INSIGHT9_PC_COLOR_BGR8  = 3,  /* decoded RGB-camera image, bytes B,G,R
                                   * (what cv::imdecode returns)              */
};

typedef struct {
    int   pixel_step;          /* Use every Nth depth pixel in u and v (>= 1)  */
    float min_depth;           /* Keep z >  min_depth (metres)                 */
    float max_depth;           /* Keep z <= max_depth (metres); <= 0: no limit */

    /* Optional colour source. */
    int            color_format;       /* INSIGHT9_PC_COLOR_*                     */
    const uint8_t *color;              /* First pixel of the image                 */
    int            color_width;        /* Image width  in pixels                   */
    int            color_height;       /* Image height in pixels                   */
    int            color_row_stride;   /* Bytes per row;   0 = width*pixel_stride  */
    int            color_pixel_stride; /* Bytes per pixel; 0 = 1 (GRAY8) / 3 (RGB) */
    const camera_calib *rgb_calib;     /* Required for RGB8/BGR8: intrinsics plus
                                        * the rgb->left extrinsic. The image size
                                        * must match rgb_calib->intrinsics.        */
    int keep_uncolored;                /* With RGB8/BGR8, points that fall outside
                                        * the RGB image are dropped (0, default) or
                                        * kept with rgb = 0 (1).                   */

    /* Optional 4x4 row-major transform applied to every output point. It maps
     * points from the LEFT camera optical frame into the frame you want (for
     * example T_world_left from insight9_pointcloud_pose_to_transform()).
     * NULL: points stay in the LEFT camera optical frame (x right, y down,
     * z forward). */
    const float *transform;
} insight9_pointcloud_params;

/**
 * @brief Fill params with defaults: pixel_step 2, min_depth 0, max_depth 2.0 m,
 *        no colour, no transform.
 */
void insight9_pointcloud_default_params(insight9_pointcloud_params *params);

/**
 * @brief Upper bound on the number of points one call can produce, i.e. the
 *        capacity to allocate for insight9_receive_depth_to_pointcloud().
 * @return 0 if any argument is <= 0.
 */
size_t insight9_pointcloud_max_points(int depth_w, int depth_h, int pixel_step);

/**
 * @brief Back-project a depth image into a point cloud.
 *
 * The depth stream is registered to the LEFT grayscale camera, so each sampled
 * depth pixel is deprojected with the LEFT intrinsics. Streams are rectified
 * (p == k), so no lens distortion is applied. With RGB8/BGR8 colour, each
 * point is transformed into the RGB frame (inverse of the rgb->left extrinsic)
 * and projected with the RGB intrinsics to pick its colour.
 *
 * @param depth        Depth buffer (uint16, row-major), 1 unit = 1 mm.
 * @param depth_w      Depth width.
 * @param depth_h      Depth height. Valid rows only: exclude the 2 metadata
 *                     rows at the bottom of the Z16 frame.
 * @param left_calib   LEFT camera calibration.
 * @param params       Options; NULL uses insight9_pointcloud_default_params().
 * @param out          Output points. Caller allocates at least
 *                     insight9_pointcloud_max_points(depth_w, depth_h,
 *                     params->pixel_step) entries.
 * @param out_capacity Number of entries in out.
 * @param out_count    Receives the number of points written.
 * @return 0 on success, -1 on invalid arguments or too small a buffer.
 */
int insight9_receive_depth_to_pointcloud(const uint16_t *depth,
                                         int depth_w, int depth_h,
                                         const camera_calib *left_calib,
                                         const insight9_pointcloud_params *params,
                                         insight9_point *out,
                                         size_t out_capacity,
                                         size_t *out_count);

/**
 * @brief Build the 4x4 row-major transform T_world_left from a VIO pose.
 *
 * T_world_left = T_world_body * T_body_left, where T_world_body is the pose
 * delivered by vio_callback and T_body_left is the fixed transform from the
 * LEFT camera to the VIO body frame. Pass the result as
 * insight9_pointcloud_params::transform to get world-frame points.
 *
 * @param px,py,pz,qx,qy,qz,qw VIO pose (as from vio_callback).
 * @param body_to_left  ROS-convention transform with parent = VIO body frame
 *                      and child = LEFT camera (p_body = R * p_left + t).
 *                      NULL treats the pose as the LEFT camera pose itself.
 * @param T_out         Receives 16 floats, row-major.
 * @return 0 on success, -1 on invalid arguments.
 */
int insight9_pointcloud_pose_to_transform(float px, float py, float pz,
                                          float qx, float qy, float qz, float qw,
                                          const camera_extrinsics *body_to_left,
                                          float T_out[16]);

/*
 * Global point cloud map: accumulates world-frame frames into one cloud.
 *
 * A frame is kept only as a keyframe, when the camera has moved at least
 * keyframe_translation or turned at least keyframe_rotation_deg since the last
 * keyframe. Each keyframe is voxel-downsampled on its own. Keyframes taken
 * farther than keep_radius from the current camera position are dropped, and
 * insight9_pointcloud_map_get() returns only points within keep_radius of it.
 * All map functions are thread-safe for the same map.
 */
typedef struct insight9_pointcloud_map insight9_pointcloud_map;

typedef struct {
    float keyframe_translation;   /* metres;  default 0.03                   */
    float keyframe_rotation_deg;  /* degrees; default 1.0                    */
    float voxel_size;             /* metres;  default 0.05; <= 0 disables it */
    float keep_radius;            /* metres;  default 100;  <= 0: no limit   */
} insight9_pointcloud_map_params;

void insight9_pointcloud_map_default_params(insight9_pointcloud_map_params *params);

/**
 * @brief Create a map. params NULL uses the defaults.
 * @return Map handle, or NULL on failure. Free with insight9_pointcloud_map_destroy().
 */
insight9_pointcloud_map *insight9_pointcloud_map_create(const insight9_pointcloud_map_params *params);

void insight9_pointcloud_map_destroy(insight9_pointcloud_map *map);

/** @brief Drop every point and keyframe. */
void insight9_pointcloud_map_clear(insight9_pointcloud_map *map);

/**
 * @brief Offer one frame to the map.
 * @param points       World-frame points, i.e. produced with
 *                     params.transform = T_world_left.
 * @param count        Number of points.
 * @param T_world_left The same 4x4 row-major camera pose.
 * @return 1 if the frame was added as a keyframe, 0 if it was skipped,
 *         -1 on invalid arguments.
 */
int insight9_pointcloud_map_insert(insight9_pointcloud_map *map,
                                   const insight9_point *points, size_t count,
                                   const float T_world_left[16]);

/**
 * @brief Copy the current global cloud.
 * @param out          Destination, or NULL to only query the point count.
 * @param out_capacity Number of entries in out.
 * @param out_count    Receives the number of points written, or with out ==
 *                     NULL the number available.
 * @return 0 on success, 1 if out was too small and the cloud was truncated,
 *         -1 on invalid arguments.
 */
int insight9_pointcloud_map_get(insight9_pointcloud_map *map,
                                insight9_point *out, size_t out_capacity,
                                size_t *out_count);


#ifdef __cplusplus
}
#endif

#endif // INSIGHT_SDK_H
