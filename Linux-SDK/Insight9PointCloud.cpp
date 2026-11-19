// Depth -> point cloud conversion and global point cloud map.
//
// This file is identical in every platform SDK directory; only the header
// name differs, which is selected below.

#if defined(__linux__)
#include "Insight_9_receive.h"
#else
#include "Insight9SDK.h"
#endif

#include <math.h>
#include <string.h>

#include <deque>
#include <mutex>
#include <new>
#include <unordered_set>
#include <vector>

namespace {

// Row-major 3x3 rotation from a quaternion (x, y, z, w). The quaternion is
// normalized first; returns false for a zero quaternion.
bool quat_to_rot(double qx, double qy, double qz, double qw, double R[9]) {
    double n = sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (!(n > 1e-12)) return false;
    qx /= n; qy /= n; qz /= n; qw /= n;
    R[0] = 1 - 2 * (qy * qy + qz * qz);
    R[1] = 2 * (qx * qy - qz * qw);
    R[2] = 2 * (qx * qz + qy * qw);
    R[3] = 2 * (qx * qy + qz * qw);
    R[4] = 1 - 2 * (qx * qx + qz * qz);
    R[5] = 2 * (qy * qz - qx * qw);
    R[6] = 2 * (qx * qz - qy * qw);
    R[7] = 2 * (qy * qz + qx * qw);
    R[8] = 1 - 2 * (qx * qx + qy * qy);
    return true;
}

inline uint32_t pack_rgb(uint32_t r, uint32_t g, uint32_t b) {
    return (r << 16) | (g << 8) | b;
}

inline bool within_radius(const insight9_point &p, const float c[3], float r2) {
    float dx = p.x - c[0], dy = p.y - c[1], dz = p.z - c[2];
    return dx * dx + dy * dy + dz * dz <= r2;
}

}  // namespace

// ==================== Single frame ====================

void insight9_pointcloud_default_params(insight9_pointcloud_params *params) {
    if (!params) return;
    memset(params, 0, sizeof(*params));
    params->pixel_step = 2;
    params->min_depth = 0.0f;
    params->max_depth = 2.0f;
    params->color_format = INSIGHT9_PC_COLOR_NONE;
}

size_t insight9_pointcloud_max_points(int depth_w, int depth_h, int pixel_step) {
    if (depth_w <= 0 || depth_h <= 0 || pixel_step <= 0) return 0;
    size_t cols = (size_t)((depth_w + pixel_step - 1) / pixel_step);
    size_t rows = (size_t)((depth_h + pixel_step - 1) / pixel_step);
    return cols * rows;
}

int insight9_receive_depth_to_pointcloud(const uint16_t *depth,
                                         int depth_w, int depth_h,
                                         const camera_calib *left_calib,
                                         const insight9_pointcloud_params *params,
                                         insight9_point *out,
                                         size_t out_capacity,
                                         size_t *out_count) {
    if (out_count) *out_count = 0;
    if (!depth || !left_calib || !out || !out_count ||
        depth_w <= 0 || depth_h <= 0) {
        return -1;
    }

    insight9_pointcloud_params p;
    if (params) {
        p = *params;
    } else {
        insight9_pointcloud_default_params(&p);
    }
    if (p.pixel_step < 1) return -1;
    if (out_capacity < insight9_pointcloud_max_points(depth_w, depth_h, p.pixel_step)) {
        return -1;
    }

    // LEFT intrinsics (row-major k = [fx 0 cx; 0 fy cy; 0 0 1]).
    const double dfx = left_calib->intrinsics.k[0];
    const double dfy = left_calib->intrinsics.k[4];
    const double dcx = left_calib->intrinsics.k[2];
    const double dcy = left_calib->intrinsics.k[5];
    if (dfx == 0.0 || dfy == 0.0) return -1;

    // ---- Colour source ----
    const int fmt = p.color_format;
    const bool use_gray = (fmt == INSIGHT9_PC_COLOR_GRAY8);
    const bool use_rgb = (fmt == INSIGHT9_PC_COLOR_RGB8 || fmt == INSIGHT9_PC_COLOR_BGR8);
    if (fmt != INSIGHT9_PC_COLOR_NONE && !use_gray && !use_rgb) return -1;

    int px_stride = 0, row_stride = 0;
    if (use_gray || use_rgb) {
        if (!p.color || p.color_width <= 0 || p.color_height <= 0) return -1;
        px_stride = p.color_pixel_stride > 0 ? p.color_pixel_stride : (use_gray ? 1 : 3);
        if (use_rgb && px_stride < 3) return -1;
        row_stride = p.color_row_stride > 0 ? p.color_row_stride : p.color_width * px_stride;
        if (row_stride < (p.color_width - 1) * px_stride + (use_gray ? 1 : 3)) return -1;
    }
    // The grayscale eye must be pixel-aligned with the depth map.
    if (use_gray && (p.color_width != depth_w || p.color_height != depth_h)) return -1;

    // RGB extrinsic: parent = LEFT, child = RGB; in ROS convention it maps a
    // point rgb->left: p_left = R*p_rgb + t. For left->rgb use the inverse,
    // p_rgb = R^T * (p_left - t). Same convention as align_depth_to_rgb.
    double Rc[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double tc[3] = {0, 0, 0};
    double rfx = 0, rfy = 0, rcx = 0, rcy = 0;
    const int r_off = (fmt == INSIGHT9_PC_COLOR_BGR8) ? 2 : 0;
    const int b_off = (fmt == INSIGHT9_PC_COLOR_BGR8) ? 0 : 2;
    if (use_rgb) {
        if (!p.rgb_calib) return -1;
        rfx = p.rgb_calib->intrinsics.k[0];
        rfy = p.rgb_calib->intrinsics.k[4];
        rcx = p.rgb_calib->intrinsics.k[2];
        rcy = p.rgb_calib->intrinsics.k[5];
        if (rfx == 0.0 || rfy == 0.0) return -1;
        const double *q = p.rgb_calib->extrinsics.rotation;
        if (!quat_to_rot(q[0], q[1], q[2], q[3], Rc)) return -1;
        for (int i = 0; i < 3; i++) tc[i] = p.rgb_calib->extrinsics.translation[i];
    }

    // ---- Output transform ----
    const float *T = p.transform;

    const float min_d = p.min_depth;
    const float max_d = p.max_depth;
    const int step = p.pixel_step;
    size_t n = 0;

    for (int v = 0; v < depth_h; v += step) {
        const uint16_t *row = depth + (size_t)v * depth_w;
        for (int u = 0; u < depth_w; u += step) {
            uint16_t d = row[u];
            if (d == 0) continue;                       // no measurement
            float z = (float)d * 0.001f;                // mm -> m
            if (!(z > min_d)) continue;
            if (max_d > 0.0f && z > max_d) continue;

            float x = (float)((u - dcx) * z / dfx);
            float y = (float)((v - dcy) * z / dfy);

            uint32_t rgb = 0;
            bool colored = true;
            if (use_gray) {
                uint32_t g = p.color[(size_t)v * row_stride + (size_t)u * px_stride];
                rgb = pack_rgb(g, g, g);
            } else if (use_rgb) {
                colored = false;
                double ax = x - tc[0], ay = y - tc[1], az = z - tc[2];
                double xr = Rc[0] * ax + Rc[3] * ay + Rc[6] * az;
                double yr = Rc[1] * ax + Rc[4] * ay + Rc[7] * az;
                double zr = Rc[2] * ax + Rc[5] * ay + Rc[8] * az;
                if (zr > 1e-6) {
                    int cu = (int)floor(rfx * xr / zr + rcx + 0.5);
                    int cv = (int)floor(rfy * yr / zr + rcy + 0.5);
                    if (cu >= 0 && cu < p.color_width && cv >= 0 && cv < p.color_height) {
                        const uint8_t *px = p.color + (size_t)cv * row_stride + (size_t)cu * px_stride;
                        rgb = pack_rgb(px[r_off], px[1], px[b_off]);
                        colored = true;
                    }
                }
                if (!colored && !p.keep_uncolored) continue;
            }

            insight9_point &o = out[n++];
            if (T) {
                o.x = T[0] * x + T[1] * y + T[2] * z + T[3];
                o.y = T[4] * x + T[5] * y + T[6] * z + T[7];
                o.z = T[8] * x + T[9] * y + T[10] * z + T[11];
            } else {
                o.x = x; o.y = y; o.z = z;
            }
            o.rgb = rgb;
        }
    }
    *out_count = n;
    return 0;
}

int insight9_pointcloud_pose_to_transform(float px, float py, float pz,
                                          float qx, float qy, float qz, float qw,
                                          const camera_extrinsics *body_to_left,
                                          float T_out[16]) {
    if (!T_out) return -1;

    double Rw[9];
    if (!quat_to_rot(qx, qy, qz, qw, Rw)) return -1;
    double tw[3] = {px, py, pz};

    double Rb[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double tb[3] = {0, 0, 0};
    if (body_to_left) {
        const double *q = body_to_left->rotation;
        if (!quat_to_rot(q[0], q[1], q[2], q[3], Rb)) return -1;
        for (int i = 0; i < 3; i++) tb[i] = body_to_left->translation[i];
    }

    // T_world_left = [Rw tw] * [Rb tb] = [Rw*Rb, Rw*tb + tw]
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            T_out[r * 4 + c] = (float)(Rw[r * 3 + 0] * Rb[0 * 3 + c] +
                                       Rw[r * 3 + 1] * Rb[1 * 3 + c] +
                                       Rw[r * 3 + 2] * Rb[2 * 3 + c]);
        }
        T_out[r * 4 + 3] = (float)(Rw[r * 3 + 0] * tb[0] + Rw[r * 3 + 1] * tb[1] +
                                   Rw[r * 3 + 2] * tb[2] + tw[r]);
    }
    T_out[12] = 0.0f; T_out[13] = 0.0f; T_out[14] = 0.0f; T_out[15] = 1.0f;
    return 0;
}

// ==================== Global map ====================

struct insight9_pointcloud_map {
    struct Keyframe {
        float position[3];
        std::vector<insight9_point> points;
    };

    std::mutex mutex;
    insight9_pointcloud_map_params params;
    std::deque<Keyframe> keyframes;

    bool has_keyframe_pose = false;
    float keyframe_pose[16];        // pose of the most recent keyframe

    bool has_position = false;
    float position[3];              // camera position of the most recent insert
};

void insight9_pointcloud_map_default_params(insight9_pointcloud_map_params *params) {
    if (!params) return;
    params->keyframe_translation = 0.03f;
    params->keyframe_rotation_deg = 1.0f;
    params->voxel_size = 0.05f;
    params->keep_radius = 100.0f;
}

insight9_pointcloud_map *insight9_pointcloud_map_create(const insight9_pointcloud_map_params *params) {
    insight9_pointcloud_map *map = new (std::nothrow) insight9_pointcloud_map();
    if (!map) return nullptr;
    if (params) {
        map->params = *params;
    } else {
        insight9_pointcloud_map_default_params(&map->params);
    }
    return map;
}

void insight9_pointcloud_map_destroy(insight9_pointcloud_map *map) {
    delete map;
}

void insight9_pointcloud_map_clear(insight9_pointcloud_map *map) {
    if (!map) return;
    std::lock_guard<std::mutex> lock(map->mutex);
    map->keyframes.clear();
    map->has_keyframe_pose = false;
    map->has_position = false;
}

static bool map_is_keyframe(const insight9_pointcloud_map *map, const float *T) {
    if (!map->has_keyframe_pose) return true;
    const float *L = map->keyframe_pose;

    double dx = T[3] - L[3], dy = T[7] - L[7], dz = T[11] - L[11];
    double translation = sqrt(dx * dx + dy * dy + dz * dz);

    // trace(R_last^T * R) = sum of element-wise products of the two rotations.
    double trace = 0.0;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            trace += (double)L[r * 4 + c] * T[r * 4 + c];
    double cos_a = (trace - 1.0) * 0.5;
    if (cos_a > 1.0) cos_a = 1.0;
    if (cos_a < -1.0) cos_a = -1.0;
    double angle_deg = acos(cos_a) * (180.0 / 3.14159265358979323846);

    return translation >= map->params.keyframe_translation ||
           angle_deg >= map->params.keyframe_rotation_deg;
}

// Keep the first point that falls into each voxel, preserving input order.
static void voxel_downsample(std::vector<insight9_point> &pts, float voxel) {
    if (voxel <= 0.0f || pts.empty()) return;
    const float inv = 1.0f / voxel;
    const int64_t kBias = (int64_t)1 << 20;     // 21 bits per axis
    const int64_t kMask = ((int64_t)1 << 21) - 1;
    std::unordered_set<uint64_t> seen;
    seen.reserve(pts.size() * 2);
    size_t w = 0;
    for (size_t i = 0; i < pts.size(); i++) {
        const insight9_point &pt = pts[i];
        int64_t ix = (int64_t)floorf(pt.x * inv) + kBias;
        int64_t iy = (int64_t)floorf(pt.y * inv) + kBias;
        int64_t iz = (int64_t)floorf(pt.z * inv) + kBias;
        uint64_t key = ((uint64_t)(ix & kMask) << 42) |
                       ((uint64_t)(iy & kMask) << 21) |
                        (uint64_t)(iz & kMask);
        if (seen.insert(key).second) pts[w++] = pt;
    }
    pts.resize(w);
}

int insight9_pointcloud_map_insert(insight9_pointcloud_map *map,
                                   const insight9_point *points, size_t count,
                                   const float T_world_left[16]) {
    if (!map || !T_world_left || (count > 0 && !points)) return -1;

    std::lock_guard<std::mutex> lock(map->mutex);
    const float *T = T_world_left;
    map->position[0] = T[3];
    map->position[1] = T[7];
    map->position[2] = T[11];
    map->has_position = true;

    if (!map_is_keyframe(map, T)) return 0;

    const float radius = map->params.keep_radius;
    const float r2 = radius * radius;

    insight9_pointcloud_map::Keyframe kf;
    memcpy(kf.position, map->position, sizeof(kf.position));
    kf.points.reserve(count);
    for (size_t i = 0; i < count; i++) {
        if (radius > 0.0f && !within_radius(points[i], map->position, r2)) continue;
        kf.points.push_back(points[i]);
    }
    voxel_downsample(kf.points, map->params.voxel_size);
    map->keyframes.push_back(std::move(kf));

    memcpy(map->keyframe_pose, T, sizeof(map->keyframe_pose));
    map->has_keyframe_pose = true;

    // Drop keyframes taken too far from the current position.
    if (radius > 0.0f) {
        std::deque<insight9_pointcloud_map::Keyframe> kept;
        for (auto &k : map->keyframes) {
            float dx = k.position[0] - map->position[0];
            float dy = k.position[1] - map->position[1];
            float dz = k.position[2] - map->position[2];
            if (dx * dx + dy * dy + dz * dz <= r2) kept.push_back(std::move(k));
        }
        map->keyframes.swap(kept);
    }
    return 1;
}

int insight9_pointcloud_map_get(insight9_pointcloud_map *map,
                                insight9_point *out, size_t out_capacity,
                                size_t *out_count) {
    if (out_count) *out_count = 0;
    if (!map || !out_count) return -1;

    std::lock_guard<std::mutex> lock(map->mutex);
    const float radius = map->params.keep_radius;
    const bool crop = radius > 0.0f && map->has_position;
    const float r2 = radius * radius;

    size_t n = 0;
    bool truncated = false;
    for (const auto &k : map->keyframes) {
        for (const auto &pt : k.points) {
            if (crop && !within_radius(pt, map->position, r2)) continue;
            if (out) {
                if (n >= out_capacity) { truncated = true; break; }
                out[n] = pt;
            }
            n++;
        }
        if (truncated) break;
    }
    *out_count = n;
    return truncated ? 1 : 0;
}
