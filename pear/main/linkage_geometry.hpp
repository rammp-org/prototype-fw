#pragma once

/// PEAR corner-linkage geometry: pure functions (no ESP-IDF), host-testable
/// (see test/linkage_geometry_test.cpp).
///
/// Each wheel module hangs on a motor-driven 4-bar linkage at one corner of the
/// base. The linkage is assumed to be a PARALLELOGRAM 4-bar: the coupler (the
/// wheel-module mount) stays parallel to the base while the crank swings, so
/// the module's height is a pure function of the crank angle,
///
///     h(a) = crank_length * sin(a - level_angle)
///
/// with `level_angle` the crank angle at which the module sits at its
/// reference height (h = 0, "level"). Angles are in degrees at the motor
/// output shaft (positive = module up), heights in metres. The small
/// fore/aft shift of the wheel contact point along the crank's arc is ignored
/// (fixed-footprint assumption; the drive kinematics use the nominal
/// half-length / half-width).
///
/// Base pose <-> corner heights. The base frame is x forward, y LEFT, z up
/// (right-handed). A pose is (elevation, front-back tilt, lateral tilt):
///
///   * elevation_m:     mean corner height (0 = all linkages level)
///   * fb_tilt_deg:     FRONT UP is positive
///   * lateral_tilt_deg: LEFT UP is positive
///
///     h_corner = e + sx * half_length * tan(fb) + sy * half_width * tan(lat)
///
/// with sx = +1 front / -1 back and sy = +1 left / -1 right. The inverse takes
/// the mean, the front-back and the left-right height differences.
///
/// Every function rejects non-finite input (returns false / clamps to the
/// nearest limit) so a bad network value can never reach a motor.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace pear {

/// The four corners, in the order every per-corner table in this project uses
/// (and the order of the PEAR_* rows of the RTPS seat-axis / diagnostics /
/// axis tables).
enum class Corner : std::size_t { FR = 0, FL = 1, BR = 2, BL = 3 };
inline constexpr std::size_t kCornerCount = 4;
inline constexpr std::array<Corner, kCornerCount> kCorners{Corner::FR, Corner::FL, Corner::BR,
                                                           Corner::BL};

constexpr std::size_t index(Corner corner) { return static_cast<std::size_t>(corner); }

constexpr const char *corner_name(Corner corner) {
  switch (corner) {
  case Corner::FR:
    return "FR";
  case Corner::FL:
    return "FL";
  case Corner::BR:
    return "BR";
  default:
    return "BL";
  }
}

/// +1 for the front corners, -1 for the back ones.
constexpr float corner_sx(Corner corner) {
  return (corner == Corner::FR || corner == Corner::FL) ? 1.0f : -1.0f;
}
/// +1 for the left corners, -1 for the right ones.
constexpr float corner_sy(Corner corner) {
  return (corner == Corner::FL || corner == Corner::BL) ? 1.0f : -1.0f;
}

namespace geometry {

inline constexpr float kPi = 3.14159265358979323846f;
constexpr float deg_to_rad(float deg) { return deg * (kPi / 180.0f); }
constexpr float rad_to_deg(float rad) { return rad * (180.0f / kPi); }

/// One value per corner, indexed by Corner.
template <typename T> struct PerCorner {
  std::array<T, kCornerCount> values{};
  constexpr T &operator[](Corner corner) { return values[index(corner)]; }
  constexpr const T &operator[](Corner corner) const { return values[index(corner)]; }
};

/// Corner heights (m) or angles (deg).
using Corners = PerCorner<float>;

struct Geometry {
  float half_length_m;   ///< front/back corners sit at x = +/- half_length
  float half_width_m;    ///< left/right corners sit at y = +/- half_width
  float crank_length_m;  ///< 4-bar crank length (module rise per unit sin)
  float level_angle_deg; ///< crank angle at the reference (level) height
  float min_angle_deg;   ///< lowest allowed crank angle
  float max_angle_deg;   ///< highest allowed crank angle
};

struct Pose {
  float elevation_m{0.0f};      ///< mean corner height
  float fb_tilt_deg{0.0f};      ///< front up positive
  float lateral_tilt_deg{0.0f}; ///< left up positive
};

inline bool is_finite(const Pose &pose) {
  return std::isfinite(pose.elevation_m) && std::isfinite(pose.fb_tilt_deg) &&
         std::isfinite(pose.lateral_tilt_deg);
}

inline bool is_finite(const Corners &corners) {
  return std::all_of(corners.values.begin(), corners.values.end(),
                     [](float v) { return std::isfinite(v); });
}

/// Height (m) of the module above the level reference for a crank angle (deg).
inline float height_from_angle(const Geometry &g, float angle_deg) {
  if (!std::isfinite(angle_deg))
    return 0.0f;
  return g.crank_length_m * std::sin(deg_to_rad(angle_deg - g.level_angle_deg));
}

/// Crank angle (deg) that puts the module at height h (m), clamped to the
/// crank's reach and to [min_angle, max_angle]. Non-finite h clamps to the
/// nearest limit of the level height (i.e. the level angle, then clamped).
inline float angle_from_height(const Geometry &g, float h_m) {
  float ratio = 0.0f;
  if (std::isfinite(h_m) && g.crank_length_m > 0.0f)
    ratio = std::clamp(h_m / g.crank_length_m, -1.0f, 1.0f);
  const float angle = g.level_angle_deg + rad_to_deg(std::asin(ratio));
  return std::clamp(angle, g.min_angle_deg, g.max_angle_deg);
}

/// Corner heights (m) for a base pose. Non-finite pose -> all zeros.
inline Corners corner_heights_from_pose(const Geometry &g, const Pose &pose) {
  Corners heights{};
  if (!is_finite(pose))
    return heights;
  const float fb = std::tan(deg_to_rad(pose.fb_tilt_deg));
  const float lat = std::tan(deg_to_rad(pose.lateral_tilt_deg));
  for (const Corner corner : kCorners) {
    heights[corner] = pose.elevation_m + corner_sx(corner) * g.half_length_m * fb +
                      corner_sy(corner) * g.half_width_m * lat;
  }
  return heights;
}

/// Crank angles (deg, clamped) for a base pose.
inline Corners corner_angles_from_pose(const Geometry &g, const Pose &pose) {
  const Corners heights = corner_heights_from_pose(g, pose);
  Corners angles{};
  for (const Corner corner : kCorners)
    angles[corner] = angle_from_height(g, heights[corner]);
  return angles;
}

/// Corner heights (m) from crank angles (deg).
inline Corners corner_heights_from_angles(const Geometry &g, const Corners &angles) {
  Corners heights{};
  for (const Corner corner : kCorners)
    heights[corner] = height_from_angle(g, angles[corner]);
  return heights;
}

/// The (least-squares plane) pose that the four corner heights describe:
/// elevation = mean, fb = atan2(front - back, 2 * half_length),
/// lat = atan2(left - right, 2 * half_width). Non-finite input -> zero pose.
inline Pose pose_from_corner_heights(const Geometry &g, const Corners &h) {
  Pose pose{};
  if (!is_finite(h))
    return pose;
  const float front = 0.5f * (h[Corner::FR] + h[Corner::FL]);
  const float back = 0.5f * (h[Corner::BR] + h[Corner::BL]);
  const float left = 0.5f * (h[Corner::FL] + h[Corner::BL]);
  const float right = 0.5f * (h[Corner::FR] + h[Corner::BR]);
  pose.elevation_m = 0.5f * (front + back);
  pose.fb_tilt_deg = rad_to_deg(std::atan2(front - back, 2.0f * g.half_length_m));
  pose.lateral_tilt_deg = rad_to_deg(std::atan2(left - right, 2.0f * g.half_width_m));
  return pose;
}

/// The pose the four crank angles (deg) describe.
inline Pose pose_from_corner_angles(const Geometry &g, const Corners &angles) {
  return pose_from_corner_heights(g, corner_heights_from_angles(g, angles));
}

} // namespace geometry
} // namespace pear
