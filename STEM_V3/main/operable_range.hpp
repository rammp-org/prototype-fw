#pragma once

// STEM v3.1 operable range: the set of motor angles the firmware will move the
// linkage to with a pose command (MOVE / HOME / tilt).
//
// Source: "STEM v3.1 - Motor Zero Reference & Reachable Map", Rev C (12 Nov 2026).
// A collision-checked sweep of the as-built CAD on a 1 deg encoder grid gave the
// reachable, collision-free poses; the operational zone is the part of that whose
// seat axis (M3) lies inside the spec envelope (x -34..+266 mm, plate height
// <= 725 mm). The simple bounds below cover 94 % of the operational zone and
// contain NO colliding cell (they are deliberately conservative; ~5 % of the cells
// inside sit slightly past the spec envelope but are reachable and collision-free).
//
// Encoder angles are those of the calibrated zero (see kFullDownReference):
//   a = enc101, b = enc103 (the odd motors; enc102 = -a, enc104 = -b, enc106 = -enc105).
// The firmware's pair angles are the PRIMARY (even) motors, so a = -M1_Seat, b = -M2_Seat.
//
//   6 <= a <= 90
//   max(-47, 0.6 a - 96) <= b <= min(12, 1.9 a - 34, 70 - 1.25 a)
//   seat: -20 deg (forward, front down) <= tilt <= +50 deg (recline, front up),
//         tilt measured from level (the reach study's band c +/- 35 on enc105 is
//         exactly this range; here it is checked against the IK's exact level
//         angle instead of the band's quadratic fit)
//
// Known limits of the bounds: the spec "full up" pose (a 89.5, b -40.6) sits 1.3 deg
// outside the upper-right corner; seat-to-frame collisions while tilting were not
// checked; torque (load-dependent) is a separate limit, not part of this range.
//
// Header-only and free of ESP-IDF so the host tests can check it.

#include <algorithm>
#include <array>
#include <cmath>

namespace stem {

struct OperableRange {
  float a_min_deg{6.0f};
  float a_max_deg{90.0f};
  float tilt_min_deg{-20.0f}; ///< forward tilt limit (front down), from level
  float tilt_max_deg{50.0f};  ///< recline limit (front up), from level
  /// Grows the base region outward by this many degrees on every bound (0 = the
  /// study's collision-free bounds exactly).
  float tolerance_deg{0.0f};

  static float b_lower(float a) { return std::max(-47.0f, 0.6f * a - 96.0f); }
  static float b_upper(float a) { return std::min({12.0f, 1.9f * a - 34.0f, 70.0f - 1.25f * a}); }

  /// Base linkage (rear + front pairs), in calibrated encoder angles a = enc101, b = enc103.
  bool base_contains(float a, float b) const {
    if (!std::isfinite(a) || !std::isfinite(b)) {
      return false;
    }
    const float t = tolerance_deg;
    return a >= a_min_deg - t && a <= a_max_deg + t && b >= b_lower(a) - t && b <= b_upper(a) + t;
  }

  bool tilt_contains(float tilt_deg) const {
    return std::isfinite(tilt_deg) && tilt_deg >= tilt_min_deg && tilt_deg <= tilt_max_deg;
  }

  /// Pair angles in Pair order (M1_Seat, M2_Seat, M3_Seat; primary motors 102 / 104 / 106)
  /// plus the seat tilt from level that the seat pair angle encodes.
  bool contains(const std::array<float, 3> &pair_deg, float tilt_deg) const {
    return base_contains(-pair_deg[0], -pair_deg[1]) && tilt_contains(tilt_deg);
  }
};

} // namespace stem
