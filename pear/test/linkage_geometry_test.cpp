// Host-side test for the PEAR corner-linkage geometry (no ESP-IDF needed).
//
//   c++ -std=c++20 -I../main linkage_geometry_test.cpp && ./a.out
//
// Checks the pose -> corner heights -> crank angles -> pose round trip, the
// height <-> angle inverse pair, the clamping at the crank's reach and the
// angle limits, the tilt sign conventions (front up = positive fb tilt, left
// up = positive lateral tilt) and that non-finite input never escapes.

#include <cmath>
#include <cstdio>
#include <limits>

#include "linkage_geometry.hpp"

namespace {

int failures = 0;

#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      ++failures;                                                                                  \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                  \
    }                                                                                              \
  } while (0)

#define CHECK_NEAR(actual, expected, tol)                                                          \
  do {                                                                                             \
    const float a_ = (actual);                                                                     \
    const float e_ = (expected);                                                                   \
    if (!(std::fabs(a_ - e_) <= (tol))) {                                                          \
      ++failures;                                                                                  \
      std::printf("FAIL %s:%d: %s = %.5f, expected %.5f (tol %.5f)\n", __FILE__, __LINE__,         \
                  #actual, a_, e_, (tol));                                                         \
    }                                                                                              \
  } while (0)

using pear::Corner;
using pear::geometry::Corners;
using pear::geometry::Geometry;
using pear::geometry::Pose;

// the hw_config placeholders (kept in sync by hand: this test has no ESP-IDF)
constexpr Geometry kGeometry = {
    .half_length_m = 0.406f,
    .half_width_m = 0.267f,
    .crank_length_m = 0.15f,
    .level_angle_deg = {{0.0f, 0.0f, 0.0f, 0.0f}},
    .min_angle_deg = {{-30.0f, -30.0f, -30.0f, -30.0f}},
    .max_angle_deg = {{60.0f, 60.0f, 60.0f, 60.0f}},
};

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

void test_corner_signs() {
  CHECK(pear::corner_sx(Corner::FR) == 1.0f);
  CHECK(pear::corner_sx(Corner::FL) == 1.0f);
  CHECK(pear::corner_sx(Corner::BR) == -1.0f);
  CHECK(pear::corner_sx(Corner::BL) == -1.0f);
  CHECK(pear::corner_sy(Corner::FL) == 1.0f);
  CHECK(pear::corner_sy(Corner::BL) == 1.0f);
  CHECK(pear::corner_sy(Corner::FR) == -1.0f);
  CHECK(pear::corner_sy(Corner::BR) == -1.0f);
}

void test_height_angle_inverse() {
  // level: angle = level angle -> height 0
  CHECK_NEAR(pear::geometry::height_from_angle(kGeometry, Corner::FR, 0.0f), 0.0f, 1e-6f);
  CHECK_NEAR(pear::geometry::angle_from_height(kGeometry, Corner::FR, 0.0f), 0.0f, 1e-5f);
  // 30 deg up -> crank/2
  CHECK_NEAR(pear::geometry::height_from_angle(kGeometry, Corner::FR, 30.0f), 0.075f, 1e-5f);
  CHECK_NEAR(pear::geometry::angle_from_height(kGeometry, Corner::FR, 0.075f), 30.0f, 1e-3f);
  // round trip across the range
  for (float angle = -30.0f; angle <= 60.0f; angle += 2.5f) {
    const float h = pear::geometry::height_from_angle(kGeometry, Corner::FR, angle);
    CHECK_NEAR(pear::geometry::angle_from_height(kGeometry, Corner::FR, h), angle, 1e-3f);
  }
  // a non-zero level angle shifts the reference
  Geometry shifted = kGeometry;
  shifted.level_angle_deg = {{15.0f, 15.0f, 15.0f, 15.0f}};
  CHECK_NEAR(pear::geometry::height_from_angle(shifted, Corner::FR, 15.0f), 0.0f, 1e-6f);
  CHECK_NEAR(pear::geometry::angle_from_height(shifted, Corner::FR, 0.0f), 15.0f, 1e-5f);
  CHECK_NEAR(pear::geometry::angle_from_height(shifted, Corner::FR, 0.075f), 45.0f, 1e-3f);
}

void test_per_corner_level() {
  // corners mounted at different angles: the same height is a different angle
  // per corner, and the pose from equal heights is level
  Geometry mixed = kGeometry;
  mixed.level_angle_deg = {{-10.0f, 115.0f, 100.0f, -105.0f}};
  mixed.min_angle_deg = {{-80.0f, 60.0f, 30.0f, -170.0f}};
  mixed.max_angle_deg = {{60.0f, 170.0f, 170.0f, -40.0f}};
  for (const Corner c : pear::kCorners) {
    CHECK_NEAR(pear::geometry::height_from_angle(mixed, c, mixed.level_angle_deg[c]), 0.0f, 1e-6f);
    CHECK_NEAR(pear::geometry::angle_from_height(mixed, c, 0.075f),
               mixed.level_angle_deg[c] + 30.0f, 1e-3f);
  }
  const Corners level = mixed.level_angle_deg;
  const Pose pose = pear::geometry::pose_from_corner_angles(mixed, level);
  CHECK_NEAR(pose.elevation_m, 0.0f, 1e-6f);
  CHECK_NEAR(pose.fb_tilt_deg, 0.0f, 1e-5f);
  CHECK_NEAR(pose.lateral_tilt_deg, 0.0f, 1e-5f);
  const Corners back = pear::geometry::corner_angles_from_pose(mixed, {0.05f, 0.0f, 0.0f});
  for (const Corner c : pear::kCorners)
    CHECK_NEAR(back[c], mixed.level_angle_deg[c] + 19.47f, 0.01f); // asin(1/3)
}

void test_clamping() {
  // beyond the crank's reach: asin clamps to +/-90 then the angle limits apply
  CHECK_NEAR(pear::geometry::angle_from_height(kGeometry, Corner::FR, 1.0f), 60.0f, 1e-6f);
  CHECK_NEAR(pear::geometry::angle_from_height(kGeometry, Corner::FR, -1.0f), -30.0f, 1e-6f);
  // inside the reach but outside the angle limits
  const float h70 = 0.15f * std::sin(70.0f * pear::geometry::kPi / 180.0f);
  CHECK_NEAR(pear::geometry::angle_from_height(kGeometry, Corner::FR, h70), 60.0f, 1e-6f);
  const float hm45 = 0.15f * std::sin(-45.0f * pear::geometry::kPi / 180.0f);
  CHECK_NEAR(pear::geometry::angle_from_height(kGeometry, Corner::FR, hm45), -30.0f, 1e-6f);
  // a zero crank cannot divide by zero
  Geometry flat = kGeometry;
  flat.crank_length_m = 0.0f;
  CHECK_NEAR(pear::geometry::angle_from_height(flat, Corner::FR, 0.1f), 0.0f, 1e-6f);
}

void test_pose_signs() {
  // elevation only: every corner at the same height
  {
    const Corners h = pear::geometry::corner_heights_from_pose(kGeometry, {0.05f, 0.0f, 0.0f});
    for (const Corner c : pear::kCorners)
      CHECK_NEAR(h[c], 0.05f, 1e-6f);
  }
  // front up: the front corners rise, the back ones drop, by half_length*tan
  {
    const Corners h = pear::geometry::corner_heights_from_pose(kGeometry, {0.0f, 5.0f, 0.0f});
    const float dz = 0.406f * std::tan(5.0f * pear::geometry::kPi / 180.0f);
    CHECK_NEAR(h[Corner::FR], dz, 1e-6f);
    CHECK_NEAR(h[Corner::FL], dz, 1e-6f);
    CHECK_NEAR(h[Corner::BR], -dz, 1e-6f);
    CHECK_NEAR(h[Corner::BL], -dz, 1e-6f);
    CHECK(h[Corner::FR] > h[Corner::BR]);
  }
  // left up: the left corners rise, the right ones drop, by half_width*tan
  {
    const Corners h = pear::geometry::corner_heights_from_pose(kGeometry, {0.0f, 0.0f, 4.0f});
    const float dz = 0.267f * std::tan(4.0f * pear::geometry::kPi / 180.0f);
    CHECK_NEAR(h[Corner::FL], dz, 1e-6f);
    CHECK_NEAR(h[Corner::BL], dz, 1e-6f);
    CHECK_NEAR(h[Corner::FR], -dz, 1e-6f);
    CHECK_NEAR(h[Corner::BR], -dz, 1e-6f);
    CHECK(h[Corner::FL] > h[Corner::FR]);
  }
  // the +/-90 deg boundary of the seat-axis table: tan() must not flip sign
  // past pi/2 - a maximum "front up" saturates the FRONT corners at the top
  for (const float tilt : {90.0f, -90.0f}) {
    const Corners a = pear::geometry::corner_angles_from_pose(kGeometry, {0.0f, tilt, 0.0f});
    const float up =
        tilt > 0 ? kGeometry.max_angle_deg[Corner::FR] : kGeometry.min_angle_deg[Corner::FR];
    const float down =
        tilt > 0 ? kGeometry.min_angle_deg[Corner::FR] : kGeometry.max_angle_deg[Corner::FR];
    CHECK_NEAR(a[Corner::FR], up, 1e-6f);
    CHECK_NEAR(a[Corner::FL], up, 1e-6f);
    CHECK_NEAR(a[Corner::BR], down, 1e-6f);
    CHECK_NEAR(a[Corner::BL], down, 1e-6f);
    const Corners l = pear::geometry::corner_angles_from_pose(kGeometry, {0.0f, 0.0f, tilt});
    CHECK_NEAR(l[Corner::FL], up, 1e-6f);
    CHECK_NEAR(l[Corner::FR], down, 1e-6f);
  }
  // the inverse sees the same signs
  {
    Corners h{};
    h[Corner::FR] = 0.02f;
    h[Corner::FL] = 0.02f;
    h[Corner::BR] = -0.02f;
    h[Corner::BL] = -0.02f;
    const Pose p = pear::geometry::pose_from_corner_heights(kGeometry, h);
    CHECK(p.fb_tilt_deg > 0.0f);
    CHECK_NEAR(p.lateral_tilt_deg, 0.0f, 1e-6f);
    CHECK_NEAR(p.elevation_m, 0.0f, 1e-6f);
  }
  {
    Corners h{};
    h[Corner::FL] = 0.03f;
    h[Corner::BL] = 0.03f;
    h[Corner::FR] = 0.01f;
    h[Corner::BR] = 0.01f;
    const Pose p = pear::geometry::pose_from_corner_heights(kGeometry, h);
    CHECK(p.lateral_tilt_deg > 0.0f);
    CHECK_NEAR(p.fb_tilt_deg, 0.0f, 1e-6f);
    CHECK_NEAR(p.elevation_m, 0.02f, 1e-6f);
  }
}

void test_pose_round_trip() {
  // pose -> heights -> angles -> heights -> pose, inside the reach (every
  // corner height within [crank*sin(min), crank*sin(max)] = [-0.075, 0.130] m)
  const Pose poses[] = {
      {0.0f, 0.0f, 0.0f},   {0.03f, 0.0f, 0.0f},   {0.0f, 3.0f, 0.0f},  {0.0f, 0.0f, -4.0f},
      {0.02f, -2.0f, 3.0f}, {-0.02f, 2.0f, -3.0f}, {0.06f, 6.0f, 2.0f}, {0.08f, -3.0f, -3.0f},
  };
  for (const Pose &pose : poses) {
    const Corners heights = pear::geometry::corner_heights_from_pose(kGeometry, pose);
    CHECK_NEAR(pear::geometry::pose_from_corner_heights(kGeometry, heights).elevation_m,
               pose.elevation_m, 1e-5f);
    const Corners angles = pear::geometry::corner_angles_from_pose(kGeometry, pose);
    for (const Corner c : pear::kCorners) {
      CHECK(angles[c] >= kGeometry.min_angle_deg[c] && angles[c] <= kGeometry.max_angle_deg[c]);
    }
    const Pose back = pear::geometry::pose_from_corner_angles(kGeometry, angles);
    CHECK_NEAR(back.elevation_m, pose.elevation_m, 1e-4f);
    CHECK_NEAR(back.fb_tilt_deg, pose.fb_tilt_deg, 1e-2f);
    CHECK_NEAR(back.lateral_tilt_deg, pose.lateral_tilt_deg, 1e-2f);
  }
  // a pose beyond the reach comes back clamped (approximated), never NaN
  const Corners far = pear::geometry::corner_angles_from_pose(kGeometry, {1.0f, 40.0f, 0.0f});
  for (const Corner c : pear::kCorners)
    CHECK(std::isfinite(far[c]) && far[c] <= kGeometry.max_angle_deg[c]);
  const Pose far_back = pear::geometry::pose_from_corner_angles(kGeometry, far);
  CHECK(std::isfinite(far_back.elevation_m) && std::isfinite(far_back.fb_tilt_deg));
  // ... and a pose that only one corner cannot reach is approximated: the
  // reachable corners are exact, the clamped one sits at its limit
  // (front up 4 deg + left DOWN 5 deg: BL wants -0.092 m, below -0.075 m)
  const Pose partly = {-0.04f, 4.0f, -5.0f};
  const Corners partly_h = pear::geometry::corner_heights_from_pose(kGeometry, partly);
  const Corners partly_a = pear::geometry::corner_angles_from_pose(kGeometry, partly);
  CHECK(partly_h[Corner::BL] < -0.075f);
  CHECK(partly_h[Corner::BR] > -0.075f);
  CHECK_NEAR(partly_a[Corner::BL], kGeometry.min_angle_deg[Corner::BL], 1e-6f);
  CHECK_NEAR(pear::geometry::height_from_angle(kGeometry, Corner::FR, partly_a[Corner::FR]),
             partly_h[Corner::FR], 1e-5f);
}

void test_non_finite() {
  CHECK(!pear::geometry::is_finite(Pose{kNaN, 0.0f, 0.0f}));
  CHECK(!pear::geometry::is_finite(Pose{0.0f, kInf, 0.0f}));
  CHECK(pear::geometry::is_finite(Pose{0.0f, 0.0f, 0.0f}));
  const Corners h = pear::geometry::corner_heights_from_pose(kGeometry, {kNaN, 0.0f, 0.0f});
  for (const Corner c : pear::kCorners)
    CHECK(h[c] == 0.0f);
  CHECK(std::isfinite(pear::geometry::angle_from_height(kGeometry, Corner::FR, kNaN)));
  CHECK(std::isfinite(pear::geometry::angle_from_height(kGeometry, Corner::FR, -kInf)));
  CHECK_NEAR(pear::geometry::height_from_angle(kGeometry, Corner::FR, kNaN), 0.0f, 1e-6f);
  Corners bad{};
  bad[Corner::BL] = kNaN;
  CHECK(!pear::geometry::is_finite(bad));
  const Pose p = pear::geometry::pose_from_corner_heights(kGeometry, bad);
  CHECK(p.elevation_m == 0.0f && p.fb_tilt_deg == 0.0f && p.lateral_tilt_deg == 0.0f);
}

} // namespace

int main() {
  test_corner_signs();
  test_height_angle_inverse();
  test_per_corner_level();
  test_clamping();
  test_pose_signs();
  test_pose_round_trip();
  test_non_finite();
  if (failures == 0) {
    std::printf("linkage_geometry_test: all checks passed\n");
    return 0;
  }
  std::printf("linkage_geometry_test: %d check(s) FAILED\n", failures);
  return 1;
}
