// Host-side test for the generalized 5-bar kinematics (no ESP-IDF needed).
//
//   c++ -std=c++20 -I ../main kinematics_host_test.cpp ../main/five_bar.cpp && ./a.out
//
// Checks (1) that with the STEM seat linkage's geometry (bar 170 + end
// effector 100 past J3, crossed cranks, J3 on the CCW side) the solver
// reproduces every documented STEM edge pose (STEM/Geometry.md section 7), and
// (2) that for a classic open 5-bar (wheel at J3) the inverse kinematics is the
// exact inverse of the forward kinematics over a sweep of the workspace, in
// both coupler folds, and that unreachable points / impossible crank angles are
// rejected.

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "five_bar.hpp"

namespace {

int failures = 0;

void check(bool ok, const char *what) {
  if (!ok) {
    ++failures;
    std::printf("FAIL: %s\n", what);
  }
}

void check_near(float actual, float expected, float tol, const char *what) {
  if (std::fabs(actual - expected) > tol) {
    ++failures;
    std::printf("FAIL: %s: got %.4f expected %.4f (tol %.4f)\n", what, actual, expected, tol);
  }
}

// The STEM seat linkage in this module's terms: M2 at (170, 0), cranks 310 /
// 270, J2->J3 = 270 - 100 = 170, coupler J1->J3 = 270, M3 100 past J3.
fivebar::Geometry stem_geometry() {
  fivebar::Geometry g;
  g.m2_x = 170.0f;
  g.m2_y = 0.0f;
  g.crank1 = 310.0f;
  g.crank2 = 270.0f;
  g.bar = 170.0f;
  g.coupler = 270.0f;
  g.ee_extension = 100.0f;
  g.ee_offset = 0.0f;
  g.assembly = fivebar::Assembly::Crossed;
  g.coupler_side = fivebar::CouplerSide::Ccw;
  return g;
}

struct EdgePose {
  const char *name;
  float m3_x, m3_y;
  float j1_x, j1_y, j2_x, j2_y, j3_x, j3_y;
  float theta1_deg, theta2_deg;
};

// STEM/Geometry.md section 7: analysis-frame joint coordinates and crank angles.
constexpr EdgePose kStemEdgePoses[] = {
    {"home", 66.00f, 208.50f, -94.93f, 52.07f, 300.76f, 75.13f, 152.95f, 159.10f, 14.03f, 168.88f},
    {"full_down", 66.00f, 138.50f, -99.85f, 9.13f, 309.26f, 21.36f, 156.10f, 95.11f, 3.95f,
     178.06f},
    {"full_up", 66.00f, 568.50f, -1.22f, 208.77f, 81.95f, 298.97f, 71.91f, 468.67f, 74.67f,
     129.36f},
    {"max_forward", 266.00f, 208.50f, 42.85f, 238.18f, 304.38f, -58.76f, 280.21f, 109.52f, -10.93f,
     118.10f},
    {"max_rearward", -34.00f, 208.50f, -99.90f, -7.28f, 235.90f, 201.13f, 65.96f, 205.77f, 40.45f,
     -178.46f},
};

void test_stem_edge_poses() {
  const auto g = stem_geometry();
  // the STEM calibration pose's crank angles as the branch reference
  const float ref1 = -0.16f, ref2 = 143.74f;
  for (const auto &edge : kStemEdgePoses) {
    fivebar::Pose ik{};
    char what[128];
    std::snprintf(what, sizeof(what), "STEM ik solves %s", edge.name);
    check(fivebar::solve_ik(edge.m3_x, edge.m3_y, g, ref1, ref2, ik), what);
    std::snprintf(what, sizeof(what), "STEM %s theta1", edge.name);
    check_near(ik.theta1_deg, edge.theta1_deg, 0.02f, what);
    std::snprintf(what, sizeof(what), "STEM %s theta2", edge.name);
    check_near(fivebar::normalize_angle_deg(ik.theta2_deg - edge.theta2_deg), 0.0f, 0.02f, what);
    std::snprintf(what, sizeof(what), "STEM %s J1", edge.name);
    check_near(ik.j1[0], edge.j1_x, 0.05f, what);
    check_near(ik.j1[1], edge.j1_y, 0.05f, what);
    std::snprintf(what, sizeof(what), "STEM %s J2", edge.name);
    check_near(ik.j2[0], edge.j2_x, 0.05f, what);
    check_near(ik.j2[1], edge.j2_y, 0.05f, what);
    std::snprintf(what, sizeof(what), "STEM %s J3", edge.name);
    check_near(ik.j3[0], edge.j3_x, 0.05f, what);
    check_near(ik.j3[1], edge.j3_y, 0.05f, what);
    std::snprintf(what, sizeof(what), "STEM %s M3 round trip", edge.name);
    check_near(ik.m3[0], edge.m3_x, 0.01f, what);
    check_near(ik.m3[1], edge.m3_y, 0.01f, what);
    // and the forward kinematics from the documented crank angles lands on M3
    fivebar::Pose fk{};
    std::snprintf(what, sizeof(what), "STEM fk solves %s", edge.name);
    check(fivebar::solve_fk(edge.theta1_deg, edge.theta2_deg, g, fk), what);
    std::snprintf(what, sizeof(what), "STEM fk %s M3", edge.name);
    check_near(fk.m3[0], edge.m3_x, 0.2f, what); // the doc's angles have 2 decimals
    check_near(fk.m3[1], edge.m3_y, 0.2f, what);
  }
  // uncrossed cranks violate the STEM assembly
  fivebar::Pose fk{};
  check(!fivebar::solve_fk(90.0f, 90.0f, g, fk), "STEM fk rejects an uncrossed pose");
}

// A classic open 5-bar with the wheel at J3: ik -> fk reproduces the point
// everywhere the ik succeeds, in both coupler folds; the assembly stays Open.
void test_classic_round_trip(fivebar::CouplerSide side) {
  fivebar::Geometry g;
  g.m2_x = 200.0f;
  g.crank1 = 120.0f;
  g.crank2 = 120.0f;
  g.bar = 260.0f;
  g.coupler = 260.0f;
  g.assembly = fivebar::Assembly::Open;
  g.coupler_side = side;
  const char *side_name = side == fivebar::CouplerSide::Ccw ? "ccw" : "cw";
  // reference: cranks pointing away from the linkage side of the base
  const float ref1 = side == fivebar::CouplerSide::Ccw ? 120.0f : -120.0f;
  const float ref2 = side == fivebar::CouplerSide::Ccw ? 60.0f : -60.0f;
  int solved = 0, total = 0;
  for (float x = -300.0f; x <= 500.0f; x += 10.0f) {
    for (float y = -450.0f; y <= 450.0f; y += 10.0f) {
      ++total;
      fivebar::Pose ik{};
      if (!fivebar::solve_ik(x, y, g, ref1, ref2, ik))
        continue;
      ++solved;
      char what[160];
      std::snprintf(what, sizeof(what), "classic %s ik (%.0f, %.0f) lands on the point", side_name,
                    x, y);
      check_near(ik.m3[0], x, 0.01f, what);
      check_near(ik.m3[1], y, 0.01f, what);
      // link lengths hold
      std::snprintf(what, sizeof(what), "classic %s (%.0f, %.0f) link lengths", side_name, x, y);
      check_near(std::hypot(ik.j2[0], ik.j2[1]), g.crank1, 0.01f, what);
      check_near(std::hypot(ik.j1[0] - g.m2_x, ik.j1[1] - g.m2_y), g.crank2, 0.01f, what);
      check_near(std::hypot(ik.j3[0] - ik.j2[0], ik.j3[1] - ik.j2[1]), g.bar, 0.01f, what);
      check_near(std::hypot(ik.j3[0] - ik.j1[0], ik.j3[1] - ik.j1[1]), g.coupler, 0.01f, what);
      fivebar::Pose fk{};
      std::snprintf(what, sizeof(what), "classic %s fk (%.0f, %.0f) solves", side_name, x, y);
      check(fivebar::solve_fk(ik.theta1_deg, ik.theta2_deg, g, fk), what);
      // 0.5 mm: at the workspace boundary the bar/coupler dyad is nearly
      // folded flat and the fk's circle intersection amplifies float round-off
      std::snprintf(what, sizeof(what), "classic %s fk (%.0f, %.0f) reproduces M3", side_name, x,
                    y);
      check_near(fk.m3[0], x, 0.5f, what);
      check_near(fk.m3[1], y, 0.5f, what);
    }
  }
  std::printf("classic %s: %d / %d grid points reachable\n", side_name, solved, total);
  check(solved > total / 8, "classic: a reasonable share of the grid is reachable");
  // a point far outside the reach is rejected
  fivebar::Pose far{};
  check(!fivebar::solve_ik(2000.0f, 0.0f, g, ref1, ref2, far), "classic ik rejects far points");
  // the link lengths must be positive
  fivebar::Geometry bad = g;
  bad.bar = 0.0f;
  check(!fivebar::is_valid(bad), "zero bar length is invalid");
  check(!fivebar::solve_fk(90.0f, 90.0f, bad, far), "fk refuses an invalid geometry");
}

// With an offset end effector the ik still lands on the point and the fk
// agrees (exercises the gamma rotation of the bar direction).
void test_offset_end_effector() {
  fivebar::Geometry g;
  g.m2_x = 150.0f;
  g.crank1 = 100.0f;
  g.crank2 = 100.0f;
  g.bar = 200.0f;
  g.coupler = 200.0f;
  g.ee_extension = 60.0f;
  g.ee_offset = -25.0f;
  g.assembly = fivebar::Assembly::Any;
  g.coupler_side = fivebar::CouplerSide::Ccw;
  int solved = 0;
  for (float x = -200.0f; x <= 350.0f; x += 25.0f) {
    for (float y = -50.0f; y <= 400.0f; y += 25.0f) {
      fivebar::Pose ik{};
      if (!fivebar::solve_ik(x, y, g, 110.0f, 70.0f, ik))
        continue;
      ++solved;
      char what[160];
      std::snprintf(what, sizeof(what), "offset ik (%.0f, %.0f) lands on the point", x, y);
      check_near(ik.m3[0], x, 0.01f, what);
      check_near(ik.m3[1], y, 0.01f, what);
      fivebar::Pose fk{};
      std::snprintf(what, sizeof(what), "offset fk (%.0f, %.0f) reproduces M3", x, y);
      check(fivebar::solve_fk(ik.theta1_deg, ik.theta2_deg, g, fk), what);
      check_near(fk.m3[0], x, 0.05f, what);
      check_near(fk.m3[1], y, 0.05f, what);
    }
  }
  check(solved > 20, "offset: points were reachable");
}

void test_normalize() {
  check_near(fivebar::normalize_angle_deg(190.0f), -170.0f, 1e-4f, "normalize 190");
  check_near(fivebar::normalize_angle_deg(-190.0f), 170.0f, 1e-4f, "normalize -190");
  check_near(fivebar::normalize_angle_deg(180.0f), 180.0f, 1e-4f, "normalize 180");
  check_near(fivebar::normalize_angle_deg(-180.0f), 180.0f, 1e-4f, "normalize -180");
  check_near(fivebar::normalize_angle_deg(720.5f), 0.5f, 1e-3f, "normalize 720.5");
}

} // namespace

int main() {
  test_normalize();
  test_stem_edge_poses();
  test_classic_round_trip(fivebar::CouplerSide::Ccw);
  test_classic_round_trip(fivebar::CouplerSide::Cw);
  test_offset_end_effector();
  if (failures) {
    std::printf("%d failure(s)\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("kinematics host test: all checks passed\n");
  return EXIT_SUCCESS;
}
