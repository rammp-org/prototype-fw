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

#include <algorithm>
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
// Assembly::Open means the whole loop stays a simple polygon, not only that
// the cranks are apart: over the classic 5-bar's grid, every Open solution
// passes loop_is_open(), and Open refuses points whose only candidates fold the
// bar or the coupler back across a crank (which Any accepts) - the poses that
// collided on the real linkage with the crank-only test.
void test_open_rejects_folded_loops() {
  fivebar::Geometry g{};
  g.m2_x = 200.0f;
  g.m2_y = 0.0f;
  g.crank1 = g.crank2 = 150.0f;
  g.bar = g.coupler = 250.0f;
  g.coupler_side = fivebar::CouplerSide::Ccw;
  const fivebar::Point m1 = {0.0f, 0.0f}, m2 = {g.m2_x, g.m2_y};
  const float ref1 = -90.0f, ref2 = -90.0f;
  int open_solved = 0, any_only = 0, any_folded = 0;
  for (float x = -300.0f; x <= 500.0f; x += 25.0f) {
    for (float y = -450.0f; y <= 250.0f; y += 25.0f) {
      fivebar::Pose any{}, open{};
      g.assembly = fivebar::Assembly::Any;
      const bool any_ok = fivebar::solve_ik(x, y, g, ref1, ref2, any);
      g.assembly = fivebar::Assembly::Open;
      const bool open_ok = fivebar::solve_ik(x, y, g, ref1, ref2, open);
      if (open_ok) {
        ++open_solved;
        char what[96];
        std::snprintf(what, sizeof(what), "open ik (%.0f, %.0f) is a simple loop", x, y);
        check(fivebar::loop_is_open(g, m1, m2, open.j1, open.j2, open.j3), what);
        // and fk of it, under Open, agrees (fk applies the same test)
        fivebar::Pose fk{};
        std::snprintf(what, sizeof(what), "open fk (%.0f, %.0f) accepts the ik pose", x, y);
        check(fivebar::solve_fk(open.theta1_deg, open.theta2_deg, g, fk), what);
      }
      if (any_ok && !open_ok) {
        ++any_only;
        if (!fivebar::loop_is_open(g, m1, m2, any.j1, any.j2, any.j3))
          ++any_folded;
      }
    }
  }
  std::printf("open loop: %d points solved; %d points Any solves but Open refuses (%d of them with "
              "a folded loop)\n",
              open_solved, any_only, any_folded);
  check(open_solved > 100, "open: a reasonable share of the grid is reachable");
  check(any_only > 0 && any_folded == any_only,
        "open refuses exactly the points whose Any pose folds the loop");
}

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

// The reported bug: dragging the target across M2's home (theta2 = -90 deg) made
// the browser solver flip to the mirror assembly. Each pose was solved against a
// FIXED reference (home), and "closest to the reference" scoring swaps branches
// as the solution crosses the reference angle - jumping to the cranks-toward-
// each-other assembly. The fix is to solve each pose against the PREVIOUS solved
// pose (continuation): the firmware already does this with the measured pose, and
// the console now does it while dragging the target. This proves the solver, with
// Open pinned, inverts a smooth FK-generated path straight through M2 home without
// a branch flip when fed the previous pose, and records how far a fixed-home
// reference drifts over the same path.
void test_continuity_across_m2_home() {
  fivebar::Geometry g{};
  g.m2_x = 200.0f;
  g.m2_y = 0.0f;
  g.crank1 = g.crank2 = 150.0f;
  g.bar = g.coupler = 250.0f;
  g.assembly = fivebar::Assembly::Open;
  g.coupler_side = fivebar::CouplerSide::Ccw;

  // A path whose true crank-2 angle sweeps through -90 (M2 home): hold theta1 and
  // sweep theta2 across -90. FK gives the exact M3 the IK must invert.
  const float theta1 = -70.0f;
  float ref1 = 0.0f, ref2 = 0.0f; // continuation reference, seeded from the first pose
  float prev_cont_t2 = 0.0f;
  bool have_prev = false;
  int checked = 0, home_checked = 0;
  float max_cont_step = 0.0f, max_fixed_dev = 0.0f;
  float min_cont_t2 = 1e9f, max_cont_t2 = -1e9f;
  for (float theta2 = -70.0f; theta2 >= -110.0f; theta2 -= 1.0f) {
    fivebar::Pose fk{};
    if (!fivebar::solve_fk(theta1, theta2, g, fk))
      continue; // skip crank combos that are not an Open loop
    if (!have_prev) {
      ref1 = theta1;
      ref2 = theta2;
    }
    fivebar::Pose cont{};
    if (!fivebar::solve_ik(fk.m3[0], fk.m3[1], g, ref1, ref2, cont))
      continue;
    ++checked;
    min_cont_t2 = std::min(min_cont_t2, cont.theta2_deg);
    max_cont_t2 = std::max(max_cont_t2, cont.theta2_deg);
    if (have_prev)
      max_cont_step = std::max(
          max_cont_step, std::fabs(fivebar::normalize_angle_deg(cont.theta2_deg - prev_cont_t2)));
    // In the home neighborhood - clear of the crank2 dyad singularity near
    // theta2 ~ -105, where a fixed-theta1 sweep legitimately changes branch -
    // continuation must recover the exact FK branch.
    if (theta2 <= -80.0f && theta2 >= -100.0f) {
      ++home_checked;
      check_near(cont.theta2_deg, theta2, 0.5f, "continuation recovers theta2 near M2 home");
      check_near(cont.theta1_deg, theta1, 0.5f, "continuation recovers theta1 near M2 home");
    }
    ref1 = cont.theta1_deg;
    ref2 = cont.theta2_deg;
    prev_cont_t2 = cont.theta2_deg;
    have_prev = true;
    // contrast: a fixed home reference is the old browser behavior
    fivebar::Pose fixed{};
    if (fivebar::solve_ik(fk.m3[0], fk.m3[1], g, -90.0f, -90.0f, fixed))
      max_fixed_dev = std::max(
          max_fixed_dev, std::fabs(fivebar::normalize_angle_deg(fixed.theta2_deg - theta2)));
  }
  std::printf("m2-home continuity: %d poses (%d near home); max continuation step %.1f deg, max "
              "fixed-home deviation %.1f deg\n",
              checked, home_checked, max_cont_step, max_fixed_dev);
  check(checked > 20, "m2-home: the sweep produced poses across home");
  check(home_checked > 10, "m2-home: covered the home neighborhood");
  check(min_cont_t2 < -90.0f && max_cont_t2 > -90.0f, "continuation sweeps across M2 home");
  check(max_cont_step < 10.0f, "continuation stays smooth across M2 home (no branch flip)");
  check(max_fixed_dev > max_cont_step + 3.0f,
        "a fixed home reference drifts off the branch far more than continuation");
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
  test_open_rejects_folded_loops();
  test_classic_round_trip(fivebar::CouplerSide::Ccw);
  test_classic_round_trip(fivebar::CouplerSide::Cw);
  test_offset_end_effector();
  test_continuity_across_m2_home();
  if (failures) {
    std::printf("%d failure(s)\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("kinematics host test: all checks passed\n");
  return EXIT_SUCCESS;
}
