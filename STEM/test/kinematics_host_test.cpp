// Host-side test for the STEM 5-bar kinematics (no ESP-IDF needed).
//
//   c++ -std=c++20 -I ../main kinematics_host_test.cpp ../main/ik_5bar.cpp && ./a.out
//
// Checks the inverse kinematics against the joint tables in Geometry.md, that
// the forward kinematics inverts it (ik -> fk reproduces M3) at every
// documented edge pose and across a sweep of the reachable workspace, and that
// the reference-relative variants agree with the raw ones.

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "ik_5bar.hpp"

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

struct EdgePose {
  const char *name;
  float m3_x, m3_y;
  float j1_x, j1_y, j2_x, j2_y, j3_x, j3_y, j5_x, j5_y;
  float theta1_deg, theta2_deg;
};

// Geometry.md section 7: analysis-frame joint coordinates and crank angles.
constexpr EdgePose kEdgePoses[] = {
    {"home", 66.00f, 208.50f, -94.93f, 52.07f, 300.76f, 75.13f, 152.95f, 159.10f, 245.82f, 120.14f,
     14.03f, 168.88f},
    {"full_down", 66.00f, 138.50f, -99.85f, 9.13f, 309.26f, 21.36f, 156.10f, 95.11f, 251.40f,
     62.54f, 3.95f, 178.06f},
    {"full_up", 66.00f, 568.50f, -1.22f, 208.77f, 81.95f, 298.97f, 71.91f, 468.67f, 89.79f,
     369.56f, 74.67f, 129.36f},
    {"max_forward", 266.00f, 208.50f, 42.85f, 238.18f, 304.38f, -58.76f, 280.21f, 109.52f,
     306.31f, 12.24f, -10.93f, 118.10f},
    {"max_rearward", -34.00f, 208.50f, -99.90f, -7.28f, 235.90f, 201.13f, 65.96f, 205.77f,
     166.25f, 215.03f, 40.45f, -178.46f},
};

void test_edge_poses() {
  for (const auto &pose : kEdgePoses) {
    stem::IkSolution ik{};
    char what[128];
    std::snprintf(what, sizeof(what), "ik solves %s", pose.name);
    check(stem::solve_ik_for_m3(pose.m3_x, pose.m3_y, ik), what);
    // The document quotes joints to 0.01 mm and angles to 0.01 deg.
    const float pos_tol = 0.05f;
    check_near(ik.j1[0], pose.j1_x, pos_tol, pose.name);
    check_near(ik.j1[1], pose.j1_y, pos_tol, pose.name);
    check_near(ik.j2[0], pose.j2_x, pos_tol, pose.name);
    check_near(ik.j2[1], pose.j2_y, pos_tol, pose.name);
    check_near(ik.j3[0], pose.j3_x, pos_tol, pose.name);
    check_near(ik.j3[1], pose.j3_y, pos_tol, pose.name);
    check_near(ik.j5[0], pose.j5_x, pos_tol, pose.name);
    check_near(ik.j5[1], pose.j5_y, pos_tol, pose.name);
    check_near(ik.theta1_deg, pose.theta1_deg, 0.02f, pose.name);
    check_near(ik.theta2_deg, pose.theta2_deg, 0.02f, pose.name);

    // FK from the documented crank angles must land on the documented M3.
    stem::IkSolution fk{};
    std::snprintf(what, sizeof(what), "fk solves %s", pose.name);
    check(stem::solve_fk(pose.theta1_deg, pose.theta2_deg, fk), what);
    // 0.01 deg of crank angle moves the far end of a 310 mm crank ~0.05 mm.
    check_near(fk.m3[0], pose.m3_x, 0.2f, pose.name);
    check_near(fk.m3[1], pose.m3_y, 0.2f, pose.name);
    check_near(fk.j3[0], pose.j3_x, 0.2f, pose.name);
    check_near(fk.j3[1], pose.j3_y, 0.2f, pose.name);
  }
}

// ik -> fk must reproduce the input point everywhere the IK has a solution.
void test_round_trip_sweep() {
  int solved = 0;
  int round_trips = 0;
  for (float x = -450.0f; x <= 550.0f; x += 10.0f) {
    for (float y = 50.0f; y <= 750.0f; y += 10.0f) {
      stem::IkSolution ik{};
      if (!stem::solve_ik_for_m3(x, y, ik)) {
        continue;
      }
      ++solved;
      stem::IkSolution fk{};
      if (!stem::solve_fk(ik.theta1_deg, ik.theta2_deg, fk)) {
        ++failures;
        std::printf("FAIL: fk rejected ik solution at (%.0f, %.0f) theta=(%.2f, %.2f)\n", x, y,
                    ik.theta1_deg, ik.theta2_deg);
        continue;
      }
      const float err = std::hypot(fk.m3[0] - x, fk.m3[1] - y);
      if (err > 0.01f) {
        ++failures;
        std::printf("FAIL: round trip at (%.0f, %.0f) -> (%.3f, %.3f), err %.4f\n", x, y, fk.m3[0],
                    fk.m3[1], err);
        continue;
      }
      ++round_trips;
    }
  }
  std::printf("sweep: %d reachable points, %d round-tripped\n", solved, round_trips);
  check(solved > 1000, "sweep found a plausible reachable region");
}

void test_reference_variants() {
  const auto &ref = stem::kFullDownReference;
  stem::IkSolution ik{};
  check(stem::solve_ik_for_m3_reference(0.0f, 0.0f, ref, ik), "ik_reference at the reference");
  // At the reference point the relative crank angles are (by definition of the
  // reference) the raw angles minus the reference angles.
  stem::IkSolution raw{};
  check(stem::solve_ik_for_m3(ref.x, ref.y, raw), "ik at the reference");
  check_near(ik.theta1_deg, stem::normalize_angle_deg(raw.theta1_deg - ref.m1_angle_deg), 1e-3f,
             "reference theta1");
  check_near(ik.theta2_deg, stem::normalize_angle_deg(raw.theta2_deg - ref.m2_angle_deg), 1e-3f,
             "reference theta2");

  // fk_reference inverts ik_reference at a few relative offsets
  for (const auto [dx, dy] : {std::pair{0.0f, 0.0f}, std::pair{-50.0f, 100.0f},
                              std::pair{80.0f, 300.0f}, std::pair{-100.0f, 20.0f}}) {
    stem::IkSolution rel_ik{};
    if (!stem::solve_ik_for_m3_reference(dx, dy, ref, rel_ik)) {
      continue;
    }
    stem::IkSolution rel_fk{};
    check(stem::solve_fk_reference(rel_ik.theta1_deg, rel_ik.theta2_deg, ref, rel_fk),
          "fk_reference solves");
    check_near(rel_fk.m3[0] - ref.x, dx, 0.01f, "fk_reference x");
    check_near(rel_fk.m3[1] - ref.y, dy, 0.01f, "fk_reference y");
    check_near(rel_fk.theta1_deg, rel_ik.theta1_deg, 1e-3f, "fk_reference theta1");
    check_near(rel_fk.theta2_deg, rel_ik.theta2_deg, 1e-3f, "fk_reference theta2");
    check_near(rel_fk.m3_angle_deg, rel_ik.m3_angle_deg, 1e-3f, "fk_reference m3 angle");
  }
}

void test_fk_rejects_impossible() {
  stem::IkSolution fk{};
  // Uncrossed cranks (both pointing straight up) violate the pin invariant.
  check(!stem::solve_fk(90.0f, 90.0f, fk), "fk rejects an uncrossed pose");
}

// The documented design travel must stay solvable after the assembly-mode
// checks: the full vertical range at the home x, and the full fore/aft range
// at the home height (Geometry.md section 7/8).
void test_design_travel_reachable() {
  for (float y_plate = 290.0f; y_plate <= 720.0f; y_plate += 5.0f) {
    stem::IkSolution ik{};
    if (!stem::solve_ik_for_m3(66.0f, y_plate - 151.5f, ik)) {
      ++failures;
      std::printf("FAIL: design travel: (66, y_plate %.0f) unreachable\n", y_plate);
    }
  }
  for (float x_plate = -34.0f; x_plate <= 266.0f; x_plate += 5.0f) {
    stem::IkSolution ik{};
    if (!stem::solve_ik_for_m3(x_plate, 360.0f - 151.5f, ik)) {
      ++failures;
      std::printf("FAIL: design travel: (x_plate %.0f, 360) unreachable\n", x_plate);
    }
  }
}

} // namespace

int main() {
  test_edge_poses();
  test_round_trip_sweep();
  test_reference_variants();
  test_fk_rejects_impossible();
  test_design_travel_reachable();
  if (failures) {
    std::printf("%d failure(s)\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("kinematics host test: all checks passed\n");
  return EXIT_SUCCESS;
}
