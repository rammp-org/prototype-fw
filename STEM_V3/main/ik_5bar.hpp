#pragma once

#include <array>

namespace stem {

struct GeometryConfig {
  float m1_x = 0.0f;
  float m1_y = 0.0f;
  float m2_x = 170.0f;
  float m2_y = 0.0f;
  float j4_x = -38.0f;
  float j4_y = 8.0f;
  float bar_length_m3_to_j2 = 270.0f;
  float bar_length_m3_to_j3 = 100.0f;
  float bar_length_m3_to_j5 = 200.0f;
  float bar_length_j1_to_j3 = 270.0f;
  float bar_length_j5_perp_offset = 12.0f;
  float crank_length_m1_to_j2 = 310.0f;
  float crank_length_m2_to_j1 = 270.0f;
};

struct IkReference {
  float x = 0.0f;
  float y = 0.0f;
  float m1_angle_deg = 0.0f;
  float m2_angle_deg = 0.0f;
  float m3_angle_deg = 0.0f;
};

// Reference = how the motors' zeros map onto the linkage:
//   x, y          origin of the x_rel / y_rel coordinates (Home = (0, 0)); the
//                 v3.1 "software zero" pose, M3 at (170, 230), a safe reachable pose
//   m1, m2, m3    crank / seat-bar angles (absolute, analysis frame) when the
//                 paired motors read 0 deg. From the v3.1 test-stand zero
//                 calibration (12 Nov 2026: arm tip on the plate, output bolt hole
//                 at 12 o'clock): theta1 = -11.27 + enc101, theta2 = 169.18 + enc103,
//                 seat level when enc105 = bar angle (M3->J2) + 33.76, i.e. a
//                 plate angle (J2->M3) of 146.24 + seat pair. Verified 18 Nov 2026:
//                 Set left -11.11 / right 25.44 put the rig on the software-zero
//                 STEP pose (theta1 -0.75, theta2 143.67) within ~0.6 deg.
//   (was {170, 230, -0.16, 143.74, 121.23}: an earlier eyeball zero at that pose)
inline constexpr IkReference kFullDownReference{170.0f, 230.0f, -11.27f, 169.18f, 146.24f};

// Joint positions are absolute in the analysis frame (origin at M1, +x toward
// M2, +y up; see Geometry.md). Angles are absolute for the raw solvers and
// relative to the IkReference for the *_reference solvers.
struct IkSolution {
  std::array<float, 2> m3{};
  std::array<float, 2> j1{};
  std::array<float, 2> j2{};
  std::array<float, 2> j3{};
  std::array<float, 2> j5{};
  float theta1_deg = 0.0f;
  float theta2_deg = 0.0f;
  float phi1_deg = 0.0f;
  float phi2_deg = 0.0f;
  float plate_angle_deg = 0.0f;
  float m3_angle_deg = 0.0f;
  float x_ref = 0.0f;
  float y_ref = 0.0f;
  float m1_angle_ref_deg = 0.0f;
  float m2_angle_ref_deg = 0.0f;
  float m3_angle_ref_deg = 0.0f;
};

float normalize_angle_deg(float angle_deg);

// Inverse kinematics: output point M3 -> crank angles (and every joint).
// Where more than one assembly satisfies the crossed-link invariant (this
// happens near the full-down pose), the branch whose crank angles are closest
// to `branch_reference` is returned; the default is the documented calibration
// pose of the as-built machine.
bool solve_ik_for_m3(float x_rel, float y_rel, IkSolution &solution,
                    const GeometryConfig &config = GeometryConfig{},
                    const IkReference &branch_reference = kFullDownReference);
bool solve_ik_for_m3_reference(float x_rel, float y_rel, const IkReference &reference,
                              IkSolution &solution,
                              const GeometryConfig &config = GeometryConfig{});

// Forward kinematics: crank angles theta1 (M1->J2) and theta2 (M2->J1) -> every
// joint, including the output point M3. Returns false if the cranks cannot be
// closed by the coupler or the pose leaves the crossed-link branch that the
// physical pins enforce (see Geometry.md section 6). This is the inverse of
// solve_ik_for_m3(): for any reachable M3, ik -> fk reproduces M3.
bool solve_fk(float theta1_deg, float theta2_deg, IkSolution &solution,
              const GeometryConfig &config = GeometryConfig{});
// Same, with the crank angles given relative to the reference (the convention
// the actuators are commanded in). The returned angles are relative too.
bool solve_fk_reference(float theta1_rel_deg, float theta2_rel_deg, const IkReference &reference,
                        IkSolution &solution, const GeometryConfig &config = GeometryConfig{});

} // namespace stem
