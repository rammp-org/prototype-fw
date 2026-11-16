#pragma once

// Planar 5-bar (two-DOF) linkage kinematics with a configurable geometry:
//
//          M1 (origin) ---- base ---- M2 (m2_x, m2_y)
//           |                          |
//     crank1 (theta1)            crank2 (theta2)
//           |                          |
//          J2 ------- bar -------- J3 ------- coupler ------- J1
//                                   |
//                 ee_extension along J2->J3, ee_offset 90 deg CCW of it
//                                   |
//                                  M3  <- the end effector (the wheel axle)
//
// Four pinned links close the loop M1 -> J2 -> J3 -> J1 -> M2; the two cranks
// are the actuated joints (angles from +x, counter-clockwise positive, in the
// linkage's own frame: M1 at the origin, +x toward M2). The end effector M3 is
// J3 itself (ee_extension = ee_offset = 0, the classic 5-bar) or a point on
// the rigid J2-J3 bar extended past J3 (the STEM seat linkage: bar 170,
// extension 100).
//
// A 5-bar has several assemblies for one end-effector point (each dyad can
// fold either way); the machine lives in one of them and never jumps. The
// inverse kinematics therefore returns the candidate that is closest to a
// reference pose (the measured one, or the configured home), optionally
// restricted to the cranks-crossed (STEM) or cranks-open assembly, and the
// coupler/bar dyad's fold (J3 on the CCW or the CW side of J1->J2) is a fixed
// property of the machine that both solvers honour.
//
// Standard-library only: shared by the firmware, the host test and (as a
// line-by-line port) the console web app, which must command exactly the pose
// the device will solve.

#include <array>
#include <cstdint>

namespace fivebar {

using Point = std::array<float, 2>;

/// Which crank assembly the machine is built in.
enum class Assembly : uint8_t {
  Any = 0,     ///< whichever candidate is closest to the reference pose (continuity)
  Crossed = 1, ///< the cranks cross (STEM): J1 left of M1->J2 and J2 right of M2->J1
  Open = 2,    ///< the loop M1-J2-J3-J1-M2 is a simple polygon: no two non-adjacent links
               ///< touch (cranks apart, bar and coupler not folded back across a crank
               ///< or the ground, the end-effector arm likewise); see loop_is_open()
};

/// On which side of the J1->J2 line the bar/coupler joint J3 folds.
enum class CouplerSide : uint8_t {
  Ccw = 0, ///< J3 on the counter-clockwise side of J1->J2 (STEM)
  Cw = 1,
};

struct Geometry {
  float m2_x{200.0f}; ///< M2 relative to M1 (mm)
  float m2_y{0.0f};
  float crank1{150.0f};     ///< M1 -> J2
  float crank2{150.0f};     ///< M2 -> J1
  float bar{250.0f};        ///< J2 -> J3 (the distal link of crank 1; the end effector rides on it)
  float coupler{250.0f};    ///< J1 -> J3 (the distal link of crank 2)
  float ee_extension{0.0f}; ///< M3 past J3 along J2->J3 (0 = M3 is J3)
  float ee_offset{0.0f};    ///< M3 perpendicular offset, 90 deg CCW of J2->J3
  Assembly assembly{Assembly::Any};
  CouplerSide coupler_side{CouplerSide::Ccw};
};

/// A solved pose: every joint, the crank angles and the bar's direction.
struct Pose {
  Point m3{};
  Point j1{};
  Point j2{};
  Point j3{};
  float theta1_deg{0.0f};    ///< M1 -> J2 from +x, CCW
  float theta2_deg{0.0f};    ///< M2 -> J1 from +x, CCW
  float bar_angle_deg{0.0f}; ///< J2 -> J3 from +x, CCW
};

/// (-180, 180]
float normalize_angle_deg(float angle_deg);

/// The Assembly::Open test on the loop points: the pentagon M1 -> J2 -> J3 ->
/// J1 -> M2 is a simple polygon (no two non-adjacent links intersect: crank 1
/// vs crank 2 and vs the coupler, the bar vs crank 2 and vs the ground segment
/// between the motor mounts, the coupler vs the ground), and the end-effector
/// arm past J3 (when there is one) crosses neither crank nor the ground.
bool loop_is_open(const Geometry &geometry, const Point &m1, const Point &m2, const Point &j1,
                  const Point &j2, const Point &j3);

/// Every length positive and the loop closable at all (crank1 + bar and
/// crank2 + coupler can meet somewhere).
bool is_valid(const Geometry &geometry);

/// Forward kinematics: crank angles -> pose. False when the bar and the
/// coupler cannot meet (the loop does not close) or the pose violates the
/// configured assembly.
bool solve_fk(float theta1_deg, float theta2_deg, const Geometry &geometry, Pose &pose);

/// Inverse kinematics: end effector (x, y) -> pose, the assembly candidate
/// closest to the reference crank angles (sum of the absolute angle
/// differences). False when the point is out of reach or no candidate
/// satisfies the configured assembly / coupler side. For every reachable
/// point, solve_fk(pose.theta1, pose.theta2) reproduces (x, y).
bool solve_ik(float x, float y, const Geometry &geometry, float reference_theta1_deg,
              float reference_theta2_deg, Pose &pose);

} // namespace fivebar
