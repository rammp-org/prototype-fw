// Host test for level_trajectory.hpp (no ESP-IDF): the seat stays at its
// commanded tilt along a planned pose move, every pair moves together, and the
// path respects the speed limit and the operable range.
#include "level_trajectory.hpp"

#include <cmath>
#include <cstdio>
#include <random>

namespace {
int failures = 0;
void check(bool ok, const char *what) {
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    ++failures;
  }
}
using stem::LevelTrajectory;
const stem::IkReference ref = stem::kFullDownReference;
const stem::GeometryConfig geo{};
const stem::OperableRange range{};
const std::array<float, 3> lo{-95.0f, -17.0f, -55.0f}, hi{-1.0f, 52.0f, 155.0f};

// pair angles of an absolute M3 point (analysis frame) at a tilt
bool pairs_at(float x, float y, float tilt, LevelTrajectory::Pairs &p) {
  stem::IkSolution s{};
  if (!stem::solve_ik_for_m3_reference(x - ref.x, y - ref.y, ref, s, geo)) return false;
  p = {-s.theta1_deg, -s.theta2_deg, -s.m3_angle_deg + tilt};
  return true;
}
// seat tilt from level of a pair triple (inverse of the pair convention)
float tilt_of(const LevelTrajectory::Pairs &p) {
  stem::IkSolution s{};
  if (!stem::solve_fk_reference(-p[0], -p[1], ref, s, geo)) return NAN;
  return p[2] + s.m3_angle_deg;
}
// walk a plan: worst tilt error after the seat first reached `tilt`, max pair speed,
// and whether left / right stayed put until the seat was level
struct Walk { float worst_tilt = 0, max_speed = 0; bool level_first = true; };
Walk walk(const LevelTrajectory &t, float tilt) {
  Walk w; const float dt = 0.002f; bool levelled = false;
  const auto p0 = t.at(0.0f);
  for (float time = 0; time <= t.duration() + 0.01f; time += dt) {
    const auto a = t.at(time), b = t.at(time + dt);
    for (int k = 0; k < 3; ++k) w.max_speed = std::max(w.max_speed, std::fabs(b[k] - a[k]) / dt);
    const float e = std::fabs(tilt_of(a) - tilt);
    if (e < 0.02f) levelled = true;
    if (levelled) w.worst_tilt = std::max(w.worst_tilt, e);
    else if (std::fabs(a[0] - p0[0]) > 1e-4f || std::fabs(a[1] - p0[1]) > 1e-4f) w.level_first = false;
  }
  return w;
}
} // namespace

int main() {
  stem::LevelTrajectoryLimits lim{};
  LevelTrajectory::Pairs start{}, goal{};
  // CAD home -> high and forward, seat level
  check(pairs_at(66.0f, 208.5f, 0.0f, start) && pairs_at(206.0f, 540.0f, 0.0f, goal), "IK of the test poses");
  LevelTrajectory t;
  check(t.plan(start, 206.0f - ref.x, 540.0f - ref.y, 0.0f, ref, geo, range, lo, hi, lim) ==
            LevelTrajectory::Result::Ok, "home -> high forward plans");
  Walk w = walk(t, 0.0f);
  std::printf("home -> (206, 540): %.2f s, %zu samples, worst tilt %.3f deg, max pair speed %.2f deg/s\n",
              t.duration(), t.samples(), w.worst_tilt, w.max_speed);
  check(w.worst_tilt < 0.05f, "seat stays level along the whole move");
  check(w.max_speed < lim.speed_deg_s * 1.03f, "no pair faster than the speed limit");
  for (int k = 0; k < 3; ++k) check(std::fabs(t.end()[k] - goal[k]) < 1e-3f, "ends at the target's pair angles");

  // start 10 deg off level: seat levels first, left / right wait
  auto tilted = start; tilted[2] += 10.0f;
  check(t.plan(tilted, 206.0f - ref.x, 540.0f - ref.y, 0.0f, ref, geo, range, lo, hi, lim) ==
            LevelTrajectory::Result::Ok, "tilted start plans");
  w = walk(t, 0.0f);
  check(w.level_first, "a tilted seat is levelled before left / right move");
  check(w.worst_tilt < 0.05f, "and stays level after");

  // with a tilt: keeps the tilt
  LevelTrajectory::Pairs s10{};
  pairs_at(66.0f, 208.5f, 10.0f, s10);
  check(t.plan(s10, 150.0f - ref.x, 400.0f - ref.y, 10.0f, ref, geo, range, lo, hi, lim) ==
            LevelTrajectory::Result::Ok, "tilted move plans");
  check(walk(t, 10.0f).worst_tilt < 0.05f, "a 10 deg tilt is held along the move");

  // outside the operable range: rejected, nothing to run
  check(t.plan(start, 300.0f - ref.x, 300.0f - ref.y, 0.0f, ref, geo, range, lo, hi, lim) !=
            LevelTrajectory::Result::Ok, "a target outside the range is rejected");

  // a re-plan at speed starts at that speed and never exceeds the limit
  check(t.plan(start, 206.0f - ref.x, 540.0f - ref.y, 0.0f, ref, geo, range, lo, hi, lim, 8.0f) ==
            LevelTrajectory::Result::Ok && std::fabs(t.speed_at(0.0f) - 8.0f) < 1e-3f, "re-plan keeps the initial speed");
  check(walk(t, 0.0f).max_speed < lim.speed_deg_s * 1.03f, "re-plan stays under the limit");

  // random moves inside the range: level everywhere when planned
  std::mt19937 rng(7);
  std::uniform_real_distribution<float> X(-34.0f, 266.0f), Y(150.0f, 568.0f);
  int planned = 0, rejected = 0; float worst = 0;
  for (int i = 0; i < 300; ++i) {
    LevelTrajectory::Pairs a{}, b{};
    const float x0 = X(rng), y0 = Y(rng), x1 = X(rng), y1 = Y(rng);
    if (!pairs_at(x0, y0, 0.0f, a) || !pairs_at(x1, y1, 0.0f, b)) continue;
    if (!range.contains(a, 0.0f) || !range.contains(b, 0.0f)) continue;
    if (t.plan(a, x1 - ref.x, y1 - ref.y, 0.0f, ref, geo, range, lo, hi, lim) != LevelTrajectory::Result::Ok) { ++rejected; continue; }
    ++planned; worst = std::max(worst, walk(t, 0.0f).worst_tilt);
  }
  std::printf("random moves: %d planned, %d rejected (path leaves the range), worst tilt %.3f deg\n", planned, rejected, worst);
  check(planned > 50 && worst < 0.05f, "random in-range moves stay level");

  if (failures == 0) std::printf("level_trajectory: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
