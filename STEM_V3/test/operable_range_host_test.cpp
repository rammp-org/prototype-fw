// Host test for operable_range.hpp (no ESP-IDF): the v3.1 reach-study bounds.
#include "operable_range.hpp"

#include <cstdio>
#include <cstdlib>

namespace {
int failures = 0;
void check(bool ok, const char *what) {
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    ++failures;
  }
}
} // namespace

int main() {
  const stem::OperableRange r{};
  // Poses from the reach study (a = enc101, b = enc103), and the software-zero
  // Home pose (Set left -11.11 / right 25.44 on 18 Nov 2026).
  check(r.base_contains(11.11f, -25.44f), "home (software zero pose) is inside");
  check(r.base_contains(20.0f, 0.0f), "calibration build pose (101 +20, 103 0) is inside");
  check(!r.base_contains(0.0f, 0.0f), "all-zero encoders (a collision) are outside");
  check(!r.base_contains(89.5f, -40.6f), "full up is 1.3 deg outside without tolerance");
  stem::OperableRange tolerant{};
  tolerant.tolerance_deg = 1.5f;
  check(tolerant.base_contains(89.5f, -40.6f), "full up is inside with 1.5 deg tolerance");
  check(!r.base_contains(5.9f, -30.0f) && !r.base_contains(90.1f, -42.0f), "a range is 6..90");
  check(!r.base_contains(30.0f, 12.5f) && !r.base_contains(30.0f, -47.5f), "b caps 12 / -47");
  check(!r.base_contains(10.0f, -14.0f), "b <= 1.9 a - 34 near the left edge");
  // pair angles are the primary (even) motors: a = -left, b = -right
  check(r.contains({-11.11f, 25.44f, 25.0f}, 0.0f), "pair-angle form, level seat");
  check(r.contains({-11.11f, 25.44f, 25.0f}, -20.0f) && r.contains({-11.11f, 25.44f, 25.0f}, 50.0f),
        "tilt limits are inclusive");
  check(!r.contains({-11.11f, 25.44f, 25.0f}, -20.5f) && !r.contains({-11.11f, 25.44f, 25.0f}, 50.5f),
        "tilt beyond -20 / +50 is outside");
  check(!r.contains({11.11f, -25.44f, 0.0f}, 0.0f), "wrong sign (odd motor angles) is outside");
  if (failures == 0) {
    std::printf("operable range host test: all checks passed\n");
  }
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
