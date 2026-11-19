#pragma once

// Routes a pose move around the curved edges of the operable range.
//
// When the straight line from the seat axis's position to the target leaves
// the operable range (its lower-left, lower-right and upper-right edges curve
// into it), the move instead follows the shortest path that stays inside: a
// visibility graph over the region's concave corners (operable_region.hpp),
// searched with Dijkstra. That is the textbook shortest path in a polygon - it
// only bends at concave corners, and runs straight everywhere else. The bends
// are then rounded (a small blend at each corner) so the seat does not change
// direction abruptly; LevelTrajectory runs the whole route as one move and
// still checks every point against the exact operable range.
//
// Pure geometry, analysis frame (mm), host-tested
// (test/level_trajectory_host_test.cpp).

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "operable_region.hpp"

namespace stem {

struct Vec2 {
  float x{0.0f};
  float y{0.0f};
};

class RoutePlanner {
public:
  /// Within this distance of either end, a segment may cross the visibility
  /// outline: the start (measured) or the target may sit in the 2 mm margin.
  static constexpr float kEndSlackMm = 4.0f;
  /// Largest corner blend (mm along each leg).
  static constexpr float kBlendMm = 15.0f;
  /// A blend cuts its corner by at most this much (mm): the waypoints sit 6 mm
  /// inside the region, the visibility outline 2 mm inside.
  static constexpr float kMaxCutMm = 2.5f;

  /// The segment a-b stays inside the region (2 mm margin).
  static bool visible(Vec2 a, Vec2 b) {
    const float len = std::hypot(b.x - a.x, b.y - a.y);
    if (len < 2.0f * kEndSlackMm) {
      return true; // too short to leave the region by more than the margin
    }
    const float t_lo = kEndSlackMm / len, t_hi = 1.0f - t_lo;
    const auto &poly = region::kVisibility;
    for (size_t i = 0; i < poly.size(); ++i) {
      const auto &p = poly[i];
      const auto &q = poly[(i + 1) % poly.size()];
      float t = 0.0f;
      if (intersects(a, b, {p.x, p.y}, {q.x, q.y}, t) && t > t_lo && t < t_hi) {
        return false;
      }
    }
    return inside({(a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f});
  }

  /// Shortest route from start to goal inside the region: the corners it bends
  /// at, then the goal (not the start). Empty when there is none.
  static std::vector<Vec2> route(Vec2 start, Vec2 goal) {
    if (visible(start, goal)) {
      return {goal};
    }
    // nodes: 0 = start, 1 = goal, 2.. = waypoints
    const auto &wp = region::kWaypoints;
    const size_t n = wp.size() + 2;
    auto node = [&](size_t i) -> Vec2 {
      return i == 0 ? start : i == 1 ? goal : Vec2{wp[i - 2].x, wp[i - 2].y};
    };
    std::vector<float> dist(n, std::numeric_limits<float>::infinity());
    std::vector<int> prev(n, -1);
    std::vector<bool> done(n, false);
    dist[0] = 0.0f;
    for (size_t iter = 0; iter < n; ++iter) {
      size_t u = n;
      for (size_t i = 0; i < n; ++i) {
        if (!done[i] && std::isfinite(dist[i]) && (u == n || dist[i] < dist[u])) {
          u = i;
        }
      }
      if (u == n || u == 1) {
        break;
      }
      done[u] = true;
      const Vec2 a = node(u);
      for (size_t v = 1; v < n; ++v) {
        if (done[v] || v == u) {
          continue;
        }
        const Vec2 b = node(v);
        const float d = dist[u] + std::hypot(b.x - a.x, b.y - a.y);
        if (d < dist[v] && visible(a, b)) {
          dist[v] = d;
          prev[v] = static_cast<int>(u);
        }
      }
    }
    if (!std::isfinite(dist[1])) {
      return {};
    }
    std::vector<Vec2> path;
    for (int v = 1; v != 0; v = prev[static_cast<size_t>(v)]) {
      path.push_back(node(static_cast<size_t>(v)));
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  /// Round every interior corner of start -> path with a quadratic blend:
  /// returns the dense points after start (the goal last).
  static std::vector<Vec2> round_corners(Vec2 start, const std::vector<Vec2> &path) {
    if (path.size() < 2) {
      return path;
    }
    std::vector<Vec2> out;
    Vec2 prev = start;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
      const Vec2 w = path[i], next = path[i + 1];
      const float lp = std::hypot(prev.x - w.x, prev.y - w.y);
      const float ln = std::hypot(next.x - w.x, next.y - w.y);
      if (lp < 1e-3f || ln < 1e-3f) {
        out.push_back(w);
        prev = w;
        continue;
      }
      const Vec2 up{(prev.x - w.x) / lp, (prev.y - w.y) / lp};
      const Vec2 un{(next.x - w.x) / ln, (next.y - w.y) / ln};
      // the blend's middle sits 0.5*d*cos(phi/2) inside the corner (phi = angle
      // between the legs): keep that cut under kMaxCutMm
      const float cos_half = std::sqrt(std::max(0.0f, 0.5f * (1.0f + up.x * un.x + up.y * un.y)));
      float d = std::min({kBlendMm, 0.45f * lp, 0.45f * ln});
      if (cos_half > 1e-3f) {
        d = std::min(d, 2.0f * kMaxCutMm / cos_half);
      }
      const Vec2 a{w.x + up.x * d, w.y + up.y * d}, b{w.x + un.x * d, w.y + un.y * d};
      out.push_back(a);
      constexpr int kSteps = 8;
      for (int k = 1; k < kSteps; ++k) {
        const float t = static_cast<float>(k) / kSteps, s = 1.0f - t;
        out.push_back({s * s * a.x + 2 * s * t * w.x + t * t * b.x,
                       s * s * a.y + 2 * s * t * w.y + t * t * b.y});
      }
      out.push_back(b);
      prev = b;
    }
    out.push_back(path.back());
    return out;
  }

  /// Point inside the visibility outline (ray casting).
  static bool inside(Vec2 p) {
    const auto &poly = region::kVisibility;
    bool in = false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
      const auto &a = poly[i], &b = poly[j];
      if (((a.y > p.y) != (b.y > p.y)) && (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)) {
        in = !in;
      }
    }
    return in;
  }

private:
  // segment a-b crosses segment p-q; t = position of the crossing along a-b
  static bool intersects(Vec2 a, Vec2 b, Vec2 p, Vec2 q, float &t) {
    const float rx = b.x - a.x, ry = b.y - a.y, sx = q.x - p.x, sy = q.y - p.y;
    const float den = rx * sy - ry * sx;
    if (std::fabs(den) < 1e-9f) {
      return false; // parallel
    }
    const float qpx = p.x - a.x, qpy = p.y - a.y;
    t = (qpx * sy - qpy * sx) / den;
    const float u = (qpx * ry - qpy * rx) / den;
    return t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f;
  }
};

} // namespace stem
