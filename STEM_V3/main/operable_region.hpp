#pragma once

// Operable range of the seat axis M3, as a polygon, for routing pose moves
// around its curved edges (route_planner.hpp). GENERATED - do not edit by hand.
//
// Source: the Rev C encoder bounds (operable_range.hpp) and the pair limits,
// evaluated through the IK (calibrated zero -11.27 / 169.18, kFullDownReference)
// on a 1 mm grid of M3 positions, clipped to the design travel (max rear x = -34,
// max fwd x = 266, full up y = 568.5), in the analysis frame (origin M1, mm).
// Generated offline (grid -> polygon union -> 2 mm / 6 mm insets); redo it if
// the bounds, limits or calibration change - the console's REGION_VIS /
// REGION_WAYPOINTS hold the same data. The planner still checks every path point against the
// exact OperableRange; these outlines only choose the route.

#include <array>

namespace stem::region {

struct Point {
  float x;
  float y;
};

/// Region shrunk by 2 mm: a route segment must stay inside it.
inline constexpr std::array<Point, 87> kVisibility{{
    {-31.5f, 288.0f},
    {-21.5f, 234.0f},
    {-19.5f, 230.0f},
    {-17.7f, 216.0f},
    {-16.5f, 216.0f},
    {-13.7f, 201.0f},
    {-7.4f, 198.0f},
    {13.3f, 180.0f},
    {15.3f, 180.0f},
    {43.5f, 151.8f},
    {64.5f, 173.0f},
    {66.7f, 173.0f},
    {71.5f, 177.8f},
    {91.5f, 188.6f},
    {103.5f, 191.6f},
    {103.5f, 193.0f},
    {123.5f, 195.8f},
    {123.5f, 197.1f},
    {155.7f, 196.0f},
    {159.4f, 199.7f},
    {166.0f, 202.5f},
    {167.1f, 204.7f},
    {182.1f, 211.6f},
    {183.1f, 213.6f},
    {188.5f, 215.4f},
    {188.5f, 217.0f},
    {192.8f, 217.0f},
    {199.5f, 222.0f},
    {203.7f, 222.0f},
    {209.5f, 225.7f},
    {216.5f, 226.9f},
    {218.5f, 228.9f},
    {233.5f, 233.6f},
    {263.5f, 239.6f},
    {264.0f, 473.6f},
    {262.2f, 478.0f},
    {260.5f, 478.0f},
    {257.1f, 487.0f},
    {255.5f, 487.0f},
    {252.8f, 494.6f},
    {250.8f, 495.6f},
    {249.7f, 500.0f},
    {243.5f, 506.8f},
    {243.5f, 508.8f},
    {235.5f, 518.8f},
    {232.9f, 525.0f},
    {230.9f, 525.0f},
    {232.1f, 529.5f},
    {224.8f, 533.0f},
    {221.7f, 533.0f},
    {219.7f, 535.0f},
    {216.7f, 535.0f},
    {214.7f, 537.0f},
    {211.7f, 537.0f},
    {209.7f, 539.0f},
    {206.7f, 539.0f},
    {204.5f, 541.1f},
    {198.5f, 542.1f},
    {196.5f, 544.2f},
    {177.5f, 549.0f},
    {168.5f, 553.0f},
    {164.5f, 553.0f},
    {161.5f, 555.0f},
    {157.5f, 555.0f},
    {147.5f, 559.3f},
    {117.5f, 566.0f},
    {42.8f, 566.5f},
    {25.2f, 561.0f},
    {20.5f, 557.0f},
    {17.7f, 557.0f},
    {16.7f, 555.0f},
    {11.7f, 553.0f},
    {10.7f, 551.0f},
    {8.6f, 551.0f},
    {6.6f, 548.0f},
    {4.3f, 548.0f},
    {-6.5f, 537.2f},
    {-6.5f, 534.9f},
    {-9.5f, 532.9f},
    {-9.5f, 531.2f},
    {-14.8f, 525.9f},
    {-16.9f, 519.6f},
    {-18.9f, 518.6f},
    {-24.5f, 506.2f},
    {-24.5f, 503.2f},
    {-26.7f, 501.0f},
    {-32.0f, 481.7f}
}};

/// Concave corners of the region shrunk by 6 mm: the waypoints a route bends at.
inline constexpr std::array<Point, 18> kWaypoints{{
    {-23.5f, 274.0f},
    {17.0f, 184.0f},
    {62.5f, 177.0f},
    {80.8f, 189.0f},
    {99.5f, 197.0f},
    {119.5f, 201.2f},
    {164.4f, 208.2f},
    {184.5f, 221.0f},
    {195.5f, 226.0f},
    {205.5f, 229.1f},
    {229.5f, 236.9f},
    {256.5f, 474.0f},
    {251.5f, 483.0f},
    {239.5f, 504.3f},
    {225.8f, 521.0f},
    {153.5f, 551.0f},
    {24.5f, 553.0f},
    {-2.5f, 532.8f}
}};

} // namespace stem::region
