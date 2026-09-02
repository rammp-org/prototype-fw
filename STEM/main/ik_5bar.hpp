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

inline constexpr IkReference kFullDownReference{170.0f, 230.0f, -0.16f, 143.74f, 121.23f};

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
bool solve_ik_for_m3(float x_rel, float y_rel, IkSolution &solution,
                    const GeometryConfig &config = GeometryConfig{});
bool solve_ik_for_m3_reference(float x_rel, float y_rel, const IkReference &reference,
                              IkSolution &solution,
                              const GeometryConfig &config = GeometryConfig{});

} // namespace stem
