// Host-side test for the PEAR linkage wire protocol (linkage_protocol.hpp):
// every fixed-layout payload the firmware builds parses back to the same
// values, the sizes match the documented layouts, and the settings validation
// catches the mistakes the console could send. No ESP-IDF needed:
//
//   c++ -std=c++20 -I ../main protocol_host_test.cpp ../main/five_bar.cpp && ./a.out

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <span>
#include <string>

#include "linkage_protocol.hpp"

namespace proto = linkage_protocol;

namespace {

int failures = 0;

void check(bool ok, const char *what) {
  if (!ok) {
    ++failures;
    std::printf("FAIL: %s\n", what);
  }
}

bool near(float a, float b, float tol = 1e-6f) { return std::fabs(a - b) <= tol; }

void test_settings_round_trip() {
  proto::Settings s;
  s.geometry.m2_x = 123.5f;
  s.geometry.m2_y = -4.0f;
  s.geometry.crank1 = 310.0f;
  s.geometry.crank2 = 270.0f;
  s.geometry.bar = 170.0f;
  s.geometry.coupler = 270.0f;
  s.geometry.ee_extension = 100.0f;
  s.geometry.ee_offset = 12.0f;
  s.geometry.assembly = fivebar::Assembly::Crossed;
  s.geometry.coupler_side = fivebar::CouplerSide::Cw;
  s.home_theta1_deg = 14.03f;
  s.home_theta2_deg = 168.88f;
  s.motors[0] = {.node_id = 107,
                 .direction = -1,
                 .min_deg = -60.0f,
                 .max_deg = 60.0f,
                 .theta_at_zero_deg = -0.16f,
                 .torque_limit_permille = 450};
  s.motors[1] = {.node_id = 108,
                 .direction = 1,
                 .min_deg = -45.5f,
                 .max_deg = 90.0f,
                 .theta_at_zero_deg = 143.74f,
                 .torque_limit_permille = 500};
  s.profile_velocity_dps = 60.0f;
  s.profile_accel_dps2 = 20.0f;
  s.profile_decel_dps2 = 25.0f;
  const auto bytes = s.serialize();
  check(bytes.size() == proto::Settings::kSize, "settings size matches kSize");
  const auto back = proto::Settings::parse(bytes);
  check(back.has_value(), "settings parse");
  if (!back)
    return;
  const auto &b = *back;
  check(near(b.geometry.m2_x, 123.5f) && near(b.geometry.m2_y, -4.0f), "settings m2");
  check(near(b.geometry.crank1, 310.0f) && near(b.geometry.crank2, 270.0f), "settings cranks");
  check(near(b.geometry.bar, 170.0f) && near(b.geometry.coupler, 270.0f), "settings bar/coupler");
  check(near(b.geometry.ee_extension, 100.0f) && near(b.geometry.ee_offset, 12.0f),
        "settings end effector");
  check(b.geometry.assembly == fivebar::Assembly::Crossed, "settings assembly");
  check(b.geometry.coupler_side == fivebar::CouplerSide::Cw, "settings coupler side");
  check(near(b.home_theta1_deg, 14.03f) && near(b.home_theta2_deg, 168.88f), "settings home");
  check(b.motors[0].node_id == 107 && b.motors[0].direction == -1, "motor 1 id / direction");
  check(near(b.motors[0].min_deg, -60.0f) && near(b.motors[0].max_deg, 60.0f), "motor 1 limits");
  check(near(b.motors[0].theta_at_zero_deg, -0.16f), "motor 1 theta at zero");
  check(b.motors[0].torque_limit_permille == 450, "motor 1 torque limit");
  check(b.motors[1].node_id == 108 && b.motors[1].direction == 1, "motor 2 id / direction");
  check(near(b.motors[1].min_deg, -45.5f) && near(b.motors[1].max_deg, 90.0f), "motor 2 limits");
  check(near(b.motors[1].theta_at_zero_deg, 143.74f), "motor 2 theta at zero");
  check(b.motors[1].torque_limit_permille == 500, "motor 2 torque limit");
  check(near(b.profile_velocity_dps, 60.0f) && near(b.profile_accel_dps2, 20.0f) &&
            near(b.profile_decel_dps2, 25.0f),
        "settings profile");
  std::string why;
  check(b.validate(500, why), ("valid settings validate: " + why).c_str());
  // a short or foreign-version payload is refused
  std::vector<uint8_t> shorter(bytes.begin(), bytes.end() - 1);
  check(!proto::Settings::parse(shorter).has_value(), "short settings refused");
  auto wrong_version = bytes;
  wrong_version[0] = 9;
  check(!proto::Settings::parse(wrong_version).has_value(), "settings version 9 refused");
}

void test_settings_validation() {
  std::string why;
  proto::Settings s;
  s.motors[0].node_id = 1;
  s.motors[1].node_id = 2;
  check(s.validate(1000, why), "defaults validate");
  auto bad = s;
  bad.geometry.crank1 = 0.0f;
  check(!bad.validate(1000, why), "zero crank refused");
  bad = s;
  bad.motors[0].min_deg = 10.0f;
  bad.motors[0].max_deg = -10.0f;
  check(!bad.validate(1000, why), "inverted limits refused");
  bad = s;
  bad.motors[1].direction = 2;
  check(!bad.validate(1000, why), "direction 2 refused");
  bad = s;
  bad.motors[1].torque_limit_permille = 800;
  check(!bad.validate(500, why), "torque limit above the ceiling refused");
  check(why.find("maximum") != std::string::npos, "torque limit error names the maximum");
  bad = s;
  bad.motors[1].node_id = 1;
  check(!bad.validate(1000, why), "duplicate node ids refused");
  bad = s;
  bad.profile_velocity_dps = 0.0f;
  check(!bad.validate(1000, why), "zero profile velocity refused");
  bad = s;
  bad.motors[0].node_id = 0;
  bad.motors[1].node_id = 0;
  check(bad.validate(1000, why), "no motors at all is allowed (bench without motors)");
}

void test_motor_angle_mapping() {
  proto::MotorSettings m{.node_id = 1,
                         .direction = -1,
                         .min_deg = -90,
                         .max_deg = 90,
                         .theta_at_zero_deg = 120.0f,
                         .torque_limit_permille = 300};
  // the crank at 120 deg reads 0; CW direction: crank at 100 deg -> motor +20
  check(near(m.theta_to_motor(120.0f), 0.0f), "theta at zero -> 0");
  check(near(m.theta_to_motor(100.0f), 20.0f), "cw mapping");
  check(near(m.motor_to_theta(20.0f), 100.0f), "cw inverse");
  // the wrap: the crank at -170 deg (= 190 deg) is 70 deg CCW of the 120 deg
  // zero, so a CW-positive motor reads -70
  check(near(m.theta_to_motor(-170.0f), -70.0f), "wrap through 180");
  check(near(m.motor_to_theta(-70.0f), -170.0f), "wrap inverse");
  check(m.within_limits(90.0f) && !m.within_limits(90.5f), "limit check");
}

void test_status_round_trip() {
  proto::Status s;
  s.stream_enabled = true;
  s.can_ok = true;
  s.pose_valid = true;
  s.target_valid = false;
  s.comm_ok = true;
  s.enabled = false;
  s.faulted = true;
  s.moving = true;
  s.sequence = 123456;
  s.timestamp_ms = 987654321;
  s.stream_period_ms = 100;
  s.x = 12.5f;
  s.y = -300.25f;
  s.theta1_deg = 95.5f;
  s.theta2_deg = 84.5f;
  s.bar_angle_deg = -20.0f;
  s.target_x = std::numeric_limits<float>::quiet_NaN();
  s.target_y = std::numeric_limits<float>::quiet_NaN();
  s.motors[0] = {.node_id = 107,
                 .present = true,
                 .comm_ok = true,
                 .enabled = true,
                 .faulted = false,
                 .target_reached = true,
                 .zero_set = true,
                 .statusword = 0x1637,
                 .error_code = 0,
                 .angle_deg = 12.25f,
                 .velocity_dps = -3.5f,
                 .torque_permille = -120,
                 .current_permille = 55,
                 .torque_limit_permille = 450};
  s.motors[1] = {.node_id = 108,
                 .present = true,
                 .comm_ok = false,
                 .enabled = false,
                 .faulted = true,
                 .target_reached = false,
                 .zero_set = false,
                 .statusword = 0x0008,
                 .error_code = 0x7121,
                 .angle_deg = -88.0f,
                 .velocity_dps = 0.0f,
                 .torque_permille = 0,
                 .current_permille = 0,
                 .torque_limit_permille = 300,
                 .rated_torque_cnm = 1250};
  s.can = {.state = proto::CanStatus::kErrorWarning,
           .tx_error_count = 97,
           .rx_error_count = 3,
           .bus_error_count = 1000,
           .bus_off_count = 2,
           .signal_errors = 7,
           .ack_errors = 123456};
  const auto bytes = s.serialize();
  check(bytes.size() == proto::Status::kSize, "status size matches kSize");
  check(proto::Status::kSize == proto::Status::kSizeV1 + proto::CanStatus::kSize &&
            proto::Status::kSizeV1 == 87,
        "v2 status is the v1 layout (87 B) plus the CAN block");
  // a v1 device sends 87 bytes: still parses, the CAN block stays zero
  {
    const auto v1 =
        proto::Status::parse(std::span<const uint8_t>(bytes).first(proto::Status::kSizeV1));
    check(v1.has_value() && v1->can.state == proto::CanStatus::kNotStarted &&
              v1->can.ack_errors == 0 && v1->motors[1].rated_torque_cnm == 1250,
          "a v1-length status parses with an empty CAN block");
  }
  const auto back = proto::Status::parse(bytes);
  check(back.has_value(), "status parse");
  if (!back)
    return;
  const auto &b = *back;
  check(b.can.state == proto::CanStatus::kErrorWarning && b.can.tx_error_count == 97 &&
            b.can.rx_error_count == 3 && b.can.bus_error_count == 1000 &&
            b.can.bus_off_count == 2 && b.can.signal_errors == 7 && b.can.ack_errors == 123456,
        "CAN block round-trips");
  check(b.motors[1].rated_torque_cnm == 1250 && b.motors[0].rated_torque_cnm == 0,
        "rated torque round-trips (0 = not read yet)");
  check(b.stream_enabled && b.can_ok && b.pose_valid && !b.target_valid && b.comm_ok &&
            !b.enabled && b.faulted && b.moving,
        "status flags");
  check(b.sequence == 123456 && b.timestamp_ms == 987654321 && b.stream_period_ms == 100,
        "status counters");
  check(near(b.x, 12.5f) && near(b.y, -300.25f) && near(b.theta1_deg, 95.5f) &&
            near(b.theta2_deg, 84.5f) && near(b.bar_angle_deg, -20.0f),
        "status pose");
  check(std::isnan(b.target_x) && std::isnan(b.target_y), "status NaN target survives");
  const auto &m0 = b.motors[0];
  check(m0.node_id == 107 && m0.present && m0.comm_ok && m0.enabled && !m0.faulted &&
            m0.target_reached && m0.zero_set,
        "motor 1 flags");
  check(m0.statusword == 0x1637 && m0.error_code == 0, "motor 1 words");
  check(near(m0.angle_deg, 12.25f) && near(m0.velocity_dps, -3.5f), "motor 1 readings");
  check(m0.torque_permille == -120 && m0.current_permille == 55 && m0.torque_limit_permille == 450,
        "motor 1 torque / current");
  const auto &m1 = b.motors[1];
  check(m1.node_id == 108 && !m1.comm_ok && m1.faulted && m1.error_code == 0x7121 &&
            near(m1.angle_deg, -88.0f),
        "motor 2");
}

void test_small_payloads() {
  check(proto::SetStream{true, 250}.serialize().size() == 3, "SetStream size");
  const auto ss = proto::SetStream::parse(proto::SetStream{true, 250}.serialize());
  check(ss && ss->enabled && ss->period_ms == 250, "SetStream round trip");
  const auto pt = proto::Point2::parse(proto::Point2{-12.5f, 300.0f}.serialize());
  check(pt && near(pt->x, -12.5f) && near(pt->y, 300.0f), "Point2 round trip");
  const auto sm = proto::SetMotor::parse(proto::SetMotor{1, -33.3f}.serialize());
  check(sm && sm->motor == 1 && near(sm->degrees, -33.3f), "SetMotor round trip");
  const auto zm = proto::ZeroMotor::parse(proto::ZeroMotor{1, 2, 123.0f}.serialize());
  check(zm && zm->motor == 1 && zm->mode == 2 && near(zm->value, 123.0f), "ZeroMotor round trip");
  check(!proto::ZeroMotor::parse(std::vector<uint8_t>{1, 2, 3}).has_value(),
        "short ZeroMotor refused");
  const auto ms = proto::MotorSelect::parse(std::vector<uint8_t>{0xFF});
  check(ms && ms->motor == proto::kAllMotors, "MotorSelect all");
  proto::IkResult r;
  r.x = 1.0f;
  r.y = 2.0f;
  r.reachable = true;
  r.within_limits = false;
  r.theta1_deg = 10.0f;
  r.theta2_deg = 20.0f;
  r.motor_deg = {-10.0f, 20.0f};
  r.pose.j1 = {1.0f, 2.0f};
  r.pose.j2 = {3.0f, 4.0f};
  r.pose.j3 = {5.0f, 6.0f};
  r.pose.m3 = {7.0f, 8.0f};
  const auto rb = r.serialize();
  check(rb.size() == proto::IkResult::kSize, "IkResult size");
  const auto rr = proto::IkResult::parse(rb);
  check(rr && rr->reachable && !rr->within_limits && near(rr->theta1_deg, 10.0f) &&
            near(rr->motor_deg[1], 20.0f) && near(rr->pose.m3[0], 7.0f) &&
            near(rr->pose.j1[1], 2.0f),
        "IkResult round trip");
  const auto err = proto::make_error_payload(0x16, 22, "unreachable");
  check(err.size() == 1 + 4 + 11 && err[0] == 0x16 && proto::get_u32(err, 1) == 22,
        "error payload");
  proto::Info info{.max_torque_permille = 500,
                   .poll_period_ms = 50,
                   .project = "pear_linkage",
                   .version = "1.0",
                   .build = "today"};
  const auto ib = info.serialize();
  check(ib.size() == 6 + 1 + 12 + 1 + 3 + 1 + 5 && ib[0] == proto::kProtocolVersion && ib[1] == 2,
        "info payload");
}

} // namespace

int main() {
  test_settings_round_trip();
  test_settings_validation();
  test_motor_angle_mapping();
  test_status_round_trip();
  test_small_payloads();
  if (failures) {
    std::printf("%d failure(s)\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("protocol host test: all checks passed\n");
  return EXIT_SUCCESS;
}
