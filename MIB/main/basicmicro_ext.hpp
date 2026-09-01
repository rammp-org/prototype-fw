#pragma once

#include <mutex>
#include <system_error>
#include <vector>

#include "basicmicro.hpp"

namespace mib {

/// \brief espp::Basicmicro extended with the MCP position-PID commands
///        (61-64) that the upstream component does not implement.
class BasicmicroExt : public espp::Basicmicro {
public:
  using espp::Basicmicro::Basicmicro;

  /// \brief Set the motor 1 position PID constants (command 61).
  /// \details P/I/D are transferred scaled by 1024 (Basicmicro reference
  ///          library convention for the position loop; the velocity loop uses
  ///          65536). Deadzone is in encoder counts; min/max position bound
  ///          the moves the position loop will command. The factory default
  ///          for every position constant is zero, i.e. position commands
  ///          produce no motion until these are configured.
  bool set_position_pid_m1(float p, float i, float d, uint32_t max_i, uint32_t deadzone,
                           int32_t min_pos, int32_t max_pos, std::error_code &ec) {
    return set_position_pid(kSetPositionPidM1, p, i, d, max_i, deadzone, min_pos, max_pos, ec);
  }

  /// \brief Set the motor 2 position PID constants (command 62). See
  ///        set_position_pid_m1() for scaling.
  bool set_position_pid_m2(float p, float i, float d, uint32_t max_i, uint32_t deadzone,
                           int32_t min_pos, int32_t max_pos, std::error_code &ec) {
    return set_position_pid(kSetPositionPidM2, p, i, d, max_i, deadzone, min_pos, max_pos, ec);
  }

  /// \brief Read the motor 1 position PID constants (command 63).
  bool read_position_pid_m1(float &p, float &i, float &d, uint32_t &max_i, uint32_t &deadzone,
                            int32_t &min_pos, int32_t &max_pos, std::error_code &ec) {
    return read_position_pid(kReadPositionPidM1, p, i, d, max_i, deadzone, min_pos, max_pos, ec);
  }

  /// \brief Read the motor 2 position PID constants (command 64).
  bool read_position_pid_m2(float &p, float &i, float &d, uint32_t &max_i, uint32_t &deadzone,
                            int32_t &min_pos, int32_t &max_pos, std::error_code &ec) {
    return read_position_pid(kReadPositionPidM2, p, i, d, max_i, deadzone, min_pos, max_pos, ec);
  }

protected:
  static constexpr Command kSetPositionPidM1 = static_cast<Command>(61);
  static constexpr Command kSetPositionPidM2 = static_cast<Command>(62);
  static constexpr Command kReadPositionPidM1 = static_cast<Command>(63);
  static constexpr Command kReadPositionPidM2 = static_cast<Command>(64);
  static constexpr float kPositionPidScale = 1024.0f;

  /// Commands 61/62. Wire order is D, P, I, MaxI, Deadzone, MinPos, MaxPos.
  bool set_position_pid(Command cmd, float p, float i, float d, uint32_t max_i, uint32_t deadzone,
                        int32_t min_pos, int32_t max_pos, std::error_code &ec) {
    std::vector<uint8_t> payload;
    espp::detail::append_u32_be(payload, static_cast<uint32_t>(d * kPositionPidScale));
    espp::detail::append_u32_be(payload, static_cast<uint32_t>(p * kPositionPidScale));
    espp::detail::append_u32_be(payload, static_cast<uint32_t>(i * kPositionPidScale));
    espp::detail::append_u32_be(payload, max_i);
    espp::detail::append_u32_be(payload, deadzone);
    espp::detail::append_i32_be(payload, min_pos);
    espp::detail::append_i32_be(payload, max_pos);
    std::scoped_lock lk(mutex_);
    return write_command(cmd, payload, ec);
  }

  /// Commands 63/64. Reply order is P, I, D, MaxI, Deadzone, MinPos, MaxPos.
  bool read_position_pid(Command cmd, float &p, float &i, float &d, uint32_t &max_i,
                         uint32_t &deadzone, int32_t &min_pos, int32_t &max_pos,
                         std::error_code &ec) {
    uint8_t data[28] = {};
    {
      std::scoped_lock lk(mutex_);
      if (!read_command(cmd, data, ec)) {
        return false;
      }
    }
    p = static_cast<float>(espp::detail::read_u32_be(data, 0)) / kPositionPidScale;
    i = static_cast<float>(espp::detail::read_u32_be(data, 4)) / kPositionPidScale;
    d = static_cast<float>(espp::detail::read_u32_be(data, 8)) / kPositionPidScale;
    max_i = espp::detail::read_u32_be(data, 12);
    deadzone = espp::detail::read_u32_be(data, 16);
    min_pos = espp::detail::read_i32_be(data, 20);
    max_pos = espp::detail::read_i32_be(data, 24);
    return true;
  }
};

} // namespace mib
