#pragma once

/// Hardware + control configuration for the holonomic-drive MIB on the Waveshare
/// ESP32-P4-ETH kit.
///
/// The board has no local UI: the joystick HMI (pace-hmi-fw) talks to it over
/// Ethernet with the shared RTPS messages (external/rammp-rtps), and the four
/// Reflex RMD-X6-S2 wheel motors hang off a CAN transceiver on the TWAI GPIOs.

#include <chrono>

#include <driver/gpio.h>

namespace hw_config {

/////////////////////////////////////////////////////////////////////////////
// CAN bus (RMD-X6-S2 motors, 1 Mbit/s)
/////////////////////////////////////////////////////////////////////////////

/// TWAI TX -> CAN transceiver TXD. The same pins the MIB project uses on this
/// kit; swap the two constants if the transceiver is wired the other way.
inline constexpr gpio_num_t kCanTxGpio = GPIO_NUM_17;
/// TWAI RX <- CAN transceiver RXD.
inline constexpr gpio_num_t kCanRxGpio = GPIO_NUM_16;
/// Bus bit rate the motors are configured for.
inline constexpr uint32_t kCanBitrateBps = 1'000'000;

/////////////////////////////////////////////////////////////////////////////
// Control parameters
/////////////////////////////////////////////////////////////////////////////

/// Fixed-rate control loop period (wheel-speed computation + CAN commands).
inline constexpr std::chrono::milliseconds kControlPeriod{20}; // 50 Hz
/// Motor status polling period (round-robin, one motor per tick).
inline constexpr std::chrono::milliseconds kStatusPollPeriod{250}; // 4 Hz: each motor at 1 Hz
/// Per-status-read reply timeout; bounds how long a poll holds the CAN mutex.
inline constexpr uint32_t kMotorStatusReadTimeoutMs = 20;
/// After the joystick has been centered for this long, control falls back to
/// the local (CLI) setpoint, which is zeroed on the handover.
inline constexpr std::chrono::milliseconds kJoystickReleaseTimeout{500};
/// A motor's status is reported STALE when it has not been read for this long.
inline constexpr std::chrono::milliseconds kMotorStatusStaleTimeout{3000};

/// Hard per-wheel OUTPUT-shaft speed limit (RPM): if any computed wheel speed
/// exceeds this, all wheels are scaled down together (preserving the motion
/// direction). Same value the Tab5 holo_deck firmware drives with.
inline constexpr float kMaxWheelRpm = 90.0f;

/// Translation-speed and chassis-rotation limits per HMI drive profile
/// (MIB::DriveProfile LOW / NORMAL / HIGH). NORMAL is what the Tab5 firmware
/// used as its default; the joystick's full deflection maps to these.
struct ProfileLimits {
  float max_speed_mps;
  float max_rotation_rpm;
};
inline constexpr ProfileLimits kProfileLow{0.5f, 3.0f};
inline constexpr ProfileLimits kProfileNormal{1.0f, 6.0f};
inline constexpr ProfileLimits kProfileHigh{1.5f, 9.0f};

/// Scale applied to the rotation rate commanded from the joystick TWIST axis
/// (twist * scale * max-rotation). The platform's wheel angles give it weak
/// rotation authority (see the kinematics note in HoloDeckPlatform), so the
/// Tab5 firmware exposed this as a slider; wheel commands stay bounded by
/// kMaxWheelRpm regardless.
inline constexpr float kTwistRotationScale = 1.0f;

/////////////////////////////////////////////////////////////////////////////
// RTPS link timing (must agree with the HMI: pace-hmi-fw main/hmi_rtps_spec.hpp)
/////////////////////////////////////////////////////////////////////////////

/// How often MibStatus and Diagnostics are published (the HMI expects 500 ms
/// and declares the link lost after 2000 ms without MibStatus).
inline constexpr std::chrono::milliseconds kStatusPublishPeriod{500};
/// The HMI streams XYTwist at ~30 Hz while driving. If none arrives for this
/// long while ENABLED, the platform is commanded to zero velocity (as if the
/// stick were centered).
inline constexpr std::chrono::milliseconds kJoystickHoldTimeout{500};
/// If no XYTwist arrives for this long while ENABLED, the joystick link is
/// considered lost: driving is disabled (e-stop) and MibStatus reports IDLE
/// with a "joystick link lost" note until the HMI enables driving again.
inline constexpr std::chrono::milliseconds kJoystickLostTimeout{2000};

} // namespace hw_config
