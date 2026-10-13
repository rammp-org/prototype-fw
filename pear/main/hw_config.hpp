#pragma once

/// Hardware + control configuration for the PEAR prototype base's MIB on the
/// Waveshare ESP32-P4-ETH kit.
///
/// The board has no local UI: the joystick HMI (pace-hmi-fw) talks to it over
/// Ethernet with the shared RTPS messages (external/rammp-rtps). One CAN bus
/// carries eight CANopen DS402 nodes: four AMPS L2DB4830 hub-wheel drives
/// (velocity) and four Eyou linkage motors (profile position), one of each per
/// corner. Every per-corner table below is in pear::Corner order (FR, FL, BR,
/// BL).
///
/// Values marked TODO(hardware) are placeholders to be measured / verified on
/// the kit before the base carries a person.

#include <array>
#include <chrono>
#include <cstdint>

#include <driver/gpio.h>

#include "ds402_velocity_motor.hpp"
#include "eyou_motor.hpp"
#include "holo_deck_platform.hpp"
#include "linkage_geometry.hpp"

namespace hw_config {

/////////////////////////////////////////////////////////////////////////////
// CAN bus (one bus, 8 CANopen nodes, 1 Mbit/s)
/////////////////////////////////////////////////////////////////////////////

/// TWAI TX -> CAN transceiver TXD, TWAI RX <- transceiver RXD. These are the
/// pins every project in this repo drives the kit's transceiver with: STEM_V3
/// names them the other way round (kCanTxGpio = 16, kCanRxGpio = 17) but
/// passes them to CanBus(rx_gpio, tx_gpio) in that order, so its node ends up
/// with RX = 16 / TX = 17, which is also what holo_deck_mib and MIB use.
/// main.cpp passes these to the ctor's matching parameters.
/// TODO(hardware): verify on the PEAR kit (a wrong pair shows as "CAN bus
/// started" but every SDO timing out); swap the two values if so.
inline constexpr gpio_num_t kCanTxGpio = GPIO_NUM_17;
inline constexpr gpio_num_t kCanRxGpio = GPIO_NUM_16;
/// Bus bit rate every node is configured for.
inline constexpr uint32_t kCanBitrateBps = 1'000'000;

/////////////////////////////////////////////////////////////////////////////
// Motor CAN node ids (pear::Corner order: FR, FL, BR, BL)
/////////////////////////////////////////////////////////////////////////////

/// Hub-wheel drives (AMPS L2DB4830-CAFC, velocity set-points).
inline constexpr std::array<uint8_t, pear::kCornerCount> kWheelNodeIds = {2, 1, 3, 4};
/// Corner-linkage motors (Eyou, profile position, zero kept in NVS).
inline constexpr std::array<uint8_t, pear::kCornerCount> kLinkageNodeIds = {107, 108, 109, 110};

/////////////////////////////////////////////////////////////////////////////
// Base geometry (drive kinematics, HoloDeckPlatform)
/////////////////////////////////////////////////////////////////////////////

/// Nominal footprint and wheels: the holodeck's values (the PEAR base is the
/// same layout: four omniwheels at +/-30 deg / +/-150 deg). The linkages swing
/// the contact points a little fore/aft; that footprint change is ignored
/// (fixed configurable geometry) - a follow-up.
/// TODO(hardware): measure on the PEAR base.
inline constexpr float kHalfLengthM = 0.406f;
inline constexpr float kHalfWidthM = 0.267f;
inline constexpr float kWheelDiameterM = 0.21f;
/// Wheel drive directions in HoloDeckPlatform wheel order (TL, TR, BL, BR).
inline constexpr std::array<float, 4> kWheelAnglesDegrees = {30.0f, 150.0f, -30.0f, -150.0f};
/// Per-wheel direction sign in HoloDeckPlatform wheel order (+1 / -1).
inline constexpr std::array<float, 4> kWheelDirections = {1.0f, 1.0f, 1.0f, 1.0f};

/// Which wheel-drive node each HoloDeckPlatform wheel (TL, TR, BL, BR) commands.
///
/// This is the holodeck's hardware-tuned mapping ({2, 4, 3, 1}, together with
/// the wheel angles, direction signs and the twist scale above / below) and the
/// PEAR wheel ids were assigned to match the holodeck's, so it reproduces the
/// holodeck's driving behaviour as long as node 2/1/3/4 sit at the same
/// corners on PEAR (FR/FL/BR/BL). Read against kWheelNodeIds it says
/// TL -> FR(2), TR -> BL(4), BL -> BR(3), BR -> FL(1), which is NOT the
/// platform's own geometric naming (that would be TL -> FL(1), TR -> FR(2),
/// BL -> BL(4), BR -> BR(3) with +y = left): the holodeck never documented
/// which physical corner each id is at, and its weak rotation authority (the
/// 20x twist scale) hints the ids and angles compensate each other.
/// TODO(hardware): on the kit, `enable` then `speed 0.2 0 0` must roll the base
/// straight forward, `speed 0 0.2 0` straight right, `speed 0 0 3` turn
/// counter-clockwise; if not, fix this table (or kWheelDirections) first.
inline constexpr std::array<size_t, 4> kWheelNodeIdsByPlatformWheel = {2, 4, 3, 1};

inline constexpr HoloDeckPlatform::Configuration kPlatformConfiguration = {
    .wheel_diameter_m = kWheelDiameterM,
    .half_length_m = kHalfLengthM,
    .half_width_m = kHalfWidthM,
    .wheel_angles_degrees = kWheelAnglesDegrees,
    .motor_directions = kWheelDirections,
};

/////////////////////////////////////////////////////////////////////////////
// Hub-wheel drives (AMPS L2DB4830-CAFC)
/////////////////////////////////////////////////////////////////////////////

/// Acceleration / deceleration ramps (rev/s^2 at the output shaft). These live
/// in the drive (0x6083 / 0x6084): a step in the streamed set-point is ramped
/// by the drive itself, so the 50 Hz loop sends raw targets. 2 rev/s^2 is a
/// gentle start for a loaded base.
/// TODO(hardware): tune on the base.
inline constexpr float kWheelAccelerationRps2 = 2.0f;
inline constexpr float kWheelDecelerationRps2 = 2.0f;
/// The drive's own speed ceiling 0x6080 (rpm); 0 = leave the drive's setting.
inline constexpr uint16_t kWheelMaxSpeedRpm = 0;

/// The wheel driver configuration: the Ds402VelocityMotor defaults ARE the
/// L2DB4830 with its default 4096 feedback resolution (velocity DEC per rpm,
/// position counts per rev, actual speed 0x60F9:19, current 0x6078,
/// temperature 0x60F7:0B, error code 0x2601); only the ramps / ceiling are
/// this project's.
inline constexpr Ds402VelocityMotor::Config kWheelDriveConfig = {
    .acceleration_rps2 = kWheelAccelerationRps2,
    .deceleration_rps2 = kWheelDecelerationRps2,
    .max_speed_rpm = kWheelMaxSpeedRpm,
    .log_level = espp::Logger::Verbosity::INFO,
};

/////////////////////////////////////////////////////////////////////////////
// Corner linkages (Eyou motors, profile position)
/////////////////////////////////////////////////////////////////////////////

/// The 4-bar geometry (see linkage_geometry.hpp for the model). Angles are at
/// the motor output shaft relative to the software zero stored in NVS
/// (`zero` / `save_zero` on the CLI with the linkage at its level position),
/// positive = module up.
/// TODO(hardware): crank length, the level angle and the mechanical range are
/// placeholders. The seat-axis table rows L1..L4 (rammp-rtps, -30..60 deg)
/// must cover [min, max].
inline constexpr float kLinkageCrankLengthM = 0.15f;
inline constexpr float kLinkageLevelAngleDeg = 0.0f;
inline constexpr float kLinkageAngleMinDeg = -30.0f;
inline constexpr float kLinkageAngleMaxDeg = 60.0f;

inline constexpr pear::geometry::Geometry kLinkageGeometry = {
    .half_length_m = kHalfLengthM,
    .half_width_m = kHalfWidthM,
    .crank_length_m = kLinkageCrankLengthM,
    .level_angle_deg = kLinkageLevelAngleDeg,
    .min_angle_deg = kLinkageAngleMinDeg,
    .max_angle_deg = kLinkageAngleMaxDeg,
};

/// Profile-position motion profile (STEM_V3's values as a start).
/// TODO(hardware): tune for the loaded base.
inline constexpr EyouMotor::ProfilePositionConfig kLinkageProfile = {
    .velocity_degrees_per_second = 60.0f,
    .acceleration_degrees_per_second_squared = 20.0f,
    .deceleration_degrees_per_second_squared = 20.0f,
};

/// A linkage is "at target" within this; for the CLI / status display only.
inline constexpr float kLinkageSettleTolDeg = 1.0f;
/// Linkage status poll period (round-robin, one motor per tick).
inline constexpr std::chrono::milliseconds kLinkageStatusPollPeriod{250};
/// Per-read reply timeout for a linkage status poll.
inline constexpr uint32_t kLinkageStatusReadTimeoutMs = 50;
/// A linkage's status is STALE when it has not been read for this long.
inline constexpr std::chrono::milliseconds kLinkageStatusStaleTimeout{3000};

/////////////////////////////////////////////////////////////////////////////
// Drive control parameters
/////////////////////////////////////////////////////////////////////////////

/// Fixed-rate control loop period (wheel-speed computation + CAN commands).
inline constexpr std::chrono::milliseconds kControlPeriod{20}; // 50 Hz
/// Wheel status polling period (round-robin, one motor per tick).
inline constexpr std::chrono::milliseconds kStatusPollPeriod{250}; // 4 Hz: each motor at 1 Hz
/// Per-status-read reply timeout; bounds how long a poll holds a drive's
/// transaction mutex (the control loop waits on it for that motor).
inline constexpr uint32_t kMotorStatusReadTimeoutMs = 20;
/// After the joystick has been centered for this long, control falls back to
/// the local (CLI) setpoint, which is zeroed on the handover.
inline constexpr std::chrono::milliseconds kJoystickReleaseTimeout{500};
/// A wheel's status is reported STALE when it has not been read for this long.
inline constexpr std::chrono::milliseconds kMotorStatusStaleTimeout{3000};
/// After a failed (re-)enable of the wheel drives from DRIVE, wait this long
/// before the control loop tries again (each attempt can block for the SDO
/// timeouts of a missing drive).
inline constexpr std::chrono::milliseconds kWheelRearmRetryPeriod{1000};

/// Hard per-wheel OUTPUT-shaft speed limit (RPM): if any computed wheel speed
/// exceeds this, all wheels are scaled down together (preserving the motion
/// direction). The holodeck's value.
inline constexpr float kMaxWheelRpm = 90.0f;

/// Translation-speed and chassis-rotation limits per HMI drive profile
/// (MIB::DriveProfile LOW / NORMAL / HIGH); the joystick's full deflection
/// maps to these.
struct ProfileLimits {
  float max_speed_mps;
  float max_rotation_rpm;
};
inline constexpr ProfileLimits kProfileLow{0.5f, 3.0f};
inline constexpr ProfileLimits kProfileNormal{1.0f, 6.0f};
inline constexpr ProfileLimits kProfileHigh{1.5f, 9.0f};

/// Scale applied to the rotation rate commanded from the joystick TWIST axis
/// (twist * scale * max-rotation). Tuned on the holodeck (see
/// kWheelNodeIdsByPlatformWheel); wheel commands stay bounded by kMaxWheelRpm
/// regardless.
inline constexpr float kTwistRotationScale = 20.0f;

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
/// How often each motor's rammp::MotorState is published (the motor layer's
/// idle rate is >= 10 Hz).
inline constexpr std::chrono::milliseconds kMotorStatePublishPeriod{100};

} // namespace hw_config
