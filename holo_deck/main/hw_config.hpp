#pragma once

#include <chrono>

#include <driver/gpio.h>
#include <hal/adc_types.h>

/// \file hw_config.hpp
/// \brief All external wiring and tuning constants for the holo_deck firmware
///        on the M5Stack Tab5.
///
/// Pin availability on the Tab5 (see espp::M5StackTab5 in the m5stack-tab5
/// BSP header for the full list of pins the board already commits):
///
/// * ESP32-P4 ADC1 covers GPIO16-23 only. Of those, the Tab5 already uses
///   GPIO22 (LCD backlight) and GPIO23 (touch interrupt), and GPIO20/21 are
///   hard-wired to the on-board RS-485 transceiver, so they are NOT usable as
///   analog inputs. That leaves GPIO16, GPIO17 (M5-Bus pins 2/4) and GPIO18,
///   GPIO19 (M5-Bus MOSI/MISO) as the only ADC1-capable header pins.
/// * ESP32-P4 ADC2 covers GPIO49-54. Of those, GPIO52 (M5-Bus pin 10) and
///   the Grove port pins GPIO53/GPIO54 are broken out on the Tab5.
/// * The joystick therefore spans BOTH ADC units (X/Y on ADC1, twist on
///   ADC2), which requires two OneshotAdc instances (verified against
///   soc/esp32p4/include/soc/adc_channel.h).
/// * The CAN transceiver hangs off the M5-Bus "PC_TX"/"PC_RX" UART pins
///   (GPIO6/GPIO7): they are plain digital header pins that nothing on the
///   board drives, and the TWAI controller can route to any GPIO via the
///   GPIO matrix. This keeps the Grove port completely free.
namespace hw_config {

/////////////////////////////////////////////////////////////////////////////
// CAN bus (RMD-X6-S2 motors, 1 Mbit/s) - M5-Bus header
/////////////////////////////////////////////////////////////////////////////

/// TWAI TX -> CAN transceiver TXD. M5-Bus "PC_TX" pin (GPIO6). PC_TX is the
/// Tab5's transmit-role pin on that header pair, so the CAN TX (an output)
/// goes here.
/// NOTE: the BSP header also names GPIO6 `camera_reset_io`, but the camera
/// reset is actually driven through the PI4IOE5V6408 IO expander (P6 of
/// 0x43), not the physical GPIO - and this firmware never initializes the
/// camera - so GPIO6 is free.
inline constexpr gpio_num_t kCanTxGpio = GPIO_NUM_6;
/// TWAI RX <- CAN transceiver RXD. M5-Bus "PC_RX" pin (GPIO7, the
/// receive-role pin of the pair).
inline constexpr gpio_num_t kCanRxGpio = GPIO_NUM_7;
/// The M5-Bus header also provides 5 V, 3V3 and GND for the transceiver
/// module. Use a transceiver with 3.3 V logic levels on TXD/RXD (e.g.
/// SN65HVD230, or a TJA1050 module with level-safe RXD).

/////////////////////////////////////////////////////////////////////////////
// 3-axis analog joystick + pushbutton - M5-Bus header
// (power the joystick from the M5-Bus 3V3 pin, NOT 5V!)
/////////////////////////////////////////////////////////////////////////////

/// The joystick potentiometers MUST be supplied from 3.3 V so the wiper
/// voltage stays inside the ADC range. 12 dB attenuation measures roughly
/// 0-3.3 V, matching a 3.3 V-supplied pot.
inline constexpr adc_atten_t kJoystickAdcAttenuation = ADC_ATTEN_DB_12;

/// X/Y are on ADC1; the twist axis is on ADC2 (two separate ADC units, and
/// therefore two OneshotAdc instances - see the file comment).
inline constexpr adc_unit_t kJoystickXYAdcUnit = ADC_UNIT_1;
inline constexpr adc_unit_t kJoystickTwistAdcUnit = ADC_UNIT_2;

/// Joystick X (left/right) wiper -> GPIO16 (ADC1_CH0, M5-Bus pin 2 "G16").
inline constexpr adc_channel_t kJoystickXAdcChannel = ADC_CHANNEL_0;
/// Joystick Y (forward/back) wiper -> GPIO17 (ADC1_CH1, M5-Bus pin 4 "G17").
inline constexpr adc_channel_t kJoystickYAdcChannel = ADC_CHANNEL_1;
/// Twist / rotation axis wiper -> GPIO52 (ADC2_CH3, M5-Bus pin 10 "G52").
inline constexpr adc_channel_t kJoystickTwistAdcChannel = ADC_CHANNEL_3;

/// Joystick pushbutton -> GPIO48 (M5-Bus pin 22 "G48"), other side to GND.
/// The internal pull-up is enabled and the button is assumed ACTIVE-LOW
/// (pressed = the pin reads 0). Pressing it toggles between ENABLED and
/// E-STOPPED - the same shared state as the GUI ENABLE/STOP button and the
/// CLI enable / estop commands.
inline constexpr gpio_num_t kJoystickButtonGpio = GPIO_NUM_48;
/// Level read on kJoystickButtonGpio while the button is pressed.
inline constexpr int kJoystickButtonActiveLevel = 0;
/// The button level must be stable for this long before a press/release is
/// accepted (debounce).
inline constexpr std::chrono::milliseconds kJoystickButtonDebounce{50};

/// Per-axis calibration, in millivolts as returned by the calibrated ADC
/// read. Defaults assume a 3.3 V-supplied pot centered at half scale; adjust
/// after measuring your joystick (the `status` CLI command prints the live
/// normalized values, and raw mV is logged at DEBUG level).
inline constexpr float kJoystickXCenterMv = 1650.0f;
inline constexpr float kJoystickXMinMv = 150.0f;
inline constexpr float kJoystickXMaxMv = 3150.0f;
/// Set true if pushing the stick right decreases the measured voltage.
inline constexpr bool kJoystickXInverted = false;

inline constexpr float kJoystickYCenterMv = 1650.0f;
inline constexpr float kJoystickYMinMv = 150.0f;
inline constexpr float kJoystickYMaxMv = 3150.0f;
/// Set true if pushing the stick forward decreases the measured voltage.
inline constexpr bool kJoystickYInverted = false;

inline constexpr float kJoystickTwistCenterMv = 1650.0f;
inline constexpr float kJoystickTwistMinMv = 150.0f;
inline constexpr float kJoystickTwistMaxMv = 3150.0f;
/// Set true if twisting counter-clockwise decreases the measured voltage.
inline constexpr bool kJoystickTwistInverted = false;

/// Circular deadzone radius around center for the X/Y pair, as a fraction of
/// the unit circle. Inside this radius the translation command is exactly 0
/// (this is also what releases joystick control back to the GUI).
inline constexpr float kJoystickCenterDeadzoneRadius = 0.12f;
/// Deadzone at the rim of the unit circle: deflections beyond
/// (1 - kJoystickRangeDeadzone) count as full deflection.
inline constexpr float kJoystickRangeDeadzone = 0.08f;
/// Deadband (in mV) around the twist axis center inside which rotation is 0.
inline constexpr float kJoystickTwistDeadbandMv = 200.0f;

/////////////////////////////////////////////////////////////////////////////
// Control parameters
/////////////////////////////////////////////////////////////////////////////

/// Fixed-rate control loop period (wheel-speed computation + CAN commands).
inline constexpr std::chrono::milliseconds kControlPeriod{20}; // 50 Hz
/// Physical joystick sampling period.
inline constexpr std::chrono::milliseconds kJoystickPeriod{20}; // 50 Hz
/// Motor status polling period (round-robin, one motor per tick).
inline constexpr std::chrono::milliseconds kStatusPollPeriod{500}; // 2 Hz
/// After the physical joystick has been centered for this long, control
/// falls back to the GUI/CLI setpoint (which is zeroed on the handover).
inline constexpr std::chrono::milliseconds kJoystickReleaseTimeout{500};
/// A motor's status is shown as STALE when it has not been successfully read
/// for this long.
inline constexpr std::chrono::milliseconds kMotorStatusStaleTimeout{3000};

/// Hard per-wheel speed limit: if any computed wheel speed exceeds this, all
/// wheels are scaled down together (preserving the motion direction).
inline constexpr float kMaxWheelRpm = 30.0f;
/// Default (and maximum-selectable) translation speed limits, m/s.
inline constexpr float kDefaultMaxSpeedMps = 0.25f;
inline constexpr float kMinSelectableMaxSpeedMps = 0.05f;
inline constexpr float kMaxSelectableMaxSpeedMps = 0.50f;
/// Default (and maximum-selectable) chassis rotation rate limits, RPM.
inline constexpr float kDefaultMaxRotationRpm = 3.0f;
inline constexpr float kMinSelectableMaxRotationRpm = 0.5f;
inline constexpr float kMaxSelectableMaxRotationRpm = 6.0f;

} // namespace hw_config
