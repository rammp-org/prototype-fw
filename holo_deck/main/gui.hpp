#pragma once

#include <array>
#include <functional>
#include <mutex>
#include <string_view>

#include "m5stack-tab5.hpp"

#include "logger.hpp"
#include "task.hpp"

#include "holo_deck_controller.hpp"

/// The Gui class encapsulates all of the LVGL UI for the holo_deck firmware,
/// following the espp + LVGL C++ pattern (see the m5stack-tab5 BSP example):
///
/// * All LVGL objects are created in init_ui(), which is broken into small
///   member functions - one per logical piece of the UI.
/// * The class owns the task which calls lv_task_handler(), and a recursive
///   mutex which guards every LVGL call. Public methods lock that mutex, so
///   other tasks can safely call them.
/// * LVGL event callbacks are registered with `this` as the user-data and
///   dispatched through a single static trampoline (event_callback) into
///   member functions.
///
/// The single 1280x720 landscape screen shows:
/// * a top bar with the active control source (GUI / JOYSTICK / STOPPED) and
///   a large ENABLE / STOP (e-stop) button,
/// * the commanded velocity as a vector on a circle plus numeric
///   vx / vy / |v| / heading / rotation-rate readouts,
/// * four motor tiles (id, measured RPM, commanded RPM, temperature,
///   OK/STALE),
/// * a circular touch drag-pad (virtual joystick) for translation, a
///   rotation slider, and sliders for the max-speed / max-rotation limits.
///
/// The drag-pad and rotation slider only act when the GUI is the active
/// source; when the physical joystick has taken over they become read-only
/// indicators showing the live commanded values.
class Gui {
public:
  /// Callback for the translation drag-pad: normalized platform-frame values
  /// (forward, left), each in [-1, 1].
  using translation_callback_t = std::function<void(float forward, float left)>;
  /// Callback for the rotation slider: normalized counter-clockwise rate in
  /// [-1, 1].
  using rotation_callback_t = std::function<void(float ccw)>;
  /// Callback for the ENABLE / STOP button: true to enable, false to e-stop.
  using enable_callback_t = std::function<void(bool enable)>;
  /// Callback for the limit sliders, in real units (m/s or RPM).
  using limit_callback_t = std::function<void(float value)>;

  /// Configuration for the Gui
  struct Config {
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Log verbosity
  };

  /// Construct the Gui: builds the UI and starts the LVGL update task.
  /// @param config The configuration for the Gui
  explicit Gui(const Config &config)
      : logger_({.tag = "Gui", .level = config.log_level}) {
    init_ui();
    update_task_.start();
  }

  ~Gui() {
    update_task_.stop();
    deinit_ui();
  }

  /// Set the callback fired while the translation drag-pad is used.
  void set_translation_callback(translation_callback_t callback) {
    translation_callback_ = std::move(callback);
  }

  /// Set the callback fired when the rotation slider is moved.
  void set_rotation_callback(rotation_callback_t callback) {
    rotation_callback_ = std::move(callback);
  }

  /// Set the callback fired when the ENABLE / STOP button is pressed.
  void set_enable_callback(enable_callback_t callback) { enable_callback_ = std::move(callback); }

  /// Set the callback fired when the max-speed slider is moved.
  void set_max_speed_callback(limit_callback_t callback) {
    max_speed_callback_ = std::move(callback);
  }

  /// Set the callback fired when the max-rotation slider is moved.
  void set_max_rotation_callback(limit_callback_t callback) {
    max_rotation_callback_ = std::move(callback);
  }

  /// Refresh every widget from a controller state snapshot. Thread-safe;
  /// call it periodically (e.g. 10 Hz) from any task.
  void update_state(const HoloDeckController::State &state);

protected:
  // geometry (1280x720 landscape)
  static constexpr int TOP_BAR_HEIGHT = 90;
  static constexpr int VIZ_SIZE = 300;
  static constexpr int PAD_SIZE = 340;
  static constexpr int KNOB_SIZE = 60;
  static constexpr int SLIDER_WIDTH = 460;

  void init_ui();
  void deinit_ui();

  // the individual pieces of the UI, called from init_ui()
  void init_top_bar();
  void init_vector_display();
  void init_motor_tiles();
  void init_drag_pad();
  void init_sliders();

  // the LVGL update task: calls lv_task_handler() under the mutex
  bool update(std::mutex &m, std::condition_variable &cv);

  // single trampoline for all LVGL events; dispatches to the member
  // functions below based on the event target / code
  static void event_callback(lv_event_t *e);
  void on_pressed(lv_event_t *e);
  void on_pad_pressing(lv_event_t *e);
  void on_pad_released(lv_event_t *e);
  void on_value_changed(lv_event_t *e);

  // helpers, called with the mutex held
  void set_knob_position(float forward, float left);
  void set_vector_line(float forward_norm, float left_norm);
  void show_source(HoloDeckController::Source source);

  // LVGL objects
  lv_obj_t *source_label_{nullptr};
  lv_obj_t *enable_button_{nullptr};
  lv_obj_t *enable_button_label_{nullptr};
  lv_obj_t *vector_circle_{nullptr};
  lv_obj_t *vector_line_{nullptr};
  lv_obj_t *velocity_label_{nullptr};
  lv_obj_t *limits_label_{nullptr};
  std::array<lv_obj_t *, 4> motor_tiles_{};
  std::array<lv_obj_t *, 4> motor_tile_labels_{};
  lv_obj_t *pad_{nullptr};
  lv_obj_t *pad_knob_{nullptr};
  lv_obj_t *pad_label_{nullptr};
  lv_obj_t *rotation_slider_{nullptr};
  lv_obj_t *rotation_label_{nullptr};
  lv_obj_t *max_speed_slider_{nullptr};
  lv_obj_t *max_speed_label_{nullptr};
  lv_obj_t *max_rotation_slider_{nullptr};
  lv_obj_t *max_rotation_label_{nullptr};

  lv_style_t vector_line_style_;
  lv_point_precise_t vector_line_points_[2];

  translation_callback_t translation_callback_{nullptr};
  rotation_callback_t rotation_callback_{nullptr};
  enable_callback_t enable_callback_{nullptr};
  limit_callback_t max_speed_callback_{nullptr};
  limit_callback_t max_rotation_callback_{nullptr};

  // last state received via update_state(), used by the event handlers (to
  // only act when the GUI is the active source) and to detect transitions
  bool gui_source_active_{false};
  bool last_enabled_{false};
  HoloDeckController::Source last_source_{HoloDeckController::Source::STOPPED};
  float last_max_speed_{0.0f};
  float last_max_rotation_{0.0f};

  espp::Task update_task_{
      {.callback = [this](auto &m, auto &cv) { return update(m, cv); },
       .task_config = {
           .name = "gui", .stack_size_bytes = 12 * 1024, .priority = 20, .core_id = 1}}};
  espp::Logger logger_;
  std::recursive_mutex mutex_;
  bool ui_ready_{false};
};
