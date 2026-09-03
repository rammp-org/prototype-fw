#include <algorithm>
#include <cmath>

#include "gui.hpp"

#include "format.hpp"

#include "hw_config.hpp"

namespace {
constexpr float kRadiansToDegrees = 180.0f / 3.14159265358979323846f;
// slider integer scaling: the LVGL sliders carry integers, the limits are
// floats
constexpr float kSpeedSliderScale = 0.01f;   // slider unit -> m/s
constexpr float kRotationSliderScale = 0.1f; // slider unit -> RPM
constexpr int kRotationCommandRange = 100;   // rotation slider is +/- this
} // namespace

void Gui::init_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  init_top_bar();
  init_vector_display();
  init_motor_tiles();
  init_drag_pad();
  init_sliders();
  ui_ready_ = true;
}

void Gui::deinit_ui() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  ui_ready_ = false;
  lv_obj_clean(lv_screen_active());
  // lv_obj_clean() deleted every object under the screen; null the pointers
  // we hold so nothing dereferences a freed object
  source_label_ = nullptr;
  enable_button_ = nullptr;
  enable_button_label_ = nullptr;
  vector_circle_ = nullptr;
  vector_line_ = nullptr;
  velocity_label_ = nullptr;
  limits_label_ = nullptr;
  motor_tiles_ = {};
  motor_tile_labels_ = {};
  pad_ = nullptr;
  pad_knob_ = nullptr;
  pad_label_ = nullptr;
  rotation_slider_ = nullptr;
  rotation_label_ = nullptr;
  max_speed_slider_ = nullptr;
  max_speed_label_ = nullptr;
  max_rotation_slider_ = nullptr;
  max_rotation_label_ = nullptr;
}

void Gui::init_top_bar() {
  lv_obj_t *screen = lv_screen_active();

  lv_obj_t *title = lv_label_create(screen);
  lv_label_set_text(title, "Holo Deck");
  lv_obj_align(title, LV_ALIGN_TOP_LEFT, 20, 30);

  // the active-source indicator is the most prominent element in the bar; it
  // is recolored by show_source()
  source_label_ = lv_label_create(screen);
  lv_label_set_text(source_label_, "STOPPED");
  lv_obj_set_style_bg_opa(source_label_, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(source_label_, lv_palette_main(LV_PALETTE_RED), 0);
  lv_obj_set_style_text_color(source_label_, lv_color_white(), 0);
  lv_obj_set_style_pad_all(source_label_, 16, 0);
  lv_obj_set_style_radius(source_label_, 8, 0);
  lv_obj_align(source_label_, LV_ALIGN_TOP_MID, 0, 18);

  enable_button_ = lv_btn_create(screen);
  lv_obj_set_size(enable_button_, 220, 70);
  lv_obj_align(enable_button_, LV_ALIGN_TOP_RIGHT, -20, 10);
  lv_obj_set_style_bg_color(enable_button_, lv_palette_main(LV_PALETTE_GREEN), 0);
  enable_button_label_ = lv_label_create(enable_button_);
  lv_label_set_text(enable_button_label_, "ENABLE");
  lv_obj_center(enable_button_label_);
  lv_obj_add_event_cb(enable_button_, event_callback, LV_EVENT_PRESSED, this);
}

void Gui::init_vector_display() {
  lv_obj_t *screen = lv_screen_active();

  // circle showing the commanded velocity vector (direction + magnitude,
  // full radius = the max-speed limit)
  vector_circle_ = lv_obj_create(screen);
  lv_obj_set_size(vector_circle_, VIZ_SIZE, VIZ_SIZE);
  lv_obj_align(vector_circle_, LV_ALIGN_TOP_LEFT, 30, TOP_BAR_HEIGHT + 20);
  lv_obj_set_style_radius(vector_circle_, LV_RADIUS_CIRCLE, 0);
  lv_obj_clear_flag(vector_circle_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(vector_circle_, LV_OBJ_FLAG_CLICKABLE);

  lv_obj_t *center_dot = lv_obj_create(vector_circle_);
  lv_obj_set_size(center_dot, 10, 10);
  lv_obj_set_style_radius(center_dot, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(center_dot, lv_palette_main(LV_PALETTE_GREY), 0);
  lv_obj_center(center_dot);

  lv_style_init(&vector_line_style_);
  lv_style_set_line_width(&vector_line_style_, 8);
  lv_style_set_line_color(&vector_line_style_, lv_palette_main(LV_PALETTE_BLUE));
  lv_style_set_line_rounded(&vector_line_style_, true);

  vector_line_ = lv_line_create(vector_circle_);
  vector_line_points_[0] = {0, 0};
  vector_line_points_[1] = {0, 0};
  lv_line_set_points(vector_line_, vector_line_points_, 2);
  lv_obj_add_style(vector_line_, &vector_line_style_, 0);
  set_vector_line(0.0f, 0.0f);

  velocity_label_ = lv_label_create(screen);
  lv_label_set_text(velocity_label_, "");
  lv_obj_align(velocity_label_, LV_ALIGN_TOP_LEFT, VIZ_SIZE + 60, TOP_BAR_HEIGHT + 40);

  limits_label_ = lv_label_create(screen);
  lv_label_set_text(limits_label_, "");
  lv_obj_align(limits_label_, LV_ALIGN_TOP_LEFT, VIZ_SIZE + 60, TOP_BAR_HEIGHT + 200);
}

void Gui::init_motor_tiles() {
  lv_obj_t *screen = lv_screen_active();
  static constexpr int TILE_WIDTH = 285;
  static constexpr int TILE_HEIGHT = 120;
  static constexpr int TILE_X = 30;
  static constexpr int TILE_Y = 450;
  for (size_t i = 0; i < motor_tiles_.size(); ++i) {
    lv_obj_t *tile = lv_obj_create(screen);
    lv_obj_set_size(tile, TILE_WIDTH, TILE_HEIGHT);
    lv_obj_align(tile, LV_ALIGN_TOP_LEFT, TILE_X + static_cast<int>(i % 2) * (TILE_WIDTH + 15),
                 TILE_Y + static_cast<int>(i / 2) * (TILE_HEIGHT + 15));
    lv_obj_clear_flag(tile, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(tile, 3, 0);
    lv_obj_set_style_border_color(tile, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_t *label = lv_label_create(tile);
    lv_label_set_text_fmt(label, "M%d\nwaiting...", static_cast<int>(i) + 1);
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 0);
    motor_tiles_[i] = tile;
    motor_tile_labels_[i] = label;
  }
}

void Gui::init_drag_pad() {
  lv_obj_t *screen = lv_screen_active();

  pad_ = lv_obj_create(screen);
  lv_obj_set_size(pad_, PAD_SIZE, PAD_SIZE);
  lv_obj_align(pad_, LV_ALIGN_TOP_RIGHT, -180, TOP_BAR_HEIGHT + 20);
  lv_obj_set_style_radius(pad_, LV_RADIUS_CIRCLE, 0);
  lv_obj_clear_flag(pad_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(pad_, event_callback, LV_EVENT_PRESSING, this);
  lv_obj_add_event_cb(pad_, event_callback, LV_EVENT_RELEASED, this);
  lv_obj_add_event_cb(pad_, event_callback, LV_EVENT_PRESS_LOST, this);

  pad_knob_ = lv_obj_create(pad_);
  lv_obj_set_size(pad_knob_, KNOB_SIZE, KNOB_SIZE);
  lv_obj_set_style_radius(pad_knob_, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(pad_knob_, lv_palette_main(LV_PALETTE_BLUE), 0);
  // the knob is purely an indicator: never let it swallow the pad's touches
  lv_obj_clear_flag(pad_knob_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_center(pad_knob_);

  pad_label_ = lv_label_create(screen);
  lv_label_set_text(pad_label_, "Drag to translate (up = forward)");
  lv_obj_align_to(pad_label_, pad_, LV_ALIGN_OUT_BOTTOM_MID, 0, 8);
}

void Gui::init_sliders() {
  lv_obj_t *screen = lv_screen_active();
  static constexpr int SLIDER_X = -60;
  static constexpr int SLIDER_Y = TOP_BAR_HEIGHT + PAD_SIZE + 80;
  static constexpr int SLIDER_SPACING = 75;

  struct SliderSpec {
    lv_obj_t **slider;
    lv_obj_t **label;
    int min;
    int max;
    int value;
    int y_offset;
  };
  const SliderSpec sliders[] = {
      {&rotation_slider_, &rotation_label_, -kRotationCommandRange, kRotationCommandRange, 0, 0},
      {&max_speed_slider_, &max_speed_label_,
       static_cast<int>(hw_config::kMinSelectableMaxSpeedMps / kSpeedSliderScale),
       static_cast<int>(hw_config::kMaxSelectableMaxSpeedMps / kSpeedSliderScale),
       static_cast<int>(hw_config::kDefaultMaxSpeedMps / kSpeedSliderScale), SLIDER_SPACING},
      {&max_rotation_slider_, &max_rotation_label_,
       static_cast<int>(hw_config::kMinSelectableMaxRotationRpm / kRotationSliderScale),
       static_cast<int>(hw_config::kMaxSelectableMaxRotationRpm / kRotationSliderScale),
       static_cast<int>(hw_config::kDefaultMaxRotationRpm / kRotationSliderScale),
       2 * SLIDER_SPACING},
  };
  for (const auto &spec : sliders) {
    lv_obj_t *label = lv_label_create(screen);
    lv_obj_align(label, LV_ALIGN_TOP_RIGHT, SLIDER_X - (SLIDER_WIDTH - 440),
                 SLIDER_Y + spec.y_offset);
    lv_obj_t *slider = lv_slider_create(screen);
    lv_obj_set_width(slider, SLIDER_WIDTH);
    lv_slider_set_range(slider, spec.min, spec.max);
    lv_slider_set_value(slider, spec.value, LV_ANIM_OFF);
    lv_obj_align(slider, LV_ALIGN_TOP_RIGHT, SLIDER_X, SLIDER_Y + spec.y_offset + 28);
    lv_obj_add_event_cb(slider, event_callback, LV_EVENT_VALUE_CHANGED, this);
    *spec.slider = slider;
    *spec.label = label;
  }
  lv_label_set_text(rotation_label_, "Rotation (left = CCW)");
  lv_label_set_text(max_speed_label_, "Max speed");
  lv_label_set_text(max_rotation_label_, "Max rotation");
}

bool Gui::update(std::mutex &m, std::condition_variable &cv) {
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    lv_task_handler();
  }
  std::unique_lock<std::mutex> lock(m);
  cv.wait_for(lock, std::chrono::milliseconds(16));
  return false; // don't stop the task
}

void Gui::event_callback(lv_event_t *e) {
  auto *gui = static_cast<Gui *>(lv_event_get_user_data(e));
  if (!gui) {
    return;
  }
  switch (lv_event_get_code(e)) {
  case LV_EVENT_PRESSED:
    gui->on_pressed(e);
    break;
  case LV_EVENT_PRESSING:
    gui->on_pad_pressing(e);
    break;
  case LV_EVENT_RELEASED:
  case LV_EVENT_PRESS_LOST:
    gui->on_pad_released(e);
    break;
  case LV_EVENT_VALUE_CHANGED:
    gui->on_value_changed(e);
    break;
  default:
    break;
  }
}

void Gui::on_pressed(lv_event_t *e) {
  auto *target = static_cast<lv_obj_t *>(lv_event_get_target(e));
  if (target == enable_button_ && enable_callback_) {
    // request the opposite of the last known state: STOP when enabled
    // (e-stop always wins, from any source), ENABLE when stopped
    enable_callback_(!last_enabled_);
  }
}

void Gui::on_pad_pressing(lv_event_t *e) {
  auto *target = static_cast<lv_obj_t *>(lv_event_get_target(e));
  if (target != pad_) {
    return;
  }
  // the drag-pad only acts when the GUI is the active source; otherwise it
  // is a read-only indicator driven by update_state()
  if (!gui_source_active_) {
    return;
  }
  lv_indev_t *indev = lv_indev_get_act();
  if (!indev) {
    return;
  }
  lv_point_t point;
  lv_indev_get_point(indev, &point);
  lv_area_t coords;
  lv_obj_get_coords(pad_, &coords);
  const float center_x = (coords.x1 + coords.x2) / 2.0f;
  const float center_y = (coords.y1 + coords.y2) / 2.0f;
  const float radius = (PAD_SIZE - KNOB_SIZE) / 2.0f;
  float dx = (point.x - center_x) / radius;
  float dy = (point.y - center_y) / radius;
  const float magnitude = std::sqrt(dx * dx + dy * dy);
  if (magnitude > 1.0f) {
    dx /= magnitude;
    dy /= magnitude;
  }
  // screen up (-y) = forward (+x platform), screen left (-x) = left (+y)
  const float forward = -dy;
  const float left = -dx;
  set_knob_position(forward, left);
  if (translation_callback_) {
    translation_callback_(forward, left);
  }
}

void Gui::on_pad_released(lv_event_t *e) {
  auto *target = static_cast<lv_obj_t *>(lv_event_get_target(e));
  if (target != pad_) {
    return;
  }
  if (!gui_source_active_) {
    return;
  }
  // the pad is a spring-return control: release = stop translating
  set_knob_position(0.0f, 0.0f);
  if (translation_callback_) {
    translation_callback_(0.0f, 0.0f);
  }
}

void Gui::on_value_changed(lv_event_t *e) {
  auto *target = static_cast<lv_obj_t *>(lv_event_get_target(e));
  if (target == rotation_slider_) {
    // the rotation command only acts when the GUI is the active source
    if (!gui_source_active_) {
      return;
    }
    if (rotation_callback_) {
      // slider right = turn right (clockwise) = negative CCW rate
      const float ccw =
          -static_cast<float>(lv_slider_get_value(rotation_slider_)) / kRotationCommandRange;
      rotation_callback_(ccw);
    }
  } else if (target == max_speed_slider_) {
    if (max_speed_callback_) {
      max_speed_callback_(lv_slider_get_value(max_speed_slider_) * kSpeedSliderScale);
    }
  } else if (target == max_rotation_slider_) {
    if (max_rotation_callback_) {
      max_rotation_callback_(lv_slider_get_value(max_rotation_slider_) * kRotationSliderScale);
    }
  }
}

void Gui::set_knob_position(float forward, float left) {
  const float radius = (PAD_SIZE - KNOB_SIZE) / 2.0f;
  const int x = static_cast<int>(-left * radius);
  const int y = static_cast<int>(-forward * radius);
  lv_obj_align(pad_knob_, LV_ALIGN_CENTER, x, y);
}

void Gui::set_vector_line(float forward_norm, float left_norm) {
  const float radius = (VIZ_SIZE - 40) / 2.0f;
  const float center = VIZ_SIZE / 2.0f - 15.0f; // inside the padded circle
  // lv_value_precise_t may be integral (LV_USE_FLOAT=0), so round explicitly
  const auto precise = [](float value) {
    return static_cast<lv_value_precise_t>(std::lround(value));
  };
  vector_line_points_[0] = {precise(center), precise(center)};
  vector_line_points_[1] = {precise(center - left_norm * radius),
                            precise(center - forward_norm * radius)};
  lv_line_set_points(vector_line_, vector_line_points_, 2);
}

void Gui::show_source(HoloDeckController::Source source) {
  switch (source) {
  case HoloDeckController::Source::GUI:
    lv_label_set_text(source_label_, "SOURCE: GUI");
    lv_obj_set_style_bg_color(source_label_, lv_palette_main(LV_PALETTE_BLUE), 0);
    break;
  case HoloDeckController::Source::JOYSTICK:
    lv_label_set_text(source_label_, "SOURCE: JOYSTICK");
    lv_obj_set_style_bg_color(source_label_, lv_palette_main(LV_PALETTE_ORANGE), 0);
    break;
  default:
    lv_label_set_text(source_label_, "STOPPED");
    lv_obj_set_style_bg_color(source_label_, lv_palette_main(LV_PALETTE_RED), 0);
    break;
  }
  lv_obj_align(source_label_, LV_ALIGN_TOP_MID, 0, 18);
}

void Gui::update_state(const HoloDeckController::State &state) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!ui_ready_) {
    return;
  }

  const bool gui_active = state.source == HoloDeckController::Source::GUI;
  const bool source_changed = state.source != last_source_;
  gui_source_active_ = gui_active;
  last_enabled_ = state.enabled;

  show_source(state.source);

  // ENABLE / STOP button: pressing it e-stops when enabled, enables when
  // stopped
  if (state.enabled) {
    lv_label_set_text(enable_button_label_, LV_SYMBOL_STOP " STOP");
    lv_obj_set_style_bg_color(enable_button_, lv_palette_main(LV_PALETTE_RED), 0);
  } else {
    lv_label_set_text(enable_button_label_, LV_SYMBOL_PLAY " ENABLE");
    lv_obj_set_style_bg_color(enable_button_, lv_palette_main(LV_PALETTE_GREEN), 0);
  }

  // numeric velocity readout
  const float speed = std::sqrt(state.vx_mps * state.vx_mps + state.vy_mps * state.vy_mps);
  const float heading_degrees =
      speed > 0.0f ? std::atan2(state.vy_mps, state.vx_mps) * kRadiansToDegrees : 0.0f;
  auto velocity_text = fmt::format("vx (fwd):  {:+.3f} m/s\n"
                                   "vy (left): {:+.3f} m/s\n"
                                   "|v|:       {:.3f} m/s\n"
                                   "heading:   {:+.0f} deg\n"
                                   "rotation:  {:+.2f} RPM (CCW)",
                                   state.vx_mps, state.vy_mps, speed, heading_degrees, state.w_rpm);
  lv_label_set_text(velocity_label_, velocity_text.c_str());

  auto limits_text = fmt::format("Limits: {:.2f} m/s | {:.1f} RPM | wheel {:.0f} RPM",
                                 state.max_speed_mps, state.max_rotation_rpm, state.max_wheel_rpm);
  lv_label_set_text(limits_label_, limits_text.c_str());

  // vector visualization (full deflection = the max-speed limit)
  const float max_speed = std::max(state.max_speed_mps, 1e-3f);
  set_vector_line(std::clamp(state.vx_mps / max_speed, -1.0f, 1.0f),
                  std::clamp(state.vy_mps / max_speed, -1.0f, 1.0f));

  // motor tiles
  for (size_t i = 0; i < motor_tiles_.size(); ++i) {
    const auto &motor = state.motors[i];
    std::string text;
    if (motor.valid) {
      text = fmt::format("M{}  {}\ncmd {:+.1f} RPM\nmeas {:+.1f} RPM  {} C", motor.id,
                         motor.stale ? "STALE" : "OK", motor.commanded_rpm, motor.velocity_rpm,
                         motor.temperature_c);
    } else {
      text = fmt::format("M{}  NO DATA\ncmd {:+.1f} RPM", motor.id, motor.commanded_rpm);
    }
    lv_label_set_text(motor_tile_labels_[i], text.c_str());
    lv_obj_set_style_border_color(
        motor_tiles_[i],
        motor.stale ? lv_palette_main(LV_PALETTE_RED) : lv_palette_main(LV_PALETTE_GREEN), 0);
  }

  // the touch controls: user-driven while the GUI is the active source,
  // read-only indicators of the live command otherwise
  if (!gui_active) {
    set_knob_position(std::clamp(state.vx_mps / max_speed, -1.0f, 1.0f),
                      std::clamp(state.vy_mps / max_speed, -1.0f, 1.0f));
    const float max_rotation = std::max(state.max_rotation_rpm, 1e-3f);
    const int rotation_value = static_cast<int>(
        std::clamp(-state.w_rpm / max_rotation, -1.0f, 1.0f) * kRotationCommandRange);
    lv_slider_set_value(rotation_slider_, rotation_value, LV_ANIM_OFF);
  } else if (source_changed) {
    // handover (back) to the GUI: the controller zeroed its setpoint, snap
    // the controls to match so they don't re-command stale values
    set_knob_position(0.0f, 0.0f);
    lv_slider_set_value(rotation_slider_, 0, LV_ANIM_OFF);
  }
  last_source_ = state.source;

  // keep the limit sliders in sync with the controller (e.g. when limits are
  // changed over the CLI), without fighting an in-progress drag
  if (state.max_speed_mps != last_max_speed_ &&
      !lv_obj_has_state(max_speed_slider_, LV_STATE_PRESSED)) {
    lv_slider_set_value(max_speed_slider_,
                        static_cast<int>(state.max_speed_mps / kSpeedSliderScale), LV_ANIM_OFF);
  }
  if (state.max_rotation_rpm != last_max_rotation_ &&
      !lv_obj_has_state(max_rotation_slider_, LV_STATE_PRESSED)) {
    lv_slider_set_value(max_rotation_slider_,
                        static_cast<int>(state.max_rotation_rpm / kRotationSliderScale),
                        LV_ANIM_OFF);
  }
  last_max_speed_ = state.max_speed_mps;
  last_max_rotation_ = state.max_rotation_rpm;
}
