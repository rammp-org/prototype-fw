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
  lv_obj_t *screen = lv_screen_active();

  // Effective display size at runtime (the BSP may present the native 720x1280
  // portrait panel rotated to 1280x720 landscape). Lay everything out relative
  // to these so the UI fills the screen and never overlaps in either case.
  lv_display_t *disp = lv_display_get_default();
  const int32_t screen_w = lv_display_get_horizontal_resolution(disp);
  const int32_t screen_h = lv_display_get_vertical_resolution(disp);
  const bool landscape = screen_w >= screen_h;
  // The drag pad / vector circle scale with the smaller screen dimension so
  // they always fit next to the other panel.
  const int32_t small_dim = std::min(screen_w, screen_h);
  pad_size_ = std::clamp(static_cast<int>(small_dim / (landscape ? 3 : 2)), 200, 340);
  viz_size_ = static_cast<int>(pad_size_ * 0.85f);

  // Root: a vertical flex of [top bar][content (grows)][motor row], full width.
  lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_pad_all(screen, 8, 0);
  lv_obj_set_style_pad_row(screen, 8, 0);
  lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(screen, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

  init_top_bar(screen);

  // Content area grows to fill the middle; a row in landscape (left panel +
  // right panel), a column in portrait (stacked).
  lv_obj_t *content = lv_obj_create(screen);
  lv_obj_remove_style_all(content);
  lv_obj_set_width(content, LV_PCT(100));
  lv_obj_set_flex_grow(content, 1);
  lv_obj_clear_flag(content, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_flex_flow(content, landscape ? LV_FLEX_FLOW_ROW : LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(content, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_set_style_pad_column(content, 10, 0);
  lv_obj_set_style_pad_row(content, 10, 0);

  // Each panel takes an explicit share of the content container. In landscape
  // the two panels sit side-by-side (48% each, leaving room for the column
  // gap); in portrait they stack full-width. flex_grow is intentionally NOT
  // used here - with the children's content (a fixed-size circle / drag-pad)
  // driving the intrinsic width, flex_grow failed to expand the panels and
  // they collapsed to ~10-20% of the screen. Explicit LV_PCT widths match the
  // full-width pattern used by the top bar and motor row, which works reliably.
  const int panel_width_pct = landscape ? 48 : 100;

  // Left panel: velocity vector + numeric readouts + limits.
  lv_obj_t *left = lv_obj_create(content);
  lv_obj_remove_style_all(left);
  lv_obj_set_width(left, LV_PCT(panel_width_pct));
  lv_obj_set_height(left, LV_PCT(100));
  lv_obj_clear_flag(left, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_flex_flow(left, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(left, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
  lv_obj_set_style_pad_row(left, 10, 0);
  init_vector_display(left, viz_size_);

  // Right panel: touch drag-pad + rotation / limit sliders.
  lv_obj_t *right = lv_obj_create(content);
  lv_obj_remove_style_all(right);
  lv_obj_set_width(right, LV_PCT(panel_width_pct));
  lv_obj_set_height(right, LV_PCT(100));
  lv_obj_clear_flag(right, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_flex_flow(right, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(right, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
  lv_obj_set_style_pad_row(right, 8, 0);
  init_drag_pad(right, pad_size_);
  init_sliders(right);

  // Bottom: the four motor tiles in a row, full width.
  init_motor_tiles(screen);

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
  disable_button_ = nullptr;
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

void Gui::init_top_bar(lv_obj_t *parent) {
  // A fixed-height row: title | source badge (grows) | DISABLE | ENABLE/STOP.
  lv_obj_t *bar = lv_obj_create(parent);
  lv_obj_remove_style_all(bar);
  lv_obj_set_width(bar, LV_PCT(100));
  lv_obj_set_height(bar, LV_SIZE_CONTENT);
  lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(bar, 12, 0);

  lv_obj_t *title = lv_label_create(bar);
  lv_label_set_text(title, "Holo Deck");

  // the active-source indicator is the most prominent element in the bar; it
  // grows to take the middle space and is recolored by show_source()
  source_label_ = lv_label_create(bar);
  lv_label_set_text(source_label_, "STOPPED");
  lv_obj_set_style_bg_opa(source_label_, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(source_label_, lv_palette_main(LV_PALETTE_RED), 0);
  lv_obj_set_style_text_color(source_label_, lv_color_white(), 0);
  lv_obj_set_style_pad_all(source_label_, 12, 0);
  lv_obj_set_style_radius(source_label_, 8, 0);
  lv_obj_set_style_text_align(source_label_, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_flex_grow(source_label_, 1);

  // DISABLE: halt the motors at the control level (distinct from STOP).
  disable_button_ = lv_btn_create(bar);
  lv_obj_set_height(disable_button_, 64);
  lv_obj_set_style_bg_color(disable_button_, lv_palette_main(LV_PALETTE_GREY), 0);
  lv_obj_t *disable_label = lv_label_create(disable_button_);
  lv_label_set_text(disable_label, LV_SYMBOL_POWER " DISABLE");
  lv_obj_center(disable_label);
  lv_obj_add_event_cb(disable_button_, event_callback, LV_EVENT_PRESSED, this);

  enable_button_ = lv_btn_create(bar);
  lv_obj_set_height(enable_button_, 64);
  lv_obj_set_style_bg_color(enable_button_, lv_palette_main(LV_PALETTE_GREEN), 0);
  enable_button_label_ = lv_label_create(enable_button_);
  lv_label_set_text(enable_button_label_, LV_SYMBOL_PLAY " ENABLE");
  lv_obj_center(enable_button_label_);
  lv_obj_add_event_cb(enable_button_, event_callback, LV_EVENT_PRESSED, this);
}

void Gui::init_vector_display(lv_obj_t *parent, int viz_size) {
  // circle showing the commanded velocity vector (direction + magnitude,
  // full radius = the max-speed limit)
  vector_circle_ = lv_obj_create(parent);
  lv_obj_set_size(vector_circle_, viz_size, viz_size);
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

  velocity_label_ = lv_label_create(parent);
  lv_label_set_text(velocity_label_, "");

  limits_label_ = lv_label_create(parent);
  lv_label_set_text(limits_label_, "");
}

void Gui::init_motor_tiles(lv_obj_t *parent) {
  // A full-width row of four equal tiles.
  lv_obj_t *row = lv_obj_create(parent);
  lv_obj_remove_style_all(row);
  lv_obj_set_width(row, LV_PCT(100));
  lv_obj_set_height(row, LV_SIZE_CONTENT);
  lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
  lv_obj_set_style_pad_column(row, 8, 0);
  for (size_t i = 0; i < motor_tiles_.size(); ++i) {
    lv_obj_t *tile = lv_obj_create(row);
    lv_obj_set_flex_grow(tile, 1);
    lv_obj_set_height(tile, LV_SIZE_CONTENT);
    lv_obj_clear_flag(tile, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(tile, 3, 0);
    lv_obj_set_style_border_color(tile, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_pad_all(tile, 8, 0);
    lv_obj_t *label = lv_label_create(tile);
    lv_label_set_text_fmt(label, "M%d\nwaiting...", static_cast<int>(i) + 1);
    motor_tiles_[i] = tile;
    motor_tile_labels_[i] = label;
  }
}

void Gui::init_drag_pad(lv_obj_t *parent, int pad_size) {
  pad_ = lv_obj_create(parent);
  lv_obj_set_size(pad_, pad_size, pad_size);
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

  pad_label_ = lv_label_create(parent);
  lv_label_set_text(pad_label_, "Drag to translate (up = forward)");
}

void Gui::init_sliders(lv_obj_t *parent) {
  struct SliderSpec {
    lv_obj_t **slider;
    lv_obj_t **label;
    int min;
    int max;
    int value;
  };
  const SliderSpec sliders[] = {
      {&rotation_slider_, &rotation_label_, -kRotationCommandRange, kRotationCommandRange, 0},
      {&max_speed_slider_, &max_speed_label_,
       static_cast<int>(hw_config::kMinSelectableMaxSpeedMps / kSpeedSliderScale),
       static_cast<int>(hw_config::kMaxSelectableMaxSpeedMps / kSpeedSliderScale),
       static_cast<int>(hw_config::kDefaultMaxSpeedMps / kSpeedSliderScale)},
      {&max_rotation_slider_, &max_rotation_label_,
       static_cast<int>(hw_config::kMinSelectableMaxRotationRpm / kRotationSliderScale),
       static_cast<int>(hw_config::kMaxSelectableMaxRotationRpm / kRotationSliderScale),
       static_cast<int>(hw_config::kDefaultMaxRotationRpm / kRotationSliderScale)},
  };
  for (const auto &spec : sliders) {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_t *slider = lv_slider_create(parent);
    lv_obj_set_width(slider, LV_PCT(90));
    lv_slider_set_range(slider, spec.min, spec.max);
    lv_slider_set_value(slider, spec.value, LV_ANIM_OFF);
    lv_obj_add_event_cb(slider, event_callback, LV_EVENT_VALUE_CHANGED, this);
    *spec.slider = slider;
    *spec.label = label;
  }
  lv_label_set_text(rotation_label_, "Rotation (left = CCW)");
  lv_label_set_text(max_speed_label_, "Max speed");
  lv_label_set_text(max_rotation_label_, "Max rotation (deg/s)");
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
  } else if (target == disable_button_ && disable_callback_) {
    // control-level motor disable (independent of the ENABLE/STOP toggle)
    disable_callback_();
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
  const float radius = (pad_size_ - KNOB_SIZE) / 2.0f;
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
  const float radius = (pad_size_ - KNOB_SIZE) / 2.0f;
  const int x = static_cast<int>(-left * radius);
  const int y = static_cast<int>(-forward * radius);
  lv_obj_align(pad_knob_, LV_ALIGN_CENTER, x, y);
}

void Gui::set_vector_line(float forward_norm, float left_norm) {
  const float radius = (viz_size_ - 40) / 2.0f;
  const float center = viz_size_ / 2.0f - 15.0f; // inside the padded circle
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

  // Source / mode banner. DISABLED (control-level motor halt) is distinct from
  // STOPPED (e-stop holding zero velocity).
  if (state.mode == HoloDeckController::Mode::DISABLED) {
    lv_label_set_text(source_label_, "MOTORS DISABLED");
    lv_obj_set_style_bg_color(source_label_, lv_palette_darken(LV_PALETTE_GREY, 2), 0);
  } else {
    show_source(state.source);
  }

  // ENABLE / STOP button: pressing it e-stops when enabled (DRIVE), enables
  // otherwise (STOPPED or DISABLED both return to DRIVE).
  if (state.enabled) {
    lv_label_set_text(enable_button_label_, LV_SYMBOL_STOP " STOP");
    lv_obj_set_style_bg_color(enable_button_, lv_palette_main(LV_PALETTE_RED), 0);
  } else {
    lv_label_set_text(enable_button_label_, LV_SYMBOL_PLAY " ENABLE");
    lv_obj_set_style_bg_color(enable_button_, lv_palette_main(LV_PALETTE_GREEN), 0);
  }
  // DISABLE button: highlighted while the motors are disabled.
  lv_obj_set_style_bg_color(disable_button_,
                            state.mode == HoloDeckController::Mode::DISABLED
                                ? lv_palette_main(LV_PALETTE_ORANGE)
                                : lv_palette_main(LV_PALETTE_GREY),
                            0);

  // numeric velocity readout
  const float speed = std::sqrt(state.vx_mps * state.vx_mps + state.vy_mps * state.vy_mps);
  const float heading_degrees =
      speed > 0.0f ? std::atan2(state.vy_mps, state.vx_mps) * kRadiansToDegrees : 0.0f;
  // Rotation is shown in deg/s (more intuitive than chassis RPM).
  const float w_dps = state.w_rpm * hw_config::kRpmToDegPerSec;
  auto velocity_text = fmt::format("vx (fwd):  {:+.3f} m/s\n"
                                   "vy (left): {:+.3f} m/s\n"
                                   "|v|:       {:.3f} m/s\n"
                                   "heading:   {:+.0f} deg\n"
                                   "rotation:  {:+.0f} deg/s (CCW)",
                                   state.vx_mps, state.vy_mps, speed, heading_degrees, w_dps);
  lv_label_set_text(velocity_label_, velocity_text.c_str());

  auto limits_text =
      fmt::format("Limits: {:.2f} m/s | {:.0f} deg/s | wheel {:.0f} RPM", state.max_speed_mps,
                  state.max_rotation_rpm * hw_config::kRpmToDegPerSec, state.max_wheel_rpm);
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
