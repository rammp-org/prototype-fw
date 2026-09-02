#include <array>
#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "ik_5bar.hpp"
#include "motor_actuator.hpp"
#include "paired_actuator.hpp"

namespace {

constexpr uart_port_t kUartPort = UART_NUM_0;
constexpr int kUartBaudRate = 115200;
constexpr int kUartBufferSize = 512;
constexpr uint32_t kReadTimeoutTicks = pdMS_TO_TICKS(100);
constexpr float kMoveRpm = 2.0f;
constexpr float kSetRpm = 5.0f;

SemaphoreHandle_t response_mutex;

void send_response(const char *format, ...) {
  char response[256];
  va_list args;
  va_start(args, format);
  const int length = std::vsnprintf(response, sizeof(response), format, args);
  va_end(args);
  if (length > 0) {
    if (response_mutex != nullptr) {
      xSemaphoreTake(response_mutex, portMAX_DELAY);
    }
    uart_write_bytes(kUartPort, response, length);
    if (response_mutex != nullptr) {
      xSemaphoreGive(response_mutex);
    }
  }
}

class StemController {
public:
  StemController(PairedActuator &right, PairedActuator &left, PairedActuator &seat)
      : right_(right), left_(left), seat_(seat) {}

  bool move(float x, float y) {
    stem::GeometryConfig config{};
    stem::IkSolution solution{};
    if (!stem::solve_ik_for_m3_reference(x, y, stem::kFullDownReference, solution, config)) {
      return false;
    }

    const bool left_ok = left_.set_position(-solution.theta1_deg, kMoveRpm);
    const bool right_ok = right_.set_position(-solution.theta2_deg, kMoveRpm);
    const bool seat_ok = seat_.set_position(-solution.m3_angle_deg, kMoveRpm);
    if (!left_ok || !right_ok || !seat_ok) {
      return false;
    }

    send_response("OK MOVE x=%.3f y=%.3f left=%.3f right=%.3f seat=%.3f\n", x, y,
                  solution.theta1_deg, solution.theta2_deg, solution.m3_angle_deg);
    return true;
  }

  void home() {
    if (!move(0.0f, 0.0f)) {
      send_response("ERROR HOME failed\n");
      return;
    }
    send_response("OK HOME\n");
  }

  void set_pair(const char *name, float degrees) {
    PairedActuator *pair = pair_for_name(name);
    if (pair == nullptr) {
      send_response("ERROR SET unknown_pair=%s\n", name);
      return;
    }
    if (!pair->set_position(degrees, kSetRpm)) {
      send_response("ERROR SET failed pair=%s\n", name);
      return;
    }
    send_response("OK SET pair=%s degrees=%.3f\n", name, degrees);
  }

  void stop() {
    const bool right_ok = right_.stop();
    const bool left_ok = left_.stop();
    const bool seat_ok = seat_.stop();
    send_response("%s STOP\n", right_ok && left_ok && seat_ok ? "OK" : "ERROR");
  }

  void release() {
    const bool right_ok = right_.release_brake();
    const bool left_ok = left_.release_brake();
    const bool seat_ok = seat_.release_brake();
    send_response("%s RELEASE\n", right_ok && left_ok && seat_ok ? "OK" : "ERROR");
  }

  void status() const {
    const auto right = right_.get_position();
    const auto left = left_.get_position();
    const auto seat = seat_.get_position();
    send_response("OK STATUS {\"right\":[%.3f,%.3f],\"left\":[%.3f,%.3f],\"seat\":[%.3f,%.3f]}\n",
                  right[0], right[1], left[0], left[1], seat[0], seat[1]);
  }

private:
  PairedActuator *pair_for_name(const char *name) {
    if (std::strcmp(name, "right") == 0) return &right_;
    if (std::strcmp(name, "left") == 0) return &left_;
    if (std::strcmp(name, "seat") == 0) return &seat_;
    return nullptr;
  }

  PairedActuator &right_;
  PairedActuator &left_;
  PairedActuator &seat_;
};

struct ProtocolContext {
  std::atomic<StemController *> controller{nullptr};
};

void process_command(char *line, ProtocolContext &context) {
  char command[16]{};
  if (std::sscanf(line, "%15s", command) != 1) return;

  if (std::strcmp(command, "PING") == 0) {
    send_response("OK PONG protocol=1\n");
    return;
  }
  if (std::strcmp(command, "HOME") == 0) {
    if (auto *controller = context.controller.load()) {
      controller->home();
    } else {
      send_response("ERROR controller_not_ready\n");
    }
    return;
  }
  if (std::strcmp(command, "STATUS") == 0) {
    if (auto *controller = context.controller.load()) {
      controller->status();
    } else {
      send_response("ERROR controller_not_ready\n");
    }
    return;
  }
  if (std::strcmp(command, "STOP") == 0) {
    if (auto *controller = context.controller.load()) {
      controller->stop();
    } else {
      send_response("ERROR controller_not_ready\n");
    }
    return;
  }
  if (std::strcmp(command, "RELEASE") == 0) {
    if (auto *controller = context.controller.load()) {
      controller->release();
    } else {
      send_response("ERROR controller_not_ready\n");
    }
    return;
  }

  float x = 0.0f;
  float y = 0.0f;
  if (std::strcmp(command, "MOVE") == 0 &&
      std::sscanf(line, "%*s %f %f", &x, &y) == 2 && std::isfinite(x) && std::isfinite(y)) {
    auto *controller = context.controller.load();
    if (controller == nullptr) {
      send_response("ERROR controller_not_ready\n");
    } else if (!controller->move(x, y)) {
      send_response("ERROR MOVE unreachable_or_failed x=%.3f y=%.3f\n", x, y);
    }
    return;
  }

  char pair[8]{};
  float degrees = 0.0f;
  if (std::strcmp(command, "SET") == 0 &&
      std::sscanf(line, "%*s %7s %f", pair, &degrees) == 2 && std::isfinite(degrees)) {
    auto *controller = context.controller.load();
    if (controller == nullptr) {
      send_response("ERROR controller_not_ready\n");
    } else {
      controller->set_pair(pair, degrees);
    }
    return;
  }

  send_response("ERROR invalid_command\n");
}

void serial_task(void *context) {
  auto &protocol = *static_cast<ProtocolContext *>(context);
  char line[128]{};
  size_t length = 0;
  uint8_t byte = 0;

  while (true) {
    const int count = uart_read_bytes(kUartPort, &byte, 1, kReadTimeoutTicks);
    if (count != 1) continue;
    send_response("RX byte=0x%02X (%u)\n", byte, byte);
    if (byte == '\r') continue;
    if (byte == '\n') {
      line[length] = '\0';
      send_response("RX frame length=%u text=\\\"%s\\\"\n",
                    static_cast<unsigned>(length), line);
      process_command(line, protocol);
      length = 0;
      continue;
    }
    if (length + 1 >= sizeof(line)) {
      length = 0;
      send_response("ERROR command_too_long\n");
      continue;
    }
    line[length++] = static_cast<char>(byte);
  }
}

void heartbeat_task(void *) {
  while (true) {
    send_response("HEARTBEAT\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

} // namespace

extern "C" void app_main(void) {
  const uart_config_t uart_config = {
      .baud_rate = kUartBaudRate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 0,
      .source_clk = UART_SCLK_DEFAULT,
      .flags = {},
  };
  const esp_err_t config_result = uart_param_config(kUartPort, &uart_config);
  if (config_result != ESP_OK) {
    esp_rom_printf("ERROR UART0 config failed: %s\n", esp_err_to_name(config_result));
    return;
  }
  if (!uart_is_driver_installed(kUartPort)) {
    const esp_err_t install_result = uart_driver_install(
        kUartPort, kUartBufferSize, kUartBufferSize, 0, nullptr, 0);
    if (install_result != ESP_OK) {
      esp_rom_printf("ERROR UART0 driver install failed: %s\n",
                     esp_err_to_name(install_result));
      return;
    }
  }

  response_mutex = xSemaphoreCreateMutex();
  if (response_mutex == nullptr) {
    esp_rom_printf("ERROR UART0 response mutex allocation failed\n");
    return;
  }

  static ProtocolContext protocol;
  send_response("READY protocol=1 transport=uart0 baud=115200\n");
  xTaskCreate(serial_task, "serial_protocol", 4096, &protocol, 5, nullptr);
  xTaskCreate(heartbeat_task, "heartbeat", 2048, nullptr, 4, nullptr);

  MotorCanBus can_bus(GPIO_NUM_16, GPIO_NUM_17);
  if (!can_bus.start()) {
    send_response("ERROR CAN start_failed\n");
    return;
  }

  MotorActuator::CommunicationFunction communicate =
      [&can_bus](const MotorPacket &command, MotorPacket &response, uint32_t timeout_ms) {
        if (timeout_ms == 0) return can_bus.send(command);
        return can_bus.request(command, response, timeout_ms);
      };
  MotorActuator actuators[] = {
      MotorActuator(communicate, 1), MotorActuator(communicate, 2),
      MotorActuator(communicate, 3), MotorActuator(communicate, 4),
      MotorActuator(communicate, 5), MotorActuator(communicate, 6),
  };

  PairedActuator right(actuators[3], actuators[2]);
  PairedActuator left(actuators[1], actuators[0]);
  PairedActuator seat(actuators[5], actuators[4]);
  right.zero_position(false);
  left.zero_position(false);
  seat.zero_position(false);
  right.set_position_limits(-60.0f, 60.0f);
  left.set_position_limits(-60.0f, 60.0f);
  seat.set_position_limits(-60.0f, 60.0f);

  StemController controller(right, left, seat);
  protocol.controller.store(&controller);
  send_response("READY controller=ready\n");
}
