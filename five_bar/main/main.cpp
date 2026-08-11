#include <atomic>
#include <chrono>
#include <thread>

#include "logger.hpp"
#include "motorgo-plink.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {

  auto &board = espp::MotorGoPlink::get();

  espp::Logger logger({.tag = "five_bar", .level = espp::Logger::Verbosity::DEBUG});

  logger.info("Bootup");

  if (!board.initialize_leds()) {
    logger.warn("Failed to initialize indicator LEDs");
  } else {
    board.start_led_breathing();
  }

  // init motor
  if (!board.initialize_motors()) {
    logger.error("Failed to initialize motor drivers");
  } else {
    board.stop_all_motors();
  }

  // move first motor to 20% speed for 5 seconds and stop
  if (!board.set_motor_speed(0, 0.5f)) {
    logger.error("Failed to set motor 1 speed");
  } else {
    logger.info("Motor 1 speed set to 20%");
    std::this_thread::sleep_for(5s);
    board.stop_motor(0);
    logger.info("Motor 1 stopped");
  }

  // init the encoders
  if (!board.initialize_encoders(false)) {
    logger.warn("Failed to initialize encoders");
  } else {
    logger.info("Encoder polling enabled");
  }

  std::atomic<int> counter{0};

  // also print in the main thread
  while (true) {
    logger.debug("[{}] Hello World!", counter++);
    std::this_thread::sleep_for(5s);
  }
}
