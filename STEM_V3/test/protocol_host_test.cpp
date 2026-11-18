// Host-side test for the STEM wire protocol helpers (stem_protocol.hpp) against
// the real espp stream_frame codec: every frame the firmware builds must parse
// back with the right module / type / reply flag / payload. Run via
// run_host_tests.sh (needs the espp stream_frame header on the include path).

#include <cstdio>
#include <cstdlib>
#include <string>

#include "stem_protocol.hpp"

namespace proto = stem_proto;

namespace {

int failures = 0;

void check(bool ok, const char *what) {
  if (!ok) {
    ++failures;
    std::printf("FAIL: %s\n", what);
  }
}

proto::stream::Frame parse_one(const std::vector<uint8_t> &bytes, const char *what) {
  proto::stream::StreamParser parser;
  const auto frames = parser.feed(bytes);
  check(frames.size() == 1, what);
  return frames.empty() ? proto::stream::Frame{} : frames.front();
}

} // namespace

int main() {
  // A request type has the reply flag clear; reply / telemetry types set it.
  {
    std::vector<uint8_t> payload;
    proto::put_f32(payload, 12.5f);
    proto::put_f32(payload, -3.0f);
    proto::put_f32(payload, 2.0f);
    const auto f = parse_one(proto::build(proto::Msg::MoveTo, payload), "MOVE_TO parses");
    check(f.module == proto::kModule, "MOVE_TO module");
    check(f.type == static_cast<uint8_t>(proto::Msg::MoveTo), "MOVE_TO type");
    check(!f.is_reply(), "MOVE_TO is a request");
    check(f.payload.size() == 12, "MOVE_TO payload size");
    check(proto::get_f32_at(f.payload, 0) == 12.5f && proto::get_f32_at(f.payload, 4) == -3.0f &&
              proto::get_f32_at(f.payload, 8) == 2.0f,
          "MOVE_TO f32 round trip");
  }
  {
    const auto f = parse_one(proto::make_ok(proto::Msg::SetStreaming, 200), "OK parses");
    check(f.type == static_cast<uint8_t>(proto::Msg::Ok) && f.is_reply(), "OK is a reply");
    check(f.payload.size() == 5 && f.payload[0] == static_cast<uint8_t>(proto::Msg::SetStreaming),
          "OK carries the request type");
    check(proto::get_u32(std::span<const uint8_t>(f.payload).subspan(1)) == 200, "OK value");
  }
  {
    const auto f =
        parse_one(proto::make_error(proto::Msg::MoveTo, 33, "move rejected: unreachable"),
                  "ERROR parses");
    check(f.type == static_cast<uint8_t>(proto::Msg::Error) && f.is_reply(), "ERROR is a reply");
    check(f.payload.size() == 1 + 4 + 26, "ERROR payload layout");
    check(f.payload[0] == static_cast<uint8_t>(proto::Msg::MoveTo), "ERROR request type");
    check(proto::get_u32(std::span<const uint8_t>(f.payload).subspan(1)) == 33, "ERROR code");
    const std::string message(f.payload.begin() + 5, f.payload.end());
    check(message == "move rejected: unreachable", "ERROR message");
  }
  {
    // Strings are u8-length-prefixed and truncated at 255 bytes.
    std::vector<uint8_t> payload;
    proto::put_str(payload, std::string(300, 'x'));
    check(payload.size() == 256 && payload[0] == 255, "put_str truncates to 255");
  }
  {
    // Getters reject short payloads instead of reading past the end.
    const std::vector<uint8_t> three{1, 2, 3};
    check(!proto::get_f32_at(three, 0).has_value(), "get_f32_at bounds");
    check(proto::get_u16_at(three, 1).value() == 0x0302, "get_u16_at little-endian");
    check(!proto::get_u8_at(three, 3).has_value(), "get_u8_at bounds");
  }
  {
    // Two frames back to back, with junk in between, both parse (resync).
    auto bytes = proto::build(proto::Msg::GetInfo);
    bytes.insert(bytes.end(), {0x00, 0x54, 0xFF, 0x13});
    const auto second = proto::build(proto::Msg::Stop);
    bytes.insert(bytes.end(), second.begin(), second.end());
    proto::stream::StreamParser parser;
    const auto frames = parser.feed(bytes);
    check(frames.size() == 2, "two frames with junk between parse");
    check(frames.size() == 2 && frames[0].type == static_cast<uint8_t>(proto::Msg::GetInfo) &&
              frames[1].type == static_cast<uint8_t>(proto::Msg::Stop),
          "frame order preserved");
  }

  if (failures) {
    std::printf("%d failure(s)\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("protocol host test: all checks passed\n");
  return EXIT_SUCCESS;
}
