#pragma once

#include <cstdint>

#include "common/LineSensor.h"

namespace robot {

enum class LineObservationKind : std::uint8_t {
  Invalid = 0,
  Lost = 1,
  Centered = 2,
  LeftOnly = 3,
  RightOnly = 4,
  LostWithHistory = 5,
  LostWithoutHistory = 6,
};

struct LineObservation {
  LineObservationKind kind{LineObservationKind::Invalid};

  bool leftOnTape{false};
  bool rightOnTape{false};
  std::int8_t lastKnownSide{0};
  bool lineVisible{false};
  bool hasHistory{false};
  Milliseconds timestampMs{0};

  // Compatibility fields for existing call sites and docs.
  bool left_black{false};
  bool right_black{false};
  std::int8_t error{0};
  std::int8_t last_known_side{0};
  bool line_visible{false};
  bool safe_to_drive{false};
  Milliseconds timestamp_ms{0};
  std::int16_t line_error_milli{0};
  Milliseconds observed_at_ms{0};
};

constexpr std::int8_t normalizeLastKnownSide(std::int8_t side) {
  return side > 0 ? static_cast<std::int8_t>(1)
                  : (side < 0 ? static_cast<std::int8_t>(-1)
                              : static_cast<std::int8_t>(0));
}

LineObservation observeDigitalLineSensors(bool left_black, bool right_black,
                                          std::int8_t previous_last_known_side,
                                          Milliseconds timestamp_ms);
LineObservation observeDigitalLineSensorLevels(
    bool left_electrical_high, bool right_electrical_high,
    std::int8_t previous_last_known_side, Milliseconds timestamp_ms);
LineObservation observeRearLineSensorsForReverseTravel(
    bool back_left_black, bool back_right_black,
    std::int8_t previous_last_known_side, Milliseconds timestamp_ms);
LineObservation observeFrontLine(const FrontLineSensorSnapshot& snapshot);

}  // namespace robot
