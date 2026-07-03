#pragma once

#include <cstdint>

#include "common/Units.h"

namespace robot {

enum class LineSample : std::uint8_t {
  Unknown = 0,
  OffTape = 1,
  OnTape = 2,
};

struct FrontLineSensorSnapshot {
  LineSample left{LineSample::Unknown};
  LineSample right{LineSample::Unknown};
  Milliseconds captured_at_ms{0};
  bool valid{false};
};

class ILineSensorReader {
 public:
  virtual ~ILineSensorReader() = default;
  virtual FrontLineSensorSnapshot readSnapshot(Milliseconds now_ms) = 0;
};

class NullLineSensorReader final : public ILineSensorReader {
 public:
  FrontLineSensorSnapshot readSnapshot(Milliseconds now_ms) override {
    return {LineSample::Unknown, LineSample::Unknown, now_ms, false};
  }
};

}  // namespace robot
