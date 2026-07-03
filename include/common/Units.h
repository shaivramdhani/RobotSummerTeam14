#pragma once

#include <cstdint>

namespace robot {

using Milliseconds = std::uint32_t;

constexpr Milliseconds kDefaultCommunicationTimeoutMs = 250U;
constexpr std::int16_t kMinCommandMilli = -1000;
constexpr std::int16_t kMaxCommandMilli = 1000;

constexpr std::int16_t clampCommandMilli(const std::int16_t command_milli) {
  return command_milli < kMinCommandMilli
             ? kMinCommandMilli
             : (command_milli > kMaxCommandMilli ? kMaxCommandMilli
                                                 : command_milli);
}

}  // namespace robot
