#pragma once

#include <cstddef>
#include <cstdint>

#include "common/Units.h"

namespace robot {

enum class EventSeverity : std::uint8_t {
  Info = 0,
  Warn = 1,
  Fault = 2,
};

enum class EventSource : std::uint8_t {
  Web = 0,
  Serial = 1,
  Line = 2,
  Motor = 3,
  Uart = 4,
  System = 5,
};

constexpr std::size_t kEventMessageSize = 64U;
constexpr std::size_t kEventLogCapacity = 16U;

struct EventRecord {
  Milliseconds timestamp_ms{0};
  EventSeverity severity{EventSeverity::Info};
  EventSource source{EventSource::System};
  char message[kEventMessageSize]{};
};

const char* eventSeverityName(EventSeverity severity);
const char* eventSourceName(EventSource source);

class EventLog {
 public:
  void add(Milliseconds timestamp_ms, EventSeverity severity,
           EventSource source, const char* message);
  std::size_t size() const { return count_; }
  std::size_t capacity() const { return kEventLogCapacity; }
  bool newest(std::size_t newest_index, EventRecord& output) const;

 private:
  EventRecord records_[kEventLogCapacity]{};
  std::size_t next_index_{0};
  std::size_t count_{0};
};

}  // namespace robot
