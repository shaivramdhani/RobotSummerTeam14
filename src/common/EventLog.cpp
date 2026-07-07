#include "common/EventLog.h"

#include <cstring>

namespace robot {

const char* eventSeverityName(const EventSeverity severity) {
  switch (severity) {
    case EventSeverity::Info:
      return "INFO";
    case EventSeverity::Warn:
      return "WARN";
    case EventSeverity::Fault:
      return "FAULT";
  }
  return "INFO";
}

const char* eventSourceName(const EventSource source) {
  switch (source) {
    case EventSource::Web:
      return "WEB";
    case EventSource::Serial:
      return "SERIAL";
    case EventSource::Line:
      return "LINE";
    case EventSource::Motor:
      return "MOTOR";
    case EventSource::Uart:
      return "UART";
    case EventSource::System:
      return "SYSTEM";
  }
  return "SYSTEM";
}

void EventLog::add(const Milliseconds timestamp_ms,
                   const EventSeverity severity,
                   const EventSource source,
                   const char* const message) {
  EventRecord& record = records_[next_index_];
  record.timestamp_ms = timestamp_ms;
  record.severity = severity;
  record.source = source;
  record.message[0] = '\0';

  if (message != nullptr) {
    std::strncpy(record.message, message, kEventMessageSize - 1U);
    record.message[kEventMessageSize - 1U] = '\0';
  }

  next_index_ = (next_index_ + 1U) % kEventLogCapacity;
  if (count_ < kEventLogCapacity) {
    ++count_;
  }
}

bool EventLog::newest(const std::size_t newest_index,
                      EventRecord& output) const {
  if (newest_index >= count_) {
    output = {};
    return false;
  }

  const std::size_t newest_slot =
      (next_index_ + kEventLogCapacity - 1U) % kEventLogCapacity;
  const std::size_t slot =
      (newest_slot + kEventLogCapacity -
       (newest_index % kEventLogCapacity)) %
      kEventLogCapacity;
  output = records_[slot];
  return true;
}

}  // namespace robot
