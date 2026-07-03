#pragma once

#include <cstdint>

namespace robot::esp2 {

enum class StepperPositionRequest : std::uint8_t {
  Disabled = 0,
  HoldCurrent = 1,
  MoveToBottom = 2,
  MoveToMiddle = 3,
  MoveToTop = 4,
};

enum class ServoBankRequest : std::uint8_t {
  Disabled = 0,
  HoldCurrent = 1,
  PrepareClaws = 2,
  ReleaseSequentially = 3,
};

class StepperController {
 public:
  void initializeDisabled() {}
  void disable() { request_ = StepperPositionRequest::Disabled; }
  void request(StepperPositionRequest request) { request_ = request; }
  StepperPositionRequest request() const { return request_; }

 private:
  StepperPositionRequest request_{StepperPositionRequest::Disabled};
};

class ServoBankController {
 public:
  void initializeDisabled() {}
  void disable() { request_ = ServoBankRequest::Disabled; }
  void request(ServoBankRequest request) { request_ = request; }
  ServoBankRequest request() const { return request_; }

 private:
  ServoBankRequest request_{ServoBankRequest::Disabled};
};

}  // namespace robot::esp2
