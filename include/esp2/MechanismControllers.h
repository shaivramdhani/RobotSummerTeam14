#pragma once

#include <cstdint>

namespace robot::esp2 {

enum class ServoBankRequest : std::uint8_t {
  Disabled = 0,
  HoldCurrent = 1,
  PrepareClaws = 2,
  ReleaseSequentially = 3,
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
