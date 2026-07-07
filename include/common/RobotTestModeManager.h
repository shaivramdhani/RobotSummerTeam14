#pragma once

#include "common/RobotTestMode.h"
#include "common/Units.h"

namespace robot {

class RobotTestModeManager {
 public:
  RobotTestMode currentMode() const { return current_mode_; }
  RobotTestMode previousMode() const { return previous_mode_; }
  Milliseconds lastModeChangeMs() const { return last_mode_change_ms_; }

  bool setMode(RobotTestMode mode, Milliseconds now_ms);
  void emergencyStop(Milliseconds now_ms);

  bool motorsMayBeCommanded() const;
  bool acceptsDriveCommand() const;
  bool acceptsSingleMotorCommand() const;
  bool acceptsLineFollowerCommand() const;

 private:
  RobotTestMode current_mode_{RobotTestMode::Disabled};
  RobotTestMode previous_mode_{RobotTestMode::Disabled};
  Milliseconds last_mode_change_ms_{0};
};

}  // namespace robot
