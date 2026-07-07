#include "common/RobotTestModeManager.h"

namespace robot {

bool RobotTestModeManager::setMode(const RobotTestMode mode,
                                   const Milliseconds now_ms) {
  if (mode == current_mode_) {
    last_mode_change_ms_ = now_ms;
    return true;
  }

  previous_mode_ = current_mode_;
  current_mode_ = mode;
  last_mode_change_ms_ = now_ms;
  return true;
}

void RobotTestModeManager::emergencyStop(const Milliseconds now_ms) {
  setMode(RobotTestMode::Disabled, now_ms);
}

bool RobotTestModeManager::motorsMayBeCommanded() const {
  return robotTestModeAllowsMotion(current_mode_);
}

bool RobotTestModeManager::acceptsDriveCommand() const {
  return current_mode_ == RobotTestMode::ManualDriveTest ||
         current_mode_ == RobotTestMode::DistributedDriveTest;
}

bool RobotTestModeManager::acceptsSingleMotorCommand() const {
  return current_mode_ == RobotTestMode::SingleMotorTest;
}

bool RobotTestModeManager::acceptsLineFollowerCommand() const {
  return current_mode_ == RobotTestMode::LineFollowTest;
}

}  // namespace robot
