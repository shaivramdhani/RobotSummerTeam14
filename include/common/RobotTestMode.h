#pragma once

#include <cstdint>

namespace robot {

enum class RobotTestMode : std::uint8_t {
  Disabled = 0,
  SensorMonitor = 1,
  SingleMotorTest = 2,
  ManualDriveTest = 3,
  DistributedDriveTest = 4,
  LineSensorTest = 5,
  LineFollowTest = 6,
  MechanismTest = 7,
  AutonomousDryRun = 8,
  AutonomousSolarPanel = 9,
};

const char* robotTestModeName(RobotTestMode mode);
bool parseRobotTestMode(const char* text, RobotTestMode& mode);
bool robotTestModeAllowsMotion(RobotTestMode mode);
bool robotTestModeRequiresRearLink(RobotTestMode mode);
bool robotTestModeIsSensorOnly(RobotTestMode mode);

}  // namespace robot
