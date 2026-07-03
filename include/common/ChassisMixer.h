#pragma once

#include "common/ChassisCommand.h"
#include "common/LineFollower.h"
#include "common/MotorOutput.h"

namespace robot {

FourWheelCommand calculateDisabledFourWheelCommand(
    const ChassisCommand& command);
FourWheelCommand mixDifferentialLineFollow(float correction,
                                           const LineFollowerConfig& config,
                                           Milliseconds now_ms);

}  // namespace robot
