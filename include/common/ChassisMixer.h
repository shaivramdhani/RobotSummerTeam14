#pragma once

#include "common/ChassisCommand.h"
#include "common/LineFollower.h"
#include "common/MotorOutput.h"

namespace robot {

FourWheelCommand calculateDisabledFourWheelCommand(
    const ChassisCommand& command);
FourWheelCommand mixOpenLoopMecanum(float lateral_command,
                                    float forward_command,
                                    float yaw_command, float duty,
                                    Milliseconds now_ms,
                                    Milliseconds timeout_ms);
FourWheelCommand mixDifferentialLineFollow(float correction,
                                           const LineFollowerConfig& config,
                                           Milliseconds now_ms);

}  // namespace robot
