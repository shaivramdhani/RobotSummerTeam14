#include "esp1/MissionStateMachine.h"

namespace robot::esp1 {

MissionOutputs MissionStateMachine::update(const MissionInputs& inputs) {
  if (hasFault(inputs.esp1_health) || hasFault(inputs.esp2_health)) {
    forceSafeStop(hasFault(inputs.esp1_health) ? inputs.esp1_health.active_fault
                                               : inputs.esp2_health.active_fault);
  }

  MissionOutputs outputs{};
  outputs.chassis_command = disabledChassisCommand();
  outputs.mechanism_command_requested = false;
  return outputs;
}

void MissionStateMachine::forceSafeStop(const FaultCode fault) {
  state_ = MissionState::SafeStopped;
  last_fault_ = fault;
}

}  // namespace robot::esp1
