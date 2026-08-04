#include "common/HabitatCycleAutonomy.h"

namespace robot {

const char* habitatCyclePhaseName(const HabitatCyclePhase phase) {
  switch (phase) {
    case HabitatCyclePhase::Idle:
      return "IDLE";
    case HabitatCyclePhase::Pickup:
      return "PICKUP";
    case HabitatCyclePhase::Placement:
      return "PLACEMENT";
    case HabitatCyclePhase::Complete:
      return "COMPLETE";
    case HabitatCyclePhase::Fault:
      return "FAULT";
  }
  return "IDLE";
}

void resetHabitatCycleAutonomy(HabitatCycleAutonomy& autonomy) {
  autonomy = {};
}

void startHabitatCycleAutonomy(HabitatCycleAutonomy& autonomy) {
  autonomy = {};
  autonomy.phase = HabitatCyclePhase::Pickup;
}

bool advanceHabitatCycleAfterPickup(HabitatCycleAutonomy& autonomy) {
  if (autonomy.phase != HabitatCyclePhase::Pickup ||
      autonomy.active_profile_index >= kHabitatProfileCount) {
    return false;
  }
  autonomy.phase = HabitatCyclePhase::Placement;
  return true;
}

bool advanceHabitatCycleAfterPlacement(HabitatCycleAutonomy& autonomy) {
  if (autonomy.phase != HabitatCyclePhase::Placement ||
      autonomy.active_profile_index >= kHabitatProfileCount) {
    return false;
  }
  autonomy.completed_profile_count = static_cast<std::uint8_t>(
      autonomy.active_profile_index + 1U);
  if (autonomy.completed_profile_count >= kHabitatProfileCount) {
    autonomy.phase = HabitatCyclePhase::Complete;
    return true;
  }
  ++autonomy.active_profile_index;
  autonomy.phase = HabitatCyclePhase::Pickup;
  return true;
}

void failHabitatCycleAutonomy(HabitatCycleAutonomy& autonomy) {
  if (habitatCycleActive(autonomy)) {
    autonomy.phase = HabitatCyclePhase::Fault;
  }
}

bool habitatCycleActive(const HabitatCycleAutonomy& autonomy) {
  return autonomy.phase == HabitatCyclePhase::Pickup ||
         autonomy.phase == HabitatCyclePhase::Placement;
}

}  // namespace robot
