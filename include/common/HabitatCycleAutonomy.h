#pragma once

#include <cstdint>

namespace robot {

constexpr std::uint8_t kHabitatProfileCount = 3U;

enum class HabitatCyclePhase : std::uint8_t {
  Idle = 0,
  Pickup = 1,
  Placement = 2,
  Complete = 3,
  Fault = 4,
};

struct HabitatCycleAutonomy {
  HabitatCyclePhase phase{HabitatCyclePhase::Idle};
  std::uint8_t active_profile_index{0U};
  std::uint8_t completed_profile_count{0U};
};

const char* habitatCyclePhaseName(HabitatCyclePhase phase);
void resetHabitatCycleAutonomy(HabitatCycleAutonomy& autonomy);
void startHabitatCycleAutonomy(HabitatCycleAutonomy& autonomy);
bool advanceHabitatCycleAfterPickup(HabitatCycleAutonomy& autonomy);
bool advanceHabitatCycleAfterPlacement(HabitatCycleAutonomy& autonomy);
void failHabitatCycleAutonomy(HabitatCycleAutonomy& autonomy);
bool habitatCycleActive(const HabitatCycleAutonomy& autonomy);

}  // namespace robot
