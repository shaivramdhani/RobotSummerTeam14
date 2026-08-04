#pragma once

#include <cstdint>

#include "common/Units.h"

namespace robot {

enum class FinalCompetitionState : std::uint8_t {
  WaitForStart = 0,
  AutonomousSolar = 1,
  HabitatCycle = 2,
  TowerPieces = 3,
  PegFinder = 4,
  Complete = 5,
  Fault = 6,
};

struct FinalCompetitionInputs {
  bool solar_complete{false};
  bool solar_fault{false};
  bool habitat_complete{false};
  bool habitat_fault{false};
  bool tower_complete{false};
  bool tower_fault{false};
  bool peg_finder_complete{false};
  bool peg_finder_fault{false};
};

struct FinalCompetitionAutonomy {
  FinalCompetitionState state{FinalCompetitionState::WaitForStart};
  Milliseconds state_entered_at_ms{0U};
};

struct FinalCompetitionUpdate {
  FinalCompetitionState state{FinalCompetitionState::WaitForStart};
  bool should_start_solar{false};
  bool should_start_habitat{false};
  bool should_start_tower{false};
  bool should_start_peg_finder{false};
};

const char* finalCompetitionStateName(FinalCompetitionState state);
void resetFinalCompetitionAutonomy(FinalCompetitionAutonomy& autonomy,
                                   Milliseconds now_ms);
FinalCompetitionUpdate startFinalCompetitionAutonomy(
    FinalCompetitionAutonomy& autonomy, Milliseconds now_ms);
void failFinalCompetitionAutonomy(FinalCompetitionAutonomy& autonomy,
                                  Milliseconds now_ms);
FinalCompetitionUpdate updateFinalCompetitionAutonomy(
    FinalCompetitionAutonomy& autonomy,
    const FinalCompetitionInputs& inputs, Milliseconds now_ms);

}  // namespace robot
