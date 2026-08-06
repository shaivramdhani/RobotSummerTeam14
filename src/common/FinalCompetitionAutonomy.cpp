#include "common/FinalCompetitionAutonomy.h"

namespace robot {

namespace {

void enterState(FinalCompetitionAutonomy& autonomy,
                const FinalCompetitionState state,
                const Milliseconds now_ms) {
  autonomy.state = state;
  autonomy.state_entered_at_ms = now_ms;
}

}  // namespace

const char* finalCompetitionStateName(const FinalCompetitionState state) {
  switch (state) {
    case FinalCompetitionState::WaitForStart:
      return "WAIT_FOR_START";
    case FinalCompetitionState::AutonomousSolar:
      return "AUTONOMOUS_SOLAR";
    case FinalCompetitionState::HabitatCycle:
      return "HABITAT_CYCLE";
    case FinalCompetitionState::TowerPieces:
      return "TOWER_PIECES";
    case FinalCompetitionState::PegFinder:
      return "PEG_FINDER";
    case FinalCompetitionState::Complete:
      return "COMPLETE";
    case FinalCompetitionState::Fault:
      return "FAULT";
  }
  return "WAIT_FOR_START";
}

void resetFinalCompetitionAutonomy(FinalCompetitionAutonomy& autonomy,
                                   const Milliseconds now_ms) {
  autonomy = {};
  autonomy.state_entered_at_ms = now_ms;
}

FinalCompetitionUpdate startFinalCompetitionAutonomy(
    FinalCompetitionAutonomy& autonomy, const Milliseconds now_ms) {
  enterState(autonomy, FinalCompetitionState::AutonomousSolar, now_ms);
  FinalCompetitionUpdate update{};
  update.state = autonomy.state;
  update.should_start_solar = true;
  return update;
}

void failFinalCompetitionAutonomy(FinalCompetitionAutonomy& autonomy,
                                  const Milliseconds now_ms) {
  enterState(autonomy, FinalCompetitionState::Fault, now_ms);
}

FinalCompetitionUpdate updateFinalCompetitionAutonomy(
    FinalCompetitionAutonomy& autonomy,
    const FinalCompetitionInputs& inputs, const Milliseconds now_ms) {
  FinalCompetitionUpdate update{};
  update.state = autonomy.state;

  switch (autonomy.state) {
    case FinalCompetitionState::AutonomousSolar:
      if (inputs.solar_fault) {
        failFinalCompetitionAutonomy(autonomy, now_ms);
      } else if (inputs.solar_complete) {
        enterState(autonomy, FinalCompetitionState::HabitatCycle, now_ms);
        update.should_start_habitat = true;
      }
      break;
    case FinalCompetitionState::HabitatCycle:
      if (inputs.habitat_fault) {
        failFinalCompetitionAutonomy(autonomy, now_ms);
      } else if (inputs.habitat_complete) {
        enterState(autonomy, FinalCompetitionState::TowerPieces, now_ms);
        update.should_start_tower = true;
      }
      break;
    case FinalCompetitionState::TowerPieces:
      if (inputs.tower_fault) {
        failFinalCompetitionAutonomy(autonomy, now_ms);
      } else if (inputs.tower_ready_for_peg_finder ||
                 inputs.tower_complete) {
        enterState(autonomy, FinalCompetitionState::PegFinder, now_ms);
        update.should_start_peg_finder = true;
      }
      break;
    case FinalCompetitionState::PegFinder:
      if (inputs.peg_finder_fault) {
        failFinalCompetitionAutonomy(autonomy, now_ms);
      } else if (inputs.peg_finder_complete) {
        enterState(autonomy, FinalCompetitionState::Complete, now_ms);
      }
      break;
    case FinalCompetitionState::WaitForStart:
    case FinalCompetitionState::Complete:
    case FinalCompetitionState::Fault:
      break;
  }

  update.state = autonomy.state;
  return update;
}

}  // namespace robot
