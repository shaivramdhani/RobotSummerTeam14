#pragma once

#include <cstdint>

#include "common/ChassisCommand.h"
#include "common/FaultHealth.h"

namespace robot::esp1 {

enum class MissionState : std::uint8_t {
  SafeStopped = 0,
  FollowTapeToSolarPanel = 1,
  DetectIrBeacon = 2,
  TraverseToSolarLimitSwitches = 3,
  InsertHook = 4,
  RemoveCover = 5,
  RejoinTapeLine = 6,
  NavigateToHabitatPieces = 7,
  CollectHabitatPiece = 8,
  PlaceHabitatPieceOnRing = 9,
  RepeatHabitatSequence = 10,
  NavigateToTowerPieces = 11,
  PickUpTowerPieces = 12,
  LocateTowerBuildMarkings = 13,
  CloseFunnel = 14,
  ReleaseTowerPiecesSequentially = 15,
  Complete = 16,
};

struct MissionInputs {
  HealthReport esp1_health{};
  HealthReport esp2_health{};
};

struct MissionOutputs {
  ChassisCommand chassis_command{};
  bool mechanism_command_requested{false};
};

class MissionStateMachine {
 public:
  MissionState state() const { return state_; }
  MissionOutputs update(const MissionInputs& inputs);
  void forceSafeStop(FaultCode fault);

 private:
  MissionState state_{MissionState::SafeStopped};
  FaultCode last_fault_{FaultCode::None};
};

}  // namespace robot::esp1
