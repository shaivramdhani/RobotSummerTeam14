# Mission State Machine

ESP1 owns the mission-level autonomous state machine. The scaffold defines state
names but does not implement transitions or behavior.

## States

1. `SafeStopped`
2. `FollowTapeToSolarPanel`
3. `DetectIrBeacon`
4. `TraverseToSolarLimitSwitches`
5. `InsertHook`
6. `RemoveCover`
7. `RejoinTapeLine`
8. `NavigateToHabitatPieces`
9. `CollectHabitatPiece`
10. `PlaceHabitatPieceOnRing`
11. `RepeatHabitatSequence`
12. `NavigateToTowerPieces`
13. `PickUpTowerPieces`
14. `LocateTowerBuildMarkings`
15. `CloseFunnel`
16. `ReleaseTowerPiecesSequentially`
17. `Complete`

## Current Behavior

The scaffold remains in `SafeStopped` and emits disabled chassis commands. Faults
force `SafeStopped`.

## Transition TODOs

- Define entry and exit conditions for every state.
- Define which sensor snapshots each transition may read.
- Define timeout behavior for each state.
- Define recovery behavior after stale communication or conflicting limit
  switches.
- Define when mechanisms may be commanded and how commands are acknowledged.
