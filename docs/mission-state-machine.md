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

ESP2's explicit `HABITAT_PIECES` mode line-follows with the front sensors,
ignores LSS2 for an explicitly configured detection delay, and then records
the black-line detection when the ESP1-owned LSS2 reports black. It then drives
straight backward at an explicitly configured duty for an explicitly
configured duration and latches stop. A separate search timeout stops the mode
if the line is never detected. LSS2 must be configured and its sensor snapshots
must remain fresh through detection. The VL53L0X is telemetry-only for this
mode. The separate ESP1 mission-state transition into
`NavigateToHabitatPieces` remains unimplemented.

ESP2 also exposes a standalone `HABITAT_PLACEMENT` route intended for the later
handoff from the unfinished pickup sequence. It reverse-line-follows on the
rear sensors until LSS1 is black, pauses, performs a bounded IMU CCW turn,
drives forward, lowers the slide to its bottom limit, opens the Habitat Pusher,
pushes forward, retreats, performs a bounded IMU CW turn, pauses, drives
forward, pauses again, and strafes right until either front line sensor is
black before closing the pusher. Each motion/search has an adjustable bound;
configuration, stale communication, sensor, turn, stepper, servo, and timeout
failures stop the chassis. Automatic pickup-to-placement chaining remains TODO
until the pickup route is complete.

## Transition TODOs

- Define entry and exit conditions for every state.
- Define which sensor snapshots each transition may read.
- Define timeout behavior for each state.
- Define recovery behavior after stale communication or conflicting limit
  switches.
- Define when mechanisms may be commanded and how commands are acknowledged.
- Calibrate the LSS2 detection delay/search timeout and reverse duty/duration in
  the dedicated ESP2 mode, then define how the ESP1 mission state requests and
  observes that approach.
