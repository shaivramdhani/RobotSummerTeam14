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
ignores LSS2 for an explicitly configured detection delay, then stops all four
wheels when fresh LSS2 black is detected. LSS3 remains telemetry-only. On the
next control update it drives straight backward at an explicitly configured
duty for an explicitly configured duration, then enters `DISTANCE_STRAFING`.
That pickup step moves in the configured left/right direction through IMU
heading hold and counts distinct fresh laser-measurement entries at or below
the configured threshold. Consecutive in-zone values count once; an
above-threshold value rearms the next count. Reaching the configured count
enters `POST_COUNT_STOP_DELAY`. It then alternates `EXIT_STRAFE_PULSE` and
`EXIT_DISTANCE_CHECK`: each pulse is timed, every check occurs stopped and
requires a new measurement sequence, above threshold advances to
`LOWER_SLIDE`, and at-or-below repeats the pulse. The distance-strafe timeout
bounds the complete count/delay/pulse/check sequence. The slide then seeks its
bottom limit, `APPROACH_PIECE` drives forward until a new valid laser result is
at or below a second threshold, and the stopped transition starts a
step-counted lift. `LIFT_START_DELAY` holds every wheel stopped for its
adjustable duration while the lift continues. `REVERSE_AFTER_PICKUP` and
`REAR_LINE_REACQUIRE` then run while that lift continues. Rear-line
reacquisition uses IMU heading hold in the
opposite direction from the original distance strafe and stops when either
rear sensor sees black. If necessary, the chassis waits stopped for the lift;
once both conditions are complete, the route automatically requests
`HABITAT_PLACEMENT`. Every search and mechanism motion has an independent
timeout. A separate overall timeout stops the mode if LSS2 is not acquired.
LSS2 must be configured and its shared snapshot must remain fresh until it
latches. The VL53L0X is not a start gate; a fresh N/A/no-target result is
represented as 65536 mm, so it remains outside a counted near zone and
satisfies an exit check.
A stale/frozen stream supplies no new sample and does not stop the strafe
before its timeout. The separate ESP1
mission-state transition into
`NavigateToHabitatPieces` remains unimplemented.

ESP2 also exposes a standalone `HABITAT_PLACEMENT` route and automatically
hands off to it after the pickup lift and rear-line reacquisition. It captures the fresh continuous
IMU heading at route start, reverse-line-follows on the rear sensors until LSS1
is black, pauses, turns back to the captured initial heading, and performs a
timed right strafe. It then turns CCW to the captured heading plus or minus the
configured offset, drives forward, lowers the slide to its bottom limit, opens
the Habitat Pusher, pushes forward, retreats, performs a bounded IMU CW turn,
drives backward, strafes left, pauses, drives forward, pauses again, and
strafes right until either front line sensor is black before closing the
pusher. Each motion/search has an adjustable bound; configuration, stale
communication, sensor, turn, stepper, servo, and timeout failures stop the
chassis.

## Transition TODOs

- Define entry and exit conditions for every state.
- Define which sensor snapshots each transition may read.
- Define timeout behavior for each state.
- Define recovery behavior after stale communication or conflicting limit
  switches.
- Define when mechanisms may be commanded and how commands are acknowledged.
- Calibrate the LSS2 detection delay, LSS2 search timeout, reverse
  duty/duration, distance-zone strafe settings, and shared IMU heading-hold
  tuning in the dedicated ESP2 mode, then define how the ESP1 mission state
  requests and observes that approach.
