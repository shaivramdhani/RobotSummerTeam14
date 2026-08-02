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
ignores LSS2-left and LSS3-right for an explicitly configured detection delay,
then latches either fresh black detection. A detected sensor stops both wheels
on its side while the other side continues forward until its own sensor also
latches black. It then drives straight backward at an explicitly configured
duty for an explicitly configured duration, then enters `DISTANCE_STRAFING`.
That final pickup step moves in the configured left/right direction and counts
distinct valid laser-measurement entries above the configured threshold.
Consecutive above-threshold values count once; a value at or below the
threshold rearms the next count. Reaching the configured count ends that
strafe and begins a timed strafe in the opposite direction to compensate for
overshoot. The chassis stops while the slide seeks its bottom limit, then
drives forward until a new valid high-accuracy laser measurement is at or below
the pickup threshold. A relative slide lift starts at that point and runs
concurrently with a timed reverse and an opposite-direction return strafe. The
return strafe stops when either rear line sensor sees black; if the lift is
still active, the chassis remains stopped until it completes. The route then
enters `COMPLETE`, ready for the separately started Habitat Placement route.
Each distance search, slide action, and open-loop motion has an independent
bound. A separate overall timeout stops the mode if both side lines are not
acquired. Both side sensors and both rear sensors
must be configured and their shared snapshot must remain fresh until both
latch or while the return-line strafe is active, respectively. The VL53L0X is
not a start gate; invalid/no-signal readings do not count, do not stop either
laser-driven motion, and leave the appropriate timeout in control. The separate ESP1
mission-state transition into
`NavigateToHabitatPieces` remains unimplemented.

ESP2 also exposes a standalone `HABITAT_PLACEMENT` route intended for the later
handoff from the unfinished pickup sequence. It captures the fresh continuous
IMU heading at route start, reverse-line-follows on the rear sensors until LSS1
is black, pauses, turns back to the captured initial heading, and performs a
timed right strafe. It then turns CCW to the captured heading plus or minus the
configured offset, drives forward, lowers the slide to its bottom limit, opens
the Habitat Pusher, pushes forward, retreats, performs a bounded IMU CW turn,
drives backward, strafes left, pauses, drives forward, pauses again, and
strafes right until either front line sensor is black before closing the
pusher. Each motion/search has an adjustable bound; configuration, stale
communication, sensor, turn, stepper, servo, and timeout failures stop the
chassis. Automatic pickup-to-placement chaining remains TODO until the pickup
route is complete.

## Transition TODOs

- Define entry and exit conditions for every state.
- Define which sensor snapshots each transition may read.
- Define timeout behavior for each state.
- Define recovery behavior after stale communication or conflicting limit
  switches.
- Define when mechanisms may be commanded and how commands are acknowledged.
- Calibrate the shared LSS2/LSS3 detection delay, search/alignment timeout, and
  reverse duty/duration in the dedicated ESP2 mode, then define how the ESP1
  mission state requests and observes that approach.
