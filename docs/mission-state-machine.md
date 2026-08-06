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
ignores LSS2/LSS3 HIGH readings for an explicitly configured start window,
then stops all four wheels when either fresh sensor reads black. LSS2 enters
`SIDE_LINE_ALIGNING` clockwise until LSS3 sees black; LSS3 enters it
counter-clockwise until LSS2 sees black. The alignment rotation uses the
profile's line-follow duty and the chassis mixer's physical yaw convention. Detecting the
opposite sensor stops all four wheels for one update before `REVERSING` drives
straight backward at an explicitly configured duty and duration, then enters
`CLOSE_PUSHER_BEFORE_DISTANCE_STRAFE` before `DISTANCE_STRAFING`.
That pickup step moves in the configured left/right direction through IMU
heading hold and counts distinct fresh laser-measurement entries at or below
the configured threshold. After the timed reverse, the chassis stops and ESP2
commands the Habitat Pusher closed before entering the distance strafe. Its
per-profile initial count-ignore window keeps the
count at zero while still tracking zone occupancy; a target held near through
the window must leave and re-enter before counting. Consecutive in-zone values count once; an
above-threshold value rearms the next count. Reaching the configured count
enters `POST_COUNT_STOP_DELAY`. It then alternates `EXIT_STRAFE_PULSE` and
`EXIT_DISTANCE_CHECK`: each pulse is timed, every check occurs stopped and
requires a new measurement sequence, above threshold advances to
`APPROACH_PIECE` when the slide is already down (or to `LOWER_SLIDE` as a
stopped wait), and at-or-below repeats the pulse. The distance-strafe timeout
bounds the counting strafe. Expiry before the target count stops that motion,
records `DISTANCE_STRAFE_TIMEOUT`, enters `POST_COUNT_STOP_DELAY`, and gives the
normal exit pulse/check sequence a fresh timeout window. If that exit sequence
also exhausts the bound, it advances to `APPROACH_PIECE` if the slide is down or
the stopped `LOWER_SLIDE` wait otherwise. The slide has been
seeking its bottom limit concurrently since the pickup profile began;
`APPROACH_PIECE` drives forward until the active-high ESP2 GPIO48
habitat-piece limit switch is pressed. If the approach times out first, it
records `APPROACH_LIMIT_TIMEOUT` without faulting the autonomous run. Both the
switch and timeout paths stop the forward command for one update, then start a
bounded `PRE_LIFT_REVERSE` using the pickup reverse duty. Its stopped completion starts
the step-counted lift. `LIFT_START_DELAY` holds every wheel stopped for its
adjustable duration while the lift continues. `REVERSE_AFTER_PICKUP` and
`REAR_LINE_REACQUIRE` then run while that lift continues. Rear-line
reacquisition uses its independently adjustable duty with IMU heading hold in the
opposite direction from the original distance strafe and stops when either
rear sensor sees black. If necessary, the chassis waits stopped for the lift;
once both conditions are complete, the route automatically requests
`HABITAT_PLACEMENT`. Every search and mechanism motion has an independent
timeout; the distance-strafe and approach timeouts are recoverable pickup-tail
fallbacks. A separate overall timeout stops the mode if the first side line or
the opposite alignment line is not acquired. Both side sensors must be
configured and their shared snapshot must remain fresh until both latch. The
VL53L0X is not a start gate; a fresh N/A/no-target result is
represented as 65536 mm, so it remains outside a counted near zone and
satisfies an exit check.
A stale/frozen stream supplies no new sample and does not stop the strafe
before its timeout. The separate ESP1
mission-state transition into
`NavigateToHabitatPieces` remains unimplemented.

ESP2 also exposes a standalone `HABITAT_PLACEMENT` route and automatically
hands off to it after each pickup lift and rear-line reacquisition. A Habitat
Pieces Start runs Pickup 1, Placement 1, Pickup 2, Placement 2, Pickup 3, and
Placement 3. Each placement captures a fresh continuous IMU heading at its own
start. It reverse-line-follows on the rear sensors until LSS1
is black, pauses, turns back to the captured
initial heading, and performs a timed right strafe. It then turns CCW to the
captured heading plus or minus the configured offset, starts lowering the slide
while driving forward, waits at the bottom limit if the drive finishes first,
opens the Habitat Pusher, pushes forward, retreats, performs a bounded IMU CW
turn, drives backward, strafes left, strafes right, pauses, drives forward,
pauses again, and performs the return strafe before closing the pusher. The
return source is selectable per matching placement profile; migrated defaults
are Front, Front, Rear. The pre-CCW, post-CW left/right, and return strafes all
use shared IMU heading hold. Every pickup/placement transition commands an
all-wheel stop. In `FINAL_COMPETITION`, Placement 3 must use the rear return
source; completion starts Tower Pieces backward line following, then PegFinder,
without changing the parent mode. Each
motion/search has an adjustable bound; configuration, stale
communication, sensor, turn, stepper, servo, and timeout failures stop the
chassis.

Final Competition is staged with the slider up, funnel closed, and Solar Hook
down/closed. Outputs remain disabled at boot. Solar commands the hook closed
when its route starts. At the final left line-return strafe it starts the
bounded funnel opening while leaving the hook closed. Finding either front
line sensor stops the strafe, commands the hook open, and advances immediately
into the forward-line exit; the funnel finishes on its independent timer.
Habitat Pickup 1 starts only after both the route and funnel action finish,
with the hook servo still held open for the Habitat and Tower stages.

## Transition TODOs

- Define entry and exit conditions for every state.
- Define which sensor snapshots each transition may read.
- Define timeout behavior for each state.
- Define recovery behavior after stale communication or conflicting limit
  switches.
- Define when mechanisms may be commanded and how commands are acknowledged.
- Calibrate the LSS2/LSS3 start-ignore window, side-line search/alignment timeout, reverse
  duty/duration, distance-zone strafe settings, and shared IMU heading-hold
  tuning in the dedicated ESP2 mode, then define how the ESP1 mission state
  requests and observes that approach.
