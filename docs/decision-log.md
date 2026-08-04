# Decision Log

## 2026-07-01: Initial Repository Scaffold

Decision: Use one PlatformIO monorepo with environments `esp1`, `esp2`, and
`native`.

Reason: The robot has two cooperating ESP32-S3 processors with shared protocol
and pure logic that should remain tested in one place.

## 2026-07-01: ESP1 Owns Mission, ESP2 Owns Motion

Decision: ESP1 is the mission controller. ESP2 owns logical four-wheel motion
calculation and mechanisms.

Reason: This follows the current architecture decision and keeps front line
following close to the front line sensors.

## 2026-07-01: No GPIO Numbers Yet

Decision: Pin configuration files list signals but use `-1` TODO placeholders.

Reason: GPIO numbers, active levels, voltages, timing values, and PWM resources
are not confirmed.

## 2026-07-01: No Motor-Moving Code

Decision: Motor and chassis interfaces exist, but the only implemented mixer
returns disabled wheel commands.

Reason: The task is architecture and compile-safe stubs only.

## 2026-07-02: Digital Line Following Feature

Decision: Implement digital two-sensor line following on ESP2 with PID control,
differential-style four-wheel mixing, runtime serial tuning, telemetry, and
rear-wheel UART commands to ESP1.

Reason: The first physical milestone is following black electrical tape using
`LSFL` and `LSFR`.

Safety note: Hardware outputs remain disabled until GPIOs, PWM resources,
H-bridge mode, UART pins/baud, per-wheel direction signs, and maximum safe duty
are verified in `PinConfig.h`.

## 2026-07-27: Stage 3 IMU Heading-Held Manual Strafe

Decision: Add a dedicated `IMU_STRAFE_TEST` mode with a separately tuned PD
heading controller. Capture heading once at held-strafe start, combine bounded
lateral and yaw duties before the existing mecanum mixer, and retain the
existing output and IMU-acquisition ownership paths unchanged.

Reason: Explicit mode selection prevents IMU correction from affecting normal
manual drive, line-following PID, Stage 2 turns, or autonomous routines. The
combined-duty validation and existing browser deadman bound the new manual
motion.

## 2026-07-28: Autonomous IMU Motion Reuses Manual-Test Tuning

Decision: Route every Solar lateral state, the Tower initial strafe and
shimmy, and the optional Time Trial transition through the shared Stage 3
heading-hold controller. Route Tower and PegFinder clockwise turns through the
shared Stage 2 angle controller. Autonomous panels retain only strafe
durations/timeouts and turn angles; they do not own separate IMU motion gains
or duty limits.

Reason: One live tuning source prevents tested IMU values from drifting away
from autonomous behavior. Each owning mode still handles its own state
transition and faults, while stale IMU/link data stops all wheel outputs.

## 2026-07-30: ESP1 Owns the Habitat Distance Sensor

Decision: Put the VL53L0X V2 on ESP1 SDA GPIO10/SCL GPIO9, use the existing
ESP1 sensor-acquisition task as its sole owner, and transport complete immutable
snapshots to ESP2 in a dedicated UART message.

Reason: The IMU is physically and logically owned by ESP2, so there is no
cross-processor I2C bus to share. A local ESP1 bus keeps the distance sensor
close to the mission controller, isolates it from IMU traffic and failures,
and avoids expanding the already compact ESP1 status payload.

Safety note: The shared habitat distance-stop gate defaults locked and fails
safe on missing, invalid, or stale data. It is not connected to motion until
the currently stubbed habitat approach and its physical stop-distance and
freshness calibrations exist.

## 2026-07-31: Explicit Habitat Pieces Laser Approach Mode (superseded)

Decision: Add ESP2 `HABITAT_PIECES` mode. It reuses the front-sensor line
follower with an independent default duty of `0.12`, evaluates
`HabitatDistanceStop` before every wheel command, and latches all-wheel stop at
the configured laser distance. It also applies an adjustable overall run
timeout so an approach that never reaches the distance threshold cannot run
indefinitely.

Reason: The approach can be calibrated and exercised independently of the
still-stubbed ESP1 mission transitions while preserving ESP2's ownership of
four-wheel motion calculation.

Safety note: Stop distance, maximum sample age, and run timeout remain zero by
default. Start is locked until all three are nonzero and all line, motor, link,
and duty requirements pass. Fresh driver-error-free no-target measurements are
allowed so the robot can approach until the sensor acquires the piece. Missing,
unavailable, frozen, or driver-faulted laser data and an expired run timeout
stop the mode instead of allowing unbounded motion.

Profile behavior: ESP1 configures the Adafruit high-accuracy profile before
continuous ranging starts and remains the sole laser owner. HabitatPieces
confirms that profile over the expiring rear-wheel command before enabling
motion. Stop, fault, completion, mode exit, and command expiry all retain high
accuracy. Its 200 ms timing budget is paired with a 200 ms continuous
intermeasurement period.

## 2026-07-31: Habitat Pieces Stops on Delayed LSS2 Detection (superseded)

Decision: Replace the Habitat Pieces laser-distance stop gate with the
ESP1-owned LSS2 digital line sensor. Front-sensor line following begins
immediately. LSS2 is ignored for an explicitly configured nonzero delay; after
that delay, a fresh black level latches completion and all-wheel stop. The
existing overall run timeout remains and must be longer than the delay.

Reason: The intended field sequence is to drive past early side markings and
only begin watching for the Habitat Pieces stop line after a known travel
interval.

Safety note: No detection delay is invented. Start remains locked until the
delay and timeout are valid, LSS2 has a real GPIO configuration, its ESP1
sensor packet is fresh, and all existing front-line, motor, UART, and duty gates
pass. Stale/unconfigured LSS2 data, front line loss, rear-command failure, or
timeout stops all four wheels. The laser remains independent telemetry and is
not consulted by this mode.

## 2026-07-31: Habitat Pieces Reverses After Delayed LSS2 Detection (superseded)

Decision: Retain immediate front-sensor line following and the delayed LSS2
search, but transition a latched black detection to a new `REVERSING` state.
That state commands straight backward motion through the existing mecanum mixer
at an explicitly configured duty for an explicitly configured duration, then
latches `COMPLETE` and all-wheel stop. The existing timeout now bounds finding
LSS2; the reverse has its own duration.

Reason: The intended field sequence requires a repeatable backward displacement
after reaching the side line rather than stopping at the first detection.

Safety note: Reverse duty and duration default to zero and lock out Start until
explicitly configured. LSS2 must remain configured and fresh through detection.
Once detection is latched, LSS2 is no longer an input to the bounded reverse;
configured motors, fresh rear status, expiring rear commands, explicit Stop,
and mode-exit shutdown remain active throughout it.

## 2026-08-01: Habitat Pieces Independently Aligns LSS2 and LSS3

Decision: Keep the existing front line-follow, detection delay, overall timeout,
and bounded reverse settings. After the delay, either LSS2-left or LSS3-right
can initiate `SIDE_LINE_ALIGNING`. Each black detection latches independently:
LSS2 stops both left wheels and LSS3 stops both right wheels. The undetected
side continues forward at the configured line-follow duty. The route enters
`REVERSING` only after both latches are set, including when both sensors detect
black in the same update.

Reason: The robot must square both physical side sensors to the marking before
performing the already-configured backward move.

Safety note: A latch never clears during the run, so a stopped wheel side cannot
restart if its sensor subsequently reads white. Both GPIO configurations and
the shared sensor-packet freshness are required until both latch. The existing
`run_timeout_ms` bounds initial line search plus side alignment; timeout, stale
sensor data, hardware/link loss, or rear-command failure disables all wheels.
The subsequent reverse retains its independent configured duration and normal
command-expiry gates.

## 2026-08-01: Habitat Pieces Counts Laser Zones After Reverse

Decision: Extend the pickup route after `REVERSING` with a configurable
`DISTANCE_STRAFING` state. The robot strafes left or right at the configured
duty and counts rising entries into the valid distance zone strictly above the
configured millimetre threshold. Consecutive above-threshold measurements form
one count; a valid measurement at or below the threshold rearms the next count.
The target count completes and stops the route.

Reason: Distinct far-distance zones represent gaps between habitat pieces, and
measurement-sequence gating prevents a repeated UART heartbeat or frozen sample
from manufacturing extra gaps.

Safety note: Direction, threshold, target count, duty, and timeout all default
to unconfigured. Laser availability does not block route Start, and invalid,
stale, wrong-profile, or no-signal samples neither count nor rearm. The strafe
uses the normal expiring four-wheel command path and faults stopped when its
configured timeout expires before the target count is reached.

## 2026-08-01: Habitat Pieces Returns to LSS2 Stop and IMU Strafe

Decision: Supersede the independent LSS2/LSS3 alignment behavior above. After
the configured delay, LSS2 alone stops all four wheels before the bounded
reverse begins; LSS3 remains telemetry-only. Run the distance-zone strafe
through the shared IMU heading-hold controller. Keep the VL53L0X in its
normal/default profile. Represent a fresh N/A/no-target attempt as 65536 mm,
one above the maximum configurable uint16 threshold.

Reason: The pickup route now uses LSS2 as its single stopping reference and
needs yaw correction during lateral motion. Treating a fresh no-target result
as far distance lets the existing zone counter detect open gaps.

Safety note: Start now requires valid shared IMU heading-hold tuning and a
healthy IMU as well as LSS2. LSS2 detection produces a full stopped control
update before reverse. Stale or frozen laser data is not substituted and does
not create new counts; measurement-sequence gating and the adjustable strafe
timeout remain active. IMU loss uses the shared bounded pause/recovery path and
faults stopped if recovery fails.

## 2026-08-02: Habitat Pieces Counts Near Zones and Pulses to the Exit

Decision: Supersede the earlier far-zone counter. `DISTANCE_STRAFING` now
counts distinct entries at or below the configured threshold. On reaching the
target, stop for an adjustable delay, then repeat adjustable-duration strafing
pulses in the same direction and at the same duty. Stop all wheels after every
pulse and wait for a new measurement sequence. Complete only when that fresh
check is above threshold; otherwise repeat the pulse.

Reason: The near-distance zones identify the habitat pieces, while short
post-count pulses let the chassis move beyond the final detected piece without
making a distance decision while the robot is still moving.

Safety note: Both added timings default to unconfigured. The existing distance
strafe timeout bounds counting, stop delay, pulses, and sensor waits. A fresh
N/A result remains the 65536 mm sentinel and therefore satisfies the exit
check. Stale or repeated samples cannot satisfy a check, and each check occurs
with all four wheels commanded stopped.

## 2026-08-02: Habitat Pieces Pickup Tail and Placement Handoff

Decision: After the final above-threshold exit check, lower the linear slide to
its bottom switch, drive forward until the dedicated active-high GPIO48
habitat-piece limit switch is pressed, stop, and begin a configured
step-counted lift. Run that lift
through a configurable all-wheel-stop delay, then concurrently with a timed
reverse and an IMU-held strafe opposite the original distance-strafe
direction. Stop the chassis when either rear line sensor sees
black, wait stopped if the lift is unfinished, and then automatically request
the existing Habitat Placement route.

Safety note: Slide-down, limit-switch approach, lift, reverse, and rear-line
reacquisition settings default to unconfigured. Each search has an independent
timeout. Laser data does not control the forward approach; it remains confined
to the strafe/count/exit phases. Stepper command failure, conflicting limits,
stale rear-line data, IMU failure, or timeout stops the chassis and stepper.

## 2026-08-03: Restore High-Accuracy Laser at 200 ms

Decision: Supersede the 2026-08-01 normal-profile selection. Use the VL53L0X
high-accuracy profile with a 200 ms continuous intermeasurement period for
idle, stopped, and moving operation. Motor-command expiry still disables the
rear wheels, but no longer changes the retained laser profile.

Reason: The pickup route now prioritizes measurement quality over refresh
speed. A single shared operational-profile constant keeps ESP1 initialization,
ESP2 commands, and Habitat Pieces profile validation consistent.

## 2026-08-03: Habitat Pieces Pre-Lift Reverse and Return Duty

Decision: After the habitat-piece limit-switch approach stops, reverse for an independently
adjustable duration using the existing pickup reverse duty, stop again, and
then start the lift. Give the opposite-direction rear-line IMU strafe its own
adjustable duty instead of reusing the distance-counting strafe duty.

Safety note: Both new values default to unconfigured. The pre-lift motion is
duration-bounded, the rear-line strafe retains its independent timeout, and
each transition produces an all-wheel stopped update.

## 2026-08-03: Habitat Placement Post-CW Right Strafe

Decision: After the existing post-clockwise left strafe, add a timed right
strafe before the existing stopped delay and forward-exit step. Reuse the
post-clockwise strafe duty and give the new right strafe its own adjustable,
saved duration.

Safety note: The duration defaults to unconfigured, is limited to the shared
maximum autonomous timing, and therefore locks Habitat Placement Start until
set. Existing command-expiry and rear-link safety behavior remains active.

## 2026-08-03: IMU Heading Hold for Every Habitat Placement Strafe

Decision: Route the pre-CCW right strafe, post-CW left and right strafes, and
final right strafe to the front line through the shared IMU heading-hold
controller. Each strafe keeps its existing direction, duty, duration, or sensor
stop condition.

Safety note: Habitat Placement Start now validates every strafe duty against
the shared IMU correction reserve. Loss of fresh IMU data pauses and stops the
strafe during the bounded recovery window; recovery timeout or controller/link
failure latches a stopped autonomy fault.

## 2026-08-03: Alternating Three-Profile Habitat Cycle

Decision: Store three independently editable Habitat Pieces pickup profiles
and three matching placement profiles. One Habitat Pieces Start executes
Pickup 1, Placement 1, Pickup 2, Placement 2, Pickup 3, and Placement 3. Each
placement profile selects Front or Rear as its return-line source; migration
copies legacy settings into all three profiles and assigns Front, Front, Rear.
Each placement captures a fresh initial heading at its own start.

Safety note: Start validates every pickup and placement profile before motion.
Every pickup/placement handoff commands an all-wheel stop, and stale or
unavailable required line data, IMU/link failure, or any existing per-step
timeout faults the complete sequence stopped.

## 2026-08-03: Concurrent Placement Slide and Independent Exit-Pulse Duty

Decision: Start the Habitat Placement lower-limit search when
`FORWARD_TO_SLIDE` begins and keep it active through the following slide state.
Also give Habitat Pieces `EXIT_STRAFE_PULSE` its own adjustable duty instead of
reusing the long distance-counting strafe duty. Existing saved pickup settings
inherit the counting-strafe duty until the new value is saved.

Safety note: The slide timeout begins with the earlier concurrent command; an
unexpected stop or timeout before the bottom limit faults both mechanisms. The
new pulse duty is validated against the motor cap and IMU correction reserve,
and every pulse still ends with an all-wheel stop before checking distance.

## 2026-08-04: Habitat Pieces Sensor-Directed Side-Line Rotation

Decision: Supersede the LSS2-only Habitat Pieces stop behavior and the earlier
single-wheel side alignment. During pickup line following, either LSS2 or LSS3
stops all four wheels. LSS2-first selects clockwise chassis rotation until LSS3
sees black; LSS3-first selects counter-clockwise rotation until LSS2 sees
black. If both sensors see black together, skip rotation. Stop all wheels again
before continuing through the existing reverse, distance-strafe, and exit-pulse
steps.

Safety note: Rotation reuses the pickup profile's adjustable line-follow duty,
uses the chassis mixer's physical yaw convention, and remains bounded by the
pickup run timeout. Both side-sensor snapshots must remain configured and fresh until
alignment completes, and the transition into and out of rotation emits an
all-wheel stopped update.
