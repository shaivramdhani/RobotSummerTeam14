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

## 2026-08-01: Habitat Pickup Finishes at the Rear Placement Line

Decision: Extend Habitat Pieces after the distance-zone target with an
opposite-direction timed compensation strafe, slide homing to the bottom limit,
a forward laser-threshold approach, a relative slide lift, a timed reverse, and
an opposite-direction return strafe until either rear line sensor detects
black. Start the slide lift at the same time as the reverse and continue it
during the rear-line return; completion requires both the lift target and the
rear-line detection. Do not automatically start Habitat Placement.

Reason: The compensation corrects predictable strafe overshoot, the laser
approach positions the pickup, and the final rear-line alignment leaves the
robot at the expected starting condition for the separately controlled
placement route. The existing nonblocking stepper driver permits mechanism and
chassis progress in one periodic motion task without a blocking wait.

Safety note: Every new duty, duration, speed, step count, distance threshold,
and timeout defaults to unconfigured. The forward approach accepts only a new
valid high-accuracy measurement after the step begins; invalid, repeated,
stale, and no-signal samples keep the bounded approach moving. Slide command or
limit failures, stale/unavailable rear-line data during return, chassis command
failure, and any timeout stop the chassis and stepper.
