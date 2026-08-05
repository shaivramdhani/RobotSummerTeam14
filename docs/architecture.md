# Architecture

The robot uses two ESP32-S3 processors in one PlatformIO monorepo.

ESP1 is the mission controller. It owns the autonomous mission state machine,
mission-level safety decisions, ESP1-local sensors, rear wheel outputs, the
funnel motor, Solar Hook servo, and one side of the UART link.

ESP2 is the motion controller and mechanism node. It owns front line sensing,
logical four-wheel motion calculation, front wheel outputs, the stepper, the
remaining servo mechanisms, mechanism limit switches, the I2C IMU, and one
side of the UART link.

## Command Flow

ESP1 sends mission and chassis-motion commands to ESP2. ESP2 computes logical
four-wheel commands, applies front wheel commands locally, and sends rear wheel
commands back to ESP1. ESP1 applies only the rear wheel commands it owns.

ESP2 now implements the first functional motion feature: digital two-sensor line
following. The feature remains hardware-gated; motor adapters refuse to drive
until the owning ESP's pin, PWM, UART, H-bridge, direction, and safe-duty
configuration is complete.

## Task Model

Use a small number of periodic or event-driven FreeRTOS tasks:

- ESP1 mission task: mission state evaluation, command publication, health
  monitoring.
- ESP1 sensor-acquisition task on core 0: sole owner of the ESP1 `Wire` bus and
  VL53L0X on SDA GPIO10/SCL GPIO9. It polls measurement readiness without
  waiting for a range to complete only when Habitat requests acquisition,
  samples the IR inputs only when Solar requests acquisition, and
  publishes the newest complete `LaserDistanceSnapshot` through a static
  one-element queue.
- ESP2 motion task on core 1: line sensor acquisition, pure line observation,
  chassis control, mechanism coordination, and consumption of the latest IMU
  snapshot.
- ESP2 sensor-acquisition task on core 0: sole runtime owner of the I2C IMU.
  It publishes the newest complete `ImuAcquisitionSnapshot` through a static,
  one-element FreeRTOS queue. A blocked I2C transaction therefore cannot block
  the motion task.
- UART handling may be integrated into those tasks or split into one clearly
  bounded communication task per processor after timing requirements are known.

Do not create one task per sensor. The ESP2 sensor-acquisition task is the one
shared acquisition path for work that must be isolated from control; add future
bounded sensor work to that service instead of adding a task for each device.
Sensor acquisition produces immutable snapshots that downstream logic consumes.

ESP1 initializes the VL53L0X with the library's high-accuracy sensor profile
but leaves ranging stopped. Habitat Pickup and Placement request continuous
timed ranging with a 200 ms intermeasurement period, a 10 ms nonblocking service
period, a 100 kHz bus, and a 5 ms I2C transaction timeout. Solar similarly
requests IR acquisition only for its active route. Both acquisition requests
expire with a stale ESP2 command. A new laser result is sent
to ESP2 in a dedicated CRC-protected UART frame; an unchanged startup or fault
snapshot is resent every 100 ms as a heartbeat. ESP2 measures freshness from
arrival of a new measurement sequence, not from receipt of that heartbeat.

The current ESP1 mission state machine still emits only disabled chassis
commands. ESP2's explicit `HABITAT_PIECES` mode line-follows with the front
sensors at a separately adjustable duty. It ignores HIGH readings from the
ESP1-owned LSS2 and LSS3 inputs for a locked-by-default
`side_line_ignore_after_start_ms`. Either input may latch after the gate and
immediately stops all four wheels. LSS2 selects clockwise rotation until LSS3
sees black; LSS3 selects counter-clockwise rotation until LSS2 sees black. The
rotation uses the profile's line-follow duty and the chassis mixer's physical
clockwise/counter-clockwise yaw convention. A second all-wheel stop separates alignment from the
straight-backward open-loop mecanum command at configured
`reverse_duty` for `reverse_duration_ms`. The next state strafes left or right
at configured duty through the shared IMU heading-hold controller while
consuming fresh high-accuracy laser attempts with a new measurement sequence.
A per-profile `distance_count_ignore_ms` gate holds the count at zero after
distance strafing begins. Readings still update the zone latch during the gate,
so a target held near must leave and re-enter after the gate to count.
A fresh N/A/no-target result is represented as 65536 mm, one above every
configurable threshold. A new reading at or below `distance_threshold_mm`
increments the zone count when the preceding reading was above; consecutive
in-zone samples count once, and an above-threshold sample rearms the next
entry. Reaching `distance_zone_target_count` stops every wheel for
`post_count_stop_delay_ms`. The controller then alternates
`exit_strafe_pulse_ms` IMU-held strafe pulses at the independent
`exit_strafe_duty`, with stopped waits for a new measurement sequence. The
bottom-limit slide search starts with each pickup profile and runs concurrently
with the line, reverse, and distance route. A fresh above-threshold check
advances directly to the pickup tail when the slide is already down, or to a
stopped wait for that same bounded search; at-or-below repeats the pulse. The pickup tail then performs a
bounded GPIO48 limit-switch approach, a timed pre-lift reverse, and a step-counted
lift during a stopped adjustable delay. It then runs the lift concurrently
with a timed reverse and IMU-strafes at an independently adjustable duty
opposite the original direction until either rear sensor detects tape. Once
the lift is also complete, ESP2 requests the
existing Habitat Placement route. `distance_strafe_timeout_ms` bounds all of these
distance-strafe phases even if the stream becomes unavailable.
`run_timeout_ms` bounds both the first side-line search and the opposite-sensor
alignment, and must be longer than the detection delay. LSS2/LSS3 configuration
and snapshot freshness are required until both latch. The reverse remains bounded
by its own duration and the normal motor/link command-expiry gates.
The Habitat cycle coordinator stores three pickup profiles and three matching
placement profiles. One Habitat Pieces Start alternates Pickup 1, Placement 1,
Pickup 2, Placement 2, Pickup 3, and Placement 3. Every placement captures a
fresh initial heading when that placement begins. Its return line source is a
per-profile Front/Rear setting; migrated defaults are Front, Front, Rear. The
slide lower-limit search begins with, and runs concurrently through, each
forward-to-slide drive; its timeout is measured from that earlier start. The
third placement ends stopped on the rear line. `FINAL_COMPETITION` then starts
Tower Pieces in reverse line-following without changing the parent mode.

Competition setup assumes the slider is physically up, the funnel is
physically closed, and the Solar Hook is down/closed. Actuator outputs still
initialize disabled. When Solar starts, ESP2 commands the adjustable hook
closed angle. After Solar's forward line-follow exit, it commands the
adjustable open angle and refreshes a signed, timeout-protected funnel command
for the configured duration before declaring Solar complete and handing off.

The VL53L0X operates in its high-accuracy profile. It is not a start gate.
Fresh N/A/no-target results participate as the large-distance sentinel; an
unavailable or frozen measurement stream does not immediately block or stop
the pickup route, and the bounded strafe timeout remains the terminal safety
gate.

ESP2 initializes and calibrates the IMU during disabled startup, then transfers
all runtime updates and heading-reset commands to the sensor-acquisition
service. The motion task never calls `Wire` or mutates the IMU. Before startup
bias calibration, ESP2 initializes its local outputs disabled and sends
explicit disabled rear-wheel and funnel commands to ESP1, including a short
resend after ESP1 has had time to service its UART.

The read-only IMU soak test is an always-available telemetry view, not a robot
motion mode. The acquisition service keeps its existing 10 ms period and owns
fixed-size success, failure, timing, and liveness counters. The web task can
only enqueue a reset on a separate static one-element queue. It cannot access
`Wire`, reinitialize the device, reset heading, start a turn, change robot
mode, or issue a wheel command. Heading resets retain their separate queue and
existing acknowledgement path.

Stage 2 adds a shared IMU turn controller plus the manual `IMU_TURN_TEST` mode. Its
hardware-independent PD controller captures one continuous relative-heading
target, applies yaw-rate damping, and requires both angle and rate to remain
inside their configured limits for the full settling time. Tower Pieces and
PegFinder supply their mode-specific clockwise angles to this same controller;
all output limits, gains, tolerances, settling time, timeout, and polarity stay
owned by the shared IMU Turn configuration. Resulting wheel commands pass
through `applyWheelCommand()` unchanged.

Stage 3 adds a shared heading-hold controller plus the manual
`IMU_STRAFE_TEST` mode. Its
hardware-independent PD heading controller captures the current continuous
heading once when a held left/right strafe begins. The motion task combines
the configured lateral duty with a limited yaw correction before calling the
existing mecanum mixer, then passes the resulting `FourWheelCommand` through
`applyWheelCommand()` unchanged. Solar lateral stages, the Tower initial
strafe and shimmy, and the optional Time Trial transition use the same live
Stage 3 configuration while retaining their mode-specific durations/timeouts.
A Tower shimmy has stopped pre/post delays, starts in its configured direction,
and shortens only its first directional pulse to half the normal duration.
Tower also owns one finite, relative upward stepper jog launched with the first
rear-line-follow update. The stepper ISR advances that jog concurrently with
the mission state machine, while target tracking prevents the later
bottom-limit search from replacing an unfinished jog.
A dedicated browser heartbeat expires manual held tests after the existing
command timeout. Autonomous uses state-machine durations and the normal
communication expiry path.

## Shared Code

Hardware-independent types and pure logic live in `include/common` and
`src/common`. GPIO, PWM, ADC, UART peripheral setup, and board-specific details
must stay outside mission logic and native tests.
