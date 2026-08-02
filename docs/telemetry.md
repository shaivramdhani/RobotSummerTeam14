# Telemetry And Robot Test Interface

ESP2 owns the first telemetry dashboard. For the current drive-test wiring,
ESP2 drives the physical front motors locally and sends physical back-motor
commands to ESP1 over UART. See `docs/drive-test.md` for the exact wheel map.

This interface is for bench testing only. It starts in `DISABLED`, initializes
actuators disabled, and refuses motion until a test mode is explicitly
selected.

## WiFi Dashboard

ESP2 starts a WiFi softAP:

| Setting | Value |
| --- | --- |
| SSID | `Team14Robot` |
| Password | `robotdebug` |
| URL | `http://192.168.4.1/` |

After flashing ESP2, connect a laptop or phone to the AP and open the URL above.
The page uses polling JSON endpoints. WebSocket/SSE is not required.
The top of the dashboard includes an `IR Beacon` panel showing ESP1 GPIO7
frequency-selective telemetry when ESP1 status packets are fresh. ESP1 GPIO2
selects the target beacon frequency: `HIGH` selects 1 kHz and `LOW` selects
10 kHz.

The `Ultrasonic 1` panel reports the HC-SR04 as unconfigured because its former
GPIO11/GPIO12 pair now belongs to LSS2/LSS3. The acquisition code remains
available for two future non-conflicting pins.

The `Laser distance` panel shows ESP1's VL53L0X V2 reading in millimetres and
centimetres. It reports configuration, initialization, ranging, validity and
freshness; the active `HIGH_ACCURACY` profile; sensor and driver statuses;
measurement and UART sequences; success
and failure counters; acquisition timing; and the configured GPIOs, address,
and intermeasurement period. The sensor uses ESP1 SDA GPIO10/SCL GPIO9 and a
dedicated processor-local I2C bus.

The `Habitat Pieces` panel owns the explicit `HABITAT_PIECES` mode. It exposes
an independent front line-follow duty (default `0.12`), an adjustable LSS2/LSS3
detection delay, overall search/alignment timeout, reverse duty, reverse
duration, distance-strafe direction/threshold/target count/duty/timeout, state,
both sensor inputs/latches, per-side drive state, live distance/count state,
opposite compensation duty/duration, slide-down speed/timeout, forward pickup
duty/distance/timeout, slide-lift steps/speed/timeout, post-pickup reverse
duty/duration, return-strafe duty/timeout, rear-line state, and all associated
elapsed/remaining times. Apply validates the values; Save stores them in
ESP2 preferences. The existing `lss2_detection_delay_ms` setting arms both side
sensors. All motion settings other than the `0.12` front line-follow default
start unconfigured. The delay must be nonzero and shorter than the overall
search/alignment timeout; the distance-strafe timeout is capped at 30000 ms.

On Start, front-sensor line following begins immediately. LSS2 and LSS3 are
ignored until `lss2_detection_delay_ms` elapses. Detection then arms both
sensors. The first fresh black sensor transitions to `SIDE_LINE_ALIGNING` and
latches its side: LSS2 is physically left and disables both left wheels; LSS3
is physically right and disables both right wheels. The other side continues
forward at `line_follow_duty` until its sensor also latches black. Latches do
not clear if a stopped sensor later reads white. When both are latched, a
straight-backward mecanum command runs at `reverse_duty` for
`reverse_duration_ms`; it then enters `DISTANCE_STRAFING`. The configured
left/right strafe continues while new valid laser measurements are compared
with `distance_threshold_mm`. Each transition from at-or-below to above the
threshold increments `distance_zone_count`; consecutive above-threshold values
count once, and repeated UART snapshots with the same measurement sequence do
not count again. Reaching `distance_zone_target_count` enters
`COMPENSATION_STRAFING`, which moves opposite the initial strafe for the
configured duration. `LOWERING_SLIDE` then holds the chassis stopped while the
linear slide seeks its bottom limit. `FORWARD_TO_DISTANCE` drives forward until
a new valid measurement is at or below `forward_stop_distance_mm`.
Invalid, stale, wrong-profile, repeated, and no-signal readings do
not increment or rearm the count, but do not immediately stop the robot; the
configured `distance_strafe_timeout_ms` faults and stops the strafe if the
target is not reached. The forward approach uses the same new-measurement gate:
unavailable data leaves it driving until its separate timeout.

At the forward threshold, the relative slide lift starts non-blockingly while
`POST_PICKUP_REVERSING` drives backward for its configured duration. The lift
continues during `RETURN_LINE_STRAFING`, which moves opposite the initial
distance-count direction until either LSBL or LSBR detects black. If the rear
line arrives first, `WAIT_FOR_SLIDE_LIFT` keeps all wheels stopped until the
lift completes; if the lift finishes first, the return strafe continues. Both
conditions are required before `COMPLETE`. Completion does not automatically
start Habitat Placement.

The mode is rejected unless both side and rear sensors, slide hardware, and
limits are configured. Missing/stale side-sensor data
before both latch, front line loss before alignment, rear-command failure, or
expiration of `run_timeout_ms` enters `FAULT` and stops all wheels. During the
bounded reverse and distance strafe, the side sensors are no longer required;
fresh rear-line data becomes mandatory only during the return strafe. Slide
failure, conflicting limits, or any new timeout stops the slide and all wheels.
Motor/link gates and expiring rear commands remain active throughout. The
VL53L0X is not a Habitat Pieces Start gate, so the approach can begin without a
reading.

The `IMU` panel shows ESP2's MPU-6050-compatible sensor state. Runtime I2C reads
run in the core-0 sensor-acquisition task and are published to the core-1 motion
task as immutable snapshots. The `imu` JSON object includes configuration,
initialization, calibration, health, freshness, whether the acquisition task
is running, current snapshot age, last completed and maximum completed
acquisition durations, total acquisition attempts, successful and failed
reads, consecutive failures, the last successful-read timestamp, and
device-acknowledgement and runtime-register-verification flags; SDA/SCL GPIOs;
numeric I2C address and
`WHO_AM_I`; the exact
initialization failure stage and last raw Arduino `Wire` status; raw gyro Z;
bias-corrected yaw rate; continuous relative heading; selected repeated-start
or stop/start register-read mode; and sample timing. A rising
snapshot age with a running acquisition task indicates that acquisition is
delayed or blocked. The dashboard renders the address and identity in
hexadecimal. The driver probes the standard repeated-start read first during
startup and falls back once to a stop/start read for compatible modules; the
selected mode is then fixed. The current software assignment is SDA GPIO18 and
SCL GPIO17; both still require physical PCB verification.

Disconnect diagnostics report the current specific reason, the retained last
disconnect reason, disconnect count and timestamp, and the last low-level read
failure reason and timestamp. Reasons distinguish initialization stages,
acquisition-task loss, missing or stale samples, Arduino `Wire` buffer/NACK/bus
errors/timeouts, incomplete reads, runtime-register mismatches, and invalid
yaw or heading values. Each new disconnect or change in disconnect reason is
also written to the serial log with sample/snapshot ages and lifetime read
counts.

The nested `imu.acquisition_timing` object and matching dashboard rows separate
the acquisition path into current and lifetime-maximum timings:

- acquisition-loop start interval, which exposes task scheduling/preemption;
- zero-timeout command/reset queue synchronization work;
- `Wire.beginTransmission()` duration, which includes waiting for Wire's
  internal mutex;
- the complete 14-byte measurement-register read;
- successful-read timestamp to completion of shared-snapshot publication; and
- the snapshot queue-overwrite duration.

`successful_sample_publication_gap_us` measures the complete wall-clock gap
between two successful acquisition publications, including the absolute task
delay, core-0 starvation or suspension, all command processing, I2C work, and
publication. Its producer-side maximum updates when the next publication
finally occurs. Because core 1 can declare a stale fault while that gap is
still open, `current_observed_publication_gap_us` and its maximum independently
measure the growing time since the last successful publication without waiting
for core 0 to resume.

The runtime IMU task has no application critical section or blocking queue
receive. It intentionally blocks in `vTaskDelayUntil()` between absolute 10 ms
periods. Arduino Wire takes its internal mutex with `portMAX_DELAY` and its I2C
transaction can block up to the configured 5 ms bus timeout. Search confirms
that the IMU is the only application owner of `Wire`.

The IMU task is pinned to core 0 at priority 1. No other application-created
task is pinned there, but this ESP32-S3 framework configuration pins both the
Wi-Fi task and lwIP TCP/IP task to core 0; lwIP runs at priority 18 and Wi-Fi is
also a high-priority system task. The ESP-IDF main/system work and timer/event
infrastructure can also execute above the IMU task on that core. These system
tasks can preempt the IMU task even when the I2C read and publication timings
remain short. The Arduino event task, Arduino loop task, ESP2 motion/web task,
and stepper ownership are on core 1 and therefore cannot directly consume
core-0 task time.

The `imu_recovery` JSON object and IMU dashboard rows report autonomous turn
and strafe pause state, the saved heading, fresh-sample confirmation progress,
current or last pause duration, the 30-second recovery bound, pause counts, and
total paused time. During an autonomous IMU outage, all wheel and funnel
commands are repeatedly sent disabled and the owning mission/controller timers
are frozen. Three new consecutive fresh samples with verified MPU runtime
registers resume the same phase and target. A timeout or failed stop command
uses the owning mode's existing terminal fault path. Manual IMU Turn and
Heading-Held Strafe tests still fault immediately on loss.

The IMU panel is also the read-only soak-test view. `Reset soak counters`
enqueues a request to the acquisition service's separate fixed one-element
queue; it clears only total attempts, successful reads, failed reads,
consecutive failures, and maximum completed duration. The next scheduled read
can make the soak counters nonzero immediately. The dashboard's `Reads OK /
failed (lifetime)` values are separate uptime-lifetime counters and are never
cleared by the soak reset. `Refresh values` performs only the normal telemetry
fetch. Neither control changes robot mode, heading, turn state, IMU
initialization, drivetrain output, or accesses `Wire` from the web task. The
32-bit microsecond timestamps naturally wrap with `micros()`.

The separate `IMU Turn Test` panel implements Stage 2 manual turns. It exposes
Apply, Save, Turn +90, Turn -90, Reset angle, and Stop controls. Reset angle
queues the zero operation to the sensor owner and clears the inactive turn
state; it is rejected during an active turn, and another turn cannot start
until the reset sequence is acknowledged. The controller uses
`kp * angle_error - kd * yaw_rate`, clamps that request to the configured
maximum duty, and stops driving while it verifies settling. Completion requires
both the absolute angle error and absolute yaw rate to remain within their
limits continuously for the required settling time.

All eight IMU-turn fields default to zero/unconfigured and therefore lock out
motion until the team enters measured values:

| Field | Purpose |
| --- | --- |
| Maximum rotation duty | Absolute output clamp and hardware-speed limit for the test. |
| Kp | Duty requested per degree of remaining angle error. |
| Kd | Duty removed in proportion to measured yaw rate to damp wheel momentum. |
| Angle tolerance | Maximum absolute heading error permitted during settling. |
| Maximum finishing yaw rate | Maximum absolute rotation rate permitted during settling. |
| Required settling time | How long both completion conditions must remain true. |
| Overall turn timeout | Hard deadline that faults and stops an unfinished turn; limited by the existing 30-second timed-test cap. |
| Yaw command polarity | Measured `+1`/`-1` mapping between positive IMU heading and drivetrain yaw. |

Start is also rejected unless the IMU is configured, initialized, calibrated,
healthy, and fresh; both ESP2 front motors are configured; and the ESP1
rear-wheel link is configured and fresh. An active turn faults and stops all
four wheels if any of those runtime gates fails. This test mode does not alter
line following or any autonomous routine.

When an IMU-unavailable turn fault is raised, `imu_turn.availability_fault`
retains the exact gate evaluation that raised it: origin and reason, every IMU
availability boolean, shared-snapshot availability, evaluation/publication/
successful-read timestamps and ages, the freshness threshold, front-motor
configuration, and rear-link/status state. Its `latched` and
`imu_currently_available` fields distinguish a historical controller fault
from the current IMU state. The same capture is printed as
`IMU_TURN_AVAILABILITY_FAULT` on serial.

The active manual turn and heading-held-strafe controllers each re-peek the
length-one IMU snapshot queue immediately before their runtime availability
gate. The exact returned snapshot is then reused for heading, yaw rate, and
the controller calculation. This prevents web handling or other earlier
core-1 loop work from aging the input used by the gate. For diagnosis, a
read-only queue peek after a turn fault condition is already determined still
records the gate snapshot and newest publication/sample sequence numbers,
whether they match, the gate fetch timestamp, and fetch-to-gate age.

The `IMU Heading-Held Strafe` panel implements Stage 3. Holding Left or Right
enters `IMU_STRAFE_TEST`, captures the current heading exactly once, and
refreshes the existing 700 ms command deadman through a dedicated heartbeat.
The 10 ms motion loop calculates
`kp * angle_error - kd * yaw_rate`, clamps it to the maximum yaw-correction
duty, applies the configured yaw polarity, and combines it with the requested
lateral duty in the existing mecanum mixer. Releasing the button sends an
explicit stop; a missing heartbeat also stops the controller before the mode
can produce another command.

All five Stage 3 fields default to zero/unconfigured and lock out motion:

| Field | Purpose |
| --- | --- |
| Maximum strafe duty | Absolute lateral duty used by the held left/right test. |
| Kp | Yaw-correction duty requested per degree of heading error. |
| Kd | Duty opposing measured yaw rate to damp unwanted rotation. |
| Maximum yaw correction duty | Absolute clamp on the correction added to the strafe. |
| Yaw command polarity | Measured `+1`/`-1` mapping between positive IMU heading and drivetrain yaw. |

Maximum strafe duty plus maximum yaw correction duty must not exceed the
hardware duty cap. Apply validates the settings, Save persists valid settings
to NVS, and telemetry reports configuration validity, state, fault, direction,
current/target heading, error, yaw rate, P and damping terms, correction duty,
and elapsed time. Stage 3 does not reset heading and is rejected while an IMU
heading reset is pending.

## Modes

| Mode | Motion allowed | Purpose |
| --- | --- | --- |
| `DISABLED` | No | Default boot mode, telemetry only. |
| `SENSOR_MONITOR` | No | Continuous sensor monitoring. |
| `SINGLE_MOTOR_TEST` | One wheel only | Direction and inversion checks. |
| `MANUAL_DRIVE_TEST` | ESP2-local front wheels only | Deadman-protected local drive commands. |
| `DISTRIBUTED_DRIVE_TEST` | All four wheels | ESP2 front wheels plus ESP1 back wheel commands. |
| `LINE_SENSOR_TEST` | No | Raw LSFL/LSFR/LSS and interpreted front-line error. |
| `LINE_FOLLOW_TEST` | Yes, gated | Digital two-sensor line follower. |
| `REAR_LINE_SENSOR_TEST` | No | Raw ESP1 LSBL/LSBR telemetry and reverse-travel line interpretation. |
| `REAR_LINE_FOLLOW_TEST` | Yes, gated | Reverse travel using rear sensors and independent rear PID settings. |
| `MECHANISM_TEST` | Mechanisms only, gated | Open/close claw, winch, and ESP1 Solar Hook servos and test the ESP1 funnel motor with drive outputs stopped. |
| `AUTONOMOUS_SOLAR_PANEL` | Yes, gated | Line follow, beacon alignment, solar-panel contact, timed forward motion, and rear-line reacquisition. |
| `HABITAT_PIECES` | Yes, gated | Front-sensor line following, independent LSS2-left/LSS3-right alignment, timed reverse, bounded laser-zone count strafe, opposite compensation, slide down, forward laser approach, concurrent slide lift/reverse, then opposite return strafe until either rear sensor detects black. |
| `AUTONOMOUS_TOWER_PIECES` | Yes, gated | Reverse line following, timed chassis motion, shimmy search, then the winch/claw/stepper collection tail. |
| `PEG_FINDER` | Yes, gated | IMU-angle clockwise turn, timed linear chassis sequence, limit-terminated funnel, then sequential claw opening. |
| `TIME_TRIAL` | Yes, gated | Autonomous Solar, a configurable transition, Tower Pieces, a configurable delay, then PegFinder. |
| `IMU_TURN_TEST` | Yes, gated | Manual-only relative +90/-90 PD yaw tests; inactive, complete, stopped, and faulted states command zero. |
| `IMU_STRAFE_TEST` | Yes, gated | Manual-only held left/right strafing with a once-captured IMU heading target and deadman heartbeat. |
| `AUTONOMOUS_DRY_RUN` | No | Stub view for future mission dry runs. |

Mode changes stop actuators before switching. Sensor-only modes keep motors and
mechanisms disabled. `MECHANISM_TEST` keeps drive outputs disabled. Normal
Tower Pieces and PegFinder stopped stages and mode-specific faults keep any
commanded claw and winch PWM signals enabled so the servos hold position.
`/api/stop` is an emergency release: it disables the claw, winch, and Solar
Hook servo PWM outputs, which can let the mechanisms move or open if they
require powered holding, and sends a disabled funnel command to ESP1.

## Solar Hook Servo

The `Solar Hook Servo Control` panel owns the ESP1 servo on GPIO3. It exposes
independent absolute Open and Closed angles from 0–180 degrees, plus Apply,
Save, Open, Close, and Disable controls. Both angles default to unset so the
servo cannot move from an invented calibration. Open/Close requires a fresh
ESP1 status report and the ESP1 hardware-configured flag. The ESP1 output uses
LEDC channel 6 at 50 Hz, 12-bit resolution, and the same 1000–2000 µs pulse
range as the claws.

ESP1 initializes the output detached and disabled. Entering mechanism mode
first stops other actuators; the requested Solar Hook command is then sent.
Disable and the global emergency STOP send an explicit disabled command that
detaches GPIO3. Save persists `shopen` and `shclosed` in ESP2 NVS.

## Time Trial

The `Time Trial` dashboard panel runs `AUTONOMOUS_SOLAR_PANEL`,
`AUTONOMOUS_TOWER_PIECES`, and `PEG_FINDER` behavior in that order while the
reported mode remains `TIME_TRIAL`. It directly references the live solar,
tower, PegFinder, line-following, and servo configurations; it does not copy
those values. Time Trial Start applies the visible settings from all three
individual panels and the shared Servos panel before requesting the run.

After solar reacquires the rear line, Time Trial stops the chassis, waits for
`post_solar_delay_ms`, and optionally strafes right for
`strafe_right_duration_ms` using the shared IMU Strafe tuning. A duration of
`0` skips the transition strafe.
After Tower Pieces completes, it stops chassis and funnel motion without
disabling the servo PWM signals, waits for `post_tower_delay_ms`, explicitly
holds all claws and the winch closed, and starts PegFinder. All three
transition values persist through the main configuration Save action. Telemetry exposes
the combined state and values under `time_trial`.

## Line Sensor Bench Test

Use this when you only want to verify comparator states and line interpretation,
without driving motors:

- Dashboard: press `Sensor Test` in the line-sensor panel. This switches ESP2
  to `LINE_SENSOR_TEST`, disables actuators, and keeps updating
  LSFL/LSFR/LSS/LSS2/LSS3.
- Serial: run `mode line-sensor`, then `line status`.
- Telemetry: watch `line.lsfl_level`, `line.lsfr_level`, `line.lss_level`,
  `line.lss2_level`, and `line.lss3_level` for `HIGH`, `LOW`, or `UNKNOWN`;
  `HIGH` means black tape. LSS is ESP1 GPIO4, LSS2 is GPIO11, and LSS3 is
  GPIO12. Side sensors are not reported as usable unless the ESP1 line-sensor
  stream is fresh. Habitat Pieces uses LSS2 as its left-side stop sensor and
  LSS3 as its right-side stop sensor.

For the rear sensors, press `Rear Sensor Test` or run
`mode rear-line-sensor`, then `rear-line status`. ESP1 samples LSBL on GPIO17
and LSBR on GPIO18 as digital inputs with `HIGH` meaning black tape. It sends a
CRC-protected `SensorSnapshot` every `10 ms`; ESP2 reports raw levels,
sequence, age, configuration, and freshness without enabling motors. For
reverse travel, LSBR is reported as logical left and LSBL as logical right.
The same fixed-size packet carries LSS configuration and level so tower-piece
crossings are observed at the `10 ms` sensor-stream period.

## Tower Pieces

The `Tower Pieces` dashboard panel exposes reverse line-following duty,
second-line timeout, the live LSS level, a `0 / 2` crossing count, the delay
after that second crossing, right-strafe duration, the following pause,
clockwise rotation angle, a post-rotation pause, timed backward duty and
duration, separate right and left shimmy durations,
the shimmy timeout, optional final-reverse duty and duration, five mechanism
delays, and independent down/up stepper speeds. Start enters
`AUTONOMOUS_TOWER_PIECES` and uses the independent rear PID gains with the
panel's initial reverse-line-duty magnitude. A crossing is one LSS LOW-to-HIGH
transition; holding LSS HIGH cannot increment the count repeatedly. If LSS is
already HIGH at start, firmware waits for LOW before accepting a later HIGH as
a crossing.

The second crossing stops all four wheels in `POST_LINE_DELAY`. When that delay
expires, the robot enters `STRAFE_RIGHT` for the configured duration using the
shared IMU Strafe tuning, stops in `POST_STRAFE_PAUSE`, turns clockwise through
the configured angle using the shared IMU Turn tuning, stops in
`POST_ROTATION_PAUSE`, and then drives backward for the configured duration.
It next starts an IMU-aligned strafe right and alternates right and left after
each direction's configured duration. The shimmy ends as soon as either LSBL
or LSBR is HIGH. There is no final line-follow stage. Instead, the mode runs
this tail:

1. Drive backward at `final_reverse_duty` for
   `final_reverse_duration_ms`. A duration of `0 ms` skips this optional stage.
2. Wait for `post_final_reverse_delay_ms`, then command the winch open.
3. Wait for `post_winch_open_delay_ms`, then command all three claws open.
4. Wait for `post_claws_open_delay_ms`, then move the stepper down at
   `stepper_down_speed_steps_per_second` until the bottom limit activates.
5. Wait for `post_stepper_bottom_delay_ms`, then command all three claws closed.
6. Wait for `post_claws_closed_delay_ms`, then move the stepper up at
   `stepper_up_speed_steps_per_second` until the top limit activates.
7. Command the winch closed and complete.

The final-reverse duty and duration default to `0`. All Tower Pieces pause and
delay stages default to `1000 ms`, and both stepper limit-search speeds default
to `2000` driver microsteps per second. Other motion duties, durations, and
timeouts remain `0` (unconfigured), so Start is rejected until the required
values are positive and within their safe caps.

The default open/closed servo angles are claw 1 `23/110`, claw 2 `40/100`, claw
3 `80/180`, and winch `0/180` degrees. Tower Pieces reads the same live settings
as the separate `Servos` panel. Updating that panel rewrites any active servo
hold and changes later Tower Pieces commands; `/api/claws/save` persists the
shared values to NVS.

If the first timeout expires before the second side line, telemetry reports
`SIDE_LINE_TIMEOUT`; if the shimmy timeout expires before a back line is
detected, it reports `SHIMMY_TIMEOUT`. Stale ESP1 status, stale rear/side sensor
packets, line loss without history, incomplete hardware, failed rear commands,
servo command rejection, stepper search failure, or conflicting stepper limits
also stop the mode. Its stopped and mechanism stages stop chassis and stepper
motion without disabling the servo holding signals. `/api/stop` stops the mode
and disables the servo PWM outputs as an emergency release.

## PegFinder

The `PegFinder` dashboard panel runs clockwise rotation, a stopped pause,
backward drive, another stopped pause, forward drive, and funnel-forward
operation until its ESP2 GPIO47 limit is HIGH. It then waits and opens claws
in an adjustable permutation, using one adjustable interval between claw
commands. After all three are open, it waits for an adjustable delay and runs
the funnel in reverse at an adjustable duty for an adjustable duration. There
is no line-follow stage.

Start enters `PEG_FINDER` only after the front motors, rear command link, and
ESP1-owned funnel motor are ready, GPIO47 is configured, all three claw open
angles are valid, and ESP1 status is fresh. GPIO47 uses the same active-high
convention as the other normally-open limit switches: LOW is released and HIGH
is pressed. The funnel stops as soon as the switch is pressed. If it remains
LOW for the adjustable funnel timeout, PegFinder faults and stops instead of
continuing. Its duties and all timings default to `0` (unconfigured), so Start
is rejected until the required values are positive and within their safe caps.
ESP2 refreshes rear-wheel and funnel commands during active phases; ESP1
independently stops those motors when commands become stale. PegFinder pauses
and mode-specific faults stop chassis and funnel motion without disabling an
existing servo hold. Claw commands use the live open angles from the shared
Servos panel.

## IR Beacon Bench Test

- Set GPIO2 switch HIGH and expose the ESP1 GPIO7 sensor to a 1 kHz beacon.
  Confirm `ir_1khz_goertzel_amplitude` rises and `ir_beacon_detected` becomes
  true after the confirmation count.
- Keep GPIO2 HIGH and try a 10 kHz beacon. Confirm it does not falsely trigger
  1 kHz mode.
- Set GPIO2 switch LOW and repeat with a 10 kHz beacon.
- Block the beacon and confirm detection clears after the configured clear
  windows.
- While driving on a safe stand, confirm the dashboard keeps updating and motor
  commands remain responsive.

## Solar Contact Retry

If the front-right contact switch is hit but the back-right switch is not when
the initial right-strafe contact timeout expires, ESP2 performs one correction:
it strafes left, moves forward, and then makes one more right-strafe attempt.
The second right-strafe attempt faults on its own timeout and cannot start a
second correction sequence. Both switches stop the sequence successfully from
any contact-motion state.

The dashboard exposes `Retry left strafe ms`, `Retry forward ms`, and
`Retry right timeout ms`. Apply updates the values at runtime; Save persists
them to NVS. The two new motion durations default to `0` until the team tunes
them on the real robot; the retry timeout initially uses the existing contact
timeout.

## Safety Behavior

- Motors do not move at boot.
- All actuator outputs initialize disabled.
- `/api/stop` works from every mode.
- The dashboard always shows a STOP button.
- Manual drive commands expire after `700 ms` unless refreshed.
- Single-motor dashboard commands are press-and-hold with a `700 ms` deadman
  timeout if browser refreshes stop.
- Drive and line-follow tests have timed caps. The current cap is `5000 ms`.
- Single-motor commands accept normalized duty values up to `1.0`, so `0.7`
  is valid. Values outside `[-1, 1]` are rejected.
- All motion commands are rejected if the mode is wrong, the duty is out of
  range, duration is too long, arguments are malformed, or required hardware is
  not configured.
- `LINE_FOLLOW_TEST` requires configured line sensors, local motors, UART, a
  fresh ESP1 status link, nonzero maximum duty, and nonzero verified hardware
  duty cap.
- `REAR_LINE_FOLLOW_TEST` requires configured GPIO17/GPIO18 rear sensors, a
  fresh rear `SensorSnapshot`, configured local motors and UART, a fresh ESP1
  status link, nonzero maximum duty, and a nonzero verified hardware duty cap.
- Rear following stops all four wheels if rear sensor data exceeds
  `remoteCommandTimeoutMs`, ESP1 status becomes stale, or the line is lost
  without history.
- `HABITAT_PIECES` requires a valid independent line-follow duty, a nonzero
  LSS2/LSS3 detection delay, a longer nonzero search/alignment timeout, a valid
  reverse duty and nonzero reverse duration, configured LSS2, LSS3 and front
  line sensors and motors, a configured distance-strafe direction, threshold,
  target count, duty and timeout, all follow-on chassis/slide settings,
  configured bottom/top slide limits and rear line sensors, fresh ESP1
  sensor/status data, and a configured rear link. It ignores both side sensors only during the delay, independently stops
  the detected side, and begins the timed reverse only after both latch. Stale
  side-sensor data before both latch, front-line loss before alignment,
  rear-command failure, or timeout stops all four wheels. Successful reverse
  completion begins the distance strafe. Its target count advances to the
  opposite compensation strafe; its timeout stops all motion. The subsequent
  slide down, forward laser approach, concurrent lift/reverse, and rear-line
  return strafe each have an independent stop bound. Laser availability does
  not gate Start.
- `AUTONOMOUS_TOWER_PIECES` adds configured GPIO4 LSS, a positive panel duty,
  and a nonzero panel timeout to the rear-follow requirements. It stops on the
  second distinct LSS rising edge, timeout, or any rear-follow safety fault.
- ESP1 back motors stop on stale, invalid, duplicate, corrupt, or disabled
  wheel command packets.
- ESP2 stops line following if the rear command link is unhealthy.
- `IMU_TURN_TEST` starts locked because its tuning defaults to zero. It requires
  valid tuning, a calibrated/fresh/healthy IMU, both front motor adapters, and
  a fresh configured rear link. It stops on IMU loss, rear-link loss, a failed
  rear command, invalid controller data, explicit Stop, completion, or the
  overall timeout. Its settling state sends disabled wheel commands.
- `IMU_STRAFE_TEST` also starts locked. It requires its separately valid
  tuning and the same IMU/motor/rear-link gates. It stops on gate loss,
  controller or rear-command failure, button release, explicit Stop, mode
  change, or command-heartbeat expiry.
- Solar autonomous motion also stops if a rear-wheel command cannot be sent or
  ESP1 reports that its received commands have gone stale.
- After both solar-panel limit switches are hit, the robot waits for
  `post_contact_forward_start_delay_ms`, drives forward for
  `post_contact_forward_duration_ms` (default `1000 ms`) at
  `post_contact_forward_duty`, waits for
  `line_reacquire_strafe_start_delay_ms`, then strafes left at
  the shared IMU Strafe test duty while holding the captured heading until
  LSBL or LSBR reports black. Both delays default to `0 ms`, and all wheel
  outputs remain disabled during them.
- Claw and winch servo commands are rejected unless the corresponding ESP2 PWM
  config is complete, the requested open or closed angle is set, and that
  absolute angle is within `0..180` degrees. Open and closed targets are
  configured independently. Defaults are claw 1 `23/110`, claw 2 `40/100`,
  claw 3 `80/180`, and winch `0/180` degrees (open/closed). The `Servos` panel
  and Tower Pieces share these live settings; `/api/claws/save` persists them
  to NVS.
- Tower Pieces and PegFinder ordinary stopped stages leave commanded servo PWM
  enabled so the claws and winch continue holding. `/api/stop` disables that
  PWM, so a servo mechanism that depends on powered holding can mechanically
  release.
- Funnel commands are press-and-hold with the same `700 ms` deadman timeout as
  single-motor tests. ESP1 initializes the funnel output disabled, rejects stale
  or corrupt packets, and reports whether the funnel PWM hardware is configured.
- Browser requests call high-level command handlers; they do not write GPIO/PWM
  directly.

## Motion Failure Diagnostics

The dashboard provides an on-demand rolling motion trace. It is not polled
automatically. Press `Reset before run`, perform one raised-wheel reproduction,
then press `Refresh after failure` and copy the complete JSON report.

The fixed-capacity trace samples active motion every 100 ms and retains the
newest samples. Stopped time does not overwrite the trace. IMU completion,
IMU timeout, IMU faults, and command-deadman stops freeze it automatically;
requesting the report also freezes an unfrozen trace.

The report includes maximum loop, latest completed IMU acquisition, and
web-handler durations; missed
deadlines; web drive/stop counts and drive heartbeats processed after Stop;
requested and applied FL/FR/BL/BR commands; direct ESP2 LEDC duty readback for
the four front PWM channels; rear sequence and status ages; ESP1 applied rear
commands; command-deadman state; and IMU turn state, heading, target, error,
yaw rate, and rotation output.

## Endpoints

| Endpoint | Method | Description |
| --- | --- | --- |
| `/` | GET | Dashboard HTML. |
| `/api/status` | GET | Compact status JSON. |
| `/api/telemetry` | GET | Full telemetry JSON. |
| `/api/imu/soak/reset-counters` | GET/POST | Enqueue a reset of only the fixed IMU soak counters; does not change IMU, heading, turn, mode, or actuator state. |
| `/api/diagnostics` | GET | Freeze if necessary and return the retained motion-failure trace. |
| `/api/diagnostics/reset` | GET/POST | Clear and restart the bounded motion trace. |
| `/api/diagnostics/freeze` | GET/POST | Freeze the current trace without changing actuator state. |
| `/api/stop` | GET/POST | Emergency stop. |
| `/api/mode?mode=<mode>` | GET/POST | Change test mode safely. |
| `/api/drive?vx=<>&vy=<>&wz=<>&duty=<>` | GET/POST | Manual/distributed drive command. |
| `/api/motor?id=<FL|FR|BL|BR>&speed=<>` | GET/POST | Single motor hold/deadman command. Use `speed=0` to release. |
| `/api/invert?id=<FL|FR>` | GET/POST | Toggle front motor runtime inversion and save it. Rear inversion is a TODO on ESP1. |
| `/api/sensors` | GET | Supported sensor states. |
| `/api/line` | GET | LSFL/LSFR/LSS level, black booleans, error, last side, visibility. |
| `/api/rear-line` | GET | Physical LSBL/LSBR levels plus reverse-travel logical mapping, error, freshness, sequence, and sample age. |
| `/api/line-follow/start?ms=<>` | GET/POST | Switch to `LINE_FOLLOW_TEST` and start line following. |
| `/api/line-follow/stop` | GET/POST | Stop line following. |
| `/api/line-follow/config?kp=<>&ki=<>&kd=<>&base=<>&max-duty=<>&max-correction=<>&integral-limit=<>&derivative-limit=<>&derivative-alpha=<>&polarity=<>&telemetry=<>` | GET/POST | Runtime PID/config update. |
| `/api/rear-line-follow/start?ms=<>` | GET/POST | Switch to `REAR_LINE_FOLLOW_TEST` and start reverse travel using the independent rear PID configuration. |
| `/api/rear-line-follow/stop` | GET/POST | Stop reverse rear line following. |
| `/api/rear-line-follow/config?kp=<>&ki=<>&kd=<>&base=<>&max-duty=<>&max-correction=<>&integral-limit=<>&derivative-limit=<>&derivative-alpha=<>&polarity=<>&telemetry=<>` | GET/POST | Update the independent rear PID/config; `base` is a positive reverse-speed magnitude. |
| `/api/autonomous/habitat-pieces/start` | GET/POST | Enter `HABITAT_PIECES` and request the gated front line-follow approach. |
| `/api/autonomous/habitat-pieces/stop` | GET/POST | Stop all Habitat Pieces wheel outputs and reset its latch/state. |
| `/api/autonomous/habitat-pieces/config?duty=<>&lss2-detection-delay-ms=<>&run-timeout-ms=<>&reverse-duty=<>&reverse-duration-ms=<>&distance-strafe-direction=<LEFT_OR_RIGHT>&distance-threshold-mm=<>&distance-zone-count=<>&distance-strafe-duty=<>&distance-strafe-timeout-ms=<>&compensation-strafe-duty=<>&compensation-strafe-duration-ms=<>&slide-down-speed-steps-per-second=<>&slide-down-timeout-ms=<>&forward-to-distance-duty=<>&forward-stop-distance-mm=<>&forward-to-distance-timeout-ms=<>&slide-lift-steps=<>&slide-lift-speed-steps-per-second=<>&slide-lift-timeout-ms=<>&post-pickup-reverse-duty=<>&post-pickup-reverse-duration-ms=<>&return-strafe-duty=<>&return-line-timeout-ms=<>` | GET/POST | Validate and apply the complete pickup route. Search/alignment timeout must exceed the detection delay; each route bound must be nonzero and at most 30000 ms; stepper values must fit the configured hardware; active-run changes are rejected. The legacy parameter name `lss2-detection-delay-ms` applies to both side sensors. The compensation and return directions are automatically opposite `distance-strafe-direction`. |
| `/api/autonomous/habitat-placement/config?...&initial-heading-timeout-ms=<>&pre-ccw-strafe-right-duty=<>&pre-ccw-strafe-right-ms=<>&post-cw-reverse-duty=<>&post-cw-reverse-ms=<>&post-cw-strafe-left-duty=<>&post-cw-strafe-left-ms=<>` | GET/POST | Validate the complete Habitat Placement route, including the return-to-initial-heading turn, pre-CCW right strafe, and the timed reverse/left strafe after the clockwise turn. |
| `/api/autonomous/habitat-placement/start` | GET/POST | Enter `HABITAT_PLACEMENT` and request the fully gated placement route. |
| `/api/autonomous/habitat-placement/stop` | GET/POST | Stop the placement route, stepper, and all wheel outputs. |
| `/api/autonomous/solar/start` | GET/POST | Start the gated solar-panel autonomous test. |
| `/api/autonomous/solar/config?...&retry-forward-duty=<>&post-contact-forward-ms=<>&post-contact-forward-duty=<>&post-contact-forward-delay-ms=<>&post-forward-strafe-delay-ms=<>` | GET/POST | Update solar autonomy settings, including adjustable strafe times/timeouts and post-contact motion. All lateral stages use the shared IMU Strafe tuning; `retry-forward-duty` applies only to the non-strafe forward adjustment. |
| `/api/autonomous/tower-pieces/start` | GET/POST | Enter the tower-pieces mode and request gated reverse line following. |
| `/api/autonomous/tower-pieces/config?duty=<>&timeout-ms=<>&post-line-delay-ms=<>&strafe-duration-ms=<>&post-strafe-pause-ms=<>&rotation-angle-deg=<>&post-rotation-pause-ms=<>&reverse-duty=<>&reverse-duration-ms=<>&shimmy-right-ms=<>&shimmy-left-ms=<>&shimmy-timeout-ms=<>&final-reverse-duty=<>&final-reverse-duration-ms=<>&post-final-reverse-delay-ms=<>&post-winch-open-delay-ms=<>&post-claws-open-delay-ms=<>&stepper-down-speed-steps-per-second=<>&post-stepper-bottom-delay-ms=<>&post-claws-closed-delay-ms=<>&stepper-up-speed-steps-per-second=<>` | GET/POST | Update Tower Pieces timings, clockwise angle, non-IMU motion, optional final reverse, mechanism delays, and stepper speeds. Initial/shimmy strafes use shared IMU Strafe tuning; the clockwise turn uses shared IMU Turn tuning. |
| `/api/autonomous/peg-finder/start` | GET/POST | Enter `PEG_FINDER` and request the gated IMU-turn/chassis sequence. |
| `/api/autonomous/peg-finder/config?clockwise-angle-deg=<>&post-rotation-pause-ms=<>&reverse-duty=<>&reverse-duration-ms=<>&post-reverse-pause-ms=<>&forward-duty=<>&forward-duration-ms=<>&funnel-duty=<>&funnel-timeout-ms=<>&post-funnel-limit-delay-ms=<>&claw-open-interval-ms=<>&claw-order-1=<1..3>&claw-order-2=<1..3>&claw-order-3=<1..3>&post-claws-open-delay-ms=<>&funnel-reverse-duty=<>&funnel-reverse-duration-ms=<>` | GET/POST | Update the PegFinder IMU-controlled clockwise angle, chassis motion, limit-terminated forward funnel phase, claw order/timings, and timed reverse funnel phase. The three claw-order values must be a permutation of 1, 2, and 3. All turn tuning/output limits come from the shared IMU Turn panel. |
| `/api/autonomous/time-trial/start` | GET/POST | Enter `TIME_TRIAL`, validate all three included modes, and start Autonomous Solar. |
| `/api/autonomous/time-trial/config?post-solar-delay-ms=<>&strafe-right-duration-ms=<>&post-tower-delay-ms=<>` | GET/POST | Update only the transitions between the three included modes. The optional right strafe uses shared IMU Strafe tuning; a duration of `0` skips it. |
| `/api/imu-turn/config?max-duty=<>&kp=<>&kd=<>&tolerance-deg=<>&finish-rate-dps=<>&settle-ms=<>&timeout-ms=<>&polarity=<>` | GET/POST | Validate and apply all manual IMU-turn tuning. Changing tuning during an active turn is rejected. |
| `/api/imu-turn/start?degrees=<90|-90>` | GET/POST | Stop the previous mode, enter `IMU_TURN_TEST`, validate all safety gates, capture the current heading, and start the requested relative turn. |
| `/api/imu-turn/stop` | GET/POST | Stop all four wheel outputs and leave the IMU controller in its explicit `STOPPED` state. |
| `/api/imu-turn/reset-angle` | GET/POST | Queue a continuous-heading reset to the sensor-acquisition owner and clear the inactive turn state. Rejected while a turn is active; a new turn is rejected until the reset sequence is acknowledged. |
| `/api/imu-turn/save` | GET/POST | Persist the currently valid IMU-turn tuning to NVS. |
| `/api/imu-strafe/config?strafe-duty=<>&kp=<>&kd=<>&max-correction-duty=<>&polarity=<>` | GET/POST | Validate and apply all Stage 3 heading-held-strafe tuning. Changing tuning while active is rejected. |
| `/api/imu-strafe/start?direction=<-1|1>` | GET/POST | Stop the previous mode, enter `IMU_STRAFE_TEST`, validate all gates, capture heading once, and start a held left/right strafe. |

| `/api/imu-strafe/heartbeat` | GET/POST | Refresh only the active Stage 3 command deadman; does not recapture or update the heading target. |
| `/api/imu-strafe/stop` | GET/POST | Stop all four wheel outputs and leave the heading-hold controller in `STOPPED`. |
| `/api/imu-strafe/save` | GET/POST | Persist the currently valid Stage 3 tuning to NVS. |
| `/api/claw?id=<1|2|3>&state=<open|close>` | GET/POST | Switch to `MECHANISM_TEST` and command one claw servo. |
| `/api/claws?state=<open|close>` | GET/POST | Switch to `MECHANISM_TEST` and command all three claw servos. |
| `/api/winch?state=<open|close>` | GET/POST | Switch to `MECHANISM_TEST` and command the ESP2 GPIO6 MCPWM winch servo. |
| `/api/claws/config?claw1-open=<>&claw1-closed=<>&...&winch-open=<>&winch-closed=<>` | GET/POST | Set independent absolute open and closed angles for each claw and the winch. Legacy claw start/direction arguments remain accepted for migration. |
| `/api/claws/save` | GET/POST | Save claw and winch open/closed angles to NVS. |
| `/api/solar-hook?state=<open|close|disable>` | GET/POST | Command or explicitly detach the ESP1 GPIO3 Solar Hook servo. Open/Close switches to `MECHANISM_TEST` and requires fresh ESP1 hardware status. |
| `/api/solar-hook/config?open-angle=<>&closed-angle=<>` | GET/POST | Apply independently adjustable Solar Hook angles in the range 0–180 degrees. |
| `/api/solar-hook/save` | GET/POST | Save the Solar Hook Open and Closed angles to ESP2 NVS. |
| `/api/funnel?speed=<>` | GET/POST | Switch to `MECHANISM_TEST` and send a timed ESP1 funnel motor command. Use `speed=0` to release. |
| `/api/config` | GET | Current tunable settings. |
| `/api/config/save` | GET/POST | Save line-following, solar-autonomy, tower-pieces, PegFinder, and Time Trial transition tunables to NVS. |
| `/api/events` | GET | Fixed-size recent event log. |

All command endpoints return JSON with `ok` and either `message` or `error`.

### Autonomous rear-line handling

Solar strafes at its independently adjustable rear-line strafe duty until
either rear sensor detects tape. It then stops strafing and runs the existing
backward rear-line PID for the configured duration. Both values are exposed in
`solar_strafe_speeds`.

Tower Pieces uses its original timed, alternating shimmy and stops when either
rear sensor detects tape. Its initial strafe and shimmy duties remain
independently adjustable.

Tower Pieces also exposes the initial timed-strafe duty, right-sensor crossing
cooldown and off-line re-arm time, and the delay before the slide moves down.
`tower_line_control` reports whether the crossing detector is armed, the
accepted crossing count, rejected crossing count, and the latest accepted or
rejected edge flags. Solar also exposes independent duties for its
initial-right, retry-left, and retry-right strafes.

## Telemetry Fields

`/api/telemetry` includes:

- General: `uptime_ms`, `current_mode`, `previous_mode`, `enabled`,
  `fault_active`, `fault_code`, `fault_message`, `last_command_age_ms`,
  `deadman_remaining_ms`, `wifi_clients`, `ip_address`, `free_heap_bytes`,
  `reset_reason`.
- Line: `lsfl_raw_level`, `lsfr_raw_level`, `lss_raw_level`, `lss2_raw_level`,
  `lss3_raw_level`, `lsfl_level`, `lsfr_level`, `lss_level`, `lss2_level`,
  `lss3_level`, `lsfl_black`, `lsfr_black`, `lss_black`, `lss2_black`,
  `lss3_black`, `lss_configured`, `lss2_configured`, `lss3_configured`,
  `line_error`, `line_visible`,
  `has_history`/`hasHistory`, `last_known_line_side`,
  `line_follower_enabled`.
- Rear line: `rear_line.lsbl_raw_level`, `lsbr_raw_level`, electrical levels,
  black booleans, `configured`, `data_fresh`, `sequence`, `sample_age_ms`,
  `captured_at_ms`, `line_error`, visibility/history, last-known side, and
  `line_follower_enabled`. `logical_left_source` is `LSBR` and
  `logical_right_source` is `LSBL` for reverse travel.
- PID: `kp`, `ki`, `kd`, `baseDuty`, `maxDuty`, `maxCorrection`,
  `integralLimit`, `derivativeLimit`, `derivativeFilterAlpha`,
  `steeringPolarity`, `controlPeriodMs`, `remoteCommandTimeoutMs`,
  `telemetryEnabled`, `p_term`, `i_term`, `d_term`, `correction`.
- Rear PID: the same fields under `rear_pid`, stored independently, plus
  `effectiveBaseDuty`, which is negative while commanding reverse travel.
- Habitat Pieces: `habitat_pieces.state`, `stop_reason`,
  `time_in_state_ms`, `line_follow_duty`, `lss2_detection_delay_ms`,
  `lss2_detection_remaining_ms`, `run_timeout_ms`, `run_elapsed_ms`,
  `timeout_remaining_ms`, `reverse_duty`, `reverse_duration_ms`,
  `reverse_elapsed_ms`, `reverse_remaining_ms`, `distance_strafe_direction`,
  `distance_threshold_mm`, `distance_zone_target_count`,
  `distance_strafe_duty`, `distance_strafe_timeout_ms`,
  `distance_strafe_elapsed_ms`, `distance_strafe_remaining_ms`, `distance_mm`,
  `distance_zone_count`, `configuration_valid`, `start_ready`,
  `lss2_configured`, `lss2_data_fresh`, `lss3_configured`,
  `lss3_data_fresh`, `lss2_detection_armed`, both raw black states and latch
  states, `should_stop`, `target_reached`, `line_following`,
  `side_line_aligning`, `left_side_driving`, `right_side_driving`, `reversing`,
  `distance_strafing`, `distance_measurement_available`, `distance_sample_new`,
  `distance_zone_active`, `distance_zone_entered`, compensation
  duty/duration/elapsed/remaining and `compensation_strafing`, slide-down
  speed/timeout/elapsed/remaining plus `lowering_slide` and
  `slide_bottom_ready`, forward pickup duty/threshold/timeout timing and
  `forward_to_distance`/`forward_distance_reached`, slide-lift
  steps/speed/timeout timing and started/complete flags, post-pickup reverse
  duty/duration/timing, return direction/duty/timeout/timing, rear-line
  configuration/freshness/left/right/detected flags,
  `waiting_for_slide_lift`, and `timed_out`.
- IMU turn: `imu_turn.configuration_valid`, `active`, controller `state` and
  `fault_reason`; all eight tuning fields; start/current/target/relative
  headings, angle error and yaw rate; proportional, damping, and clamped
  rotation terms; and elapsed/settling times.
- IMU autonomous recovery: `imu_recovery.turn_paused`, `strafe_paused`, saved
  headings, pause elapsed times, maximum pause, required/current fresh-sample
  confirmations, pause counts, and total paused time.
- Tower pieces: `tower_pieces.state`, `fault_reason`, `time_in_state_ms`,
  `reverse_line_duty`, `side_line_timeout_ms`, `post_line_delay_ms`,
  `strafe_right_duty` (shared IMU value), `strafe_right_duration_ms`,
  `post_strafe_pause_ms`, `clockwise_rotation_duty` (shared IMU value),
  `clockwise_rotation_angle_deg`, `post_rotation_pause_ms`, `reverse_duty`,
  `reverse_duration_ms`, `shimmy_duty`, `shimmy_right_duration_ms`,
  `shimmy_left_duration_ms`, `shimmy_timeout_ms`,
  `final_reverse_duty`, `final_reverse_duration_ms`,
  `post_final_reverse_delay_ms`, `post_winch_open_delay_ms`,
  `post_claws_open_delay_ms`, `stepper_down_speed_steps_per_second`,
  `post_stepper_bottom_delay_ms`, `post_claws_closed_delay_ms`,
  `stepper_up_speed_steps_per_second`,
  `side_line_count`, `target_side_line_count`,
  side-sensor configuration/level, `back_line_detected`, and active chassis,
  final-reverse, and stepper flags.
- PegFinder: `peg_finder.state`, `fault_reason`, `time_in_state_ms`,
  `clockwise_duty` (shared IMU value), `clockwise_angle_deg`, `post_rotation_pause_ms`,
  `reverse_duty`, `reverse_duration_ms`, `post_reverse_pause_ms`,
  `forward_duty`, `forward_duration_ms`, `funnel_forward_duty`,
  `funnel_forward_timeout_ms`, `post_funnel_limit_delay_ms`,
  `claw_open_interval_ms`, `claw_open_order`, `post_claws_open_delay_ms`,
  `funnel_reverse_duty`, `funnel_reverse_duration_ms`,
  `funnel_limit_configured`, `funnel_limit_high`, and active chassis, funnel,
  and claw-command flags.
- Time Trial: `time_trial.state`, `time_in_state_ms`,
  `post_solar_delay_ms`, `strafe_right_duty` (shared IMU value),
  `strafe_right_duration_ms`, `post_tower_delay_ms`, and
  `strafing_right`.
- Solar autonomy: state, time in state, fault reason, IR thresholds and
  confirmation, initial contact timeout, strafe duty/delay,
  `retry_strafe_left_duration_ms`, `retry_forward_duration_ms`,
  `retry_forward_duty`, and
  `retry_strafe_timeout_ms`, `post_contact_forward_duration_ms`,
  `post_contact_forward_duty`, and
  `line_reacquire_strafe_duty`, `post_contact_forward_start_delay_ms`, and
  `line_reacquire_strafe_start_delay_ms`.
- Motor telemetry: ESP2-local physical FL/FR desired/applied milli-duty and
  ESP1 funnel desired/applied milli-duty, enabled, inversion, configured.
- ESP1 command status: physical BL/BR command, sequence, age, link health,
  configured flag, packet error count.
- ESP1 remote status from compact `HealthReport` frames: availability, uptime,
  mode, fault status, rear applied commands, rear inversion flags, funnel applied
  command/configuration, side-line sensor configuration/raw level, and
  ultrasonic 1 configuration, validity, distance, and echo duration.
- Ultrasonic 1: `ultrasonic_1.configured`, `data_fresh`, `echo_valid`,
  `distance_mm`, `echo_duration_us`, and `sample_age_ms`.
- Laser distance: `laser_distance.available`, `configured`, `initialized`,
  `ranging`, `data_fresh`, `data_valid`, `profile`, `distance_mm`,
  `measurement_sequence`, `packet_sequence`, `sensor_range_status`,
  `driver_status`, `sda_gpio`, `scl_gpio`, `i2c_address`, `captured_at_ms`,
  `sample_age_ms`, `snapshot_age_ms`, `intermeasurement_period_ms`,
  success/failure/consecutive-failure counters, and current/maximum acquisition
  durations. A UART heartbeat can keep `snapshot_age_ms` low while
  `sample_age_ms` continues to increase.
- Servos: `claws.claw_1`/`claw_2`/`claw_3`, `habitat_pusher`, and `winch`
  report hardware configuration, GPIO, `pwmBackend`, LEDC or MCPWM resource,
  frequency/resolution, independently configured absolute open/closed angles,
  output enabled, and commanded angle/open state. The pusher is GPIO5/LEDC 7;
  the winch is GPIO6/MCPWM unit 0, timer 0, generator A.
- Solar Hook: `solar_hook` reports ESP1 hardware configuration, independently
  configured Open/Closed angles, PWM output state, commanded angle, and
  commanded Open/Closed state. The `esp1` object also carries the remote
  hardware/output/angle status.
- IR beacon telemetry from ESP1 GPIO7/GPIO2:
  `selectedBeaconFrequencyHz`,
  `switchRawState`, `switchDebouncedState`, `latest_raw_adc_sample`,
  `adc_sample_mean`, `ir_adc_min`, `ir_adc_max`, `ir_amplitude_pp`,
  `ir_1khz_goertzel_amplitude`, `ir_10khz_goertzel_amplitude`,
  `ir_selected_frequency_amplitude`, `ir_active_threshold`,
  `ir_beacon_detected`, `ir_consecutive_detection_count`,
  `ir_adc_sample_rate_hz`, and `motor_command_magnitude_milli`.
- Future/stub fields for the remaining IR, ultrasonic 2, stepper, servos, limit
  switches, and battery-style expansion.

## Serial Commands

Open the ESP2 serial monitor at 115200 baud:

```text
help
status
stop

mode disabled
mode sensors
mode single-motor
mode manual-drive
mode distributed-drive
mode line-sensor
mode line-follow
mode rear-line-sensor
mode rear-line-follow
mode habitat-pieces
mode mechanism
mode autonomous-dry-run

sensor status
line status
rear-line status
habitat config <duty> <side-delay-ms> <search-align-timeout-ms> <reverse-duty> <reverse-duration-ms> <left|right> <distance-threshold-mm> <zone-count> <strafe-duty> <strafe-timeout-ms>
habitat start
habitat status
habitat stop

motor test FL 0.10 1000
motor test FR 0.10 1000
motor test BL 0.10 1000
motor test BR 0.10 1000
motor invert FL
motor invert FR

drive fwd 0.10 1000
drive back 0.10 1000
drive left 0.10 1000
drive right 0.10 1000
drive cw 0.10 1000
drive ccw 0.10 1000

lf start 5000
lf stop
lf status
lf kp 0.10
lf ki 0.00
lf kd 0.00
lf base 0.20
lf speed 0.20
lf max-duty 0.35
lf max-correction 0.20
lf integral-limit 1.00
lf derivative-limit 20.00
lf derivative-alpha 0.00
lf polarity 1
lf reset
lf telemetry on
lf telemetry off
rlf start 5000
rlf stop
rlf status
rlf reset
rlf kp 0.10
rlf ki 0.00
rlf kd 0.00
rlf base 0.20
rlf max-duty 0.35
rlf max-correction 0.20
rlf polarity 1
rlf telemetry on
```

The compact serial `habitat config` command updates the original line,
alignment, reverse, and zone-count fields while retaining the pickup-extension
values already in memory. Configure the full pickup-extension field set through
the Habitat Pieces dashboard/API before using that serial command.

`rlf` tunes a separate rear configuration. Its initial values are copied from
the front follower, but subsequent `rlf` commands and saves do not change `lf`
settings. `rlf base` is stored as a positive magnitude; the follower applies a
negative effective base so all four wheels travel in the opposite direction.

Malformed commands print `rejected: <reason>`.

## Line Sensor Mapping

HIGH means black tape; LOW means white/non-tape.

| LSFL | LSFR | Error | Visible | Last side |
| --- | --- | --- | --- | --- |
| HIGH | HIGH | `0` | true | preserved |
| LOW | HIGH | `-1` | true | `-1` |
| HIGH | LOW | `+1` | true | `+1` |
| LOW | LOW after `+1` | `+5` | false | `+1` |
| LOW | LOW after `-1` | `-5` | false | `-1` |
| LOW | LOW no history | `0` | false | `0` |

Line error mapping is logical and is not changed by motor inversion.

## Bring-Up Checklist

1. Raise the robot so wheels cannot move it across the table.
2. Fill in verified GPIOs, LEDC channels, PWM frequency/resolution, H-bridge
   mode, UART pins/baud, wheel `forward_sign`, and `maximum_safe_test_duty`.
3. Build and upload ESP1 and ESP2.
4. Connect to `Team14Robot` and open `http://192.168.4.1/`.
5. Confirm the dashboard boots in `DISABLED`.
6. Enter `SINGLE_MOTOR_TEST` and hold one wheel at a low duty briefly. Correct
   compile-time `forward_sign` first; use runtime inversion only for ESP2
   front-wheel bench calibration.
7. Enter `DISTRIBUTED_DRIVE_TEST` and verify all directions with wheels raised.
8. Do not use line-sensor or line-following modes for this drive test.
9. Confirm ESP1 remote status becomes available on the dashboard after UART is
    configured.
10. Prove stale-command and UART shutdown before any floor test.

Passing software builds do not prove physical safety or line-following behavior.
Hardware verification is still required.

## Habitat Placement

ESP2 telemetry publishes `habitat_placement` with the current state, fault
reason, time in state, configuration/readiness flags, and every adjustable
route value. The dashboard applies, saves, starts, and stops the route through
`/api/autonomous/habitat-placement/config`, `/start`, and `/stop`. The
`claws.habitat_pusher` object reports the new servo's hardware, calibration,
enabled output, and commanded target; manual testing uses
`/api/habitat-pusher?state=open|close`.

The clockwise turn now flows directly into `REVERSE_AFTER_CLOCKWISE`, then
`STRAFE_LEFT_AFTER_CLOCKWISE`, before the existing post-clockwise delay. Both
motions have independently adjustable positive duty magnitudes and durations;
the chassis mixer applies the backward and left signs.

When Habitat Placement starts, it captures one fresh continuous IMU heading
before enabling rear-line motion. After LSS1 and the configured stopped delay,
`TURN_TO_INITIAL_HEADING` returns to that exact target. The route then runs
`STRAFE_RIGHT_BEFORE_COUNTER_CLOCKWISE` for its independently adjustable duty
and duration. `TURN_COUNTER_CLOCKWISE` targets the same captured heading plus
or minus the configured physical CCW offset according to the measured IMU yaw
polarity, rather than taking a new heading capture. Later IMU samples only
close the error to those fixed targets. `initial_heading_captured`,
`initial_heading_deg`, and `counter_clockwise_target_heading_deg` retain the
saved values in Habitat Placement telemetry after the shared controller is
reused by the later clockwise turn. The return turn uses
`initial_heading_turn_timeout_ms` and the shared adjustable IMU maximum turn
duty; the right strafe uses
`pre_counter_clockwise_strafe_right_duty` and
`pre_counter_clockwise_strafe_right_duration_ms`.
