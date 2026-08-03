# Safety

Runtime motor-moving behavior now exists behind explicit commands and verified
hardware configuration. With the current TODO configuration, motor outputs remain
disabled and line following refuses to start.

## Startup

- Initialize all actuator outputs disabled.
- The ESP1 Solar Hook servo initializes detached on GPIO3. Its Open/Closed
  angles default to unset, and the dashboard rejects movement until an angle is
  configured and fresh ESP1 hardware status is available.
- Do not attach PWM to motors until GPIOs, PWM resources, frequencies,
  H-bridge mode, direction sign, and disabled states are confirmed.
- High-level mission code must not directly access GPIO or PWM.
- ESP2 line following requires `lf start`; motors do not start at boot.
- IMU turn tuning defaults to zero/unconfigured; a dashboard turn cannot start
  until every controller value and mounting polarity is explicitly configured.

## Communication

- Every actuator command must expire.
- Both processors must disable local motor outputs if valid communication
  becomes stale.
- Communication packets must include version, message type, sequence number, and
  integrity check.

## Control Loops

- Do not use blocking `delay()` in operational code.
- Use `vTaskDelayUntil` with explicit millisecond-to-tick conversion for
  periodic FreeRTOS work.
- Do not dynamically allocate memory in control loops.
- Pass immutable snapshots and messages between tasks.
- Runtime VL53L0X I2C transactions execute only in ESP1's shared
  sensor-acquisition task. Readiness is polled; operational code never waits
  for a ranging cycle to complete.
- Laser UART heartbeats do not make old measurements fresh. ESP2 freshness is
  based on arrival of a changed measurement sequence.
- `HABITAT_PIECES` line-follows immediately and ignores LSS2 only for an
  explicitly configured nonzero detection delay. A fresh black LSS2 input
  stops all four wheels for one control update, then the bounded
  straight-backward move begins. LSS3 remains telemetry-only in this route.
  Reverse duty and duration are locked at zero until configured; completion
  begins the separately bounded distance-zone strafe. Direction, threshold,
  target count, duty, post-count stop delay, exit-pulse duration, and timeout
  are locked until explicitly configured. The
  overall LSS2 search timeout must be longer than the detection delay.
- Habitat Pieces requires a configured LSS2 input, fresh shared ESP1 sensor
  snapshot, valid shared IMU heading-hold tuning, and a healthy IMU at Start.
  Missing configuration, stale LSS2 data, front line loss before detection,
  rear-command failure, or timeout stops all four wheels. No detection delay,
  search timeout, reverse duty/duration, distance direction,
  threshold, count, duty, post-count delay, exit-pulse duration, or distance
  timeout is compiled in; all must be explicitly configured. During the timed reverse, continued side-
  sensor data is not required, but motor and communication safety gates remain
  active. Distance strafing uses the shared IMU heading-hold controller. During
  that step, only new, fresh normal-profile measurement sequences affect the
  counter. Only entries at or below threshold count, and an above-threshold
  sample rearms the counter. After the count, each timed exit pulse ends with
  an all-wheel stop before a new measurement is evaluated. A fresh N/A/no-target
  result is substituted with 65536 mm and satisfies the exit check, while an
  unavailable or stale stream supplies no sample. Repeated sequences cannot
  increment the count or satisfy a post-pulse check, and the configured timeout
  bounds the full count/delay/pulse/check sequence. Clearing the final zone begins a bounded
  bottom-limit search and a separately bounded valid-distance approach. The
  step-counted lift begins while every wheel remains stopped for the configured
  lift-start delay, then runs concurrently with the bounded reverse and
  opposite-direction IMU rear-line strafe. Either rear sensor stops chassis
  motion; the route waits stopped for an unfinished lift before handing off to
  Habitat Placement. Stepper-command failure, conflicting limits, stale rear
  line data, or any slide/approach/lift/reacquisition timeout stops both the
  chassis and slide.
- Runtime MPU-6050 I2C transactions execute only in ESP2's sensor-acquisition
  task. The core-1 motion task consumes a copied snapshot and treats data older
  than the IMU freshness limit as unavailable.
- IMU heading resets are queued to the sensor owner. A turn cannot start until
  the corresponding reset sequence has been acknowledged in a published
  snapshot.
- The read-only IMU soak controls only refresh telemetry or enqueue a reset of
  fixed acquisition counters. They do not change mode, heading, IMU
  initialization, turn state, or any actuator command.
- IMU yaw control is selected by `IMU_TURN_TEST`, `IMU_STRAFE_TEST`, or the
  explicit autonomous turn/strafe states. It is not injected into normal
  manual drive, forward/backward autonomous motion, or line-following control.
- `IMU_STRAFE_TEST` requires valid locked-out-by-default tuning, a fresh
  healthy IMU, configured front motors, and a fresh configured rear link.
  Releasing the held control, losing its browser heartbeat, losing either
  sensor/link gate, pressing Stop, or changing mode stops all wheel commands.
- Maximum strafe duty plus maximum yaw-correction duty must fit within the
  verified hardware duty cap.
- Autonomous IMU motion requires the same valid shared tuning, fresh healthy
  IMU, configured motors, and fresh rear link as its manual test. An IMU I2C
  outage during an autonomous turn or strafe immediately disables all four
  wheels and freezes the owning mission and controller timers. The saved
  heading and controller target are retained. Motion resumes only after three
  new consecutive fresh samples; the pause is bounded by the existing
  30-second timed-motion cap. Expiry becomes the owning mode's terminal
  `IMU_UNAVAILABLE` fault. Manual IMU test modes remain terminal-on-loss.
- After a failed runtime read, the MPU driver verifies `WHO_AM_I`, wake,
  filter, gyro-range, and accelerometer-range registers before accepting a
  recovery sample. A sensor power cycle therefore cannot resume autonomous
  motion with reset register values; it remains stopped until the bounded
  recovery expires. Rear-link, motor, configuration, and command failures
  remain immediately terminal.

## Faults

Initial fault categories:

- Communication stale
- Invalid command
- Limit switch conflict
- Hardware not configured

Fault handling currently forces mission state to `SafeStopped`.

## Hardware Configuration Gate

The motor adapters check all of these before driving:

- Both PWM GPIOs assigned.
- Both LEDC channels assigned.
- PWM frequency and resolution assigned.
- H-bridge mode selected.
- Per-wheel `forward_sign` set to `+1` or `-1`.
- Runtime duty is within `maximum_safe_test_duty`.

If any item is missing, the adapter remains disabled.
