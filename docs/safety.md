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
- `HABITAT_PIECES` line-follows immediately and ignores both LSS2-left and
  LSS3-right only for an explicitly configured nonzero detection delay. Each
  black detection latches its wheel side stopped; the other side continues
  until its sensor latches, then the bounded straight-backward move begins.
  Reverse duty and duration are locked at zero until configured; completion
  begins the separately bounded distance-zone strafe. Direction, threshold,
  target count, duty, and timeout are locked until explicitly configured. The
  overall search/alignment timeout must be longer than the detection delay.
- Habitat Pieces requires configured LSS2 and LSS3 inputs and a fresh shared
  ESP1 sensor snapshot at start and until both detections latch. Missing
  configuration, stale side-sensor data, front line loss before alignment,
  rear-command failure, or timeout stops all four wheels. No detection delay,
  search/alignment timeout, reverse duty/duration, distance direction,
  threshold, count, duty, or distance timeout is compiled in; all must be
  explicitly configured. During the timed reverse, continued side-
  sensor data is not required, but motor and communication safety gates remain
  active. During distance strafing, only new valid high-accuracy measurement
  sequences can affect the counter. Repeated, invalid, stale, or no-signal
  readings never increment it; the configured timeout stops the motion if the
  target count is not reached.
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
