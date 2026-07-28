# Test Plan

## Required Checks

Run these before merging firmware scaffold changes:

```sh
pio run -e esp1
pio run -e esp2
pio test -e native
```

The GitHub Actions workflow runs the same checks.

## Native Tests

Native tests cover pure, hardware-independent logic:

- Digital front line sensor truth table and line-loss history.
- PID proportional, integral, derivative, clamp, and reset behavior.
- Differential motor mixing and steering polarity.
- IMU turn configuration lockout, continuous relative target capture, PD output
  and clamp behavior, yaw-rate damping, dual angle/rate settling conditions,
  settling reset, timeout, explicit stop, mode policy, and telemetry JSON.
- Rear-drive command packet validation and stale/explicit-stop behavior.
- Mission transition logic after transitions are defined.

Native tests must not include Arduino, GPIO, PWM, UART peripherals, or motor
drivers.

## Firmware Build Checks

ESP1 and ESP2 firmware builds verify that board-specific entry points, headers,
and compile-time ownership boundaries remain valid.

## Future Hardware Tests

Add hardware-in-loop tests only after safe actuator infrastructure exists.
Hardware tests must begin by proving disabled startup states and stale-command
shutdown behavior.

For the read-only IMU soak test:

1. Keep the robot disabled and record the current mode, heading, IMU-turn state,
   and all four applied wheel commands.
2. Press `Reset soak counters`; confirm attempts and successes resume increasing
   at approximately the existing 10 ms acquisition period.
3. Confirm successes plus failures equals total attempts, maximum completed
   duration never decreases between resets, a successful read updates its
   timestamp, and a failure increments both failed and consecutive counts.
4. Press `Refresh values`; confirm it only reloads telemetry.
5. Confirm both buttons leave the recorded mode, heading, turn state, and wheel
   commands unchanged. Repeat while another safe test mode is selected to
   confirm the soak view is independent of robot mode.
6. Run for the desired soak interval and confirm memory use remains stable; no
   per-read log or queue is retained.

For Stage 2, keep the wheels raised for the first tests:

1. Confirm Apply rejects zero/incomplete tuning and Start rejects an unhealthy
   or stale IMU and rear link.
2. Configure a deliberately low maximum rotation duty and all other required
   values.
3. Request +90 and confirm the initial measured heading moves toward the target.
   If it moves away, press Stop and reverse only `yaw_command_polarity`.
4. Confirm Stop immediately disables front and rear wheel commands.
5. Confirm disconnecting the IMU or rear link during a turn faults and stops.
   An IMU bus stall must make the published snapshot stale and fault the turn
   without stretching the core-1 motion-loop interval by the duration of the
   stalled I2C transaction.
6. Before reproducing an intermittent front/rear stop mismatch, use the
   dashboard `Reset before run` control. After the wheels have stopped, use
   `Refresh after failure` and retain the full motion-diagnostics JSON. Confirm
   it contains loop/web/IMU maximum durations, front LEDC readback, all four
   command values, rear status ages, and the terminal trigger.
7. Confirm completion occurs only after both angle error and yaw rate remain
   within limits for the full settling time.
8. Repeat -90 before any floor test. Tune on a stand before increasing the duty
   clamp.
9. Press Reset angle while idle and confirm the request is acknowledged within
   subsequent telemetry before Start is accepted.
