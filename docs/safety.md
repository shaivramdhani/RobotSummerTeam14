# Safety

Runtime motor-moving behavior now exists behind explicit commands and verified
hardware configuration. With the current TODO configuration, motor outputs remain
disabled and line following refuses to start.

## Startup

- Initialize all actuator outputs disabled.
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
- Runtime MPU-6050 I2C transactions execute only in ESP2's sensor-acquisition
  task. The core-1 motion task consumes a copied snapshot and treats data older
  than the IMU freshness limit as unavailable.
- IMU heading resets are queued to the sensor owner. A turn cannot start until
  the corresponding reset sequence has been acknowledged in a published
  snapshot.
- The read-only IMU soak controls only refresh telemetry or enqueue a reset of
  fixed acquisition counters. They do not change mode, heading, IMU
  initialization, turn state, or any actuator command.
- IMU yaw control is selected only by `IMU_TURN_TEST`; it is not injected into
  shared wheel output or line-following control.

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
