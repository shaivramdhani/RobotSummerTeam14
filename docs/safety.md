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
