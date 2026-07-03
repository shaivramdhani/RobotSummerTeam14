# Repository Rules

This repository contains firmware for a two-ESP32-S3 autonomous competition
robot. Keep changes conservative, typed, testable, and safe.

## Safety

- Do not generate runtime code that can move a motor until the team explicitly
  asks for hardware behavior.
- All actuator outputs must initialize disabled.
- Motor commands must expire after a communication timeout.
- Both processors must stop local motors when valid communication becomes stale.
- Operational code must not use blocking `delay()`.
- Periodic FreeRTOS work should use an absolute-period design such as
  `vTaskDelayUntil` plus explicit millisecond-to-tick conversion.
- Do not dynamically allocate memory in a control loop.

## Hardware

- Never invent GPIO numbers, active levels, voltages, timing values, PWM
  channels, PWM frequencies, servo pulse ranges, or calibration values.
- Represent missing information with clearly marked TODOs.
- Keep all GPIO definitions in one configuration location per ESP.
- Every physical peripheral must have exactly one software owner.
- Do not choose the final PWM peripheral allocation until the PWM resource review
  is complete.

## Architecture

- ESP1 is the mission controller.
- ESP2 is the motion controller and mechanism node.
- ESP2 owns logical four-wheel motion calculation.
- ESP2 applies front-wheel commands locally and sends rear-wheel commands to
  ESP1.
- ESP1 sends mission and chassis-motion commands to ESP2.
- High-level mission code must not directly access GPIO or PWM.
- Avoid global mutable state.
- Pass immutable sensor snapshots or messages between tasks.
- Do not create one FreeRTOS task per sensor.
- Use a small number of clearly defined periodic or event-driven tasks.
- No wheel encoders currently exist; do not assume encoders.

## Code Style

- Use C++17.
- Prefer `enum class` and typed structures over magic integers.
- Use fixed-width integer types for communication packets.
- Preserve exact units in names, such as `timeout_ms` or `speed_mps`.
- Keep hardware-independent logic in `include/common` and `src/common` so it can
  be tested with the native PlatformIO environment.
- Treat legacy line-sensor task structure as behavioral reference only. Use one
  coherent sensor-acquisition path and one motion-control path.

## Required Checks

Run these before considering scaffold or interface changes complete:

```sh
pio run -e esp1
pio run -e esp2
pio test -e native
```
