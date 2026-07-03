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
