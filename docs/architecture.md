# Architecture

The robot uses two ESP32-S3 processors in one PlatformIO monorepo.

ESP1 is the mission controller. It owns the autonomous mission state machine,
mission-level safety decisions, ESP1-local sensors, rear wheel outputs, the
funnel motor, and one side of the UART link.

ESP2 is the motion controller and mechanism node. It owns front line sensing,
logical four-wheel motion calculation, front wheel outputs, the stepper, the
servo mechanisms, mechanism limit switches, and one side of the UART link.

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
- ESP2 motion task: line sensor acquisition, pure line observation, chassis
  control, mechanism coordination.
- UART handling may be integrated into those tasks or split into one clearly
  bounded communication task per processor after timing requirements are known.

Do not create one task per sensor. Sensor acquisition should produce immutable
snapshots that downstream logic consumes.

## Shared Code

Hardware-independent types and pure logic live in `include/common` and
`src/common`. GPIO, PWM, ADC, UART peripheral setup, and board-specific details
must stay outside mission logic and native tests.
