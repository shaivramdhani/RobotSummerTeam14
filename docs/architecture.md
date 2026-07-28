# Architecture

The robot uses two ESP32-S3 processors in one PlatformIO monorepo.

ESP1 is the mission controller. It owns the autonomous mission state machine,
mission-level safety decisions, ESP1-local sensors, rear wheel outputs, the
funnel motor, and one side of the UART link.

ESP2 is the motion controller and mechanism node. It owns front line sensing,
logical four-wheel motion calculation, front wheel outputs, the stepper, the
servo mechanisms, mechanism limit switches, the I2C IMU, and one side of the
UART link.

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

Stage 2 adds a separate, manual-only `IMU_TURN_TEST` mode. Its
hardware-independent PD controller captures one continuous relative-heading
target, applies yaw-rate damping, and requires both angle and rate to remain
inside their configured limits for the full settling time. Only this mode
generates IMU-corrected wheel commands. It passes the resulting
`FourWheelCommand` through `applyWheelCommand()` unchanged. Line following,
autonomous states, and the line PID do not invoke the IMU controller.

## Shared Code

Hardware-independent types and pure logic live in `include/common` and
`src/common`. GPIO, PWM, ADC, UART peripheral setup, and board-specific details
must stay outside mission logic and native tests.
