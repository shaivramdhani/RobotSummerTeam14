# RobotSummerTeam14

Firmware for a two-ESP32-S3 autonomous competition robot.

The current functional milestone is digital two-sensor tape following with ESP2
controlling front motors and sending rear motor commands to ESP1. Runtime motor
outputs remain disabled until GPIOs, PWM resources, UART pins, H-bridge mode,
per-wheel direction signs, and maximum safe test duty are filled in the owning
ESP's `PinConfig.h`.

## Toolchain

- PlatformIO Core 6.1.19
- PlatformIO Espressif 32 platform `espressif32@6.12.0`
- Arduino-ESP32 framework package `3.20017.241212+sha.dcc1105b`
- C++17
- Arduino-ESP32 plus FreeRTOS APIs for firmware tasks

The temporary board target is `esp32-s3-devkitc-1` for both processors. Replace
it only when the exact ESP32-S3 board/module choice is confirmed.

## Build

```sh
pio run -e esp1
pio run -e esp2
```

## Native Tests

```sh
pio test -e native
```

Native tests are for hardware-independent logic only. Hardware drivers and GPIO
access do not belong in the native test target.

## Telemetry Dashboard

ESP2 starts a test-only WiFi softAP:

```text
SSID: Team14Robot
Password: robotdebug
Dashboard: http://192.168.4.1/
```

The dashboard boots in `DISABLED`, exposes `/api/telemetry`, `/api/stop`,
mode-gated drive/motor/line-follow/claw commands, and refuses actuator movement
while required hardware settings are TODO. See `docs/telemetry.md`.

For the current two-ESP wiring, flash both ESPs and use the ESP2 dashboard to
test individual wheels, distributed drive, front line sensors, and the
line-following PID/PD tuning loop. See `docs/drive-test.md` and
`docs/line-following-plan.md`.

## Quick Commands

```sh
pio run -e esp1
pio run -e esp2
pio run -e esp1 -t upload --upload-port /dev/ttyUSB_TODO
pio run -e esp2 -t upload --upload-port /dev/ttyUSB_TODO
pio device monitor -e esp2 --port /dev/ttyUSB_TODO
pio test -e native
```

## ESP2 Line-Follower Commands

Open the ESP2 serial monitor at 115200 baud and use:

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
sensor status
line status
rear-line status
motor test FL 0.10 1000
drive fwd 0.10 1000
lf start
lf stop
lf status
lf kp <value>
lf ki <value>
lf kd <value>
lf base <normalized-duty>
lf speed <normalized-duty>
lf max-duty <normalized-duty>
lf max-correction <value>
lf integral-limit <value>
lf derivative-limit <value>
lf derivative-alpha <value>
lf polarity <1|-1>
lf period-ms <integer>
lf timeout-ms <integer>
lf reset
lf telemetry on
lf telemetry off
rlf start
rlf stop
rlf status
rlf reset
rlf kp <value>
rlf ki <value>
rlf kd <value>
rlf base <positive-normalized-duty-magnitude>
rlf max-duty <normalized-duty>
rlf max-correction <value>
rlf polarity <1|-1>
rlf telemetry on
rlf telemetry off
```

`lf start` and movement tests reject commands while required hardware facts are
still TODO or the ESP1 status link is stale. See `docs/line-following-plan.md`
for the bring-up procedure.

`rlf start` drives the robot in reverse using ESP1 rear sensors LSBL/LSBR on
GPIO17/GPIO18. In the reverse travel frame, physical LSBR is logical left and
physical LSBL is logical right. Rear PID settings are independent and are
initialized from the front settings the first time; the rear dashboard panel,
`rlf` commands, and saved NVS keys then tune them separately. The entered rear
base is a positive magnitude and firmware applies it as a negative wheel
command. Starting also requires a fresh rear-sensor snapshot.

The ESP2 dashboard also has a `Tower Pieces` panel. Its Start button enters
`AUTONOMOUS_TOWER_PIECES`, follows the rear line backward, counts distinct LSS
LOW-to-HIGH transitions, and stops all four wheels on transition two. It then
waits, strafes right for a configured duration, pauses, rotates clockwise for a
configured duration, pauses again, and drives backward for a configured
duration. It then alternates right and left strafes, starting right, until either
back line sensor is HIGH or the shimmy timeout expires. After detecting that
line, it can perform an optional timed backward drive (`0 ms` skips it), waits,
opens the winch, waits, opens all three claws, waits, and lowers the stepper to
the bottom limit. It then waits, closes all three claws, waits, raises the
stepper to the top limit, and closes the winch. There is no final line-following
stage. The right and left shimmy durations and all tail settings are adjustable
in the panel and can be saved to NVS.

Tower Pieces motion duties and durations remain `0` until configured, including
the optional final reverse. Its pause and delay stages default to `1000 ms`, and
both limit-search speeds default to `2000` driver microsteps per second. The
shared servo defaults are claw 1 open/closed `23/110`, claw 2 `40/100`, claw 3
`80/180`, and winch `0/180` degrees. Tower Pieces uses the same live angles as
the `Servos` panel, so panel changes update an active servo hold and later
sequence commands; `/api/claws/save` persists them to NVS.

The dashboard also provides a `PegFinder` mode. It rotates clockwise for a
configured duration, pauses, drives backward for a configured duration,
pauses again, drives forward for a configured duration, and then runs the
ESP1-owned funnel motor forward until the ESP2 GPIO47 limit switch is pressed.
GPIO47 is LOW while released and HIGH while pressed. If the switch is not
pressed before the adjustable timeout, PegFinder stops with a fault. After a
configurable delay, it opens claws 1, 2, and 3 in order with a configurable
delay between each command, using the live angles from the shared `Servos`
panel. Its chassis and funnel duties and all timings remain `0` (unconfigured)
until the team enters verified values.

The `Time Trial` panel runs Autonomous Solar, Tower Pieces, and PegFinder in
that order. It uses the same live configuration objects as the three individual
panels, so applying or saving an individual setting also changes the combined
run. Pressing Time Trial Start first applies all three visible panels and the
shared servo angles. Its only separate settings are the delay after solar, an
optional timed right strafe and duty before Tower Pieces, and the delay after
Tower Pieces. These transition values default to `0`; a `0 ms` strafe skips
that motion. At the Tower Pieces-to-PegFinder handoff, the claws and winch are
commanded closed and their PWM outputs remain enabled.

## Upload

Use the correct serial port for the processor connected to USB.

```sh
pio run -e esp1 -t upload --upload-port /dev/ttyUSB_TODO
pio run -e esp2 -t upload --upload-port /dev/ttyUSB_TODO
```

## Monitor

```sh
pio device monitor -e esp1 --port /dev/ttyUSB_TODO
pio device monitor -e esp2 --port /dev/ttyUSB_TODO
```

## Repository Layout

```text
include/common/   Shared typed interfaces and pure logic
include/esp1/     ESP1 pin configuration and mission interfaces
include/esp2/     ESP2 pin configuration and mechanism interfaces
src/common/       Hardware-independent implementation files
src/esp1/         ESP1 firmware entry point
src/esp2/         ESP2 firmware entry point
test/             Native tests for pure algorithms
docs/             Architecture and operational documentation
```

## Processor Roles

ESP1 is the mission controller. It owns mission sequencing, rear motors, funnel
motor, rear/right-side limit switches, rear line sensors, IR inputs, ultrasonic
signals, and its side of the UART link.

ESP2 is the motion and mechanism node. It owns front line sensors, front motors,
logical four-wheel motion calculation, stepper, servos, mechanism limit
switches, and its side of the UART link. ESP2 applies front-wheel commands
locally and sends rear-wheel commands to ESP1.

Both processors must initialize local actuator outputs disabled and stop local
motors if valid communication becomes stale.
