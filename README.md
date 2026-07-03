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

## ESP2 Line-Follower Commands

Open the ESP2 serial monitor at 115200 baud and use:

```text
lf start
lf stop
lf status
lf kp <value>
lf ki <value>
lf kd <value>
lf speed <normalized-duty>
lf max-duty <normalized-duty>
lf max-correction <value>
lf polarity <1|-1>
lf period-ms <integer>
lf telemetry on
lf telemetry off
lf reset
lf test sensor
lf test motor <fl|fr|bl|br> <duty>
lf test drive <duty>
lf test stop
```

`lf start` and movement tests reject commands while required hardware facts are
still TODO. See `docs/line-following-plan.md` for the bring-up procedure.

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
