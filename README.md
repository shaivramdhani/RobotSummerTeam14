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

The dashboard also reports the ESP1-owned VL53L0X V2 distance sensor on SDA
GPIO10/SCL GPIO9. ESP1 initializes it at boot but ranges only while a Habitat
Pickup or Placement step is active. The `HABITAT_PIECES` mode follows the front line at an
independently adjustable duty (default `0.12`) and ignores the ESP1-owned LSS2
and LSS3 for an adjustable detection delay. Either sensor then stops all four
wheels. LSS2 starts clockwise rotation until LSS3 sees black; LSS3 starts
counter-clockwise rotation until LSS2 sees black. Rotation uses the same
profile line-follow duty and configured yaw polarity. The robot stops again,
then drives straight backward at an adjustable duty and duration,
then strafes in an adjustable left/right direction through IMU heading hold
while counting distinct entries at or below an adjustable laser-distance
threshold. Each pickup profile has an adjustable count-ignore duration during
which the count stays zero. A target held in-zone across that gate must leave
and re-enter before counting. Consecutive in-zone samples count once, and an above-threshold
sample rearms the next count. At the target count, the robot stops for an
adjustable delay, then repeats adjustable-duration IMU-strafe pulses with an
independently adjustable duty, all-wheel stop, and fresh laser check between
pulses. The slide begins seeking its bottom limit at the start of every pickup
profile, concurrently with line following, alignment, reverse, and distance
strafing. An above-threshold check clears the final piece and begins the pickup
tail immediately if the slide is already down, or waits stopped for the same
bounded slide search to finish. The robot then drives forward until the active-high ESP2 GPIO48 habitat-piece
limit switch is pressed, briefly reverses for an adjustable duration using the
pickup reverse duty, and starts a step-counted
lift during an adjustable stopped delay before continuing concurrently with a
timed reverse and opposite-direction IMU strafe at its own adjustable duty.
Either rear line sensor stops the strafe; lift completion then automatically
starts Habitat Placement. Fresh N/A/no-target laser results are treated as
65536 mm for the distance-strafe checks. One adjustable timeout bounds the initial
strafe, stop delay, pulses, and checks. All added strafe settings default to
unconfigured. While enabled, the laser uses its high-accuracy profile with a
200 ms continuous intermeasurement period.

Habitat Pieces stores three independently adjustable pickup profiles. One
Habitat Pieces Start alternates the matching routes in this fixed order:
Pickup 1, Placement 1, Pickup 2, Placement 2, Pickup 3, Placement 3. Matching
placement profiles remain independently adjustable, including a selectable
front/rear return-line source; the migrated defaults are Front, Front, Rear.
Each placement starts with a fresh IMU heading capture, and its slide starts
seeking the bottom limit during the forward-to-slide drive.

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

Solar's initial right approach and final left front-line reacquisition use the
shared IMU Strafe controller. Retry-left and retry-right use direct open-loop
mecanum strafing at independently adjustable duties, with no IMU correction.
All stages retain their existing adjustable times/timeouts. Once either front
sensor sees tape, Solar follows the front line forward at its own adjustable
duty and duration. Stage the robot with the slider up, funnel mechanically
closed, and Solar Hook down/closed. Solar commands the hook's adjustable closed
angle at route start; after the forward line follow, it commands the adjustable
open angle and runs the funnel at an adjustable signed duty for an adjustable
duration before reporting Solar complete. Positive duty uses the existing
funnel-forward convention and negative duty uses reverse.

The ESP2 dashboard also has a `Tower Pieces` panel. Its Start button enters
`AUTONOMOUS_TOWER_PIECES`, follows the rear line backward, counts distinct LSS
LOW-to-HIGH transitions, and stops all four wheels on transition two. From the
same start instant, the stepper performs a configured relative upward jog at
the shared Tower up speed while the chassis route continues. It then
waits, performs an IMU-aligned right strafe for a configured duration, pauses,
uses the IMU turn controller to rotate clockwise through a configured angle,
pauses again, and drives backward for a configured duration. It then alternates
IMU-aligned right and left strafes, starting in an adjustable direction after
an adjustable stopped delay. The first pulse is 50% of its direction's normal
duration; later pulses use the full configured left/right duration. When either
back line sensor becomes HIGH, it waits for an adjustable stopped post-shimmy
delay. It can then perform an optional timed backward drive (`0 ms` skips it), waits,
opens the winch, waits, opens all three claws, waits for both its configured
pre-bottom delay and the initial lift to finish, and lowers the stepper to
the bottom limit. It then waits, closes all three claws, waits, raises the
stepper to the top limit, and closes the winch. There is no final line-following
stage. The shimmy direction, pre/post delays, right and left durations, and all
tail settings are adjustable in the panel and can be saved to NVS.

Tower Pieces strafe duty/correction/gains come directly from the shared IMU
Strafe panel, and its turn duty/gains/tolerances/timeout come directly from the
shared IMU Turn panel. Its mode-specific durations, clockwise angle, other
motion duties, optional final reverse, and initial lift distance remain `0`
until configured. Its
pause and delay stages default to `1000 ms`, and
both limit-search speeds default to `2000` driver microsteps per second. The
shared servo defaults are claw 1 open/closed `23/110`, claw 2 `40/100`, claw 3
`80/180`, and winch `0/180` degrees. Tower Pieces uses the same live angles as
the `Servo Control` panel, so panel changes update an active servo hold and later
sequence commands; `/api/claws/save` persists them to NVS.

An adjustable Tower Pieces start window ignores LSS HIGH readings during a
mode handoff. A HIGH held through the window must return LOW before it can
count as a crossing.

The dashboard also provides a `PegFinder` mode. It uses the IMU turn
controller to rotate clockwise through a configured angle, pauses, drives
backward for a configured duration, pauses again, drives forward for a
configured duration, and then runs the ESP1-owned funnel motor forward until
the ESP2 GPIO47 limit switch is pressed.
GPIO47 is LOW while released and HIGH while pressed. If the switch is not
pressed before the adjustable timeout, PegFinder stops with a fault. After a
configurable delay, it opens all three claws in a dashboard-selected order with
a configurable delay between each command, using the live angles from the
shared `Servo Control` panel. Between claw openings it can run adjustable short
left/right shake pulses to free pieces in the funnel. All three shake values
default to zero, which disables shaking until they are calibrated. After each
enabled left/right shake, the chassis waits stopped for an adjustable delay
before opening the next claw. It then waits
for another configurable delay and runs the funnel in reverse at an adjustable
duty for an adjustable duration. The
PegFinder turn output cap, Kp, Kd, tolerance, settling, timeout, and
measured yaw polarity all come from the shared IMU Turn panel. Its angle,
chassis and funnel duties, and timings
remain `0` (unconfigured) until the team enters verified values.

The `Time Trial` panel runs Autonomous Solar, Tower Pieces, and PegFinder in
that order. It uses the same live configuration objects as the three individual
panels, so applying or saving an individual setting also changes the combined
run. Pressing Time Trial Start first applies all three visible panels and the
shared servo angles. Its only separate settings are the delay after solar, an
optional timed right strafe before Tower Pieces, and the delay after Tower
Pieces. The transition strafe uses the shared IMU Strafe tuning. These
transition values default to `0`; a `0 ms` strafe skips
that motion. At the Tower Pieces-to-PegFinder handoff, the claws and winch are
commanded closed and their PWM outputs remain enabled.

The `Final competition` panel runs Autonomous Solar, all three interleaved
Habitat pickup/placement profiles, Tower Pieces, and PegFinder as one mode.
It assumes the slider is initially up, funnel closed, and Solar Hook
down/closed. Solar opens the hook and funnel before handing Habitat a forward
front-line follow. Each Habitat pickup profile
uses its adjustable LSS2/LSS3 start-ignore window. Placement profile 3 must be
configured to reacquire the rear line; after that search completes, Tower
Pieces begins its backward line follow and applies its own adjustable LSS
ignore window. The final mode then hands Tower completion to PegFinder.

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
signals, the VL53L0X V2 distance sensor on GPIO10/GPIO9, the GPIO3 Solar Hook
servo, and its side of the UART link.

ESP2 is the motion and mechanism node. It owns front line sensors, front motors,
logical four-wheel motion calculation, stepper, the remaining servos,
mechanism limit switches, and its side of the UART link. ESP2 applies
front-wheel commands locally and sends rear-wheel commands to ESP1.

The ESP2 dashboard includes the Solar Hook controls in the shared `Servo
Control` panel. Its Open and Closed angles default to `0/148` degrees and remain
independently adjustable. ESP1 drives it on GPIO3 with LEDC channel 6 using the
same timing and pulse range as the claw servos; the output starts detached and
disabled.

Both processors must initialize local actuator outputs disabled and stop local
motors if valid communication becomes stale.
