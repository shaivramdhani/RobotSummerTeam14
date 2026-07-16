# Two-ESP Drive Test

Use this procedure to flash both ESP32-S3 boards and drive-test the robot from
the ESP2 telemetry dashboard. Line sensing and line following are not part of
this test.

## Wiring Configured In Firmware

ESP2 owns the physical front motors, the right IR input, and the WiFi telemetry
dashboard:

| Signal | GPIO |
| --- | --- |
| `RightIRFiltered` | 9 |
| `PWMFL0` | 15 |
| `PWMFL1` | 16 |
| `PWMFR0` | 41 |
| `PWMFR1` | 42 |
| UART TX to ESP1 RX | 21 -> ESP1 GPIO 40 |
| UART RX from ESP1 TX | 40 <- ESP1 GPIO 21 |

ESP1 owns the physical back motors:

| Signal | GPIO |
| --- | --- |
| `PWMBL0` | 42 |
| `PWMBL1` | 41 |
| `PWMBR0` | 15 |
| `PWMBR1` | 16 |
| UART TX to ESP2 RX | 21 -> ESP2 GPIO 40 |
| UART RX from ESP2 TX | 40 <- ESP2 GPIO 21 |

Both ESPs use 115200 baud on the inter-ESP UART. Motor PWM is configured for
1000 Hz, 10-bit resolution, and a normalized `1.0` maximum test duty.

## Flash Both ESPs

Flash ESP1 first, then ESP2. Replace the upload ports with the actual serial
ports shown by PlatformIO.

```sh
pio run -e esp1 -t upload --upload-port /dev/ttyUSB_ESP1
pio run -e esp2 -t upload --upload-port /dev/ttyUSB_ESP2
```

You should flash both ESPs for drive testing. ESP2 hosts the web page and drives
the front wheels, but ESP1 is required for the back wheels.

## Connect To The Dashboard

1. Power both ESPs and confirm their grounds are common.
2. Connect to WiFi `Team14Robot`.
3. Use password `robotdebug`.
4. Open `http://192.168.4.1/`.
5. Confirm the page loads and the fault field says `none`.
6. Confirm ESP1 link/status becomes available. If it does not, check the UART
   cross-wiring and common ground.

## Individual Wheel Test

Keep the robot lifted so wheels cannot move it.

1. Select `FL`, duty `0.10`, and hold `Hold Forward`.
2. Hold `Hold Back` to spin the same wheel the other direction.
3. Repeat for `FR`, `BL`, and `BR`.
4. Press STOP between tests if anything looks wrong.

Expected command routing:

| Dashboard wheel | Physical output |
| --- | --- |
| `FL` | ESP2 `PWMFL0/PWMFL1` |
| `FR` | ESP2 `PWMFR0/PWMFR1` |
| `BL` | ESP1 `PWMBL0/PWMBL1` |
| `BR` | ESP1 `PWMBR0/PWMBR1` |

If a wheel spins backward, stop testing and change that wheel's compile-time
`forward_sign` in the owning `PinConfig.h`. Dashboard inversion applies to the
ESP2 front wheels; ESP1 back-wheel inversion should be changed in
`include/esp1/PinConfig.h`.

## Driving In All Directions

1. Set duty to `0.10`.
2. Hold each direction button briefly: FWD, BACK, LEFT, RIGHT, CW, CCW.
3. Increase duty only after all directions are correct at low duty.

The browser refreshes commands while a direction button is held. If commands
stop arriving, the deadman timeout stops the motors.

## Serial Alternative

Open the ESP2 serial monitor:

```sh
pio device monitor -e esp2 --port /dev/ttyUSB_ESP2
```

Commands:

```text
mode single-motor
motor test FL 0.10 1000
motor test FR 0.10 1000
motor test BL 0.10 1000
motor test BR 0.10 1000

mode distributed-drive
drive fwd 0.10 1000
drive back 0.10 1000
drive left 0.10 1000
drive right 0.10 1000
drive cw 0.10 1000
drive ccw 0.10 1000
stop
```

Serial `motor test` commands remain timed. The browser dashboard is the
press-and-hold motor test path.

## Do Not Use For This Test

Do not use `LINE_FOLLOW_TEST` while performing the motor-only drive test. Finish
wheel direction, inversion, stale-command, and UART checks before floor-testing
line following.

## Line-Following UI

The dashboard also exposes front line-sensor telemetry and line-following
PID/PD controls for the next bring-up step. See `docs/line-following-plan.md`.
