# Digital Line Following

The first functional feature is digital two-sensor tape following using ESP2
front sensors `LSFL` and `LSFR` and all four mecanum drive motors. ESP2 owns the
line follower and front motors. ESP1 receives rear motor commands from ESP2 and
applies the rear motors it owns.

Reverse rear following is also supported. ESP1 samples rear digital sensors
LSBL (GPIO17) and LSBR (GPIO18) every `10 ms` and sends a CRC-protected
`SensorSnapshot` to ESP2. ESP2 still owns the follower, four-wheel calculation,
and all motion commands. The robot travels backward in this mode.

## Architecture

ESP2 uses one coherent fixed-period control task:

1. Read `LSFL` and `LSFR` once.
2. Convert the digital readings into a `LineObservation`.
3. Update the PID controller.
4. Mix the correction into four wheel commands.
5. Apply the front wheel commands locally.
6. Send rear wheel commands to ESP1.
7. Publish optional rate-limited telemetry.

The legacy design with separate ADC sampling, line-state classification, and
analog-error FreeRTOS tasks is behavioral reference only. Do not copy that task
architecture.

## Digital Sensor Truth Table

The sensors are digital. HIGH means black tape. LOW means white or non-tape
surface. Firmware uses `pinMode(..., INPUT)` only; it does not enable
`INPUT_PULLUP`.

| LSFL | LSFR | Meaning | Error | Line visible | Last side |
| --- | --- | --- | --- | --- | --- |
| HIGH | HIGH | both sensors on tape | `0` | true | preserve previous side |
| LOW | HIGH | right sensor on tape | `-1` | true | `-1` |
| HIGH | LOW | left sensor on tape | `+1` | true | `+1` |
| LOW | LOW after `+1` history | line lost to left-history side | `+5` | false | `+1` |
| LOW | LOW after `-1` history | line lost to right-history side | `-5` | false | `-1` |
| LOW | LOW without history | unsafe/no history | `0` | false | `0` |

Positive error is the exact logical result specified for `LSFL` HIGH and
`LSFR` LOW. Physical steering direction is handled separately by
`steeringPolarity`.

Reverse rear mode applies the identical table in the direction-of-travel frame:
physical LSBR is the logical left input and physical LSBL is the logical right
input. This swap accounts for viewing the robot after a 180-degree change in
travel direction.

## PID Controller

The controller computes:

```text
correction = Kp * error + Ki * integral + Kd * derivative
```

Defaults are conservative: `Ki = 0`, `Kd = 0`, and `baseDuty = 0`. The first
physical test should be proportional-only. Integral accumulates only while the
line is visible and `abs(error) <= 1`; it resets while the line is lost.
Derivative uses actual elapsed time in seconds and is clamped before it can
create a large spike from `+/-1` to `+/-5`.

Runtime tunables live in `LineFollowerConfig`:

- `kp`, `ki`, `kd`
- `baseDuty`
- `maxDuty`
- `maxCorrection`
- `integralLimit`
- `derivativeLimit`
- `derivativeFilterAlpha`
- `steeringPolarity`
- `controlPeriodMs`
- `remoteCommandTimeoutMs`
- `telemetryEnabled`

## Motor Mixing

The line follower uses differential-style steering:

```text
signedCorrection = steeringPolarity * correction
leftSideCommand  = baseDuty - signedCorrection
rightSideCommand = baseDuty + signedCorrection
```

The left command goes to front-left and back-left. The right command goes to
front-right and back-right. Mixing produces logical signed normalized commands;
per-wheel direction signs and H-bridge PWM truth tables stay in each ESP's pin
configuration.

Reverse rear following uses the same correction math after swapping the rear
sensor sides, but applies `baseDuty` as a negative command. A positive value in
the rear Base control is therefore a speed magnitude; telemetry exposes both
that magnitude and the effective negative base command.

If either side exceeds `maxDuty`, the pair is scaled back so the commands
remain inside the configured limit.

## Serial Commands on ESP2

Use the ESP2 serial monitor at 115200 baud.

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
lf integral-limit <value>
lf derivative-limit <value>
lf derivative-alpha <value>
lf polarity <1|-1>
lf period-ms <integer>
lf timeout-ms <integer>
lf telemetry on
lf telemetry off
lf reset
```

Malformed values are rejected. `lf max-duty` cannot exceed the verified
`maximum_safe_test_duty` in `include/esp2/PinConfig.h`. For the current drive
test profile that value is `1.0`, so use the dashboard/serial duty setting
carefully and start below the desired test value.

Rear commands use an independent `LineFollowerConfig`:

```text
mode rear-line-sensor
rear-line status
rlf start [duration-ms]
rlf stop
rlf status
rlf reset
rlf telemetry on|off
rlf <tuning-name> <value>
```

`REAR_LINE_SENSOR_TEST` never enables actuators. `REAR_LINE_FOLLOW_TEST`
requires both rear sensors to be configured, a fresh rear snapshot, fresh ESP1
health, both ESP2 front motors, and the distributed rear-motor link. Stale rear
sensor data stops all wheels. On first initialization, rear tunables copy the
front tunables; after that, `rlf` commands, the Reverse Rear Line Following
dashboard panel, and separate NVS keys maintain them independently.

## Telemetry

When telemetry is enabled, ESP2 prints about 10 Hz:

```text
lf_csv,timestamp_ms,current_mode,line_follower_enabled,lsfl_level,lsfr_level,
left_black,right_black,error,last_known_side,line_visible,has_history,
kp,ki,kd,proportional_term,integral_term,derivative_term,correction,
steering_polarity,base_duty,max_duty,max_correction,
front_left_command,front_right_command,back_left_command,back_right_command,
rear_link_healthy,rear_command_sequence,rear_command_age_ms
```

Sensor-only mode prints:

```text
sensor,timestamp_ms,lsfl_level,lsfr_level,left_black,right_black,error,line_visible
```

Rear sensor-only mode prints:

```text
rear_sensor,timestamp_ms,lsbl_level,lsbr_level,logical_left_black,logical_right_black,error,
line_visible,configured,data_fresh,snapshot_sequence,snapshot_age_ms
```

Rear-follow telemetry starts with `rlf_csv` and reports the independent rear
PID and wheel-command fields. Its logical-left/right booleans use LSBR/LSBL,
respectively, followed by rear snapshot configuration, freshness, sequence,
and age.

## Bring-Up Procedure

1. Raise the robot so the wheels cannot drive it away.
2. Fill in verified GPIOs, UART pins/baud, PWM resources, motor truth table,
   per-wheel `forward_sign`, and `maximum_safe_test_duty`.
3. Build and upload both processors.
4. Run `line status` and confirm HIGH on black tape and LOW on white surface.
5. Run `motor test <wheel> <low-duty> <ms>` one wheel at a time and correct
   `forward_sign` values until each wheel moves forward.
6. Run `drive fwd <low-duty> <ms>` to confirm ESP2-to-ESP1 rear drive commands.
7. Place the robot on a simple straight tape course.
8. Set `lf ki 0` and `lf kd 0`.
9. Set a low `lf speed` slightly above the motors' reliable movement threshold.
10. Increase `lf kp` until the robot corrects strongly enough.
11. If it oscillates, add `lf kd` gradually.
12. Increase speed only after low-speed behavior is reliable.
13. Add small `Ki` only if a persistent one-sided bias remains after motor
    calibration.
14. Re-test line-loss recovery and rear communication timeout.
