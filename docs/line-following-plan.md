# Digital Line Following

The first functional feature is digital two-sensor tape following using ESP2
front sensors `LSFL` and `LSFR` and all four mecanum drive motors. ESP2 owns the
line follower and front motors. ESP1 receives rear motor commands from ESP2 and
applies the rear motors it owns.

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

The sensors are digital. LOW means black tape. HIGH means white or non-tape
surface. Firmware uses `pinMode(..., INPUT)` only; it does not enable
`INPUT_PULLUP`.

| LSFL | LSFR | Meaning | Error | Line visible | Last side |
| --- | --- | --- | --- | --- | --- |
| LOW | LOW | both sensors on tape | `0` | true | preserve previous side |
| LOW | HIGH | left sensor on tape | `+1` | true | `+1` |
| HIGH | LOW | right sensor on tape | `-1` | true | `-1` |
| HIGH | HIGH after `+1` history | line lost to left-history side | `+5` | false | `+1` |
| HIGH | HIGH after `-1` history | line lost to right-history side | `-5` | false | `-1` |
| HIGH | HIGH without history | unsafe/no history | `0` | false | `0` |

Positive error is the exact logical result specified for `LSFL` LOW and `LSFR`
HIGH. Physical steering direction is handled separately by `steeringPolarity`.

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
- `maximumDuty`
- `maximumCorrection`
- `integralLimit`
- `derivativeLimit`
- `derivativeFilterCoefficient`
- `steeringPolarity`
- `controlPeriodMs`
- `rearCommandTimeoutMs`
- `initialLineSearchTimeoutMs`
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

If either side exceeds `maximumDuty`, the pair is scaled back so the commands
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

Malformed values are rejected. `lf max-duty` cannot exceed the verified
`maximum_safe_test_duty` in `include/esp2/PinConfig.h`. Because that value is
currently `0.0`, any movement command is rejected until the team verifies a safe
maximum duty.

## Telemetry

When telemetry is enabled, ESP2 prints about 10 Hz:

```text
lf_csv,timestamp_ms,left_black,right_black,error,line_visible,
proportional_term,integral_term,derivative_term,correction,
front_left_command,front_right_command,back_left_command,back_right_command,
rear_command_age_ms,rear_link_healthy
```

Sensor-only mode prints:

```text
sensor,timestamp_ms,left_black,right_black
```

## Bring-Up Procedure

1. Raise the robot so the wheels cannot drive it away.
2. Fill in verified GPIOs, UART pins/baud, PWM resources, motor truth table,
   per-wheel `forward_sign`, and `maximum_safe_test_duty`.
3. Build and upload both processors.
4. Run `lf test sensor` and confirm LOW on black tape and HIGH on white surface.
5. Run `lf test motor <wheel> <low-duty>` one wheel at a time and correct
   `forward_sign` values until each wheel moves forward.
6. Run `lf test drive <low-duty>` to confirm ESP2-to-ESP1 rear drive commands.
7. Place the robot on a simple straight tape course.
8. Set `lf ki 0` and `lf kd 0`.
9. Set a low `lf speed` slightly above the motors' reliable movement threshold.
10. Increase `lf kp` until the robot corrects strongly enough.
11. If it oscillates, add `lf kd` gradually.
12. Increase speed only after low-speed behavior is reliable.
13. Add small `Ki` only if a persistent one-sided bias remains after motor
    calibration.
14. Re-test line-loss recovery and rear communication timeout.
