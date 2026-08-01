# Test Plan

## Required Checks

Run these before merging firmware scaffold changes:

```sh
pio run -e esp1
pio run -e esp2
pio test -e native
```

The GitHub Actions workflow runs the same checks.

## Native Tests

Native tests cover pure, hardware-independent logic:

- Digital front line sensor truth table and line-loss history.
- PID proportional, integral, derivative, clamp, and reset behavior.
- Differential motor mixing and steering polarity.
- IMU turn configuration lockout, continuous relative target capture, PD output
  and clamp behavior, yaw-rate damping, dual angle/rate settling conditions,
  settling reset, timeout, explicit stop, mode policy, and telemetry JSON.
- Rear-drive command packet validation and stale/explicit-stop behavior.
- Rear/side line-sensor packet round-trip for LSBL, LSBR, LSS, LSS2, and the
  telemetry-only LSS3 path.
- VL53L0X snapshot packet round-trip, corruption rejection, and the
  standalone habitat-distance gate's locked configuration, invalid/stale-data,
  threshold latching, reset, and millisecond-wrap behavior.
- `HABITAT_PIECES` default-duty/configuration lockout, LSS2 ignore interval,
  detection arming, black-line transition, timed reverse duty/duration,
  adjustable search timeout, and terminal stop behavior.
- Mission transition logic after transitions are defined.

Native tests must not include Arduino, GPIO, PWM, UART peripherals, or motor
drivers.

## Firmware Build Checks

ESP1 and ESP2 firmware builds verify that board-specific entry points, headers,
and compile-time ownership boundaries remain valid.

## Future Hardware Tests

Add hardware-in-loop tests only after safe actuator infrastructure exists.
Hardware tests must begin by proving disabled startup states and stale-command
shutdown behavior.

For the read-only VL53L0X bring-up, keep the robot disabled:

1. Verify ESP1 SDA is GPIO10, SCL is GPIO9, grounds are common, and the exact
   carrier's power and pull-up voltages are safe.
2. Boot both processors and confirm the ESP2 dashboard reports address `0x29`,
   configured/initialized/ranging `yes`, and increasing measurement and packet
   sequences.
3. Move a flat target through the intended working range. Confirm millimetre
   readings change monotonically and compare them with a physical measurement.
4. Test the actual habitat piece at the final sensor height and angle, including
   its edges, color/reflectance variation, and the robot's expected ambient
   lighting.
5. Disconnect the sensor with actuators disabled. Confirm `data_valid` becomes
   false or `sample_age_ms` grows past freshness while UART snapshot heartbeats
   alone do not restore freshness.
6. Record range performance for any future mission that consumes the laser.

Keep the wheels raised for the first `HABITAT_PIECES` gate test:

1. Verify ESP1 LSS2 GPIO11 and LSS3 GPIO12. Confirm telemetry shows both
   configured and fresh with correct white/black polarity; confirm only LSS2
   affects Habitat Pieces.
2. Prove a zero detection delay, zero timeout, timeout not longer than the
   delay, zero reverse duty, or zero reverse duration prevents an approach
   command.
3. With the wheels raised, Start and confirm line following begins immediately
   while `lss2_detection_armed=false`.
4. Present black to LSS2 during the ignore interval and confirm the wheels keep
   following. At the configured delay, confirm detection arms and a currently
   black input transitions to straight reverse at the configured duty.
5. Repeat with LSS2 white through the delay, then present black and confirm
   `LSS2_BLACK_DETECTED` and `REVERSING`. Confirm the reverse continues for the
   full configured duration, then enters `COMPLETE` and latches stop.
6. Disconnect or freeze the LSS2 sensor packet before detection and confirm both
   processors stop their locally owned wheels through the normal command
   expiration path.
7. During reverse, confirm LSS2 changes no longer alter the latched timed move,
   while stale rear status or a failed rear command still stops it.
8. Keep LSS2 white and confirm the search timeout stops all four wheels,
   enters `FAULT`, and reports `timed_out=true`.

For the read-only IMU soak test:

1. Keep the robot disabled and record the current mode, heading, IMU-turn state,
   and all four applied wheel commands.
2. Press `Reset soak counters`; confirm attempts resume increasing at
   approximately the existing 10 ms acquisition period while the displayed
   lifetime Reads OK and Reads Failed counters do not reset.
3. Confirm the lifetime read counters never decrease during the current boot,
   maximum completed duration never decreases between resets, a successful
   read updates its timestamp, and a failure increments both failed and
   consecutive counts.
4. Press `Refresh values`; confirm it only reloads telemetry.
5. Confirm both buttons leave the recorded mode, heading, turn state, and wheel
   commands unchanged. Repeat while another safe test mode is selected to
   confirm the soak view is independent of robot mode.
6. Run for the desired soak interval and confirm memory use remains stable; no
   per-read log or queue is retained.
7. Interrupt each available IMU failure mechanism in turn and confirm the
   current and retained disconnect reasons identify the exact condition, the
   disconnect count/timestamp update, and the same reason appears once in the
   serial log at the disconnect transition.

For Stage 2, keep the wheels raised for the first tests:

1. Confirm Apply rejects zero/incomplete tuning and Start rejects an unhealthy
   or stale IMU and rear link.
2. Configure a deliberately low maximum rotation duty and all other required
   values.
3. Request +90 and confirm the initial measured heading moves toward the target.
   If it moves away, press Stop and reverse only `yaw_command_polarity`.
4. Confirm Stop immediately disables front and rear wheel commands.
5. Confirm disconnecting the IMU or rear link during a turn faults and stops.
   An IMU bus stall must make the published snapshot stale and fault the turn
   without stretching the core-1 motion-loop interval by the duration of the
   stalled I2C transaction.
6. Before reproducing an intermittent front/rear stop mismatch, use the
   dashboard `Reset before run` control. After the wheels have stopped, use
   `Refresh after failure` and retain the full motion-diagnostics JSON. Confirm
   it contains loop/web/IMU maximum durations, front LEDC readback, all four
   command values, rear status ages, and the terminal trigger.
7. Confirm completion occurs only after both angle error and yaw rate remain
   within limits for the full settling time.
8. Repeat -90 before any floor test. Tune on a stand before increasing the duty
   clamp.
9. Press Reset angle while idle and confirm the request is acknowledged within
   subsequent telemetry before Start is accepted.

For Stage 3, keep the wheels raised for initial tests:

1. Confirm the five zero/default settings lock motion, and Apply rejects a
   strafe-duty plus correction-duty sum above the hardware cap.
2. Enter a low strafe duty, low correction limit, Kp, Kd, and the polarity
   verified during Stage 2; Apply and Save them.
3. Hold Left and confirm target heading equals the heading captured at start.
   Keep holding while rotating the chassis slightly by hand and confirm target
   heading does not change.
4. Confirm the correction initially drives measured heading back toward the
   target. If it drives away, release immediately and correct the polarity
   before continuing.
5. Confirm Hold Right captures a new target and produces the opposite lateral
   wheel pattern while preserving the same yaw convention.
6. Release each button and confirm front and rear commands stop immediately.
   Close or suspend the dashboard while held and confirm the 700 ms deadman
   stops the controller without resuming.
7. Disconnect or stall the IMU and make the ESP1 status link stale in separate
   tests; either condition must fault and stop all four wheels.
8. Confirm Reset angle is rejected while a strafe is active and a strafe start
   is rejected while a reset acknowledgement is pending.
9. Run normal manual drive, Stage 2 turns, and front/rear line following to
   confirm none receives an IMU-strafe yaw correction.
10. Run every Solar lateral stage, the Tower initial strafe and both shimmy
    directions, and the optional Time Trial transition strafe. Confirm the
    reported autonomous mode does not change to `IMU_STRAFE_TEST`, each strafe
    captures a heading target, and the shared Stage 3 duty/gain changes affect
    every one of those motions.
11. Run the Tower and PegFinder clockwise turns with low, verified angles.
    Confirm their panels contain angles rather than timed-turn settings and
    that both turns use the shared Stage 2 duty/gains/tolerance/timeout.
12. During each autonomous IMU strafe/turn family, briefly interrupt SDA or
    SCL. Confirm all four wheel commands become disabled, the mission phase and
    its elapsed time stop advancing, the saved heading/target remain fixed,
    and motion resumes only after three new fresh samples.
13. Keep the IMU disconnected beyond the displayed recovery limit. Confirm the
    owning autonomy enters its existing terminal `IMU_UNAVAILABLE` fault and
    does not restart.
14. With wheels raised, power-cycle the IMU during an autonomous IMU phase.
    Confirm runtime-register verification becomes false and the phase never
    resumes using the MPU reset defaults.

## Habitat Placement Bring-Up

1. Keep wheels raised. Verify the Habitat Pusher signal is GPIO5/LEDC 7 and
   the Winch signal is GPIO6/MCPWM unit 0, timer 0, generator A. Confirm neither
   output produces servo pulses during boot or while disabled. Then calibrate
   pusher Closed and Open so the physical close-to-open motion is CW.
2. Enter every Habitat Placement duty, duration, angle, timeout, slide speed,
   and pusher-open settle time. Confirm Start Ready remains false if any field,
   LSS1/rear/front sensor, IMU, stepper limit, pusher target, or link is absent.
3. Exercise the route one phase at a time with conservative settings. Confirm
   LSS1 stops reverse line-following, each delay holds all wheels stopped, both
   turns obey their own timeout, and the slide stops at the bottom switch.
4. At CCW entry, record the displayed saved start/target headings. Confirm the
   target equals the start plus or minus the configured offset for the verified
   IMU polarity and remains unchanged throughout that turn.
5. After the CW turn, confirm the robot runs the configured backward duty for
   its full duration, then the configured left-strafe duty for its full
   duration, before beginning the existing stopped delay.
6. Confirm the final right strafe stops when either front sensor reads black,
   closes the Habitat Pusher, and remains stopped in Complete.
7. In separate trials, withhold LSS1, the bottom limit, IMU data, ESP1 status,
   and the final front line. Each condition must stop or time out without
   advancing to the next motion phase.
