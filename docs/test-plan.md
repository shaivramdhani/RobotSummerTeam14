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
- Rear-drive laser/IR acquisition request round-trip and stale-command disable
  behavior, with the selected laser profile retained independently.
- Rear/side line-sensor packet round-trip for LSBL, LSBR, LSS, LSS2, and LSS3.
- VL53L0X snapshot packet round-trip, corruption rejection, and the
  standalone habitat-distance gate's locked configuration, invalid/stale-data,
  threshold latching, reset, and millisecond-wrap behavior.
- `HABITAT_PIECES` default-duty/configuration lockout, LSS2/LSS3 ignore interval,
  detection arming, LSS2-clockwise/LSS3-counter-clockwise alignment and
  opposite-sensor stop, timed reverse duty/duration, adjustable search/alignment
  timeout, distinct at-or-below-threshold laser-zone
  counting, per-profile initial count-ignore behavior, repeated/invalid
  measurement rejection, left/right strafe mixing,
  post-count stop delay, stopped exit-pulse checks, overall strafe timeout, and
  bottom-limit lowering, GPIO48 limit-switch approach, concurrent step-counted
  lift/reverse, opposite IMU strafe, rear-line stop, and placement handoff.
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
   configured/initialized `yes`, ranging `no`, and no increasing measurement
   sequence before a Habitat Pickup or Placement step starts.
3. Start a Habitat step and confirm ranging becomes `yes` and measurement and
   packet sequences increase. Stop or complete Habitat and confirm ranging
   returns to `no`. Let the ESP2 command become stale and confirm acquisition
   also stops.
4. Move a flat target through the intended working range. Confirm millimetre
   readings change monotonically and compare them with a physical measurement.
5. Test the actual habitat piece at the final sensor height and angle, including
   its edges, color/reflectance variation, and the robot's expected ambient
   lighting.
6. Disconnect the sensor with actuators disabled. Confirm `data_valid` becomes
   false or `sample_age_ms` grows past freshness while UART snapshot heartbeats
   alone do not restore freshness.
7. Record range performance for any future mission that consumes the laser.

For the read-only IR acquisition gate, keep the robot disabled or its wheels
raised:

1. Confirm IR acquisition is stopped at boot and in all standalone modes other
   than Solar.
2. Start standalone Solar and confirm acquisition becomes active. Repeat for
   the Solar stages of Time Trial and Final Competition.
3. Finish, fault, or stop Solar and confirm acquisition stops and detection is
   cleared. Let the rear command become stale and confirm ESP1 also disables
   acquisition locally.

Keep the wheels raised for the first `HABITAT_PIECES` gate test:

1. Verify ESP1 LSS2 GPIO11 and LSS3 GPIO12. Confirm telemetry shows both
   configured and fresh with correct white/black polarity. Confirm either can
   stop line following and select the alignment direction.
2. Prove a zero detection delay, zero timeout, timeout not longer than the
   delay, zero reverse duty/duration, missing distance-strafe direction, zero
   distance threshold/count/duty/timeout, or distance timeout above 30000 ms
   prevents an approach command.
3. With the wheels raised, Start and confirm line following begins immediately
   while `lss2_detection_armed=false`.
4. Present black to both LSS2 and LSS3 during the ignore interval and confirm
   the wheels keep following and neither input latches. At the configured
   delay, confirm the side-sensor gate arms.
5. Present black to LSS2 only. Confirm `LSS2_DETECTED`, an immediate all-wheel
   stop, then clockwise `SIDE_LINE_ALIGNING` at the profile's line-follow duty.
   Keep LSS3 white and confirm rotation continues; present LSS3 black and confirm
   another all-wheel stop before `REVERSING`.
6. Repeat with LSS3 first. Confirm `LSS3_DETECTED`, the immediate stop, then
   counter-clockwise rotation until LSS2 sees black. Verify both physical
   directions with wheels raised before field testing.
7. Present both sensors black in the same armed update. Confirm the robot stops,
   skips rotation because alignment is already complete, and then reverses.
8. Confirm the reverse continues for the full configured duration, then enters
   `DISTANCE_STRAFING` in the configured left/right direction at the configured
   duty. Confirm the high-accuracy laser profile remains selected and IMU
   heading hold corrects yaw during the strafe.
9. Disconnect or freeze the shared line-sensor packet before both sensors latch and
   confirm both processors stop their locally owned wheels through the normal
   command-expiration path.
10. During reverse, confirm side-sensor changes no longer alter the latched timed
   move, while stale rear status or a failed rear command still stops it.
11. Keep both sensors white and confirm the overall side-line timeout stops all
    four wheels. Repeat after one sensor starts alignment while the opposite
    sensor remains white; confirm the same timeout enters `FAULT` with
    `timed_out=true`.
12. With the distance strafe active, supply consecutive valid measurements at
    or below the configured threshold during the profile's count-ignore window.
    Confirm the count remains zero and the ignore telemetry counts down. Hold
    the reading in-zone past the gate and confirm it still does not count until
    one fresh above-threshold reading rearms the detector and a later fresh
    at-or-below reading enters the zone. Then confirm consecutive in-zone
    measurements count only once.
13. Confirm repeated UART packets carrying the same measurement sequence never
    increment the count. Confirm a fresh N/A/no-target result displays as the
    65536 mm sentinel and acts above any configurable threshold. Confirm a
    stale or frozen measurement stream supplies no new sample and does not
    prevent the bounded strafe from starting.
14. Reach the target count and confirm `DISTANCE_ZONE_COUNT_REACHED`, an
    all-wheel stop for the configured delay, and then one timed strafe pulse.
    Confirm the pulse uses its independent exit duty rather than the long
    counting-strafe duty. Confirm the pulse ends with all wheels stopped and no
    decision is made until a new measurement sequence arrives. Supply
    at-or-below and confirm another pulse; supply above and confirm
    `DISTANCE_EXIT_REACHED` and `LOWER_SLIDE`. Repeat without completing the
    distance phases and confirm `DISTANCE_STRAFE_TIMEOUT`, `FAULT`, and
    all-wheel stop.
15. Confirm the slide moves down only at the configured limit-search speed and
    stops at the debounced bottom switch. Confirm a missing bottom input reaches
    `SLIDE_DOWN_TIMEOUT` and stops both the chassis and slide.
16. Confirm `APPROACH_PIECE` drives forward at the configured duty regardless
    of laser readings. Press the active-high GPIO48 habitat-piece limit switch
    and confirm an immediate all-wheel stop before the configured pre-lift
    reverse. Confirm that reverse uses the pickup reverse duty, ends at its
    independent duration, and stops before the lift begins. Leave the switch
    released and confirm `APPROACH_LIMIT_TIMEOUT` faults and stops all wheels.
17. Confirm the slide starts lifting by the configured steps while every wheel
    remains stopped for the configured lift-start delay. After that delay,
    confirm the lift continues while the chassis performs the configured timed
    reverse. Test lift completion both before and after reverse completion; a
    stopped or timed-out stepper must fault safely.
18. Confirm rear-line reacquisition strafes at its independent duty through IMU
    heading hold opposite the initial distance-strafe direction. Either rear sensor must stop all
    wheels. If the lift is unfinished, confirm the chassis remains stopped
    until the lift completes. Test stale rear data and reacquisition timeout.
19. With Habitat Placement fully configured, confirm completion automatically
    enters `HABITAT_PLACEMENT` and begins its rear-line-follow start sequence.

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
10. Run Solar's initial right approach and final front-line reacquisition, the
    Tower initial strafe and both shimmy directions, and the optional Time Trial
    transition strafe. Confirm the reported autonomous mode does not change to
    `IMU_STRAFE_TEST`, each captures a heading target, and shared Stage 3
    duty/gain changes affect them. Separately run Solar retry-left and
    retry-right; confirm they use their independently adjustable duties and do
    not enter IMU heading hold.
11. For Tower shimmy, select Left and Right in separate runs. Confirm the
    chassis stays stopped for the configured pre-delay, the selected direction
    runs first for exactly half its normal duration, all later pulses use full
    left/right durations, and back-line detection stops motion for the complete
    post-delay before the optional final reverse.
12. Run the Tower and PegFinder clockwise turns with low, verified angles.
    Confirm their panels contain angles rather than timed-turn settings and
    that both turns use the shared Stage 2 duty/gains/tolerance/timeout.
13. Start Tower with a small calibrated initial lift. Confirm the stepper begins
    its relative upward jog in the first rear-line-follow phase while the wheels
    continue updating. Confirm the later bottom-limit search waits if the jog is
    unfinished, and that insufficient upward travel or an early stop rejects or
    faults the route with all outputs stopped.
14. During each autonomous IMU strafe/turn family, briefly interrupt SDA or
    SCL. Confirm all four wheel commands become disabled, the mission phase and
    its elapsed time stop advancing, the saved heading/target remain fixed,
    and motion resumes only after three new fresh samples.
15. Keep the IMU disconnected beyond the displayed recovery limit. Confirm the
    owning autonomy enters its existing terminal `IMU_UNAVAILABLE` fault and
    does not restart.
16. With wheels raised, power-cycle the IMU during an autonomous IMU phase.
    Confirm runtime-register verification becomes false and the phase never
    resumes using the MPU reset defaults.

## Habitat Placement Bring-Up

1. Keep wheels raised. Verify the Habitat Pusher signal is GPIO5/LEDC 7 and
   the Winch signal is GPIO6/MCPWM unit 0, timer 0, generator A. Confirm neither
   output produces servo pulses during boot or while disabled. Then calibrate
   pusher Closed and Open so the physical close-to-open motion is CW.
2. Select Habitat Pieces pickup profiles 1, 2, and 3 in turn and enter each
   complete pickup route. Select the three matching Habitat Placement profiles
   and enter every placement duty, duration, angle, timeout, slide speed,
   pusher-open settle time, and Front/Rear return source. Confirm each selector
   reloads its own saved values. Confirm Habitat Pieces Start Ready remains
   false if any pickup/placement profile or required sensor, IMU, stepper limit,
   pusher target, or link is absent. For each pickup profile, confirm the slide
   begins lowering with the first front-line-follow step, reaches the bottom
   safely while the chassis route continues, and faults/stops if its timeout
   expires.
3. Exercise the route one phase at a time with conservative settings. Confirm
   the initial heading is captured before rear-line motion begins, LSS1 stops
   reverse line-following, and each delay holds all wheels stopped.
4. After LSS1 and its delay, confirm the robot returns to the displayed initial
   heading within the dedicated timeout, then strafes right at the configured
   duty for the full configured duration before beginning the CCW turn. Disturb
   the chassis gently and confirm the IMU correction holds the strafe heading.
5. Confirm the CCW target equals the initial heading plus or minus the
   configured offset for the verified IMU polarity and remains unchanged
   throughout line following, the return turn, right strafe, and CCW turn.
   Confirm all IMU turns obey their configured timeout. Confirm the slide starts
   moving down at the beginning of the forward-to-slide drive, continues while
   the chassis drives, and reaches the bottom switch before the pusher opens.
6. After the CW turn, confirm the robot runs the configured backward duty for
   its full duration, then the configured left-strafe duty for its full
   duration, then strafes right at the same duty for its independently
   configured duration before beginning the existing stopped delay and forward
   drive. Confirm both strafes hold heading with the IMU.
7. Start from Habitat Pieces. Confirm telemetry and physical motion alternate
   Pickup 1, Placement 1, Pickup 2, Placement 2, Pickup 3, Placement 3, with an
   all-wheel stop at every handoff. Confirm each placement captures a new IMU
   heading rather than retaining Placement 1's heading.
8. With return sources set to Front, Front, Rear, confirm Placements 1 and 2
   stop their return strafe when either front sensor reads black. Confirm
   Placement 3 ignores the front condition and stops on either rear sensor.
   Repeat with a different source selection to prove the setting is per
   profile. Confirm each return strafe holds heading with the IMU.
9. Confirm the profile-specific pickup direction/duties and placement values
   appear in sequence on telemetry and completion reports three completed
   pickup/placement pairs.

## Final Competition Bring-Up

1. Keep the wheels raised and configure Solar's front-line exit duty/duration
   and signed funnel-open duty/duration, all six Habitat profiles, Tower's
   start-ignore window, and PegFinder shake.
   Confirm Start is rejected unless Placement 3 returns to the rear line.
2. Stage the slider up, funnel closed, and Solar Hook down/closed. Confirm boot
   still leaves all actuator outputs disabled, then Solar Start commands the
   hook's configured closed angle.
3. Confirm Solar's final strafe stops on a front sensor and its forward PID
   runs for the configured duration. Verify the hook then moves to its open
   angle and the funnel runs in the calibrated physical opening direction for
   exactly the configured duration before Habitat Pickup 1 begins.
4. Present LSS2/LSS3 HIGH during each Habitat start-ignore window and confirm
   neither reading changes the route until its profile's gate opens.
5. After Placement 3 finds the rear line, confirm the chassis stops for the
   ownership handoff and Tower begins backward rear-line following. Hold LSS
   HIGH through Tower's ignore window; verify it must return LOW before a new
   HIGH can count.
6. In PegFinder, confirm a left pulse followed by a right pulse occurs between
   claw openings 1/2 and 2/3, with stopped transitions and the configured duty
   and durations. Confirm the configured stopped post-shake delay completes
   before the next claw opens. Confirm all-zero shake values skip both shake
   and post-shake-delay states.
7. Force one included-mode timeout at a time and confirm Final Competition
   enters `FAULT`, stops chassis/funnel/stepper outputs, and does not advance.
8. In separate trials, withhold LSS1, the bottom limit, IMU data, ESP1 status,
    or the configured return line. Each condition must stop or time out without
    advancing to the next motion phase or profile.
