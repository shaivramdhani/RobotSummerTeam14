# Safety

Runtime motor-moving behavior now exists behind explicit commands and verified
hardware configuration. With the current TODO configuration, motor outputs remain
disabled and line following refuses to start.

## Startup

- Initialize all actuator outputs disabled.
- The ESP1 Solar Hook servo initializes detached on GPIO3. Its Open/Closed
  angles default to `0/148` degrees, and the dashboard still requires fresh
  ESP1 hardware status before accepting movement.
- Do not attach PWM to motors until GPIOs, PWM resources, frequencies,
  H-bridge mode, direction sign, and disabled states are confirmed.
- High-level mission code must not directly access GPIO or PWM.
- ESP2 line following requires `lf start`; motors do not start at boot.
- IMU turn tuning defaults to zero/unconfigured; a dashboard turn cannot start
  until every controller value and mounting polarity is explicitly configured.

## Communication

- Every actuator command must expire.
- Both processors must disable local motor outputs if valid communication
  becomes stale.
- Communication packets must include version, message type, sequence number, and
  integrity check.

## Control Loops

- Do not use blocking `delay()` in operational code.
- Use `vTaskDelayUntil` with explicit millisecond-to-tick conversion for
  periodic FreeRTOS work.
- Do not dynamically allocate memory in control loops.
- Pass immutable snapshots and messages between tasks.
- Runtime VL53L0X I2C transactions execute only in ESP1's shared
  sensor-acquisition task. Readiness is polled; operational code never waits
  for a ranging cycle to complete.
- VL53L0X ranging is enabled only for active Habitat Pickup/Placement, and IR
  sampling is enabled only for active Solar. Both acquisition requests expire
  with the rear command timeout, so ESP1 stops them if ESP2 communication goes
  stale.
- Laser UART heartbeats do not make old measurements fresh. ESP2 freshness is
  based on arrival of a changed measurement sequence.
- `HABITAT_PIECES` line-follows immediately and ignores LSS2/LSS3 HIGH readings
  for an explicitly configured nonzero start window. A fresh black reading from
  either sensor stops all four wheels for one control update. LSS2 then selects
  clockwise rotation until LSS3 reads black; LSS3 selects counter-clockwise
  rotation until LSS2 reads black. A second all-wheel stop separates alignment
  from the bounded straight-backward move. Rotation uses the profile's existing
  adjustable line-follow duty and the chassis mixer's physical yaw convention.
  Reverse duty and duration are locked at zero until configured; completion
  begins the separately bounded distance-zone strafe. Direction, threshold,
  target count, duty, post-count stop delay, independent exit-pulse duty,
  exit-pulse duration, and timeout are locked until explicitly configured. The
  per-profile distance-count ignore duration may be zero, but it must fit
  inside the bounded distance-strafe timeout. The
  overall side-line search/alignment timeout must be longer than the detection delay.
- Habitat Pieces requires configured LSS2 and LSS3 inputs, fresh shared ESP1 sensor
  snapshot, valid shared IMU heading-hold tuning, and a healthy IMU at Start.
  Missing configuration, stale side-sensor data before alignment completes,
  rear-command failure, or side-line search/alignment timeout stops all four
  wheels and faults the route. No detection delay,
  search timeout, reverse duty/duration, distance direction,
  threshold, count, duty, post-count delay, exit-pulse duty/duration, or
  distance timeout is compiled in; all must be explicitly configured. The
  independent exit duty inherits a legacy saved counting-strafe duty until it
  is explicitly saved. During the timed reverse, continued side-
  sensor data is not required, but motor and communication safety gates remain
  active. The existing overall timeout bounds both initial side-line search and
  rotation toward the opposite sensor. Distance strafing uses the shared IMU
  heading-hold controller. During
  that step, only new, fresh high-accuracy measurement sequences affect the
  counter. Only entries at or below threshold count, and an above-threshold
  sample rearms the counter. After the count, each timed exit pulse ends with
  an all-wheel stop before a new measurement is evaluated. A fresh N/A/no-target
  result is substituted with 65536 mm and satisfies the exit check, while an
  unavailable or stale stream supplies no sample. Repeated sequences cannot
  increment the count or satisfy a post-pulse check, and the configured timeout
  bounds the counting strafe. Expiry stops the distance motion and continues
  through the post-count delay and exit pulse/check sequence with a fresh
  bound. If that exit bound expires, the route continues to the slide/approach
  pickup-tail gate without faulting.
  A bounded bottom-limit search begins when each pickup profile starts and runs
  concurrently with the early chassis route. Clearing the final zone begins the separately bounded
  active-high GPIO48 limit-switch approach immediately if the slide is already
  down, or holds the chassis stopped until that search finishes. A
  duration-bounded pre-lift reverse stops before the step-counted lift begins.
  The lift begins while every wheel remains stopped for the configured
  lift-start delay, then continues concurrently with the bounded reverse and
  opposite-direction IMU rear-line strafe at its independent duty. Either rear
  sensor stops chassis
  motion; the route waits stopped for an unfinished lift before handing off to
  Habitat Placement. An approach timeout stops the forward command, records
  `APPROACH_LIMIT_TIMEOUT`, and continues through the bounded reverse/lift
  pickup tail. Stepper-command failure, conflicting limits, stale rear line
  data, or a slide, lift, or reacquisition timeout stops both the chassis and
  slide.
- Habitat Pieces Start validates all three pickup profiles and all three
  matching placement profiles before any motion. They alternate strictly in
  numeric order, with an explicit all-wheel stop at each handoff. Each
  placement's Front/Rear return source is saved and validated per profile. The
  lower-limit search starts concurrently with the forward-to-slide drive, so
  the configured stepper timeout is measured from that command. A stopped
  stepper before the bottom limit, timeout, conflicting limits, stale required
  line data, or an IMU/link failure stops both chassis and slide.
- Slider-up and funnel-closed are physical staging assumptions, not boot-time
  actuator commands or sensed conditions; all outputs still initialize
  disabled. Solar Start requires configured hook targets and funnel hardware,
  commands the hook closed/down, and rejects an unset final funnel duty or
  duration. The final signed funnel command is capped, refreshed before its
  communication timeout, stopped at the configured duration, and stopped on
  any link or mode fault. The initial panel-contact strafe remains bounded;
  front-only contact enters one bounded retry, while no-front contact continues
  into the bounded post-contact forward drive. An expired retry also continues
  forward without permitting another retry. Stale communication, unavailable
  hardware, IMU failure, and rear-command failure remain terminal.
- ESP2 GPIO10 is an externally driven Final Competition run switch. A
  LOW-to-HIGH transition requests the route; LOW while the parent mode is
  active invokes the same full emergency-stop path as the dashboard STOP.
  Booting with the input already HIGH does not start motion.
- `FINAL_COMPETITION` validates all included modes before Solar moves and
  requires Placement 3 to use the rear return-line source. Each ownership
  handoff commands stopped outputs. Tower ignores side-line HIGH only for its
  bounded configured window, which must be shorter than its search timeout.
  Tower's pre/post-shimmy delays command every chassis output stopped and are
  capped at 30000 ms; the shimmy search retains its independent timeout.
  Tower Start rejects a zero initial-lift distance, an active top limit, or a
  relative lift that exceeds the remaining configured stepper travel. The
  finite jog runs concurrently with the chassis, faults if it stops before its
  tracked target, and must complete before the later bottom-limit search starts.
  PegFinder shake remains disabled while any shake field is only partially
  configured; when enabled, both directional pulses and the stopped post-shake
  delay are duration-bounded.
- Runtime MPU-6050 I2C transactions execute only in ESP2's sensor-acquisition
  task. The core-1 motion task consumes a copied snapshot and treats data older
  than the IMU freshness limit as unavailable.
- IMU heading resets are queued to the sensor owner. A turn cannot start until
  the corresponding reset sequence has been acknowledged in a published
  snapshot.
- The read-only IMU soak controls only refresh telemetry or enqueue a reset of
  fixed acquisition counters. They do not change mode, heading, IMU
  initialization, turn state, or any actuator command.
- IMU yaw control is selected by `IMU_TURN_TEST`, `IMU_STRAFE_TEST`, or the
  explicit autonomous turn/strafe states. It is not injected into normal
  manual drive, forward/backward autonomous motion, or line-following control.
- `IMU_STRAFE_TEST` requires valid locked-out-by-default tuning, a fresh
  healthy IMU, configured front motors, and a fresh configured rear link.
  Releasing the held control, losing its browser heartbeat, losing either
  sensor/link gate, pressing Stop, or changing mode stops all wheel commands.
- Maximum strafe duty plus maximum yaw-correction duty must fit within the
  verified hardware duty cap.
- Autonomous IMU motion requires the same valid shared tuning, fresh healthy
  IMU, configured motors, and fresh rear link as its manual test. Every
  autonomous IMU start, update, and recovery gate re-peeks the acquisition
  queue immediately before evaluating freshness so synchronous web work cannot
  make an older cached copy appear disconnected. An IMU I2C
  outage during an autonomous turn or strafe immediately disables all four
  wheels and freezes the owning mission and controller timers. The saved
  heading and controller target are retained. Motion resumes only after three
  new consecutive fresh samples; the pause is bounded by the existing
  30-second timed-motion cap. Expiry becomes the owning mode's terminal
  `IMU_UNAVAILABLE` fault. Manual IMU test modes remain terminal-on-loss.
- Habitat Pieces gates the first iteration of distance strafing, exit pulses,
  and rear-line reacquisition. Habitat Placement does the same for every IMU
  turn and strafe. A transition into one of these phases therefore pauses on
  an unavailable IMU instead of bypassing recovery and faulting immediately.
- After a failed runtime read, the MPU driver verifies `WHO_AM_I`, wake,
  filter, gyro-range, and accelerometer-range registers before accepting a
  recovery sample. A sensor power cycle therefore cannot resume autonomous
  motion with reset register values; it remains stopped until the bounded
  recovery expires. Rear-link, motor, configuration, and command failures
  remain immediately terminal.

## Faults

Initial fault categories:

- Communication stale
- Invalid command
- Limit switch conflict
- Hardware not configured

Fault handling currently forces mission state to `SafeStopped`.

## Hardware Configuration Gate

The motor adapters check all of these before driving:

- Both PWM GPIOs assigned.
- Both LEDC channels assigned.
- PWM frequency and resolution assigned.
- H-bridge mode selected.
- Per-wheel `forward_sign` set to `+1` or `-1`.
- Runtime duty is within `maximum_safe_test_duty`.

If any item is missing, the adapter remains disabled.
