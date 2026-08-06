# Calibration

Only the explicitly verified values below are known. Do not invent remaining
calibration values.

## Line Sensors

- `LSFL` GPIO: TODO
- `LSFR` GPIO: TODO
- Digital polarity: HIGH means black tape, LOW means white/non-tape.
- `LSS` threshold: TODO
- `LSS2` GPIO11; threshold: TODO
- `LSS3` GPIO12; threshold: TODO
- `LSBL` threshold: TODO
- `LSBR` threshold: TODO
- Rear/side active levels or ADC ranges: TODO
- Calibrate Habitat Pieces pickup profiles 1, 2, and 3 independently: LSS2
  and LSS3 HIGH-ignore window, overall side-line search/alignment timeout,
  line-follow duty, reverse duty/duration, distance-strafe left/right direction,
  distance threshold, target zone count, strafe duty, and strafe timeout: TODO;
  set only after raised-wheel and field-path testing. Distance strafing also
  requires calibrated shared IMU heading-hold tuning. Calibrate the short
  exit-pulse strafe duty independently from the long counting-strafe duty.
  Verify counting-strafe expiry stops lateral motion, runs the configured
  post-count delay, and starts the exit pulse/check sequence. Verify a later
  exit-sequence expiry continues to the slide/approach pickup tail without a
  route fault.
  Verify that an LSS2-first alignment is physically clockwise and an LSS3-first
  alignment is counter-clockwise under the configured wheel inversions.
- For each Habitat Pieces pickup profile, calibrate slide-down speed/timeout,
  GPIO48 habitat-piece limit-switch
  operation, approach duty/timeout, pre-lift reverse duration, lift
  steps/speed/timeout/start delay, post-pickup reverse duty/duration, and
  opposite-direction rear-line strafe duty/timeout: TODO. Verify
  the bottom switch, active-high approach switch, step-counted lift distance,
  both rear sensors, and the
  automatic Habitat Placement handoff with wheels raised first. Verify each
  profile starts lowering the slide at pickup start and that the mechanism can
  reach the bottom safely while the chassis route is still running. With the
  approach switch deliberately left released, verify the configured timeout
  stops forward motion and safely continues through the reverse/lift tail.
- Calibrate the Solar exit front-line reacquisition strafe duty, forward PID
  duty, strafe timeout, forward PID duty, and forward PID duration
  independently. Verify that either front sensor finding tape overrides the
  timeout and produces a stopped handoff before the PID begins.
- Calibrate Solar's initial and retry contact timeouts with the wheels raised.
  Verify front-only contact starts exactly one left/forward/right retry, while
  no-front contact or an expired retry continues through the configured
  post-contact forward duty and duration without latching a Solar fault.
- Calibrate Solar's final signed funnel-open duty and duration with the chassis
  raised/secured. Positive uses the existing Forward convention; negative uses
  Reverse. Confirm the physical opening direction before saving a nonzero
  value. Verify the Solar Hook's adjustable `0°` open/up and `148°`
  closed/down positions. Confirm the funnel starts with the final left strafe,
  the hook stays closed until either front sensor finds the line, the funnel
  runs for its full independent duration as the route continues, and the hook
  remains powered open through Habitat and Tower.
- Calibrate the Tower Pieces side-line HIGH-ignore window so it spans the
  Habitat-to-Tower handoff but remains shorter than the side-line timeout.
  Verify that a sensor held HIGH through the window must go LOW before the next
  HIGH counts.

## IR Beacon

- `LeftIRFiltered` active level: TODO
- `RightIRFiltered` active level: TODO
- `FREQ` GPIO2 selects 1 kHz when HIGH and 10 kHz when LOW.
- Confirm IR acquisition reports active only during Solar and stops after Solar
  completion, fault, Stop, or stale ESP2 commands.

## VL53L0X Distance Sensor

- Sensor/carrier: team-described `VL53L0X V2`; confirm the exact carrier board.
- Owner: ESP1 sensor-acquisition task.
- SDA: GPIO10.
- SCL: GPIO9.
- 7-bit I2C address: `0x29`.
- Bus clock: 100 kHz.
- High-accuracy sensor profile with a 200 ms continuous intermeasurement
  period while Habitat Pickup or Placement is active. ESP1 selects this profile
  before ranging starts; idle, stopped, fault, and stale-command states retain
  the profile but disable acquisition.
- The Adafruit high-accuracy preset disables the range-ignore threshold. If a
  distant or absent target repeatedly appears as a short valid range, inspect
  protective film, dust, nearby chassis edges, cover-window air gap/tilt, and
  emitter/receiver separation before tuning a calibrated crosstalk or
  range-ignore threshold. Do not substitute an arbitrary short range as far;
  that would also hide a real close obstacle.
- Confirm the exact board's supply voltage, logic/pull-up voltage, field of
  view, cover-window behavior, target reflectance response, and mounting offset
  before using it for motion.

## Motors and Mechanisms

- Motor driver disabled states: TODO
- Motor PWM frequency/resolution: TODO
- Motor H-bridge truth table: TODO
- Per-wheel forward inversion/sign: TODO
- Maximum safe test duty: TODO
- Servo safe pulse ranges: TODO
- Solar Hook servo uses the team-confirmed claw servo settings:
  50 Hz, 12-bit, 1000–2000 µs. Its adjustable Open/Closed angles default to
  `0/148` degrees.
- Habitat Pusher uses GPIO5/LEDC 7 at 50 Hz, 12-bit, and 1000–2000 µs. Its
  open/closed angles remain TODO.
  Calibrate the two targets so the physical arm rotates clockwise from Closed
  to Open; do not infer this from increasing or decreasing angle values.
- Winch uses GPIO6/MCPWM unit 0, timer 0, generator A at 50 Hz with a 1 MHz
  timer resolution and 1000–2000 µs pulses. Its existing adjustable open and
  closed angles remain the position calibration.
- Calibrate the three matching Habitat Placement profiles independently. Each profile's
  rear-line duty, LSS1 timeout/delay, return-to-initial-
  heading timeout, pre-CCW right-strafe duty/duration, both configured IMU turn
  angles/timeouts, after-CW reverse duty/duration, shared left/right strafe
  duty and both strafe durations, all other
  translation duties/durations, slide speed/timeout, and return-line strafe
  duty/timeout are TODO field-calibration values. Select the return sensor per
  profile; the migrated defaults are Front, Front, Rear. The
  slide timeout begins when the concurrent forward-to-slide drive starts. The return turn uses
  the shared adjustable IMU maximum rotation duty. Every placement strafe uses
  the shared IMU heading-hold gains, yaw-correction limit, and measured yaw
  command polarity.
- PegFinder shake duty, left-pulse duration, and right-pulse duration: TODO.
  All three default to zero, which disables shaking. Tune with the robot raised
  and confirm each left/right pulse is small enough to preserve funnel and line
  alignment before enabling it in Final Competition.
- PegFinder stopped delay after each completed shake and before the next claw
  or the post-third-claw funnel reverse: TODO. Tune it independently from the
  claw-open interval. At each selection, verify the chosen claw reaches its
  open angle and both non-selected claws return to their closed angles.
- Tower shimmy initial direction, full left/right durations, and stopped delays
  before/after shimmy: TODO. Remember that firmware automatically applies 50%
  of the selected direction's duration to the first pulse.
- Tower initial concurrent upward jog distance: TODO. Calibrate in driver
  microsteps with enough remaining travel at Tower start; it uses the Tower
  stepper-up speed and must finish before the later bottom-limit search.
- Stepper step timing and direction polarity: TODO
- Limit switch active levels: TODO
