# Calibration

Only the explicitly verified values below are known. Do not invent remaining
calibration values.

## Line Sensors

- `LSFL` GPIO: TODO
- `LSFR` GPIO: TODO
- Digital polarity: HIGH means black tape, LOW means white/non-tape.
- `LSS` threshold: TODO
- `LSS2` GPIO11; threshold: TODO
- `LSS3` GPIO12; threshold: TODO; not yet used by autonomy
- `LSBL` threshold: TODO
- `LSBR` threshold: TODO
- Rear/side active levels or ADC ranges: TODO
- Habitat Pieces LSS2 detection delay/search timeout and reverse duty/duration:
  TODO; set only after raised-wheel and field-path testing.

## IR Beacon

- `LeftIRFiltered` active level: TODO
- `RightIRFiltered` active level: TODO
- `FREQ` GPIO2 selects 1 kHz when HIGH and 10 kHz when LOW.

## Ultrasonic Sensors

- Ultrasonic 1 trigger/echo GPIOs are unassigned because its previous GPIO12
  and GPIO11 assignments now belong to LSS3 and LSS2. It remains disabled until
  two non-conflicting pins are assigned and verified.

- Ultrasonic 1 is an HC-SR04 on ESP1, but its replacement trigger/echo GPIOs
  are TODO.
- Echo is divided to 3.3 V in hardware.
- Trigger pulse: 10 us HIGH, from the HC-SR04 data sheet.
- Valid data-sheet range: 20-4000 mm.
- Distance conversion uses the data-sheet 340 m/s sound velocity and divides
  round-trip time by two.
- Echo timeout: 23530 us, derived from the 4000 mm maximum range. A timeout or
  a converted value outside the valid range is reported as no valid echo.
- Bench-check distance accuracy and environmental sensitivity before using this
  reading for autonomous decisions.
- Ultrasonic 2 model, pins, level shifting, and timing: TODO.

## VL53L0X Distance Sensor

- Sensor/carrier: team-described `VL53L0X V2`; confirm the exact carrier board.
- Owner: ESP1 sensor-acquisition task.
- SDA: GPIO10.
- SCL: GPIO9.
- 7-bit I2C address: `0x29`.
- Bus clock: 100 kHz.
- High-accuracy sensor profile with a 200 ms continuous intermeasurement
  period. ESP1 selects this profile before ranging starts, and idle, active,
  stopped, fault, and stale-command states retain it.
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
  50 Hz, 12-bit, 1000–2000 µs. Open/Closed angles remain unset until calibrated.
- Habitat Pusher uses GPIO5/LEDC 7 at 50 Hz, 12-bit, and 1000–2000 µs. Its
  open/closed angles remain TODO.
  Calibrate the two targets so the physical arm rotates clockwise from Closed
  to Open; do not infer this from increasing or decreasing angle values.
- Winch uses GPIO6/MCPWM unit 0, timer 0, generator A at 50 Hz with a 1 MHz
  timer resolution and 1000–2000 µs pulses. Its existing adjustable open and
  closed angles remain the position calibration.
- Habitat Placement rear-line duty, LSS1 timeout/delay, both IMU turn angles
  and timeouts, after-CW reverse and left-strafe duties/durations, all other
  translation duties/durations, slide speed/timeout, and final front-line
  strafe duty/timeout are TODO field-calibration values.
- Stepper step timing and direction polarity: TODO
- Limit switch active levels: TODO
