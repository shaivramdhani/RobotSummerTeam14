# Calibration

Only the explicitly verified values below are known. Do not invent remaining
calibration values.

## Line Sensors

- `LSFL` GPIO: TODO
- `LSFR` GPIO: TODO
- Digital polarity: HIGH means black tape, LOW means white/non-tape.
- `LSS` threshold: TODO
- `LSBL` threshold: TODO
- `LSBR` threshold: TODO
- Rear/side active levels or ADC ranges: TODO

## IR Beacon

- `LeftIRFiltered` active level: TODO
- `RightIRFiltered` active level: TODO
- `FREQ` expected frequency range: TODO

## Ultrasonic Sensors

- Ultrasonic 1 is an HC-SR04 on ESP1: trigger GPIO12, echo GPIO11.
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

## Motors and Mechanisms

- Motor driver disabled states: TODO
- Motor PWM frequency/resolution: TODO
- Motor H-bridge truth table: TODO
- Per-wheel forward inversion/sign: TODO
- Maximum safe test duty: TODO
- Servo safe pulse ranges: TODO
- Stepper step timing and direction polarity: TODO
- Limit switch active levels: TODO
