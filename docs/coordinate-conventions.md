# Coordinate Conventions

These conventions are software contracts only. Confirm physical wiring and
mechanical orientation before enabling hardware outputs.

## Robot Frame

- `forward_command_milli > 0` means command forward in the robot frame.
- `lateral_command_milli > 0` means command right in the robot frame.
- `yaw_command_milli > 0` means command clockwise rotation when viewed from
  above.
- Command values are normalized in milli-units from `-1000` to `1000`, not motor
  duty-cycle percentages.

## IMU Heading

- `heading_deg` is a continuous relative angle formed by integrating the
  bias-corrected gyro Z rate. It is not wrapped at `+/-180` or `360` degrees.
- An IMU turn captures the current heading once and adds `+90` or `-90` degrees
  to form the target.
- Mode fields labeled clockwise are positive physical angles. Firmware converts
  them into the IMU heading sign with `yaw_command_polarity`; entering `90`
  therefore requests a 90-degree clockwise chassis turn for either valid IMU
  mounting polarity.
- An IMU heading-held strafe captures the current heading once at strafe start
  and retains that target until Stop, fault, deadman expiry, or mode change.
- The sensor's positive mounted Z direction is not assumed to match the
  drivetrain's positive-yaw direction. `yaw_command_polarity` must be measured
  with the wheels raised and set to `+1` or `-1` before turn motion is allowed.
- Stage 2 turns and Stage 3 strafes have independently tuned gains but use the
  same measured yaw-command polarity convention.
- Autonomous Solar, Tower Pieces (including shimmy), PegFinder, and the
  Time Trial transition reuse those same live Stage 2/Stage 3 tuning objects;
  mode panels provide only the required turn angles and strafe durations or
  timeouts.
- Line-following error and steering polarity remain independent of IMU heading.

## Line Error

Front digital line observation uses `error`:

- `0` means both front sensors see black tape, or no signed error is safe to
  use.
- `+1` means `LSFL` sees black and `LSFR` sees white.
- `-1` means `LSFL` sees white and `LSFR` sees black.
- `+5` means both sensors see white after the last known side was `+1`.
- `-5` means both sensors see white after the last known side was `-1`.

Physical steering direction is controlled by `steeringPolarity`, not by changing
the sensor truth table.

## Time

Use explicit millisecond suffixes for wall-clock and timeout values, such as
`issued_at_ms`, `timeout_ms`, and `captured_at_ms`.
