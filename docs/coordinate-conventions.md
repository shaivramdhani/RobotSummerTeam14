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
