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
- Rear-drive command packet validation and stale/explicit-stop behavior.
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
