# Re-Enabling Removed Dashboard Features Later

The current dashboard is intentionally motor-only for bring-up:

- STOP
- ESP1 UART status
- all-direction drive
- single-wheel forward/back testing
- recent events

The underlying code for line following, sensor telemetry, modes, PID tuning,
and JSON endpoints is still in the repository. It is hidden from the first page
so motor bring-up is less cluttered.

## Re-Expose Modes

The endpoint still exists:

```text
/api/mode?mode=<mode>
```

To bring mode selection back, add a `<select>` to `kDashboardHtml` in
`src/esp2/main.cpp` and call:

```js
api('/api/mode?mode=' + selectedMode)
```

Supported internal modes are still defined in:

```text
include/common/RobotTestMode.h
src/common/RobotTestMode.cpp
```

## Re-Expose Line Sensors

The endpoint still exists:

```text
/api/line
```

To bring the panel back, add HTML fields for:

- `LSFL`
- `LSFR`
- `leftBlack`
- `rightBlack`
- `error`
- `lastKnownSide`
- `lineVisible`

Then poll `/api/line` or use the existing `/api/telemetry` line object.

## Re-Expose Line Following

The endpoints still exist:

```text
/api/line-follow/start?ms=5000
/api/line-follow/stop
/api/line-follow/config?kp=<>&ki=<>&kd=<>&base=<>&max=<>&max-correction=<>&polarity=<>
```

Before re-enabling this in the UI, verify the line sensor GPIOs in
`include/esp2/PinConfig.h`. For the current motor-only bring-up they are still
unassigned, so line following should not be used.

## Re-Expose Raw Telemetry

The full JSON endpoint still exists:

```text
/api/telemetry
```

To restore the raw JSON panel, add:

```html
<pre id="raw"></pre>
```

and update it from the dashboard polling loop:

```js
qs('raw').textContent = JSON.stringify(j, null, 2)
```

## Re-Expose Future Mechanisms

Stepper, servo, and mechanism fields are still placeholders in
`TelemetrySnapshot`. Do not add movement controls until safe drivers and pin
configuration are implemented.
