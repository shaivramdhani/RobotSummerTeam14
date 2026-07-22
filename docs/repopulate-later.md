# Dashboard Feature Notes

The dashboard now exposes the motor bring-up and line-following controls:

- STOP
- ESP1 UART status
- all-direction drive
- single-wheel forward/back testing
- front line-sensor status
- line-follow start/stop and PID/PD runtime tuning
- recent events

The sections below are retained as implementation notes for future dashboard
expansion.

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

## Line-Following Endpoints

The endpoints still exist:

```text
/api/line-follow/start?ms=5000
/api/line-follow/stop
/api/line-follow/config?kp=<>&ki=<>&kd=<>&base=<>&max-duty=<>&max-correction=<>&integral-limit=<>&derivative-limit=<>&derivative-alpha=<>&polarity=<>&telemetry=<>
```

Before floor testing, verify the line sensor GPIOs and active-HIGH comparator
behavior in `include/esp2/PinConfig.h`.

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

Pusher and mechanism limit fields are still placeholders in
`TelemetrySnapshot`. The servo telemetry panel includes the three claws and the
GPIO6 winch; movement remains gated by configured open/closed angles and the
PWM settings in `include/esp2/PinConfig.h`.
