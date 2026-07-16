# Telemetry And Robot Test Interface

ESP2 owns the first telemetry dashboard. For the current drive-test wiring,
ESP2 drives the physical front motors locally and sends physical back-motor
commands to ESP1 over UART. See `docs/drive-test.md` for the exact wheel map.

This interface is for bench testing only. It starts in `DISABLED`, initializes
actuators disabled, and refuses motion until a test mode is explicitly
selected.

## WiFi Dashboard

ESP2 starts a WiFi softAP:

| Setting | Value |
| --- | --- |
| SSID | `Team14Robot` |
| Password | `robotdebug` |
| URL | `http://192.168.4.1/` |

After flashing ESP2, connect a laptop or phone to the AP and open the URL above.
The page uses polling JSON endpoints. WebSocket/SSE is not required.
The top of the dashboard includes an `IR Beacon` panel showing ESP1 GPIO7
frequency-selective telemetry when ESP1 status packets are fresh. ESP1 GPIO2
selects the target beacon frequency: `HIGH` selects 1 kHz and `LOW` selects
10 kHz.

## Modes

| Mode | Motion allowed | Purpose |
| --- | --- | --- |
| `DISABLED` | No | Default boot mode, telemetry only. |
| `SENSOR_MONITOR` | No | Continuous sensor monitoring. |
| `SINGLE_MOTOR_TEST` | One wheel only | Direction and inversion checks. |
| `MANUAL_DRIVE_TEST` | ESP2-local front wheels only | Deadman-protected local drive commands. |
| `DISTRIBUTED_DRIVE_TEST` | All four wheels | ESP2 front wheels plus ESP1 back wheel commands. |
| `LINE_SENSOR_TEST` | No | Raw LSFL/LSFR and interpreted line error. |
| `LINE_FOLLOW_TEST` | Yes, gated | Digital two-sensor line follower. |
| `MECHANISM_TEST` | Claw servos only, gated | Open/close claw servos with drive outputs stopped. |
| `AUTONOMOUS_DRY_RUN` | No | Stub view for future mission dry runs. |

Mode changes stop actuators before switching. Sensor-only modes keep motors and
mechanisms disabled. `MECHANISM_TEST` keeps drive outputs disabled and `/api/stop`
disables the claw servo outputs.

## Line Sensor Bench Test

Use this when you only want to verify comparator states and line interpretation,
without driving motors:

- Dashboard: press `Sensor Test` in the line-sensor panel. This switches ESP2
  to `LINE_SENSOR_TEST`, disables actuators, and keeps updating LSFL/LSFR.
- Serial: run `mode line-sensor`, then `line status`.
- Telemetry: watch `line.lsfl_level` and `line.lsfr_level` for `HIGH`, `LOW`,
  or `UNKNOWN`; `HIGH` means black tape for the current front sensors.

## IR Beacon Bench Test

- Set GPIO2 switch HIGH and expose the ESP1 GPIO7 sensor to a 1 kHz beacon.
  Confirm `ir_1khz_goertzel_amplitude` rises and `ir_beacon_detected` becomes
  true after the confirmation count.
- Keep GPIO2 HIGH and try a 10 kHz beacon. Confirm it does not falsely trigger
  1 kHz mode.
- Set GPIO2 switch LOW and repeat with a 10 kHz beacon.
- Block the beacon and confirm detection clears after the configured clear
  windows.
- While driving on a safe stand, confirm the dashboard keeps updating and motor
  commands remain responsive.

## Safety Behavior

- Motors do not move at boot.
- All actuator outputs initialize disabled.
- `/api/stop` works from every mode.
- The dashboard always shows a STOP button.
- Manual drive commands expire after `700 ms` unless refreshed.
- Single-motor dashboard commands are press-and-hold with a `700 ms` deadman
  timeout if browser refreshes stop.
- Drive and line-follow tests have timed caps. The current cap is `5000 ms`.
- Single-motor commands accept normalized duty values up to `1.0`, so `0.7`
  is valid. Values outside `[-1, 1]` are rejected.
- All motion commands are rejected if the mode is wrong, the duty is out of
  range, duration is too long, arguments are malformed, or required hardware is
  not configured.
- `LINE_FOLLOW_TEST` requires configured line sensors, local motors, UART, a
  fresh ESP1 status link, nonzero maximum duty, and nonzero verified hardware
  duty cap.
- ESP1 back motors stop on stale, invalid, duplicate, corrupt, or disabled
  wheel command packets.
- ESP2 stops line following if the rear command link is unhealthy.
- Claw servo commands are rejected unless the ESP2 claw PWM config is complete,
  the per-claw start angle is set, and the derived open angle stays within
  `0..180` degrees. Open is always start plus or minus `90` degrees.
- Browser requests call high-level command handlers; they do not write GPIO/PWM
  directly.

## Endpoints

| Endpoint | Method | Description |
| --- | --- | --- |
| `/` | GET | Dashboard HTML. |
| `/api/status` | GET | Compact status JSON. |
| `/api/telemetry` | GET | Full telemetry JSON. |
| `/api/stop` | GET/POST | Emergency stop. |
| `/api/mode?mode=<mode>` | GET/POST | Change test mode safely. |
| `/api/drive?vx=<>&vy=<>&wz=<>&duty=<>` | GET/POST | Manual/distributed drive command. |
| `/api/motor?id=<FL|FR|BL|BR>&speed=<>` | GET/POST | Single motor hold/deadman command. Use `speed=0` to release. |
| `/api/invert?id=<FL|FR>` | GET/POST | Toggle front motor runtime inversion and save it. Rear inversion is a TODO on ESP1. |
| `/api/sensors` | GET | Supported sensor states. |
| `/api/line` | GET | LSFL/LSFR level, black booleans, error, last side, visibility. |
| `/api/line-follow/start?ms=<>` | GET/POST | Switch to `LINE_FOLLOW_TEST` and start line following. |
| `/api/line-follow/stop` | GET/POST | Stop line following. |
| `/api/line-follow/config?kp=<>&ki=<>&kd=<>&base=<>&max-duty=<>&max-correction=<>&integral-limit=<>&derivative-limit=<>&derivative-alpha=<>&polarity=<>&telemetry=<>` | GET/POST | Runtime PID/config update. |
| `/api/claw?id=<1|2|3>&state=<open|close>` | GET/POST | Switch to `MECHANISM_TEST` and command one claw servo. |
| `/api/claws?state=<open|close>` | GET/POST | Switch to `MECHANISM_TEST` and command all three claw servos. |
| `/api/claws/config?claw1-start=<>&claw1-dir=<1|-1>&...` | GET/POST | Runtime claw start angle and 90-degree direction update. |
| `/api/claws/save` | GET/POST | Save claw start angles and directions to NVS. |
| `/api/config` | GET | Current tunable settings. |
| `/api/config/save` | GET/POST | Save line-following tunables to NVS. |
| `/api/events` | GET | Fixed-size recent event log. |

All command endpoints return JSON with `ok` and either `message` or `error`.

## Telemetry Fields

`/api/telemetry` includes:

- General: `uptime_ms`, `current_mode`, `previous_mode`, `enabled`,
  `fault_active`, `fault_code`, `fault_message`, `last_command_age_ms`,
  `deadman_remaining_ms`, `wifi_clients`, `ip_address`, `free_heap_bytes`,
  `reset_reason`.
- Line: `lsfl_raw_level`, `lsfr_raw_level`, `lsfl_level`, `lsfr_level`,
  `lsfl_black`, `lsfr_black`, `line_error`, `line_visible`,
  `has_history`/`hasHistory`, `last_known_line_side`,
  `line_follower_enabled`.
- PID: `kp`, `ki`, `kd`, `baseDuty`, `maxDuty`, `maxCorrection`,
  `integralLimit`, `derivativeLimit`, `derivativeFilterAlpha`,
  `steeringPolarity`, `controlPeriodMs`, `remoteCommandTimeoutMs`,
  `telemetryEnabled`, `p_term`, `i_term`, `d_term`, `correction`.
- ESP2-local motor telemetry: physical FL/FR desired/applied milli-duty,
  enabled, inversion, configured.
- ESP1 command status: physical BL/BR command, sequence, age, link health,
  configured flag, packet error count.
- ESP1 remote status from compact `HealthReport` frames: availability, uptime,
  mode, fault status, rear applied commands, and rear inversion flags.
- Claws: `rotation_deg`, plus `claw_1`/`claw_2`/`claw_3` hardware configured,
  start configured, output enabled, start angle, open angle, direction, and
  commanded angle/open state.
- IR beacon telemetry from ESP1 GPIO7/GPIO2: `selectedBeaconFrequencyHz`,
  `switchRawState`, `switchDebouncedState`, `latest_raw_adc_sample`,
  `adc_sample_mean`, `ir_adc_min`, `ir_adc_max`, `ir_amplitude_pp`,
  `ir_1khz_goertzel_amplitude`, `ir_10khz_goertzel_amplitude`,
  `ir_selected_frequency_amplitude`, `ir_active_threshold`,
  `ir_beacon_detected`, `ir_consecutive_detection_count`,
  `ir_adc_sample_rate_hz`, and `motor_command_magnitude_milli`.
- Future/stub fields for IR, ultrasonic, stepper, remaining servos, limit
  switches, and battery-style expansion.

## Serial Commands

Open the ESP2 serial monitor at 115200 baud:

```text
help
status
stop

mode disabled
mode sensors
mode single-motor
mode manual-drive
mode distributed-drive
mode line-sensor
mode line-follow
mode mechanism
mode autonomous-dry-run

sensor status
line status

motor test FL 0.10 1000
motor test FR 0.10 1000
motor test BL 0.10 1000
motor test BR 0.10 1000
motor invert FL
motor invert FR

drive fwd 0.10 1000
drive back 0.10 1000
drive left 0.10 1000
drive right 0.10 1000
drive cw 0.10 1000
drive ccw 0.10 1000

lf start 5000
lf stop
lf status
lf kp 0.10
lf ki 0.00
lf kd 0.00
lf base 0.20
lf speed 0.20
lf max-duty 0.35
lf max-correction 0.20
lf integral-limit 1.00
lf derivative-limit 20.00
lf derivative-alpha 0.00
lf polarity 1
lf reset
lf telemetry on
lf telemetry off
```

Malformed commands print `rejected: <reason>`.

## Line Sensor Mapping

HIGH means black tape; LOW means white/non-tape.

| LSFL | LSFR | Error | Visible | Last side |
| --- | --- | --- | --- | --- |
| HIGH | HIGH | `0` | true | preserved |
| LOW | HIGH | `-1` | true | `-1` |
| HIGH | LOW | `+1` | true | `+1` |
| LOW | LOW after `+1` | `+5` | false | `+1` |
| LOW | LOW after `-1` | `-5` | false | `-1` |
| LOW | LOW no history | `0` | false | `0` |

Line error mapping is logical and is not changed by motor inversion.

## Bring-Up Checklist

1. Raise the robot so wheels cannot move it across the table.
2. Fill in verified GPIOs, LEDC channels, PWM frequency/resolution, H-bridge
   mode, UART pins/baud, wheel `forward_sign`, and `maximum_safe_test_duty`.
3. Build and upload ESP1 and ESP2.
4. Connect to `Team14Robot` and open `http://192.168.4.1/`.
5. Confirm the dashboard boots in `DISABLED`.
6. Enter `SINGLE_MOTOR_TEST` and hold one wheel at a low duty briefly. Correct
   compile-time `forward_sign` first; use runtime inversion only for ESP2
   front-wheel bench calibration.
7. Enter `DISTRIBUTED_DRIVE_TEST` and verify all directions with wheels raised.
8. Do not use line-sensor or line-following modes for this drive test.
9. Confirm ESP1 remote status becomes available on the dashboard after UART is
    configured.
10. Prove stale-command and UART shutdown before any floor test.

Passing software builds do not prove physical safety or line-following behavior.
Hardware verification is still required.
