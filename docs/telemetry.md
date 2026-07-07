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
| `MECHANISM_TEST` | No | Stub view for future stepper, servo, funnel, claw, and limits. |
| `AUTONOMOUS_DRY_RUN` | No | Stub view for future mission dry runs. |

Mode changes stop actuators before switching. Sensor-only modes keep motors and
mechanisms disabled.

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
- `LINE_FOLLOW_TEST` requires configured line sensors, local motors, UART,
  nonzero maximum duty, and nonzero verified hardware duty cap. The current
  drive-test pin profile leaves line sensor pins unassigned, so line following
  will not start.
- ESP1 back motors stop on stale, invalid, duplicate, corrupt, or disabled
  wheel command packets.
- ESP2 stops line following if the rear command link is unhealthy.
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
| `/api/line-follow/start?ms=<>` | GET/POST | Start line following in `LINE_FOLLOW_TEST`. |
| `/api/line-follow/stop` | GET/POST | Stop line following. |
| `/api/line-follow/config?kp=<>&ki=<>&kd=<>&base=<>&max=<>&max-correction=<>&polarity=<>` | GET/POST | Runtime PID/config update. |
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
- Line: `lsfl_raw_level`, `lsfr_raw_level`, `lsfl_black`, `lsfr_black`,
  `line_error`, `line_visible`, `last_known_line_side`,
  `line_follower_enabled`.
- PID: `kp`, `ki`, `kd`, `baseDuty`, `maximumDuty`, `maximumCorrection`,
  `steeringPolarity`, `p_term`, `i_term`, `d_term`, `correction`.
- ESP2-local motor telemetry: physical FL/FR desired/applied milli-duty,
  enabled, inversion, configured.
- ESP1 command status: physical BL/BR command, sequence, age, link health,
  configured flag, packet error count.
- ESP1 remote status from compact `HealthReport` frames: availability, uptime,
  mode, fault status, rear applied commands, and rear inversion flags.
- Future/stub fields for IR, ultrasonic, stepper, servos, limit switches, and
  battery-style expansion.

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
lf polarity 1
lf telemetry on
lf telemetry off
```

Malformed commands print `rejected: <reason>`.

## Line Sensor Mapping

LOW means black tape; HIGH means white/non-tape.

| LSFL | LSFR | Error | Visible | Last side |
| --- | --- | --- | --- | --- |
| LOW | LOW | `0` | true | preserved |
| LOW | HIGH | `+1` | true | `+1` |
| HIGH | LOW | `-1` | true | `-1` |
| HIGH | HIGH after `+1` | `+5` | false | `+1` |
| HIGH | HIGH after `-1` | `-5` | false | `-1` |
| HIGH | HIGH no history | `0` | false | `0` |

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
