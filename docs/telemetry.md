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
| `LINE_SENSOR_TEST` | No | Raw LSFL/LSFR/LSS and interpreted front-line error. |
| `LINE_FOLLOW_TEST` | Yes, gated | Digital two-sensor line follower. |
| `REAR_LINE_SENSOR_TEST` | No | Raw ESP1 LSBL/LSBR telemetry and reverse-travel line interpretation. |
| `REAR_LINE_FOLLOW_TEST` | Yes, gated | Reverse travel using rear sensors and independent rear PID settings. |
| `MECHANISM_TEST` | Mechanisms only, gated | Open/close claw and winch servos and test the ESP1 funnel motor with drive outputs stopped. |
| `AUTONOMOUS_SOLAR_PANEL` | Yes, gated | Line follow, beacon alignment, solar-panel contact, timed forward motion, and rear-line reacquisition. |
| `AUTONOMOUS_TOWER_PIECES` | Yes, gated | Reverse rear-line following until the second distinct side-line crossing or timeout. |
| `AUTONOMOUS_DRY_RUN` | No | Stub view for future mission dry runs. |

Mode changes stop actuators before switching. Sensor-only modes keep motors and
mechanisms disabled. `MECHANISM_TEST` keeps drive outputs disabled and `/api/stop`
disables the claw and winch servo outputs and sends a disabled funnel command to ESP1.

## Line Sensor Bench Test

Use this when you only want to verify comparator states and line interpretation,
without driving motors:

- Dashboard: press `Sensor Test` in the line-sensor panel. This switches ESP2
  to `LINE_SENSOR_TEST`, disables actuators, and keeps updating LSFL/LSFR/LSS.
- Serial: run `mode line-sensor`, then `line status`.
- Telemetry: watch `line.lsfl_level`, `line.lsfr_level`, and `line.lss_level`
  for `HIGH`, `LOW`, or `UNKNOWN`; `HIGH` means black tape for all three
  sensors. LSS is ESP1 GPIO4 and reports `UNKNOWN` unless its configuration is
  present and the ESP1 line-sensor stream is fresh.

For the rear sensors, press `Rear Sensor Test` or run
`mode rear-line-sensor`, then `rear-line status`. ESP1 samples LSBL on GPIO17
and LSBR on GPIO18 as digital inputs with `HIGH` meaning black tape. It sends a
CRC-protected `SensorSnapshot` every `10 ms`; ESP2 reports raw levels,
sequence, age, configuration, and freshness without enabling motors. For
reverse travel, LSBR is reported as logical left and LSBL as logical right.
The same fixed-size packet carries LSS configuration and level so tower-piece
crossings are observed at the `10 ms` sensor-stream period.

## Tower Pieces

The `Tower Pieces` dashboard panel exposes reverse line-following duty,
second-line timeout, the live LSS level, a `0 / 2` crossing count, the delay
after that second crossing, right-strafe duty and duration, the following
pause, clockwise rotation duty and duration, a post-rotation pause, timed
backward duty and duration, shimmy duty, separate right and left durations,
and a shimmy timeout.
Start enters
`AUTONOMOUS_TOWER_PIECES` and uses the independent rear PID gains with the
panel's reverse-duty magnitude. A crossing is one LSS LOW-to-HIGH transition;
holding LSS HIGH cannot increment the count repeatedly. If LSS is already HIGH
at start, firmware waits for LOW before accepting a later HIGH as a crossing.

The second crossing stops all four wheels in `POST_LINE_DELAY`. When that delay
expires, the robot enters `STRAFE_RIGHT` for the configured duration, stops in
`POST_STRAFE_PAUSE`, rotates clockwise for the configured duration, stops in
`POST_ROTATION_PAUSE`, and then drives backward for the configured duration.
It next starts by strafing right and alternates right and left after each
direction's configured duration. The run completes as soon as either LSBL or
LSBR is HIGH
during the shimmy. If the first timeout expires before the second side line,
telemetry reports `SIDE_LINE_TIMEOUT`; if the shimmy timeout expires before a
back line is detected, it reports `SHIMMY_TIMEOUT`. Stale ESP1
status, stale rear/side sensor packets, line loss without history, incomplete
hardware, and failed rear commands also stop the mode. All new timings and
open-loop duties default to `0` as explicit TODOs, and Start is rejected until
nonzero verified values are applied.

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

## Solar Contact Retry

If the front-right contact switch is hit but the back-right switch is not when
the initial right-strafe contact timeout expires, ESP2 performs one correction:
it strafes left, moves forward, and then makes one more right-strafe attempt.
The second right-strafe attempt faults on its own timeout and cannot start a
second correction sequence. Both switches stop the sequence successfully from
any contact-motion state.

The dashboard exposes `Retry left strafe ms`, `Retry forward ms`, and
`Retry right timeout ms`. Apply updates the values at runtime; Save persists
them to NVS. The two new motion durations default to `0` until the team tunes
them on the real robot; the retry timeout initially uses the existing contact
timeout.

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
- `REAR_LINE_FOLLOW_TEST` requires configured GPIO17/GPIO18 rear sensors, a
  fresh rear `SensorSnapshot`, configured local motors and UART, a fresh ESP1
  status link, nonzero maximum duty, and a nonzero verified hardware duty cap.
- Rear following stops all four wheels if rear sensor data exceeds
  `remoteCommandTimeoutMs`, ESP1 status becomes stale, or the line is lost
  without history.
- `AUTONOMOUS_TOWER_PIECES` adds configured GPIO4 LSS, a positive panel duty,
  and a nonzero panel timeout to the rear-follow requirements. It stops on the
  second distinct LSS rising edge, timeout, or any rear-follow safety fault.
- ESP1 back motors stop on stale, invalid, duplicate, corrupt, or disabled
  wheel command packets.
- ESP2 stops line following if the rear command link is unhealthy.
- Solar autonomous motion also stops if a rear-wheel command cannot be sent or
  ESP1 reports that its received commands have gone stale.
- After both solar-panel limit switches are hit, the robot waits for
  `post_contact_forward_start_delay_ms`, drives forward for
  `post_contact_forward_duration_ms` (default `1000 ms`) at
  `post_contact_forward_duty`, waits for
  `line_reacquire_strafe_start_delay_ms`, then strafes left at
  `line_reacquire_strafe_duty` until LSBL or LSBR reports black. Both delays
  default to `0 ms`, and all wheel outputs remain disabled during them.
- Claw and winch servo commands are rejected unless the corresponding ESP2 PWM
  config is complete, the requested open or closed angle is set, and that
  absolute angle is within `0..180` degrees. Open and closed targets are
  configured independently.
- Funnel commands are press-and-hold with the same `700 ms` deadman timeout as
  single-motor tests. ESP1 initializes the funnel output disabled, rejects stale
  or corrupt packets, and reports whether the funnel PWM hardware is configured.
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
| `/api/line` | GET | LSFL/LSFR/LSS level, black booleans, error, last side, visibility. |
| `/api/rear-line` | GET | Physical LSBL/LSBR levels plus reverse-travel logical mapping, error, freshness, sequence, and sample age. |
| `/api/line-follow/start?ms=<>` | GET/POST | Switch to `LINE_FOLLOW_TEST` and start line following. |
| `/api/line-follow/stop` | GET/POST | Stop line following. |
| `/api/line-follow/config?kp=<>&ki=<>&kd=<>&base=<>&max-duty=<>&max-correction=<>&integral-limit=<>&derivative-limit=<>&derivative-alpha=<>&polarity=<>&telemetry=<>` | GET/POST | Runtime PID/config update. |
| `/api/rear-line-follow/start?ms=<>` | GET/POST | Switch to `REAR_LINE_FOLLOW_TEST` and start reverse travel using the independent rear PID configuration. |
| `/api/rear-line-follow/stop` | GET/POST | Stop reverse rear line following. |
| `/api/rear-line-follow/config?kp=<>&ki=<>&kd=<>&base=<>&max-duty=<>&max-correction=<>&integral-limit=<>&derivative-limit=<>&derivative-alpha=<>&polarity=<>&telemetry=<>` | GET/POST | Update the independent rear PID/config; `base` is a positive reverse-speed magnitude. |
| `/api/autonomous/solar/start` | GET/POST | Start the gated solar-panel autonomous test. |
| `/api/autonomous/solar/config?...&post-contact-forward-ms=<>&post-contact-forward-duty=<>&line-reacquire-duty=<>&post-contact-forward-delay-ms=<>&post-forward-strafe-delay-ms=<>` | GET/POST | Update solar autonomy settings, including contact correction and post-contact motion/line-reacquisition tuning. |
| `/api/autonomous/tower-pieces/start` | GET/POST | Enter the tower-pieces mode and request gated reverse line following. |
| `/api/autonomous/tower-pieces/config?duty=<>&timeout-ms=<>&post-line-delay-ms=<>&strafe-duty=<>&strafe-duration-ms=<>&post-strafe-pause-ms=<>&rotation-duty=<>&rotation-duration-ms=<>&post-rotation-pause-ms=<>&reverse-duty=<>&reverse-duration-ms=<>&shimmy-duty=<>&shimmy-right-ms=<>&shimmy-left-ms=<>&shimmy-timeout-ms=<>` | GET/POST | Update tower-pieces reverse line following, delayed right strafe, timed rotation, pauses, timed backward drive, and independently timed right/left shimmy settings. |
| `/api/claw?id=<1|2|3>&state=<open|close>` | GET/POST | Switch to `MECHANISM_TEST` and command one claw servo. |
| `/api/claws?state=<open|close>` | GET/POST | Switch to `MECHANISM_TEST` and command all three claw servos. |
| `/api/winch?state=<open|close>` | GET/POST | Switch to `MECHANISM_TEST` and command the ESP2 GPIO6 winch servo. |
| `/api/claws/config?claw1-open=<>&claw1-closed=<>&...&winch-open=<>&winch-closed=<>` | GET/POST | Set independent absolute open and closed angles for each claw and the winch. Legacy claw start/direction arguments remain accepted for migration. |
| `/api/claws/save` | GET/POST | Save claw and winch open/closed angles to NVS. |
| `/api/funnel?speed=<>` | GET/POST | Switch to `MECHANISM_TEST` and send a timed ESP1 funnel motor command. Use `speed=0` to release. |
| `/api/config` | GET | Current tunable settings. |
| `/api/config/save` | GET/POST | Save line-following, solar-autonomy, and tower-pieces tunables to NVS. |
| `/api/events` | GET | Fixed-size recent event log. |

All command endpoints return JSON with `ok` and either `message` or `error`.

## Telemetry Fields

`/api/telemetry` includes:

- General: `uptime_ms`, `current_mode`, `previous_mode`, `enabled`,
  `fault_active`, `fault_code`, `fault_message`, `last_command_age_ms`,
  `deadman_remaining_ms`, `wifi_clients`, `ip_address`, `free_heap_bytes`,
  `reset_reason`.
- Line: `lsfl_raw_level`, `lsfr_raw_level`, `lss_raw_level`, `lsfl_level`,
  `lsfr_level`, `lss_level`, `lsfl_black`, `lsfr_black`, `lss_black`,
  `lss_configured`, `line_error`, `line_visible`,
  `has_history`/`hasHistory`, `last_known_line_side`,
  `line_follower_enabled`.
- Rear line: `rear_line.lsbl_raw_level`, `lsbr_raw_level`, electrical levels,
  black booleans, `configured`, `data_fresh`, `sequence`, `sample_age_ms`,
  `captured_at_ms`, `line_error`, visibility/history, last-known side, and
  `line_follower_enabled`. `logical_left_source` is `LSBR` and
  `logical_right_source` is `LSBL` for reverse travel.
- PID: `kp`, `ki`, `kd`, `baseDuty`, `maxDuty`, `maxCorrection`,
  `integralLimit`, `derivativeLimit`, `derivativeFilterAlpha`,
  `steeringPolarity`, `controlPeriodMs`, `remoteCommandTimeoutMs`,
  `telemetryEnabled`, `p_term`, `i_term`, `d_term`, `correction`.
- Rear PID: the same fields under `rear_pid`, stored independently, plus
  `effectiveBaseDuty`, which is negative while commanding reverse travel.
- Tower pieces: `tower_pieces.state`, `fault_reason`, `time_in_state_ms`,
  `reverse_line_duty`, `side_line_timeout_ms`, `post_line_delay_ms`,
  `strafe_right_duty`, `strafe_right_duration_ms`,
  `post_strafe_pause_ms`, `clockwise_rotation_duty`,
  `clockwise_rotation_duration_ms`, `post_rotation_pause_ms`, `reverse_duty`,
  `reverse_duration_ms`, `shimmy_duty`, `shimmy_right_duration_ms`,
  `shimmy_left_duration_ms`,
  `shimmy_timeout_ms`, `side_line_count`, `target_side_line_count`,
  side-sensor configuration/level, active-motion flags, and
  `back_line_detected`.
- Solar autonomy: state, time in state, fault reason, IR thresholds and
  confirmation, initial contact timeout, strafe duty/delay,
  `retry_strafe_left_duration_ms`, `retry_forward_duration_ms`, and
  `retry_strafe_timeout_ms`, `post_contact_forward_duration_ms`,
  `post_contact_forward_duty`, and
  `line_reacquire_strafe_duty`, `post_contact_forward_start_delay_ms`, and
  `line_reacquire_strafe_start_delay_ms`.
- Motor telemetry: ESP2-local physical FL/FR desired/applied milli-duty and
  ESP1 funnel desired/applied milli-duty, enabled, inversion, configured.
- ESP1 command status: physical BL/BR command, sequence, age, link health,
  configured flag, packet error count.
- ESP1 remote status from compact `HealthReport` frames: availability, uptime,
  mode, fault status, rear applied commands, rear inversion flags, funnel applied
  command/configuration, and side-line sensor configuration/raw level.
- Servos: `claws.claw_1`/`claw_2`/`claw_3` and `claws.winch` report hardware
  configured, independently configured absolute open/closed angles, output
  enabled, and commanded angle/open state.
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
mode rear-line-sensor
mode rear-line-follow
mode mechanism
mode autonomous-dry-run

sensor status
line status
rear-line status

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
rlf start 5000
rlf stop
rlf status
rlf reset
rlf kp 0.10
rlf ki 0.00
rlf kd 0.00
rlf base 0.20
rlf max-duty 0.35
rlf max-correction 0.20
rlf polarity 1
rlf telemetry on
```

`rlf` tunes a separate rear configuration. Its initial values are copied from
the front follower, but subsequent `rlf` commands and saves do not change `lf`
settings. `rlf base` is stored as a positive magnitude; the follower applies a
negative effective base so all four wheels travel in the opposite direction.

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
