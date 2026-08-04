# UART Protocol

The UART protocol includes fixed-size rear-drive and mechanism-test command
paths from ESP2 to ESP1.

## Packet Header

Every packet must include:

| Field | Type | Notes |
| --- | --- | --- |
| `version` | `uint8_t` | Current scaffold version is `2` |
| `message_type` | `uint8_t` enum | Identifies payload type |
| `sequence` | `uint16_t` | Monotonic per sender, wrap allowed |
| `payload_size` | `uint16_t` | Must be no larger than fixed payload buffer |
| `integrity_crc16` | `uint16_t` | CRC-16-CCITT over the finalized packet bytes |

Use fixed-width integer types in all packet structures.

Version 2 changes the laser-profile flag semantics from long range to high
accuracy. ESP1 and ESP2 must run matching firmware; a version mismatch is
rejected so an old long-range acknowledgement cannot be mistaken for high
accuracy.

## Initial Message Types

- `Heartbeat`
- `ChassisCommand`
- `RearWheelCommand`
- `MechanismCommand`
- `SensorSnapshot`
- `HealthReport`
- `Fault`
- `LaserDistanceSnapshot`

## Frame Format

Frames are serialized as:

| Bytes | Field |
| --- | --- |
| 0 | magic `0xA5` |
| 1 | magic `0x5A` |
| 2 | protocol version |
| 3 | message type |
| 4-5 | sequence, little-endian |
| 6-7 | payload size, little-endian |
| 8-9 | CRC-16-CCITT, little-endian |
| 10... | payload |

The CRC covers version, message type, sequence, payload size, and payload. It
does not include the magic bytes or CRC field.

## Rear Wheel Command Payload

`RearWheelCommand` uses a 13-byte payload:

| Byte(s) | Field |
| --- | --- |
| 0 | flags: bit 0 rear wheels enabled, bit 1 request VL53L0X high-accuracy profile |
| 1-2 | signed back-left command milli-units |
| 3-4 | signed back-right command milli-units |
| 5-8 | sender timestamp in ms |
| 9-12 | command timeout in ms |

Signed motor commands are normalized milli-units from `-1000` to `1000`.

## Line Sensor Snapshot Payload

ESP1 publishes `SensorSnapshot` every `10 ms` using a 6-byte payload:

| Byte(s) | Field |
| --- | --- |
| 0-3 | ESP1 capture timestamp in ms |
| 4 | flags: bit 0 rear sensors configured, bit 1 LSBL high, bit 2 LSBR high, bit 3 LSS configured, bit 4 LSS high, bit 5 LSS2 configured, bit 6 LSS2 high |
| 5 | flags: bit 0 LSS3 configured, bit 1 LSS3 high |

The shared snapshot keeps rear following and current/future autonomous
side-line inputs on one coherent ESP1 acquisition path. HIGH means black tape.
Habitat Pieces consumes LSS2 on ESP1 GPIO11 as its all-wheel stop sensor. LSS3
on ESP1 GPIO12 remains available as telemetry but does not control this route.
ESP2 also accepts the prior 5-byte snapshot as a migration aid and reports LSS3
unconfigured for that packet. Flash ESP2 before ESP1 when updating processors
separately; the older ESP2 firmware does not understand the new 6-byte packet.

## Laser Distance Snapshot Payload

ESP1 publishes `LaserDistanceSnapshot` (`message_type = 8`) after each completed
VL53L0X measurement and resends the unchanged snapshot at the 100 ms heartbeat
period. It uses a 34-byte payload:

| Byte(s) | Field |
| --- | --- |
| 0-3 | ESP1 measurement-completion timestamp in ms |
| 4-5 | distance in mm; `0` when the measurement is invalid |
| 6-7 | measurement sequence; increments once per completed read or driver-error attempt |
| 8 | flags: bit 0 configured, bit 1 initialized, bit 2 continuous ranging active, bit 3 data valid, bit 4 high-accuracy profile active |
| 9 | VL53L0X range status; `0` is valid, `255` means no range result |
| 10 | signed VL53L0X driver status |
| 11-14 | successful measurement count |
| 15-18 | failed measurement count |
| 19-20 | consecutive failed measurement count |
| 21-24 | current acquisition/service duration in us |
| 25-28 | maximum acquisition/service duration in us |
| 29 | signed SDA GPIO (`10` in the current ESP1 configuration) |
| 30 | signed SCL GPIO (`9` in the current ESP1 configuration) |
| 31 | 7-bit I2C address (`0x29`) |
| 32-33 | continuous-ranging intermeasurement period in ms |

The UART header sequence identifies frame delivery. The payload measurement
sequence identifies new sensor data. Consumers must calculate measurement
freshness from receipt of a changed measurement sequence; receipt of an
unchanged heartbeat does not refresh sensor data.

The rear-wheel command's motor outputs expire with the command, but its last
laser-profile request is retained because sensor quality is independent of
chassis command freshness. The initial and operational profile is high
accuracy. ESP1 also
publishes immediately when the active profile changes. Habitat Pieces does not
use laser profile acknowledgement as a Start gate. Its IMU-held pickup strafe
consumes only new, fresh high-accuracy measurement attempts to count distinct
entries at or below the configured distance threshold. After the target count,
the same sequence gate ensures every stopped post-pulse check uses a newly
acquired result. A fresh N/A/no-target attempt is represented locally on ESP2
as 65536 mm, so it is outside the counted zone and satisfies the exit check;
stale or repeated snapshots do neither, and the strafe timeout remains
authoritative.

## Funnel Mechanism Command Payload

`MechanismCommand` uses a 12-byte payload for the ESP1-owned funnel motor:

| Byte(s) | Field |
| --- | --- |
| 0 | mechanism target, `1` means funnel motor |
| 1 | flags, bit 0 means enabled |
| 2-3 | signed funnel command milli-units |
| 4-7 | sender timestamp in ms |
| 8-11 | command timeout in ms |

ESP1 rejects malformed, duplicate, corrupt, wrong-target, or stale funnel
commands and applies a disabled motor command instead.

## Solar Hook Mechanism Command Payload

`MechanismCommand` uses a 3-byte payload for the ESP1-owned Solar Hook servo:

| Byte(s) | Field |
| --- | --- |
| 0 | mechanism target, `2` means Solar Hook servo |
| 1 | flags, bit 0 means PWM output enabled |
| 2 | absolute target angle in degrees, 0–180 |

The command is a latched servo-position request like the existing claw
commands. A disabled command detaches GPIO3. ESP1 rejects malformed, corrupt,
wrong-target, unknown-flag, or out-of-range commands without changing the
current output.

## Safety Rules

- Each side must track the time of the last valid packet.
- Commands that affect actuators must include or imply an expiration.
- If valid communication is stale, each processor disables its local motors.
- Invalid version, invalid size, failed integrity check, or unsupported message
  type must not update actuator commands.
- ESP1 disables both rear motors after three consecutive invalid rear packets.
- ESP1 disables the funnel motor after three consecutive invalid funnel packets.
- ESP1 disables motors when explicit disabled rear or funnel commands arrive.
- ESP1 boots with both rear motors disabled.
- ESP1 boots with the funnel motor disabled.
- ESP1 boots with the Solar Hook servo PWM detached and disabled.
- ESP2 boots with both front motors disabled.

## ESP1 Compact Status Payload

ESP1 publishes a compact `HealthReport` frame periodically on the same UART
link. ESP2 parses this frame and exposes the latest remote ESP1 status through
the dashboard telemetry.

Payload size is 47 bytes. ESP2 also accepts the prior 45-byte payload during a
rolling firmware update.

| Byte(s) | Field |
| --- | --- |
| 0-3 | ESP1 uptime in ms |
| 4 | `RobotTestMode` numeric value |
| 5 | `FaultCode` numeric value |
| 6-7 | back-left applied command milli-units |
| 8-9 | back-right applied command milli-units |
| 10 | flags: bit 0 fault active, bit 1 BL inverted, bit 2 BR inverted, bit 3 IR beacon detected, bit 4 IR switch raw high, bit 5 IR switch debounced high, bit 6 funnel configured |
| 11-12 | IR ADC average |
| 13-14 | IR ADC minimum |
| 15-16 | IR ADC maximum |
| 17-18 | IR peak-to-peak amplitude |
| 19-20 | selected IR frequency in Hz |
| 21-22 | latest IR ADC sample |
| 23-24 | 1 kHz Goertzel amplitude |
| 25-26 | 10 kHz Goertzel amplitude |
| 27-28 | selected-frequency amplitude |
| 29-30 | active IR threshold |
| 31-34 | IR ADC sample rate in Hz |
| 35 | IR consecutive detection count |
| 36-37 | funnel applied command milli-units |
| 38 | flags: bit 0 solar limit switches configured, bit 1 back-right solar limit raw high, bit 2 front-right solar limit raw high, bit 3 side line sensor configured, bit 4 side line sensor raw high, bit 5 ultrasonic 1 configured, bit 6 ultrasonic 1 echo valid |
| 39-40 | ultrasonic 1 distance in mm |
| 41-44 | ultrasonic 1 echo pulse duration in us |
| 45 | Solar Hook flags: bit 0 hardware configured, bit 1 PWM output enabled |
| 46 | Solar Hook commanded angle in degrees, or `255` when disabled/unset |

## TODO

- Confirm UART port, pins, voltage levels, baud rate, framing, and grounding.
- Fill in UART config in `include/esp1/PinConfig.h` and
  `include/esp2/PinConfig.h`.
