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
- `DiagnosticReport`
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
| 0 | flags: bit 0 rear wheels enabled, bit 1 request VL53L0X high-accuracy profile, bit 2 enable VL53L0X acquisition, bit 3 enable IR acquisition, bit 4 enable rear-line acquisition |
| 1-2 | signed back-left command milli-units |
| 3-4 | signed back-right command milli-units |
| 5-8 | sender timestamp in ms |
| 9-12 | command timeout in ms |

Signed motor commands are normalized milli-units from `-1000` to `1000`.

## Line Sensor Snapshot Payload

ESP1 publishes `SensorSnapshot` at the sensor/control rate only while ESP2
requests rear-line acquisition, using a 6-byte payload:

| Byte(s) | Field |
| --- | --- |
| 0-3 | ESP1 capture timestamp in ms |
| 4 | flags: bit 0 rear sensors configured, bit 1 LSBL high, bit 2 LSBR high, bit 3 LSS configured, bit 4 LSS high, bit 5 LSS2 configured, bit 6 LSS2 high |
| 5 | flags: bit 0 LSS3 configured, bit 1 LSS3 high |

The shared snapshot keeps rear following and autonomous side-line inputs on one
coherent ESP1 acquisition path. HIGH means black tape. Habitat Pieces stops
line following when either LSS2 on ESP1 GPIO11 or LSS3 on ESP1 GPIO12 sees
black. An LSS2-first detection selects clockwise rotation until LSS3 sees
black; an LSS3-first detection selects counter-clockwise rotation until LSS2
sees black.
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

The rear-wheel command's motor outputs and both sensor-acquisition requests
expire with the command. Its last laser-profile request is retained because
sensor quality is independent of chassis command freshness. The initial and
operational profile is high accuracy. ESP1 initializes the laser without
starting continuous ranging, starts it only while Habitat Pickup or Placement
is active, and publishes immediately when acquisition state or profile changes.
Habitat Pieces does not
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

## ESP1 Status Payloads

ESP1 publishes a 19-byte operational `HealthReport` every 100 ms and a 22-byte
`DiagnosticReport` every 250 ms. ESP2 merges both into its latest remote ESP1
status. The control-safety freshness timer is updated only by the operational
report.

### Operational HealthReport

| Byte(s) | Field |
| --- | --- |
| 0-3 | ESP1 uptime in ms |
| 4 | `RobotTestMode` numeric value |
| 5 | `FaultCode` numeric value |
| 6-7 | back-left applied command milli-units |
| 8-9 | back-right applied command milli-units |
| 10-11 | funnel applied command milli-units |
| 12 | flags: bit 0 fault active, bit 1 funnel configured, bit 2 solar limit switches configured, bit 3 back-right solar limit raw high, bit 4 front-right solar limit raw high, bit 5 Solar Hook configured, bit 6 Solar Hook output enabled, bit 7 IR acquisition enabled |
| 13 | Solar Hook commanded angle in degrees, or `255` when disabled/unset |
| 14-15 | selected IR frequency in Hz |
| 16-17 | selected-frequency IR amplitude |
| 18 | flags: bit 0 IR beacon detected, bit 1 BL inverted, bit 2 BR inverted, bit 3 side line sensor configured, bit 4 side line sensor raw high |

### DiagnosticReport

| Byte(s) | Field |
| --- | --- |
| 0 | flags: bit 0 IR switch raw high, bit 1 IR switch debounced high |
| 1-2 | IR ADC average |
| 3-4 | IR ADC minimum |
| 5-6 | IR ADC maximum |
| 7-8 | IR peak-to-peak amplitude |
| 9-10 | latest IR ADC sample |
| 11-12 | 1 kHz Goertzel amplitude |
| 13-14 | 10 kHz Goertzel amplitude |
| 15-16 | active IR threshold |
| 17 | IR consecutive detection count |
| 18-21 | IR ADC sample rate in Hz |

## TODO

- Confirm UART port, pins, voltage levels, baud rate, framing, and grounding.
- Fill in UART config in `include/esp1/PinConfig.h` and
  `include/esp2/PinConfig.h`.
