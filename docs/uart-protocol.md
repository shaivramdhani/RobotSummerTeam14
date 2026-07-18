# UART Protocol

The UART protocol includes fixed-size rear-drive and mechanism-test command
paths from ESP2 to ESP1.

## Packet Header

Every packet must include:

| Field | Type | Notes |
| --- | --- | --- |
| `version` | `uint8_t` | Current scaffold version is `1` |
| `message_type` | `uint8_t` enum | Identifies payload type |
| `sequence` | `uint16_t` | Monotonic per sender, wrap allowed |
| `payload_size` | `uint16_t` | Must be no larger than fixed payload buffer |
| `integrity_crc16` | `uint16_t` | CRC-16-CCITT over the finalized packet bytes |

Use fixed-width integer types in all packet structures.

## Initial Message Types

- `Heartbeat`
- `ChassisCommand`
- `RearWheelCommand`
- `MechanismCommand`
- `SensorSnapshot`
- `HealthReport`
- `Fault`

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
| 0 | flags, bit 0 means enabled |
| 1-2 | signed back-left command milli-units |
| 3-4 | signed back-right command milli-units |
| 5-8 | sender timestamp in ms |
| 9-12 | command timeout in ms |

Signed motor commands are normalized milli-units from `-1000` to `1000`.

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
- ESP2 boots with both front motors disabled.

## ESP1 Compact Status Payload

ESP1 publishes a compact `HealthReport` frame periodically on the same UART
link. ESP2 parses this frame and exposes the latest remote ESP1 status through
the dashboard telemetry.

Payload size is 39 bytes:

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
| 38 | flags: bit 0 solar limit switches configured, bit 1 back-right solar limit raw high, bit 2 front-right solar limit raw high, bit 3 side line sensor configured, bit 4 side line sensor raw high |

## TODO

- Confirm UART port, pins, voltage levels, baud rate, framing, and grounding.
- Fill in UART config in `include/esp1/PinConfig.h` and
  `include/esp2/PinConfig.h`.
