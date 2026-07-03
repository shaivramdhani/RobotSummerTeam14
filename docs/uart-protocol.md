# UART Protocol

The UART protocol now includes a minimal fixed-size rear-drive command path from
ESP2 to ESP1.

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

## Safety Rules

- Each side must track the time of the last valid packet.
- Commands that affect actuators must include or imply an expiration.
- If valid communication is stale, each processor disables its local motors.
- Invalid version, invalid size, failed integrity check, or unsupported message
  type must not update actuator commands.
- ESP1 disables both rear motors after three consecutive invalid rear packets.
- ESP1 disables both rear motors when an explicit disabled rear command arrives.
- ESP1 boots with both rear motors disabled.
- ESP2 boots with both front motors disabled.

## TODO

- Confirm UART port, pins, voltage levels, baud rate, framing, and grounding.
- Fill in UART config in `include/esp1/PinConfig.h` and
  `include/esp2/PinConfig.h`.
