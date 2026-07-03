#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "common/Units.h"

namespace robot {

constexpr std::uint8_t kUartProtocolVersion = 1U;
constexpr std::size_t kUartMaxPayloadSize = 48U;
constexpr std::size_t kUartFrameOverheadSize = 10U;
constexpr std::uint8_t kUartFrameMagic0 = 0xA5U;
constexpr std::uint8_t kUartFrameMagic1 = 0x5AU;

enum class UartMessageType : std::uint8_t {
  Heartbeat = 1,
  ChassisCommand = 2,
  RearWheelCommand = 3,
  MechanismCommand = 4,
  SensorSnapshot = 5,
  HealthReport = 6,
  Fault = 7,
};

struct UartPacketHeader {
  std::uint8_t version{kUartProtocolVersion};
  UartMessageType message_type{UartMessageType::Heartbeat};
  std::uint16_t sequence{0};
  std::uint16_t payload_size{0};
  std::uint16_t integrity_crc16{0};
};

struct UartPacket {
  UartPacketHeader header{};
  std::array<std::uint8_t, kUartMaxPayloadSize> payload{};
};

std::uint16_t calculateCrc16Ccitt(const std::uint8_t* data,
                                  std::size_t size);
std::uint16_t calculatePacketIntegrity(const UartPacket& packet);
bool packetHeaderLooksValid(const UartPacketHeader& header);
bool packetLooksValid(const UartPacket& packet);
bool encodeUartFrame(const UartPacket& packet, std::uint8_t* frame,
                     std::size_t frame_capacity, std::size_t& frame_size);

enum class UartFrameParserStatus : std::uint8_t {
  NeedMoreData = 0,
  PacketReady = 1,
  InvalidFrame = 2,
};

class UartFrameParser {
 public:
  UartFrameParser() = default;
  void reset();
  UartFrameParserStatus push(std::uint8_t byte, UartPacket& packet);

 private:
  enum class State : std::uint8_t {
    WaitingForMagic0,
    WaitingForMagic1,
    ReadingHeader,
    ReadingPayload,
  };

  State state_{State::WaitingForMagic0};
  std::uint8_t header_bytes_read_{0};
  std::uint16_t payload_bytes_read_{0};
  UartPacket pending_packet_{};
};

class IUartLink {
 public:
  virtual ~IUartLink() = default;
  virtual bool send(const UartPacket& packet) = 0;
  virtual bool receive(UartPacket& packet) = 0;
  virtual bool isStale(Milliseconds now_ms) const = 0;
};

class NullUartLink final : public IUartLink {
 public:
  bool send(const UartPacket& packet) override {
    last_sent_header_ = packet.header;
    return false;
  }

  bool receive(UartPacket& packet) override {
    packet = {};
    return false;
  }

  bool isStale(Milliseconds now_ms) const override {
    return now_ms >= kDefaultCommunicationTimeoutMs;
  }

  const UartPacketHeader& lastSentHeader() const { return last_sent_header_; }

 private:
  UartPacketHeader last_sent_header_{};
};

}  // namespace robot
