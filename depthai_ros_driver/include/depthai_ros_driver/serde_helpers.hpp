#pragma once

// dai headers
#include <depthai/xlink/XLinkStream.hpp>

// libnop headers
#include <nop/status.h>

// cpp headers
#include <cstddef>
#include <cstdint>

namespace rr {
/**
 * @brief Reads a packet from an XLinkStream and frees it on destruction
 * @note Make sure to call `getData` or else the packet will go un-processed
 */
class PacketReader {
public:
    PacketReader(dai::XLinkStream& stream)
            : _stream(stream) {
        _packet = _stream.readRaw();  // blocking read

        _serialized_size = static_cast<std::uint32_t>(fromLE(_packet->data + _packet->length - 4));
        if (_serialized_size > _packet->length) {
            throw std::runtime_error("Bad packet, couldn't parse");
        }

        _buf_size = _packet->length - 8 - _serialized_size;
    }
    ~PacketReader() { _stream.readRawRelease(); }

    auto packet() { return _packet; }

    std::uint32_t serializedSize() { return _serialized_size; };

    std::uint8_t* serializedDataBeginPtr() { return _packet->data + _buf_size; }

    auto bufferData() {
        // copy data part
        std::vector<std::uint8_t> data(_packet->data, _packet->data + _buf_size);
        return data;
    }

    // [[deprecated("Uses dai interface, to be removed soon")]] auto getData() {
    //     return dai::StreamMessageParser::parseMessageToADatatype(_packet);
    // }

private:
    std::uint32_t fromLE(std::uint8_t* data) {
        return data[0] + data[1] * 256 + data[2] * 256 * 256 + data[3] * 256 * 256 * 256;
    };

    dai::XLinkStream& _stream;
    streamPacketDesc_t* _packet;

    std::uint32_t _serialized_size;
    std::uint32_t _buf_size;
};

class PacketWriter {
public:
    PacketWriter(dai::XLinkStream& stream)
            : _stream(stream) {}

    auto toLE(std::uint32_t num, std::uint8_t* le_data) {
        for (int i = 0; i < 4; ++i) {
            le_data[i] = static_cast<std::uint8_t>((num >> (i * 8)) && 0xff);
        }
    };

    void write(const std::uint8_t* data, size_t data_size, const std::uint8_t* ser, size_t ser_size,
            std::int32_t datatype, std::vector<std::uint8_t>& buf) {
        size_t packet_size = data_size + ser_size + 8;

        buf.resize(packet_size);
        toLE(datatype, buf.data() + packet_size - 8);
        toLE(data_size, buf.data() + packet_size - 4);

        std::memcpy(buf.data(), data, data_size);
        std::memcpy(buf.data() + data_size, ser, ser_size);

        _stream.write(buf);
    }

    dai::XLinkStream& _stream;
};

// needs to work with libnop writer interface:
// https://github.com/google/libnop/blob/master/docs/getting-started.md#basic-writer-interface
class VectorWriter {
public:
    template <typename... Args>
    VectorWriter(Args&&... args)
            : vector{std::forward<Args>(args)...} {}
    VectorWriter(const VectorWriter&) = default;
    VectorWriter& operator=(const VectorWriter&) = default;

    nop::Status<void> Prepare(std::size_t /*size*/) { return {}; }

    nop::Status<void> Write(std::uint8_t byte) {
        vector.push_back(byte);
        return ReturnStatus();
    }

    nop::Status<void> Write(const void* begin, const void* end) {
        vector.insert(vector.end(), static_cast<const std::uint8_t*>(begin), static_cast<const std::uint8_t*>(end));
        return ReturnStatus();
    }

    nop::Status<void> Skip(std::size_t padding_bytes, std::uint8_t padding_value = 0x00) {
        for (std::size_t i = 0; i < padding_bytes; i++) {
            vector.push_back(padding_value);
            auto status = ReturnStatus();
            if (!status)
                return status;
        }

        return {};
    }

    const std::vector<std::uint8_t>& ref() const { return vector; }
    std::vector<std::uint8_t>& ref() { return vector; }
    std::vector<std::uint8_t>&& take() { return std::move(vector); }

private:
    nop::Status<void> ReturnStatus() { return {}; }

    std::vector<std::uint8_t> vector;
};
}  // namespace rr