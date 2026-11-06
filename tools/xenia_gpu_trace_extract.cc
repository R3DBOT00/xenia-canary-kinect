#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "third_party/snappy/snappy.h"
#include "xenia/gpu/trace_protocol.h"

using namespace xe::gpu;

template <typename T>
const T* Read(const std::vector<uint8_t>& data, size_t offset) {
  if (offset > data.size() || sizeof(T) > data.size() - offset) return nullptr;
  // Trace payload lengths need not preserve the alignment of the next command.
  static thread_local T value;
  std::memcpy(&value, data.data() + offset, sizeof(T));
  return &value;
}

int main(int argc, char** argv) {
  if (argc != 5) {
    std::fprintf(stderr, "usage: extract trace.xtr address length output.raw\n");
    return 2;
  }
  uint32_t target = 0, target_length = 0;
  try {
    size_t address_end = 0, length_end = 0;
    const auto address = std::stoull(argv[2], &address_end, 0);
    const auto length = std::stoull(argv[3], &length_end, 0);
    if (address > UINT32_MAX || length > UINT32_MAX ||
        argv[2][address_end] || argv[3][length_end]) return 2;
    target = static_cast<uint32_t>(address);
    target_length = static_cast<uint32_t>(length);
  } catch (const std::exception&) {
    return 2;
  }
  std::ifstream input(argv[1], std::ios::binary);
  std::vector<uint8_t> data((std::istreambuf_iterator<char>(input)), {});
  if (data.size() < sizeof(TraceHeader)) return 3;

  size_t offset = sizeof(TraceHeader);
  while (offset < data.size()) {
    const auto* type = Read<TraceCommandType>(data, offset);
    if (!type) return 4;
    switch (*type) {
      case TraceCommandType::kPrimaryBufferStart: {
        const auto* command = Read<PrimaryBufferStartCommand>(data, offset);
        if (!command) return 4;
        if (command->count > (data.size() - offset - sizeof(*command)) / 4) return 8;
        offset += sizeof(*command) + size_t(command->count) * 4;
        break;
      }
      case TraceCommandType::kPrimaryBufferEnd:
        offset += sizeof(PrimaryBufferEndCommand);
        break;
      case TraceCommandType::kIndirectBufferStart: {
        const auto* command = Read<IndirectBufferStartCommand>(data, offset);
        if (!command) return 4;
        if (command->count > (data.size() - offset - sizeof(*command)) / 4) return 8;
        offset += sizeof(*command) + size_t(command->count) * 4;
        break;
      }
      case TraceCommandType::kIndirectBufferEnd:
        offset += sizeof(IndirectBufferEndCommand);
        break;
      case TraceCommandType::kPacketStart: {
        const auto* command = Read<PacketStartCommand>(data, offset);
        if (!command) return 4;
        if (command->count > (data.size() - offset - sizeof(*command)) / 4) return 8;
        offset += sizeof(*command) + size_t(command->count) * 4;
        break;
      }
      case TraceCommandType::kPacketEnd:
        offset += sizeof(PacketEndCommand);
        break;
      case TraceCommandType::kMemoryRead:
      case TraceCommandType::kMemoryWrite: {
        const auto* command = Read<MemoryCommand>(data, offset);
        if (!command) return 4;
        const size_t payload_offset = offset + sizeof(*command);
        if (*type == TraceCommandType::kMemoryRead &&
            uint64_t(target) >= command->base_ptr &&
            uint64_t(target) + target_length <=
                uint64_t(command->base_ptr) + command->decoded_length) {
          if (command->decoded_length > 512u * 1024u * 1024u) return 5;
          std::vector<uint8_t> decoded(command->decoded_length);
          bool ok = false;
          if (command->encoding_format == MemoryEncodingFormat::kNone) {
            if (command->encoded_length == command->decoded_length) {
              std::memcpy(decoded.data(), data.data() + payload_offset,
                          command->decoded_length);
              ok = true;
            }
          } else if (command->encoding_format == MemoryEncodingFormat::kSnappy) {
            size_t actual_length = 0;
            if (!snappy::GetUncompressedLength(
                    reinterpret_cast<const char*>(data.data() + payload_offset),
                    command->encoded_length, &actual_length) ||
                actual_length != command->decoded_length) return 5;
            ok = snappy::RawUncompress(
                reinterpret_cast<const char*>(data.data() + payload_offset),
                command->encoded_length,
                reinterpret_cast<char*>(decoded.data()));
          }
          if (!ok) return 5;
          const size_t relative = target - command->base_ptr;
          const uint8_t* slice = decoded.data() + relative;
          uint64_t nonzero = 0;
          uint64_t hash = 1469598103934665603ull;
          for (uint32_t i = 0; i < target_length; ++i) {
            nonzero += slice[i] != 0;
            hash = (hash ^ slice[i]) * 1099511628211ull;
          }
          std::ofstream output(argv[4], std::ios::binary);
          output.write(reinterpret_cast<const char*>(slice), target_length);
          std::printf(
              "base=%08X decoded=%u target=%08X length=%u nonzero=%llu "
              "fnv1a=%016llX\n",
              command->base_ptr, command->decoded_length, target, target_length,
              static_cast<unsigned long long>(nonzero),
              static_cast<unsigned long long>(hash));
          return output ? 0 : 6;
        }
        offset = payload_offset + command->encoded_length;
        break;
      }
      case TraceCommandType::kEdramSnapshot: {
        const auto* command = Read<EdramSnapshotCommand>(data, offset);
        if (!command) return 4;
        offset += sizeof(*command) + command->encoded_length;
        break;
      }
      case TraceCommandType::kEvent:
        offset += sizeof(EventCommand);
        break;
      case TraceCommandType::kRegisters: {
        const auto* command = Read<RegistersCommand>(data, offset);
        if (!command) return 4;
        offset += sizeof(*command) + command->encoded_length;
        break;
      }
      case TraceCommandType::kGammaRamp: {
        const auto* command = Read<GammaRampCommand>(data, offset);
        if (!command) return 4;
        offset += sizeof(*command) + command->encoded_length;
        break;
      }
      default:
        return 7;
    }
    if (offset > data.size()) return 8;
  }
  return 9;
}
