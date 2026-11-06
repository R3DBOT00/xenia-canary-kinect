#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <set>
#include <unordered_set>
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

uint32_t LoadBE32(const uint8_t* data) {
  return uint32_t(data[0]) << 24 | uint32_t(data[1]) << 16 |
         uint32_t(data[2]) << 8 | uint32_t(data[3]);
}

struct FrameStats {
  uint64_t draws = 0;
  uint64_t packets = 0;
  std::array<uint64_t, 8> edram_modes{};
  std::unordered_set<uint32_t> pixel_shaders;
  std::unordered_set<uint32_t> vertex_shaders;
  std::unordered_set<uint32_t> copy_destinations;
  std::set<std::array<uint32_t, 5>> resolves;
};

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  std::ifstream input(argv[1], std::ios::binary);
  std::vector<uint8_t> data((std::istreambuf_iterator<char>(input)), {});
  if (data.size() < sizeof(TraceHeader)) return 3;

  std::array<uint64_t, 12> commands{};
  std::array<uint64_t, 128> type3{};
  uint64_t packet_dwords = 0, draws = 0, swaps = 0;
  uint64_t read_encoded = 0, read_decoded = 0;
  uint64_t write_encoded = 0, write_decoded = 0;
  uint64_t frame_draws = 0, frame_packets = 0;
  std::vector<uint64_t> draws_per_frame, packets_per_frame;
  std::array<uint32_t, 0x5000> registers{};
  FrameStats frame_stats;
  std::vector<FrameStats> frame_stats_list;
  size_t offset = sizeof(TraceHeader);
  while (offset < data.size()) {
    const auto* type = Read<TraceCommandType>(data, offset);
    if (!type || uint32_t(*type) >= commands.size()) return 4;
    ++commands[uint32_t(*type)];
    switch (*type) {
      case TraceCommandType::kPrimaryBufferStart: {
        const auto* c = Read<PrimaryBufferStartCommand>(data, offset);
        if (!c) return 4;
        if (c->count > (data.size() - offset - sizeof(*c)) / 4) return 8;
        offset += sizeof(*c) + size_t(c->count) * 4;
        break;
      }
      case TraceCommandType::kPrimaryBufferEnd:
        offset += sizeof(PrimaryBufferEndCommand);
        break;
      case TraceCommandType::kIndirectBufferStart: {
        const auto* c = Read<IndirectBufferStartCommand>(data, offset);
        if (!c) return 4;
        if (c->count > (data.size() - offset - sizeof(*c)) / 4) return 8;
        offset += sizeof(*c) + size_t(c->count) * 4;
        break;
      }
      case TraceCommandType::kIndirectBufferEnd:
        offset += sizeof(IndirectBufferEndCommand);
        break;
      case TraceCommandType::kPacketStart: {
        const auto* c = Read<PacketStartCommand>(data, offset);
        if (!c) return 4;
        if (c->count > (data.size() - offset - sizeof(*c)) / 4) return 8;
        const uint8_t* words = data.data() + offset + sizeof(*c);
        packet_dwords += c->count;
        ++frame_packets;
        ++frame_stats.packets;
        if (c->count) {
          uint32_t header = LoadBE32(words);
          const uint32_t packet_type = header >> 30;
          if (packet_type == 0) {
            const uint32_t base = header & 0x7FFF;
            const uint32_t value_count = ((header >> 16) & 0x3FFF) + 1;
            const bool one_register = (header & 0x8000) != 0;
            for (uint32_t i = 0; i < value_count && i + 1 < c->count; ++i) {
              const uint32_t index = base + (one_register ? 0 : i);
              if (index < registers.size()) {
                registers[index] = LoadBE32(words + (i + 1) * 4);
              }
            }
          } else if (packet_type == 3) {
            uint32_t opcode = (header >> 8) & 0x7F;
            ++type3[opcode];
            draws += opcode == 0x22 || opcode == 0x36;
            frame_draws += opcode == 0x22 || opcode == 0x36;
            if (opcode == 0x22 || opcode == 0x36) {
              ++frame_stats.draws;
              const uint32_t mode = registers[0x2208] & 7;
              ++frame_stats.edram_modes[mode];
              frame_stats.pixel_shaders.insert(registers[0x21F6]);
              frame_stats.vertex_shaders.insert(registers[0x21F7]);
              if (mode == 6) {
                frame_stats.copy_destinations.insert(registers[0x2319]);
                frame_stats.resolves.insert({registers[0x2319],
                                             registers[0x231A],
                                             registers[0x231B],
                                             registers[0x2318],
                                             registers[0x2001]});
              }
            }
            swaps += opcode == 0x64;
            if ((opcode == 0x55 || opcode == 0x56) && c->count >= 3) {
              const uint32_t base = LoadBE32(words + 4) & 0xFFFF;
              for (uint32_t i = 2; i < c->count; ++i) {
                if (base + i - 2 < registers.size()) {
                  registers[base + i - 2] = LoadBE32(words + i * 4);
                }
              }
            } else if (opcode == 0x2D && c->count >= 3) {
              const uint32_t offset_type = LoadBE32(words + 4);
              if (((offset_type >> 16) & 0xFF) == 4) {
                const uint32_t base = 0x2000 + (offset_type & 0x7FF);
                for (uint32_t i = 2; i < c->count; ++i) {
                  if (base + i - 2 < registers.size()) {
                    registers[base + i - 2] = LoadBE32(words + i * 4);
                  }
                }
              }
            }
          }
        }
        offset += sizeof(*c) + size_t(c->count) * 4;
        break;
      }
      case TraceCommandType::kPacketEnd:
        offset += sizeof(PacketEndCommand);
        break;
      case TraceCommandType::kMemoryRead:
      case TraceCommandType::kMemoryWrite: {
        const auto* c = Read<MemoryCommand>(data, offset);
        if (!c) return 4;
        if (*type == TraceCommandType::kMemoryRead) {
          read_encoded += c->encoded_length;
          read_decoded += c->decoded_length;
        } else {
          write_encoded += c->encoded_length;
          write_decoded += c->decoded_length;
        }
        offset += sizeof(*c) + c->encoded_length;
        break;
      }
      case TraceCommandType::kEdramSnapshot: {
        const auto* c = Read<EdramSnapshotCommand>(data, offset);
        if (!c) return 4;
        offset += sizeof(*c) + c->encoded_length;
        break;
      }
      case TraceCommandType::kEvent:
        draws_per_frame.push_back(frame_draws);
        packets_per_frame.push_back(frame_packets);
        frame_stats_list.push_back(std::move(frame_stats));
        frame_stats = FrameStats{};
        frame_draws = 0;
        frame_packets = 0;
        offset += sizeof(EventCommand);
        break;
      case TraceCommandType::kRegisters: {
        const auto* c = Read<RegistersCommand>(data, offset);
        if (!c) return 4;
        if (c->encoded_length > data.size() - offset - sizeof(*c) ||
            c->first_register > registers.size() ||
            c->register_count > registers.size() - c->first_register) return 7;
        const uint8_t* payload = data.data() + offset + sizeof(*c);
        std::vector<uint8_t> decoded(size_t(c->register_count) * 4);
        bool decoded_ok = false;
        if (c->encoding_format == MemoryEncodingFormat::kNone &&
            c->encoded_length == decoded.size()) {
          std::memcpy(decoded.data(), payload, decoded.size());
          decoded_ok = true;
        } else if (c->encoding_format == MemoryEncodingFormat::kSnappy) {
          size_t actual_length = 0;
          if (!snappy::GetUncompressedLength(
                  reinterpret_cast<const char*>(payload), c->encoded_length,
                  &actual_length) || actual_length != decoded.size()) return 7;
          decoded_ok = snappy::RawUncompress(
              reinterpret_cast<const char*>(payload), c->encoded_length,
              reinterpret_cast<char*>(decoded.data()));
        }
        if (!decoded_ok) return 7;
        for (uint32_t i = 0; i < c->register_count; ++i) {
          if (c->first_register + i < registers.size()) {
            std::memcpy(&registers[c->first_register + i],
                        decoded.data() + size_t(i) * 4, 4);
          }
        }
        offset += sizeof(*c) + c->encoded_length;
        break;
      }
      case TraceCommandType::kGammaRamp: {
        const auto* c = Read<GammaRampCommand>(data, offset);
        if (!c) return 4;
        offset += sizeof(*c) + c->encoded_length;
        break;
      }
      default:
        return 5;
    }
    if (offset > data.size()) return 6;
  }

  std::printf(
      "bytes=%zu commands=%llu packets=%llu packet_dwords=%llu draws=%llu "
      "swaps=%llu read_encoded=%llu read_decoded=%llu write_encoded=%llu "
      "write_decoded=%llu\n",
      data.size(), static_cast<unsigned long long>(commands[0] + commands[1] +
                                                   commands[2] + commands[3] +
                                                   commands[4] + commands[5] +
                                                   commands[6] + commands[7] +
                                                   commands[8] + commands[9] +
                                                   commands[10] + commands[11]),
      static_cast<unsigned long long>(commands[4]),
      static_cast<unsigned long long>(packet_dwords),
      static_cast<unsigned long long>(draws),
      static_cast<unsigned long long>(swaps),
      static_cast<unsigned long long>(read_encoded),
      static_cast<unsigned long long>(read_decoded),
      static_cast<unsigned long long>(write_encoded),
      static_cast<unsigned long long>(write_decoded));
  for (uint32_t opcode = 0; opcode < type3.size(); ++opcode) {
    if (type3[opcode]) {
      std::printf("opcode_%02X=%llu\n", opcode,
                  static_cast<unsigned long long>(type3[opcode]));
    }
  }
  uint64_t zero_draw_frames = 0;
  for (uint64_t value : draws_per_frame) zero_draw_frames += value == 0;
  std::printf("frames=%zu zero_draw_frames=%llu\n", draws_per_frame.size(),
              static_cast<unsigned long long>(zero_draw_frames));
  const size_t first = draws_per_frame.size() > 30 ? draws_per_frame.size() - 30 : 0;
  size_t terminal_pattern_first = frame_stats_list.size();
  for (size_t i = 0; i < frame_stats_list.size(); ++i) {
    const auto& stats = frame_stats_list[i];
    if (stats.edram_modes[4] == 176 && stats.edram_modes[6] == 8) {
      terminal_pattern_first = i;
      break;
    }
  }
  if (terminal_pattern_first < frame_stats_list.size()) {
    std::printf("terminal_pattern_first_frame=%zu\n", terminal_pattern_first + 1);
  }
  for (size_t i = first; i < draws_per_frame.size(); ++i) {
    const FrameStats& stats = frame_stats_list[i];
    std::printf("frame_%zu draws=%llu packets=%llu modes=0:%llu,4:%llu,5:%llu,6:%llu ps=%zu vs=%zu copy_dest=%zu\n", i + 1,
                static_cast<unsigned long long>(draws_per_frame[i]),
                static_cast<unsigned long long>(packets_per_frame[i]),
                static_cast<unsigned long long>(stats.edram_modes[0]),
                static_cast<unsigned long long>(stats.edram_modes[4]),
                static_cast<unsigned long long>(stats.edram_modes[5]),
                static_cast<unsigned long long>(stats.edram_modes[6]),
                stats.pixel_shaders.size(), stats.vertex_shaders.size(),
                stats.copy_destinations.size());
  }
  if (!frame_stats_list.empty()) {
    std::printf("last_frame_copy_destinations=");
    bool first_destination = true;
    for (uint32_t destination : frame_stats_list.back().copy_destinations) {
      std::printf("%s%08X", first_destination ? "" : ",", destination);
      first_destination = false;
    }
    std::printf("\nlast_frame_pixel_shaders=");
    bool first_shader = true;
    for (uint32_t shader : frame_stats_list.back().pixel_shaders) {
      std::printf("%s%08X", first_shader ? "" : ",", shader);
      first_shader = false;
    }
    std::printf("\nlast_frame_vertex_shaders=");
    first_shader = true;
    for (uint32_t shader : frame_stats_list.back().vertex_shaders) {
      std::printf("%s%08X", first_shader ? "" : ",", shader);
      first_shader = false;
    }
    std::printf("\n");
    for (const auto& resolve : frame_stats_list.back().resolves) {
      std::printf(
          "last_frame_resolve base=%08X pitch=%08X info=%08X control=%08X "
          "color=%08X\n",
          resolve[0], resolve[1], resolve[2], resolve[3], resolve[4]);
    }
  }
  return 0;
}
