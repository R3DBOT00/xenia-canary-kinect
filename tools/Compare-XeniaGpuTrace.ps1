param(
  [Parameter(Mandatory = $true)]
  [string[]]$TracePath,
  [string]$OutputPath
)

$ErrorActionPreference = 'Stop'

function Read-U32LE([byte[]]$data, [int64]$offset) {
  if ($offset -lt 0 -or $offset + 4 -gt $data.LongLength) {
    throw "Trace read outside file at offset $offset"
  }
  return [BitConverter]::ToUInt32($data, [int]$offset)
}

function Read-U32BE([byte[]]$data, [int64]$offset) {
  $value = Read-U32LE $data $offset
  return (($value -band 0x000000FF) -shl 24) -bor
    (($value -band 0x0000FF00) -shl 8) -bor
    (($value -band 0x00FF0000) -shr 8) -bor
    (($value -band 0xFF000000) -shr 24)
}

$command_names = @(
  'PrimaryBufferStart', 'PrimaryBufferEnd', 'IndirectBufferStart',
  'IndirectBufferEnd', 'PacketStart', 'PacketEnd', 'MemoryRead',
  'MemoryWrite', 'EdramSnapshot', 'Event', 'Registers', 'GammaRamp'
)

$opcode_names = @{
  0x21 = 'REG_RMW'; 0x22 = 'DRAW_INDX'; 0x23 = 'VIZ_QUERY'
  0x25 = 'SET_STATE'; 0x27 = 'IM_LOAD'; 0x2B = 'IM_LOAD_IMMEDIATE'
  0x2C = 'IM_STORE'; 0x2D = 'SET_CONSTANT'; 0x2E = 'LOAD_CONSTANT_CONTEXT'
  0x2F = 'LOAD_ALU_CONSTANT'; 0x34 = 'DRAW_INDX_BIN'
  0x35 = 'DRAW_INDX_2_BIN'; 0x36 = 'DRAW_INDX_2'; 0x3B = 'INVALIDATE_STATE'
  0x3D = 'MEM_WRITE'; 0x3E = 'REG_TO_MEM'; 0x44 = 'COND_EXEC'
  0x45 = 'COND_WRITE'; 0x46 = 'EVENT_WRITE'; 0x4A = 'SET_SHADER_BASES'
  0x4B = 'SET_BIN_BASE_OFFSET'; 0x4F = 'MEM_WRITE_CNTR'
  0x50 = 'SET_BIN_MASK'; 0x51 = 'SET_BIN_SELECT'; 0x54 = 'INTERRUPT'
  0x55 = 'SET_CONSTANT2'; 0x56 = 'SET_SHADER_CONSTANTS'
  0x58 = 'EVENT_WRITE_SHD'; 0x59 = 'EVENT_WRITE_CFL'
  0x5A = 'EVENT_WRITE_EXT'; 0x5B = 'EVENT_WRITE_ZPD'
  0x5E = 'CONTEXT_UPDATE'; 0x60 = 'SET_BIN_MASK_LO'; 0x64 = 'XE_SWAP'
}

function Analyze-Trace([string]$path) {
  $resolved = (Resolve-Path -LiteralPath $path).Path
  [byte[]]$data = [IO.File]::ReadAllBytes($resolved)
  if ($data.Length -lt 48) { throw "Trace header is truncated: $resolved" }

  $version = Read-U32LE $data 0
  if ($version -ne 1) { throw "Unsupported trace version $version in $resolved" }

  $counts = [ordered]@{}
  foreach ($name in $command_names) { $counts[$name] = 0L }
  $opcodes = @{}
  $packet_dwords = 0L
  $memory_read_encoded = 0L
  $memory_read_decoded = 0L
  $memory_write_encoded = 0L
  $memory_write_decoded = 0L
  $draws = 0L
  $swaps = 0L
  $swap_packets = [Collections.Generic.List[object]]::new()
  $memory_ranges = [Collections.Generic.List[object]]::new()
  $offset = 48L

  while ($offset -lt $data.LongLength) {
    $type = [int](Read-U32LE $data $offset)
    if ($type -lt 0 -or $type -ge $command_names.Count) {
      throw "Invalid command type $type at offset $offset in $resolved"
    }
    $counts[$command_names[$type]]++

    switch ($type) {
      0 { $offset += 12L + 4L * (Read-U32LE $data ($offset + 8)) }
      1 { $offset += 4 }
      2 { $offset += 12L + 4L * (Read-U32LE $data ($offset + 8)) }
      3 { $offset += 4 }
      4 {
        $count = Read-U32LE $data ($offset + 8)
        $packet_dwords += $count
        if ($count -gt 0) {
          $header = Read-U32BE $data ($offset + 12)
          if (($header -shr 30) -eq 3) {
            $opcode = [int](($header -shr 8) -band 0x7F)
            $name = if ($opcode_names.ContainsKey($opcode)) {
              $opcode_names[$opcode]
            } else {
              'TYPE3_0x{0:X2}' -f $opcode
            }
            if (-not $opcodes.ContainsKey($name)) { $opcodes[$name] = 0L }
            $opcodes[$name]++
            if ($opcode -eq 0x22 -or $opcode -eq 0x36) { $draws++ }
            if ($opcode -eq 0x64) {
              $swaps++
              $words = [Collections.Generic.List[string]]::new()
              for ($i = 0; $i -lt $count; $i++) {
                $words.Add(('{0:X08}' -f (Read-U32BE $data ($offset + 12 + 4L * $i))))
              }
              $swap_packets.Add([ordered]@{ DwordCount = $count; Words = @($words) })
            }
          }
        }
        $offset += 12L + 4L * $count
      }
      5 { $offset += 4 }
      6 {
        $base_ptr = Read-U32LE $data ($offset + 4)
        $encoding = Read-U32LE $data ($offset + 8)
        $encoded = Read-U32LE $data ($offset + 12)
        $decoded = Read-U32LE $data ($offset + 16)
        $memory_ranges.Add([ordered]@{
          Direction = 'Read'; Base = '{0:X08}' -f $base_ptr
          Encoding = $encoding; EncodedBytes = $encoded; DecodedBytes = $decoded
        })
        $memory_read_encoded += $encoded
        $memory_read_decoded += $decoded
        $offset += 20L + $encoded
      }
      7 {
        $base_ptr = Read-U32LE $data ($offset + 4)
        $encoding = Read-U32LE $data ($offset + 8)
        $encoded = Read-U32LE $data ($offset + 12)
        $decoded = Read-U32LE $data ($offset + 16)
        $memory_ranges.Add([ordered]@{
          Direction = 'Write'; Base = '{0:X08}' -f $base_ptr
          Encoding = $encoding; EncodedBytes = $encoded; DecodedBytes = $decoded
        })
        $memory_write_encoded += $encoded
        $memory_write_decoded += $decoded
        $offset += 20L + $encoded
      }
      8 { $offset += 12L + (Read-U32LE $data ($offset + 8)) }
      9 { $offset += 8 }
      10 { $offset += 24L + (Read-U32LE $data ($offset + 20)) }
      11 { $offset += 16L + (Read-U32LE $data ($offset + 12)) }
    }
    if ($offset -gt $data.LongLength) {
      throw "Command crosses end of file at offset $offset in $resolved"
    }
  }

  $ordered_opcodes = [ordered]@{}
  foreach ($entry in ($opcodes.GetEnumerator() | Sort-Object Value -Descending)) {
    $ordered_opcodes[$entry.Key] = $entry.Value
  }
  [ordered]@{
    Path = $resolved
    FileBytes = $data.LongLength
    Version = $version
    BuildCommit = [Text.Encoding]::ASCII.GetString($data, 4, 40).Trim([char]0)
    TitleId = '{0:X8}' -f (Read-U32LE $data 44)
    Commands = $counts
    PacketDwords = $packet_dwords
    DrawPackets = $draws
    SwapPackets = $swaps
    SwapPacketData = @($swap_packets)
    MemoryReadEncodedBytes = $memory_read_encoded
    MemoryReadDecodedBytes = $memory_read_decoded
    MemoryWriteEncodedBytes = $memory_write_encoded
    MemoryWriteDecodedBytes = $memory_write_decoded
    MemoryRanges = @($memory_ranges)
    Type3Opcodes = $ordered_opcodes
  }
}

$reports = @($TracePath | ForEach-Object { Analyze-Trace $_ })
$json = $reports | ConvertTo-Json -Depth 8
if ($OutputPath) {
  $parent = Split-Path -Parent $OutputPath
  if ($parent -and -not (Test-Path -LiteralPath $parent)) {
    New-Item -ItemType Directory -Force -Path $parent | Out-Null
  }
  $json | Set-Content -LiteralPath $OutputPath -Encoding ascii
}
$json
