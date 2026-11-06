#!/usr/bin/env python3
"""Synthetic regression checks: usage, truncation, unaligned records, extraction.

Usage: python3 -B tools/test_gpu_trace_tools.py ANALYZE_BINARY EXTRACT_BINARY
No game content, captured traces, or persistent output files are used.
"""
from pathlib import Path
import os
import struct
import subprocess
import sys
import tempfile


def main():
    analyze, extract = map(str, map(Path, sys.argv[1:3]))
    header = struct.pack('<I40sI', 1, b'', 0)
    count = 0
    with tempfile.TemporaryDirectory(prefix='xenia-trace-test-') as temp:
        trace = Path(temp) / 'synthetic.xtr'
        output = Path(temp) / 'slice.raw'
        def run(binary, args, expected=None):
            nonlocal count
            result = subprocess.run([binary, *map(str, args)], capture_output=True,
                                    timeout=10, env=dict(os.environ, ASAN_OPTIONS='detect_leaks=0'))
            assert result.returncode >= 0, (binary, result.returncode, result.stderr.decode())
            assert b'AddressSanitizer' not in result.stderr and b'runtime error:' not in result.stderr, result.stderr
            if expected is not None:
                assert result.returncode == expected, (result.returncode, result.stderr)
            count += 1
            return result
        run(analyze, [], 2)
        run(extract, [], 2)
        trace.write_bytes(b'')
        run(analyze, [trace], 3)
        run(extract, [trace, 0, 4, output], 3)
        # All variable-sized commands truncated immediately after their tag.
        for tag in [0, 2, 4, 6, 7, 8, 10, 11]:
            trace.write_bytes(header + struct.pack('<I', tag))
            assert run(analyze, [trace]).returncode != 0
            assert run(extract, [trace, 0, 4, output]).returncode != 0
        # A raw memory payload with an odd length leaves the next tag unaligned.
        trace.write_bytes(header + struct.pack('<IIIII', 6, 0x1000, 0, 3, 3) + b'abc' + struct.pack('<II', 9, 0))
        run(analyze, [trace], 0)
        run(extract, [trace, '0x1000', 3, output], 0)
        assert output.read_bytes() == b'abc'
        run(extract, [trace, 'invalid', 3, output], 2)
        run(extract, [trace, '0x100000000', 3, output], 2)
        # Declared packet size cannot extend past the actual file.
        trace.write_bytes(header + struct.pack('<III', 4, 0, 0xFFFFFFFF))
        assert run(analyze, [trace]).returncode != 0
        assert run(extract, [trace, 0, 4, output]).returncode != 0
        # Snappy payload claims a decoded length inconsistent with the command.
        trace.write_bytes(header + struct.pack('<IIIII', 6, 0, 1, 1, 1) + b'\x64')
        assert run(extract, [trace, 0, 1, output]).returncode != 0
        # Registers header declares absent bytes; no read or allocation may occur.
        trace.write_bytes(header + struct.pack('<III?3xII', 10, 0, 1, False, 0, 4))
        assert run(analyze, [trace]).returncode != 0
    print(f'PASS: {count} synthetic trace tool checks')

if __name__ == '__main__':
    main()
