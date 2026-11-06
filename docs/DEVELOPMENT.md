# Development

## Dependencies

This repository pins third-party repositories using Git submodules. Their
sources are not bundled into this snapshot. Fetch them into a development clone:

```sh
git submodule update --init --recursive
```

For a Windows/MinGW build, apply the included SDL2 patch after checking it:

```sh
git -C third_party/SDL2 apply --check ../../patches/SDL2-mingw-disable-gles.patch
git -C third_party/SDL2 apply ../../patches/SDL2-mingw-disable-gles.patch
```

Linux development requires a C++20 compiler, CMake, Ninja and Make, Python 3,
GTK3, SDL2, LZ4, ALSA, Vulkan, and X11/XCB development packages. The inherited
Snappy configure step invokes Make by default even when the top-level build
uses Ninja. Source dependencies must match `SUBMODULES.json`.

```sh
cmake -S . -B ../xenia-build -G Ninja -DCMAKE_BUILD_TYPE=Release   -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DXENIA_BUILD_TESTS=ON
cmake --build ../xenia-build --target xenia-base-tests
../xenia-build/bin/Linux/xenia-base-tests --list-tests
```

See `PUBLIC_RELEASE.md` for validation results and limitations before treating
these inherited targets as a supported full build. Upstream build guidance is
also available in `building.md`. The public copy omits the prebuilt vswhere
executable and Android packaging; Windows toolchain detection may require a
separately installed Microsoft vswhere tool. No proprietary SDK is included.

## Kinect diagnostics

The optional Kinect for Windows v1 backend is enabled only when CMake finds the
SDK headers and library on Windows. Acquire and install those independently
under their own terms. No hardware/SDK validation was performed on Linux.

The physical test launcher now requires an explicit local image path:

```powershell
./tools/Invoke-XenectPhysicalKinectTest.ps1 -GamePath 'X:\your-library\test.iso'
```

Do not commit diagnostic outputs, game images, screenshots, keys, or host
configuration. The original 360io private checkout is not required by the
public source tree.

## Release checks

Run `python3 -B tools/check_public_release.py` on the clean, unpopulated release
checkout, before fetching submodules. It checks the manifest, permitted binary
assets, dependency pins, credential patterns, and the complete local Git object
set. Builds and initialized dependencies belong in a separate development clone.

## Synthetic GPU trace regression tests

Build `tools/xenia_gpu_trace_analyze.cc` and `tools/xenia_gpu_trace_extract.cc`
with the source root and `src` on the include path, Snappy headers available,
and Snappy linked. AddressSanitizer and UndefinedBehaviorSanitizer are
recommended for this test. Then run:

```sh
python3 -B tools/test_gpu_trace_tools.py /path/to/analyze /path/to/extract
```

The test creates synthetic inputs in a temporary directory and removes them
afterward. It does not require any console executable or captured trace.
