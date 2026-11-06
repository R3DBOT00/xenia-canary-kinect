# Xenia — 360io fork

An experimental fork of Xenia Canary developed alongside the 360io project.
It explores Kinect integration, guest kernel compatibility, graphics behavior,
and host build support for Xbox 360 emulation research.

## Focus

- **Kinect and NUI:** expanded camera, depth, skeleton, input, and guest service handling, with an optional Kinect for Windows v1 SDK backend.
- **Kernel compatibility:** work on module loading, LZX decompression, files, threads, synchronization, avatars, and application services.
- **Graphics:** changes to render targets, texture handling, shader translation, memory exports, and GPU trace diagnostics.
- **Host support:** CMake and Windows/MinGW compatibility changes, alongside the inherited Linux code paths.

## Status

This is a development source release, not a claim of complete Kinect support
or universal game compatibility. The Windows Kinect backend requires separately
installed hardware, drivers, and SDK components. These are not distributed here.
See the validation report for the exact tests performed and remaining limits.

## Relationship to upstream

Based on Xenia Canary commit `ed4540a641acd9f0a65132eac143322ee34c174b`.
This is an independent fork, not an official Xenia or Xenia Canary release.
Upstream authorship, source notices, and licenses are preserved. The public
branch extends that verified upstream revision; private development history is
not included.

## Public source scope

Includes the emulator source, selected development tools, build files, licensed
upstream icons, and dependency references pinned to commits. It does not include
games, console firmware, a recompiled dashboard, Proto Stealth, personal keyvaults,
credentials, captured traces, or the proprietary Kinect SDK.

## Documentation

- [Development and testing](docs/DEVELOPMENT.md)
- [Fork changes](docs/FORK_CHANGES.md)
- [Licenses and third-party components](docs/THIRD_PARTY.md)
- [Public source audit](docs/PUBLIC_RELEASE.md)
- [Pinned dependencies](docs/SUBMODULES.json)

## License

Xenia code and this fork's changes retain the [BSD 3-Clause license](LICENSE).
Third-party code retains its own terms. Upstream icons are covered separately
by [CC BY-SA 4.0](assets/icon/LICENSE). No upstream affiliation or endorsement
is implied.
