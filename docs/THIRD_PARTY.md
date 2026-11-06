# Licensing and provenance

## Xenia and fork code

The original Xenia BSD 3-Clause license and source notices are retained in
`LICENSE`. This release does not relicense Xenia as MIT and does not claim
upstream endorsement. Selected local changes are distributed under the same
BSD terms. Local file provenance cannot establish authorship conclusively;
the project owner must confirm rights to any unpublished contributions before
publication.

Upstream source: [Xenia Canary](https://github.com/xenia-canary/xenia-canary).
The base revision is recorded in the README. No original `.git` directories, remotes, reflogs, or private commit history are
copied. The branch is based on the same upstream revision fetched independently
from the public GitHub fork. Previously published upstream objects remain part
of its ancestry; they are recorded separately in `UPSTREAM_OBJECTS.json`.

## Included third-party source

| Component | Terms and notice location |
| --- | --- |
| TinySHA1 | ISC-style permission notice in `third_party/crypto/TinySHA1.hpp` |
| DES | MIT, `third_party/crypto/des/LICENSE` |
| Rijndael | Public-domain dedication in the source headers |
| SHA-256 | Upstream files state zlib licensing when acquired; attribution to Stephan Brumme retained |
| DXBC checksum | AMD attribution and RSA MD5 permission notices retained in the source |
| FXAA | BSD 3-Clause, `third_party/fxaa/LICENSE` |
| half | MIT, `third_party/half/LICENSE.txt` |
| LLVM excerpts and clang-format scripts | University of Illinois/NCSA notices in their LICENSE files |
| microprofile | Unlicense/public-domain notice in each header |
| libmspack | LGPL-2.1-or-later; sources and `third_party/mspack/COPYING.LIB` retained |
| RenderDoc API | MIT notice in `third_party/renderdoc/renderdoc_app.h` |
| stb | MIT/public-domain alternatives retained in source files |
| FFmpeg configuration overlay | Upstream overlay states LGPL-2.1-or-later; FFmpeg itself is an external pinned dependency |

Modified third-party source is identified in `FORK_CHANGES.md`; the SHA-256
implementation includes the local endian/Windows portability changes.
Binary redistribution has additional obligations, particularly for LGPL
components. This preparation distributes source only and is not a binary
license-compliance certification.

## Icons

The original Xenia project icon files are retained unchanged from the base
revision. They are attributed to the Xenia project and contributors, sourced
from the upstream `assets/icon` directory, and covered by the supplied
[CC BY-SA 4.0 license](../assets/icon/LICENSE). The Discord app image is identical
to the upstream 1024-pixel icon. The images are not proprietary Xbox assets.
This fork uses them as inherited application resources, not as an endorsement.

## External dependencies

`SUBMODULES.json` records 34 public dependency URLs, exact Git revisions, and
available top-level license notice paths. The corresponding gitlinks and
`.gitmodules` preserve those pins. Submodule source trees and their histories
are not distributed in the public snapshot; developers fetch them separately.
Their licenses remain independent of the Xenia license. Major examples include
FFmpeg (LGPL/GPL depending on configuration), SDL2 (zlib), libusb (LGPL), and
permissive libraries such as fmt, Snappy, Vulkan headers, and zstd. Consult the
license at the exact pinned revision before distributing dependency binaries.

The SDL2 Windows configuration patch is included separately. Local deletions
in SDL2, libusb, zstd, and a nested DirectXShaderCompiler dependency were not
exported: clean pinned dependency sources retain their files and notices.

The Microsoft Kinect SDK is optional, proprietary, and not redistributed.
Neither the SDK nor its drivers are covered by this repository's license.

## Excluded reference material

Untracked Velocity/avatar reference excerpts were excluded because their
provenance and redistribution terms were not established. Reference PDFs,
screenshots, prebuilt vswhere, standalone SPIR-V binaries, original CI release
workflows, and Android packaging are also omitted from this source release.
