# Public source release audit

## Scope

This edition was prepared in a separate directory. The original fork and its
private development artifacts were not modified. Source selection started from
tracked upstream files plus seven explicitly reviewed local implementation and
diagnostic files. Nothing was copied by recursively exporting the private
working directory.

The selected sources were scanned for common credential patterns and personal
Unix paths. No matches remained. A manually identified Windows game-image path
list was removed from the physical Kinect launcher and replaced by `-GamePath`.
Pattern scanning cannot prove that arbitrary encoded secrets are absent.

## Included and excluded

Included: emulator sources, local Kinect/kernel/GPU/build changes, source tests,
selected tools, retained license notices, unchanged licensed upstream icons,
and 34 pinned external dependency references. `PUBLIC_FILES.json` is the exact
file/hash inventory. Large generated PPC assembly test sources are retained
because they are upstream test inputs, not captured guest binaries.

Excluded from the release tree:

- Dashboard 17489 recompilation, Proto Stealth, games, XEX/XBE files, firmware,
  personal keyvaults, credential files, logs, screenshots, and GPU trace captures.
- Private `.git` directories, local reflogs, original remote configuration,
  builds, caches, toolchain installations, and object files.
- Untracked avatar reference excerpts whose redistribution terms were unclear.
- Reference PDFs/screenshots, original release CI workflows, Android packaging,
  precompiled vswhere, and standalone SPIR-V binaries.
- Dirty dependency trees and nested dependency histories. The SDL2 source
  adjustment is preserved as a patch; local deletions of files/notices are not.

The only retained binary assets are the licensed upstream application icons.
Shader bytecode represented as upstream source headers is retained with its
corresponding source; this is distinct from compiled console executables.

## Cryptographic material

No personal keys, credentials, or keyvault files are included. The emulator
source inherits publicly distributed cryptographic constants and algorithms
from Xenia, including loader/kernel constants. These are not represented as
personal credentials, and this report does not claim the tree contains no
cryptographic material. If a release policy prohibits all embedded constants,
it needs a separate functional redesign and review before distribution.

## History and GitHub destination

The owner supplied `https://github.com/R3DBOT00/xenia-canary-kinect` and requested
a push. The destination default branch was newer/different than the local base.
The prepared branch is `codex/kinect-public`, based on the independently fetched
public upstream revision recorded in the README. It does not replace
`canary_experimental` and does not require a force push.

`UPSTREAM_OBJECTS.json` identifies the pre-existing upstream objects. Previously
published upstream ancestry may contain files excluded from the new working
tree; those files have not been erased from upstream history. The release
verifier checks every additional Git object against the reviewed snapshot and
rejects new unrelated blobs, symlinks, unexpected dependency pins, and remotes.
This is not a claim that the entire existing GitHub fork history was rewritten
or certified. No private fork history was imported.

## Validation and export fixes

Validation used an isolated source/build directory and clean dependency
revisions. Missing nested sample/media dependencies were not substituted with
private files. They are not required by the selected base test target.

- Fixed a C++ anonymous-union nested type declaration rejected by Clang, retaining
  the same `KernelVersion` field layout. The original source was unchanged.
- Fixed truncated-command crashes, unaligned reads, packet/register payload
  bounds, decompressed-length checks, and invalid numeric arguments in the two
  GPU trace utilities.
- Both trace tools passed 28 synthetic regression checks with GCC AddressSanitizer
  and UndefinedBehaviorSanitizer. No real traces or games were used.

Full emulator rendering, game compatibility, Windows/MinGW runtime behavior,
physical Kinect hardware, and proprietary SDK integration have not been
validated by this export. The public snapshot is experimental source, not a
validated binary release.

## Licensing limits

BSD source notices, third-party terms, and the icon's CC BY-SA license are
preserved. See `THIRD_PARTY.md`. Ownership of unpublished local contributions
cannot be proved from file contents alone. There is no blanket MIT relicensing
or claim that all dependencies share Xenia's license.

Local audit logs, original file inventories, and build outputs remain outside
this repository and must not be uploaded with it.
