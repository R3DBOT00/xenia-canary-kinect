#!/usr/bin/env python3
"""Validate a clean release snapshot before initializing submodules.

The manifest is a manually reviewed release allowlist, not proof of authorship.
No matched secret values are printed. All Git objects, including unreachable
ones, are checked. Previously public upstream objects are explicitly pinned; new private history is rejected.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = 'docs/PUBLIC_FILES.json'
PATTERNS = {
    'private key': rb'-----BEGIN (?:RSA |EC |OPENSSH |DSA )?PRIVATE KEY-----',
    'GitHub token': rb'gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,}',
    'AWS key': rb'AKIA[0-9A-Z]{16}',
    'credential URL': rb'https?://[^\s/]+:[^\s/@]+@',
    'personal path': rb'/(?:home|Users)/[A-Za-z0-9_.-]+/',
}

def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args], stderr=subprocess.DEVNULL)

def inspect(name, data, binary=False):
    if len(data) > 8 * 1024 * 1024:
        raise ValueError('Oversized file: ' + name)
    if not binary:
        if b'\0' in data:
            raise ValueError('Unexpected binary: ' + name)
        for label, pattern in PATTERNS.items():
            if re.search(pattern, data):
                raise ValueError(f'{label} match: {name} (value withheld)')

def main():
    manifest = json.loads((ROOT / MANIFEST).read_text())
    expected = {row['path']: row for row in manifest}
    if len(expected) != len(manifest):
        raise ValueError('Duplicate manifest entry')
    pins = {row['path']: row['commit'] for row in json.loads((ROOT / 'docs/SUBMODULES.json').read_text())}
    baseline = json.loads((ROOT / 'docs/UPSTREAM_OBJECTS.json').read_text())
    upstream_objects = set(baseline['objects'])
    seen, blobs = set(), set()
    for base, dirs, files in os.walk(ROOT, followlinks=False):
        for name in dirs:
            if (Path(base) / name).is_symlink():
                raise ValueError('Directory symlink')
        if Path(base) == ROOT and '.git' in dirs:
            dirs.remove('.git')
        for name in files:
            p = Path(base) / name
            rel = p.relative_to(ROOT).as_posix()
            if p.is_symlink() or not p.is_file():
                raise ValueError('Non-regular file: ' + rel)
            row = expected.get(rel)
            if row is None and rel != MANIFEST:
                raise ValueError('Unreviewed file: ' + rel)
            if p.stat().st_size > 8 * 1024 * 1024:
                raise ValueError('Oversized file: ' + rel)
            data = p.read_bytes()
            inspect(rel, data, row and row['binary'])
            if row and hashlib.sha256(data).hexdigest() != row['sha256']:
                raise ValueError('Changed file requires review: ' + rel)
            if row and row['binary'] and not rel.startswith(('assets/icon/', 'assets/discord/')):
                raise ValueError('Binary outside reviewed icon paths')
            seen.add(rel)
            blobs.add(data)
    if seen != set(expected) | {MANIFEST}:
        raise ValueError('Missing reviewed file')
    if (ROOT / '.git').exists():
        if not (ROOT / '.git').is_dir() or (ROOT / '.git/objects/info/alternates').exists():
            raise ValueError('External Git storage')
        if git('remote').decode().splitlines() != ['origin']:
            raise ValueError('Unexpected remotes')
        for args in [('remote', 'get-url', '--all', 'origin'),
                     ('remote', 'get-url', '--push', '--all', 'origin')]:
            if git(*args).decode().splitlines() != ['https://github.com/R3DBOT00/xenia-canary-kinect.git']:
                raise ValueError('Unexpected remote destination')
        objects = git('cat-file', '--batch-all-objects', '--batch-check=%(objectname) %(objecttype) %(objectsize)').splitlines()
        commits = 0
        for line in objects:
            oid, kind, size = line.decode().split()
            if oid in upstream_objects:
                continue
            if int(size) > 8 * 1024 * 1024:
                raise ValueError('Oversized Git object')
            data = git('cat-file', kind, oid)
            if kind == 'blob' and data not in blobs:
                raise ValueError('Unreviewed historical blob: ' + oid)
            if kind in {'commit', 'tag'}:
                inspect('Git metadata', data)
            if kind == 'commit':
                commits += 1
                parents = [line[7:] for line in data.decode().splitlines() if line.startswith('parent ')]
                if parents != [baseline['commit']]:
                    raise ValueError('Release must directly extend the reviewed upstream base')
                names, links = set(), {}
                for item in git('ls-tree', '-r', '-z', oid).split(b'\0'):
                    if not item:
                        continue
                    meta, name = item.split(b'\t', 1)
                    mode, objtype, target = meta.decode().split()
                    name = name.decode()
                    if mode == '160000' and objtype == 'commit':
                        links[name] = target
                    elif mode in {'100644', '100755'} and objtype == 'blob':
                        names.add(name)
                    else:
                        raise ValueError('Unexpected tree entry')
                if names != seen or links != pins:
                    raise ValueError('Git tree differs from reviewed snapshot/pins')
        if commits != 1:
            raise ValueError('Expected exactly one release commit above the upstream base')
        print(f'Git: {len(objects)} objects, one release commit, {len(pins)} pinned dependencies, approved origin')
    print(f'PASS: {len(seen)} reviewed files; no extra files or detected credential patterns')

if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print('FAIL:', exc, file=sys.stderr)
        sys.exit(1)
