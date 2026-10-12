#!/usr/bin/env python3
"""Package the documented hardware pipeline with local source and evidence."""
import argparse
from datetime import date
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote, urlsplit
import zipfile

ROOT = Path(__file__).resolve().parent.parent
PRIVATE_REPO = re.compile(
    r'https?://(?:www\.)?github\.com/[^/\s)]+/'
    r'VisualInertialOdometry_ros2_gazebo(?:[^\s)<>"\']*)'
)
LINK = re.compile(r'\]\(([^)\s]+)\)')
COMPONENTS = ['ov_core', 'ov_init', 'ov_msckf', 'ov_hw290',
              'hw290_stereo', 'config', 'benchmark', 'LICENSE']
DOCS = ['OPENVINS_IMPLEMENTATION_GUIDE.md', 'CALIBRATION_TOOLS.md',
        'DRIFT_FIX.md', 'CAMERA_RESEARCH_100M.md']


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def write(path, data, mode=0o644):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data if isinstance(data, bytes) else data.encode())
    path.chmod(mode)


def local_link(target, document):
    return os.path.relpath(target, document.parent)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--destination', type=Path, default=ROOT / 'OPENVINS_HANDOVER')
    parser.add_argument('--revision', default='HEAD')
    args = parser.parse_args()
    dest = args.destination.resolve()
    # The original local evidence must already be present. Never guess its source.
    previous = json.loads((dest / 'manifest.json').read_text())
    for entry in previous['files']:
        if entry.get('provenance') == 'local_evidence':
            assert digest((dest / entry['file']).read_bytes()) == entry['sha256'], entry['file']
    rev = git('rev-parse', args.revision).decode().strip()
    prefix = git('rev-parse', '--show-prefix').decode().strip()
    source = dest / 'source/open_vins'
    inventory = []
    rows = git('ls-tree', '-r', '-z', rev, '--', *COMPONENTS).split(b'\0')
    for row in filter(None, rows):
        meta, name = row.split(b'\t', 1)
        mode, kind, oid = meta.decode().split()
        relative = Path(name.decode())
        assert kind == 'blob' and mode in ('100644', '100755'), name
        data = git('cat-file', 'blob', oid)
        write(source / relative, data, 0o755 if mode == '100755' else 0o644)
        inventory.append({'path': str(relative), 'git_blob': oid,
                          'original_sha256': digest(data)})
    write(source / 'SOURCE_REVISION', rev + '\n')
    write(source / 'README.md', '''# OpenVINS hardware source

This directory is the build workspace. It contains the four ROS 2 packages,
hardware scripts, firmware, calibration files, configurations and benchmark tools.
The source version is recorded in SOURCE_REVISION. Git metadata is not included.

From this directory on Ubuntu 22.04, run as a normal account:

```bash
./hw290_stereo/setup_hw290_openvins.sh --with-calibration-tools
source hw290_stereo/env_hw290.sh
```

Read the [implementation guide](../../OPENVINS_IMPLEMENTATION_GUIDE.md) before
flashing firmware or operating a new rig. A new assembly needs calibration.
Internet access is required for system packages and external tools.
ORB-SLAM3 source and historical raw sensor recordings are not included.
''')

    included = {entry['source'].removeprefix(prefix): dest / entry['file']
                for entry in previous['files']
                if entry.get('provenance') == 'git' and 'source' in entry}
    included.update({f'hw290_stereo/{name}': dest / name for name in DOCS})
    included['OPENVINS_HANDOVER/README.md'] = dest / 'README.md'

    def rewrite(data, document, original=None):
        text = data.decode()

        def private(match):
            url = urlsplit(match.group(0))
            parts = url.path.strip('/').split('/')
            # Repository blob/tree URLs carry the original path after the revision.
            if len(parts) >= 5 and parts[2] in ('blob', 'tree'):
                path = '/'.join(parts[4:]).removeprefix(prefix)
                target = source / path
                if not target.exists():
                    raise ValueError(f'Missing local source for {match.group(0)}')
            else:
                target = source
            return local_link(target, document) + ('#' + url.fragment if url.fragment else '')

        text = PRIVATE_REPO.sub(private, text)
        if original is not None:
            def relative(match):
                url = match.group(1)
                if url.startswith(('http:', 'https:', '#', 'mailto:')):
                    return match.group(0)
                path, sep, fragment = url.partition('#')
                resolved = os.path.normpath(str(original.parent / unquote(path)))
                target = included.get(resolved, source / resolved)
                return '](' + local_link(target, document) + (sep + fragment if sep else '') + ')'
            text = LINK.sub(relative, text)
        text = text.replace('/home/ac/VisualInertialOdometry_ros2_gazebo/open_vins',
                            '$HOME/OPENVINS_HANDOVER/source/open_vins')
        return text.encode()

    # Top-level guides use local source or included evidence links.
    for name in DOCS:
        original = Path('hw290_stereo') / name
        write(dest / name, rewrite((source / original).read_bytes(), dest / name, original))
    # Reference notes may contain links generated by the previous packaging step.
    for document in sorted((dest / 'reference_files').rglob('*.md')):
        write(document, rewrite(document.read_bytes(), document))
    # The source README links back to the documentation, outside the source workspace.
    hardware_readme = source / 'hw290_stereo/README.md'
    text = hardware_readme.read_text().replace(
        '../OPENVINS_HANDOVER/README.md', '../../../README.md')
    write(hardware_readme, text)

    write(dest / 'README.md', '''# OpenVINS source and documentation

This folder contains the complete source for the working ELP stereo, HW290 and
Arduino Nano pipeline, its setup scripts, technical documentation and test evidence.
Copy the whole folder. No personal Git repository or account is needed.

## Install

The tested host is Ubuntu 22.04 x86_64 with ROS 2 Humble. Copy or extract this
folder to `$HOME/OPENVINS_HANDOVER`, then open Bash as your normal account:

```bash
cd "$HOME/OPENVINS_HANDOVER"
sha256sum -c SHA256SUMS
cd source/open_vins
./hw290_stereo/setup_hw290_openvins.sh --with-calibration-tools
source hw290_stereo/env_hw290.sh
```

Change the path if the folder is stored elsewhere. The installer requests sudo
for system packages. Internet access is required for Ubuntu/ROS packages,
Arduino tools, the public usb_cam driver and optional Kalibr/Allan builds.
The OpenVINS implementation itself is already supplied locally.

Continue with Sections 5-9 of the [implementation guide](OPENVINS_IMPLEMENTATION_GUIDE.md)
for wiring checks, firmware upload, calibration and acceptance. A new assembly
needs its own calibration. The included tower profile applies only to the
unchanged reference assembly. IMU noise remains provisional.

## Contents

| Path | Contents |
| --- | --- |
| [Implementation guide](OPENVINS_IMPLEMENTATION_GUIDE.md) | Assembly, installation, calibration, operation and acceptance |
| [Calibration tools](CALIBRATION_TOOLS.md) | Kalibr/Allan build and fitting procedures |
| [Drift fixes](DRIFT_FIX.md) | Calibration and ROS delivery corrections |
| [Source workspace](source/open_vins/README.md) | Estimator, sensor tools, scripts, firmware, configs and licenses |
| `reference_files/` | Selected tower calibration, CAD and target files |
| `evidence/calibration/` | Original fit reports and calibration results |
| `evidence/desk_test/` | State estimates, trajectory plot and resource summaries |
| [Files and sources](FILES_AND_SOURCES.md) | Local source map and public dependencies |
| [Camera research](CAMERA_RESEARCH_100M.md) | Future 100 m aerial camera study |
| `manifest.json`, `SHA256SUMS` | Source version, file inventory and integrity checks |

## Delivery limits

No Git history, compiled binaries, Docker images or daily work reports are included.
Build outputs are generated on the destination device. Software licenses and
required notices remain in the source files.

Historical raw bags are excluded: approximately 1.7 GB for calibration and 2.9 GB
for the desk test. Record new data for a new device. The supplied reports and
state estimates support inspection but cannot replay the original sensor stream.
The desk result demonstrates bounded operation for that test, not independently
measured accuracy. The camera study has no validated 100 m flight result.
''')

    old_sources = (dest / 'FILES_AND_SOURCES.md').read_text()
    external = old_sources.split('## External source repositories', 1)[1]
    external = external.split('## Files produced on the destination device', 1)[0]
    write(dest / 'FILES_AND_SOURCES.md', '''# Files and sources

## Local implementation

All project source is supplied in `source/open_vins`. Its version is recorded
in `source/open_vins/SOURCE_REVISION`. No project clone or Git account is needed.

| Component | Local path |
| --- | --- |
| Core and feature tracking | [ov_core](source/open_vins/ov_core) |
| Initializer | [ov_init](source/open_vins/ov_init) |
| Estimator and ROS delivery fixes | [ov_msckf](source/open_vins/ov_msckf) |
| C++ sensor and calibration tools | [ov_hw290](source/open_vins/ov_hw290) |
| Setup, firmware and launch scripts | [hw290_stereo](source/open_vins/hw290_stereo) |
| Estimator configuration examples | [config](source/open_vins/config) |
| Analysis and benchmark tools | [benchmark](source/open_vins/benchmark) |
| License | [LICENSE](source/open_vins/LICENSE) |

The custom AprilTag source and license are in `ov_hw290/third_party`.
The manifest records the exported source version, original blob hashes and
delivered file checksums. Documentation links and setup paths use the local copy.

## Reference files and evidence

`reference_files/` contains the selected calibration, firmware, timestamp patch,
CAD and target assets. The complete working copies are also in the source workspace.
`evidence/calibration/` contains twelve original fit/capture files and
`evidence/desk_test/` contains eight original validation files. These evidence
files are unchanged. Historical paths in fit outputs describe the original run.

## External source repositories
''' + external + '''## Files generated during installation and calibration

The installer creates `install_vio/`, `install_camera/`, build directories and
version logs. Capture creates new sensor bags and measured target metadata.
Kalibr and Allan analysis produce new camera/IMU chains, noise estimates and
fit reports. Acceptance produces trajectory and resource summaries for that rig.

Paths marked `REPLACE_WITH_*` or `/path/to/*` in commands are new-device inputs.
They are not missing files from the supplied source.
''')

    # Resolve every local Markdown link before updating the manifest or archive.
    links = 0
    for document in dest.rglob('*.md'):
        for url in LINK.findall(document.read_text()):
            if url.startswith(('http:', 'https:', '#', 'mailto:')):
                continue
            path = unquote(url.split('#', 1)[0])
            target = (document.parent / path).resolve()
            if not target.is_relative_to(dest) or not target.exists():
                raise ValueError(f'Broken local link: {document.relative_to(dest)} -> {url}')
            links += 1
    old_evidence = {entry['file']: entry for entry in previous['files']
                    if entry.get('provenance') == 'local_evidence'}
    files = []
    for path in sorted(dest.rglob('*')):
        if not path.is_file() or path.name in ('manifest.json', 'SHA256SUMS'):
            continue
        data = path.read_bytes()
        if PRIVATE_REPO.search(data.decode(errors='ignore')):
            raise ValueError(f'Personal repository link remains: {path}')
        name = path.relative_to(dest).as_posix()
        entry = {'file': name, 'sha256': digest(data), 'size_bytes': len(data)}
        if name in old_evidence:
            entry.update(provenance='local_evidence', source=old_evidence[name]['source'])
            assert entry['sha256'] == old_evidence[name]['sha256']
        files.append(entry)
    manifest = {'bundle': dest.name, 'date': date.today().isoformat(),
                'source_revision': rev, 'source_directory': 'source/open_vins',
                'runtime_acquired_by': 'included local source snapshot',
                'git_metadata_included': False, 'raw_bags_included': False,
                'files': files, 'runtime_source_index': inventory,
                'validated_local_markdown_links': links}
    write(dest / 'manifest.json', json.dumps(manifest, indent=2) + '\n')
    paths = sorted(p for p in dest.rglob('*') if p.is_file() and p != dest / 'SHA256SUMS')
    write(dest / 'SHA256SUMS', ''.join(
        f'{digest(p.read_bytes())}  {p.relative_to(dest).as_posix()}\n' for p in paths))
    archive = dest.with_suffix('.zip')
    with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_DEFLATED) as output:
        for path in paths + [dest / 'SHA256SUMS']:
            output.write(path, path.relative_to(dest.parent))
    print(f'Source revision: {rev}')
    print(f'Source files: {len(inventory)}; local links: {links}; checksums: {len(paths)}')
    print(f'Folder: {dest}\nZIP: {archive} ({archive.stat().st_size:,} bytes)')


if __name__ == '__main__':
    main()
