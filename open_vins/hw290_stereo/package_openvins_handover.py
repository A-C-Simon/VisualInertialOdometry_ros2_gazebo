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
              'hw290_stereo', 'config', 'benchmark/evaluate_ate.py',
              'benchmark/monitor_process.py', 'LICENSE']
DOCS = ['OPENVINS_IMPLEMENTATION_GUIDE.md', 'CALIBRATION_TOOLS.md', 'DRIFT_FIX.md']
EXCLUDED = {'hw290_stereo/CAMERA_RESEARCH_100M.md', 'hw290_stereo/REPAIR_NOTES.md',
            'hw290_stereo/ORB_SELECTED_PROFILE.md', 'hw290_stereo/run_selected_orb.sh',
            'hw290_stereo/package_openvins_handover.py'}
EXCLUDED_PREFIXES = ('hw290_stereo/calibration/20261001/', 'hw290_stereo/handover/')


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
        if str(relative) in EXCLUDED or str(relative).startswith(EXCLUDED_PREFIXES):
            continue
        assert kind == 'blob' and mode in ('100644', '100755'), name
        data = git('cat-file', 'blob', oid)
        write(source / relative, data, 0o755 if mode == '100755' else 0o644)
        inventory.append({'path': str(relative), 'git_blob': oid,
                          'original_sha256': digest(data)})
    exported = {entry['path'] for entry in inventory}
    for entry in previous.get('runtime_source_index', []):
        old = entry['path']
        if old not in exported and (source / old).is_file():
            # Remove only files managed by the previous source inventory.
            target = (source / old).resolve()
            assert target.is_relative_to(source)
            target.unlink()
    camera_study = dest / 'CAMERA_RESEARCH_100M.md'
    if camera_study.exists():
        camera_study.unlink()
    write(source / 'SOURCE_REVISION', rev + '\n')

    def template(name):
        return git('show', f'{rev}:{prefix}hw290_stereo/handover/{name}')

    write(source / 'README.md', template('SOURCE_README.md'))
    write(source / 'hw290_stereo/README.md', template('HARDWARE_README.md'))
    included = {str(path.relative_to(dest / 'reference_files')): path
                for path in (dest / 'reference_files').rglob('*') if path.is_file()}
    included.update({f'hw290_stereo/{name}': dest / name for name in DOCS})

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
    for document in sorted((dest / 'reference_files').rglob('*.md')):
        original = document.relative_to(dest / 'reference_files')
        write(document, rewrite((source / original).read_bytes(), document, original))
    write(dest / 'README.md', template('README.md'))
    write(dest / 'FILES_AND_SOURCES.md', template('FILES_AND_SOURCES.md'))

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
