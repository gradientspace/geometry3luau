"""Build a prebuilt, drop-in Roblox model of the library. Standard library only.

    python tools/package.py                         # both profiles
    python tools/package.py --profile production    # dist/production/geometry3luau.rbxmx (runtime modules only)
    python tools/package.py --profile development   # dist/development/geometry3luau.rbxmx (+ Tests, Examples)

The model is one ModuleScript named `geometry3luau` (src/init.luau) whose children are the other src/*.luau
modules. The development profile adds a `Tests` folder (tests/*.luau, run with
require(geometry3luau.Tests.TestRunner)()) and an `Examples` folder (examples/*.luau).
Each output directory also gets sources.json (for tools/verify_package.luau) and manifest.json.
"""
from pathlib import Path
import argparse
import hashlib
import json
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
NAME = 'geometry3luau'
SOURCE_LIMIT = 199_999  # Roblox script source size limit (bytes)


def collect(profile):
    files = []
    for path in sorted((ROOT / 'src').glob('*.luau')):
        files.append({'name': NAME if path.stem == 'init' else path.stem,
                      'path': '' if path.stem == 'init' else path.stem,
                      'source': path.read_bytes().decode('utf-8')})
    if profile == 'development':
        for folder, prefix in (('tests', 'Tests/'), ('examples', 'Examples/')):
            for path in sorted((ROOT / folder).glob('*.luau')):
                files.append({'name': path.stem, 'path': prefix + path.stem, 'source': path.read_bytes().decode('utf-8')})
    for entry in files:
        size = len(entry['source'].encode('utf-8'))
        if size > SOURCE_LIMIT:
            raise SystemExit(f"Roblox source size limit exceeded: {entry['path'] or NAME} ({size} bytes)")
    return files


def build(profile):
    files = collect(profile)
    doc = ET.Element('roblox', {'version': '4'})
    counter = [0]

    def item(parent, classname, name, source=None, value=None):
        counter[0] += 1
        node = ET.SubElement(parent, 'Item', {'class': classname, 'referent': f'RBX{counter[0]}'})
        props = ET.SubElement(node, 'Properties')
        ET.SubElement(props, 'string', {'name': 'Name'}).text = name
        if source is not None:
            ET.SubElement(props, 'ProtectedString', {'name': 'Source'}).text = source
        if value is not None:
            ET.SubElement(props, 'string', {'name': 'Value'}).text = value
        return node

    main = next(f for f in files if f['path'] == '')
    module = item(doc, 'ModuleScript', NAME, main['source'])
    folders = {'': module}
    for f in files:
        if not f['path']:
            continue
        parent_path, _, _ = f['path'].rpartition('/')
        if parent_path not in folders:
            folders[parent_path] = item(module, 'Folder', parent_path)
        item(folders[parent_path], 'ModuleScript', f['name'], f['source'])
    # license text travels with the source (Boost Software License requirement)
    item(module, 'StringValue', 'LICENSE', value=(ROOT / 'LICENSE').read_bytes().decode('utf-8'))

    output = ROOT / 'dist' / profile
    output.mkdir(parents=True, exist_ok=True)
    artifact = output / f'{NAME}.rbxmx'
    with open(artifact, 'wb') as fh:  # binary: no newline translation on Windows
        ET.ElementTree(doc).write(fh, encoding='utf-8', xml_declaration=False)
    (output / 'sources.json').write_text(json.dumps(files), encoding='utf-8')
    version = re.search(r'Version = "([^"]+)"', main['source']).group(1)
    manifest = {
        'name': NAME, 'profile': profile, 'version': version, 'modules': len(files),
        'sourceBytes': sum(len(f['source'].encode('utf-8')) for f in files),
        'sha256': hashlib.sha256(artifact.read_bytes()).hexdigest(),
        'files': [{'path': f['path'], 'sha256': hashlib.sha256(f['source'].encode('utf-8')).hexdigest()} for f in files],
    }
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    print(f"{profile}: {artifact.relative_to(ROOT)}  version {version}, {manifest['modules']} modules, "
          f"{manifest['sourceBytes']} bytes of source")
    return artifact


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--profile', choices=('development', 'production', 'all'), default='all')
    args = parser.parse_args()
    for p in (('production', 'development') if args.profile == 'all' else (args.profile,)):
        build(p)
