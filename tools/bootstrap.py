"""Install the pinned tool binaries listed in toolchain.lock.json into .tools/<name>/.

Downloads only the exact official release archives and verifies their SHA-256 before extracting.
Standard library only. Usage: python tools/bootstrap.py
"""
from pathlib import Path
import hashlib
import io
import json
import platform
import sys
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / '.tools'


def platform_key():
    system = {'Linux': 'linux', 'Darwin': 'macos', 'Windows': 'windows'}.get(platform.system(), platform.system().lower())
    arch = {'arm64': 'aarch64', 'AMD64': 'x86_64', 'x86_64': 'x86_64', 'aarch64': 'aarch64'}.get(platform.machine(), platform.machine())
    return f'{system}-{arch}'


def tool_path(name, program=None):
    """Path of an installed tool binary (program defaults to the tool's first program)."""
    lock = json.loads((ROOT / 'toolchain.lock.json').read_text())
    entry = lock['platforms'][platform_key()][name]
    return TOOLS / name / (program or entry['programs'][0])


def install(quiet=False):
    key = platform_key()
    lock = json.loads((ROOT / 'toolchain.lock.json').read_text())
    if key not in lock['platforms']:
        raise SystemExit(f'Unsupported toolchain platform: {key}')
    TOOLS.mkdir(exist_ok=True)
    for name, entry in lock['platforms'][key].items():
        target = TOOLS / name
        receipt = target / 'receipt.json'
        if receipt.exists():
            saved = json.loads(receipt.read_text())
            if saved.get('archive') == entry['sha256'] and all(
                    (target / p).is_file() and hashlib.sha256((target / p).read_bytes()).hexdigest() == h
                    for p, h in saved.get('binaries', {}).items()) and set(saved.get('binaries', {})) == set(entry['programs']):
                continue
        if not quiet:
            print(f'Downloading {name} {entry["version"]} ({key})', flush=True)
        request = urllib.request.Request(entry['url'], headers={'User-Agent': 'geometry3luau-toolchain'})
        with urllib.request.urlopen(request, timeout=120) as response:
            archive = response.read()
        if hashlib.sha256(archive).hexdigest() != entry['sha256']:
            raise SystemExit(f'Checksum mismatch: {name}')
        target.mkdir(exist_ok=True)
        hashes = {}
        with zipfile.ZipFile(io.BytesIO(archive)) as source:
            for program in entry['programs']:
                members = [p for p in source.namelist() if Path(p).name == program and not p.endswith('/')]
                if len(members) != 1:
                    raise SystemExit(f'Missing or ambiguous binary in archive: {program}')
                data = source.read(members[0])
                path = target / program
                path.write_bytes(data)
                path.chmod(0o755)
                hashes[program] = hashlib.sha256(data).hexdigest()
        receipt.write_text(json.dumps({'archive': entry['sha256'], 'version': entry['version'], 'binaries': hashes}, indent=2) + '\n')
    return TOOLS


if __name__ == '__main__':
    print(install())
