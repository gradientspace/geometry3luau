"""One local/CI gate: install pinned tools, run all headless test suites, build and verify the packages.

    python tools/validate.py              # everything
    python tools/validate.py --tests      # only the headless tests
    python tools/validate.py --suite DMesh3Tests --suite ReducerTests

Steps: bootstrap pinned tools (tools/bootstrap.py) -> headless tests under Lune (tools/headless.luau)
-> package both profiles (tools/package.py) -> Rojo builds of both project files -> verify every
artifact matches the sources (tools/verify_package.luau).
"""
from pathlib import Path
import argparse
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bootstrap import install, tool_path  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


def run(*cmd):
    print('>', ' '.join(str(c) for c in cmd), flush=True)
    result = subprocess.run([str(c) for c in cmd], cwd=ROOT)
    if result.returncode:
        raise SystemExit(f'Gate failed: {Path(str(cmd[0])).name} (exit {result.returncode})')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tests', action='store_true', help='only run the headless tests')
    parser.add_argument('--suite', action='append', default=[], help='run only this test suite (repeatable)')
    args = parser.parse_args()
    started = time.monotonic()
    install(quiet=True)
    lune, rojo = tool_path('lune'), tool_path('rojo')

    suite_args = []
    for s in args.suite:
        suite_args += ['--suite', s]
    (ROOT / '.validation').mkdir(exist_ok=True)
    run(lune, 'run', 'tools/headless.luau', '--out', '.validation/headless.json', *suite_args)
    if args.tests or args.suite:
        return

    run(sys.executable, 'tools/package.py')
    run(rojo, 'build', 'production.project.json', '-o', 'dist/production/geometry3luau.rbxm')
    run(rojo, 'build', 'default.project.json', '-o', 'dist/development/geometry3luau.rbxm')
    run(lune, 'run', 'tools/verify_package.luau')
    print(f'All gates passed in {time.monotonic() - started:.1f}s. Packages are in dist/.')


if __name__ == '__main__':
    main()
