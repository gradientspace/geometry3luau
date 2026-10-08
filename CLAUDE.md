# geometry3luau — agent guide

geometry3luau is a Luau port of the C# library [geometry3Sharp](https://github.com/gradientspace/geometry3Sharp)
(branch `dotnet8`) for Roblox, plus a thin conversion layer to/from Roblox `EditableMesh` / `MeshPart`.

## Rules
- The port is line-by-line: keep C# class/method names (including the `Vector3d`-style `d` suffixes) and
  algorithm structure. Only the double versions of C# float/double types are ported. Follow **PORTING.md** for every
  translation decision (0-based ids, Index3i as arrays, immutable vectors, properties -> methods,
  ref/out -> extra returns, iterators). When adding a port, put "Port of geometry3Sharp <path>" and the Boost
  license lines in the module header and list API differences there.
- Flat module layout: `src/<ClassName>.luau`, required as `require(script.Parent.ClassName)`. `src/init.luau`
  exposes every module lazily by name, so new modules need no registration.
- Only `RobloxMesh.luau` touches engine services. Everything else must run headless under Lune.
- Keep every module under 200 KB (Roblox script source limit; `tools/package.py` enforces it).
- License: Boost Software License 1.0 (same as geometry3Sharp). Do not add third-party code.

## Commands
- `python tools/bootstrap.py` — install pinned Lune/Rojo/StyLua/Luau into `.tools/` (checksums in `toolchain.lock.json`)
- `python tools/validate.py` — headless tests + package build + verification (the CI gate)
- `python tools/validate.py --suite DMesh3Tests` — run selected suites
- `.tools/lune/lune(.exe) run tools/headless.luau [--suite Name]` — run tests directly
- `python tools/package.py` — build `dist/production/geometry3luau.rbxmx` and `dist/development/geometry3luau.rbxmx`
- In Studio, with the development package: `require(geometry3luau.Tests.TestRunner)()` runs all suites
  including the engine tests (`RobloxMeshTests`).

## Layout
- `src/` library modules (one per C# class; plus `MeshArrays`, `EditableMeshIO`, `RobloxMesh`, `MakeSolid`)
- `tests/` suites (`*Tests.luau` return `function(G3, h) ... return h:results() end`), `TestHarness`,
  `TestMeshes` (test mesh generators), `MockEditableMesh`, `TestRunner` (Studio)
- `examples/` callable examples included in the development package
- `tools/` bootstrap, headless runner, packaging, verification
