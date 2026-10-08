# geometry3luau

A Luau port of [geometry3Sharp](https://github.com/gradientspace/geometry3Sharp) (branch `dotnet8`) for Roblox:
dynamic triangle meshes (`DMesh3`), simplification (`Reducer`), isotropic remeshing (`Remesher`), AABB trees with
fast winding numbers (`DMeshAABBTree3`), winding-number and signed-distance grids, marching cubes, mesh repair
(`MeshAutoRepair`, `MergeCoincidentEdges`, hole filling), and conversion to and from Roblox `EditableMesh` /
`MeshPart`.

The port follows the C# source line by line, with the same class and method names, so the geometry3Sharp code
and documentation apply directly. [PORTING.md](PORTING.md) lists the systematic differences (0-based ids, index
tuples as arrays, immutable vectors, properties as methods, `out` parameters as extra return values, iterators).

Requiring the library has no side effects. Only `RobloxMesh` touches engine services, and only when called;
everything else is plain Luau and also runs outside Roblox (the test suite runs under [Lune](https://lune-org.github.io/docs)).

## Install

Use the prebuilt model: `dist/production/geometry3luau.rbxmx` (runtime modules only), or
`dist/development/geometry3luau.rbxmx` (adds the `Tests` and `Examples` folders). CI uploads both as the
`geometry3luau-packages` artifact. To build them yourself (Python 3.10+, no other dependencies):

```sh
python tools/package.py          # writes dist/production/... and dist/development/...
python tools/validate.py         # installs pinned tools, runs all tests, builds and verifies the packages
```

Insert the `.rbxmx` into `ReplicatedStorage` (or `ServerStorage`) and require it:

```lua
local G3 = require(game.ReplicatedStorage.geometry3luau)
print(G3.Version)
```

Every module is a field of the package (`G3.DMesh3`, `G3.Remesher`, `G3.MarchingCubes`, ...), loaded on first
access. Rojo users can sync `production.project.json` (or `default.project.json` for tests/examples) instead.

## Quick start

```lua
local G3 = require(game.ReplicatedStorage.geometry3luau)
local V = G3.Vector3d.new

-- build a mesh
local mesh = G3.DMesh3.new()
local a, b, c, d = mesh:AppendVertex(V(0, 0, 0)), mesh:AppendVertex(V(1, 0, 0)), mesh:AppendVertex(V(0, 1, 0)), mesh:AppendVertex(V(0, 0, 1))
mesh:AppendTriangle(a, c, b); mesh:AppendTriangle(a, b, d); mesh:AppendTriangle(a, d, c); mesh:AppendTriangle(b, c, d)
print(mesh:VertexCount(), mesh:TriangleCount(), mesh:IsClosed())

-- read a MeshPart (world space), simplify it, and write it back as a new MeshPart
local m = G3.RobloxMesh.FromMeshPart(workspace.MyMeshPart)
local reducer = G3.Reducer.new(m)
reducer:ReduceToTriangleCount(2000)
local part = G3.RobloxMesh.ToMeshPart(m, { parent = workspace })
```

### Make solid

`MakeSolid` turns any triangle mesh (open, self-intersecting, overlapping pieces, inconsistent orientation) into a
closed, manifold solid: it samples the fast mesh winding number on a narrow-band grid
(`MeshWindingNumberGrid`), extracts the 0.5 iso-surface with `MarchingCubes`, then remeshes (`Remesher`, projected
back onto the surface) and simplifies (`Reducer`).

```lua
local solid = G3.MakeSolid.new(mesh)
solid.GridResolution = 96              -- cells across the longest axis (or set solid.CellSize)
solid.TargetTriangleCount = 8000       -- optional simplification
solid.Progress = G3.ProgressCancel.Yielding(0.1)   -- yield every 0.1s to avoid script timeouts
local result = solid:Compute()         -- closed DMesh3
```

`SolidMode = G3.MakeSolid.SolidModes.SignedDistance` uses a signed distance grid instead, and supports
`SignedDistanceOffset` to grow or shrink the solid. The `Examples.MakeSolidMeshPart` module (development package)
does the whole MeshPart → solid MeshPart round trip. Each stage is also usable on its own; see the header of
`src/MakeSolid.luau` for the exact calls.

### Repair

```lua
local repair = G3.MeshAutoRepair.new(mesh)
repair:Apply()
local fixed = repair.Mesh   -- welded, oriented, holes filled, duplicates/degenerates removed
```

## Roblox conversion

| Module | Purpose |
| --- | --- |
| `EditableMeshIO` | `Read(editable, options)` / `Write(mesh, editable, options)` for any object with the `EditableMesh` method API. It does not depend on Roblox types: pass `Vector3`/`Vector2`/`Color3` constructors in `options` (the Roblox globals are the default). Supports welding (`weld = "ids" \| "positions" \| "none"`), per-corner normals/UVs stored in the `TriNormals`/`TriUVs` attributes, vertex normals/UVs/colors, and position transforms. |
| `MeshArrays` | Plain-array conversion (`ToArrays`/`FromArrays`), vertex welding, and spatial partitioning of a mesh into pieces under the `EditableMesh` limits (`PartitionTriangles`, `ExtractSubmesh`). |
| `RobloxMesh` | Engine bindings: `FromEditableMesh`, `ToEditableMesh`, `FromMeshPart` (world/local/mesh space), `ToMeshPart`, `ToMeshParts` (splits meshes over 20,000 triangles / 60,000 vertices), `ApplyToMeshPart`. |

DMesh3 and `EditableMesh` both treat counter-clockwise triangles as front-facing, so faces are converted without
reordering. Creating editable meshes and mesh parts needs the corresponding Roblox API access (Studio, or a server
with `EditableMesh` permissions).

## Long computations in Roblox

Algorithms take a `ProgressCancel` (`Progress` field, as in C#) and poll it frequently. In Roblox,
`G3.ProgressCancel.Yielding(seconds)` returns one that calls `task.wait()` periodically, so large remeshes or
grids can run without hitting the script timeout. All "parallel" C# code runs sequentially.

## What is ported

Core: `DMesh3` (+ attributes, edge operators, validity checks), `DMesh3Builder`, `DVector`, `RefCountVector`,
`SmallListSet`, `VectorArray`, vector/matrix/quaternion/frame/box/interval types (including `Vector4d`,
`Matrix2d`, `Matrix4d`, `AxisAlignedBox2d`, `Box2d`), `MathUtil`, `IndexUtil`, `MeshIndexUtil`, `MeshTransforms`.
Spatial and queries: `DMeshAABBTree3` (nearest point, ray casts, intersections, winding number, fast winding
number), `FastWindingMath`, distance and intersection classes, `MeshQueries`, `MeshMeasurements`.
Refinement: `MeshRefinerBase`, `Reducer`, `Remesher`, `RemesherPro`, `RegionRemesher`, `MeshConstraints`,
`MeshConstraintUtil`, projection targets.
Grids and implicits: `DenseGrid3d/i`, `Bitmap3`, `MeshWindingNumberGrid`, `MeshSignedDistanceGrid`,
`CachingMeshSDF`, `MeshScalarSamplingGrid`, `Implicit3d`, `DenseGridTrilinearImplicit`, `MarchingCubes`.
Repair and editing: `MeshAutoRepair`, `MergeCoincidentEdges`, `RemoveDuplicateTriangles`, `MeshRepairOrientation`,
`MinimalHoleFill`, `SimpleHoleFiller`, `RemoveOccludedTriangles`, `MeshEditor`, `MeshBoundaryLoops`, selections,
`DSubmesh3`, `RegionOperator`, `MeshConnectedComponents`.
Each module header names the C# file it ports and any API differences.

## Development

```sh
python tools/bootstrap.py                     # pinned Lune / Rojo / StyLua / Luau into .tools/ (SHA-256 checked)
python tools/validate.py                      # tests + packages + verification (CI gate)
python tools/validate.py --suite RemesherTests
```

In Studio, with the development package: `require(geometry3luau.Tests.TestRunner)()` runs every suite, including
the engine tests in `RobloxMeshTests`. See [CLAUDE.md](CLAUDE.md) for the repository layout.

## License

[Boost Software License 1.0](LICENSE), same as geometry3Sharp. geometry3Sharp is copyright Ryan Schmidt / gradientspace.
