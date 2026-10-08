# Porting conventions (geometry3Sharp → Luau)

geometry3luau is a line-by-line port of [geometry3Sharp](https://github.com/gradientspace/geometry3Sharp)
(branch `dotnet8`). Class names, method names and algorithm structure are kept as close to the C#
as Luau allows, so that the C# source can be used as documentation and so that fixes can be
carried across. This document lists the systematic translations. Follow them for every new port.

## Modules and classes

- One C# class (or a small group of tightly related types) per module in `src/`, named after the C#
  type (`DMesh3.luau`, `Remesher.luau`, ...). The folder is flat: modules require siblings with
  `require(script.Parent.Name)`. `src/init.luau` re-exports every public module.
- Classes are metatables: `local Foo = {}; Foo.__index = Foo; function Foo.new(...)`.
  Instance methods use `:`. C# static methods use `.`.
  Because the first parameter of a `:` method is the object, `Vector3d.Dot(a, b)` and `a:Dot(b)` are
  the same function, which covers most C# static/instance pairs.
- Inheritance: `setmetatable(Derived, { __index = Base })` and `Derived.__index = Derived`.
  Virtual methods are just overridden fields; call the base explicitly with `Base.Method(self, ...)`.
- Partial classes that are large are split into several modules that install methods into the class
  table (see `DMesh3EdgeOps.luau`). Keep each module well under 200 KB (Roblox source limit).
- C# overloads: use one function that dispatches on argument type/count when that is cheap and clear,
  otherwise give the overload a distinct suffix (`FindNearestTriangle` vs `FindNearestTriangleFiltered`).
  Constructor overloads become named constructors (`AxisAlignedBox3d.FromMinMax(a, b)`); document
  them in the module header.
- C# properties become methods with the same name: `mesh.VertexCount` → `mesh:VertexCount()`,
  `v.Length` → `v:Length()`, `box.Center` → `box:Center()`.
  Plain public *fields* stay fields (`remesher.MinEdgeLength`, `v.x`, `box.Min`).
  Property setters with side effects become `SetName(value)` methods.
- `private`/`protected` members keep their C# names (`find_edge`, `vertices_refcount`). Luau has no
  access control; that is fine.
- Enums become frozen tables of numbers with the C# values: `MeshResult.Ok == 0`.
  `[Flags]` enums use `bit32.band/bor`.
- Interfaces (`IProjectionTarget`, `IMesh`, `ImplicitFunction3d`) are duck-typed: an object
  implementing the methods is accepted. Document the required methods in the module header.
- Delegates/`Func<>`/`Action<>` become plain Luau functions.

## Numbers

- Luau numbers are doubles. `float` and `double` both map to number, so only the double version of a
  C# float/double type pair is ported, under its C# name: `Vector3d`, `Vector2d`, `AxisAlignedBox3d`,
  `Frame3d`, `Quaterniond`, `Matrix4d`, ... The float types (`Vector3f`, `Frame3f`, `AxisAlignedBox2f`,
  `VectorArray3f`, ...) are not ported and have no aliases: C# code using them uses the `d` type.
  (Results may differ from C# in the last bits where C# deliberately used float.)
- Geometric types always carry the `d` (or `i`) suffix, which also keeps them distinct from the Roblox
  `Vector2` / `Vector3` types. Where C# only has a float type next to an integer one, the float type is
  renamed with `d`: `DenseGrid2f` / `DenseGrid3f` → `DenseGrid2d` / `DenseGrid3d` (next to `DenseGrid3i`),
  `Vector3fGeoAttribute` → `Vector3dGeoAttribute`.
- Integer division: C# `a / b` on ints → `a // b` **only when both are non-negative**; C# truncates
  toward zero, Luau `//` floors. For possibly-negative values use `MathUtil.IntDiv(a, b)` (truncating).
  Same for `%`: C# remainder has the sign of the dividend; Luau `%` has the sign of the divisor.
  Use `math.fmod` for C# semantics on possibly-negative operands.
- Casts `(int)x` truncate toward zero: use `MathUtil.TruncInt(x)` (not `math.floor`) for possibly
  negative values.
- `double.MaxValue` → `MathUtil.MaxDouble` (finite, 1.7976931348623157e308), `double.MinValue` →
  `-MathUtil.MaxDouble`, `float.MaxValue` → `MathUtil.MaxFloat`, `int.MaxValue` → `MathUtil.MaxInt`.
  Never substitute `math.huge` for these: code relies on them being finite.
- `Math.Sign` → `MathUtil.Sign` (returns 0 for 0, like C#; `math.sign` is also fine in Luau).

## Indices and ids

- Mesh element ids (vertex/triangle/edge ids, grid indices, ...) are **0-based**, exactly as in C#,
  and `DMesh3.InvalidID == -1`. This keeps all id arithmetic (`3*vid`, `>= 0` checks) unchanged.
- Luau arrays are 1-based. `List<int>`/`int[]` → Luau array; C# `list[i]` → `list[i + 1]`,
  `list.Count` → `#list`, `list.Add(x)` → `table.insert(list, x)`.
- `DVector<T>` is ported as a class (`DVector.luau`) whose `get(i)/set(i, v)/insert(v, i)` methods take
  0-based indices like C#. Hot code may read its backing array directly: element `i` is `dv.data[i + 1]`.
  The `data` table of a DVector is never replaced, so it can be cached in a local.
- `Index2i`, `Index3i`, `Index4i` are plain Luau arrays `{a, b}`, `{a, b, c}`, `{a, b, c, d}`:
  `t.a` → `t[1]`, `t.b` → `t[2]`, `t.c` → `t[3]`, `t.d` → `t[4]`, and C# `t[j]` → `t[j + 1]`.
  Helper functions for these types live in `Index.luau` (`Index.Index3i.IndexOf(t, v)` returns a
  0-based position like C#). They are mutable arrays: see "Value types" below.
- `HashSet<int>` → `{ [int]: true }`, `Dictionary<K,V>` → table. **Tables are compared by identity**,
  so a C# `HashSet<Vector3i>` or `Dictionary<Index2i, ...>` must use a scalar key (a linear grid index
  or a string such as `x .. "," .. y .. "," .. z`).

## Value types (C# structs)

C# structs are copied on assignment; Luau tables are shared. To keep this safe:

- **Vectors and quaternions are immutable by convention.** `Vector2d`, `Vector3d`, `Vector3i`,
  `Quaterniond`, `Matrix3d` instances must never have their fields changed after construction
  (the shared constants such as `Vector3d.Zero` are frozen and will error if you try).
  Translate mutating C# operations into new values:
  - `v.Normalize()` → `v = v:Normalized()`; `double len = v.Normalize()` → `v, len = v:NormalizedAndLength()`
  - `v.x = 5` → `v = Vector3d.new(5, v.y, v.z)`; `v[i] = f` → `v = v:With(i, f)` (0-based i)
  - `v += w` → `v = v + w`
- **Boxes, frames, segments, rays, and other compound structs are mutable** (`AxisAlignedBox3d:Contain`,
  `Frame3d:Translate`). Their mutating methods replace their vector fields rather than mutating
  them. When C# copies such a struct (assignment, by-value parameter, property getter returning a
  stored struct) **and either copy is later mutated**, call `:Clone()`. Getters returning a stored box
  (`mesh:CachedBounds()`, `tree:Bounds()`) return a clone.
- `IndexNi` arrays: clone (`table.clone(t)`) before mutating one you did not just create.
- Static readonly struct constants: immutable types expose frozen values (`Vector3d.Zero`);
  mutable types expose constructors (`AxisAlignedBox3d.Empty()` returns a fresh box).

## ref / out parameters

- `out` and `ref` parameters become extra return values, in order, after the normal return value:
  `bool GetEdge(int eID, ref int a, ref int b, ref int t0, ref int t1)` →
  `local ok, a, b, t0, t1 = mesh:GetEdge4(eID)` (see the module header for the exact name).
  `void GetTriVertices(int tID, ref Vector3d v0, ...)` → `local v0, v1, v2 = mesh:GetTriVertices(tID)`.
- C# `out` struct results such as `EdgeSplitInfo` become returned tables:
  `MeshResult r = mesh.SplitEdge(eid, out split)` → `local r, split = mesh:SplitEdge(eid)`.

## Iteration

- `IEnumerable<T>` methods return a Luau iterator function usable in a generic for:
  `foreach (int vid in mesh.VertexIndices())` → `for vid in mesh:VertexIndices() do`.
  The iterator yields `nil` at the end, so it only works for non-nil elements.
- Do not modify the underlying structure while iterating, unless the C# does the same thing and
  the data structure tolerates it.

## Errors, asserts, debugging

- `throw new Exception(msg)` → `error(msg)` (include the C# class/method prefix in the message).
- `Debug.Assert` / `Util.gDevAssert` → `Util.gDevAssert(cond, msg)`; these only check when
  `Util.DebugChecks` is true (default false), like DEBUG builds.
- `System.Console.WriteLine` diagnostics → `Util.DebugPrint(...)` (no-op unless enabled).
- `Stopwatch` timing → `os.clock()`.

## Parallelism and long computations

- `gParallel.ForEach` / `Parallel.For` run sequentially (`gParallel.luau`). Keep the same block
  structure so the code still reads like the C#. `SpinLock` and `Interlocked` disappear.
- Long-running algorithms take a `ProgressCancel` (`Progress` field) like the C#. In Roblox, a
  cancel callback can also yield (`task.wait()`) to avoid script timeouts; see `ProgressCancel.luau`.

## Performance

- Put `--!native` at the top of compute-heavy modules.
- In inner loops it is fine to inline vector math as scalars (`local dx = ax - bx ...`) or to read
  `DVector.data` directly, provided the result is identical to the C# expression. Keep a comment
  showing the original expression when it is not obvious.
- Avoid closures and table allocation in the innermost loops where C# had value types.

## Testing

- Tests live in `tests/*Tests.luau`. Each suite returns `function(G3, harness)`; see
  `tests/TestHarness.luau`. Run everything headless with `python tools/run_tests.py`
  (uses Lune; install pinned tools with `python tools/bootstrap.py`).
