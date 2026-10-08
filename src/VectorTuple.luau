-- Port of geometry3Sharp math/VectorTuple.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- In C# these structs are used in place of local stack arrays (which C# does not support, but is common
-- in C++ code). In Luau a tuple is simply a plain 1-based array of vectors:
--   C# Vector3dTuple3 t = new Vector3dTuple3(a, b, c)   ->  local t = { a, b, c }   (or VectorTuple.Vector3dTuple3(a, b, c))
--   C# t.V0 / t.V1 / t.V2 / t.V3                         ->  t[1] / t[2] / t[3] / t[4]
--   C# t[i] (0-based)                                    ->  t[i + 1]
--   C# t[i] = v                                          ->  t[i + 1] = v
-- Unlike the C# structs, these arrays are shared on assignment: use table.clone(t) where C# copies a
-- tuple and either copy is later modified (the vectors inside are immutable, so a shallow copy is enough).
--
-- Types: Vector3dTuple2, Vector3dTuple3 (C# Vector3dTuple2/3), Vector2dTuple2, Vector2dTuple3, Vector2dTuple4
-- (C# Vector2dTuple2/3/4). The float Vector3fTuple3 is the same as Vector3dTuple3.
-- Each constructor accepts nil arguments (C# default struct: zero vectors) and fills them with Zero.

local Vector3d = require(script.Parent.Vector3d)
local Vector2d = require(script.Parent.Vector2d)

local VectorTuple = {}

export type Vector3dTuple = { Vector3d.Vector3d }
export type Vector2dTuple = { Vector2d.Vector2d }

-- new Vector3dTuple2(v0, v1)
function VectorTuple.Vector3dTuple2(v0: Vector3d.Vector3d?, v1: Vector3d.Vector3d?): Vector3dTuple
	return { v0 or Vector3d.Zero, v1 or Vector3d.Zero }
end

-- new Vector3dTuple3(v0, v1, v2)
function VectorTuple.Vector3dTuple3(v0: Vector3d.Vector3d?, v1: Vector3d.Vector3d?, v2: Vector3d.Vector3d?): Vector3dTuple
	return { v0 or Vector3d.Zero, v1 or Vector3d.Zero, v2 or Vector3d.Zero }
end

-- new Vector2dTuple2(v0, v1)
function VectorTuple.Vector2dTuple2(v0: Vector2d.Vector2d?, v1: Vector2d.Vector2d?): Vector2dTuple
	return { v0 or Vector2d.Zero, v1 or Vector2d.Zero }
end

-- new Vector2dTuple3(v0, v1, v2)
function VectorTuple.Vector2dTuple3(v0: Vector2d.Vector2d?, v1: Vector2d.Vector2d?, v2: Vector2d.Vector2d?): Vector2dTuple
	return { v0 or Vector2d.Zero, v1 or Vector2d.Zero, v2 or Vector2d.Zero }
end

-- new Vector2dTuple4(v0, v1, v2, v3)
function VectorTuple.Vector2dTuple4(
	v0: Vector2d.Vector2d?,
	v1: Vector2d.Vector2d?,
	v2: Vector2d.Vector2d?,
	v3: Vector2d.Vector2d?
): Vector2dTuple
	return { v0 or Vector2d.Zero, v1 or Vector2d.Zero, v2 or Vector2d.Zero, v3 or Vector2d.Zero }
end

return VectorTuple
