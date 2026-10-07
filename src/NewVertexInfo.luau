-- Port of the NewVertexInfo struct from geometry3Sharp mesh/IMesh.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Plain table { v: Vector3d, n: Vector3d, c: Vector3d, uv: Vector2d, bHaveN, bHaveUV, bHaveC }.
-- NewVertexInfo.new(v, n?, c?, uv?) sets the bHave* flags for the arguments that are given,
-- matching the C# constructor overloads.

local Vector3d = require(script.Parent.Vector3d)
local Vector2d = require(script.Parent.Vector2d)

local NewVertexInfo = {}

function NewVertexInfo.new(v, n, c, uv)
	return {
		v = v or Vector3d.Zero,
		n = n or Vector3d.Zero,
		c = c or Vector3d.Zero,
		uv = uv or Vector2d.Zero,
		bHaveN = n ~= nil,
		bHaveC = c ~= nil,
		bHaveUV = uv ~= nil,
	}
end

return NewVertexInfo
