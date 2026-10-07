-- geometry3luau: a Luau port of geometry3Sharp (https://github.com/gradientspace/geometry3Sharp)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Usage:
--   local G3 = require(path.to.geometry3luau)
--   local mesh = G3.DMesh3.new()
--
-- Every module in this package is available as a field with the same name (G3.DMesh3, G3.Remesher,
-- G3.MarchingCubes, ...). Modules are loaded on first access, so requiring the package is cheap and has
-- no side effects. Float/double aliases of the C# API are provided (Vector3f = Vector3d, ...).

local G3 = {}

G3.Version = "0.1.0"

-- C# float types map to the same Luau classes as their double counterparts (see PORTING.md)
local ALIASES = {
	Vector3f = "Vector3d",
	Vector2f = "Vector2d",
	AxisAlignedBox3f = "AxisAlignedBox3d",
	Frame3f = "Frame3d",
	Quaternionf = "Quaterniond",
	Matrix3f = "Matrix3d",
	Triangle3f = "Triangle3d",
	Ray3f = "Ray3d",
	Segment3f = "Segment3d",
	Line3f = "Line3d",
}

-- names that are not modules but live inside another module
local NESTED = {
	Index2i = { "Index", "Index2i" },
	Index3i = { "Index", "Index3i" },
	Index4i = { "Index", "Index4i" },
	gIndices = { "IndexUtil", "gIndices" },
	MeshComponents = { "DMesh3", "MeshComponents" },
	MeshHints = { "DMesh3", "MeshHints" },
	FailMode = { "Util", "FailMode" },
}

local function loadModule(name: string): any
	local child = script:FindFirstChild(name)
	if child == nil then
		return nil
	end
	if child:IsA("ModuleScript") then
		return require(child)
	end
	return nil
end

setmetatable(G3, {
	__index = function(t, key)
		if type(key) ~= "string" then
			return nil
		end
		local value
		local nested = NESTED[key]
		if nested then
			local m = loadModule(nested[1])
			value = m and m[nested[2]]
		else
			value = loadModule(ALIASES[key] or key)
		end
		if value ~= nil then
			rawset(t, key, value)
		end
		return value
	end,
})

-- Load every module now (normally modules load lazily on first access). Returns the package.
function G3.LoadAll()
	for _, child in script:GetChildren() do
		if child:IsA("ModuleScript") then
			local _ = G3[child.Name]
		end
	end
	return G3
end

return G3
