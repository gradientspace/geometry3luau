--!native
-- Port of geometry3Sharp spatial/SpatialFunctions.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- [TODO] delete this file if nobody is using NormalOffset   (C# marks NormalOffset [Obsolete])
--
--   SpatialFunctions.NormalOffset.new(mesh?, spatial?)  fields Mesh, Spatial (ISpatial), Distance = 0.01, UseFaceNormal = true
--   offset:FindNearestAndOffset(pos) -> Vector3d

local function MeshQueries()
	return require(script.Parent.MeshQueries) :: any
end

local SpatialFunctions = {}

-- various offset-surface functions, in class so the compute functions
-- can be passed to other functions
local NormalOffset = {}
NormalOffset.__index = NormalOffset
NormalOffset.ClassName = "SpatialFunctions.NormalOffset"

function NormalOffset.new(mesh, spatial)
	return setmetatable({
		Mesh = mesh,
		Spatial = spatial,
		Distance = 0.01,
		UseFaceNormal = true,
	}, NormalOffset)
end

function NormalOffset:FindNearestAndOffset(pos)
	local mesh = self.Mesh
	local tNearestID = self.Spatial:FindNearestTriangle(pos)
	local q = MeshQueries().TriangleDistance(mesh, tNearestID, pos)
	local b = q.TriangleBaryCoords
	local vHitNormal
	if self.UseFaceNormal == false and mesh:HasVertexNormals() then
		vHitNormal = mesh:GetTriBaryNormal(tNearestID, b.x, b.y, b.z)
	else
		vHitNormal = mesh:GetTriNormal(tNearestID)
	end
	return q.TriangleClosest + self.Distance * vHitNormal
end

SpatialFunctions.NormalOffset = NormalOffset

return SpatialFunctions
