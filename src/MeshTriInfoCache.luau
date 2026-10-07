--!native
-- Port of geometry3Sharp mesh/MeshCaches.cs (MeshTriInfoCache)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Basic cache of per-triangle information for a DMesh3.
--   MeshTriInfoCache.new(mesh)
--   fields Centroids, Normals (DVector of Vector3d), Areas (DVector of number), indexed by triangle id
--   cache:GetTriInfo(tid) -> n, a, c        (C# GetTriInfo(tid, ref n, ref a, ref c))
-- Note: the C# version sizes the vectors with mesh.TriangleCount (which fails for non-compact meshes);
-- here they are sized with mesh.MaxTriangleID so any valid triangle id can be stored.

local DVector = require(script.Parent.DVector)
local gParallel = require(script.Parent.gParallel)

local MeshTriInfoCache = {}
MeshTriInfoCache.__index = MeshTriInfoCache
MeshTriInfoCache.ClassName = "MeshTriInfoCache"

function MeshTriInfoCache.new(mesh)
	local self = setmetatable({}, MeshTriInfoCache)
	local NT = mesh:MaxTriangleID()
	self.Centroids = DVector.new()
	self.Centroids:resize(NT)
	self.Normals = DVector.new()
	self.Normals:resize(NT)
	self.Areas = DVector.new()
	self.Areas:resize(NT)
	local cd, nd, ad = self.Centroids.data, self.Normals.data, self.Areas.data
	gParallel.ForEach(mesh:TriangleIndices(), function(tid)
		local n, a, c = mesh:GetTriInfo(tid)
		cd[tid + 1] = c
		nd[tid + 1] = n
		ad[tid + 1] = a
	end)
	return self
end

function MeshTriInfoCache:GetTriInfo(tid: number)
	return self.Normals.data[tid + 1], self.Areas.data[tid + 1], self.Centroids.data[tid + 1]
end

return MeshTriInfoCache
