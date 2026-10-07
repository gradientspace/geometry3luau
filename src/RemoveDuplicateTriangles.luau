--!native
-- Port of geometry3Sharp mesh_ops/RemoveDuplicateTriangles.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Remove duplicate triangles (triangles whose vertex positions coincide within VertexTolerance,
-- optionally requiring the same orientation).
--
-- Luau port notes:
--   * RemoveDuplicateTriangles.new(mesh); fields VertexTolerance, CheckOrientation, Removed; :Apply() -> bool
--   * The private TriCentroids IPointSet is RemoveDuplicateTriangles.TriCentroids.new(mesh).
--   * The C# hash-query retry `buffer = new int[buffer.Length]` would loop forever if more than 1024
--     centroids coincide; here the buffer capacity is doubled instead.

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local MeshResult = require(script.Parent.MeshResult)
local PointSetHashtable = require(script.Parent.PointSetHashtable)

-- present mesh tri centroids as a PointSet
local TriCentroids = {}
TriCentroids.__index = TriCentroids
function TriCentroids.new(mesh)
	return setmetatable({ Mesh = mesh }, TriCentroids)
end
function TriCentroids:VertexCount(): number
	return self.Mesh:TriangleCount()
end
function TriCentroids:MaxVertexID(): number
	return self.Mesh:MaxTriangleID()
end
function TriCentroids:HasVertexNormals(): boolean
	return false
end
function TriCentroids:HasVertexColors(): boolean
	return false
end
function TriCentroids:GetVertex(i: number)
	return self.Mesh:GetTriCentroid(i)
end
function TriCentroids:GetVertexNormal(i: number)
	return Vector3d.AxisY
end
function TriCentroids:GetVertexColor(i: number)
	return Vector3d.One
end
function TriCentroids:IsVertex(tID: number): boolean
	return self.Mesh:IsTriangle(tID)
end
-- iterators allow us to work with gaps in index space
function TriCentroids:VertexIndices(): () -> number?
	return self.Mesh:TriangleIndices()
end
function TriCentroids:Timestamp(): number
	return self.Mesh:Timestamp()
end

local RemoveDuplicateTriangles = {}
RemoveDuplicateTriangles.__index = RemoveDuplicateTriangles
RemoveDuplicateTriangles.ClassName = "RemoveDuplicateTriangles"
RemoveDuplicateTriangles.TriCentroids = TriCentroids

function RemoveDuplicateTriangles.new(mesh)
	local self = setmetatable({}, RemoveDuplicateTriangles)
	self.Mesh = mesh
	self.VertexTolerance = MathUtil.ZeroTolerancef
	self.CheckOrientation = true
	self.Removed = 0
	return self
end

local function is_same_triangle(a, b, c, x, y, z, tolSqr: number): boolean
	if a:DistanceSquared(x) < tolSqr then
		if b:DistanceSquared(y) < tolSqr and c:DistanceSquared(z) < tolSqr then
			return true
		end
		if b:DistanceSquared(z) < tolSqr and c:DistanceSquared(y) < tolSqr then
			return true
		end
	elseif a:DistanceSquared(y) < tolSqr then
		if b:DistanceSquared(x) < tolSqr and c:DistanceSquared(z) < tolSqr then
			return true
		end
		if b:DistanceSquared(z) < tolSqr and c:DistanceSquared(x) < tolSqr then
			return true
		end
	elseif a:DistanceSquared(z) < tolSqr then
		if b:DistanceSquared(x) < tolSqr and c:DistanceSquared(y) < tolSqr then
			return true
		end
		if b:DistanceSquared(y) < tolSqr and c:DistanceSquared(x) < tolSqr then
			return true
		end
	end
	return false
end

function RemoveDuplicateTriangles:Apply(): boolean
	self.Removed = 0
	local Mesh = self.Mesh
	local VertexTolerance = self.VertexTolerance

	local merge_r2 = VertexTolerance * VertexTolerance

	-- construct hash table for edge midpoints
	local pointset = TriCentroids.new(Mesh)
	local hash = PointSetHashtable.new(pointset)
	local hashN = (Mesh:TriangleCount() > 100000) and 128 or 64
	hash:Build(hashN)

	local MaxTriID = Mesh:MaxTriangleID()

	-- remove duplicate triangles
	local buffer = {}
	local capacity = 1024
	for tid = 0, MaxTriID - 1 do
		if not Mesh:IsTriangle(tid) then
			continue
		end

		local centroid = Mesh:GetTriCentroid(tid)
		local ok, N = hash:FindInBall(centroid, VertexTolerance, buffer, capacity)
		while ok == false do
			capacity *= 2
			ok, N = hash:FindInBall(centroid, VertexTolerance, buffer, capacity)
		end
		if N == 1 and buffer[1] ~= tid then
			error("RemoveDuplicateTriangles.Apply: how could this happen?!")
		end
		if N <= 1 then
			continue -- unique edge
		end

		local a, b, c = Mesh:GetTriVertices(tid)
		local n = MathUtil.Normal(a, b, c)

		for i = 1, N do
			local other = buffer[i]
			if other ~= tid then
				local x, y, z = Mesh:GetTriVertices(other)
				if is_same_triangle(a, b, c, x, y, z, merge_r2) == false then
					continue
				end

				if self.CheckOrientation then
					local n2 = MathUtil.Normal(x, y, z)
					if n:Dot(n2) < 0.99 then
						continue
					end
				end

				local result = Mesh:RemoveTriangle(other, true, false)
				if result == MeshResult.Ok then
					self.Removed += 1
				end
			end
		end
	end

	return true
end

return RemoveDuplicateTriangles
