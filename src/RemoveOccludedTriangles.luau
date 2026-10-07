-- Port of geometry3Sharp mesh_ops/RemoveOccludedTriangles.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Remove "occluded" triangles, ie triangles on the "inside" of the mesh.
-- This is a fuzzy definition, current implementation is basically computing
-- something akin to ambient occlusion, and if face is fully occluded, then
-- we classify it as inside and remove it.
--
-- Luau port notes:
--   * RemoveOccludedTriangles.new(mesh, spatial?) -- spatial is a DMeshAABBTree3
--     (C# constructors RemoveOccludedTriangles(mesh) / RemoveOccludedTriangles(mesh, spatial)).
--   * Fields: RemovedT (1-based array of removed tids), RemoveFailed, PerVertex, NormalOffset,
--     WindingIsoValue, InsideMode (RemoveOccludedTriangles.CalculationMode), Progress (ProgressCancel).
--   * Uses DMeshAABBTree3: IsInside, WindingNumber, FastWindingNumber, FindNearestHitTriangle.
--   * gParallel loops run sequentially (no SpinLock needed).

local MathUtil = require(script.Parent.MathUtil)
local Vector3d = require(script.Parent.Vector3d)
local BitArray = require(script.Parent.BitArray)
local Ray3d = require(script.Parent.Ray3d)
local gParallel = require(script.Parent.gParallel)
local MeshEditor = require(script.Parent.MeshEditor)
local SimpleHoleFiller = require(script.Parent.SimpleHoleFiller)

local InvalidID = -1

local RemoveOccludedTriangles = {}
RemoveOccludedTriangles.__index = RemoveOccludedTriangles
RemoveOccludedTriangles.ClassName = "RemoveOccludedTriangles"

RemoveOccludedTriangles.CalculationMode = table.freeze({
	RayParity = 0,
	AnalyticWindingNumber = 1,
	FastWindingNumber = 2,
	SimpleOcclusionTest = 3,
})
local CalculationMode = RemoveOccludedTriangles.CalculationMode

function RemoveOccludedTriangles.new(mesh, spatial: any?)
	local self = setmetatable({}, RemoveOccludedTriangles)
	self.Mesh = mesh
	self.Spatial = spatial

	-- indices of removed triangles. List will be empty if nothing removed
	self.RemovedT = nil
	-- Mesh.RemoveTriange() can return false, if that happens, this will be true
	self.RemoveFailed = false
	-- if true, then we discard tris if any vertex is occluded.
	-- Otherwise we discard based on tri centroids
	self.PerVertex = false
	-- we nudge points out by this amount to try to counteract numerical issues
	self.NormalOffset = MathUtil.ZeroTolerance
	-- use this as winding isovalue for WindingNumber mode
	self.WindingIsoValue = 0.5
	self.InsideMode = CalculationMode.RayParity
	-- Set this to be able to cancel running remesher
	self.Progress = nil
	return self
end

-- if this returns true, abort computation.
function RemoveOccludedTriangles:Cancelled(): boolean
	return (self.Progress ~= nil) and self.Progress:Cancelled() or false
end

function RemoveOccludedTriangles:Apply(): boolean
	local Mesh = self.Mesh
	local DMeshAABBTree3 = require(script.Parent.DMeshAABBTree3)
	local DMesh3 = require(script.Parent.DMesh3)

	local testAgainstMesh = Mesh
	if self.InsideMode == CalculationMode.RayParity then
		local MeshBoundaryLoops = require(script.Parent.MeshBoundaryLoops)
		local loops = MeshBoundaryLoops.new(testAgainstMesh)
		if loops:Count() > 0 then
			testAgainstMesh = DMesh3.FromCopy(Mesh)
			for _, loop in loops.Loops do
				if self:Cancelled() then
					return false
				end
				local filler = SimpleHoleFiller.new(testAgainstMesh, loop)
				filler:Fill()
			end
		end
	end

	local spatial = (self.Spatial ~= nil and testAgainstMesh == Mesh) and self.Spatial or DMeshAABBTree3.new(testAgainstMesh, true)
	if self.InsideMode == CalculationMode.AnalyticWindingNumber then
		spatial:WindingNumber(Vector3d.Zero)
	elseif self.InsideMode == CalculationMode.FastWindingNumber then
		spatial:FastWindingNumber(Vector3d.Zero)
	end

	if self:Cancelled() then
		return false
	end

	-- ray directions
	local ray_dirs = nil
	local NR = 0
	if self.InsideMode == CalculationMode.SimpleOcclusionTest then
		ray_dirs = {
			Vector3d.AxisX,
			-Vector3d.AxisX,
			Vector3d.AxisY,
			-Vector3d.AxisY,
			Vector3d.AxisZ,
			-Vector3d.AxisZ,
		}
		NR = #ray_dirs
	end

	local InsideMode = self.InsideMode
	local WindingIsoValue = self.WindingIsoValue
	local isOccludedF = function(pt): boolean
		if InsideMode == CalculationMode.RayParity then
			return spatial:IsInside(pt)
		elseif InsideMode == CalculationMode.AnalyticWindingNumber then
			return spatial:WindingNumber(pt) > WindingIsoValue
		elseif InsideMode == CalculationMode.FastWindingNumber then
			return spatial:FastWindingNumber(pt) > WindingIsoValue
		else
			for k = 1, NR do
				local hit_tid = spatial:FindNearestHitTriangle(Ray3d.new(pt, ray_dirs[k]))
				if hit_tid == InvalidID then
					return false
				end
			end
			return true
		end
	end

	local cancel = false
	local NormalOffset = self.NormalOffset

	local vertices = nil
	if self.PerVertex then
		vertices = BitArray.new(Mesh:MaxVertexID())

		local normals = nil
		if Mesh:HasVertexNormals() == false then
			local MeshNormals = require(script.Parent.MeshNormals)
			normals = MeshNormals.new(Mesh)
			normals:Compute()
		end

		gParallel.ForEach(Mesh:VertexIndices(), function(vid)
			if cancel then
				return
			end
			if vid % 10 == 0 then
				cancel = self:Cancelled()
			end

			local c = Mesh:GetVertex(vid)
			local n = (normals == nil) and Mesh:GetVertexNormal(vid) or normals:Get(vid)
			c = c + n * NormalOffset
			vertices:Set(vid, isOccludedF(c))
		end)
	end
	if self:Cancelled() then
		return false
	end

	local RemovedT = {}
	self.RemovedT = RemovedT

	gParallel.ForEach(Mesh:TriangleIndices(), function(tid)
		if cancel then
			return
		end
		if tid % 10 == 0 then
			cancel = self:Cancelled()
		end

		local inside = false
		if self.PerVertex then
			local tri = Mesh:GetTriangle(tid)
			inside = vertices:Get(tri[1]) or vertices:Get(tri[2]) or vertices:Get(tri[3])
		else
			local c = Mesh:GetTriCentroid(tid)
			local n = Mesh:GetTriNormal(tid)
			c = c + n * NormalOffset
			inside = isOccludedF(c)
		end

		if inside then
			table.insert(RemovedT, tid)
		end
	end)

	if self:Cancelled() then
		return false
	end

	if #RemovedT > 0 then
		local editor = MeshEditor.new(Mesh)
		local bOK = editor:RemoveTriangles(RemovedT, true)
		self.RemoveFailed = (bOK == false)
	end

	return true
end

return RemoveOccludedTriangles
