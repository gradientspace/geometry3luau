-- Port of geometry3Sharp mesh_ops/MeshRepairOrientation.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Make triangle orientation consistent within each connected component (OrientComponents), and
-- optionally orient each component "outwards" via ray-cast visibility statistics (SolveGlobalOrientation).
--
-- Luau port notes:
--   * MeshRepairOrientation.new(mesh, spatial?) -- spatial is a DMeshAABBTree3 (built lazily if not given).
--   * Components is a 1-based array of { triangles = {tid...}, outFacing, inFacing }.
--   * The protected Spatial property is the method :Spatial().
--   * OrientComponents picks seed triangles in increasing tid order, matching the C# HashSet.First()
--     enumeration order.

local MathUtil = require(script.Parent.MathUtil)
local IndexUtil = require(script.Parent.IndexUtil)
local Ray3d = require(script.Parent.Ray3d)
local gParallel = require(script.Parent.gParallel)
local MeshEditor = require(script.Parent.MeshEditor)

local InvalidID = -1

local MeshRepairOrientation = {}
MeshRepairOrientation.__index = MeshRepairOrientation
MeshRepairOrientation.ClassName = "MeshRepairOrientation"

function MeshRepairOrientation.new(mesh3, spatial: any?)
	local self = setmetatable({}, MeshRepairOrientation)
	self.Mesh = mesh3
	self.spatial = spatial
	self.Components = {}
	return self
end

-- protected Spatial property
function MeshRepairOrientation:Spatial()
	if self.spatial == nil then
		local DMeshAABBTree3 = require(script.Parent.DMeshAABBTree3)
		self.spatial = DMeshAABBTree3.new(self.Mesh, true)
	end
	return self.spatial
end

-- TODO:
--  - (in merge coincident) don't merge tris with same/opposite normals (option)
--  - after orienting components, try to find adjacent open components and
--    transfer orientation between them
--  - orient via nesting

function MeshRepairOrientation:OrientComponents()
	local Mesh = self.Mesh
	self.Components = {}

	-- remaining = new HashSet<int>(Mesh.TriangleIndices())
	local remaining = {}
	local order = {}
	local nRemaining = 0
	for tid in Mesh:TriangleIndices() do
		remaining[tid] = true
		table.insert(order, tid)
		nRemaining += 1
	end
	local first_i = 1

	local stack = {}
	while nRemaining > 0 do
		local c = { triangles = {}, outFacing = 0, inFacing = 0 }

		table.clear(stack)
		-- remaining.First()
		while remaining[order[first_i]] == nil do
			first_i += 1
		end
		local start = order[first_i]
		remaining[start] = nil
		nRemaining -= 1
		table.insert(c.triangles, start)
		table.insert(stack, start)
		while #stack > 0 do
			local cur = stack[#stack]
			stack[#stack] = nil
			local tcur = Mesh:GetTriangle(cur)

			local nbrs = Mesh:GetTriNeighbourTris(cur)
			for j = 0, 2 do
				local nbr = nbrs[j + 1]
				if remaining[nbr] == nil then
					continue
				end

				local a = tcur[j + 1]
				local b = tcur[((j + 1) % 3) + 1]

				local tnbr = Mesh:GetTriangle(nbr)
				if IndexUtil.find_tri_ordered_edge(b, a, tnbr) == InvalidID then
					Mesh:ReverseTriOrientation(nbr)
				end
				table.insert(stack, nbr)
				remaining[nbr] = nil
				nRemaining -= 1
				table.insert(c.triangles, nbr)
			end
		end

		table.insert(self.Components, c)
	end
end

function MeshRepairOrientation:ComputeStatistics()
	local _ = self:Spatial() -- make sure this exists
	-- Cannot do in parallel because we set a filter on spatial DS.
	-- Also we are doing rays in parallel anyway...
	for _, c in self.Components do
		self:compute_statistics(c)
	end
end

function MeshRepairOrientation:compute_statistics(c)
	local Mesh = self.Mesh
	local spatial = self.spatial
	local triangles = c.triangles
	local NC = #triangles
	c.inFacing = 0
	c.outFacing = 0
	local dist = 2 * Mesh:CachedBounds():DiagonalLength()

	-- only want to raycast triangles in this
	local tris = {}
	for _, tid in triangles do
		tris[tid] = true
	end
	spatial.TriangleFilterF = function(tid)
		return tris[tid] == true
	end

	-- We want to try to figure out what is 'outside' relative to the world.
	-- Assumption is that faces we can hit from far away should be oriented outwards.
	-- So, for each triangle we construct far-away points in positive and negative normal
	-- direction, then raycast back towards the triangle. If we hit the triangle from
	-- one side and not the other, that is evidence we should keep/reverse that triangle.
	-- If it is not hit, or hit from both, that does not provide any evidence.
	-- We collect up this keep/reverse evidence and use the larger to decide on the global orientation.

	gParallel.BlockStartEnd(0, NC - 1, function(a, b)
		for i = a, b do
			local ti = triangles[i + 1]
			local normal, area, centroid = Mesh:GetTriInfo(ti)
			if area < MathUtil.ZeroTolerancef then
				continue
			end

			-- construct far away points
			local pos_pt = centroid + dist * normal
			local neg_pt = centroid - dist * normal

			-- raycast towards triangle from far-away point
			local hit_pos = spatial:FindNearestHitTriangle(Ray3d.new(pos_pt, -normal))
			local hit_neg = spatial:FindNearestHitTriangle(Ray3d.new(neg_pt, normal))
			if hit_pos ~= ti and hit_neg ~= ti then
				continue -- no evidence
			end
			if hit_pos == ti and hit_neg == ti then
				continue -- no evidence (?)
			end

			if hit_neg == ti then
				c.inFacing += area
			elseif hit_pos == ti then
				c.outFacing += area
			end
		end
	end)

	spatial.TriangleFilterF = nil
end

function MeshRepairOrientation:SolveGlobalOrientation()
	self:ComputeStatistics()
	local editor = MeshEditor.new(self.Mesh)
	for _, c in self.Components do
		if c.inFacing > c.outFacing then
			editor:ReverseTriangles(c.triangles)
		end
	end
end

return MeshRepairOrientation
