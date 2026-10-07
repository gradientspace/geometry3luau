-- Port of geometry3Sharp mesh/RegionRemesher.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Remesher that operates on a sub-region (triangle set) of a mesh: the region is extracted into a
-- DSubmesh3, remeshed with its region-boundary edges fixed, and re-inserted with BackPropropagate().
--
-- Luau port notes:
--   * RegionRemesher inherits Remesher (metatable inheritance). RegionRemesher.new(mesh, regionTris)
--     where regionTris is an array, set, iterator function or selection object.
--   * Properties -> methods: CurrentBaseTriangles() (1-based array of base tids).
--   * RegionRemesher.QuickRemeshFlags is the C# [Flags] enum (use bit32).
--   * QuickRemesh overloads:
--       RegionRemesher.QuickRemesh(mesh, tris, minEdgeLen, maxEdgeLen, smoothSpeed, rounds, target, flags?)
--       RegionRemesher.QuickRemeshTarget(mesh, tris, targetEdgeLen, smoothSpeed, rounds, target, flags?)
--     (target may be nil; flags default PreventNormalFlips)

local Remesher = require(script.Parent.Remesher)
local MeshConstraints = require(script.Parent.MeshConstraints)
local MeshConstraintUtil = require(script.Parent.MeshConstraintUtil)
local DSubmesh3 = require(script.Parent.DSubmesh3)
local MeshEditor = require(script.Parent.MeshEditor)
local IndexUtil = require(script.Parent.IndexUtil)
local Util = require(script.Parent.Util)
local FaceGroupUtil = require(script.Parent.FaceGroupUtil)

local InvalidID = -1

local RegionRemesher = setmetatable({}, { __index = Remesher })
RegionRemesher.__index = RegionRemesher
RegionRemesher.ClassName = "RegionRemesher"

RegionRemesher.QuickRemeshFlags = table.freeze({
	NoFlags = 0,
	PreventNormalFlips = 1,
})
local QuickRemeshFlags = RegionRemesher.QuickRemeshFlags

function RegionRemesher.new(mesh, regionTris: any)
	local self = setmetatable({}, RegionRemesher)
	Remesher.init(self, nil)
	local tris = DSubmesh3.toArray(regionTris)

	self.BaseMesh = mesh
	self.Region = DSubmesh3.new(mesh, tris)
	self.Region:ComputeBoundaryInfo(tris, #tris)
	self.mesh = self.Region.SubMesh

	-- this is only valid after BackPropagate() call!! maps submeshverts to base mesh
	self.ReinsertSubToBaseMapV = nil
	-- handle a tricky problem...see comments for DuplicateTriBehavior enum
	self.ReinsertDuplicateTriBehavior = MeshEditor.DuplicateTriBehavior.AssertContinue

	self.cur_base_tris = table.clone(tris)

	-- constrain region-boundary edges
	-- By default is initialized w/ all boundary constraints
	-- You can add more, but don't screw up!
	self.bdry_constraints = MeshConstraints.new()
	MeshConstraintUtil.FixSubmeshBoundaryEdges(self.bdry_constraints, self.Region)
	self:SetExternalConstraints(self.bdry_constraints)
	return self
end

-- list of sub-region triangles. This is either the input regionTris,
-- or the submesh triangles after they are re-inserted.
function RegionRemesher:CurrentBaseTriangles(): { number }
	return self.cur_base_tris
end

-- After remeshing we may create an internal edge between two boundary vertices [a,b].
-- Those vertices will be merged with vertices c and d in the base mesh. If the edge
-- [c,d] already exists in the base mesh, then after the merge we would have at least
-- 3 triangles at this edge. Dang.
--
-- A common example is a 'fin' triangle that would duplicate a
-- 'fin' on the border of the base mesh after removing the submesh, but this situation can
-- arise anywhere (eg think about one-triangle-wide strips).
--
-- This is very hard to remove, but we can at least avoid creating non-manifold edges (which
-- with the current DMesh3 will be prevented, hence leaving a hole) by splitting the
-- internal edge in the submesh (which presumably we were remeshing anyway, so changes are ok).
function RegionRemesher:RepairPossibleNonManifoldEdges()
	-- [TODO] do we need to repeat this more than once? I don't think so...
	local Region = self.Region
	local SubMesh = Region.SubMesh

	-- repair submesh
	local NE = SubMesh:MaxEdgeID()
	local split_edges = {}
	for eid = 0, NE - 1 do
		if SubMesh:IsEdge(eid) == false then
			continue
		end
		if SubMesh:IsBoundaryEdge(eid) then
			continue
		end
		local edgev = SubMesh:GetEdgeV(eid)
		if SubMesh:IsBoundaryVertex(edgev[1]) and SubMesh:IsBoundaryVertex(edgev[2]) then
			-- ok, we have an internal edge where both verts are on the boundary
			-- now check if it is an edge in the base mesh
			local base_a = Region:MapVertexToBaseMesh(edgev[1])
			local base_b = Region:MapVertexToBaseMesh(edgev[2])
			if base_a ~= InvalidID and base_b ~= InvalidID then
				-- both vertices in base mesh...right?
				Util.gDevAssert(Region.BaseMesh:IsVertex(base_a) and Region.BaseMesh:IsVertex(base_b))
				local base_eid = Region.BaseMesh:FindEdge(base_a, base_b)
				if base_eid ~= InvalidID then
					table.insert(split_edges, eid)
				end
			end
		end
	end

	-- split any problem edges we found and repeat this loop
	for i = 1, #split_edges do
		SubMesh:SplitEdge(split_edges[i])
	end
end

-- set group ID for entire submesh
function RegionRemesher:SetSubmeshGroupID(gid: number)
	FaceGroupUtil.SetGroupID(self.Region.SubMesh, gid)
end

-- Remove the original submesh region and merge in the remeshed version.
-- You can call this multiple times as the base-triangle-set is updated.
--
-- By default, we allow the submesh to be modified to prevent creation of
-- non-manifold edges. You can disable this, however then some of the submesh
-- triangles may be discarded.
--
-- Returns false if there were errors in insertion, ie if some triangles
-- failed to insert. Does not revert changes that were successful.
function RegionRemesher:BackPropropagate(bAllowSubmeshRepairs: boolean?): boolean
	if bAllowSubmeshRepairs ~= false then
		self:RepairPossibleNonManifoldEdges()
	end
	local BaseMesh = self.BaseMesh

	-- remove existing submesh triangles
	local editor = MeshEditor.new(BaseMesh)
	editor:RemoveTriangles(self.cur_base_tris, true)

	-- insert new submesh
	local new_tris = table.create(self.Region.SubMesh:TriangleCount(), 0)
	self.ReinsertSubToBaseMapV = nil
	local bOK, SubToNewV = editor:ReinsertSubmesh(self.Region, new_tris, self.ReinsertDuplicateTriBehavior)
	self.ReinsertSubToBaseMapV = SubToNewV

	-- assert that new triangles are all valid (goes wrong sometimes??)
	Util.gDevAssert(IndexUtil.IndicesCheck(new_tris, function(tid)
		return BaseMesh:IsTriangle(tid)
	end))

	self.cur_base_tris = new_tris
	return bOK
end

-- C# QuickRemesh(mesh, tris, minEdgeLen, maxEdgeLen, smoothSpeed, rounds, target, flags)
function RegionRemesher.QuickRemesh(
	mesh,
	tris: any,
	minEdgeLen: number,
	maxEdgeLen: number,
	smoothSpeed: number,
	rounds: number,
	target: any?,
	flags: number?
)
	local f = flags or QuickRemeshFlags.PreventNormalFlips
	local remesh = RegionRemesher.new(mesh, tris)
	if target ~= nil then
		remesh:SetProjectionTarget(target)
	end
	remesh.MinEdgeLength = minEdgeLen
	remesh.MaxEdgeLength = maxEdgeLen
	remesh.SmoothSpeedT = smoothSpeed
	if bit32.band(f, QuickRemeshFlags.PreventNormalFlips) ~= 0 then
		remesh.PreventNormalFlips = true
	end
	for k = 1, rounds do
		remesh:BasicRemeshPass()
	end
	remesh:BackPropropagate()
	return remesh
end

-- C# QuickRemesh(mesh, tris, targetEdgeLen, smoothSpeed, rounds, target, flags)
function RegionRemesher.QuickRemeshTarget(
	mesh,
	tris: any,
	targetEdgeLen: number,
	smoothSpeed: number,
	rounds: number,
	target: any?,
	flags: number?
)
	local f = flags or QuickRemeshFlags.PreventNormalFlips
	local remesh = RegionRemesher.new(mesh, tris)
	if target ~= nil then
		remesh:SetProjectionTarget(target)
	end
	remesh:SetTargetEdgeLength(targetEdgeLen)
	remesh.SmoothSpeedT = smoothSpeed
	if bit32.band(f, QuickRemeshFlags.PreventNormalFlips) ~= 0 then
		remesh.PreventNormalFlips = true
	end
	for k = 1, rounds do
		remesh:BasicRemeshPass()
	end
	remesh:BackPropropagate()
	return remesh
end

return RegionRemesher
