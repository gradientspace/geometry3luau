-- Port of geometry3Sharp mesh_ops/RegionOperator.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- This class automatically extracts a submesh from a mesh, and can re-insert it after you have
-- edited it, as long as you have not messed up the boundary
--
-- [TODO] Nearly all the code here is duplicated from RegionRemesher. Maybe this could be a base class for that?
-- [TODO] ReinsertSubToBaseMapT is not returned by the MeshEditor.ReinsertSubmesh, instead we are
--   trying to guess it here, by making some assumptions about what happens. It works for now, but
--   it would better if MeshEditor returned this information.
--
-- Luau port notes:
--   * RegionOperator.new(mesh, regionTris, submeshConfigF?) -- regionTris: array, set, iterator or selection;
--     submeshConfigF(DSubmesh3) is called before the submesh is computed.
--   * Properties -> methods: CurrentBaseTriangles() (1-based array of base tids).
--   * CurrentBaseInteriorVertices() returns a set {[vid] = true} (C# HashSet<int>).
--   * BackPropropagate / BackPropropagateVertices keep the C# (misspelled) names.

local DSubmesh3 = require(script.Parent.DSubmesh3)
local MeshEditor = require(script.Parent.MeshEditor)
local IndexMap = require(script.Parent.IndexMap)
local IndexUtil = require(script.Parent.IndexUtil)
local Util = require(script.Parent.Util)
local FaceGroupUtil = require(script.Parent.FaceGroupUtil)

local InvalidID = -1

local RegionOperator = {}
RegionOperator.__index = RegionOperator
RegionOperator.ClassName = "RegionOperator"

function RegionOperator.new(mesh, regionTris: any, submeshConfigF: ((any) -> ())?)
	local self = setmetatable({}, RegionOperator)
	local tris = DSubmesh3.toArray(regionTris)
	self.BaseMesh = mesh
	self.Region = DSubmesh3.new(mesh)
	-- this is only valid after BackPropagate() call!! maps submeshverts to base mesh
	self.ReinsertSubToBaseMapV = nil
	-- [RMS] computation of this is kind of a hack right now...
	self.ReinsertSubToBaseMapT = nil
	-- handle a tricky problem...see comments for DuplicateTriBehavior enum
	self.ReinsertDuplicateTriBehavior = MeshEditor.DuplicateTriBehavior.AssertContinue

	if submeshConfigF ~= nil then
		submeshConfigF(self.Region)
	end
	self.Region:Compute(tris)
	self.Region:ComputeBoundaryInfo(tris, #tris)

	self.cur_base_tris = table.clone(tris)
	return self
end

-- list of sub-region triangles. This is either the input regionTris,
-- or the submesh triangles after they are re-inserted.
function RegionOperator:CurrentBaseTriangles(): { number }
	return self.cur_base_tris
end

-- find base-mesh interior vertices of region (ie does not include region boundary vertices)
function RegionOperator:CurrentBaseInteriorVertices(): { [number]: boolean }
	local verts = {}
	local borderv = self.Region.BaseBorderV
	local BaseMesh = self.BaseMesh
	for _, tid in self.cur_base_tris do
		local tv = BaseMesh:GetTriangle(tid)
		if borderv[tv[1]] ~= true then
			verts[tv[1]] = true
		end
		if borderv[tv[2]] ~= true then
			verts[tv[2]] = true
		end
		if borderv[tv[3]] ~= true then
			verts[tv[3]] = true
		end
	end
	return verts
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
function RegionOperator:RepairPossibleNonManifoldEdges()
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
function RegionOperator:SetSubmeshGroupID(gid: number)
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
function RegionOperator:BackPropropagate(bAllowSubmeshRepairs: boolean?): boolean
	if bAllowSubmeshRepairs ~= false then
		self:RepairPossibleNonManifoldEdges()
	end
	local Region = self.Region
	local BaseMesh = self.BaseMesh

	-- remove existing submesh triangles
	local editor = MeshEditor.new(BaseMesh)
	editor:RemoveTriangles(self.cur_base_tris, true)

	-- insert new submesh
	local new_tris = table.create(Region.SubMesh:TriangleCount(), 0)
	self.ReinsertSubToBaseMapV = nil
	local bOK, SubToNewV = editor:ReinsertSubmesh(Region, new_tris, self.ReinsertDuplicateTriBehavior)
	self.ReinsertSubToBaseMapV = SubToNewV

	-- reconstruct this...hacky?
	local NT = Region.SubMesh:MaxTriangleID()
	local mapT = IndexMap.new(false, NT)
	self.ReinsertSubToBaseMapT = mapT
	local nti = 0
	for ti = 0, NT - 1 do
		if Region.SubMesh:IsTriangle(ti) == false then
			continue
		end
		nti += 1
		mapT:Set(ti, new_tris[nti])
	end

	-- assert that new triangles are all valid (goes wrong sometimes??)
	Util.gDevAssert(IndexUtil.IndicesCheck(new_tris, function(tid)
		return BaseMesh:IsTriangle(tid)
	end))

	self.cur_base_tris = new_tris
	return bOK
end

-- transfer vertex positions in submesh back to base mesh
function RegionOperator:BackPropropagateVertices(bRecomputeBoundaryNormals: boolean?): boolean
	local Region = self.Region
	local SubMesh, BaseMesh = Region.SubMesh, Region.BaseMesh
	local bNormals = SubMesh:HasVertexNormals() and BaseMesh:HasVertexNormals()
	for subvid in SubMesh:VertexIndices() do
		local basevid = Region.SubToBaseV:get(subvid)
		local v = SubMesh:GetVertex(subvid)
		BaseMesh:SetVertex(basevid, v)
		if bNormals then
			BaseMesh:SetVertexNormal(basevid, SubMesh:GetVertexNormal(subvid))
		end
	end

	if bRecomputeBoundaryNormals == true then
		local MeshNormals = require(script.Parent.MeshNormals)
		for basevid in Region.BaseBorderV do
			local n = MeshNormals.QuickComputeVertex(BaseMesh, basevid)
			BaseMesh:SetVertexNormal(basevid, n)
		end
	end

	return true
end

return RegionOperator
