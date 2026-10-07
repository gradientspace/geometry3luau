-- Port of geometry3Sharp mesh/DSubmesh3.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Extracts a subset of triangles of a base mesh into a new DMesh3 (SubMesh), and keeps index maps
-- between the two.
--
-- Luau port notes (see PORTING.md):
--   * DSubmesh3.new(mesh, subTriangles?, nTriEstimate?)  -- with no triangles, call Compute() later
--     (C# DSubmesh3(mesh, int[]) / DSubmesh3(mesh, IEnumerable<int>, int) / DSubmesh3(mesh)).
--     subTriangles may be an array, a set {[tid]=true}, an iterator function, or a selection object.
--   * Compute(subTriangles, nTriEstimate?) and ComputeBoundaryInfo(triangles, tri_count_est?) merge the
--     int[] / IEnumerable overloads.
--   * Fields: BaseMesh, SubMesh, WantComponents (MeshComponents flags), ComputeTriMaps, OverrideGroupID,
--     BaseSubmeshV (IndexFlagSet), BaseToSubV (IndexMap), SubToBaseV (DVector, 0-based get/set),
--     BaseToSubT (IndexMap), SubToBaseT (DVector),
--     BaseBorderE / BaseBoundaryE / BaseBorderV: C# IndexHashSet -> plain sets {[id] = true}
--     (C# `set[id]` -> `set[id] == true`).
--   * MapVerticesToSubmesh(v) / MapVerticesToBaseMesh(v): with a 2-element array (C# Index2i overload) a new
--     mapped pair is returned. MapVerticesToSubmesh with any other array is the C# int[] overload: it maps the
--     array in place (and returns it). MapVerticesToSubmeshArray(vertices) is the unambiguous in-place form.
--     MapVertexPairToSubmesh / MapVertexPairToBaseMesh are the unambiguous Index2i forms.

local DMesh3 = require(script.Parent.DMesh3)
local DVector = require(script.Parent.DVector)
local IndexMap = require(script.Parent.IndexMap)

local IndexFlagSet = IndexMap.IndexFlagSet
local InvalidID = -1

local DSubmesh3 = {}
DSubmesh3.__index = DSubmesh3
DSubmesh3.ClassName = "DSubmesh3"

-- materialize an IEnumerable<int>-like input as a 1-based array
local function toArray(src: any): { number }
	if type(src) == "function" then
		local list = {}
		for v in src do
			table.insert(list, v)
		end
		return list
	end
	local mt = getmetatable(src)
	if mt ~= nil and type(mt) == "table" and mt.GetEnumerator ~= nil then
		local list = {}
		for v in src:GetEnumerator() do
			table.insert(list, v)
		end
		return list
	end
	if type(src[1]) == "number" or next(src) == nil then
		return src
	end
	local list = {}
	for k in src do
		table.insert(list, k)
	end
	return list
end
DSubmesh3.toArray = toArray

function DSubmesh3.new(mesh, subTriangles: any?, nTriEstimate: number?)
	local self = setmetatable({}, DSubmesh3)
	self.BaseMesh = mesh
	self.SubMesh = nil
	self.WantComponents = DMesh3.MeshComponents.All
	self.ComputeTriMaps = false
	self.OverrideGroupID = -1

	self.BaseSubmeshV = nil -- vertices in base mesh that are in submesh
	self.BaseToSubV = nil -- vertex index map from base to submesh
	self.SubToBaseV = nil -- vertex index map from submesh to base mesh
	self.BaseToSubT = nil -- triangle index map from base to submesh. Only computed if ComputeTriMaps = true.
	self.SubToBaseT = nil -- triangle index map from submesh to base mesh. Only computed if ComputeTriMaps = true.

	-- boundary info
	self.BaseBorderE = nil -- internal border edge indices on base mesh. Does not include mesh boundary edges.
	self.BaseBoundaryE = nil -- mesh-boundary edges on base mesh that are in submesh
	self.BaseBorderV = nil -- border vertex indices on base mesh (ie verts of BaseBorderE - does not include mesh boundary vertices)

	if subTriangles ~= nil then
		self:Compute(subTriangles, nTriEstimate)
	end
	return self
end

function DSubmesh3:Compute(subTriangles: any, nTriEstimate: number?)
	local tris = toArray(subTriangles)
	self:compute(tris, nTriEstimate or #tris)
end

function DSubmesh3:MapVertexToSubmesh(base_vID: number): number
	return self.BaseToSubV:Get(base_vID)
end

function DSubmesh3:MapVertexToBaseMesh(sub_vID: number): number
	if sub_vID < self.SubToBaseV:Length() then
		return self.SubToBaseV:get(sub_vID)
	end
	return InvalidID
end

-- C# MapVerticesToSubmesh(Index2i v)
function DSubmesh3:MapVertexPairToSubmesh(v: { number }): { number }
	return { self.BaseToSubV:Get(v[1]), self.BaseToSubV:Get(v[2]) }
end
-- C# MapVerticesToBaseMesh(Index2i v)
function DSubmesh3:MapVertexPairToBaseMesh(v: { number }): { number }
	return { self:MapVertexToBaseMesh(v[1]), self:MapVertexToBaseMesh(v[2]) }
end

-- C# MapVerticesToSubmesh(int[] vertices): in-place
function DSubmesh3:MapVerticesToSubmeshArray(vertices: { number }): { number }
	for i = 1, #vertices do
		vertices[i] = self.BaseToSubV:Get(vertices[i])
	end
	return vertices
end

-- C# MapVerticesToSubmesh(Index2i) (2-element array -> new pair) or MapVerticesToSubmesh(int[]) (in place)
function DSubmesh3:MapVerticesToSubmesh(v: { number }): { number }
	if #v == 2 then
		return self:MapVertexPairToSubmesh(v)
	end
	return self:MapVerticesToSubmeshArray(v)
end

-- C# MapVerticesToBaseMesh(Index2i)
function DSubmesh3:MapVerticesToBaseMesh(v: { number }): { number }
	return self:MapVertexPairToBaseMesh(v)
end

function DSubmesh3:MapEdgeToSubmesh(base_eid: number): number
	local base_ev = self.BaseMesh:GetEdgeV(base_eid)
	local sub_ev = self:MapVertexPairToSubmesh(base_ev)
	return self.SubMesh:FindEdge(sub_ev[1], sub_ev[2])
end

function DSubmesh3:MapEdgesToSubmesh(edges: { number })
	for i = 1, #edges do
		edges[i] = self:MapEdgeToSubmesh(edges[i])
	end
end

function DSubmesh3:MapEdgeToBaseMesh(sub_eid: number): number
	local sub_ev = self.SubMesh:GetEdgeV(sub_eid)
	local base_ev = self:MapVertexPairToBaseMesh(sub_ev)
	return self.BaseMesh:FindEdge(base_ev[1], base_ev[2])
end

function DSubmesh3:MapTriangleToSubmesh(base_tID: number): number
	if self.ComputeTriMaps == false then
		error("DSubmesh3.MapTriangleToSubmesh: must set ComputeTriMaps = true!")
	end
	return self.BaseToSubT:Get(base_tID)
end

function DSubmesh3:MapTriangleToBaseMesh(sub_tID: number): number
	if self.ComputeTriMaps == false then
		error("DSubmesh3.MapTriangleToBaseMesh: must set ComputeTriMaps = true!")
	end
	if sub_tID < self.SubToBaseT:Length() then
		return self.SubToBaseT:get(sub_tID)
	end
	return InvalidID
end

function DSubmesh3:MapTrianglesToSubmesh(triangles: { number })
	if self.ComputeTriMaps == false then
		error("DSubmesh3.MapTrianglesToSubmesh: must set ComputeTriMaps = true!")
	end
	for i = 1, #triangles do
		triangles[i] = self.BaseToSubT:Get(triangles[i])
	end
end

-- ComputeBoundaryInfo(int[] subTriangles) / ComputeBoundaryInfo(IEnumerable<int> triangles, int tri_count_est)
function DSubmesh3:ComputeBoundaryInfo(triangles: any, tri_count_est: number?)
	local tris = toArray(triangles)
	local BaseMesh = self.BaseMesh
	local est = tri_count_est or #tris

	-- set of base-mesh triangles that are in submesh
	local sub_tris = IndexFlagSet.WithEstimate(BaseMesh:MaxTriangleID(), est)
	for _, ti in tris do
		sub_tris:Set(ti, true)
	end

	local BaseBorderV = {}
	local BaseBorderE = {}
	local BaseBoundaryE = {}
	self.BaseBorderV = BaseBorderV
	self.BaseBorderE = BaseBorderE
	self.BaseBoundaryE = BaseBoundaryE

	-- Iterate through edges in submesh roi on base mesh. If
	-- one of the tris of the edge is not in submesh roi, then this
	-- is a boundary edge.
	--
	-- (edge iteration via triangle iteration processes each internal edge twice...)
	for _, ti in tris do
		local tedges = BaseMesh:GetTriEdges(ti)
		for j = 1, 3 do
			local eid = tedges[j]
			local et = BaseMesh:GetEdgeT(eid)
			if et[2] == InvalidID then -- this is a boundary edge
				BaseBoundaryE[eid] = true
			elseif sub_tris:Get(et[1]) ~= sub_tris:Get(et[2]) then -- this is a border edge
				BaseBorderE[eid] = true
				local ve = BaseMesh:GetEdgeV(eid)
				BaseBorderV[ve[1]] = true
				BaseBorderV[ve[2]] = true
			end
		end
	end
end

-- [RMS] estimate can be zero
function DSubmesh3:compute(triangles: { number }, tri_count_est: number)
	local BaseMesh = self.BaseMesh
	local est_verts = tri_count_est // 2

	local SubMesh = DMesh3.FromComponents(bit32.band(BaseMesh:Components(), self.WantComponents))
	self.SubMesh = SubMesh

	local BaseSubmeshV = IndexFlagSet.WithEstimate(BaseMesh:MaxVertexID(), est_verts)
	local BaseToSubV = IndexMap.WithEstimate(BaseMesh:MaxVertexID(), est_verts)
	local SubToBaseV = DVector.new()
	self.BaseSubmeshV = BaseSubmeshV
	self.BaseToSubV = BaseToSubV
	self.SubToBaseV = SubToBaseV

	local ComputeTriMaps = self.ComputeTriMaps
	if ComputeTriMaps then
		self.BaseToSubT = IndexMap.WithEstimate(BaseMesh:MaxTriangleID(), tri_count_est)
		self.SubToBaseT = DVector.new()
	end

	for _, tid in triangles do
		if not BaseMesh:IsTriangle(tid) then
			error("DSubmesh3.compute: triangle " .. tostring(tid) .. " does not exist in BaseMesh!")
		end
		local base_t = BaseMesh:GetTriangle(tid)
		local new_t = { 0, 0, 0 }
		local gid = BaseMesh:GetTriangleGroup(tid)

		for j = 1, 3 do
			local base_v = base_t[j]
			local sub_v = -1
			if BaseSubmeshV:Get(base_v) == false then
				sub_v = SubMesh:AppendVertex(BaseMesh, base_v)
				BaseSubmeshV:Set(base_v, true)
				BaseToSubV:Set(base_v, sub_v)
				SubToBaseV:insert(base_v, sub_v)
			else
				sub_v = BaseToSubV:Get(base_v)
			end
			new_t[j] = sub_v
		end

		if self.OverrideGroupID >= 0 then
			gid = self.OverrideGroupID
		end
		local sub_tid = SubMesh:AppendTriangle(new_t, gid)

		if ComputeTriMaps then
			self.BaseToSubT:Set(tid, sub_tid)
			self.SubToBaseT:insert(tid, sub_tid)
		end
	end
end

function DSubmesh3.QuickSubmesh(mesh, triangles: any)
	local submesh = DSubmesh3.new(mesh, toArray(triangles))
	return submesh.SubMesh
end

return DSubmesh3
