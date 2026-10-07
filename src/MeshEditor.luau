-- Port of geometry3Sharp mesh/MeshEditor.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- This class implements various ways to do low-level edits to a mesh, in
-- a robust/reliable way.
--   - if operations fail in-progress, we try to back them out
--
-- Luau port notes (see PORTING.md):
--   * MeshEditor.new(mesh). Returned triangle lists (C# int[] / List<int>) are 1-based Luau arrays of ids.
--     Functions that return null in C# return nil.
--   * Inputs typed IEnumerable<int> accept: a Luau array of ids, a set {[id]=true}, an iterator function,
--     or a selection object (MeshFaceSelection etc, anything with :GetEnumerator()).
--     Arrays are treated like C# IList<int> (InvalidID entries skipped), everything else like IEnumerable<int>.
--   * Static/instance pairs with the same name dispatch on the first argument: when it is a DMesh3 instead of
--     a MeshEditor the static form is used. E.g. MeshEditor.RemoveTriangles(mesh, tris, bRemoveIsolated=true)
--     and editor:RemoveTriangles(tris, bRemoveIsolated); likewise RemoveUnusedVertices, RemoveSmallComponents,
--     AppendLine, AppendBox, AppendMesh.
--   * Renamed overloads:
--       RemoveTrianglesF(selectorF, bRemoveIsolatedVerts)        C# RemoveTriangles(Func<int,bool>, bool)
--       StitchUnorderedEdges(EdgePairs, group_id=-1, bAbortOnFailure=true) -> new_tris, stitch_incomplete
--         (EdgePairs is an array of Index2i arrays {eA, eB})
--       SeparateTriangles(triangles, bComputeEdgePairs) -> true, EdgePairs (array of {old_eid, new_eid} or nil)
--       DuplicateTriangles(triangles, MapV (IndexMap), group_id=-1) -> new_triangles, MapV
--       ReinsertSubmesh(sub, new_tris (array or nil), eDuplicateBehavior=AssertAbort) -> ok, SubToNewV
--         new_tris is filled in-place (new_tris[1..n]) like the C# `ref int[]`.
--       AppendMesh: editor:AppendMesh(appendMesh, appendGID=-1) -> true, mapV
--                   MeshEditor.AppendMesh(ToMesh, AppendMesh, SetGroupID=-1, Transform?) -> true   (static form)
--       AppendMeshMerge(appendMesh, mergeMapV, appendGID=-1) -> true, mapV     C# AppendMesh(IMesh, IndexMap, out int[], int)
--       MeshEditor.AppendMeshFull(ToMesh, AppendMesh, TriMap?, GroupPolicy?, SetConstantGroupID?, Transform?)
--         -> true, VertexMap, GroupMap                                          C# static AppendMesh(..., out, out, ...)
--       mapV / VertexMap / GroupMap / TriMap are 1-based arrays indexed by (id + 1), holding 0-based ids.
--       Transform (C# TransformWrapper) is an optional table { TransformPosition = fn?, TransformNormal = fn? }.
--     AppendBox: editor:AppendBox(frame, size (number or Vector3d), color?)
--                MeshEditor.AppendBox(mesh, pos, size, color?) / (mesh, pos, normal, size, color?) / (mesh, frame, size, color)
--     AppendLine: editor:AppendLine(seg, size, color?) / MeshEditor.AppendLine(mesh, seg, size)
--   * Colorf colors are Vector3d (rgb), as elsewhere in the port.
--   * Enums: MeshEditor.DuplicateTriBehavior, MeshEditor.AppendGroupPolicy.
--   * Not ported: AppendPathSolid (needs TubeGenerator / Polygon2d mesh generators, which are not part of this port).
--   * AppendBox requires Box3d and RemoveSmallComponents requires MeshMeasurements (loaded on first use).

local Util = require(script.Parent.Util)
local Vector3d = require(script.Parent.Vector3d)
local BitArray = require(script.Parent.BitArray)
local IndexMap = require(script.Parent.IndexMap)
local IndexUtil = require(script.Parent.IndexUtil)
local MeshResult = require(script.Parent.MeshResult)
local Frame3d = require(script.Parent.Frame3d)
local MeshIterators = require(script.Parent.MeshIterators)
local MathUtil = require(script.Parent.MathUtil)
local NewVertexInfo = require(script.Parent.NewVertexInfo)

local IndexFlagSet = IndexMap.IndexFlagSet
local InvalidID = -1
local NonManifoldID = -2

local MeshEditor = {}
MeshEditor.__index = MeshEditor
MeshEditor.ClassName = "MeshEditor"

-- in ReinsertSubmesh, a problem can arise where the mesh we are inserting has duplicate triangles of
-- the base mesh. This can lead to problematic behavior later. We can do various things, like delete
-- and replace that existing triangle, or just use it instead of adding a new one. Or fail, or ignore it.
-- This enum/argument controls the behavior.
-- However, fundamentally this kind of problem should be handled upstream!! For example by not trying
-- to remesh areas that contain nonmanifold geometry...
MeshEditor.DuplicateTriBehavior = table.freeze({
	AssertContinue = 0, -- check will not be done in Release!
	AssertAbort = 1,
	UseExisting = 2,
	Replace = 3,
})
local DuplicateTriBehavior = MeshEditor.DuplicateTriBehavior

MeshEditor.AppendGroupPolicy = table.freeze({
	-- directly copy GroupIDs from Source to Target
	CopyGroupIDs = 0,
	-- Allocate a new Target GroupID for each Source GroupID
	RemapGroupIDs = 1,
	-- map all GroupIDs on Source to a single GroupID on Target
	ConstantGroupID = 2,
})
local AppendGroupPolicy = MeshEditor.AppendGroupPolicy

-- iterate an IEnumerable<int>-like input: iterator function, selection object, array, or set
local function toItr(src: any): () -> number?
	if type(src) == "function" then
		return src
	end
	local mt = getmetatable(src)
	if mt ~= nil and type(mt) == "table" and mt.GetEnumerator ~= nil then
		return src:GetEnumerator()
	end
	if type(src[1]) == "number" or next(src) == nil then
		local i = 0
		return function()
			i += 1
			return src[i]
		end
	end
	local k = nil
	return function()
		k = next(src, k)
		return k
	end
end
MeshEditor.toItr = toItr

local function isArray(src: any): boolean
	return type(src) == "table" and getmetatable(src) == nil and (type(src[1]) == "number" or next(src) == nil)
end

local function asEditor(m)
	if getmetatable(m) == MeshEditor then
		return m
	end
	return MeshEditor.new(m)
end

function MeshEditor.new(mesh)
	local self = setmetatable({}, MeshEditor)
	self.Mesh = mesh
	return self
end

-- frames: array of Frame3d, spans: array of Interval1d
function MeshEditor:AddTriangleStrip(frames: { any }, spans: { any }, group_id: number?): { number }
	local N = #frames
	if N ~= #spans then
		error("MeshEditor.AddTriangleStrip: spans list is not the same size!")
	end
	local Mesh = self.Mesh
	local new_tris = table.create(2 * (N - 1), 0)

	local prev_a, prev_b = -1, -1
	local ti = 0
	for i = 1, N do
		local f = frames[i]
		local span = spans[i]

		local fy = f:Y()
		local va = f.Origin + span.a * fy
		local vb = f.Origin + span.b * fy

		-- [TODO] could compute normals here...

		local a = Mesh:AppendVertex(va)
		local b = Mesh:AppendVertex(vb)

		if prev_a ~= -1 then
			ti += 1
			new_tris[ti] = Mesh:AppendTriangle(prev_a, b, prev_b)
			ti += 1
			new_tris[ti] = Mesh:AppendTriangle(prev_a, a, b)
		end
		prev_a = a
		prev_b = b
	end

	return new_tris
end

function MeshEditor:AddTriangleFan_OrderedVertexLoop(center: number, vertex_loop: { number }, group_id: number?): { number }?
	local gid = group_id or -1
	local Mesh = self.Mesh
	local N = #vertex_loop
	local new_tris = table.create(N, 0)

	local i = 0
	while i < N do
		local a = vertex_loop[i + 1]
		local b = vertex_loop[((i + 1) % N) + 1]

		local new_tid = Mesh:AppendTriangle(center, b, a, gid)
		if new_tid < 0 then
			-- operation_failed: remove what we added so far
			if i > 0 then
				if self:remove_triangles(new_tris, i) == false then
					error("MeshEditor.AddTriangleFan_OrderedVertexLoop: failed to add fan, and also falied to back out changes.")
				end
			end
			return nil
		end

		new_tris[i + 1] = new_tid
		i += 1
	end

	return new_tris
end

function MeshEditor:AddTriangleFan_OrderedEdgeLoop(center: number, edge_loop: { number }, group_id: number?): { number }?
	local gid = group_id or -1
	local Mesh = self.Mesh
	local N = #edge_loop
	local new_tris = table.create(N, 0)

	local i = 0
	local failed = false
	while i < N do
		if Mesh:IsBoundaryEdge(edge_loop[i + 1]) == false then
			failed = true
			break
		end

		local ev = Mesh:GetOrientedBoundaryEdgeV(edge_loop[i + 1])
		local a, b = ev[1], ev[2]

		local new_tid = Mesh:AppendTriangle(center, b, a, gid)
		if new_tid < 0 then
			failed = true
			break
		end

		new_tris[i + 1] = new_tid
		i += 1
	end

	if not failed then
		return new_tris
	end

	-- operation_failed: remove what we added so far
	if i > 0 then
		if self:remove_triangles(new_tris, i - 1) == false then
			error("MeshEditor.AddTriangleFan_OrderedEdgeLoop: failed to add fan, and also failed to back out changes.")
		end
	end
	return nil
end

-- Trivial back-and-forth stitch between two vertex loops with same length.
-- Loops must have appropriate orientation (which is...??)
-- [TODO] check and fail on bad orientation
function MeshEditor:StitchLoop(vloop1: { number }, vloop2: { number }, group_id: number?): { number }?
	local gid = group_id or -1
	local Mesh = self.Mesh
	local N = #vloop1
	if N ~= #vloop2 then
		error("MeshEditor.StitchLoop: loops are not the same length!!")
	end

	local new_tris = table.create(N * 2, 0)

	local i = 0
	while i < N do
		local a = vloop1[i + 1]
		local b = vloop1[((i + 1) % N) + 1]
		local c = vloop2[i + 1]
		local d = vloop2[((i + 1) % N) + 1]

		local tid1 = Mesh:AppendTriangle(b, a, d, gid)
		local tid2 = Mesh:AppendTriangle(a, c, d, gid)
		new_tris[2 * i + 1] = tid1
		new_tris[2 * i + 2] = tid2

		if tid1 < 0 or tid2 < 0 then
			-- operation_failed: remove what we added so far
			if i > 0 then
				if self:remove_triangles(new_tris, 2 * i + 1) == false then
					error("MeshEditor.StitchLoop: failed to add all triangles, and also failed to back out changes.")
				end
			end
			return nil
		end
		i += 1
	end

	return new_tris
end

-- Trivial back-and-forth stitch between two vertex loops with same length.
-- If nearest vertices of input loops would not be matched, cycles loops so
-- that this is the case.
-- Loops must have appropriate orientation.
function MeshEditor:StitchVertexLoops_NearestV(loop0: { number }, loop1: { number }, group_id: number?): { number }?
	local Mesh = self.Mesh
	local N = #loop0
	local best_a, best_b = 0, 0
	local best_dist = MathUtil.MaxDouble
	for i = 0, N - 1 do
		local v0 = Mesh:GetVertex(loop0[i + 1])
		for j = 0, N - 1 do
			local dist_sqr = v0:DistanceSquared(Mesh:GetVertex(loop1[j + 1]))
			if dist_sqr < best_dist then
				best_dist = dist_sqr
				best_a, best_b = i, j
			end
		end
	end
	if best_a ~= best_b then
		local newLoop0 = table.create(N, 0)
		local newLoop1 = table.create(N, 0)
		for i = 0, N - 1 do
			newLoop0[i + 1] = loop0[((best_a + i) % N) + 1]
			newLoop1[i + 1] = loop1[((best_b + i) % N) + 1]
		end
		return self:StitchLoop(newLoop0, newLoop1, group_id)
	else
		return self:StitchLoop(loop0, loop1, group_id)
	end
end

-- Stitch two sets of boundary edges that are provided as unordered pairs of edges, by
-- adding triangulated quads between each edge pair.
-- If bAbortOnFailure==true and a failure is encountered during stitching, the triangles added up to that point are removed.
-- If bAbortOnFailure==false, failures are ignored and the returned triangle list may contain invalid values!
-- Returns new_tris (or nil), stitch_incomplete
function MeshEditor:StitchUnorderedEdges(EdgePairs: { { number } }, group_id: number?, bAbortOnFailure: boolean?): ({ number }?, boolean)
	local gid = group_id or -1
	local bAbort = bAbortOnFailure ~= false
	local Mesh = self.Mesh
	local N = #EdgePairs
	local new_tris = table.create(N * 2, bAbort and 0 or InvalidID)
	local stitch_incomplete = false

	local i = 0
	local failed = false
	while i < N do
		local edges = EdgePairs[i + 1]
		local skip = false

		-- look up and orient the first edge
		local edge_a = Mesh:GetEdge(edges[1])
		if edge_a[4] ~= InvalidID then
			if bAbort then
				failed = true
				break
			else
				stitch_incomplete = true
				skip = true
			end
		end
		local a, b, c, d
		if not skip then
			local edge_a_tri = Mesh:GetTriangle(edge_a[3])
			local _
			_, a, b = IndexUtil.orient_tri_edge(edge_a[1], edge_a[2], edge_a_tri)

			-- look up and orient the second edge
			local edge_b = Mesh:GetEdge(edges[2])
			if edge_b[4] ~= InvalidID then
				if bAbort then
					failed = true
					break
				else
					stitch_incomplete = true
					skip = true
				end
			end
			if not skip then
				local edge_b_tri = Mesh:GetTriangle(edge_b[3])
				_, c, d = IndexUtil.orient_tri_edge(edge_b[1], edge_b[2], edge_b_tri)

				-- swap second edge (right? should this be a parameter?)
				c, d = d, c

				local tid1 = Mesh:AppendTriangle(b, a, d, gid)
				local tid2 = Mesh:AppendTriangle(a, c, d, gid)

				if tid1 < 0 or tid2 < 0 then
					if bAbort then
						failed = true
						break
					else
						stitch_incomplete = true
						skip = true
					end
				end
				if not skip then
					new_tris[2 * i + 1] = tid1
					new_tris[2 * i + 2] = tid2
				end
			end
		end
		i += 1
	end

	if not failed then
		return new_tris, stitch_incomplete
	end

	-- operation_failed: remove what we added so far
	if i > 0 then
		if self:remove_triangles(new_tris, 2 * (i - 1)) == false then
			error("MeshEditor.StitchLoop: failed to add all triangles, and also failed to back out changes.")
		end
	end
	return nil, stitch_incomplete
end

-- Trivial back-and-forth stitch between two vertex spans with same length.
-- vertex ordering must reslut in appropriate orientation (which is...??)
-- [TODO] check and fail on bad orientation
function MeshEditor:StitchSpan(vspan1: { number }, vspan2: { number }, group_id: number?): { number }?
	local gid = group_id or -1
	local Mesh = self.Mesh
	local N = #vspan1
	if N ~= #vspan2 then
		error("MeshEditor.StitchSpan: spans are not the same length!!")
	end
	N -= 1

	local new_tris = table.create(N * 2, 0)

	local i = 0
	while i < N do
		local a = vspan1[i + 1]
		local b = vspan1[i + 2]
		local c = vspan2[i + 1]
		local d = vspan2[i + 2]

		local tid1 = Mesh:AppendTriangle(b, a, d, gid)
		local tid2 = Mesh:AppendTriangle(a, c, d, gid)

		if tid1 < 0 or tid2 < 0 then
			-- operation_failed: remove what we added so far
			if i > 0 then
				if self:remove_triangles(new_tris, 2 * (i - 1)) == false then
					error("MeshEditor.StitchLoop: failed to add all triangles, and also failed to back out changes.")
				end
			end
			return nil
		end

		new_tris[2 * i + 1] = tid1
		new_tris[2 * i + 2] = tid2
		i += 1
	end

	return new_tris
end

-- [TODO] cannot back-out this operation right now
--
-- Remove list of triangles. Values of triangles[] set to InvalidID are ignored.
--   editor:RemoveTriangles(triangles, bRemoveIsolatedVerts)
--   MeshEditor.RemoveTriangles(mesh, triangles, bRemoveIsolatedVerts = true)   (static)
-- triangles: array (C# IList: InvalidID entries skipped) or any IEnumerable (invalid tris -> returns false)
function MeshEditor.RemoveTriangles(selfOrMesh, triangles: any, bRemoveIsolatedVerts: boolean?): boolean
	local self = asEditor(selfOrMesh)
	local bRemoveIsolated = bRemoveIsolatedVerts ~= false
	local Mesh = self.Mesh
	local bAllOK = true
	if isArray(triangles) then
		for i = 1, #triangles do
			local tid = triangles[i]
			if tid == InvalidID then
				continue
			end
			local result = Mesh:RemoveTriangle(tid, bRemoveIsolated, false)
			if result ~= MeshResult.Ok then
				bAllOK = false
			end
		end
	else
		for tid in toItr(triangles) do
			if not Mesh:IsTriangle(tid) then
				bAllOK = false
				continue
			end
			local result = Mesh:RemoveTriangle(tid, bRemoveIsolated, false)
			if result ~= MeshResult.Ok then
				bAllOK = false
			end
		end
	end
	return bAllOK
end

-- [TODO] cannot back-out this operation right now
--
-- Remove all triangles identified by selectorF returning true    (C# RemoveTriangles(Func<int,bool>, bool))
function MeshEditor:RemoveTrianglesF(selectorF: (number) -> boolean, bRemoveIsolatedVerts: boolean): boolean
	local Mesh = self.Mesh
	local bAllOK = true
	local NT = Mesh:MaxTriangleID()
	for ti = 0, NT - 1 do
		if Mesh:IsTriangle(ti) == false or selectorF(ti) == false then
			continue
		end
		local result = Mesh:RemoveTriangle(ti, bRemoveIsolatedVerts, false)
		if result ~= MeshResult.Ok then
			bAllOK = false
		end
	end
	return bAllOK
end

-- Remove 'loner' triangles that have no connected neighbours.
function MeshEditor.RemoveIsolatedTriangles(mesh): boolean
	local editor = MeshEditor.new(mesh)
	return editor:RemoveTrianglesF(function(tid)
		local tnbrs = mesh:GetTriNeighbourTris(tid)
		return tnbrs[1] == InvalidID and tnbrs[2] == InvalidID and tnbrs[3] == InvalidID
	end, true)
end

-- Remove 'fin' triangles that have only one connected triangle.
-- Removing one fin can create another, by default will keep iterating
-- until all fins removed (in a not very efficient way!).
-- Pass bRepeatToConvergence=false to only do one pass.
-- [TODO] if we are repeating, construct face selection from nbrs of first list and iterate over that on future passes!
function MeshEditor.RemoveFinTriangles(mesh, removeF: ((any, number) -> boolean)?, bRepeatToConvergence: boolean?): number
	local bRepeat = bRepeatToConvergence ~= false
	local nRemoved = 0
	local to_remove = {}
	while true do
		for tid in mesh:TriangleIndices() do
			local nbrs = mesh:GetTriNeighbourTris(tid)
			local c = ((nbrs[1] ~= InvalidID) and 1 or 0) + ((nbrs[2] ~= InvalidID) and 1 or 0) + ((nbrs[3] ~= InvalidID) and 1 or 0)
			if c <= 1 then
				if removeF == nil or removeF(mesh, tid) == true then
					table.insert(to_remove, tid)
				end
			end
		end
		if #to_remove == 0 then
			return nRemoved
		end
		nRemoved += #to_remove
		MeshEditor.RemoveTriangles(mesh, to_remove, true)
		table.clear(to_remove)
		if not bRepeat then
			return nRemoved
		end
	end
end

-- Disconnect the given triangles from their neighbours, by duplicating "boundary" vertices, ie
-- vertices on edges for which one triangle is in-set and the other is not.
-- If bComputeEdgePairs is true, we return list of old/new edge pairs (useful for stitching)
-- [TODO] currently boundary-edge behaviour is to *not* duplicate boundary verts
-- Returns true, EdgePairs (array of {old_eid, new_eid}, or nil if not requested)
function MeshEditor:SeparateTriangles(triangles: any, bComputeEdgePairs: boolean): (boolean, { { number } }?)
	local Mesh = self.Mesh
	local tri_list = {}
	for tid in toItr(triangles) do
		table.insert(tri_list, tid)
	end
	local in_set = {}
	for _, tid in tri_list do
		in_set[tid] = true
	end
	local VertexMap = {}
	local EdgePairs = nil
	local edges = nil
	local OldEdgeVerts = nil
	if bComputeEdgePairs then
		EdgePairs = {}
		edges = {}
		OldEdgeVerts = {}
	end

	-- duplicate vertices on edges that are on boundary of triangles roi
	for _, tid in tri_list do
		local te = Mesh:GetTriEdges(tid)

		for j = 1, 3 do
			local et = Mesh:GetEdgeT(te[j])
			-- [TODO] what about behavior where we want to also duplicate boundary verts??
			if et[2] == InvalidID or (et[1] == tid and in_set[et[2]]) or (et[2] == tid and in_set[et[1]]) then
				te[j] = -1
			end
		end

		for j = 1, 3 do
			if te[j] == -1 then
				continue
			end
			local ev = Mesh:GetEdgeV(te[j])
			if VertexMap[ev[1]] == nil then
				VertexMap[ev[1]] = Mesh:AppendVertex(Mesh, ev[1])
			end
			if VertexMap[ev[2]] == nil then
				VertexMap[ev[2]] = Mesh:AppendVertex(Mesh, ev[2])
			end

			if bComputeEdgePairs and edges[te[j]] == nil then
				edges[te[j]] = true
				table.insert(OldEdgeVerts, ev)
				table.insert(EdgePairs, { te[j], -1 })
			end
		end
	end

	-- update triangles
	for _, tid in tri_list do
		local tv = Mesh:GetTriangle(tid)
		local tv_new = table.clone(tv)
		for j = 1, 3 do
			local newv = VertexMap[tv[j]]
			if newv ~= nil then
				tv_new[j] = newv
			end
		end
		if tv_new[1] ~= tv[1] or tv_new[2] ~= tv[2] or tv_new[3] ~= tv[3] then
			Mesh:SetTriangle(tid, tv_new)
		end
	end

	if bComputeEdgePairs then
		for k = 1, #EdgePairs do
			local old_ev = OldEdgeVerts[k]
			local new_a = VertexMap[old_ev[1]]
			local new_b = VertexMap[old_ev[2]]
			local new_eid = Mesh:FindEdge(new_a, new_b)
			Util.gDevAssert(new_eid ~= InvalidID)
			EdgePairs[k] = { EdgePairs[k][1], new_eid }
		end
	end

	return true, EdgePairs
end

-- Make a copy of provided triangles, with new vertices. You provide MapV because
-- you know if you are doing a small subset or a full-mesh-copy.
-- Returns new_triangles (array), MapV
function MeshEditor:DuplicateTriangles(triangles: any, MapV, group_id: number?): ({ number }, any)
	local gid = group_id or -1
	local Mesh = self.Mesh
	local new_triangles = {}
	for tid in toItr(triangles) do
		local tri = Mesh:GetTriangle(tid)
		for j = 1, 3 do
			local vid = tri[j]
			if MapV:Contains(vid) == false then
				local new_vid = Mesh:AppendVertex(Mesh, vid)
				MapV:Set(vid, new_vid)
				tri[j] = new_vid
			else
				tri[j] = MapV:Get(vid)
			end
		end
		local new_tid = Mesh:AppendTriangle(tri, gid)
		table.insert(new_triangles, new_tid)
	end
	return new_triangles, MapV
end

-- Reverse face orientation on a subset of triangles
function MeshEditor:ReverseTriangles(triangles: any, bFlipVtxNormals: boolean?)
	local Mesh = self.Mesh
	if bFlipVtxNormals == false then
		for tid in toItr(triangles) do
			Mesh:ReverseTriOrientation(tid)
		end
	else
		local donev = BitArray.new(Mesh:MaxVertexID())

		for tid in toItr(triangles) do
			Mesh:ReverseTriOrientation(tid)

			local tri = Mesh:GetTriangle(tid)
			for j = 1, 3 do
				local vid = tri[j]
				if donev:Get(vid) == false then
					Mesh:SetVertexNormal(vid, -Mesh:GetVertexNormal(vid))
					donev:Set(vid, true)
				end
			end
		end
	end
end

local function list_contains(list: { number }, v: number): boolean
	return table.find(list, v) ~= nil
end

-- separate triangle one-ring at vertex into connected components, and
-- then duplicate vertex once for each component
function MeshEditor:DisconnectBowtie(vid: number)
	local Mesh = self.Mesh
	local sets = {}
	for tid in Mesh:VtxTrianglesItr(vid) do
		local nbrs = Mesh:GetTriNeighbourTris(tid)
		local found = false
		for _, set in sets do
			if list_contains(set, nbrs[1]) or list_contains(set, nbrs[2]) or list_contains(set, nbrs[3]) then
				table.insert(set, tid)
				found = true
				break
			end
		end
		if found == false then
			table.insert(sets, { tid })
		end
	end
	if #sets == 1 then
		return -- not a bowtie!
	end
	-- bowtie_sorter: decreasing size
	table.sort(sets, function(l1, l2)
		return #l1 > #l2
	end)
	for k = 2, #sets do
		local copy_vid = Mesh:AppendVertex(Mesh, vid)
		local tris = sets[k]
		for _, tid in tris do
			local t = Mesh:GetTriangle(tid)
			if t[1] == vid then
				t[1] = copy_vid
			elseif t[2] == vid then
				t[2] = copy_vid
			else
				t[3] = copy_vid
			end
			Mesh:SetTriangle(tid, t, false)
		end
	end
end

-- Disconnect all bowtie vertices in mesh. Iterates because sometimes
-- disconnecting a bowtie creates new bowties (how??).
-- Returns number of remaining bowties after iterations.
function MeshEditor:DisconnectAllBowties(nMaxIters: number?): number
	local maxIters = nMaxIters or 10
	local function collect()
		local list = {}
		for vid in MeshIterators.BowtieVertices(self.Mesh) do
			table.insert(list, vid)
		end
		return list
	end
	local bowties = collect()
	local iter = 0
	while #bowties > 0 and iter < maxIters do
		iter += 1
		for _, vid in bowties do
			self:DisconnectBowtie(vid)
		end
		bowties = collect()
	end
	return #bowties
end

-- Assumption here is that Submesh has been modified, but boundary loop has
-- been preserved, and that old submesh has already been removed from this mesh.
-- So, we just have to append new vertices and then rewrite triangles
-- If new_tris is non-nil, we will return this info (filled in-place, 1-based).
-- new_tris should be sized to TriangleCount (ie it is not necessarily a map)
--
-- Returns true if submesh successfully inserted, false if any triangles failed
-- (which happens if triangle would result in non-manifold mesh). Second return value is SubToNewV (IndexMap).
function MeshEditor:ReinsertSubmesh(sub, new_tris: { number }?, eDuplicateBehavior: number?): (boolean, any)
	local eDup = eDuplicateBehavior or DuplicateTriBehavior.AssertAbort
	if sub.BaseBorderV == nil then
		error("MeshEditor.ReinsertSubmesh: Submesh does not have required boundary info. Call ComputeBoundaryInfo()!")
	end

	local Mesh = self.Mesh
	local submesh = sub.SubMesh
	local bAllOK = true

	local done_v = IndexFlagSet.WithEstimate(submesh:MaxVertexID(), submesh:TriangleCount() // 2)
	local SubToNewV = IndexMap.WithEstimate(submesh:MaxVertexID(), submesh:VertexCount())

	local nti = 0
	local NT = submesh:MaxTriangleID()
	for ti = 0, NT - 1 do
		if submesh:IsTriangle(ti) == false then
			continue
		end

		local sub_t = submesh:GetTriangle(ti)
		local gid = submesh:GetTriangleGroup(ti)

		local new_t = { 0, 0, 0 }
		for j = 1, 3 do
			local sub_v = sub_t[j]
			local new_v = -1
			if done_v:Get(sub_v) == false then
				-- first check if this is a boundary vtx on submesh and maps to a bdry vtx on base mesh
				if submesh:IsBoundaryVertex(sub_v) then
					local base_v = (sub_v < sub.SubToBaseV.size) and sub.SubToBaseV:get(sub_v) or -1
					if base_v >= 0 and Mesh:IsVertex(base_v) and sub.BaseBorderV[base_v] == true then
						-- [RMS] this should always be true, but assert in tests to find out
						Util.gDevAssert(Mesh:IsBoundaryVertex(base_v))
						if Mesh:IsBoundaryVertex(base_v) then
							new_v = base_v
						end
					end
				end

				-- if that didn't happen, append new vtx
				if new_v == -1 then
					new_v = Mesh:AppendVertex(submesh, sub_v)
				end

				SubToNewV:Set(sub_v, new_v)
				done_v:Set(sub_v, true)
			else
				new_v = SubToNewV:Get(sub_v)
			end

			new_t[j] = new_v
		end

		-- try to handle duplicate-tri case
		local skip = false
		if eDup == DuplicateTriBehavior.AssertContinue then
			Util.gDevAssert(Mesh:FindTriangle(new_t[1], new_t[2], new_t[3]) == InvalidID)
		else
			local existing_tid = Mesh:FindTriangle(new_t[1], new_t[2], new_t[3])
			if existing_tid ~= InvalidID then
				if eDup == DuplicateTriBehavior.AssertAbort then
					Util.gDevAssert(existing_tid == InvalidID)
					return false, SubToNewV
				elseif eDup == DuplicateTriBehavior.UseExisting then
					if new_tris ~= nil then
						nti += 1
						new_tris[nti] = existing_tid
					end
					skip = true
				elseif eDup == DuplicateTriBehavior.Replace then
					Mesh:RemoveTriangle(existing_tid, false)
				end
			end
		end

		if not skip then
			local new_tid = Mesh:AppendTriangle(new_t, gid)
			Util.gDevAssert(new_tid ~= InvalidID and new_tid ~= NonManifoldID)
			if not Mesh:IsTriangle(new_tid) then
				bAllOK = false
			end

			if new_tris ~= nil then
				nti += 1
				new_tris[nti] = new_tid
			end
		end
	end

	return bAllOK, SubToNewV
end

-- editor:AppendMesh(appendMesh, appendGID = -1) -> true, mapV
-- MeshEditor.AppendMesh(ToMesh, AppendMesh, SetGroupID = -1, Transform?) -> true     (C# static simplified version)
function MeshEditor.AppendMesh(selfOrMesh, appendMesh, appendGID: number?, Transform: any?): (boolean, { number }?)
	if getmetatable(selfOrMesh) ~= MeshEditor then
		-- static: Simplified version of full AppendMesh that either sets a constant GroupID, or remaps group IDs
		local SetGroupID = appendGID or -1
		local ok = MeshEditor.AppendMeshFull(
			selfOrMesh,
			appendMesh,
			nil,
			(SetGroupID >= 0) and AppendGroupPolicy.ConstantGroupID or AppendGroupPolicy.RemapGroupIDs,
			SetGroupID,
			Transform
		)
		return ok
	end
	local self = selfOrMesh
	local gidOverride = appendGID or -1
	local Mesh = self.Mesh
	local mapV = table.create(appendMesh:MaxVertexID(), 0)
	for vid in appendMesh:VertexIndices() do
		local vinfo = appendMesh:GetVertexAll(vid)
		local newvid = Mesh:AppendVertex(vinfo)
		mapV[vid + 1] = newvid
	end

	for tid in appendMesh:TriangleIndices() do
		local t = appendMesh:GetTriangle(tid)
		t[1] = mapV[t[1] + 1]
		t[2] = mapV[t[2] + 1]
		t[3] = mapV[t[3] + 1]
		local gid = appendMesh:GetTriangleGroup(tid)
		if gidOverride >= 0 then
			gid = gidOverride
		end
		Mesh:AppendTriangle(t, gid)
	end

	return true, mapV
end

function MeshEditor.Combine(...): any
	local DMesh3 = require(script.Parent.DMesh3)
	local m = DMesh3.new()
	local editor = MeshEditor.new(m)
	for _, mesh in { ... } do
		editor:AppendMesh(mesh, m:AllocateTriangleGroup())
	end
	return m
end

function MeshEditor.Append(appendTo, append)
	local editor = MeshEditor.new(appendTo)
	editor:AppendMesh(append, appendTo:AllocateTriangleGroup())
end

-- C# AppendMesh(IMesh appendMesh, IndexMap mergeMapV, out int[] mapV, int appendGID = -1)
function MeshEditor:AppendMeshMerge(appendMesh, mergeMapV, appendGID: number?): (boolean, { number })
	local gidOverride = appendGID or -1
	local Mesh = self.Mesh
	local mapV = table.create(appendMesh:MaxVertexID(), 0)
	for vid in appendMesh:VertexIndices() do
		if mergeMapV:Contains(vid) then
			mapV[vid + 1] = mergeMapV:Get(vid)
		else
			local vinfo = appendMesh:GetVertexAll(vid)
			local newvid = Mesh:AppendVertex(vinfo)
			mapV[vid + 1] = newvid
		end
	end

	for tid in appendMesh:TriangleIndices() do
		local t = appendMesh:GetTriangle(tid)
		t[1] = mapV[t[1] + 1]
		t[2] = mapV[t[2] + 1]
		t[3] = mapV[t[3] + 1]
		local gid = appendMesh:GetTriangleGroup(tid)
		if gidOverride >= 0 then
			gid = gidOverride
		end
		Mesh:AppendTriangle(t, gid)
	end

	return true, mapV
end

-- Adds geometry of AppendMesh to ToMesh. Vertex map is always returned.
-- GroupMap is returned non-nil if GroupPolicy is RemapGroupIDs.
-- TriMap must be allocated/provided by caller if desired (1-based array, TriMap[tid+1] = new tid)
-- If GroupPolicy is ConstantGroupID and SetConstantGroupID == -1, a new group will be allocated from ToMesh
-- (C# static AppendMesh(DMesh3, DMesh3, out int[], out int[]?, int[]?, AppendGroupPolicy, int, TransformWrapper))
function MeshEditor.AppendMeshFull(
	ToMesh,
	AppendMesh,
	TriMap: { number }?,
	GroupPolicy: number?,
	SetConstantGroupID: number?,
	Transform: any?
): (boolean, { number }, { number }?)
	local policy = GroupPolicy or AppendGroupPolicy.CopyGroupIDs
	local constGID = SetConstantGroupID or -1
	local TransformP = (Transform ~= nil and Transform.TransformPosition) or nil
	local TransformN = (Transform ~= nil and Transform.TransformNormal) or nil

	local VertexMap = table.create(AppendMesh:MaxVertexID(), 0)
	for vid in AppendMesh:VertexIndices() do
		local vinfo = AppendMesh:GetVertexAll(vid)
		if TransformP ~= nil then
			vinfo.v = TransformP(vinfo.v)
		end
		if vinfo.bHaveN and TransformN ~= nil then
			vinfo.n = TransformN(vinfo.n)
		end
		local new_vid = ToMesh:AppendVertex(vinfo)
		VertexMap[vid + 1] = new_vid
	end

	local GroupMap = nil
	if policy == AppendGroupPolicy.RemapGroupIDs then
		GroupMap = table.create(AppendMesh:MaxGroupID(), -1)
	end

	if constGID < 0 and policy == AppendGroupPolicy.ConstantGroupID then
		constGID = ToMesh:AllocateTriangleGroup()
	end

	for tid in AppendMesh:TriangleIndices() do
		local t = AppendMesh:GetTriangle(tid)
		t[1] = VertexMap[t[1] + 1]
		t[2] = VertexMap[t[2] + 1]
		t[3] = VertexMap[t[3] + 1]

		local gid = (policy == AppendGroupPolicy.ConstantGroupID) and constGID or AppendMesh:GetTriangleGroup(tid)
		if policy == AppendGroupPolicy.RemapGroupIDs then
			if GroupMap[gid + 1] == -1 then
				GroupMap[gid + 1] = ToMesh:AllocateTriangleGroup()
			end
			gid = GroupMap[gid + 1]
		end

		local new_tid = ToMesh:AppendTriangle(t, gid)
		if TriMap ~= nil and tid < #TriMap then
			TriMap[tid + 1] = new_tid
		end
	end

	return true, VertexMap, GroupMap
end

-- port of TrivialBox3Generator (NoSharedVertices = false, Clockwise = false) + MakeMesh, used by AppendBox
local function make_trivial_box_mesh(box)
	local DMesh3 = require(script.Parent.DMesh3)
	local gIndices = IndexUtil.gIndices
	local mesh = DMesh3.new()
	local center = box.Center
	for i = 0, 7 do
		local v = box:Corner(i)
		-- C#: (vertices[i] - Box.Center[i]).Normalized  (Vector3d indexer returns z for i >= 2)
		local ci = center:Get(i)
		local n = Vector3d.new(v.x - ci, v.y - ci, v.z - ci):Normalized()
		mesh:AppendVertex(NewVertexInfo.new(v, n))
	end
	for fi = 1, 6 do
		local f = gIndices.BoxFaces[fi]
		mesh:AppendTriangle(f[1], f[2], f[3])
		mesh:AppendTriangle(f[1], f[3], f[4])
	end
	return mesh
end

-- editor:AppendBox(frame, size (number or Vector3d), color?)
-- MeshEditor.AppendBox(mesh, pos, size, color?)          (static)
-- MeshEditor.AppendBox(mesh, pos, normal, size, color?)  (static)
-- MeshEditor.AppendBox(mesh, frame, size, color)         (static)
function MeshEditor.AppendBox(selfOrMesh, a: any, b: any, c: any, d: any)
	if getmetatable(selfOrMesh) ~= MeshEditor then
		local editor = MeshEditor.new(selfOrMesh)
		if getmetatable(a) == Frame3d then
			-- (mesh, frame, size, color)
			editor:AppendBox(a, b, c)
		elseif type(b) == "number" then
			-- (mesh, pos, size, color?)
			editor:AppendBox(Frame3d.new(a), b * Vector3d.One, c)
		else
			-- (mesh, pos, normal, size, color?)
			editor:AppendBox(Frame3d.FromZ(a, b), c * Vector3d.One, d)
		end
		return
	end
	local self = selfOrMesh
	local frame, size, color = a, b, c
	if type(size) == "number" then
		size = size * Vector3d.One
	end
	color = color or Vector3d.One -- Colorf.White
	local Box3d = require(script.Parent.Box3d)
	local mesh = make_trivial_box_mesh(Box3d.FromFrame(frame, size))
	if self.Mesh:HasVertexColors() then
		mesh:EnableVertexColors(color)
	end
	self:AppendMesh(mesh, self.Mesh:AllocateTriangleGroup())
end

-- editor:AppendLine(seg, size, color?) / MeshEditor.AppendLine(mesh, seg, size)
function MeshEditor.AppendLine(selfOrMesh, seg, size: number, color: any?)
	local self = asEditor(selfOrMesh)
	local f = Frame3d.new(seg.Center)
	f:AlignAxis(2, seg.Direction)
	self:AppendBox(f, Vector3d.new(size, size, seg.Extent), color)
end

-- Remove all bowtie vertices in mesh. Makes one pass unless
--   bRepeatUntilClean = true, in which case repeats until no more bowties found
-- Returns true if any vertices were removed
function MeshEditor:RemoveAllBowtieVertices(bRepeatUntilClean: boolean): boolean
	local Mesh = self.Mesh
	local nRemoved = 0

	while true do
		local bowties = {}
		for vID in Mesh:VertexIndices() do
			if Mesh:IsBowtieVertex(vID) then
				table.insert(bowties, vID)
			end
		end
		if #bowties == 0 then
			break
		end

		for _, vID in bowties do
			local result = Mesh:RemoveVertex(vID, true, false)
			Util.gDevAssert(result == MeshResult.Ok)
			nRemoved += 1
		end
		if bRepeatUntilClean == false then
			break
		end
	end
	return nRemoved > 0
end

-- Remove any unused vertices in mesh, ie vertices with no edges.
-- Returns number of removed vertices.
-- editor:RemoveUnusedVertices() / MeshEditor.RemoveUnusedVertices(mesh)
function MeshEditor.RemoveUnusedVertices(selfOrMesh): number
	local self = asEditor(selfOrMesh)
	local Mesh = self.Mesh
	local nRemoved = 0
	local NV = Mesh:MaxVertexID()
	for vid = 0, NV - 1 do
		if Mesh:IsVertex(vid) and Mesh:GetVtxEdgeCount(vid) == 0 then
			Mesh:RemoveVertex(vid)
			nRemoved += 1
		end
	end
	return nRemoved
end

-- Remove any connected components with volume < min_volume area < min_area
-- editor:RemoveSmallComponents(min_volume, min_area) / MeshEditor.RemoveSmallComponents(mesh, min_volume, min_area)
function MeshEditor.RemoveSmallComponents(selfOrMesh, min_volume: number, min_area: number): number
	local self = asEditor(selfOrMesh)
	local Mesh = self.Mesh
	local MeshConnectedComponents = require(script.Parent.MeshConnectedComponents)
	local MeshMeasurements = require(script.Parent.MeshMeasurements)
	local C = MeshConnectedComponents.new(Mesh)
	C:FindConnectedT()
	if C:Count() == 1 then
		return 0
	end
	local nRemoved = 0
	for _, comp in C.Components do
		local vol_area = MeshMeasurements.VolumeArea(Mesh, comp.Indices, function(vid)
			return Mesh:GetVertex(vid)
		end)
		if vol_area.x < min_volume or vol_area.y < min_area then
			MeshEditor.RemoveTriangles(Mesh, comp.Indices)
			nRemoved += 1
		end
	end
	return nRemoved
end

-- this is for backing out changes we have made... (tri_list is 1-based, count is number of entries)
function MeshEditor:remove_triangles(tri_list: { number }, count: number): boolean
	local Mesh = self.Mesh
	for i = 1, count do
		if Mesh:IsTriangle(tri_list[i]) == false then
			continue
		end
		local result = Mesh:RemoveTriangle(tri_list[i], false, false)
		if result ~= MeshResult.Ok then
			return false
		end
	end
	return true
end

return MeshEditor
