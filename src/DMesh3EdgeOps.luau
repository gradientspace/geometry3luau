--!native
-- Port of geometry3Sharp mesh/DMesh3_edge_operators.cs (partial class DMesh3)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Returns a function that installs the edge operators into the DMesh3 class table.
--
-- C# `MeshResult Op(..., out XInfo info)` -> `local result, info = mesh:Op(...)`; info structs are tables
-- with the C# field names (Index2i fields are arrays):
--   EdgeSplitInfo   { bIsBoundary, vNew, split_t, eOrigT0, eOrigT1, eOrigAB, eNewBN, eNewCN, eNewDN, eNewT2, eNewT3 }
--   EdgeFlipInfo    { eID, v0, v1, ov0, ov1, t0, t1 }
--   EdgeCollapseInfo{ vKept, vRemoved, bIsBoundary, eCollapsed, tRemoved0, tRemoved1, eRemoved0, eRemoved1, eKept0, eKept1, collapse_t }
--   MergeEdgesInfo  { eKept, eRemoved, vKept, vRemoved, eRemovedExtra, eKeptExtra }  (Vector2i fields -> 2-element arrays)
--   PokeTriangleInfo{ orig_t0, orig_tri, new_vid, new_t1, new_t2, new_edges, new_vid_barycoords }
-- Overloads: SplitEdge(eab, split_t?) / SplitEdgeVerts(vA, vB); FlipEdge(eab) / FlipEdgeVerts(vA, vB);
--            PokeTriangle(tid, baryCoordinates?)

local Util = require(script.Parent.Util)
local Vector3d = require(script.Parent.Vector3d)
local Vector2d = require(script.Parent.Vector2d)
local IndexUtil = require(script.Parent.IndexUtil)
local MeshResult = require(script.Parent.MeshResult)

local InvalidID = -1

return function(DMesh3)
	-- edits

	function DMesh3:ReverseTriOrientation(tID: number): number
		if not self:IsTriangle(tID) then
			return MeshResult.Failed_NotATriangle
		end
		self:internal_reverse_tri_orientation(tID)
		self:updateTimeStamp(true)
		return MeshResult.Ok
	end

	function DMesh3:internal_reverse_tri_orientation(tID: number)
		local t = self:GetTriangle(tID)
		self:set_triangle(tID, t[2], t[1], t[3])
		local te = self:GetTriEdges(tID)
		self:set_triangle_edges(tID, te[1], te[3], te[2])

		if self:HasAttributes() then
			self.attributes:OnReverseTriOrientation(tID)
		end
	end

	function DMesh3:ReverseOrientation(bFlipNormals: boolean?)
		for tid in self:TriangleIndices() do
			self:internal_reverse_tri_orientation(tid)
		end
		if bFlipNormals ~= false and self:HasVertexNormals() then
			local n = self.normals.data
			for vid in self:VertexIndices() do
				local i = 3 * vid
				n[i + 1], n[i + 2], n[i + 3] = -n[i + 1], -n[i + 2], -n[i + 3]
			end
		end
		self:updateTimeStamp(true)
	end

	-- Remove vertex vID, and all connected triangles if bRemoveAllTriangles = true
	-- (if false, them throws exception if there are still any triangles!)
	-- if bPreserveManifold, checks that we will not create a bowtie vertex first
	function DMesh3:RemoveVertex(vID: number, bRemoveAllTriangles: boolean?, bPreserveManifold: boolean?): number
		if not self.vertices_refcount:isValid(vID) then
			return MeshResult.Failed_NotAVertex
		end

		if bRemoveAllTriangles ~= false then
			-- if any one-ring vtx is a boundary vtx and one of its outer-ring edges is an
			-- interior edge then we will create a bowtie if we remove that triangle
			if bPreserveManifold then
				for tid in self:VtxTrianglesItr(vID) do
					local tri = self:GetTriangle(tid)
					local j = IndexUtil.find_tri_index(vID, tri)
					local oa, ob = tri[((j + 1) % 3) + 1], tri[((j + 2) % 3) + 1]
					local eid = self:find_edge(oa, ob)
					if not self:IsBoundaryEdge(eid) then
						if self:IsBoundaryVertex(oa) or self:IsBoundaryVertex(ob) then
							return MeshResult.Failed_WouldCreateBowtie
						end
					end
				end
			end

			local tris = {}
			self:GetVtxTriangles(vID, tris, true)
			for _, tID in tris do
				local result = self:RemoveTriangle(tID, false, bPreserveManifold)
				if result ~= MeshResult.Ok then
					return result
				end
			end
		end

		if self.vertices_refcount:refCount(vID) ~= 1 then
			error("DMesh3.RemoveVertex: vertex is still referenced")
		end

		self.vertices_refcount:decrement(vID)
		Util.gDevAssert(self.vertices_refcount:isValid(vID) == false)
		self.vertex_edges:Clear(vID)

		self:updateTimeStamp(true)
		return MeshResult.Ok
	end

	-- Remove a tID from the mesh. Also removes any unreferenced edges after tri is removed.
	-- If bRemoveIsolatedVertices is false, then if you remove all tris from a vert, that vert is also removed.
	-- If bPreserveManifold, we check that you will not create a bowtie vertex (and return false).
	--   If this check is not done, you have to make sure you don't create a bowtie, because other
	--   code assumes we don't have bowties, and will not handle it properly
	function DMesh3:RemoveTriangle(tID: number, bRemoveIsolatedVertices: boolean?, bPreserveManifold: boolean?): number
		if not self.triangles_refcount:isValid(tID) then
			Util.gDevAssert(false)
			return MeshResult.Failed_NotATriangle
		end

		local tv = self:GetTriangle(tID)
		local te = self:GetTriEdges(tID)

		-- if any tri vtx is a boundary vtx connected to two interior edges, then
		-- we cannot remove this triangle because it would create a bowtie vertex!
		-- (that vtx already has 2 boundary edges, and we would add two more)
		if bPreserveManifold then
			for j = 0, 2 do
				if self:IsBoundaryVertex(tv[j + 1]) then
					if not self:IsBoundaryEdge(te[j + 1]) and not self:IsBoundaryEdge(te[((j + 2) % 3) + 1]) then
						return MeshResult.Failed_WouldCreateBowtie
					end
				end
			end
		end

		-- Remove triangle from its edges. if edge has no triangles left,
		-- then it is removed.
		local ed = self.edges.data
		for j = 1, 3 do
			local eid = te[j]
			self:replace_edge_triangle(eid, tID, InvalidID)
			if ed[4 * eid + 3] == InvalidID then
				local a = ed[4 * eid + 1]
				self.vertex_edges:Remove(a, eid)

				local b = ed[4 * eid + 2]
				self.vertex_edges:Remove(b, eid)

				self.edges_refcount:decrement(eid)
			end
		end

		-- free this triangle
		self.triangles_refcount:decrement(tID)
		Util.gDevAssert(self.triangles_refcount:isValid(tID) == false)

		-- Decrement vertex refcounts. If any hit 1 and we got remove-isolated flag,
		-- we need to remove that vertex
		local bRemoveIso = bRemoveIsolatedVertices ~= false
		for j = 1, 3 do
			local vid = tv[j]
			self.vertices_refcount:decrement(vid)
			if bRemoveIso and self.vertices_refcount:refCount(vid) == 1 then
				self.vertices_refcount:decrement(vid)
				Util.gDevAssert(self.vertices_refcount:isValid(vid) == false)
				self.vertex_edges:Clear(vid)
			end
		end

		if self:HasAttributes() then
			self.attributes:OnRemoveTriangle(tID)
		end

		self:updateTimeStamp(true)
		return MeshResult.Ok
	end

	function DMesh3:SetTriangle(tID: number, newv: { number }, bRemoveIsolatedVertices: boolean?): number
		local tv = self:GetTriangle(tID)
		local te = self:GetTriEdges(tID)
		if tv[1] == newv[1] and tv[2] == newv[2] then
			te[1] = -1
		end
		if tv[2] == newv[2] and tv[3] == newv[3] then
			te[2] = -1
		end
		if tv[3] == newv[3] and tv[1] == newv[1] then
			te[3] = -1
		end

		if not self.triangles_refcount:isValid(tID) then
			Util.gDevAssert(false)
			return MeshResult.Failed_NotATriangle
		end
		if not self:IsVertex(newv[1]) or not self:IsVertex(newv[2]) or not self:IsVertex(newv[3]) then
			Util.gDevAssert(false)
			return MeshResult.Failed_NotAVertex
		end
		if newv[1] == newv[2] or newv[1] == newv[3] or newv[2] == newv[3] then
			Util.gDevAssert(false)
			return MeshResult.Failed_BrokenTopology
		end
		-- look up edges. if any already have two triangles, this would
		-- create non-manifold geometry and so we do not allow it
		local e0 = self:find_edge(newv[1], newv[2])
		local e1 = self:find_edge(newv[2], newv[3])
		local e2 = self:find_edge(newv[3], newv[1])
		if
			(te[1] ~= -1 and e0 ~= InvalidID and not self:IsBoundaryEdge(e0))
			or (te[2] ~= -1 and e1 ~= InvalidID and not self:IsBoundaryEdge(e1))
			or (te[3] ~= -1 and e2 ~= InvalidID and not self:IsBoundaryEdge(e2))
		then
			return MeshResult.Failed_BrokenTopology
		end

		-- [TODO] check that we are not going to create invalid stuff...

		-- Remove triangle from its edges. if edge has no triangles left, then it is removed.
		local ed = self.edges.data
		for j = 1, 3 do
			local eid = te[j]
			if eid ~= -1 then -- we don't need to modify this edge
				self:replace_edge_triangle(eid, tID, InvalidID)
				if ed[4 * eid + 3] == InvalidID then
					local a = ed[4 * eid + 1]
					self.vertex_edges:Remove(a, eid)

					local b = ed[4 * eid + 2]
					self.vertex_edges:Remove(b, eid)

					self.edges_refcount:decrement(eid)
				end
			end
		end

		-- Decrement vertex refcounts. If any hit 1 and we got remove-isolated flag,
		-- we need to remove that vertex
		local bRemoveIso = bRemoveIsolatedVertices ~= false
		for j = 1, 3 do
			local vid = tv[j]
			if vid ~= newv[j] then -- we don't need to modify this vertex
				self.vertices_refcount:decrement(vid)
				if bRemoveIso and self.vertices_refcount:refCount(vid) == 1 then
					self.vertices_refcount:decrement(vid)
					Util.gDevAssert(self.vertices_refcount:isValid(vid) == false)
					self.vertex_edges:Clear(vid)
				end
			end
		end

		-- ok now re-insert with new vertices
		local t = self.triangles.data
		local i = 3 * tID
		for j = 1, 3 do
			if newv[j] ~= tv[j] then
				t[i + j] = newv[j]
				self.vertices_refcount:increment(newv[j])
			end
		end

		if te[1] ~= -1 then
			self:add_tri_edge(tID, newv[1], newv[2], 0, e0)
		end
		if te[2] ~= -1 then
			self:add_tri_edge(tID, newv[2], newv[3], 1, e1)
		end
		if te[3] ~= -1 then
			self:add_tri_edge(tID, newv[3], newv[1], 2, e2)
		end

		self:updateTimeStamp(true)
		return MeshResult.Ok
	end

	---------------------------------------------------------------------------------------------------
	-- SplitEdge

	function DMesh3:SplitEdgeVerts(vA: number, vB: number)
		local eid = self:find_edge(vA, vB)
		if eid == InvalidID then
			return MeshResult.Failed_NotAnEdge, {}
		end
		return self:SplitEdge(eid)
	end

	-- Split edge eab.
	-- split_t defines position along edge, and is assumed to be based on order of vertices returned by GetEdgeV()
	function DMesh3:SplitEdge(eab: number, split_t: number?)
		local splitT = split_t or 0.5
		local split = {}
		if not self:IsEdge(eab) then
			return MeshResult.Failed_NotAnEdge, split
		end

		-- look up primary edge & triangle
		local ed = self.edges.data
		local eab_i = 4 * eab
		local a, b = ed[eab_i + 1], ed[eab_i + 2]
		local t0 = ed[eab_i + 3]
		if t0 == InvalidID then
			return MeshResult.Failed_BrokenTopology, split
		end
		local T0tv_array = self:GetTriangle(t0)
		local c
		c, a, b = IndexUtil.orient_tri_edge_and_find_other_vtx(a, b, T0tv_array)
		if self.vertices_refcount:rawRefCount(c) > 32764 then
			return MeshResult.Failed_HitValenceLimit, split
		end
		if a ~= ed[eab_i + 1] then
			splitT = 1.0 - splitT -- if we flipped a/b order we need to reverse t
		end

		-- quite a bit of code is duplicated between boundary and non-boundary case, but it
		--  is too hard to follow later if we factor it out...
		if self:IsBoundaryEdge(eab) then
			-- create new vertex
			local vNew = Vector3d.Lerp(self:GetVertex(a), self:GetVertex(b), splitT)
			local f = self:AppendVertex(vNew)
			if self:HasVertexNormals() then
				self:SetVertexNormal(f, Vector3d.Lerp(self:GetVertexNormal(a), self:GetVertexNormal(b), splitT):Normalized())
			end
			if self:HasVertexUVs() then
				self:SetVertexUV(f, Vector2d.Lerp(self:GetVertexUV(a), self:GetVertexUV(b), splitT))
			end

			-- look up edge bc, which needs to be modified
			local T0te = self:GetTriEdges(t0)
			local ebc = T0te[IndexUtil.find_edge_index_in_tri(b, c, T0tv_array) + 1]

			-- rewrite existing triangle
			self:replace_tri_vertex(t0, b, f)

			-- add new second triangle
			local t2 = self:add_triangle_only(f, b, c, InvalidID, InvalidID, InvalidID)
			if self.triangle_groups ~= nil then
				self.triangle_groups:insert(self.triangle_groups.data[t0 + 1], t2)
			end

			-- rewrite edge bc, create edge af
			self:replace_edge_triangle(ebc, t0, t2)
			local eaf = eab
			self:replace_edge_vertex(eaf, b, f)
			self.vertex_edges:Remove(b, eab)
			self.vertex_edges:Insert(f, eaf)

			-- create new edges fb and fc
			local efb = self:add_edge(f, b, t2)
			local efc = self:add_edge(f, c, t0, t2)

			-- update triangle edge-nbrs
			self:replace_triangle_edge(t0, ebc, efc)
			self:set_triangle_edges(t2, efb, ebc, efc)

			-- update vertex refcounts
			self.vertices_refcount:increment(c)
			self.vertices_refcount:increment(f, 2)

			split.bIsBoundary = true
			split.vNew = f
			split.split_t = splitT
			split.eNewBN = efb
			split.eNewCN = efc
			split.eNewDN = InvalidID
			split.eNewT2 = t2
			split.eNewT3 = InvalidID
			split.eOrigT0 = t0
			split.eOrigT1 = InvalidID
			split.eOrigAB = { a, b }

			if self.attributes ~= nil then
				self.attributes:OnSplitEdge(split)
			end

			self:updateTimeStamp(true)
			return MeshResult.Ok, split
		else -- interior triangle branch
			-- look up other triangle
			local t1 = ed[eab_i + 4]
			local T1tv_array = self:GetTriangle(t1)
			local d = IndexUtil.find_tri_other_vtx(a, b, T1tv_array)
			if self.vertices_refcount:rawRefCount(d) > 32764 then
				return MeshResult.Failed_HitValenceLimit, split
			end

			-- create new vertex
			local vNew = Vector3d.Lerp(self:GetVertex(a), self:GetVertex(b), splitT)
			local f = self:AppendVertex(vNew)
			if self:HasVertexNormals() then
				self:SetVertexNormal(f, Vector3d.Lerp(self:GetVertexNormal(a), self:GetVertexNormal(b), splitT):Normalized())
			end
			if self:HasVertexUVs() then
				self:SetVertexUV(f, Vector2d.Lerp(self:GetVertexUV(a), self:GetVertexUV(b), splitT))
			end

			-- look up edges that we are going to need to update
			-- [TODO OPT] could use ordering to reduce # of compares here
			local T0te = self:GetTriEdges(t0)
			local ebc = T0te[IndexUtil.find_edge_index_in_tri(b, c, T0tv_array) + 1]
			local T1te = self:GetTriEdges(t1)
			local edb = T1te[IndexUtil.find_edge_index_in_tri(d, b, T1tv_array) + 1]

			-- rewrite existing triangles
			self:replace_tri_vertex(t0, b, f)
			self:replace_tri_vertex(t1, b, f)

			-- add two new triangles to close holes we just created
			local t2 = self:add_triangle_only(f, b, c, InvalidID, InvalidID, InvalidID)
			local t3 = self:add_triangle_only(f, d, b, InvalidID, InvalidID, InvalidID)
			if self.triangle_groups ~= nil then
				self.triangle_groups:insert(self.triangle_groups.data[t0 + 1], t2)
				self.triangle_groups:insert(self.triangle_groups.data[t1 + 1], t3)
			end

			-- update the edges we found above, to point to new triangles
			self:replace_edge_triangle(ebc, t0, t2)
			self:replace_edge_triangle(edb, t1, t3)

			-- edge eab became eaf
			local eaf = eab --Edge * eAF = eAB;
			self:replace_edge_vertex(eaf, b, f)

			-- update a/b/f vertex-edges
			self.vertex_edges:Remove(b, eab)
			self.vertex_edges:Insert(f, eaf)

			-- create new edges connected to f  (also updates vertex-edges)
			local efb = self:add_edge(f, b, t2, t3)
			local efc = self:add_edge(f, c, t0, t2)
			local edf = self:add_edge(d, f, t1, t3)

			-- update triangle edge-nbrs
			self:replace_triangle_edge(t0, ebc, efc)
			self:replace_triangle_edge(t1, edb, edf)
			self:set_triangle_edges(t2, efb, ebc, efc)
			self:set_triangle_edges(t3, edf, edb, efb)

			-- update vertex refcounts
			self.vertices_refcount:increment(c)
			self.vertices_refcount:increment(d)
			self.vertices_refcount:increment(f, 4)

			split.bIsBoundary = false
			split.vNew = f
			split.split_t = splitT
			split.eNewBN = efb
			split.eNewCN = efc
			split.eNewDN = edf
			split.eNewT2 = t2
			split.eNewT3 = t3
			split.eOrigT0 = t0
			split.eOrigT1 = t1
			split.eOrigAB = { a, b }

			if self.attributes ~= nil then
				self.attributes:OnSplitEdge(split)
			end

			self:updateTimeStamp(true)
			return MeshResult.Ok, split
		end
	end

	---------------------------------------------------------------------------------------------------
	-- FlipEdge

	function DMesh3:FlipEdgeVerts(vA: number, vB: number)
		local eid = self:find_edge(vA, vB)
		if eid == InvalidID then
			return MeshResult.Failed_NotAnEdge, {}
		end
		return self:FlipEdge(eid)
	end

	function DMesh3:FlipEdge(eab: number)
		local flip = {}
		if not self:IsEdge(eab) then
			return MeshResult.Failed_NotAnEdge, flip
		end
		if self:IsBoundaryEdge(eab) then
			return MeshResult.Failed_IsBoundaryEdge, flip
		end

		-- find oriented edge [a,b], tris t0,t1, and other verts c in t0, d in t1
		local ed = self.edges.data
		local eab_i = 4 * eab
		local a, b = ed[eab_i + 1], ed[eab_i + 2]
		local t0, t1 = ed[eab_i + 3], ed[eab_i + 4]
		local T0tv = self:GetTriangle(t0)
		local T1tv = self:GetTriangle(t1)
		local c
		c, a, b = IndexUtil.orient_tri_edge_and_find_other_vtx(a, b, T0tv)
		local d = IndexUtil.find_tri_other_vtx(a, b, T1tv)
		if c == InvalidID or d == InvalidID then
			return MeshResult.Failed_BrokenTopology, flip
		end

		local flipped = self:find_edge(c, d)
		if flipped ~= InvalidID then
			return MeshResult.Failed_FlippedEdgeExists, flip
		end

		-- find edges bc, ca, ad, db
		local ebc = self:find_tri_neighbour_edge(t0, b, c)
		local eca = self:find_tri_neighbour_edge(t0, c, a)
		local ead = self:find_tri_neighbour_edge(t1, a, d)
		local edb = self:find_tri_neighbour_edge(t1, d, b)

		-- update triangles
		self:set_triangle(t0, c, d, b)
		self:set_triangle(t1, d, c, a)

		-- update edge AB, which becomes flipped edge CD
		self:set_edge_vertices(eab, c, d)
		self:set_edge_triangles(eab, t0, t1)
		local ecd = eab

		-- update the two other edges whose triangle nbrs have changed
		if self:replace_edge_triangle(eca, t0, t1) == -1 then
			error("DMesh3.FlipEdge: first replace_edge_triangle failed")
		end
		if self:replace_edge_triangle(edb, t1, t0) == -1 then
			error("DMesh3.FlipEdge: second replace_edge_triangle failed")
		end

		-- update triangle nbr lists (these are edges)
		self:set_triangle_edges(t0, ecd, edb, ebc)
		self:set_triangle_edges(t1, ecd, eca, ead)

		-- remove old eab from verts a and b, and decrement ref counts
		if self.vertex_edges:Remove(a, eab) == false then
			error("DMesh3.FlipEdge: first edge list remove failed")
		end
		if self.vertex_edges:Remove(b, eab) == false then
			error("DMesh3.FlipEdge: second edge list remove failed")
		end
		self.vertices_refcount:decrement(a)
		self.vertices_refcount:decrement(b)
		if not self:IsVertex(a) or not self:IsVertex(b) then
			error("DMesh3.FlipEdge: either a or b is not a vertex?")
		end

		-- add new edge ecd to verts c and d, and increment ref counts
		self.vertex_edges:Insert(c, ecd)
		self.vertex_edges:Insert(d, ecd)
		self.vertices_refcount:increment(c)
		self.vertices_refcount:increment(d)

		-- success! collect up results
		flip.eID = eab
		flip.v0 = a
		flip.v1 = b
		flip.ov0 = c
		flip.ov1 = d
		flip.t0 = t0
		flip.t1 = t1

		if self.attributes ~= nil then
			self.attributes:OnFlipEdge(flip)
		end

		self:updateTimeStamp(true)
		return MeshResult.Ok, flip
	end

	local function debug_fail(s: string)
		if Util.DebugChecks then
			error("DMesh3.CollapseEdge: check failed: " .. s)
		end
	end

	function DMesh3:check_tri(t: number)
		local tv = self:GetTriangle(t)
		if tv[1] == tv[2] or tv[1] == tv[3] or tv[2] == tv[3] then
			Util.gDevAssert(false)
		end
	end
	function DMesh3:check_edge(e: number)
		local tv = self:GetEdgeT(e)
		if tv[1] == -1 then
			Util.gDevAssert(false)
		end
	end

	---------------------------------------------------------------------------------------------------
	-- CollapseEdge

	function DMesh3:CollapseEdge(vKeep: number, vRemove: number)
		local collapse = {}

		if not self:IsVertex(vKeep) or not self:IsVertex(vRemove) then
			return MeshResult.Failed_NotAnEdge, collapse
		end

		local b = vKeep -- renaming for sanity. We remove a and keep b
		local a = vRemove

		local eab = self:find_edge(a, b)
		if eab == InvalidID then
			return MeshResult.Failed_NotAnEdge, collapse
		end

		local ed = self.edges.data
		local t0 = ed[4 * eab + 3]
		if t0 == InvalidID then
			return MeshResult.Failed_BrokenTopology, collapse
		end
		local T0tv = self:GetTriangle(t0)
		local c = IndexUtil.find_tri_other_vtx(a, b, T0tv)

		-- look up opposing triangle/vtx if we are not in boundary case
		local bIsBoundaryEdge = false
		local d = InvalidID
		local t1 = ed[4 * eab + 4]
		if t1 ~= InvalidID then
			local T1tv = self:GetTriangle(t1)
			d = IndexUtil.find_tri_other_vtx(a, b, T1tv)
			if c == d then
				return MeshResult.Failed_FoundDuplicateTriangle, collapse
			end
		else
			bIsBoundaryEdge = true
		end

		-- We cannot collapse if edge lists of a and b share vertices other
		--  than c and d  (because then we will make a triangle [x b b].
		--  Unfortunately I cannot see a way to do this more efficiently than brute-force search
		--  [TODO] if we had tri iterator for a, couldn't we check each tri for b  (skipping t0 and t1) ?
		local edges_a_count = self.vertex_edges:Count(a)
		local eac, ead, ebc, ebd = InvalidID, InvalidID, InvalidID, InvalidID
		for eid_a in self.vertex_edges:ValueItr(a) do
			local vax = self:edge_other_v(eid_a, a)
			if vax == c then
				eac = eid_a
			elseif vax == d then
				ead = eid_a
			elseif vax ~= b then
				for eid_b in self.vertex_edges:ValueItr(b) do
					if self:edge_other_v(eid_b, b) == vax then
						return MeshResult.Failed_InvalidNeighbourhood, collapse
					end
				end
			end
		end

		-- [RMS] I am not sure this tetrahedron case will detect bowtie vertices.
		-- But the single-triangle case does

		-- We cannot collapse if we have a tetrahedron. In this case a has 3 nbr edges,
		--  and edge cd exists. But that is not conclusive, we also have to check that
		--  cd is an internal edge, and that each of its tris contain a or b
		if edges_a_count == 3 and bIsBoundaryEdge == false then
			local edc = self:find_edge(d, c)
			local edc_i = 4 * edc
			if edc ~= InvalidID and ed[edc_i + 4] ~= InvalidID then
				local edc_t0 = ed[edc_i + 3]
				local edc_t1 = ed[edc_i + 4]

				if (self:tri_has_v(edc_t0, a) and self:tri_has_v(edc_t1, b)) or (self:tri_has_v(edc_t0, b) and self:tri_has_v(edc_t1, a)) then
					return MeshResult.Failed_CollapseTetrahedron, collapse
				end
			end
		elseif bIsBoundaryEdge == true and self:IsBoundaryEdge(eac) then
			-- Cannot collapse edge if we are down to a single triangle
			ebc = self:find_edge_from_tri(b, c, t0)
			if self:IsBoundaryEdge(ebc) then
				return MeshResult.Failed_CollapseTriangle, collapse
			end
		end

		-- [RMS] this was added from C++ version...seems like maybe I never hit
		--   this case? Conceivably could be porting bug but looking at the
		--   C++ code I cannot see how we could possibly have caught this case...
		--
		-- cannot collapse an edge where both vertices are boundary vertices
		-- because that would create a bowtie
		--
		-- NOTE: potentially scanning all edges here...couldn't we
		--  pick up eac/bc/ad/bd as we go? somehow?
		if bIsBoundaryEdge == false and self:IsBoundaryVertex(a) and self:IsBoundaryVertex(b) then
			return MeshResult.Failed_InvalidNeighbourhood, collapse
		end

		-- 1) remove edge ab from vtx b
		-- 2) find edges ad and ac, and tris tad, tac across those edges  (will use later)
		-- 3) for other edges, replace a with b, and add that edge to b
		-- 4) replace a with b in all triangles connected to a
		local tad, tac = InvalidID, InvalidID
		for eid in self.vertex_edges:ValueItr(a) do
			local o = self:edge_other_v(eid, a)
			if o == b then
				if self.vertex_edges:Remove(b, eid) ~= true then
					debug_fail("remove case o == b")
				end
			elseif o == c then
				if self.vertex_edges:Remove(c, eid) ~= true then
					debug_fail("remove case o == c")
				end
				tac = self:edge_other_t(eid, t0)
			elseif o == d then
				if self.vertex_edges:Remove(d, eid) ~= true then
					debug_fail("remove case o == c, step 1")
				end
				tad = self:edge_other_t(eid, t1)
			else
				if self:replace_edge_vertex(eid, a, b) == -1 then
					debug_fail("remove case else")
				end
				self.vertex_edges:Insert(b, eid)
			end

			-- [TODO] perhaps we can already have unique tri list because of the manifold-nbrhood check we need to do...
			for j = 0, 1 do
				local t_j = ed[4 * eid + 3 + j]
				if t_j ~= InvalidID and t_j ~= t0 and t_j ~= t1 then
					if self:tri_has_v(t_j, a) then
						if self:replace_tri_vertex(t_j, a, b) == -1 then
							debug_fail("remove last check")
						end
						self.vertices_refcount:increment(b)
						self.vertices_refcount:decrement(a)
					end
				end
			end
		end

		if bIsBoundaryEdge == false then
			-- remove all edges from vtx a, then remove vtx a
			self.vertex_edges:Clear(a)
			Util.gDevAssert(self.vertices_refcount:refCount(a) == 3) -- in t0,t1, and initial ref
			self.vertices_refcount:decrement(a, 3)
			Util.gDevAssert(self.vertices_refcount:isValid(a) == false)

			-- remove triangles T0 and T1, and update b/c/d refcounts
			self.triangles_refcount:decrement(t0)
			self.triangles_refcount:decrement(t1)
			self.vertices_refcount:decrement(c)
			self.vertices_refcount:decrement(d)
			self.vertices_refcount:decrement(b, 2)
			Util.gDevAssert(self.triangles_refcount:isValid(t0) == false)
			Util.gDevAssert(self.triangles_refcount:isValid(t1) == false)

			-- remove edges ead, eab, eac
			self.edges_refcount:decrement(ead)
			self.edges_refcount:decrement(eab)
			self.edges_refcount:decrement(eac)
			Util.gDevAssert(self.edges_refcount:isValid(ead) == false)
			Util.gDevAssert(self.edges_refcount:isValid(eab) == false)
			Util.gDevAssert(self.edges_refcount:isValid(eac) == false)

			-- replace t0 and t1 in edges ebd and ebc that we kept
			ebd = self:find_edge_from_tri(b, d, t1)
			if ebc == InvalidID then -- we may have already looked this up
				ebc = self:find_edge_from_tri(b, c, t0)
			end

			if self:replace_edge_triangle(ebd, t1, tad) == -1 then
				debug_fail("isboundary=false branch, ebd replace triangle")
			end

			if self:replace_edge_triangle(ebc, t0, tac) == -1 then
				debug_fail("isboundary=false branch, ebc replace triangle")
			end

			-- update tri-edge-nbrs in tad and tac
			if tad ~= InvalidID then
				if self:replace_triangle_edge(tad, ead, ebd) == -1 then
					debug_fail("isboundary=false branch, ebd replace triangle")
				end
			end
			if tac ~= InvalidID then
				if self:replace_triangle_edge(tac, eac, ebc) == -1 then
					debug_fail("isboundary=false branch, ebd replace triangle")
				end
			end
		else
			--  this is basically same code as above, just not referencing t1/d

			-- remove all edges from vtx a, then remove vtx a
			self.vertex_edges:Clear(a)
			Util.gDevAssert(self.vertices_refcount:refCount(a) == 2) -- in t0 and initial ref
			self.vertices_refcount:decrement(a, 2)
			Util.gDevAssert(self.vertices_refcount:isValid(a) == false)

			-- remove triangle T0 and update b/c refcounts
			self.triangles_refcount:decrement(t0)
			self.vertices_refcount:decrement(c)
			self.vertices_refcount:decrement(b)
			Util.gDevAssert(self.triangles_refcount:isValid(t0) == false)

			-- remove edges eab and eac
			self.edges_refcount:decrement(eab)
			self.edges_refcount:decrement(eac)
			Util.gDevAssert(self.edges_refcount:isValid(eab) == false)
			Util.gDevAssert(self.edges_refcount:isValid(eac) == false)

			-- replace t0 in edge ebc that we kept
			ebc = self:find_edge_from_tri(b, c, t0)
			if self:replace_edge_triangle(ebc, t0, tac) == -1 then
				debug_fail("isboundary=false branch, ebc replace triangle")
			end

			-- update tri-edge-nbrs in tac
			if tac ~= InvalidID then
				if self:replace_triangle_edge(tac, eac, ebc) == -1 then
					debug_fail("isboundary=true branch, ebd replace triangle")
				end
			end
		end

		collapse.vKept = vKeep
		collapse.vRemoved = vRemove
		collapse.bIsBoundary = bIsBoundaryEdge
		collapse.eCollapsed = eab
		collapse.tRemoved0 = t0
		collapse.tRemoved1 = t1
		collapse.eRemoved0 = eac
		collapse.eRemoved1 = ead
		collapse.eKept0 = ebc
		collapse.eKept1 = ebd
		collapse.collapse_t = 1.0 -- seems like this is not currently parameterized...

		if self.attributes ~= nil then
			self.attributes:OnCollapseEdge(collapse)
		end

		self:updateTimeStamp(true)
		return MeshResult.Ok, collapse
	end

	---------------------------------------------------------------------------------------------------
	-- MergeEdges

	function DMesh3:MergeEdges(eKeep: number, eDiscard: number)
		local merge_info = {
			eKept = InvalidID,
			eRemoved = InvalidID,
			vKept = { 0, 0 },
			vRemoved = { 0, 0 },
			eRemovedExtra = { InvalidID, InvalidID },
			eKeptExtra = { InvalidID, InvalidID },
		}
		if not self:IsEdge(eKeep) or not self:IsEdge(eDiscard) then
			return MeshResult.Failed_NotAnEdge, merge_info
		end

		local edgeinfo_keep = self:GetEdge(eKeep)
		local edgeinfo_discard = self:GetEdge(eDiscard)
		if edgeinfo_keep[4] ~= InvalidID or edgeinfo_discard[4] ~= InvalidID then
			return MeshResult.Failed_NotABoundaryEdge, merge_info
		end

		local a, b = edgeinfo_keep[1], edgeinfo_keep[2]
		local tab = edgeinfo_keep[3]
		local eab = eKeep
		local c, d = edgeinfo_discard[1], edgeinfo_discard[2]
		local tcd = edgeinfo_discard[3]
		local ecd = eDiscard

		-- Need to correctly orient a,b and c,d and then check that
		-- we will not join triangles with incompatible winding order
		-- I can't see how to do this purely topologically.
		-- So relying on closest-pairs testing.
		local _
		_, a, b = IndexUtil.orient_tri_edge(a, b, self:GetTriangle(tab))
		--int tcd_otherv = IndexUtil.orient_tri_edge_and_find_other_vtx(ref c, ref d, GetTriangle(tcd));
		_, c, d = IndexUtil.orient_tri_edge(c, d, self:GetTriangle(tcd))
		c, d = d, c -- joinable bdry edges have opposing orientations, so flip to get ac and b/d correspondences
		local Va, Vb, Vc, Vd = self:GetVertex(a), self:GetVertex(b), self:GetVertex(c), self:GetVertex(d)
		if (Va:DistanceSquared(Vc) + Vb:DistanceSquared(Vd)) > (Va:DistanceSquared(Vd) + Vb:DistanceSquared(Vc)) then
			return MeshResult.Failed_SameOrientation, merge_info
		end

		-- alternative that detects normal flip of triangle tcd. This is a more
		-- robust geometric test, but fails if tri is degenerate...also more expensive
		--Vector3d otherv = GetVertex(tcd_otherv);
		--Vector3d Ncd = MathUtil.FastNormalDirection(GetVertex(c), GetVertex(d), otherv);
		--Vector3d Nab = MathUtil.FastNormalDirection(GetVertex(a), GetVertex(b), otherv);
		--if (Ncd.Dot(Nab) < 0)
		--return MeshResult.Failed_SameOrientation;

		merge_info.eKept = eab
		merge_info.eRemoved = ecd

		-- if a/c or b/d are connected by an existing edge, we can't merge
		if a ~= c and self:find_edge(a, c) ~= InvalidID then
			return MeshResult.Failed_InvalidNeighbourhood, merge_info
		end
		if b ~= d and self:find_edge(b, d) ~= InvalidID then
			return MeshResult.Failed_InvalidNeighbourhood, merge_info
		end

		-- if vertices at either end already share a common neighbour vertex, and we
		-- do the merge, that would create duplicate edges. This is something like the
		-- 'link condition' in edge collapses.
		-- Note that we have to catch cases where both edges to the shared vertex are
		-- boundary edges, in that case we will also merge this edge later on
		if a ~= c then
			local other_v = (b == d) and b or -1
			for cnbr in self:VtxVerticesItr(c) do
				if cnbr ~= other_v then
					local ea = self:find_edge(a, cnbr)
					if ea ~= InvalidID then
						local ec = self:find_edge(c, cnbr)
						if not self:IsBoundaryEdge(ea) or not self:IsBoundaryEdge(ec) then
							return MeshResult.Failed_InvalidNeighbourhood, merge_info
						end
					end
				end
			end
		end
		if b ~= d then
			local other_v = (a == c) and a or -1
			for dnbr in self:VtxVerticesItr(d) do
				if dnbr ~= other_v then
					local eb = self:find_edge(b, dnbr)
					if eb ~= InvalidID then
						local edd = self:find_edge(d, dnbr)
						if not self:IsBoundaryEdge(eb) or not self:IsBoundaryEdge(edd) then
							return MeshResult.Failed_InvalidNeighbourhood, merge_info
						end
					end
				end
			end
		end

		local ed = self.edges.data

		-- [TODO] this acts on each interior tri twice. could avoid using vtx-tri iterator?
		if a ~= c then
			-- replace c w/ a in edges and tris connected to c, and move edges to a
			for eid in self.vertex_edges:ValueItr(c) do
				if eid ~= eDiscard then
					self:replace_edge_vertex(eid, c, a)
					local rc = 0
					if self:replace_tri_vertex(ed[4 * eid + 3], c, a) >= 0 then
						rc += 1
					end
					if ed[4 * eid + 4] ~= InvalidID then
						if self:replace_tri_vertex(ed[4 * eid + 4], c, a) >= 0 then
							rc += 1
						end
					end
					self.vertex_edges:Insert(a, eid)
					if rc > 0 then
						self.vertices_refcount:increment(a, rc)
						self.vertices_refcount:decrement(c, rc)
					end
				end
			end
			self.vertex_edges:Clear(c)
			self.vertices_refcount:decrement(c)
			merge_info.vRemoved[1] = c
		else
			self.vertex_edges:Remove(a, ecd)
			merge_info.vRemoved[1] = InvalidID
		end
		merge_info.vKept[1] = a

		if d ~= b then
			-- replace d w/ b in edges and tris connected to d, and move edges to b
			for eid in self.vertex_edges:ValueItr(d) do
				if eid ~= eDiscard then
					self:replace_edge_vertex(eid, d, b)
					local rc = 0
					if self:replace_tri_vertex(ed[4 * eid + 3], d, b) >= 0 then
						rc += 1
					end
					if ed[4 * eid + 4] ~= InvalidID then
						if self:replace_tri_vertex(ed[4 * eid + 4], d, b) >= 0 then
							rc += 1
						end
					end
					self.vertex_edges:Insert(b, eid)
					if rc > 0 then
						self.vertices_refcount:increment(b, rc)
						self.vertices_refcount:decrement(d, rc)
					end
				end
			end
			self.vertex_edges:Clear(d)
			self.vertices_refcount:decrement(d)
			merge_info.vRemoved[2] = d
		else
			self.vertex_edges:Remove(b, ecd)
			merge_info.vRemoved[2] = InvalidID
		end
		merge_info.vKept[2] = b

		-- replace edge cd with edge ab in triangle tcd
		self:replace_triangle_edge(tcd, ecd, eab)
		self.edges_refcount:decrement(ecd)

		-- update edge-tri adjacency
		self:set_edge_triangles(eab, tab, tcd)

		-- Once we merge ab to cd, there may be additional edges (now) connected
		-- to either a or b that are connected to the same vertex on their 'other' side.
		-- So we now have two boundary edges connecting the same two vertices - disaster!
		-- We need to find and merge these edges.
		-- Q: I don't think it is possible to have multiple such edge-pairs at a or b
		--    But I am not certain...is a bit tricky to handle because we modify edges_v...
		merge_info.eRemovedExtra = { InvalidID, InvalidID }
		merge_info.eKeptExtra = { InvalidID, InvalidID }
		for vi = 0, 1 do
			local v1, v2 = a, c -- vertices of merged edge
			if vi == 1 then
				v1, v2 = b, d
			end
			if v1 ~= v2 then
				local edges_v = self:vertex_edges_list(v1)
				local Nedges = #edges_v
				local found = false
				-- in this loop, we compare 'other' vert_1 and vert_2 of edges around v1.
				-- problem case is when vert_1 == vert_2  (ie two edges w/ same other vtx).
				--restart_merge_loop:
				local i = 1
				while i <= Nedges and found == false do
					local edge_1 = edges_v[i]
					if self:IsBoundaryEdge(edge_1) then
						local vert_1 = self:edge_other_v(edge_1, v1)
						for j = i + 1, Nedges do
							local edge_2 = edges_v[j]
							local vert_2 = self:edge_other_v(edge_2, v1)
							if vert_1 == vert_2 and self:IsBoundaryEdge(edge_2) then -- if ! boundary here, we are in deep trouble...
								-- replace edge_2 w/ edge_1 in tri, update edge and vtx-edge-nbr lists
								local tri_1 = ed[4 * edge_1 + 3]
								local tri_2 = ed[4 * edge_2 + 3]
								self:replace_triangle_edge(tri_2, edge_2, edge_1)
								self:set_edge_triangles(edge_1, tri_1, tri_2)
								self.vertex_edges:Remove(v1, edge_2)
								self.vertex_edges:Remove(vert_1, edge_2)
								self.edges_refcount:decrement(edge_2)
								merge_info.eRemovedExtra[vi + 1] = edge_2
								merge_info.eKeptExtra[vi + 1] = edge_1

								--edges_v = vertex_edges_list(v1);      // this code allows us to continue checking, ie in case we had
								--Nedges = edges_v.Count;               // multiple such edges. but I don't think it's possible.
								--goto restart_merge_loop;
								found = true -- exit outer i loop
								break -- exit inner j loop
							end
						end
					end
					i += 1
				end
			end
		end

		if self.attributes ~= nil then
			self.attributes:OnMergeEdges(merge_info)
		end

		self:updateTimeStamp(true)
		return MeshResult.Ok, merge_info
	end

	---------------------------------------------------------------------------------------------------
	-- PokeTriangle

	function DMesh3:PokeTriangle(tid: number, baryCoordinates)
		local bary = baryCoordinates or (Vector3d.One / 3.0)
		local result = {}

		if not self:IsTriangle(tid) then
			return MeshResult.Failed_NotATriangle, result
		end

		local tv = self:GetTriangle(tid)
		local te = self:GetTriEdges(tid)

		-- create new vertex with interpolated vertex attribs
		-- TODO: for per-vertex attributes in .Attribs, this will duplicate some work...
		local vinfo = self:GetTriBaryPointInfo(tid, bary.x, bary.y, bary.z)
		local center = self:AppendVertex(vinfo)

		-- add in new edges to center vtx, do not connect to triangles yet
		local eaC = self:add_edge(tv[1], center, -1, -1)
		local ebC = self:add_edge(tv[2], center, -1, -1)
		local ecC = self:add_edge(tv[3], center, -1, -1)
		self.vertices_refcount:increment(tv[1])
		self.vertices_refcount:increment(tv[2])
		self.vertices_refcount:increment(tv[3])
		self.vertices_refcount:increment(center, 3)

		-- old triangle becomes tri along first edge
		self:set_triangle(tid, tv[1], tv[2], center)
		self:set_triangle_edges(tid, te[1], ebC, eaC)

		-- add two new triangles
		local t1 = self:add_triangle_only(tv[2], tv[3], center, te[2], ecC, ebC)
		local t2 = self:add_triangle_only(tv[3], tv[1], center, te[3], eaC, ecC)

		-- second and third edges of original tri have new neighbours
		self:replace_edge_triangle(te[2], tid, t1)
		self:replace_edge_triangle(te[3], tid, t2)

		-- set the triangles for the new edges we created above
		self:set_edge_triangles(eaC, tid, t2)
		self:set_edge_triangles(ebC, tid, t1)
		self:set_edge_triangles(ecC, t1, t2)

		-- transfer groups
		if self:HasTriangleGroups() then
			local g = self.triangle_groups.data[tid + 1]
			self.triangle_groups:insert(g, t1)
			self.triangle_groups:insert(g, t2)
		end

		result.orig_t0 = tid
		result.orig_tri = tv
		result.new_vid = center
		result.new_t1 = t1
		result.new_t2 = t2
		result.new_edges = { eaC, ebC, ecC }
		result.new_vid_barycoords = bary

		if self.attributes ~= nil then
			self.attributes:OnPokeTriangle(result)
		end

		self:updateTimeStamp(true)
		return MeshResult.Ok, result
	end

	---------------------------------------------------------------------------------------------------
	-- internal

	function DMesh3:set_triangle(tid: number, v0: number, v1: number, v2: number)
		local t = self.triangles.data
		local i = 3 * tid
		t[i + 1], t[i + 2], t[i + 3] = v0, v1, v2
	end

	function DMesh3:set_triangle_edges(tid: number, e0: number, e1: number, e2: number)
		local t = self.triangle_edges.data
		local i = 3 * tid
		t[i + 1], t[i + 2], t[i + 3] = e0, e1, e2
	end

	function DMesh3:add_edge(vA: number, vB: number, tA: number, tB: number?): number
		if vB < vA then
			vA, vB = vB, vA
		end
		local eid = self.edges_refcount:allocate()
		local i = 4 * eid
		local edges = self.edges
		edges:insert(vA, i)
		edges:insert(vB, i + 1)
		edges:insert(tA, i + 2)
		edges:insert(tB or InvalidID, i + 3)

		self.vertex_edges:Insert(vA, eid)
		self.vertex_edges:Insert(vB, eid)
		return eid
	end

	function DMesh3:replace_tri_vertex(tID: number, vOld: number, vNew: number): number
		local t = self.triangles.data
		local i = 3 * tID
		if t[i + 1] == vOld then
			t[i + 1] = vNew
			return 0
		end
		if t[i + 2] == vOld then
			t[i + 2] = vNew
			return 1
		end
		if t[i + 3] == vOld then
			t[i + 3] = vNew
			return 2
		end
		return -1
	end

	function DMesh3:add_triangle_only(a: number, b: number, c: number, e0: number, e1: number, e2: number): number
		local tid = self.triangles_refcount:allocate()
		local i = 3 * tid
		local tris, tedges = self.triangles, self.triangle_edges
		tris:insert(c, i + 2)
		tris:insert(b, i + 1)
		tris:insert(a, i)
		tedges:insert(e2, i + 2)
		tedges:insert(e1, i + 1)
		tedges:insert(e0, i + 0)
		return tid
	end

	function DMesh3:allocate_edges_list(vid: number)
		if vid < self.vertex_edges:Size() then
			self.vertex_edges:Clear(vid)
		end
		self.vertex_edges:AllocateAt(vid)
	end

	function DMesh3:vertex_edges_list(vid: number): { number }
		return self.vertex_edges:ValuesArray(vid)
	end

	function DMesh3:vertex_vertices_list(vid: number): { number }
		local vnbrs = {}
		for eid in self.vertex_edges:ValueItr(vid) do
			table.insert(vnbrs, self:edge_other_v(eid, vid))
		end
		return vnbrs
	end

	function DMesh3:set_edge_vertices(eID: number, a: number, b: number)
		local e = self.edges.data
		local i = 4 * eID
		e[i + 1] = math.min(a, b)
		e[i + 2] = math.max(a, b)
	end

	function DMesh3:set_edge_triangles(eID: number, t0: number, t1: number)
		local e = self.edges.data
		local i = 4 * eID
		e[i + 3] = t0
		e[i + 4] = t1
	end

	function DMesh3:replace_edge_vertex(eID: number, vOld: number, vNew: number): number
		local e = self.edges.data
		local i = 4 * eID
		local a, b = e[i + 1], e[i + 2]
		if a == vOld then
			e[i + 1] = math.min(b, vNew)
			e[i + 2] = math.max(b, vNew)
			return 0
		elseif b == vOld then
			e[i + 1] = math.min(a, vNew)
			e[i + 2] = math.max(a, vNew)
			return 1
		end
		return -1
	end

	function DMesh3:replace_edge_triangle(eID: number, tOld: number, tNew: number): number
		local e = self.edges.data
		local i = 4 * eID
		local a, b = e[i + 3], e[i + 4]
		if a == tOld then
			if tNew == InvalidID then
				e[i + 3] = b
				e[i + 4] = InvalidID
			else
				e[i + 3] = tNew
			end
			return 0
		elseif b == tOld then
			e[i + 4] = tNew
			return 1
		end
		return -1
	end

	function DMesh3:replace_triangle_edge(tID: number, eOld: number, eNew: number): number
		local t = self.triangle_edges.data
		local i = 3 * tID
		if t[i + 1] == eOld then
			t[i + 1] = eNew
			return 0
		elseif t[i + 2] == eOld then
			t[i + 2] = eNew
			return 1
		elseif t[i + 3] == eOld then
			t[i + 3] = eNew
			return 2
		end
		return -1
	end
end
