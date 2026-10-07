-- Port of geometry3Sharp mesh/DMesh3_debug.cs (partial class DMesh3)
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Returns a function that installs debug/validation functions into the DMesh3 class table:
--   debug_print_vertex, debug_print_mesh, MeshInfoString, IsSameMesh, CheckValidity
-- (debug_check_is_vertex/triangle/edge are defined in DMesh3.luau.)

local Util = require(script.Parent.Util)
local MathUtil = require(script.Parent.MathUtil)
local IndexUtil = require(script.Parent.IndexUtil)

local InvalidID = -1

return function(DMesh3)
	function DMesh3:debug_print_vertex(v: number)
		print("Vertex " .. tostring(v))
		local tris = {}
		self:GetVtxTriangles(v, tris, false)
		print(string.format("  Tris %d  Edges %d  refcount %d", #tris, self:GetVtxEdgeCount(v), self.vertices_refcount:refCount(v)))
		for _, t in tris do
			local tv, te = self:GetTriangle(t), self:GetTriEdges(t)
			print(string.format("  t%d %d %d %d   te %d %d %d", t, tv[1], tv[2], tv[3], te[1], te[2], te[3]))
		end
		for e in self:VtxEdgesItr(v) do
			local ev, et = self:GetEdgeV(e), self:GetEdgeT(e)
			print(string.format("  e%d %d %d / %d %d", e, ev[1], ev[2], et[1], et[2]))
		end
	end

	function DMesh3:debug_print_mesh()
		for k = 0, self.vertices_refcount:max_index() - 1 do
			if not self.vertices_refcount:isValid(k) then
				print(string.format("v%d : invalid", k))
			else
				self:debug_print_vertex(k)
			end
		end
	end

	function DMesh3:MeshInfoString(): string
		local b = {}
		table.insert(
			b,
			string.format("Vertices  count %d max %d %s", self:VertexCount(), self:MaxVertexID(), self.vertices_refcount:UsageStats())
		)
		table.insert(
			b,
			string.format("Triangles count %d max %d %s", self:TriangleCount(), self:MaxTriangleID(), self.triangles_refcount:UsageStats())
		)
		table.insert(b, string.format("Edges     count %d max %d %s", self:EdgeCount(), self:MaxEdgeID(), self.edges_refcount:UsageStats()))
		table.insert(
			b,
			string.format(
				"Normals %s  Colors %s  UVs %s  Groups %s",
				tostring(self:HasVertexNormals()),
				tostring(self:HasVertexColors()),
				tostring(self:HasVertexUVs()),
				tostring(self:HasTriangleGroups())
			)
		)
		table.insert(
			b,
			string.format(
				"Closed %s Compact %s timestamp %d shape_timestamp %d  MaxGroupID %d",
				tostring(self:CachedIsClosed()),
				tostring(self:IsCompact()),
				self.timestamp,
				self.shape_timestamp,
				self.max_group_id
			)
		)
		table.insert(b, "VertexEdges " .. self.vertex_edges:MemoryUsage())
		return table.concat(b, "\n") .. "\n"
	end

	function DMesh3:IsSameMesh(
		m2,
		bCheckConnectivity: boolean,
		bCheckEdgeIDs: boolean?,
		bCheckNormals: boolean?,
		bCheckColors: boolean?,
		bCheckUVs: boolean?,
		bCheckGroups: boolean?,
		Epsilon: number?
	): boolean
		local eps = Epsilon or MathUtil.Epsilonf
		if self:VertexCount() ~= m2:VertexCount() then
			return false
		end
		if self:TriangleCount() ~= m2:TriangleCount() then
			return false
		end
		for vid in self:VertexIndices() do
			if not m2:IsVertex(vid) or not self:GetVertex(vid):EpsilonEqual(m2:GetVertex(vid), eps) then
				return false
			end
		end
		for tid in self:TriangleIndices() do
			if not m2:IsTriangle(tid) then
				return false
			end
			local a, b = self:GetTriangle(tid), m2:GetTriangle(tid)
			if a[1] ~= b[1] or a[2] ~= b[2] or a[3] ~= b[3] then
				return false
			end
		end
		if bCheckConnectivity then
			for eid in self:EdgeIndices() do
				local e = self:GetEdge(eid)
				local other_eid = m2:FindEdge(e[1], e[2])
				if other_eid == InvalidID then
					return false
				end
				local oe = m2:GetEdge(other_eid)
				if math.min(e[3], e[4]) ~= math.min(oe[3], oe[4]) or math.max(e[3], e[4]) ~= math.max(oe[3], oe[4]) then
					return false
				end
			end
		end
		if bCheckEdgeIDs then
			if self:EdgeCount() ~= m2:EdgeCount() then
				return false
			end
			for eid in self:EdgeIndices() do
				if not m2:IsEdge(eid) then
					return false
				end
				local a, b = self:GetEdge(eid), m2:GetEdge(eid)
				if a[1] ~= b[1] or a[2] ~= b[2] or a[3] ~= b[3] or a[4] ~= b[4] then
					return false
				end
			end
		end
		if bCheckNormals then
			if self:HasVertexNormals() ~= m2:HasVertexNormals() then
				return false
			end
			if self:HasVertexNormals() then
				for vid in self:VertexIndices() do
					if not self:GetVertexNormal(vid):EpsilonEqual(m2:GetVertexNormal(vid), eps) then
						return false
					end
				end
			end
		end
		if bCheckColors then
			if self:HasVertexColors() ~= m2:HasVertexColors() then
				return false
			end
			if self:HasVertexColors() then
				for vid in self:VertexIndices() do
					if not self:GetVertexColor(vid):EpsilonEqual(m2:GetVertexColor(vid), eps) then
						return false
					end
				end
			end
		end
		if bCheckUVs then
			if self:HasVertexUVs() ~= m2:HasVertexUVs() then
				return false
			end
			if self:HasVertexUVs() then
				for vid in self:VertexIndices() do
					if not self:GetVertexUV(vid):EpsilonEqual(m2:GetVertexUV(vid), eps) then
						return false
					end
				end
			end
		end
		if bCheckGroups then
			if self:HasTriangleGroups() ~= m2:HasTriangleGroups() then
				return false
			end
			if self:HasTriangleGroups() then
				for tid in self:TriangleIndices() do
					if self:GetTriangleGroup(tid) ~= m2:GetTriangleGroup(tid) then
						return false
					end
				end
			end
		end
		return true
	end

	-- This function checks that the mesh is well-formed, ie all internal data
	-- structures are consistent
	function DMesh3:CheckValidity(bAllowNonManifoldVertices: boolean?, eFailMode: number?): boolean
		local allowNonManifold = bAllowNonManifoldVertices ~= false
		local failMode = eFailMode or Util.FailMode.Throw

		local triToVtxRefs = table.create(self:MaxVertexID(), 0)

		local is_ok = true
		local function CheckOrFailF(b: boolean)
			if failMode == Util.FailMode.Throw then
				if not b then
					error("DMesh3.CheckValidity: check failed", 2)
				end
			elseif failMode == Util.FailMode.DebugAssert or failMode == Util.FailMode.gDevAssert then
				Util.gDevAssert(b)
			end
			is_ok = is_ok and b
		end

		if self.normals ~= nil then
			CheckOrFailF(self.normals.size == self.vertices.size)
		end
		if self.uv ~= nil then
			CheckOrFailF(self.uv.size // 2 == self.vertices.size // 3)
		end
		if self.triangle_groups ~= nil then
			CheckOrFailF(self.triangle_groups.size == self.triangles.size // 3)
		end

		for tID in self:TriangleIndices() do
			CheckOrFailF(self:IsTriangle(tID))
			CheckOrFailF(self.triangles_refcount:refCount(tID) == 1)

			-- vertices must exist
			local tv = self:GetTriangle(tID)
			for j = 1, 3 do
				CheckOrFailF(self:IsVertex(tv[j]))
				triToVtxRefs[tv[j] + 1] += 1
			end

			-- edges must exist and reference this tri
			local e = { 0, 0, 0 }
			for j = 0, 2 do
				local a, b = tv[j + 1], tv[((j + 1) % 3) + 1]
				e[j + 1] = self:FindEdge(a, b)
				CheckOrFailF(e[j + 1] ~= InvalidID)
				CheckOrFailF(self:edge_has_t(e[j + 1], tID))
				CheckOrFailF(e[j + 1] == self:FindEdgeFromTri(a, b, tID))
			end
			CheckOrFailF(e[1] ~= e[2] and e[1] ~= e[3] and e[2] ~= e[3])

			-- tri nbrs must exist and reference this tri, or same edge must be boundary edge
			local te = self:GetTriEdges(tID)
			for j = 0, 2 do
				local eid = te[j + 1]
				CheckOrFailF(self:IsEdge(eid))
				local tOther = self:edge_other_t(eid, tID)
				if tOther == InvalidID then
					CheckOrFailF(self:tri_is_boundary(tID))
				else
					CheckOrFailF(self:tri_has_neighbour_t(tOther, tID) == true)

					-- edge must have same two verts as tri for same index
					local a, b = tv[j + 1], tv[((j + 1) % 3) + 1]
					local ev = self:GetEdgeV(te[j + 1])
					CheckOrFailF(IndexUtil.same_pair_unordered(a, b, ev[1], ev[2]))

					-- also check that nbr edge has opposite orientation
					local othertv = self:GetTriangle(tOther)
					local found = IndexUtil.find_tri_ordered_edge(b, a, othertv)
					CheckOrFailF(found ~= InvalidID)
				end
			end
		end

		-- edge verts/tris must exist
		for eID in self:EdgeIndices() do
			CheckOrFailF(self:IsEdge(eID))
			CheckOrFailF(self.edges_refcount:refCount(eID) == 1)
			local ev = self:GetEdgeV(eID)
			local et = self:GetEdgeT(eID)
			CheckOrFailF(self:IsVertex(ev[1]))
			CheckOrFailF(self:IsVertex(ev[2]))
			CheckOrFailF(et[1] ~= InvalidID)
			CheckOrFailF(ev[1] < ev[2])
			CheckOrFailF(self:IsTriangle(et[1]))
			if et[2] ~= InvalidID then
				CheckOrFailF(self:IsTriangle(et[2]))
			end
		end

		-- verify compact check
		local is_compact = self.vertices_refcount:is_dense()
		if is_compact then
			for vid = 0, (self.vertices.size // 3) - 1 do
				CheckOrFailF(self.vertices_refcount:isValid(vid))
			end
		end

		-- vertex edges must exist and reference this vert
		for vID in self:VertexIndices() do
			CheckOrFailF(self:IsVertex(vID))

			local v = self:GetVertex(vID)
			local lsq = v:LengthSquared()
			CheckOrFailF(lsq == lsq)
			CheckOrFailF(lsq ~= math.huge)

			for edgeid in self.vertex_edges:ValueItr(vID) do
				CheckOrFailF(self:IsEdge(edgeid))
				CheckOrFailF(self:edge_has_v(edgeid, vID))

				local otherV = self:edge_other_v(edgeid, vID)
				local e2 = self:find_edge(vID, otherV)
				CheckOrFailF(e2 ~= InvalidID)
				CheckOrFailF(e2 == edgeid)
				e2 = self:find_edge(otherV, vID)
				CheckOrFailF(e2 ~= InvalidID)
				CheckOrFailF(e2 == edgeid)
			end

			for nbr_vid in self:VtxVerticesItr(vID) do
				CheckOrFailF(self:IsVertex(nbr_vid))
				local edge = self:find_edge(vID, nbr_vid)
				CheckOrFailF(self:IsEdge(edge))
			end

			local vTris, vTris2 = {}, {}
			self:GetVtxTriangles(vID, vTris, false)
			self:GetVtxTriangles(vID, vTris2, true)
			CheckOrFailF(#vTris == #vTris2)
			if allowNonManifold then
				CheckOrFailF(#vTris <= self:GetVtxEdgeCount(vID))
			else
				CheckOrFailF(#vTris == self:GetVtxEdgeCount(vID) or #vTris == self:GetVtxEdgeCount(vID) - 1)
			end
			CheckOrFailF(self.vertices_refcount:refCount(vID) == #vTris + 1)
			CheckOrFailF(triToVtxRefs[vID + 1] == #vTris)
			for _, tID in vTris do
				CheckOrFailF(self:tri_has_v(tID, vID))
			end

			-- check that edges around vert only references tris above, and reference all of them!
			local vRemoveTris = table.clone(vTris)
			for edgeid in self.vertex_edges:ValueItr(vID) do
				local edget = self:GetEdgeT(edgeid)
				CheckOrFailF(table.find(vTris, edget[1]) ~= nil)
				if edget[2] ~= InvalidID then
					CheckOrFailF(table.find(vTris, edget[2]) ~= nil)
				end
				local i0 = table.find(vRemoveTris, edget[1])
				if i0 then
					table.remove(vRemoveTris, i0)
				end
				if edget[2] ~= InvalidID then
					local i1 = table.find(vRemoveTris, edget[2])
					if i1 then
						table.remove(vRemoveTris, i1)
					end
				end
			end
			CheckOrFailF(#vRemoveTris == 0)
		end

		if self:HasAttributes() then
			self.attributes:CheckValidity(failMode)
		end

		return is_ok
	end
end
