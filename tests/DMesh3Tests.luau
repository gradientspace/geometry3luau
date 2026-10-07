-- Tests for DMesh3 construction, topology queries and edge operators.

return function(G3, h)
	local DMesh3 = G3.DMesh3
	local Vector3d = G3.Vector3d
	local MeshResult = G3.MeshResult

	-- closed octahedron: 6 vertices, 8 triangles, 12 edges
	local function makeOctahedron()
		local mesh = DMesh3.new()
		local verts = {
			Vector3d.new(1, 0, 0),
			Vector3d.new(-1, 0, 0),
			Vector3d.new(0, 1, 0),
			Vector3d.new(0, -1, 0),
			Vector3d.new(0, 0, 1),
			Vector3d.new(0, 0, -1),
		}
		for _, v in verts do
			mesh:AppendVertex(v)
		end
		local tris = {
			{ 0, 2, 4 },
			{ 2, 1, 4 },
			{ 1, 3, 4 },
			{ 3, 0, 4 },
			{ 2, 0, 5 },
			{ 1, 2, 5 },
			{ 3, 1, 5 },
			{ 0, 3, 5 },
		}
		for _, t in tris do
			local tid = mesh:AppendTriangle(t[1], t[2], t[3])
			assert(tid >= 0, "AppendTriangle failed: " .. tid)
		end
		return mesh
	end
	h.makeOctahedron = makeOctahedron

	-- planar grid of n x n quads in XY plane (open mesh)
	local function makeGrid(n: number, size: number)
		local mesh = DMesh3.new()
		for j = 0, n do
			for i = 0, n do
				mesh:AppendVertex(Vector3d.new(size * i / n, size * j / n, 0))
			end
		end
		for j = 0, n - 1 do
			for i = 0, n - 1 do
				local a = j * (n + 1) + i
				local b, c, d = a + 1, a + n + 2, a + n + 1
				mesh:AppendTriangle(a, b, c)
				mesh:AppendTriangle(a, c, d)
			end
		end
		return mesh
	end

	h:test("octahedron topology", function()
		local mesh = makeOctahedron()
		h.eq(mesh:VertexCount(), 6)
		h.eq(mesh:TriangleCount(), 8)
		h.eq(mesh:EdgeCount(), 12)
		h.truthy(mesh:IsClosed(), "closed")
		h.truthy(mesh:CheckValidity(false), "valid")
		h.truthy(mesh:IsCompact(), "compact")
		for vid in mesh:VertexIndices() do
			h.eq(mesh:GetVtxEdgeCount(vid), 4)
			h.eq(mesh:GetVtxTriangleCount(vid), 4)
			h.truthy(not mesh:IsBoundaryVertex(vid))
			h.truthy(not mesh:IsBowtieVertex(vid))
		end
		local e = mesh:FindEdge(0, 2)
		h.truthy(e >= 0)
		local ov = mesh:GetEdgeOpposingV(e)
		h.truthy((ov[1] == 4 and ov[2] == 5) or (ov[1] == 5 and ov[2] == 4))
		h.eq(mesh:FindTriangle(4, 2, 0), 0)
		local b = mesh:GetBounds()
		h.near(b.Min.x, -1)
		h.near(b.Max.z, 1)
		-- winding number inside / outside
		h.near(mesh:WindingNumber(Vector3d.Zero), 1, 1e-9)
		h.near(mesh:WindingNumber(Vector3d.new(3, 3, 3)), 0, 1e-9)
	end)

	h:test("non-manifold and degenerate triangles rejected", function()
		local mesh = makeOctahedron()
		local v = mesh:AppendVertex(Vector3d.new(2, 2, 2))
		h.eq(mesh:AppendTriangle(0, 2, v), DMesh3.NonManifoldID)
		h.eq(mesh:AppendTriangle(0, 0, v), DMesh3.DegenerateID)
		h.eq(mesh:AppendTriangle(0, 1, 99), DMesh3.InvalidID)
	end)

	h:test("split / flip / collapse edges", function()
		local mesh = makeOctahedron()
		local eid = mesh:FindEdge(0, 2)
		local r, split = mesh:SplitEdge(eid)
		h.eq(r, MeshResult.Ok)
		h.eq(mesh:VertexCount(), 7)
		h.eq(mesh:TriangleCount(), 10)
		h.truthy(mesh:CheckValidity(false))
		local p = mesh:GetVertex(split.vNew)
		h.near(p.x, 0.5)
		h.near(p.y, 0.5)

		-- flip an edge adjacent to new vertex
		local fe = mesh:FindEdge(split.vNew, 4)
		local fr, flip = mesh:FlipEdge(fe)
		h.eq(fr, MeshResult.Ok)
		h.truthy(mesh:CheckValidity(false))
		h.eq(mesh:FindEdge(split.vNew, 4), DMesh3.InvalidID)
		h.truthy(mesh:FindEdge(flip.ov0, flip.ov1) >= 0)

		-- collapse it back
		local cr, collapse = mesh:CollapseEdge(0, split.vNew)
		h.eq(cr, MeshResult.Ok, "collapse " .. MeshResult.Name(cr))
		h.truthy(mesh:CheckValidity(false))
		h.eq(mesh:VertexCount(), 6)
		h.eq(mesh:TriangleCount(), 8)
		h.truthy(mesh:IsClosed())
		h.eq(collapse.vKept, 0)
	end)

	h:test("collapse on boundary and remove triangles", function()
		local mesh = makeGrid(4, 4)
		h.eq(mesh:VertexCount(), 25)
		h.eq(mesh:TriangleCount(), 32)
		h.truthy(mesh:CheckValidity(false))
		h.truthy(not mesh:IsClosed())
		local nb = 0
		for _ in mesh:BoundaryEdgeIndices() do
			nb += 1
		end
		h.eq(nb, 16)
		-- collapse boundary edge
		local r = mesh:CollapseEdge(0, 1)
		h.eq(r, MeshResult.Ok)
		h.truthy(mesh:CheckValidity(false))
		-- remove a triangle
		local tid = mesh:TriangleIndices()()
		h.eq(mesh:RemoveTriangle(tid, true, false), MeshResult.Ok)
		h.truthy(mesh:CheckValidity(false))
		-- compact
		h.truthy(not mesh:IsCompact())
		mesh:CompactInPlace()
		h.truthy(mesh:IsCompact())
		h.truthy(mesh:CheckValidity(false))
	end)

	h:test("copy, compact copy, poke, merge edges", function()
		local mesh = makeGrid(3, 3)
		mesh:RemoveTriangle(5, true, false)
		local copy = DMesh3.FromCopy(mesh)
		h.truthy(copy:IsSameMesh(mesh, true, true))
		local compact = DMesh3.FromCopy(mesh, true)
		h.truthy(compact:IsCompact())
		h.eq(compact:TriangleCount(), mesh:TriangleCount())
		h.truthy(compact:CheckValidity(true))

		local pr, poke = compact:PokeTriangle(0)
		h.eq(pr, MeshResult.Ok)
		h.truthy(compact:CheckValidity(true))
		h.eq(poke.new_vid, compact:MaxVertexID() - 1)

		-- two separate triangles sharing coincident boundary edges -> merge
		local m = DMesh3.new()
		local a = m:AppendVertex(Vector3d.new(0, 0, 0))
		local b = m:AppendVertex(Vector3d.new(1, 0, 0))
		local c = m:AppendVertex(Vector3d.new(0, 1, 0))
		local a2 = m:AppendVertex(Vector3d.new(0, 0, 0))
		local b2 = m:AppendVertex(Vector3d.new(1, 0, 0))
		local d = m:AppendVertex(Vector3d.new(0, -1, 0))
		m:AppendTriangle(a, b, c)
		m:AppendTriangle(b2, a2, d)
		local mr = m:MergeEdges(m:FindEdge(a, b), m:FindEdge(a2, b2))
		h.eq(mr, MeshResult.Ok)
		h.truthy(m:CheckValidity(false))
		h.eq(m:VertexCount(), 4)
		h.eq(m:EdgeCount(), 5)
	end)

	h:test("attributes follow edits", function()
		local mesh = makeOctahedron()
		mesh:EnableVertexColors(Vector3d.new(1, 0, 0))
		mesh:EnableTriangleGroups(3)
		local attribs = mesh:Attribs()
		attribs:EnableTriUVs(1)
		local uvs = attribs:TriUVChannel(0)
		local TriUVs = G3.GeoAttributes.TriUVs
		local Vector2d = G3.Vector2d
		for tid in mesh:TriangleIndices() do
			uvs:SetValue(tid, TriUVs.new(Vector2d.new(0, 0), Vector2d.new(1, 0), Vector2d.new(0, 1)))
		end
		local r = mesh:SplitEdge(mesh:FindEdge(0, 2))
		h.eq(r, MeshResult.Ok)
		mesh:PokeTriangle(1)
		h.truthy(mesh:CheckValidity(false))
		h.eq(uvs:NumElements(), mesh:MaxTriangleID())
		h.eq(mesh:GetTriangleGroup(mesh:MaxTriangleID() - 1), 3)
		local c = mesh:GetVertexColor(mesh:MaxVertexID() - 1)
		h.near(c.x, 1)
		mesh:ReverseOrientation()
		h.truthy(mesh:CheckValidity(false))
		local compact = DMesh3.FromCopy(mesh, true)
		h.truthy(compact:CheckValidity(false))
		h.truthy(compact:Attribs():HasTriUVs())
	end)

	return h:results()
end
