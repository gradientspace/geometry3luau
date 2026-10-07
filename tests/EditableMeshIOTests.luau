-- Tests for EditableMeshIO / MeshArrays / DMesh3Builder using a mock EditableMesh.
-- (RobloxMesh itself needs the engine; see RobloxMeshTests.)

return function(G3, h)
	local TM = require(script.Parent.TestMeshes)
	local Mock = require(script.Parent.MockEditableMesh)
	local IO = G3.EditableMeshIO
	local MeshArrays = G3.MeshArrays
	local ctors = { Vector3 = Vector3, Vector2 = Vector2, Color3 = Color3 }

	local function signedVolume(mesh)
		local vol = 0
		for tid in mesh:TriangleIndices() do
			local a, b, c = mesh:GetTriVertices(tid)
			vol += a:Dot(b:Cross(c)) / 6
		end
		return vol
	end

	h:test("write/read round trip with shared vertices", function()
		local mesh = TM.UVSphere(G3, 2, 24, 12)
		local em = Mock.new()
		local info = IO.Write(mesh, em, ctors)
		h.eq(info.triangleCount, mesh:TriangleCount())
		h.eq(info.vertexCount, mesh:VertexCount())
		local back = IO.Read(em)
		h.eq(back:VertexCount(), mesh:VertexCount())
		h.eq(back:TriangleCount(), mesh:TriangleCount())
		h.truthy(back:IsClosed())
		h.truthy(back:CheckValidity(false))
		h.near(signedVolume(back), signedVolume(mesh), 1e-4, "volume")
	end)

	h:test("triangle soup is welded by position", function()
		local mesh = TM.Box(G3, 1, 2, 3, 2)
		-- write as soup: one editable vertex per corner
		local em = Mock.new()
		for tid in mesh:TriangleIndices() do
			local a, b, c = mesh:GetTriVertices(tid)
			local ia = em:AddVertex(Vector3.new(a.x, a.y, a.z))
			local ib = em:AddVertex(Vector3.new(b.x, b.y, b.z))
			local ic = em:AddVertex(Vector3.new(c.x, c.y, c.z))
			em:AddTriangle(ia, ib, ic)
		end
		local soup = IO.Read(em, { weld = "ids" })
		h.eq(soup:VertexCount(), 3 * mesh:TriangleCount())
		h.truthy(not soup:IsClosed())
		local welded = IO.Read(em, { weld = "positions" })
		h.eq(welded:VertexCount(), mesh:VertexCount())
		h.truthy(welded:IsClosed())
		h.truthy(welded:CheckValidity(false))
		h.near(signedVolume(welded), 6, 1e-4)
	end)

	h:test("per-triangle normals and uvs round trip", function()
		local mesh = TM.Box(G3, 2, 2, 2, 1)
		local attr = mesh:Attribs()
		attr:EnableTriNormals()
		attr:EnableTriUVs(1)
		local TriNormals, TriUVs = G3.GeoAttributes.TriNormals, G3.GeoAttributes.TriUVs
		for tid in mesh:TriangleIndices() do
			local n = mesh:GetTriNormal(tid)
			attr:TriNormals():SetValue(tid, TriNormals.new(n, n, n))
			attr:TriUVChannel(0):SetValue(tid, TriUVs.new(G3.Vector2d.new(0, 0), G3.Vector2d.new(1, 0), G3.Vector2d.new(0.5, 1)))
		end
		local em = Mock.new()
		IO.Write(mesh, em, ctors)
		local back = IO.Read(em)
		h.truthy(back:Attribs():HasTriNormals())
		h.truthy(back:Attribs():HasTriUVChannel(0))
		for tid in back:TriangleIndices() do
			local n = back:GetTriNormal(tid)
			local tn = back:Attribs():TriNormals():GetValue(tid)
			h.near(tn.A:Dot(n), 1, 1e-5, "normal matches face")
			local uv = back:Attribs():TriUVChannel(0):GetValue(tid)
			h.near(uv.C.y, 1, 1e-6)
		end
		-- vertex normal mode
		local back2 = IO.Read(em, { normals = "vertex", uvs = "none" })
		h.truthy(back2:HasVertexNormals())
		local n0 = back2:GetVertexNormal(0)
		h.near(n0:Length(), 1, 1e-6)
	end)

	h:test("computed and flat normals are written", function()
		local mesh = TM.Octahedron(G3)
		for _, mode in { "computed", "flat", "vertex" } do
			local em = Mock.new()
			IO.Write(mesh, em, { Vector3 = Vector3, normals = mode })
			local n = 0
			for _ in em.faceNormals do
				n += 1
			end
			h.eq(n, 8, mode)
		end
	end)

	h:test("MeshArrays round trip, welding, non-manifold duplication", function()
		local mesh = TM.Torus(G3, 2, 0.5, 16, 8)
		local arrays = MeshArrays.ToArrays(mesh)
		local back, info = MeshArrays.FromArrays(arrays.positions, arrays.triangles)
		h.eq(info.failed, 0)
		h.truthy(back:IsSameMesh(mesh, true))
		-- three triangles sharing one edge: third is non-manifold -> duplicated
		local pos = { 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 1 }
		local tris = { 0, 1, 2, 1, 0, 3, 0, 1, 4 }
		local m2, info2 = MeshArrays.FromArrays(pos, tris)
		h.eq(info2.failed, 0)
		h.eq(m2:TriangleCount(), 3)
		h.eq(m2:VertexCount(), 8)
		h.truthy(m2:CheckValidity(true))
		local m3, info3 = MeshArrays.FromArrays(pos, tris, { nonManifold = "discard" })
		h.eq(info3.failed, 1)
		h.eq(m3:TriangleCount(), 2)
	end)

	h:test("partition into bounded pieces", function()
		local mesh = TM.UVSphere(G3, 1, 64, 32)
		local groups = MeshArrays.PartitionTriangles(mesh, 500, 400)
		local total = 0
		local seen = {}
		for _, g in groups do
			h.truthy(#g <= 500, "triangle limit")
			total += #g
			local sub, map = MeshArrays.ExtractSubmesh(mesh, g)
			h.truthy(sub:VertexCount() <= 400, "vertex limit")
			h.eq(sub:TriangleCount(), #g)
			h.truthy(sub:CheckValidity(true))
			for _, tid in g do
				h.truthy(not seen[tid], "triangle in one group")
				seen[tid] = true
			end
			for sv, bv in map do
				h.truthy(sub:GetVertex(sv):EpsilonEqual(mesh:GetVertex(bv), 0))
			end
		end
		h.eq(total, mesh:TriangleCount())
	end)

	h:test("DMesh3Builder.Build", function()
		local mesh = G3.DMesh3Builder.Build({ 0, 0, 0, 1, 0, 0, 0, 1, 0 }, { 0, 1, 2 }, { 0, 0, 1, 0, 0, 1, 0, 0, 1 }, { 7 })
		h.eq(mesh:TriangleCount(), 1)
		h.eq(mesh:GetTriangleGroup(0), 7)
		h.near(mesh:GetVertexNormal(1).z, 1)
	end)

	return h:results()
end
