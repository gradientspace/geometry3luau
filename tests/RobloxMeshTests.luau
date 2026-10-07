-- Engine tests for RobloxMesh (EditableMesh / MeshPart conversion). Skipped headless.

return function(G3, h)
	if not h.native then
		h:skip("RobloxMesh engine tests", "requires Roblox engine (run Tests.TestRunner in Studio)")
		return h:results()
	end

	local TM = require(script.Parent.TestMeshes)
	local RobloxMesh = G3.RobloxMesh

	local function signedVolume(mesh)
		local vol = 0
		for tid in mesh:TriangleIndices() do
			local a, b, c = mesh:GetTriVertices(tid)
			vol += a:Dot(b:Cross(c)) / 6
		end
		return vol
	end

	h:test("EditableMesh round trip", function()
		local mesh = TM.UVSphere(G3, 2, 24, 12)
		local editable = RobloxMesh.ToEditableMesh(mesh)
		h.eq(#editable:GetFaces(), mesh:TriangleCount())
		local back = RobloxMesh.FromEditableMesh(editable)
		editable:Destroy()
		h.eq(back:TriangleCount(), mesh:TriangleCount())
		h.truthy(back:IsClosed(), "closed")
		h.near(signedVolume(back), signedVolume(mesh), 1e-3, "volume")
	end)

	h:test("MeshPart round trip in world space", function()
		local mesh = TM.Box(G3, 2, 4, 6, 2, G3.Vector3d.new(10, 20, 30))
		local part, editable = RobloxMesh.ToMeshPart(mesh, { cframe = CFrame.new(0, 5, 0) })
		h.near(part.Position.X, 10, 1e-3)
		h.near(part.Position.Y, 25, 1e-3)
		h.near(part.Size.Z, 6, 1e-3)
		local back = RobloxMesh.FromMeshPart(part)
		h.truthy(back:IsClosed(), "closed")
		local b = back:GetBounds()
		h.near(b.Min.x, 9, 1e-3)
		h.near(b.Max.y, 27, 1e-3)
		h.near(signedVolume(back), 48, 1e-2, "volume")
		-- scaled part: geometry follows Size
		part.Size = part.Size * 2
		local scaled = RobloxMesh.FromMeshPart(part)
		h.near(signedVolume(scaled), 48 * 8, 1e-1, "scaled volume")
		part:Destroy()
		editable:Destroy()
	end)

	h:test("ToMeshParts splits large meshes", function()
		local mesh = TM.UVSphere(G3, 4, 64, 40)
		local parts, editables = RobloxMesh.ToMeshParts(mesh, { maxTriangles = 2000 })
		h.truthy(#parts >= 3, "multiple parts")
		local tris = 0
		for i, p in parts do
			tris += #editables[i]:GetFaces()
			p:Destroy()
			editables[i]:Destroy()
		end
		h.eq(tris, mesh:TriangleCount())
	end)

	return h:results()
end
