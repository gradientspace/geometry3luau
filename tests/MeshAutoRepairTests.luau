-- Tests for MeshAutoRepair (package D): broken meshes in, closed valid outward-oriented meshes out.

return function(G3, h)
	local TM = require(script.Parent.TestMeshes)
	local V = G3.Vector3d.new

	local function signedVolume(mesh)
		local vol = 0
		for tid in mesh:TriangleIndices() do
			local a, b, c = mesh:GetTriVertices(tid)
			vol += a:Dot(b:Cross(c)) / 6
		end
		return vol
	end

	-- triangle soup: every triangle gets its own vertices; optional per-triangle orientation flip
	local function unweld(mesh, flipF)
		local soup = G3.DMesh3.new()
		for tid in mesh:TriangleIndices() do
			local a, b, c = mesh:GetTriVertices(tid)
			local ia, ib, ic = soup:AppendVertex(a), soup:AppendVertex(b), soup:AppendVertex(c)
			if flipF and flipF(tid) then
				soup:AppendTriangle(ia, ic, ib)
			else
				soup:AppendTriangle(ia, ib, ic)
			end
		end
		return soup
	end

	local function checkRepaired(mesh, expectVolume, eps, msg)
		h.truthy(mesh:CheckValidity(false), msg .. ": valid")
		h.truthy(mesh:IsClosed(), msg .. ": closed")
		h.truthy(mesh:IsCompact(), msg .. ": compact")
		local vol = signedVolume(mesh)
		h.truthy(vol > 0, msg .. ": positive volume " .. vol)
		if expectVolume then
			h.near(vol, expectVolume, eps, msg .. ": volume")
		end
		h.eq(G3.MeshValidation.HasDuplicateTriangles(mesh), G3.ValidationStatus.Ok, msg .. ": duplicates")
	end

	h:test("closed mesh passes through", function()
		local box = TM.Box(G3, 1, 2, 3, 2)
		local repair = G3.MeshAutoRepair.new(box)
		h.truthy(repair:Apply())
		checkRepaired(repair.Mesh, 6, 1e-9, "box")
		h.eq(repair.Mesh:TriangleCount(), box:TriangleCount())
		h.truthy(repair.Mesh:HasVertexNormals())
	end)

	h:test("inside-out mesh is reoriented", function()
		local sphere = TM.UVSphere(G3, 1, 16, 8)
		sphere:ReverseOrientation()
		local repair = G3.MeshAutoRepair.new(sphere)
		h.truthy(repair:Apply())
		checkRepaired(repair.Mesh, nil, 0, "sphere")
	end)

	-- (note: per-triangle random flips in a soup cannot be repaired by this algorithm, in C# either:
	--  MergeEdges refuses to join edges with the same orientation, so it leaves cracks between
	--  oppositely-oriented regions. Here the whole soup is inside-out.)
	h:test("cracked inside-out soup", function()
		local box = TM.Box(G3, 2, 2, 2, 3)
		local soup = unweld(box, function(tid)
			return true
		end)
		local repair = G3.MeshAutoRepair.new(soup)
		h.truthy(repair:Apply())
		checkRepaired(repair.Mesh, 8, 1e-9, "box soup")
		h.eq(repair.Mesh:VertexCount(), box:VertexCount())
	end)

	h:test("duplicates + holes + cracks + flipped component", function()
		-- component 1: sphere soup (cracks everywhere) with a 3-hole and a 4-hole
		local sphere = TM.UVSphere(G3, 1, 20, 10)
		local removed = {}
		-- remove one triangle (3-hole) and one quad's two triangles (4-hole) from the middle bands
		local qa = 20 + 2 * 20 * 3 + 6 -- some band triangle pair index
		removed[qa] = true
		removed[qa + 1] = true
		removed[20 + 2 * 20 * 5 + 11] = true
		local sub = G3.DMesh3.new()
		do
			local tris = {}
			for tid in sphere:TriangleIndices() do
				if not removed[tid] then
					table.insert(tris, tid)
				end
			end
			sub = G3.DSubmesh3.QuickSubmesh(sphere, tris)
		end
		local mesh = unweld(sub)
		-- component 2: an inside-out box, far away (welded)
		local box = TM.Translate(G3, TM.Box(G3, 1, 1, 1, 2), 5, 0, 0)
		box:ReverseOrientation()
		TM.Append(G3, mesh, box)
		-- duplicate a few triangles of the soup (exact copies with their own vertices)
		local nt = mesh:TriangleCount()
		for tid = 0, 40, 10 do
			local a, b, c = mesh:GetTriVertices(tid)
			mesh:AppendTriangle(mesh:AppendVertex(a), mesh:AppendVertex(b), mesh:AppendVertex(c))
		end
		h.eq(mesh:TriangleCount(), nt + 5)

		-- MeshAutoRepair does not remove duplicate faces itself (the C# helper is unused), so do it first
		local dupes = G3.RemoveDuplicateTriangles.new(mesh)
		dupes:Apply()
		h.eq(dupes.Removed, 5)

		local repair = G3.MeshAutoRepair.new(mesh)
		h.truthy(repair:Apply())
		local out = repair.Mesh
		checkRepaired(out, nil, 0, "combined")
		local cc = G3.MeshConnectedComponents.new(out)
		cc:FindConnectedT()
		h.eq(cc:Count(), 2)
		-- volume should be (sphere approx) + box
		local vol = signedVolume(out)
		h.truthy(vol > 1 + 2.8 and vol < 1 + 4.2, "volume " .. vol)
		-- the holes were filled with minimal fills (no new interior vertices)
		h.eq(out:VertexCount(), sphere:VertexCount() + box:VertexCount())
	end)

	h:test("small gaps (cracks wider than zero) are welded", function()
		-- two halves of a box whose shared seam vertices are offset by a tiny amount
		local box = TM.Box(G3, 2, 2, 2, 2)
		local soup = unweld(box)
		for vid in soup:VertexIndices() do
			local v = soup:GetVertex(vid)
			-- jitter well below RepairTolerance*2 (= 2e-6)
			local j = 3e-7 * math.sin(vid * 1.7)
			soup:SetVertex(vid, V(v.x + j, v.y - j, v.z + j))
		end
		local repair = G3.MeshAutoRepair.new(soup)
		h.truthy(repair:Apply())
		checkRepaired(repair.Mesh, 8, 1e-4, "jittered box")
	end)

	h:test("RemoveModes Interior removes inner shell", function()
		local mesh = TM.Box(G3, 4, 4, 4, 2)
		TM.Append(G3, mesh, TM.Box(G3, 1, 1, 1, 1))
		G3.MeshNormals.QuickCompute(mesh)
		local repair = G3.MeshAutoRepair.new(mesh)
		repair.RemoveMode = G3.MeshAutoRepair.RemoveModes.Interior
		h.truthy(repair:Apply())
		checkRepaired(repair.Mesh, 64, 1e-9, "outer box")
		h.eq(repair.Mesh:TriangleCount(), 48)
	end)

	h:test("cancel", function()
		local box = TM.Box(G3, 1, 1, 1, 2)
		local repair = G3.MeshAutoRepair.new(unweld(box))
		repair.Progress = G3.ProgressCancel.new(function()
			return true
		end)
		h.eq(repair:Apply(), false)
	end)

	return h:results()
end
