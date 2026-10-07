-- End-to-end tests of the "make solid" pipeline (winding grid -> marching cubes -> remesh -> reduce).

return function(G3, h)
	local TM = require(script.Parent.TestMeshes)
	local MakeSolid = G3.MakeSolid

	local function signedVolume(mesh)
		local vol = 0
		for tid in mesh:TriangleIndices() do
			local a, b, c = mesh:GetTriVertices(tid)
			vol += a:Dot(b:Cross(c)) / 6
		end
		return vol
	end

	local function checkSolid(mesh, name)
		h.truthy(mesh:TriangleCount() > 0, name .. ": empty")
		h.truthy(mesh:CheckValidity(false), name .. ": valid")
		h.truthy(mesh:IsClosed(), name .. ": closed")
		h.truthy(mesh:IsCompact(), name .. ": compact")
		h.truthy(signedVolume(mesh) > 0, name .. ": outward")
	end

	h:test("overlapping spheres become one solid (winding number)", function()
		local mesh = TM.UVSphere(G3, 1, 32, 16)
		TM.Append(G3, mesh, TM.UVSphere(G3, 1, 32, 16, G3.Vector3d.new(1, 0, 0)))
		local solid = MakeSolid.new(mesh)
		solid.GridResolution = 48
		solid.TargetTriangleCount = 1500
		local result = solid:Compute()
		checkSolid(result, "union")
		h.truthy(result:TriangleCount() <= 1500, "reduced")
		-- one connected component
		local comps = G3.MeshConnectedComponents.new(result)
		comps:FindConnectedT()
		h.eq(comps:Count(), 1, "components")
		-- union volume of two unit spheres at distance 1: 2 * 4/3 pi - lens (5/12 pi)
		local expected = 2 * (4 / 3) * math.pi - (5 / 12) * math.pi
		local vol = signedVolume(result)
		h.truthy(math.abs(vol - expected) / expected < 0.06, "union volume " .. vol .. " vs " .. expected)
		-- surface mesh (pre-remesh) is also closed
		checkSolid(solid.SurfaceMesh, "surface")
	end)

	h:test("open mesh with a hole is closed by winding number", function()
		local mesh = TM.UVSphere(G3, 1, 24, 12)
		-- cut a hole: remove a few triangles near the top
		local removed = 0
		for tid in mesh:TriangleIndices() do
			local c = mesh:GetTriCentroid(tid)
			if c.y > 0.9 then
				mesh:RemoveTriangle(tid, true, false)
				removed += 1
			end
		end
		h.truthy(removed > 0 and not mesh:IsClosed(), "hole cut")
		local result = MakeSolid.Run(mesh, { GridResolution = 32, RemeshPasses = 5 })
		checkSolid(result, "filled")
		local vol = signedVolume(result)
		h.truthy(math.abs(vol - 4.18) < 0.4, "volume " .. vol)
	end)

	h:test("signed distance mode with offset", function()
		local mesh = TM.Box(G3, 2, 2, 2, 2)
		local result = MakeSolid.Run(mesh, {
			SolidMode = MakeSolid.SolidModes.SignedDistance,
			GridResolution = 32,
			SignedDistanceOffset = 0.2,
			Remesh = false,
		})
		checkSolid(result, "offset box")
		local b = result:GetBounds()
		h.near(b:Width(), 2.4, 0.08, "offset width")
	end)

	h:test("cancel returns nil", function()
		local mesh = TM.UVSphere(G3, 1, 16, 8)
		local solid = MakeSolid.new(mesh)
		solid.Progress = function()
			return true
		end
		h.eq(solid:Compute(), nil)
		h.truthy(solid.Cancelled)
	end)

	return h:results()
end
