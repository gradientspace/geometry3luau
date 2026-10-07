-- Tests for MeshWindingNumberGrid and MeshScalarSamplingGrid, and the "make solid" pipeline:
-- DMeshAABBTree3 -> MeshWindingNumberGrid -> DenseGridTrilinearImplicit -> MarchingCubes (iso 0.5).

return function(G3, h)
	local TM = require(script.Parent.TestMeshes)
	local Vector3d = G3.Vector3d
	local MWN = G3.MeshWindingNumberGrid

	local function signedVolume(mesh)
		local vol = 0
		for tid in mesh:TriangleIndices() do
			local a, b, c = mesh:GetTriVertices(tid)
			vol += a:Dot(b:Cross(c)) / 6
		end
		return vol
	end

	-- the "make solid" pipeline: returns the (outward-facing) MC mesh and the winding grid
	local function makeSolid(mesh, cellSize, configure)
		local tree = G3.DMeshAABBTree3.new(mesh, true)
		local mwn = MWN.new(mesh, tree, cellSize)
		if configure then
			configure(mwn)
		end
		mwn:Compute()
		local iso = G3.DenseGridTrilinearImplicit.new(mwn:Grid(), mwn:GridOrigin(), mwn.CellSize)
		iso.Outside = 0 -- winding number is 0 outside the grid (default Outside is a huge positive value)
		local mc = G3.MarchingCubes.new()
		mc.Implicit = iso
		mc.IsoValue = mwn.WindingIsoValue
		mc.Bounds = iso:Bounds()
		mc.CubeSize = cellSize
		mc:Generate()
		-- winding number is ~1 inside: MC on (w - 0.5) yields an inward-facing surface, flip it
		mc.Mesh:ReverseOrientation()
		return mc.Mesh, mwn
	end

	local function checkSolid(result, expectedVolume, tol, name)
		h.truthy(result:CheckValidity(false), name .. " valid")
		h.truthy(result:IsClosed(), name .. " closed")
		local vol = signedVolume(result)
		h.near(vol, expectedVolume, tol, name .. " volume")
	end

	local sphereRatio = signedVolume(TM.UVSphere(G3, 1, 32, 16)) / (4 / 3 * math.pi)

	h:test("closed sphere: narrow band winding grid -> MarchingCubes", function()
		local sphere = TM.UVSphere(G3, 1, 32, 16)
		local result, mwn = makeSolid(sphere, 0.07)
		checkSolid(result, signedVolume(sphere), 0.03, "sphere")
		-- grid values: ~1 inside, ~0 outside (spans filled), SDF grid kept
		local g = mwn:Grid()
		local dims = mwn:Dimensions()
		h.near(g:Get(dims.x // 2, dims.y // 2, dims.z // 2), 1, 1e-3, "center inside")
		h.near(g:Get(0, 0, 0), 0, 1e-3, "corner outside")
		for _, v in g.Buffer do
			h.truthy(v ~= G3.MathUtil.MaxFloat, "all cells filled")
		end
		h.truthy(mwn:SDFGrid() ~= nil)
	end)

	h:test("two overlapping spheres -> union solid", function()
		local m = TM.UVSphere(G3, 1, 32, 16)
		TM.Append(G3, m, TM.UVSphere(G3, 1, 32, 16, Vector3d.new(1, 0.2, 0)))
		local result = makeSolid(m, 0.07)
		-- lens volume for centers distance d and radius 1: pi/12 (4 + d)(2 - d)^2
		local d = math.sqrt(1.04)
		local expected = (2 * 4 / 3 * math.pi - math.pi / 12 * (4 + d) * (2 - d) ^ 2) * sphereRatio
		checkSolid(result, expected, 0.06, "sphere union")
	end)

	h:test("two overlapping boxes -> union solid (FastWindingNumber)", function()
		-- (shifted so the box faces do not lie exactly on grid points, where the winding number is ill-defined)
		local m = TM.Box(G3, 2, 2, 2, 2, Vector3d.new(0.013, 0.021, 0.007))
		TM.Append(G3, m, TM.Box(G3, 2, 2, 2, 2, Vector3d.new(1.013, 1.021, 1.007)))
		local result = makeSolid(m, 0.05, function(mwn)
			mwn.UseFastWindingNumber = true
		end)
		-- union of two 2x2x2 cubes overlapping in a 1x1x1 cube; MC rounds the 90-degree edges/corners
		checkSolid(result, 8 + 8 - 1, 0.2, "box union")
	end)

	h:test("open mesh (sphere with a hole) still produces a closed solid", function()
		local sphere = TM.UVSphere(G3, 1, 32, 16)
		-- remove the top cap fan
		local removed = 0
		for tid in G3.Util.ArrayItr(G3.Util.ToArray(sphere:TriangleIndices())) do
			local a, b, c = sphere:GetTriVertices(tid)
			if a.y > 0.95 and b.y > 0.95 and c.y > 0.95 then
				sphere:RemoveTriangle(tid)
				removed += 1
			end
		end
		h.truthy(removed > 0 and not sphere:IsClosed())
		local result = makeSolid(sphere, 0.07)
		h.truthy(result:CheckValidity(false), "valid")
		h.truthy(result:IsClosed(), "closed")
		h.near(signedVolume(result), 4 / 3 * math.pi * sphereRatio, 0.1, "volume")
	end)

	h:test("FullGrid mode agrees with NarrowBand inside/outside classification", function()
		local sphere = TM.UVSphere(G3, 1, 16, 8)
		local tree = G3.DMeshAABBTree3.new(sphere, true)
		local a = MWN.new(sphere, tree, 0.15)
		a:Compute()
		local b = MWN.new(sphere, tree, 0.15)
		b.ComputeMode = MWN.ComputeModes.FullGrid
		b:Compute()
		local ga, gb = a:Grid(), b:Grid()
		h.eq(#ga.Buffer, #gb.Buffer)
		for i = 1, #ga.Buffer do
			h.eq(ga.Buffer[i] > 0.5, gb.Buffer[i] > 0.5, "classification " .. i)
		end
		local cancelled = MWN.new(sphere, tree, 0.15)
		cancelled.CancelF = function()
			return true
		end
		cancelled:Compute()
	end)

	h:test("MeshScalarSamplingGrid with a winding-number scalar function", function()
		local sphere = TM.UVSphere(G3, 1, 32, 16)
		local tree = G3.DMeshAABBTree3.new(sphere, true)
		local sg = G3.MeshScalarSamplingGrid.new(sphere, 0.07, function(p)
			return tree:WindingNumber(p)
		end)
		sg:Compute()
		local iso = G3.DenseGridTrilinearImplicit.FromSDFGrid(sg)
		iso.Outside = 0
		local mc = G3.MarchingCubes.new()
		mc.Implicit = iso
		mc.IsoValue = sg.IsoValue
		mc.Bounds = iso:Bounds()
		mc.CubeSize = 0.07
		mc:Generate()
		mc.Mesh:ReverseOrientation()
		checkSolid(mc.Mesh, signedVolume(sphere), 0.03, "scalar sampling")

		local dense = G3.MeshScalarSamplingGrid.new(sphere, 0.2, function(p)
			return p:Length()
		end)
		dense.ComputeMode = G3.MeshScalarSamplingGrid.ComputeModes.FullGrid
		dense:Compute()
		local p = dense:CellCenter(3, 4, 5)
		h.near(dense:Get(3, 4, 5), p:Length(), 1e-12)
	end)

	h:test("performance: winding grid for ~10k triangle sphere", function()
		-- (kept small for the interpreted test run; see the final report for 100-cells-across numbers)
		local sphere = TM.UVSphere(G3, 1, 100, 50) -- 9800 triangles
		local t0 = os.clock()
		local tree = G3.DMeshAABBTree3.new(sphere, true)
		local tTree = os.clock() - t0
		local report = {}
		for _, fast in { true, false } do
			local mwn = MWN.new(sphere, tree, 0.08)
			mwn.UseFastWindingNumber = fast
			t0 = os.clock()
			mwn:Compute()
			local tGrid = os.clock() - t0
			local iso = G3.DenseGridTrilinearImplicit.new(mwn:Grid(), mwn:GridOrigin(), mwn.CellSize)
			iso.Outside = 0
			local mc = G3.MarchingCubes.new()
			mc.Implicit = iso
			mc.IsoValue = 0.5
			mc.Bounds = iso:Bounds()
			mc.CubeSize = mwn.CellSize
			t0 = os.clock()
			mc:Generate()
			local tMC = os.clock() - t0
			local d = mwn:Dimensions()
			table.insert(
				report,
				string.format(
					"%s: grid %dx%dx%d %.2fs, MC %.2fs (%d tris)",
					fast and "fast" or "exact",
					d.x,
					d.y,
					d.z,
					tGrid,
					tMC,
					mc.Mesh:TriangleCount()
				)
			)
			h.truthy(mc.Mesh:IsClosed())
		end
		print(string.format("    MWN perf (%d tris, tree %.2fs): %s", sphere:TriangleCount(), tTree, table.concat(report, "; ")))
	end)

	return h:results()
end
