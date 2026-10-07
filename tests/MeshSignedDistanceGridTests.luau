-- Tests for MeshSignedDistanceGrid (all compute/inside modes), CachingMeshSDF(+Implicit), and
-- MarchingCubes reconstruction of the SDF zero level set.

return function(G3, h)
	local TM = require(script.Parent.TestMeshes)
	local Vector3d = G3.Vector3d
	local MSDF = G3.MeshSignedDistanceGrid

	local function signedVolume(mesh)
		local vol = 0
		for tid in mesh:TriangleIndices() do
			local a, b, c = mesh:GetTriVertices(tid)
			vol += a:Dot(b:Cross(c)) / 6
		end
		return vol
	end

	local function radiusRange(mesh, center)
		local rmin, rmax = math.huge, -math.huge
		for vid in mesh:VertexIndices() do
			local r = mesh:GetVertex(vid):Distance(center)
			rmin = math.min(rmin, r)
			rmax = math.max(rmax, r)
		end
		return rmin, rmax
	end

	local function meshImplicit(implicit, cubeSize, iso)
		local mc = G3.MarchingCubes.new()
		mc.Implicit = implicit
		mc.Bounds = implicit:Bounds()
		mc.CubeSize = cubeSize
		mc.IsoValue = iso or 0
		mc:Generate()
		return mc.Mesh
	end

	local sphere = TM.UVSphere(G3, 1, 48, 24)
	local sphereVolume = signedVolume(sphere)

	-- check SDF values against the analytic sphere distance within the band
	local function checkSphereSDF(sdf, band, tol)
		local grid = sdf:Grid()
		local o = sdf:GridOrigin()
		local checked = 0
		for k = 0, grid.nk - 1 do
			for j = 0, grid.nj - 1 do
				for i = 0, grid.ni - 1 do
					local p = Vector3d.new(i * sdf.CellSize + o.x, j * sdf.CellSize + o.y, k * sdf.CellSize + o.z)
					local exact = p:Length() - 1
					local v = grid:Get(i, j, k)
					if math.abs(exact) < band then
						checked += 1
						if math.abs(v - exact) > tol then
							error(string.format("sdf mismatch at %d %d %d: %f vs %f", i, j, k, v, exact))
						end
					end
					-- signs everywhere (away from the surface)
					if exact < -2 * tol then
						h.truthy(v < 0, "inside negative")
					elseif exact > 2 * tol then
						h.truthy(v > 0, "outside positive")
					end
				end
			end
		end
		h.truthy(checked > 100, "checked cells")
	end

	h:test("narrow band SDF of sphere (no spatial), MC reproduces sphere", function()
		local sdf = MSDF.new(sphere, 0.05)
		sdf:Compute()
		local dims = sdf:Dimensions()
		h.eq(dims.x, 45) -- (2.2 + 4*0.05) / 0.05 + 1 (bounds slightly smaller than 2 due to tessellation)
		checkSphereSDF(sdf, 0.05, 0.01)
		local imp = G3.DenseGridTrilinearImplicit.FromSDFGrid(sdf)
		local mesh = meshImplicit(imp, 0.05)
		h.truthy(mesh:CheckValidity(false), "valid")
		h.truthy(mesh:IsClosed(), "closed")
		h.near(signedVolume(mesh), sphereVolume, 0.03, "volume")
		local rmin, rmax = radiusRange(mesh, Vector3d.Zero)
		h.truthy(rmin > 0.98 and rmax < 1.01, string.format("radius %f %f", rmin, rmax))
		h.truthy(not pcall(function()
			return sdf:ClosestTriGrid()
		end), "ClosestTriGrid requires WantClosestTriGrid")
	end)

	h:test("serial path, CrossingCount, intersections and closest-tri grids", function()
		local a = MSDF.new(sphere, 0.1)
		a:Compute()
		local b = MSDF.new(sphere, 0.1)
		b.UseParallel = false
		b.InsideMode = MSDF.InsideModes.CrossingCount
		b.WantClosestTriGrid = true
		b.WantIntersectionsGrid = true
		b:Compute()
		local ga, gb = a:Grid(), b:Grid()
		h.eq(#ga.Buffer, #gb.Buffer)
		-- serial path divides by dx, parallel path multiplies by 1/dx, so the band boxes can differ by a
		-- cell at the band boundary; signs must agree everywhere and values wherever both were computed
		local ub = a:upper_bound(ga)
		local same = 0
		for i = 1, #ga.Buffer do
			local va, vb = ga.Buffer[i], gb.Buffer[i]
			h.truthy((va < 0) == (vb < 0), "same sign")
			if math.abs(va) < ub and math.abs(vb) < ub then
				h.near(va, vb, 1e-9)
				same += 1
			end
		end
		h.truthy(same > 1000)
		local ct = b:ClosestTriGrid()
		local c = ct:Get(10, 10, 2)
		h.truthy(c >= 0 and sphere:IsTriangle(c), "closest tri near surface")
		local ig = b:IntersectionsGrid()
		local total = 0
		for _, v in ig.Buffer do
			total += v
		end
		h.truthy(total > 0 and total % 2 == 0, "even total crossing count")
	end)

	h:test("full grid SDF (sweeping) gives distances far from the surface", function()
		local sdf = MSDF.new(sphere, 0.1)
		sdf.ComputeMode = MSDF.ComputeModes.FullGrid
		sdf.ExpandBounds = Vector3d.new(0.5, 0.5, 0.5)
		sdf:Compute()
		checkSphereSDF(sdf, 10, 0.02)
	end)

	h:test("spatial and flood-fill modes with DMeshAABBTree3", function()
		local tree = G3.DMeshAABBTree3.new(sphere, true)
		local a = MSDF.new(sphere, 0.05, tree)
		a:Compute()
		checkSphereSDF(a, 0.05, 0.01)

		local f = MSDF.new(sphere, 0.05, tree)
		f.ComputeMode = MSDF.ComputeModes.NarrowBand_SpatialFloodFill
		f.NarrowBandMaxDistance = 0.2
		f:Compute()
		checkSphereSDF(f, 0.2, 0.01)
		local mesh = meshImplicit(G3.DenseGridTrilinearImplicit.FromSDFGrid(f), 0.05)
		h.truthy(mesh:CheckValidity(false) and mesh:IsClosed())
		h.near(signedVolume(mesh), sphereVolume, 0.03)

		local bad = MSDF.new(sphere, 0.05)
		bad.ComputeMode = MSDF.ComputeModes.NarrowBand_SpatialFloodFill
		h.truthy(not pcall(function()
			bad:Compute()
		end), "flood fill requires spatial + band distance")
	end)

	h:test("SDF of two overlapping spheres (ParityCount union) via MarchingCubes", function()
		local m = TM.UVSphere(G3, 1, 32, 16)
		TM.Append(G3, m, TM.UVSphere(G3, 1, 32, 16, Vector3d.new(1, 0, 0)))
		local sdf = MSDF.new(m, 0.05)
		sdf:Compute()
		local mesh = meshImplicit(G3.DenseGridTrilinearImplicit.FromSDFGrid(sdf), 0.05)
		h.truthy(mesh:CheckValidity(false), "valid")
		h.truthy(mesh:IsClosed(), "closed")
		-- analytic union volume, scaled by the volume ratio of the tessellated sphere
		local ratio = signedVolume(TM.UVSphere(G3, 1, 32, 16)) / (4 / 3 * math.pi)
		local expected = (2 * 4 / 3 * math.pi - 5 * math.pi / 12) * ratio
		h.near(signedVolume(mesh), expected, 0.05)
	end)

	h:test("CachingMeshSDF lazy evaluation + MarchingCubes", function()
		local tree = G3.DMeshAABBTree3.new(sphere, true)
		local sdf = G3.CachingMeshSDF.new(sphere, 0.05, tree)
		sdf.WantClosestTriGrid = true
		sdf:Initialize()
		local grid = sdf:Grid()
		local ub = sdf.UpperBoundDistance
		local uncomputed = 0
		for _, v in grid.Buffer do
			if math.abs(v) == ub then
				uncomputed += 1
			end
		end
		h.eq(uncomputed, #grid.Buffer, "nothing computed yet")
		local imp = G3.CachingMeshSDFImplicit.new(sdf)
		local mesh = meshImplicit(imp, 0.05)
		h.truthy(mesh:CheckValidity(false), "valid")
		h.truthy(mesh:IsClosed(), "closed")
		h.near(signedVolume(mesh), sphereVolume, 0.03)
		local c = sdf:GetValue(G3.Vector3i.new(5, 5, 5))
		h.truthy(c > 0)
		uncomputed = 0
		for _, v in grid.Buffer do
			if math.abs(v) == ub then
				uncomputed += 1
			end
		end
		h.truthy(uncomputed < #grid.Buffer, "values cached")
	end)

	h:test("point_triangle_distance / point_in_triangle_2d helpers", function()
		local d =
			MSDF.point_triangle_distance(Vector3d.new(0.25, 0.25, 1), Vector3d.new(0, 0, 0), Vector3d.new(1, 0, 0), Vector3d.new(0, 1, 0))
		h.near(d, 1)
		d = MSDF.point_triangle_distance(Vector3d.new(2, 0, 0), Vector3d.new(0, 0, 0), Vector3d.new(1, 0, 0), Vector3d.new(0, 1, 0))
		h.near(d, 1)
		local ok, a, b, c = MSDF.point_in_triangle_2d(0.25, 0.25, 0, 0, 1, 0, 0, 1)
		h.truthy(ok)
		h.near(a + b + c, 1)
		h.truthy(not (MSDF.point_in_triangle_2d(2, 2, 0, 0, 1, 0, 0, 1)))
	end)

	return h:results()
end
