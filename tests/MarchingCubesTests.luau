-- Tests for MarchingCubes, Implicit3d primitives/operators, DenseGrid3f/3i, Bitmap3, AxisAlignedBox3i, GridIndexing.

return function(G3, h)
	local Vector3d = G3.Vector3d
	local Vector3i = G3.Vector3i
	local Implicit3d = G3.Implicit3d
	local MarchingCubes = G3.MarchingCubes

	local function signedVolume(mesh)
		local vol = 0
		for tid in mesh:TriangleIndices() do
			local a, b, c = mesh:GetTriVertices(tid)
			vol += a:Dot(b:Cross(c)) / 6
		end
		return vol
	end
	h.signedVolume = signedVolume

	local function radiusRange(mesh, center)
		local rmin, rmax = math.huge, -math.huge
		for vid in mesh:VertexIndices() do
			local r = mesh:GetVertex(vid):Distance(center)
			rmin = math.min(rmin, r)
			rmax = math.max(rmax, r)
		end
		return rmin, rmax
	end

	local function runMC(implicit, cubeSize, expand, configure)
		local mc = MarchingCubes.new()
		mc.Implicit = implicit
		mc.Bounds = implicit:Bounds()
		mc.Bounds:Expand(expand)
		mc.CubeSize = cubeSize
		if configure then
			configure(mc)
		end
		mc:Generate()
		return mc.Mesh, mc
	end

	h:test("AxisAlignedBox3i basics", function()
		local B = G3.AxisAlignedBox3i
		local b = B.new(0, 0, 0, 4, 5, 6)
		h.eq(b:Width(), 4)
		h.eq(b:Volume(), 120)
		h.eq(b:Center(), Vector3i.new(2, 2, 3))
		h.eq(B.new(-3, -3, -3, 0, 0, 0):Center(), Vector3i.new(-1, -1, -1), "truncating center")
		h.truthy(b:Contains(Vector3i.new(4, 5, 6)))
		h.truthy(not b:Contains(Vector3i.new(5, 5, 6)))
		local e = B.Empty()
		e:Contain(Vector3i.new(1, 2, 3))
		e:Contain(Vector3d.new(-1.5, 2.5, 7.9))
		h.eq(e.Min, Vector3i.new(-1, 2, 3))
		h.eq(e.Max, Vector3i.new(1, 2, 7))
		local n = 0
		for _ in B.new(0, 0, 0, 1, 2, 3):IndicesInclusive() do
			n += 1
		end
		h.eq(n, 2 * 3 * 4)
		n = 0
		for _ in B.new(0, 0, 0, 1, 2, 3):IndicesExclusive() do
			n += 1
		end
		h.eq(n, 6)
		h.eq(b:ClampExclusive(Vector3i.new(9, -1, 3)), Vector3i.new(3, 0, 3))
	end)

	h:test("DenseGrid3f / DenseGrid3i / Bitmap3 basics", function()
		local g = G3.DenseGrid3f.new(3, 4, 5, 1.5)
		h.eq(g:size(), 60)
		h.eq(#g.Buffer, 60)
		g:Set(1, 2, 3, 7)
		h.eq(g:GetV(Vector3i.new(1, 2, 3)), 7)
		h.eq(g.Buffer[g:to_linear(1, 2, 3) + 1], 7, "buffer layout")
		h.eq(g:to_index(g:to_linear(1, 2, 3)), Vector3i.new(1, 2, 3))
		local a, b = g:get_x_pair(0, 2, 3)
		h.eq(a, 1.5)
		h.eq(b, 7)
		local n = 0
		for _ in g:Indices() do
			n += 1
		end
		h.eq(n, 60)
		n = 0
		for _ in g:InsetIndices(1) do
			n += 1
		end
		h.eq(n, 1 * 2 * 3)
		local slice = g:get_slice(3, 2)
		h.eq(slice:Get(1, 2), 7)
		local c = G3.DenseGrid3f.FromCopy(g)
		c:Set(0, 0, 0, -1)
		h.eq(g:Get(0, 0, 0), 1.5, "copy is independent")
		g:apply(function(v)
			return v * 2
		end)
		h.eq(g:Get(1, 2, 3), 14)
		h.eq(g:Bounds().Max, Vector3i.new(3, 4, 5))

		local gi = G3.DenseGrid3i.new(2, 2, 2, 0)
		gi:increment(1, 1, 1)
		gi:atomic_incdec(1, 1, 1, false)
		gi:atomic_incdec(0, 0, 0, true)
		h.eq(gi:Get(1, 1, 1), 2)
		h.eq(gi:Get(0, 0, 0), -1)
		local bmp = gi:get_bitmap(0)
		h.truthy(bmp:Get(Vector3i.new(1, 1, 1)))
		h.truthy(not bmp:Get(Vector3i.new(0, 0, 0)))
		n = 0
		for idx in bmp:NonZeros() do
			n += 1
			h.eq(idx, Vector3i.new(1, 1, 1))
		end
		h.eq(n, 1)
		h.eq(bmp:ToLinear(Vector3i.new(1, 1, 1)), 7)
		bmp:Filter(0)
		h.truthy(not bmp:Get(Vector3i.new(1, 1, 1)), "isolated voxel filtered")
	end)

	h:test("GridIndexing", function()
		local GI = G3.GridIndexing
		local s = GI.ShiftGridIndexer3.new(Vector3d.new(1, 1, 1), 0.5)
		h.eq(s:ToGrid(Vector3d.new(2.2, 0.9, 1)), Vector3i.new(2, 0, 0))
		h.eq(s:FromGrid(Vector3i.new(2, 0, 0)), Vector3d.new(2, 1, 1))
		local m = GI.MultigridIndexer3.new(Vector3i.new(4, 4, 4))
		h.eq(m:ToBlockIndex(Vector3i.new(5, 0, -1)), Vector3i.new(1, 0, -1))
		h.eq(m:ToBlockLocal(Vector3i.new(5, 0, -1)), Vector3i.new(1, 0, 3))
		h.eq(m:FromBlock(Vector3i.new(1, 0, -1)), Vector3i.new(4, 0, -4))
	end)

	h:test("Implicit3d primitives and operators", function()
		local s1 = Implicit3d.ImplicitSphere3d.new({ Origin = Vector3d.new(0, 0, 0), Radius = 1 })
		local s2 = Implicit3d.ImplicitSphere3d.new({ Origin = Vector3d.new(1, 0, 0), Radius = 1 })
		h.near(s1:Value(Vector3d.new(2, 0, 0)), 1)
		local u = Implicit3d.ImplicitUnion3d.new({ A = s1, B = s2 })
		h.near(u:Value(Vector3d.new(2, 0, 0)), 0)
		local i = Implicit3d.ImplicitIntersection3d.new({ A = s1, B = s2 })
		h.near(i:Value(Vector3d.new(2, 0, 0)), 1)
		local d = Implicit3d.ImplicitDifference3d.new({ A = s1, B = s2 })
		h.near(d:Value(Vector3d.new(-0.5, 0, 0)), -0.5)
		local bx = Implicit3d.ImplicitAxisAlignedBox3d.new({ AABox = G3.AxisAlignedBox3d.new(-1, -1, -1, 1, 1, 1) })
		h.near(bx:Value(Vector3d.new(0, 0, 0)), -1)
		h.near(bx:Value(Vector3d.new(3, 0, 0)), 2)
		local line =
			Implicit3d.ImplicitLine3d.new({ Segment = G3.Segment3d.new(Vector3d.new(0, 0, 0), Vector3d.new(2, 0, 0)), Radius = 0.5 })
		h.near(line:Value(Vector3d.new(1, 1, 0)), 0.5)
		h.near(line:ValueXYZ(3, 0, 0), 0.5)
		local blend = Implicit3d.ImplicitBlend3d.new({ A = s1, B = s2, WeightA = 1e-9 })
		h.eq(blend:WeightA(), 0.00001, "clamped setter")
		local nu = Implicit3d.ImplicitNaryUnion3d.new({ Children = { s1, s2 } })
		h.near(nu:Value(Vector3d.new(2, 0, 0)), 0)
		local b = nu:Bounds()
		h.near(b.Max.x, 2)
		h.near(b.Min.x, -1)
		h.near(s1:Bounds().Max.x, 1, 1e-12, "child bounds not mutated")
		local off = Implicit3d.ImplicitOffset3d.new({ A = s1, Offset = 0.5 })
		h.near(off:Value(Vector3d.new(1.5, 0, 0)), 0)
		local shell = Implicit3d.ImplicitShell3d.new({ A = s1, Inside = G3.Interval1d.new(-0.1, 0.1) })
		h.truthy(shell:Value(Vector3d.new(1, 0, 0)) < 0)
		h.truthy(shell:Value(Vector3d.new(0, 0, 0)) > 0)
		local skel = Implicit3d.DistanceFieldToSkeletalField.new({ DistanceField = s1, FalloffDistance = 1 })
		h.near(skel:Value(Vector3d.new(1, 0, 0)), Implicit3d.DistanceFieldToSkeletalField.ZeroIsocontour)
	end)

	h:test("MarchingCubes sphere is closed manifold with correct volume", function()
		local s = Implicit3d.ImplicitSphere3d.new({ Origin = Vector3d.new(0.1, 0.2, 0.3), Radius = 1 })
		local mesh = runMC(s, 0.05, 0.1)
		h.truthy(mesh:TriangleCount() > 1000)
		h.truthy(mesh:CheckValidity(false), "valid")
		h.truthy(mesh:IsClosed(), "closed")
		local vol = signedVolume(mesh)
		h.near(vol, 4 / 3 * math.pi, 0.02, "volume")
		local rmin, rmax = radiusRange(mesh, s.Origin)
		h.truthy(rmin > 0.99 and rmax < 1.001, string.format("radius range %f %f", rmin, rmax))
	end)

	h:test("MarchingCubes root modes and parallel flag", function()
		local s = Implicit3d.ImplicitSphere3d.new({ Origin = Vector3d.Zero, Radius = 1 })
		local m1 = runMC(s, 0.1, 0.1, function(mc)
			mc.ParallelCompute = true
		end)
		local m2 = runMC(s, 0.1, 0.1, function(mc)
			mc.ParallelCompute = false
		end)
		h.eq(m1:VertexCount(), m2:VertexCount())
		h.eq(m1:TriangleCount(), m2:TriangleCount())
		for vid in m1:VertexIndices() do
			h.truthy(m1:GetVertex(vid) == m2:GetVertex(vid))
		end
		-- skeletal field is not a distance field, so root finding matters
		local skel = Implicit3d.DistanceFieldToSkeletalField.new({ DistanceField = s, FalloffDistance = 0.5 })
		for _, mode in
			{
				MarchingCubes.RootfindingModes.SingleLerp,
				MarchingCubes.RootfindingModes.LerpSteps,
				MarchingCubes.RootfindingModes.Bisection,
			}
		do
			local mesh = runMC(skel, 0.1, 0.1, function(mc)
				mc.IsoValue = Implicit3d.DistanceFieldToSkeletalField.ZeroIsocontour
				mc.RootMode = mode
				mc.RootModeSteps = 8
			end)
			h.truthy(mesh:CheckValidity(false), "valid mode " .. mode)
			h.truthy(mesh:IsClosed(), "closed mode " .. mode)
			local rmin, rmax = radiusRange(mesh, Vector3d.Zero)
			local tol = (mode == MarchingCubes.RootfindingModes.SingleLerp) and 0.05 or 0.01
			h.truthy(rmin > 1 - tol and rmax < 1 + tol, string.format("mode %d radius range %f %f", mode, rmin, rmax))
		end
	end)

	h:test("MarchingCubes union of overlapping spheres, CancelF", function()
		local s1 = Implicit3d.ImplicitSphere3d.new({ Origin = Vector3d.new(0, 0, 0), Radius = 1 })
		local s2 = Implicit3d.ImplicitSphere3d.new({ Origin = Vector3d.new(1, 0, 0), Radius = 1 })
		local mesh = runMC(Implicit3d.ImplicitUnion3d.new({ A = s1, B = s2 }), 0.05, 0.1)
		h.truthy(mesh:CheckValidity(false))
		h.truthy(mesh:IsClosed())
		-- union volume = 2 * sphere - lens; lens for d=1, r=1: pi/12 * (4r + d)(2r - d)^2 = 5pi/12
		local expected = 2 * 4 / 3 * math.pi - 5 * math.pi / 12
		h.near(signedVolume(mesh), expected, 0.03)

		local calls = 0
		local cancelled = runMC(s1, 0.05, 0.1, function(mc)
			mc.CancelF = function()
				calls += 1
				return calls > 5
			end
		end)
		h.truthy(cancelled:TriangleCount() < mesh:TriangleCount() / 4, "cancel stops early")
	end)

	h:test("MarchingCubes on DenseGridTrilinearImplicit", function()
		-- sample a sphere SDF on a grid, then mesh the trilinear interpolant
		local cs = 0.1
		local origin = Vector3d.new(-1.5, -1.5, -1.5)
		local n = 31
		local grid = G3.DenseGrid3f.new(n, n, n, 0)
		for k = 0, n - 1 do
			for j = 0, n - 1 do
				for i = 0, n - 1 do
					local p = origin + Vector3d.new(i, j, k) * cs
					grid:Set(i, j, k, p:Length() - 1)
				end
			end
		end
		local imp = G3.DenseGridTrilinearImplicit.new(grid, origin, cs)
		h.near(imp:Value(Vector3d.new(0, 0, 0)), -1, 0.02)
		h.eq(imp:Value(Vector3d.new(5, 0, 0)), imp.Outside)
		local grad = imp:Gradient(Vector3d.new(0.7, 0.01, 0.01))
		h.truthy(grad.x > 0 and math.abs(grad.y) < math.abs(grad.x))
		local mesh = runMC(imp, 0.05, 0)
		h.truthy(mesh:CheckValidity(false))
		h.truthy(mesh:IsClosed())
		h.near(signedVolume(mesh), 4 / 3 * math.pi, 0.05)
	end)

	h:test("MarchingCubes performance (implicit sphere 64^3, 128^3)", function()
		local s = Implicit3d.ImplicitSphere3d.new({ Origin = Vector3d.Zero, Radius = 1 })
		local report = {}
		for _, res in { 64, 128 } do
			local mc = MarchingCubes.new()
			mc.Implicit = s
			mc.Bounds = G3.AxisAlignedBox3d.new(-1.1, -1.1, -1.1, 1.1, 1.1, 1.1)
			mc.CubeSize = 2.2 / res
			local t0 = os.clock()
			mc:Generate()
			local dt = os.clock() - t0
			table.insert(report, string.format("%d^3: %.3fs (%d tris)", mc.CellDimensions.x, dt, mc.Mesh:TriangleCount()))
		end
		print("    MarchingCubes perf: " .. table.concat(report, ", "))
	end)

	return h:results()
end
