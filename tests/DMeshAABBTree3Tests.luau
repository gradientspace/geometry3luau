-- Tests for DMeshAABBTree3 (nearest triangle / vertex, ray hits, winding numbers, intersection tests)

return function(G3, h)
	local TestMeshes = require(script.Parent.TestMeshes)
	local Vector3d = G3.Vector3d
	local DMeshAABBTree3 = G3.DMeshAABBTree3
	local MeshQueries = G3.MeshQueries
	local Ray3d = G3.Ray3d
	local V = Vector3d.new

	-- deterministic pseudo-random numbers
	local seed = 12345
	local function rand()
		seed = (seed * 1103515245 + 12345) % 2147483648
		return seed / 2147483648
	end
	local function randPoint(r)
		return V((rand() * 2 - 1) * r, (rand() * 2 - 1) * r, (rand() * 2 - 1) * r)
	end

	local function checkNearest(mesh, tree, npts, r)
		for _ = 1, npts do
			local p = randPoint(r)
			local tid, dsqr = tree:FindNearestTriangleDistSqr(p)
			local tbf = MeshQueries.FindNearestTriangle_LinearSearch(mesh, p)
			local dbf = MeshQueries.TriDistanceSqr(mesh, tbf, p)
			h.truthy(tid >= 0, "found triangle")
			h.near(dsqr, dbf, 1e-12, "nearest dist matches brute force")
			h.near(MeshQueries.TriDistanceSqr(mesh, tid, p), dbf, 1e-12)
			h.eq(tree:FindNearestTriangle(p), tid)
		end
	end

	h:test("nearest triangle vs brute force (sphere, box)", function()
		local sphere = TestMeshes.UVSphere(G3, 1.0, 24, 12)
		local tree = DMeshAABBTree3.new(sphere, true)
		tree:TestCoverage()
		checkNearest(sphere, tree, 200, 2.0)

		local box = TestMeshes.Box(G3, 2, 1, 3, 4)
		local btree = DMeshAABBTree3.new(box)
		for _, strategy in
			{
				DMeshAABBTree3.BuildStrategy.TopDownMidpoint,
				DMeshAABBTree3.BuildStrategy.TopDownMedian,
				DMeshAABBTree3.BuildStrategy.BottomUpFromOneRings,
			}
		do
			btree:Build(strategy)
			btree:TestCoverage()
			checkNearest(box, btree, 100, 3.0)
		end
		for _, policy in
			{
				DMeshAABBTree3.ClusterPolicy.Fastest,
				DMeshAABBTree3.ClusterPolicy.MinimalVolume,
				DMeshAABBTree3.ClusterPolicy.FastVolumeMetric,
			}
		do
			btree:Build(DMeshAABBTree3.BuildStrategy.BottomUpFromOneRings, policy)
			btree:TestCoverage()
			checkNearest(box, btree, 50, 3.0)
		end

		-- max distance
		local far = V(10, 0, 0)
		h.eq(tree:FindNearestTriangle(far, 1.0), -1)
		h.truthy(tree:FindNearestTriangle(far, 9.5) >= 0)
	end)

	h:test("nearest vertex vs brute force", function()
		local sphere = TestMeshes.UVSphere(G3, 1.0, 16, 8)
		local tree = DMeshAABBTree3.new(sphere, true)
		for _ = 1, 100 do
			local p = randPoint(1.5)
			local vid = tree:FindNearestVertex(p)
			local vbf = MeshQueries.FindNearestVertex_LinearSearch(sphere, p)
			h.near(sphere:GetVertex(vid):DistanceSquared(p), sphere:GetVertex(vbf):DistanceSquared(p), 1e-12)
		end
	end)

	h:test("ray hits", function()
		local sphere = TestMeshes.UVSphere(G3, 1.0, 24, 12)
		local tree = DMeshAABBTree3.new(sphere, true)
		local ray = Ray3d.new(V(-5, 0.1, 0.05), V(1, 0, 0))
		local tid = tree:FindNearestHitTriangle(ray)
		h.truthy(tid >= 0)
		h.eq(tid, MeshQueries.FindHitTriangle_LinearSearch(sphere, ray))
		local isect = MeshQueries.TriangleIntersection(sphere, tid, ray)
		h.truthy(isect.RayParameter > 3.9 and isect.RayParameter < 4.05)
		local hits = {}
		h.eq(tree:FindAllHitTriangles(ray, hits), 2)
		h.eq(#hits, 2)
		-- miss
		h.eq(tree:FindNearestHitTriangle(Ray3d.new(V(-5, 3, 0), V(1, 0, 0))), -1)
		-- max distance
		h.eq(tree:FindNearestHitTriangle(ray, 3.0), -1)
		-- random rays vs brute force
		for _ = 1, 100 do
			local o = randPoint(3)
			local d = (randPoint(1) - o * 0.2):Normalized()
			local r = Ray3d.new(o, d)
			local t1 = tree:FindNearestHitTriangle(r)
			local t2 = MeshQueries.FindHitTriangle_LinearSearch(sphere, r)
			if t1 ~= t2 then
				-- allow ties at shared edges
				local i1, i2 = MeshQueries.TriangleIntersection(sphere, t1, r), MeshQueries.TriangleIntersection(sphere, t2, r)
				h.near(i1.RayParameter, i2.RayParameter, 1e-9)
			end
		end
		-- frames
		local ok, frame = MeshQueries.RayHitPointFrame(sphere, tree, ray)
		h.truthy(ok)
		h.near(frame.Origin:Length(), 1.0, 0.02)
	end)

	h:test("winding numbers and IsInside", function()
		local sphere = TestMeshes.UVSphere(G3, 1.0, 24, 12)
		local tree = DMeshAABBTree3.new(sphere, true)
		for _ = 1, 200 do
			local p = randPoint(2)
			local w = sphere:WindingNumber(p)
			local inside = w > 0.5
			h.near(tree:WindingNumber(p), w, 1e-8, "hierarchical winding number")
			local fw = tree:FastWindingNumber(p)
			-- (FWN far-field approximation error with beta=2 is a few percent near clusters)
			h.near(fw, w, 0.08, "fast winding number")
			if math.abs(w - 0.5) > 0.2 then
				h.eq(fw > 0.5, inside, "FWN classification")
			end
			if math.abs(p:Length() - 1) > 0.05 then
				h.eq(tree:IsInside(p), inside, "IsInside")
			end
		end
		h.near(tree:FastWindingNumber(Vector3d.Zero), 1, 0.08)
		h.near(tree:FastWindingNumber(V(5, 5, 5)), 0, 1e-3)

		-- compare the inlined tree evaluation against a straightforward evaluation using the
		-- vector-based FastTriWinding API on the same cache structure (C# branch_fast_winding_num)
		local FastTriWinding = G3.FastTriWinding
		local function ref_eval(iBox, p)
			local idx = tree.box_to_index[iBox + 1]
			local sum = 0
			if idx < tree.triangles_end then
				for i = 1, tree.index_list[idx + 1] do
					local a, b, c = sphere:GetTriVertices(tree.index_list[idx + i + 1])
					sum += G3.MathUtil.TriSolidAngle(a, b, c, p) / G3.MathUtil.FourPI
				end
				return sum
			end
			local c1 = tree.index_list[idx + 1]
			local children = (c1 < 0) and { -c1 - 1 } or { c1 - 1, tree.index_list[idx + 2] - 1 }
			for _, ci in children do
				if not tree:get_box(ci):Contains(p) and tree.FastWindingCache.has[ci + 1] then
					local tris = DMeshAABBTree3.oset_new()
					tree:collect_triangles(ci, tris)
					local center, r, o1, o2 = FastTriWinding.ComputeCoeffs(sphere, tris.list, G3.MeshTriInfoCache.new(sphere))
					if center:Distance(p) > tree.FWNBeta * r then
						sum += FastTriWinding.EvaluateOrder2Approx(center, o1, o2, p)
						continue
					end
				end
				sum += ref_eval(ci, p)
			end
			return sum
		end
		for _ = 1, 20 do
			local p = randPoint(2)
			h.near(tree:FastWindingNumber(p), ref_eval(tree.root_index, p), 1e-12, "FWN vs reference evaluation")
		end
		-- order-1 approximation
		tree.FWNApproxOrder = 1
		h.near(tree:FastWindingNumber(V(0.1, 0.2, 0.1)), 1, 0.05)
		h.near(tree:FastWindingNumberXYZ(3, 0, 0), 0, 0.05)
	end)

	h:test("fast winding number on torus and overlapping components", function()
		local torus = TestMeshes.Torus(G3, 2.0, 0.6, 48, 24)
		local tree = DMeshAABBTree3.new(torus, true)
		for _, q in
			{ { V(2, 0, 0), 1 }, { V(0, 0, -2), 1 }, { V(0, 0, 0), 0 }, { V(0, 1, 0), 0 }, { V(4, 0, 0), 0 }, { V(1.5, 0.2, 1.0), 1 } }
		do
			local exact = torus:WindingNumber(q[1])
			h.near(exact, q[2], 1e-6)
			h.near(tree:WindingNumber(q[1]), exact, 1e-8)
			h.near(tree:FastWindingNumber(q[1]), q[2], 0.05)
		end
		for _ = 1, 100 do
			local p = randPoint(3)
			h.near(tree:FastWindingNumber(p), torus:WindingNumber(p), 0.08)
		end

		-- two overlapping spheres: winding number 2 in overlap region
		local a = TestMeshes.UVSphere(G3, 1.0, 24, 12)
		local b = TestMeshes.UVSphere(G3, 1.0, 24, 12, V(0.8, 0, 0))
		TestMeshes.Append(G3, a, b)
		local t2 = DMeshAABBTree3.new(a, true)
		h.near(t2:FastWindingNumber(V(0.4, 0, 0)), 2, 0.05)
		h.near(t2:FastWindingNumber(V(-0.5, 0, 0)), 1, 0.05)
		h.near(t2:FastWindingNumber(V(1.5, 0, 0)), 1, 0.05)
		h.near(t2:FastWindingNumber(V(0, 3, 0)), 0, 0.05)
		h.near(t2:WindingNumber(V(0.4, 0, 0)), 2, 1e-6)
	end)

	h:test("modified mesh is detected", function()
		local mesh = TestMeshes.Octahedron(G3)
		local tree = DMeshAABBTree3.new(mesh, true)
		h.truthy(tree:IsValid())
		mesh:SetVertex(0, V(1.1, 0, 0))
		h.truthy(not tree:IsValid())
		h.truthy(not pcall(function()
			tree:FindNearestTriangle(Vector3d.Zero)
		end))
		tree:Build()
		h.truthy(tree:FindNearestTriangle(Vector3d.Zero) >= 0)
		-- filter
		tree.TriangleFilterF = function(tid)
			return tid == 3
		end
		h.eq(tree:FindNearestTriangle(V(0, 0, -5)), 3)
	end)

	h:test("mesh/mesh intersection and nearest triangle pairs", function()
		local a = TestMeshes.UVSphere(G3, 1.0, 16, 8)
		local b = TestMeshes.UVSphere(G3, 1.0, 16, 8, V(1.5, 0, 0))
		local c = TestMeshes.UVSphere(G3, 0.5, 12, 6, V(4, 0, 0))
		local ta, tb, tc = DMeshAABBTree3.new(a, true), DMeshAABBTree3.new(b, true), DMeshAABBTree3.new(c, true)
		h.truthy(ta:TestIntersection(tb), "a/b intersect")
		h.truthy(not ta:TestIntersection(tc), "a/c do not intersect")
		h.truthy(ta:TestIntersection(b), "mesh overload")
		h.truthy(not ta:TestIntersection(c), "mesh overload, no hit")
		-- transformed: move c onto a
		h.truthy(ta:TestIntersection(tc, function(v)
			return v - V(3.5, 0, 0)
		end))
		-- a triangle crossing the sphere surface
		local tri = G3.Triangle3d.new(V(0, 0, 0), V(2, 0.1, 0), V(0, 0.1, 2))
		h.truthy(ta:TestIntersection(tri))
		h.truthy(not ta:TestIntersection(G3.Triangle3d.new(V(3, 0, 0), V(4, 0, 0), V(3, 1, 0))))

		-- all intersections: the intersection curve of two spheres is a circle
		local result = ta:FindAllIntersections(tb)
		h.truthy(#result.Segments > 10, "found intersection segments")
		for _, seg in result.Segments do
			-- points on both spheres (approximately; meshes are coarse)
			h.near(seg.point0.x, 0.75, 0.15)
			h.truthy(a:IsTriangle(seg.t0) and b:IsTriangle(seg.t1))
		end
		-- brute force check of any-intersection
		local pair = MeshQueries.FindIntersectingTriangles_LinearSearch(a, b)
		h.truthy(pair[1] >= 0)

		-- nearest pair of triangles between a and c
		local np, dist = ta:FindNearestTriangles(tc, nil)
		h.truthy(np[1] >= 0 and np[2] >= 0)
		local bfPair, bfDist = MeshQueries.FindNearestTriangles_LinearSearch(a, c)
		h.near(dist, bfDist, 1e-9)
		h.near(MeshQueries.TrianglesDistance(a, np[1], c, np[2]):Get(), dist, 1e-9)
		h.near(dist, 2.5, 0.05)
		local np2, dist2 = ta:FindNearestTriangles(tc, nil, 1.0)
		h.eq(np2[1], G3.MathUtil.MaxInt)
		h.eq(dist2, 1.0) -- like C#: sqrt(max_dist^2) when nothing found
	end)

	h:test("traversal, volume, bounds", function()
		local box = TestMeshes.Box(G3, 2, 2, 2, 3)
		local tree = DMeshAABBTree3.new(box, true)
		local b = tree:Bounds()
		h.near(b.Min.x, -1, 1e-4)
		h.near(b.Max.z, 1, 1e-4)
		local ntri = 0
		local t = DMeshAABBTree3.TreeTraversal.new(nil, function(tid)
			ntri += 1
		end)
		tree:DoTraversal(t)
		h.eq(ntri, box:TriangleCount())
		h.truthy(tree:TotalVolume() > 8)
		h.truthy(tree:TotalExtentSum() > 0)
		h.truthy(tree:IsInside(V(0.1, 0.2, 0.3)))
		h.truthy(not tree:IsInside(V(1.1, 0.2, 0.3)))
		-- ISpatial checks
		h.truthy(G3.SpatialInterfaces.IsSpatial(tree))
		h.near(MeshQueries.NearestPointDistance(box, tree, V(3, 0, 0)), 2, 1e-12)
		local f = MeshQueries.NearestPointFrame(box, tree, V(3, 0.1, 0.2), true)
		h.near(f.Origin.x, 1, 1e-12)
		h.near(f:Z().x, 1, 1e-12)
		-- intersection target / normal offset
		local target = G3.DMeshIntersectionTarget.new(box, tree)
		local ok, hit, n = target:RayIntersect(Ray3d.new(V(-5, 0.1, 0.2), V(1, 0, 0)))
		h.truthy(ok)
		h.near(hit.x, -1, 1e-12)
		h.near(n.x, -1, 1e-12)
		local off = G3.SpatialFunctions.NormalOffset.new(box, tree)
		off.Distance = 0.5
		h.near(off:FindNearestAndOffset(V(3, 0, 0)).x, 1.5, 1e-12)
	end)

	return h:results()
end
