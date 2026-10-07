-- Tests for distance / intersection primitives, Box3d, Plane3d, BoundsUtil, MeshMeasurements

return function(G3, h)
	local TestMeshes = require(script.Parent.TestMeshes)
	local Vector3d = G3.Vector3d
	local Vector2d = G3.Vector2d
	local Triangle3d = G3.Triangle3d
	local V = Vector3d.new
	local V2 = Vector2d.new
	local IntersectionResult = G3.IntersectionResult
	local IntersectionType = G3.IntersectionType

	local seed = 777
	local function rand()
		seed = (seed * 1103515245 + 12345) % 2147483648
		return seed / 2147483648
	end
	local function randPoint(r)
		return V((rand() * 2 - 1) * r, (rand() * 2 - 1) * r, (rand() * 2 - 1) * r)
	end
	-- brute-force squared distance between two triangles via dense barycentric sampling
	local function sampleTri(tri, n)
		local pts = {}
		for i = 0, n do
			for j = 0, n - i do
				local a, b = i / n, j / n
				table.insert(pts, tri:PointAt(1 - a - b, a, b))
			end
		end
		return pts
	end

	h:test("DistPoint3Triangle3", function()
		local tri = Triangle3d.new(V(0, 0, 0), V(1, 0, 0), V(0, 1, 0))
		local q = G3.DistPoint3Triangle3.new(V(0.25, 0.25, 2), tri):Compute()
		h.near(q:Get(), 2)
		h.near(q.TriangleClosest.x, 0.25)
		h.near(q.TriangleBaryCoords.x, 0.5)
		q:SetPoint(V(2, 2, 0))
		h.near(q:GetSquared(), 1.5 * 1.5 * 2)
		h.near(q.TriangleClosest.x, 0.5)
		h.near(q.TriangleClosest.y, 0.5)
		local d, c, b = G3.DistPoint3Triangle3.DistanceSqr(V(-1, -1, 0), tri)
		h.near(d, 2)
		h.near(c:Length(), 0)
		h.near(b.x, 1)
		-- MeshQueries
		local mesh = TestMeshes.Octahedron(G3)
		local td = G3.MeshQueries.TriangleDistance(mesh, 0, V(1, 1, 1))
		h.near(td:Get(), (V(1, 1, 1) - V(1 / 3, 1 / 3, 1 / 3)):Length(), 1e-12)
		h.near(G3.MeshQueries.TriDistanceSqr(mesh, 0, V(1, 1, 1)), td:GetSquared(), 1e-14)
		local bary = G3.MeshQueries.BaryCoords(mesh, 0, V(1 / 3, 1 / 3, 1 / 3))
		h.near(bary.x, 1 / 3, 1e-12)
		local mn, mx, avg = G3.MeshQueries.EdgeLengthStats(mesh)
		h.near(mn, math.sqrt(2), 1e-12)
		h.near(mx, math.sqrt(2), 1e-12)
		h.near(avg, math.sqrt(2), 1e-12)
	end)

	h:test("line/segment/triangle distances vs sampling", function()
		for _ = 1, 30 do
			local t0 = Triangle3d.new(randPoint(1), randPoint(1), randPoint(1))
			local t1 = Triangle3d.new(randPoint(1) + V(1.5, 0, 0), randPoint(1) + V(1.5, 0, 0), randPoint(1) + V(1.5, 0, 0))
			local q = G3.DistTriangle3Triangle3.new(t0, t1)
			local d = q:Get()
			-- closest points are consistent
			h.near(q.Triangle0Closest:Distance(q.Triangle1Closest), d, 1e-9)
			h.near(t0:PointAt(q.Triangle0BaryCoords):Distance(q.Triangle0Closest), 0, 1e-9)
			h.near(t1:PointAt(q.Triangle1BaryCoords):Distance(q.Triangle1Closest), 0, 1e-9)
			-- brute force upper bound check
			local best = math.huge
			for _, p in sampleTri(t0, 12) do
				best = math.min(best, G3.DistPoint3Triangle3.new(p, t1):Get())
			end
			h.truthy(d <= best + 1e-9, "tri/tri distance is minimal")
			h.truthy(best - d < 0.1, "tri/tri distance close to sampled")

			-- segment / triangle
			local s = G3.Segment3d.new(randPoint(1) + V(0, 2, 0), randPoint(1) + V(0, 2, 0))
			local qs = G3.DistSegment3Triangle3.new(s, t0):Compute()
			h.near(qs.SegmentClosest:Distance(qs.TriangleClosest), qs:Get(), 1e-9)
			for k = 0, 20 do
				local p = s:PointBetween(k / 20)
				h.truthy(G3.DistPoint3Triangle3.new(p, t0):Get() >= qs:Get() - 1e-9)
			end
			-- line / segment
			local line = G3.Line3d.new(randPoint(1), randPoint(1):Normalized())
			local ql = G3.DistLine3Segment3.new(line, s):Compute()
			h.near(ql.LineClosest:Distance(ql.SegmentClosest), ql:Get(), 1e-9)
			h.near(line:PointAt(ql.LineParameter):Distance(ql.LineClosest), 0, 1e-9)
			-- line / triangle
			local qlt = G3.DistLine3Triangle3.new(line, t0):Compute()
			h.near(qlt.LineClosest:Distance(qlt.TriangleClosest), qlt:Get(), 1e-9)
		end
		-- intersecting triangles have distance 0
		local a = Triangle3d.new(V(0, 0, -1), V(1, 0, 1), V(-1, 0, 1))
		local b = Triangle3d.new(V(0, -1, 0), V(0, 1, 0), V(0.2, 0, 0.5))
		h.near(G3.DistTriangle3Triangle3.new(a, b):Get(), 0)
	end)

	h:test("ray/triangle and ray/box", function()
		local tri = Triangle3d.new(V(0, 0, 0), V(1, 0, 0), V(0, 1, 0))
		local ray = G3.Ray3d.new(V(0.2, 0.3, 5), V(0, 0, -1))
		local q = G3.IntrRay3Triangle3.new(ray, tri)
		h.truthy(q:Find())
		h.eq(q.Result, IntersectionResult.Intersects)
		h.truthy(q:IsSimpleIntersection())
		h.near(q.RayParameter, 5)
		h.near(q.TriangleBaryCoords.y, 0.2)
		h.near(q.TriangleBaryCoords.z, 0.3)
		local hit, t = G3.IntrRay3Triangle3.Intersects(ray, tri.V0, tri.V1, tri.V2)
		h.truthy(hit)
		h.near(t, 5)
		h.truthy(not G3.IntrRay3Triangle3.new(G3.Ray3d.new(V(2, 2, 5), V(0, 0, -1)), tri):Find())
		h.truthy(not G3.IntrRay3Triangle3.new(G3.Ray3d.new(V(0.2, 0.2, 5), V(0, 0, 1)), tri):Find())

		local box = G3.AxisAlignedBox3d.new(-1, -1, -1, 1, 1, 1)
		local r = G3.Ray3d.new(V(-5, 0, 0), V(1, 0, 0))
		local ok, rt = G3.IntrRay3AxisAlignedBox3.FindRayIntersectT(r, box)
		h.truthy(ok)
		h.near(rt, 4)
		h.truthy(G3.IntrRay3AxisAlignedBox3.Intersects(r, box))
		local qb = G3.IntrRay3AxisAlignedBox3.new(r, box)
		h.truthy(qb:Find())
		h.eq(qb.Type, IntersectionType.Segment)
		h.near(qb.RayParam0, 4)
		h.near(qb.RayParam1, 6)
		h.near(qb.Point1.x, 1)
		local miss = G3.Ray3d.new(V(-5, 2, 0), V(1, 0, 0))
		h.truthy(not G3.IntrRay3AxisAlignedBox3.Intersects(miss, box))
		h.truthy(not (G3.IntrRay3AxisAlignedBox3.FindRayIntersectT(miss, box)))
		local ql = G3.IntrLine3AxisAlignedBox3.new(G3.Line3d.new(V(5, 0, 0), V(1, 0, 0)), box)
		h.truthy(ql:Find())
		h.near(ql.LineParam0, -6)
		h.truthy(ql:Test())

		-- other ray utilities
		local bHit, st = G3.RayIntersection.Sphere(V(-5, 0, 0), V(1, 0, 0), Vector3d.Zero, 1)
		h.truthy(bHit)
		h.near(st, 4)
		local ok2, res = G3.IntersectionUtil.RaySphere(V(0, 0, 0), V(1, 0, 0), Vector3d.Zero, 1)
		h.truthy(ok2)
		h.eq(res.numIntersections, 1)
		h.near(res.parameter.a, 1)
		h.truthy(G3.IntersectionUtil.Intersects(ray.Origin, ray.Direction, tri.V0, tri.V1, tri.V2))
		local cHit, ct = G3.RayIntersection.InfiniteCylinder(V(0, 0, 0), V(1, 0, 0), Vector3d.Zero, V(0, 0, 1), 2)
		h.truthy(cHit)
		h.near(ct, 2)
	end)

	h:test("triangle/triangle intersection", function()
		local a = Triangle3d.new(V(-1, -1, 0), V(1, -1, 0), V(0, 1, 0))
		local b = Triangle3d.new(V(0, 0, -1), V(0.2, 0, 1), V(0, -0.5, 1))
		local q = G3.IntrTriangle3Triangle3.new(a, b)
		h.truthy(q:Test())
		h.truthy(G3.IntrTriangle3Triangle3.Intersects(a, b))
		h.truthy(q:Find())
		h.eq(q.Type, IntersectionType.Segment)
		h.eq(q.Quantity, 2)
		for i = 1, 2 do
			h.near(q.Points[i].z, 0, 1e-12)
		end
		-- separated
		local c = Triangle3d.new(V(0, 0, 1), V(1, 0, 1), V(0, 1, 1))
		h.truthy(not G3.IntrTriangle3Triangle3.Intersects(a, c))
		h.truthy(not G3.IntrTriangle3Triangle3.new(a, c):Find())
		-- vertex touching
		local d = Triangle3d.new(V(0, 0, 0), V(0, 0, 1), V(0.5, 0.5, 1))
		local qd = G3.IntrTriangle3Triangle3.new(a, d)
		h.truthy(qd:Find())
		h.eq(qd.Type, IntersectionType.Point)
		-- coplanar overlap
		local e = Triangle3d.new(V(0, -1.5, 0), V(1, 0.5, 0), V(-1, 0.5, 0))
		local qe = G3.IntrTriangle3Triangle3.new(a, e)
		h.truthy(not qe:Find(), "coplanar not reported by default")
		qe = G3.IntrTriangle3Triangle3.new(a, e)
		qe.ReportCoplanarIntersection = true
		h.truthy(qe:Find())
		h.eq(qe.Type, IntersectionType.Polygon)
		h.eq(qe.Quantity, 6)
		h.eq(#qe.PolygonPoints, 6)
		-- 2D helpers
		local t2a = G3.Triangle2d.new(V2(0, 0), V2(2, 0), V2(0, 2))
		local t2b = G3.Triangle2d.new(V2(0.5, 0.5), V2(3, 0.5), V2(0.5, 3))
		local i2 = G3.IntrTriangle2Triangle2.new(t2a, t2b)
		h.truthy(i2:Test())
		h.truthy(i2:Find())
		h.eq(i2.Quantity, 3) -- overlap is the triangle (0.5,0.5) (1.5,0.5) (0.5,1.5)
		h.eq(i2.Type, IntersectionType.Polygon)
		local seg = G3.Segment2d.new(V2(-1, 0.5), V2(3, 0.5))
		local is2 = G3.IntrSegment2Triangle2.new(seg, t2a)
		h.truthy(is2:Find())
		h.eq(is2.Type, IntersectionType.Segment)
		h.near(is2.Point0.x, 0, 1e-12)
		h.near(is2.Point1.x, 1.5, 1e-12)
		local i1 = G3.Intersector1.new(0, 2, 1, 3)
		h.truthy(i1:Find())
		h.eq(i1.NumIntersections, 2)
		h.near(i1:GetIntersection(0), 1)
		h.near(i1:GetIntersection(1), 2)
		h.truthy(seg:Intersects(G3.Segment2d.new(V2(0, 0), V2(0, 1))))
	end)

	h:test("Box3d, Plane3d, BoundsUtil", function()
		local box = G3.Box3d.new(V(1, 2, 3), V(1, 1, 2))
		local aabb = box:ToAABB()
		h.near(aabb.Min.x, 0)
		h.near(aabb.Max.z, 5)
		h.truthy(box:Contains(V(1.5, 2.5, 4)))
		h.truthy(not box:Contains(V(2.5, 2.5, 4)))
		h.near(box:DistanceSquared(V(3, 2, 3)), 1)
		h.near(box:ClosestPoint(V(3, 2, 3)).x, 2)
		h.near(box:Volume(), 16)
		h.near(box:Corner(6).x, 2)
		h.near(box:Corner(6).z, 5)
		box:Contain(V(4, 2, 3))
		h.near(box:ToAABB().Max.x, 4)
		local merged = G3.Box3d.Merge(
			G3.Box3d.FromAABB(G3.AxisAlignedBox3d.new(0, 0, 0, 1, 1, 1)),
			G3.Box3d.FromAABB(G3.AxisAlignedBox3d.new(2, 0, 0, 3, 1, 1))
		)
		h.near(merged.Extent.x, 1.5, 1e-12)
		h.near(merged.Center.x, 1.5, 1e-12)
		local fb = G3.Box3d.new(G3.Frame3d.new(V(1, 1, 1)), V(1, 2, 3))
		h.near(fb:ToAABB().Max.y, 3, 1e-12)

		local plane = G3.Plane3d.FromPoints(V(0, 0, 1), V(1, 0, 1), V(0, 1, 1))
		h.near(plane:DistanceTo(V(5, 5, 3)), 2)
		h.eq(plane:WhichSide(V(0, 0, 0)), -1)
		h.near(plane:Project(V(3, 4, 7)).z, 1)

		local tb = G3.BoundsUtil.Bounds(Triangle3d.new(V(0, 0, 0), V(1, -1, 2), V(-3, 2, 1)))
		h.near(tb.Min.x, -3)
		h.near(tb.Max.z, 2)
		local xb = G3.BoundsUtil.BoundsTransformed(G3.AxisAlignedBox3d.new(0, 0, 0, 1, 1, 1), function(v)
			return v * 2
		end)
		h.near(xb.Max.x, 2)
		local pb = G3.BoundsUtil.Bounds({ V(1, 1, 1), V(-1, 2, 0) })
		h.near(pb.Max.y, 2)
		local count = 0
		G3.BoundsUtil.TrianglesContained(TestMeshes.Octahedron(G3), function(p)
			return p.z > 0
		end, function(tid)
			count += 1
		end)
		h.eq(count, 4)
	end)

	h:test("MeshMeasurements", function()
		local box = TestMeshes.Box(G3, 2, 2, 2, 2)
		local tris = G3.Util.ToArray(box:TriangleIndices())
		local va = G3.MeshMeasurements.VolumeArea(box, tris, function(vid)
			return box:GetVertex(vid)
		end)
		h.near(va.x, 8, 1e-9)
		h.near(va.y, 24, 1e-9)
		local mass, center, inertia = G3.MeshMeasurements.MassPropertiesMesh(box)
		h.near(mass, 8, 1e-9)
		h.near(center:Length(), 0, 1e-9)
		-- solid cube side 2: I = m * (a^2 + a^2) / 12 = 8 * 8 / 12
		h.near(inertia[1][1], 8 * 8 / 12, 1e-9)
		local b = G3.MeshMeasurements.Bounds(box, nil)
		h.near(b:Volume(), 8, 1e-12)
		h.near(G3.MeshMeasurements.AreaT(box, tris), 24, 1e-9)
		h.near(G3.MeshMeasurements.Centroid(box):Length(), 0, 1e-12)
		local ext = G3.MeshMeasurements.ExtentsOnAxis(box, V(1, 0, 0))
		h.near(ext.a, -1)
		h.near(ext.b, 1)
		h.near(G3.MeshMeasurements.VertexOneRingArea(box, 0, false) > 0 and 1 or 0, 1)
		local torus = TestMeshes.Torus(G3, 2, 0.5, 24, 12)
		local g = G3.MeshMeasurements.Genus(torus)
		h.truthy(g.Valid)
		h.eq(g.Genus, 1)
		h.eq(G3.MeshMeasurements.Genus(TestMeshes.Octahedron(G3)).Genus, 0)
		h.truthy(G3.MeshMeasurements.Genus(TestMeshes.Grid(G3, 2, 1)).HasBoundary)
	end)

	h:test("fast winding math coefficients", function()
		local sphere = TestMeshes.UVSphere(G3, 1.0, 12, 6)
		local tris = G3.Util.ToArray(sphere:TriangleIndices())
		local p, r, o1, o2 = G3.FastTriWinding.ComputeCoeffs(sphere, tris)
		-- closed surface: integrated area-normal vanishes, and order2 ~ volume * I
		h.near(o1:Length(), 0, 1e-12)
		h.truthy(r >= 1 - 1e-9)
		local vol = G3.MeshMeasurements.VolumeArea(sphere, tris).x
		h.near(o2.Row0.x, vol, 1e-9)
		h.near(o2.Row1.y, vol, 1e-9)
		local p2, r2, o12, o22 = G3.FastTriWinding.ComputeCoeffs(sphere, tris, G3.MeshTriInfoCache.new(sphere))
		h.near((p2 - p):Length(), 0, 1e-12)
		h.near(r2, r, 1e-12)
		h.near((o12 - o1):Length(), 0, 1e-12)
		h.near(o22.Row2.z, o2.Row2.z, 1e-12)
		-- far field of a closed surface is ~0
		h.near(G3.FastTriWinding.EvaluateOrder2Approx(p, o1, o2, V(10, 0, 0)), 0, 1e-6)
	end)

	return h:results()
end
