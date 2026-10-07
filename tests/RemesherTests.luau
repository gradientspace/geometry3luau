-- Tests for Remesher, RemesherPro, projection targets, MeshConstraintUtil (package B)
return function(G3, h)
	local TM = require(script.Parent.TestMeshes)
	local Vector3d = G3.Vector3d

	local function volume(mesh)
		local vol = 0
		for tid in mesh:TriangleIndices() do
			local a, b, c = mesh:GetTriVertices(tid)
			vol += a:Dot(b:Cross(c)) / 6
		end
		return vol
	end

	local function edgeStats(mesh)
		local minL, maxL, sum, n = math.huge, 0, 0, 0
		for eid in mesh:EdgeIndices() do
			local _, a, b = mesh:GetEdgeVPositions(eid)
			local l = a:Distance(b)
			minL = math.min(minL, l)
			maxL = math.max(maxL, l)
			sum += l
			n += 1
		end
		return minL, maxL, sum / n
	end

	-- fraction of edges with length in [lo, hi]
	local function fractionInRange(mesh, lo, hi)
		local n, k = 0, 0
		for eid in mesh:EdgeIndices() do
			local _, a, b = mesh:GetEdgeVPositions(eid)
			local l = a:Distance(b)
			n += 1
			if l >= lo and l <= hi then
				k += 1
			end
		end
		return k / n
	end

	local function haveTree()
		return G3.DMeshAABBTree3 ~= nil and G3.DistPoint3Triangle3 ~= nil
	end

	-- a simple analytic sphere projection target
	local function sphereTarget(r)
		return {
			IsOrientedProjectionTarget = true,
			Project = function(_, p)
				local n = p:Normalized()
				return n * r, n
			end,
		}
	end

	h:test("remesh box to target edge length", function()
		local box = TM.Box(G3, 2, 2, 2, 4)
		local v0 = volume(box)
		local r = G3.Remesher.new(box)
		r:SetTargetEdgeLength(0.2)
		r:Precompute()
		r.SmoothSpeedT = 0.5
		for _ = 1, 20 do
			r:BasicRemeshPass()
		end
		h.truthy(box:CheckValidity(false), "valid")
		h.truthy(box:IsClosed(), "closed")
		local minL, maxL, avg = edgeStats(box)
		h.truthy(avg > 0.13 and avg < 0.27, "avg edge " .. avg)
		h.truthy(maxL < 0.2 * 1.33 * 1.5, "max edge " .. maxL)
		h.truthy(box:TriangleCount() > 1000, "tris " .. box:TriangleCount())
		-- no projection: smoothing shrinks the box somewhat but not catastrophically
		local v1 = volume(box)
		h.truthy(v1 > 0.5 * v0 and v1 <= v0 * 1.001, "volume " .. v1)
	end)

	h:test("remesh sphere with analytic projection", function()
		local sphere = TM.UVSphere(G3, 1, 24, 12)
		local r = G3.Remesher.new(sphere)
		r:SetTargetEdgeLength(0.1)
		r:SetProjectionTarget(sphereTarget(1))
		r:Precompute()
		for _ = 1, 15 do
			r:BasicRemeshPass()
		end
		h.truthy(sphere:CheckValidity(false))
		h.truthy(sphere:IsClosed())
		for vid in sphere:VertexIndices() do
			h.near(sphere:GetVertex(vid):Length(), 1, 1e-9)
		end
		local _, maxL, avg = edgeStats(sphere)
		h.truthy(avg > 0.07 and avg < 0.13, "avg edge " .. avg)
		h.truthy(maxL < 0.3, "max edge " .. maxL)
		local frac = fractionInRange(sphere, r.MinEdgeLength * 0.9, r.MaxEdgeLength * 1.1)
		h.truthy(frac > 0.9, "fraction in range " .. frac)
		local v = volume(sphere)
		h.near(v, 4 / 3 * math.pi, 0.05)

		-- inline projection + other smoothing types
		for _, st in { G3.Remesher.SmoothTypes.Cotan, G3.Remesher.SmoothTypes.MeanValue } do
			local s2 = TM.UVSphere(G3, 1, 16, 8)
			local r2 = G3.Remesher.new(s2)
			r2:SetTargetEdgeLength(0.25)
			r2.SmoothType = st
			r2.ProjectionMode = G3.Remesher.TargetProjectionMode.Inline
			r2.PreventNormalFlips = true
			r2:SetProjectionTarget(sphereTarget(1))
			for _ = 1, 8 do
				r2:BasicRemeshPass()
			end
			h.truthy(s2:CheckValidity(false))
			h.truthy(s2:IsClosed())
			h.near(volume(s2), 4 / 3 * math.pi, 0.25)
		end
	end)

	h:test("remesh open grid with fixed boundary", function()
		local grid = TM.Grid(G3, 6, 6)
		local nb0 = G3.Util.Count(grid:BoundaryEdgeIndices())
		local r = G3.Remesher.new(grid)
		G3.MeshConstraintUtil.FixAllBoundaryEdges(r)
		r:SetTargetEdgeLength(0.4)
		for _ = 1, 10 do
			r:BasicRemeshPass()
		end
		h.truthy(grid:CheckValidity(false))
		h.eq(G3.Util.Count(grid:BoundaryEdgeIndices()), nb0, "boundary unchanged")
		for vid in grid:VertexIndices() do
			h.near(grid:GetVertex(vid).z, 0)
		end
		h.truthy(grid:TriangleCount() > 72 * 3)

		-- PreserveBoundaryLoops: boundary can be split/collapsed but stays on the curve
		local grid2 = TM.Grid(G3, 6, 6)
		local r2 = G3.Remesher.new(grid2)
		G3.MeshConstraintUtil.PreserveBoundaryLoops(r2)
		r2.ENABLE_DEBUG_CHECKS = true
		r2:SetTargetEdgeLength(0.3)
		for _ = 1, 10 do
			r2:BasicRemeshPass()
		end
		h.truthy(grid2:CheckValidity(false))
		local nb = G3.Util.Count(grid2:BoundaryEdgeIndices())
		h.truthy(nb > nb0, "boundary refined " .. nb)
		for vid in G3.MeshIterators.BoundaryVertices(grid2) do
			local p = grid2:GetVertex(vid)
			local onBdry = math.abs(p.x) < 1e-9 or math.abs(p.x - 6) < 1e-9 or math.abs(p.y) < 1e-9 or math.abs(p.y - 6) < 1e-9
			h.truthy(onBdry, "boundary vertex on boundary")
		end
		local b = grid2:GetBounds()
		h.near(b.Min.x, 0)
		h.near(b.Max.y, 6)

		-- VertexControlF: freeze everything in x < 2
		local grid3 = TM.Grid(G3, 6, 6)
		local frozen = {}
		for vid in grid3:VertexIndices() do
			if grid3:GetVertex(vid).x < 2 then
				frozen[vid] = grid3:GetVertex(vid)
			end
		end
		local r3 = G3.Remesher.new(grid3)
		G3.MeshConstraintUtil.FixAllBoundaryEdges(r3)
		r3.EnableCollapses = false
		r3.EnableSplits = false
		r3.EnableFlips = false
		r3.VertexControlF = function(vid)
			return frozen[vid] and G3.Remesher.VertexControl.NoMovement or G3.Remesher.VertexControl.AllowAll
		end
		r3:BasicRemeshPass()
		for vid, p in frozen do
			h.truthy(grid3:GetVertex(vid) == p, "frozen vertex")
		end
	end)

	h:test("RemesherPro FastestRemesh", function()
		local sphere = TM.UVSphere(G3, 1, 16, 8)
		local r = G3.RemesherPro.new(sphere)
		r:SetTargetEdgeLength(0.1)
		r:SetProjectionTarget(sphereTarget(1))
		r:FastestRemesh()
		h.truthy(sphere:CheckValidity(false))
		h.truthy(sphere:IsClosed())
		local _, maxL, avg = edgeStats(sphere)
		h.truthy(avg > 0.06 and avg < 0.14, "avg edge " .. avg)
		h.truthy(maxL < 0.2, "max edge " .. maxL)
		for vid in sphere:VertexIndices() do
			h.near(sphere:GetVertex(vid):Length(), 1, 1e-9)
		end

		-- sharp edge reprojection on a box with analytic oriented box target
		local box = TM.Box(G3, 2, 2, 2, 3)
		local function boxProject(_, p)
			-- nearest point on surface of [-1,1]^3 and its face normal
			local ax, ay, az = math.abs(p.x), math.abs(p.y), math.abs(p.z)
			local q = Vector3d.new(math.clamp(p.x, -1, 1), math.clamp(p.y, -1, 1), math.clamp(p.z, -1, 1))
			if ax >= ay and ax >= az then
				return Vector3d.new(p.x >= 0 and 1 or -1, q.y, q.z), Vector3d.new(p.x >= 0 and 1 or -1, 0, 0)
			elseif ay >= az then
				return Vector3d.new(q.x, p.y >= 0 and 1 or -1, q.z), Vector3d.new(0, p.y >= 0 and 1 or -1, 0)
			end
			return Vector3d.new(q.x, q.y, p.z >= 0 and 1 or -1), Vector3d.new(0, 0, p.z >= 0 and 1 or -1)
		end
		local rb = G3.RemesherPro.new(box)
		rb:SetTargetEdgeLength(0.25)
		rb:SetProjectionTarget({ IsOrientedProjectionTarget = true, Project = boxProject })
		rb:SharpEdgeReprojectionRemesh(10, 5)
		h.truthy(box:CheckValidity(false))
		h.truthy(box:IsClosed())
		h.near(volume(box), 8, 0.6)

		-- non-oriented target must be rejected
		local rc = G3.RemesherPro.new(TM.Octahedron(G3))
		rc:SetProjectionTarget(G3.PlaneProjectionTarget.new(Vector3d.Zero, Vector3d.AxisZ))
		local ok = pcall(function()
			rc:SharpEdgeReprojectionRemesh(1, 1)
		end)
		h.truthy(not ok)
	end)

	h:test("projection targets", function()
		local c = G3.Circle3d.FromCenter(Vector3d.new(0, 1, 0), 2)
		local ct = G3.CircleProjectionTarget.new(c)
		local p = ct:Project(Vector3d.new(5, 3, 0))
		h.near(p.x, 2)
		h.near(p.y, 1)
		h.near(c:ArcLength(), 4 * math.pi)
		local cyl = G3.Cylinder3d.FromRadiusHeight(1, 4)
		local cy = G3.CylinderProjectionTarget.new(cyl)
		local q = cy:Project(Vector3d.new(3, 0.5, 0))
		h.near(q.x, 1)
		h.near(q.y, 0.5)
		local d = G3.DistPoint3Cylinder3.new(Vector3d.new(0, 0, 0.5), cyl):Compute()
		h.truthy(d:IsInside())
		h.near(d:Get(), 0.5)
		local cap = cy:Project(Vector3d.new(0.2, 5, 0))
		h.near(cap.y, 2)
		local pl = G3.PlaneProjectionTarget.new(Vector3d.new(0, 0, 1), Vector3d.AxisZ)
		local seq = G3.SequentialProjectionTarget.new(pl, ct)
		local sp = seq:Project(Vector3d.new(4, 1, 3))
		-- plane projection gives (4,1,1), then onto circle around (0,1,0) in XZ plane, radius 2
		h.near(sp.y, 1)
		h.near(sp.x, 8 / math.sqrt(17))
		h.near(sp.z, 2 / math.sqrt(17))

		local ts = G3.TransformSequence.new()
		ts:AppendTranslation(1, 2, 3)
		ts:AppendScale(Vector3d.new(2, 2, 2))
		local inv = ts:MakeInverse()
		local x = inv:TransformP(ts:TransformP(Vector3d.new(0.3, -0.4, 5)))
		h.near(x.x, 0.3)
		h.near(x.z, 5)

		if not haveTree() then
			h:skip("mesh projection target", "DMeshAABBTree3 not available yet")
			return
		end
		local sphere = TM.UVSphere(G3, 1, 32, 16)
		local mt = G3.MeshProjectionTarget.new(sphere)
		local mp, mn = mt:Project(Vector3d.new(0, 3, 0.01))
		h.truthy(math.abs(mp:Length() - 1) < 0.01)
		h.truthy(mn:Dot(Vector3d.AxisY) > 0.9)
		local auto = G3.MeshProjectionTarget.Auto(sphere)
		h.truthy(auto.Mesh ~= sphere)
		local tt = G3.TransformedMeshProjectionTarget.new(sphere)
		local xf = G3.TransformSequence.new()
		xf:AppendTranslation(-5, 0, 0) -- source space is sphere shifted by +5 in x
		tt:SetTransform(xf)
		local tp, tn = tt:Project(Vector3d.new(5, 3, 0))
		h.near(tp.x, 5, 0.02)
		h.near(tp.y, 1, 0.02)
		h.truthy(tn:Dot(Vector3d.AxisY) > 0.9)
	end)

	h:test("remesh with mesh projection target", function()
		if not haveTree() then
			h:skip("remesh with mesh projection target", "DMeshAABBTree3 not available yet")
			return
		end
		local source = TM.Torus(G3, 2, 0.6, 48, 24)
		local mesh = G3.DMesh3.FromCopy(source)
		local target = G3.MeshProjectionTarget.new(source)
		local r = G3.Remesher.new(mesh)
		r:SetTargetEdgeLength(0.25)
		r:SetProjectionTarget(target)
		r:Precompute()
		for _ = 1, 10 do
			r:BasicRemeshPass()
		end
		h.truthy(mesh:CheckValidity(false))
		h.truthy(mesh:IsClosed())
		local _, maxL, avg = edgeStats(mesh)
		h.truthy(avg > 0.15 and avg < 0.32, "avg edge " .. avg)
		h.truthy(maxL < 0.75, "max " .. maxL)
		local frac = fractionInRange(mesh, r.MinEdgeLength * 0.9, r.MaxEdgeLength * 1.1)
		h.truthy(frac > 0.9, "fraction in range " .. frac)
		h.near(volume(mesh), volume(source), 0.05 * volume(source))

		-- reducer with inline projection
		local m2 = G3.DMesh3.FromCopy(source)
		local red = G3.Reducer.new(m2)
		red:SetProjectionTarget(G3.MeshProjectionTarget.new(source))
		red.ProjectionMode = G3.Reducer.TargetProjectionMode.Inline
		red:ReduceToTriangleCount(600)
		h.truthy(m2:CheckValidity(false))
		h.truthy(m2:TriangleCount() <= 600)
		local m3 = G3.DMesh3.FromCopy(source)
		local red3 = G3.Reducer.new(m3)
		red3:SetProjectionTarget(G3.MeshProjectionTarget.new(source))
		red3:ReduceToTriangleCount(600)
		h.truthy(m3:CheckValidity(false))
	end)

	h:test("submesh constraints and regional projection target", function()
		if not (haveTree() and G3.DSubmesh3 ~= nil and G3.MeshFaceSelection ~= nil) then
			h:skip("submesh constraints", "DSubmesh3 / MeshFaceSelection not available")
			return
		end
		local grid = TM.Grid(G3, 8, 8)
		local tris = {}
		for tid in grid:TriangleIndices() do
			local c = grid:GetTriCentroid(tid)
			if c.x > 2 and c.x < 6 and c.y > 2 and c.y < 6 then
				table.insert(tris, tid)
			end
		end
		local sub = G3.DSubmesh3.new(grid, tris)
		sub:ComputeBoundaryInfo(tris, #tris)
		local cons = G3.MeshConstraints.new()
		G3.MeshConstraintUtil.FixSubmeshBoundaryEdges(cons, sub)
		local nb = G3.Util.Count(sub.SubMesh:BoundaryEdgeIndices())
		local nc = 0
		for eid in sub.SubMesh:EdgeIndices() do
			if cons:HasEdgeConstraint(eid) then
				nc += 1
				h.truthy(sub.SubMesh:IsBoundaryEdge(eid))
			end
		end
		h.eq(nc, nb, "all submesh border edges constrained")

		local t = G3.MeshProjectionTarget.Auto(grid, tris, 1)
		h.truthy(t.Mesh:TriangleCount() > #tris)
		local p = t:Project(Vector3d.new(4, 4, 3))
		h.near(p.z, 0)
		h.near(p.x, 4)
	end)

	h:test("cancel remesh", function()
		local sphere = TM.UVSphere(G3, 1, 16, 8)
		local r = G3.RemesherPro.new(sphere)
		r:SetTargetEdgeLength(0.05)
		local n = 0
		r.Progress = G3.ProgressCancel.new(function()
			n += 1
			return n > 50
		end)
		r:FastestRemesh()
		h.truthy(sphere:CheckValidity(false))
		h.truthy(r:Cancelled())
	end)

	return h:results()
end
