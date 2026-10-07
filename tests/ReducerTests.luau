-- Tests for Reducer (QEM simplification), QuadricError, IndexPriorityQueue, MeshConstraintUtil
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

	h:test("IndexPriorityQueue ordering", function()
		local q = G3.IndexPriorityQueue.new(100)
		q.EnableDebugChecks = true
		local vals = {}
		for i = 0, 99 do
			vals[i] = math.sin(i * 12.9898) * 43758.5453 % 1
			q:Insert(i, vals[i])
		end
		h.truthy(q:IsValidQueue())
		q:Update(50, -1)
		h.eq(q:First(), 50)
		q:Remove(50)
		h.truthy(not q:Contains(50))
		q:Insert(50, 2)
		local prev = -math.huge
		local n = 0
		while q:Count() > 0 do
			local p = q:FirstPriority()
			local id = q:Dequeue()
			h.truthy(p >= prev, "sorted")
			if id ~= 50 then
				h.eq(p, vals[id])
			end
			prev = p
			n += 1
		end
		h.eq(n, 100)
	end)

	h:test("QuadricError plane quadric", function()
		local Q = G3.QuadricError
		local q = Q.FromPlane(Vector3d.new(0, 0, 1), Vector3d.new(0, 0, 2))
		h.near(q:Evaluate(Vector3d.new(5, 3, 2)), 0)
		h.near(q:Evaluate(Vector3d.new(5, 3, 5)), 9)
		local q2 = Q.FromSum(q, Q.FromPlane(Vector3d.new(1, 0, 0), Vector3d.new(1, 0, 0)))
		q2:Add(1, Q.FromPlane(Vector3d.new(0, 1, 0), Vector3d.new(0, -3, 0)))
		local ok, p = q2:OptimalPoint()
		h.truthy(ok)
		h.near(p.x, 1)
		h.near(p.y, -3)
		h.near(p.z, 2)
		local ok2 = q:OptimalPoint()
		h.truthy(not ok2, "rank-1 quadric is degenerate")
	end)

	h:test("reduce sphere to triangle count", function()
		local sphere = TM.UVSphere(G3, 1, 64, 32)
		local v0 = volume(sphere)
		local r = G3.Reducer.new(sphere)
		r:ReduceToTriangleCount(500)
		h.truthy(sphere:TriangleCount() <= 500, "count " .. sphere:TriangleCount())
		h.truthy(sphere:TriangleCount() >= 490, "count " .. sphere:TriangleCount())
		h.truthy(sphere:CheckValidity(false), "valid")
		h.truthy(sphere:IsClosed(), "closed")
		local v1 = volume(sphere)
		h.truthy(math.abs(v1 - v0) / v0 < 0.03, string.format("volume %g vs %g", v1, v0))
		-- vertices should still be near the sphere
		for vid in sphere:VertexIndices() do
			local len = sphere:GetVertex(vid):Length()
			h.truthy(math.abs(len - 1) < 0.05, "radius " .. len)
		end
	end)

	h:test("reduce to vertex count, midpoint mode", function()
		local box = TM.Box(G3, 2, 2, 2, 8)
		local r = G3.Reducer.new(box)
		r.MinimizeQuadricPositionError = false
		r:ReduceToVertexCount(100)
		h.truthy(box:VertexCount() <= 100)
		h.truthy(box:CheckValidity(false))
		h.truthy(box:IsClosed())
		-- QEM should keep the box extents
		local b = box:GetBounds()
		h.near(b.Min.x, -1, 1e-6)
		h.near(b.Max.y, 1, 1e-6)
	end)

	h:test("reduce to edge length", function()
		local sphere = TM.UVSphere(G3, 1, 48, 24)
		local r = G3.Reducer.new(sphere)
		r:ReduceToEdgeLength(0.2)
		h.truthy(sphere:CheckValidity(false))
		h.truthy(sphere:IsClosed())
		h.truthy(sphere:TriangleCount() < 48 * 24 * 2)
		h.truthy(sphere:TriangleCount() > 20)

		local sphere2 = TM.UVSphere(G3, 1, 48, 24)
		local r2 = G3.Reducer.new(sphere2)
		r2:FastCollapsePass(0.1, 3, true)
		h.truthy(sphere2:CheckValidity(false))
		h.truthy(sphere2:IsClosed())
		h.truthy(sphere2:TriangleCount() < 48 * 24 * 2)
	end)

	h:test("boundary preservation on open grid", function()
		-- default PreserveBoundaryShape keeps boundary vertices on boundary
		local grid = TM.Grid(G3, 20, 10)
		local r = G3.Reducer.new(grid)
		r:ReduceToTriangleCount(100)
		h.truthy(grid:CheckValidity(false))
		local b = grid:GetBounds()
		h.near(b.Min.x, 0)
		h.near(b.Max.x, 10)
		h.near(b.Max.y, 10)
		for vid in grid:VertexIndices() do
			h.near(grid:GetVertex(vid).z, 0)
		end

		-- fully constrained boundary: boundary edges must all survive
		local grid2 = TM.Grid(G3, 20, 10)
		local nBdry = G3.Util.Count(grid2:BoundaryEdgeIndices())
		local r2 = G3.Reducer.new(grid2)
		local cons = G3.MeshConstraints.new()
		G3.MeshConstraintUtil.FixAllBoundaryEdges(cons, grid2)
		r2:SetExternalConstraints(cons)
		r2:ReduceToTriangleCount(50)
		h.truthy(grid2:CheckValidity(false))
		h.eq(G3.Util.Count(grid2:BoundaryEdgeIndices()), nBdry, "boundary edge count")
		h.truthy(grid2:TriangleCount() < 800)

		-- PreserveBoundaryLoops: boundary vertices stay on the boundary curve
		local grid3 = TM.Grid(G3, 20, 10)
		local r3 = G3.Reducer.new(grid3)
		local cons3 = G3.MeshConstraints.new()
		G3.MeshConstraintUtil.PreserveBoundaryLoops(cons3, grid3)
		r3:SetExternalConstraints(cons3)
		r3.ENABLE_DEBUG_CHECKS = true
		r3:ReduceToTriangleCount(100)
		h.truthy(grid3:CheckValidity(false))
		for vid in G3.MeshIterators.BoundaryVertices(grid3) do
			local p = grid3:GetVertex(vid)
			local onBdry = math.abs(p.x) < 1e-9 or math.abs(p.x - 10) < 1e-9 or math.abs(p.y) < 1e-9 or math.abs(p.y - 10) < 1e-9
			h.truthy(onBdry, "boundary vertex on boundary")
		end
	end)

	h:test("cancel via ProgressCancel", function()
		local sphere = TM.UVSphere(G3, 1, 32, 16)
		local nT = sphere:TriangleCount()
		local r = G3.Reducer.new(sphere)
		local calls = 0
		r.Progress = G3.ProgressCancel.new(function()
			calls += 1
			return calls > 10
		end)
		r:ReduceToTriangleCount(10)
		h.truthy(sphere:CheckValidity(false))
		h.truthy(sphere:TriangleCount() > 10)
		h.truthy(sphere:TriangleCount() < nT)
	end)

	return h:results()
end
