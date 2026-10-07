-- Tests for MeshBoundaryLoops, EdgeLoop, EdgeSpan, MeshNormals, MeshIterators, MeshUtil, DCurve3 (package B shared modules)
return function(G3, h)
	local TM = require(script.Parent.TestMeshes)
	local Vector3d = G3.Vector3d

	h:test("boundary loops of grid and grid with hole", function()
		local grid = TM.Grid(G3, 4, 4)
		local loops = G3.MeshBoundaryLoops.new(grid)
		h.eq(loops:Count(), 1)
		h.eq(loops:SpanCount(), 0)
		local loop = loops:Get(0)
		h.eq(loop:VertexCount(), 16)
		h.eq(loop:EdgeCount(), 16)
		h.truthy(loop:CheckValidity())
		h.truthy(loop:IsBoundaryLoop())
		h.truthy(not loop:IsInternalLoop())
		-- loop should be oriented consistently with boundary edges
		h.truthy(not loop:CorrectOrientation(), "already oriented")
		local n = 0
		for _, l in loops do
			n += 1
			h.truthy(l == loop)
		end
		h.eq(n, 1)

		-- remove two interior triangles -> second loop
		local tid = grid:FindTriangle(6, 7, 12)
		h.truthy(tid >= 0)
		grid:RemoveTriangle(tid, false, false)
		local loops2 = G3.MeshBoundaryLoops.new(grid)
		h.eq(loops2:Count(), 2)
		local small = loops2:Get(loops2:FindLoopContainingVertex(12))
		h.eq(small:VertexCount(), 3)
		h.eq(loops2:Get(loops2:MaxVerticesLoopIndex()):VertexCount(), 16)
		local fv = loops2:FindVertexIndex(12)
		h.truthy(fv[1] >= 0 and fv[2] >= 0)

		-- closed mesh has no loops
		local loops3 = G3.MeshBoundaryLoops.new(TM.Octahedron(G3))
		h.eq(loops3:Count(), 0)
	end)

	h:test("edge filter produces spans", function()
		local grid = TM.Grid(G3, 4, 4)
		local loops = G3.MeshBoundaryLoops.new(grid, false)
		-- only keep boundary edges with y == 0 (bottom row)
		loops.EdgeFilterF = function(eid)
			local _, a, b = grid:GetEdgeVPositions(eid)
			return a.y == 0 and b.y == 0
		end
		loops:Compute()
		h.eq(loops:Count(), 0)
		h.eq(loops:SpanCount(), 1)
		local span = loops.Spans[1]
		h.eq(span:EdgeCount(), 4)
		h.eq(span:VertexCount(), 5)
		h.truthy(span:CheckValidity())
		h.truthy(span:IsBoundarySpan())

		-- throw behavior
		local loops2 = G3.MeshBoundaryLoops.new(grid, false)
		loops2.EdgeFilterF = loops.EdgeFilterF
		loops2.SpanBehavior = G3.MeshBoundaryLoops.SpanBehaviors.ThrowException
		local ok, err = pcall(function()
			loops2:Compute()
		end)
		h.truthy(not ok)
		h.truthy(G3.MeshBoundaryLoops.IsException(err))
		h.truthy(err.UnclosedLoop)
	end)

	h:test("bowtie boundary splits into subloops", function()
		-- two triangle fans sharing a single vertex (bowtie)
		local mesh = G3.DMesh3.new()
		local V = Vector3d.new
		local c = mesh:AppendVertex(V(0, 0, 0))
		local a0 = mesh:AppendVertex(V(1, 0, 0))
		local a1 = mesh:AppendVertex(V(1, 1, 0))
		local b0 = mesh:AppendVertex(V(-1, 0, 0))
		local b1 = mesh:AppendVertex(V(-1, -1, 0))
		mesh:AppendTriangle(c, a0, a1)
		mesh:AppendTriangle(c, b0, b1)
		local loops = G3.MeshBoundaryLoops.new(mesh)
		h.eq(loops:Count(), 2)
		for _, l in loops.Loops do
			h.eq(l:VertexCount(), 3)
		end
	end)

	h:test("EdgeLoop / EdgeSpan constructors", function()
		local grid = TM.Grid(G3, 2, 2)
		local loop = G3.EdgeLoop.FromVertices(grid, { 0, 1, 2, 5, 8, 7, 6, 3 })
		h.truthy(loop:CheckValidity())
		local loop2 = G3.EdgeLoop.FromEdges(grid, loop.Edges)
		h.truthy(loop2:CheckValidity())
		h.eq(loop2.Vertices[1], 0)
		local rev = G3.EdgeLoop.FromVertices(grid, { 0, 3, 6, 7, 8, 5, 2, 1 }, true)
		h.eq(rev.Vertices[1], 1, "auto-oriented")
		h.eq(rev.Vertices[2], 2, "auto-oriented")
		h.truthy(loop:IsSameLoop(G3.EdgeLoop.FromCopy(loop)))
		local span = G3.EdgeSpan.FromVertices(grid, { 0, 1, 2 })
		h.truthy(span:CheckValidity())
		local span2 = G3.EdgeSpan.FromEdges(grid, span.Edges)
		h.eq(span2.Vertices[1], 0)
		h.eq(span2.Vertices[3], 2)
		local curve = loop:ToCurve()
		h.eq(curve:VertexCount(), 8)
		h.near(curve:ArcLength(), 8, 1e-9)
		local target = G3.DCurveProjectionTarget.new(curve)
		local p = target:Project(Vector3d.new(1, -1, 0))
		h.near(p.x, 1)
		h.near(p.y, 0)
		local d, iseg, t = curve:DistanceSquared(Vector3d.new(0.5, -1, 0))
		h.near(d, 1)
		h.eq(iseg, 0)
		h.near(t, 0)
	end)

	h:test("normals, iterators, util", function()
		local sphere = TM.UVSphere(G3, 1, 16, 8)
		G3.MeshNormals.QuickCompute(sphere)
		h.truthy(sphere:HasVertexNormals())
		for vid in sphere:VertexIndices() do
			local n = sphere:GetVertexNormal(vid)
			local v = sphere:GetVertex(vid)
			h.truthy(n:Dot(v:Normalized()) > 0.95, "normal points outward")
		end
		local n0 = G3.MeshNormals.QuickComputeVertex(sphere, 0)
		h.truthy(n0:Dot(sphere:GetVertex(0)) > 0.99)

		local grid = TM.Grid(G3, 3, 3)
		h.eq(G3.Util.Count(G3.MeshIterators.BoundaryVertices(grid)), 12)
		h.eq(G3.Util.Count(G3.MeshIterators.InteriorVertices(grid)), 4)
		h.eq(G3.Util.Count(G3.MeshIterators.BoundaryEdges(grid)), 12)
		h.eq(G3.Util.Count(G3.MeshIterators.BoundaryEdgeVertices(grid)), 24)
		h.eq(G3.Util.Count(G3.MeshIterators.InteriorEdges(grid)), grid:EdgeCount() - 12)

		-- smoothing a flat interior vertex keeps it in the plane
		local p = G3.MeshUtil.UniformSmooth(grid, 5, 0.5)
		h.near(p.z, 0)
		local pc = G3.MeshUtil.CotanSmooth(grid, 5, 1.0)
		h.near(pc.z, 0)
		local pm = G3.MeshUtil.MeanValueSmooth(grid, 5, 1.0)
		h.near(pm.z, 0)
		h.near(G3.MeshUtil.DiscreteGaussCurvature(grid, 5), 0, 1e-9)
		h.near(G3.MeshUtil.OpeningAngleD(grid, grid:FindEdge(5, 6)), 0, 1e-6)
		h.truthy(G3.MeshWeights.VoronoiArea(grid, 5) > 0)
		local eid = grid:FindEdge(0, 5)
		h.truthy(eid >= 0)
		h.truthy(not G3.MeshUtil.CheckIfEdgeFlipCreatesFlip(grid, eid))
	end)

	return h:results()
end
