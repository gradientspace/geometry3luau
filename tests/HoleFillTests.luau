-- Tests for SimpleHoleFiller, MinimalHoleFill, MeshRepairOrientation, RegionOperator,
-- RegionRemesher, RemoveOccludedTriangles (package D).

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

	-- sphere with the top cap (fan around the pole) removed: one hole with `slices` vertices
	local function openSphere(slices)
		local mesh = TM.UVSphere(G3, 1, slices, 8)
		local tris = {}
		for tid in mesh:VtxTrianglesItr(0) do
			table.insert(tris, tid)
		end
		G3.MeshEditor.RemoveTriangles(mesh, tris, true)
		return mesh
	end

	local function hasModule(name)
		return pcall(function()
			return G3[name] ~= nil and G3[name]
		end) and G3[name] ~= nil
	end

	h:test("SimpleHoleFiller", function()
		local mesh = openSphere(12)
		local loops = G3.MeshBoundaryLoops.new(mesh)
		h.eq(loops:Count(), 1)
		local filler = G3.SimpleHoleFiller.new(mesh, loops:Get(0))
		h.eq(filler:Validate(), G3.ValidationStatus.Ok)
		h.truthy(filler:Fill())
		h.eq(#filler.NewTriangles, 12)
		h.truthy(filler.NewVertex >= 0)
		h.truthy(mesh:IsClosed())
		h.truthy(mesh:CheckValidity(false))
		h.truthy(signedVolume(mesh) > 2.5)

		-- 3-loop: single triangle
		local m3 = TM.Octahedron(G3)
		mesh = m3
		m3:RemoveTriangle(0, false, false)
		local l3 = G3.MeshBoundaryLoops.new(m3)
		local f3 = G3.SimpleHoleFiller.new(m3, l3:Get(0))
		h.truthy(f3:Fill())
		h.eq(f3.NewVertex, -1)
		h.truthy(m3:IsClosed())
		h.near(signedVolume(m3), 4 / 3, 1e-12)
	end)

	h:test("MeshRepairOrientation", function()
		local mesh = TM.Box(G3, 1, 2, 3, 2)
		-- flip a scattered half of the triangles
		for tid = 0, mesh:MaxTriangleID() - 1, 2 do
			mesh:ReverseTriOrientation(tid)
		end
		local orient = G3.MeshRepairOrientation.new(mesh)
		orient:OrientComponents()
		h.eq(#orient.Components, 1)
		-- now consistent: every interior edge is traversed in opposite directions
		for eid in mesh:EdgeIndices() do
			local ev = mesh:GetEdgeV(eid)
			local et = mesh:GetEdgeT(eid)
			local t0, t1 = mesh:GetTriangle(et[1]), mesh:GetTriangle(et[2])
			local o0 = G3.IndexUtil.find_tri_ordered_edge(ev[1], ev[2], t0) ~= -1
			local o1 = G3.IndexUtil.find_tri_ordered_edge(ev[1], ev[2], t1) ~= -1
			h.truthy(o0 ~= o1, "consistent orientation at edge " .. eid)
		end
		orient:SolveGlobalOrientation()
		h.near(signedVolume(mesh), 6, 1e-9)

		-- an inside-out component gets flipped
		local inv = TM.Box(G3, 1, 1, 1, 1)
		inv:ReverseOrientation()
		h.truthy(signedVolume(inv) < 0)
		local o2 = G3.MeshRepairOrientation.new(inv)
		o2:OrientComponents()
		o2:SolveGlobalOrientation()
		h.near(signedVolume(inv), 1, 1e-9)
	end)

	h:test("RemoveOccludedTriangles", function()
		-- small box inside a big box: the inner one is removed
		local mesh = TM.Box(G3, 4, 4, 4, 2)
		TM.Append(G3, mesh, TM.Box(G3, 1, 1, 1, 1))
		G3.MeshNormals.QuickCompute(mesh)
		local nOuter = 6 * 2 * 2 * 2
		for _, mode in { 0, 1, 2, 3 } do
			local m = G3.DMesh3.FromCopy(mesh)
			local r = G3.RemoveOccludedTriangles.new(m)
			r.InsideMode = mode
			h.truthy(r:Apply())
			h.eq(#r.RemovedT, 12, "mode " .. mode)
			h.eq(m:TriangleCount(), nOuter)
			local m2 = G3.DMesh3.FromCopy(mesh)
			local r2 = G3.RemoveOccludedTriangles.new(m2)
			r2.InsideMode = mode
			r2.PerVertex = true
			r2:Apply()
			h.eq(#r2.RemovedT, 12, "pervertex mode " .. mode)
		end
	end)

	h:test("RegionOperator round trip", function()
		local mesh = TM.Box(G3, 2, 2, 2, 4)
		local tris = {}
		for tid in mesh:TriangleIndices() do
			if mesh:GetTriCentroid(tid).z > 0.99 then
				table.insert(tris, tid)
			end
		end
		local op = G3.RegionOperator.new(mesh, tris)
		local sub = op.Region.SubMesh
		h.eq(sub:TriangleCount(), 32)
		-- modify interior of submesh: flip a few interior edges
		local flips = 0
		for eid in sub:EdgeIndices() do
			if not sub:IsBoundaryEdge(eid) and flips < 5 then
				local r = sub:FlipEdge(eid)
				if r == G3.MeshResult.Ok then
					flips += 1
				end
			end
		end
		h.truthy(op:BackPropropagate())
		-- (RepairPossibleNonManifoldEdges splits the two corner diagonals that join border vertices)
		h.eq(#op:CurrentBaseTriangles(), sub:TriangleCount())
		h.eq(sub:TriangleCount(), 36)
		h.truthy(mesh:IsClosed())
		h.truthy(mesh:CheckValidity(false))
		h.near(signedVolume(mesh), 8, 1e-9)
		local interior = 0
		for _ in op:CurrentBaseInteriorVertices() do
			interior += 1
		end
		h.eq(interior, 9 + 2) -- + the two split-edge vertices
	end)

	if hasModule("RemesherPro") then
		h:test("MinimalHoleFill", function()
			local mesh = openSphere(16)
			local loops = G3.MeshBoundaryLoops.new(mesh)
			local fill = G3.MinimalHoleFill.new(mesh, loops:Get(0))
			h.truthy(fill:Apply())
			h.truthy(mesh:IsClosed())
			h.truthy(mesh:CheckValidity(false))
			h.eq(#fill.FillVertices, 0)
			h.eq(#fill.FillTriangles, 16 - 2)
			h.truthy(signedVolume(mesh) > 2.5)

			-- 4-hole on a box (remove two triangles of a quad)
			local box = TM.Box(G3, 1, 1, 1, 3)
			local t0 = 0
			local nbr = box:GetTriNeighbourTris(t0)
			box:RemoveTriangle(t0, false, false)
			box:RemoveTriangle(nbr[1] >= 0 and nbr[1] or nbr[2], false, false)
			local l4 = G3.MeshBoundaryLoops.new(box)
			h.eq(l4:Get(0):VertexCount(), 4)
			local f4 = G3.MinimalHoleFill.new(box, l4:Get(0))
			h.truthy(f4:Apply())
			h.truthy(box:IsClosed())
			h.near(signedVolume(box), 1, 1e-9)
		end)
		h:test("RegionRemesher", function()
			local mesh = TM.Box(G3, 2, 2, 2, 4)
			local tris = {}
			for tid in mesh:TriangleIndices() do
				if mesh:GetTriCentroid(tid).z > 0.99 then
					table.insert(tris, tid)
				end
			end
			local r = G3.RegionRemesher.QuickRemeshTarget(mesh, tris, 0.25, 0.5, 5, nil)
			h.truthy(#r:CurrentBaseTriangles() > 32)
			h.truthy(mesh:IsClosed())
			h.truthy(mesh:CheckValidity(false))
			h.near(signedVolume(mesh), 8, 1e-6)
		end)
	else
		h:skip("MinimalHoleFill", "RemesherPro not available yet")
		h:skip("RegionRemesher", "RemesherPro not available yet")
	end

	return h:results()
end
