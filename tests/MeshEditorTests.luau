-- Tests for MeshEditor, MeshConnectedComponents, selections, DSubmesh3, MeshValidation,
-- FaceGroupUtil, SphericalFibonacciPointSet (package D).

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

	local function threeBoxes()
		local mesh = G3.DMesh3.new()
		TM.Append(G3, mesh, TM.Box(G3, 1, 1, 1, 1))
		TM.Append(G3, mesh, TM.Translate(G3, TM.Box(G3, 1, 1, 1, 2), 3, 0, 0))
		TM.Append(G3, mesh, TM.Translate(G3, TM.Box(G3, 1, 1, 1, 3), 6, 0, 0))
		return mesh
	end

	h:test("MeshConnectedComponents", function()
		local mesh = threeBoxes()
		local cc = G3.MeshConnectedComponents.new(mesh)
		cc:FindConnectedT()
		h.eq(cc:Count(), 3)
		cc:SortByCount(false)
		h.eq(#cc:Get(0).Indices, 108)
		h.eq(#cc:Get(2).Indices, 12)
		h.eq(cc:LargestByCount(), 0)
		local n = 0
		for comp in cc do
			n += #comp.Indices
		end
		h.eq(n, mesh:TriangleCount())
		local parts = G3.MeshConnectedComponents.Separate(mesh)
		h.eq(#parts, 3)
		h.eq(parts[1]:TriangleCount(), 108)
		h.truthy(parts[3]:CheckValidity(false) and parts[3]:IsClosed())
		h.eq(G3.MeshConnectedComponents.LargestT(mesh):TriangleCount(), 108)
		local set = G3.MeshConnectedComponents.FindConnectedTSeed(mesh, 0)
		local nset = 0
		for _ in set do
			nset += 1
		end
		h.eq(nset, 12)
		-- filtered
		cc.FilterF = function(tid)
			return tid < 12 + 48
		end
		cc:FindConnectedT()
		h.eq(cc:Count(), 2)
	end)

	h:test("MeshFaceSelection grow / contract / flood", function()
		local mesh = TM.Grid(G3, 10, 1)
		local sel = G3.MeshFaceSelection.new(mesh)
		local center = mesh:FindTriangle(5 * 11 + 5, 5 * 11 + 6, 6 * 11 + 6)
		h.truthy(center >= 0)
		sel:Select(center)
		h.eq(sel:Count(), 1)
		sel:ExpandToFaceNeighbours()
		h.eq(sel:Count(), 4)
		local s2 = G3.MeshFaceSelection.FromCopy(sel)
		s2:ExpandToOneRingNeighbours(2)
		local s3 = G3.MeshFaceSelection.FromCopy(sel)
		s3:ExpandToOneRingNeighbours()
		s3:ExpandToOneRingNeighbours()
		h.eq(s2:Count(), s3:Count())
		h.truthy(s2:Count() > 4)
		s3:ContractBorderByOneRingNeighbours()
		h.truthy(s3:Count() < s2:Count())
		local border = s2:FindBorderTris()
		local nbrs = s2:FindNeighbourTris()
		h.truthy(#border > 0 and #nbrs > 0)
		local flood = G3.MeshFaceSelection.new(mesh)
		flood:FloodFill(center)
		h.eq(flood:Count(), mesh:TriangleCount())
		local half = G3.MeshFaceSelection.new(mesh)
		half:FloodFill({ center }, function(tid)
			return mesh:GetTriCentroid(tid).x > 0.5
		end)
		h.eq(half:Count(), 100)
		local arr = half:ToArray()
		h.eq(#arr, 100)
		half:Deselect(arr)
		h.eq(half:Count(), 0)
		-- LocalOptimize fills ears / clips fins
		local s4 = G3.MeshFaceSelection.new(mesh)
		s4:SelectF(function(tid)
			return mesh:GetTriCentroid(tid).x < 0.5
		end)
		local c0 = s4:Count()
		s4:Select(mesh:FindTriangle(5, 6, 17)) -- isolated extra triangle (a fin/ear)
		s4:LocalOptimize(true, true)
		h.truthy(s4:Count() >= c0)
		-- vertex selection conversions
		local vsel = G3.MeshVertexSelection.FromFaceSelection(mesh, flood)
		h.eq(vsel:Count(), mesh:VertexCount())
		local left = G3.MeshFaceSelection.new(mesh)
		left:SelectF(function(tid)
			return mesh:GetTriCentroid(tid).x < 0.5
		end)
		h.eq(left:Count(), 100)
		local isel = G3.MeshVertexSelection.new(mesh)
		isel:SelectInteriorVertices(left)
		h.eq(isel:Count(), 55)
		local fsel = G3.MeshFaceSelection.FromVertexSelection(mesh, isel, 3)
		h.eq(fsel:Count(), 80)
		local esel = G3.MeshEdgeSelection.FromFaceSelection(mesh, fsel, 2)
		h.truthy(esel:Count() > 0)
		local bsel = G3.MeshEdgeSelection.new(mesh)
		bsel:SelectBoundaryTriEdges(fsel)
		h.eq(bsel:Count(), 2 * 4 + 2 * 10)
		local bv = G3.MeshVertexSelection.new(mesh)
		bv:SelectConnectedBoundaryV(0)
		h.eq(bv:Count(), 40)
		local fv = G3.MeshVertexSelection.new(mesh)
		fv:FloodFill(60, function(vid)
			return mesh:GetVertex(vid).y < 0.55
		end)
		h.eq(fv:Count(), 66)
	end)

	h:test("DSubmesh3 + ReinsertSubmesh", function()
		local mesh = TM.Box(G3, 2, 2, 2, 4)
		local sel = G3.MeshFaceSelection.new(mesh)
		sel:SelectF(function(tid)
			return mesh:GetTriCentroid(tid).z > 0.99
		end)
		h.eq(sel:Count(), 32)
		local sub = G3.DSubmesh3.new(mesh, sel)
		h.eq(sub.SubMesh:TriangleCount(), 32)
		h.eq(sub.SubMesh:VertexCount(), 25)
		sub:ComputeBoundaryInfo(sel)
		local nb = 0
		for _ in sub.BaseBorderE do
			nb += 1
		end
		h.eq(nb, 16)
		local sv = sub:MapVertexToSubmesh(sub:MapVertexToBaseMesh(3))
		h.eq(sv, 3)
		-- reinsert: remove region and insert again
		local editor = G3.MeshEditor.new(mesh)
		h.truthy(editor:RemoveTriangles(sel:ToArray(), true))
		h.truthy(not mesh:IsClosed())
		local new_tris = {}
		local ok, mapV = editor:ReinsertSubmesh(sub, new_tris)
		h.truthy(ok)
		h.eq(#new_tris, 32)
		h.truthy(mesh:IsClosed())
		h.truthy(mesh:CheckValidity(false))
		h.near(signedVolume(mesh), 8, 1e-9)
		-- quick submesh / tri maps
		local sub2 = G3.DSubmesh3.new(mesh)
		sub2.ComputeTriMaps = true
		sub2:Compute(new_tris)
		h.eq(sub2:MapTriangleToBaseMesh(0), new_tris[1])
		h.eq(sub2:MapTriangleToSubmesh(new_tris[5]), 4)
	end)

	h:test("MeshEditor stitch / fan / remove / separate / bowties", function()
		local mesh = G3.DMesh3.new()
		local loop0, loop1 = {}, {}
		local N = 8
		for i = 0, N - 1 do
			local a = 2 * math.pi * i / N
			table.insert(loop0, mesh:AppendVertex(V(math.cos(a), math.sin(a), 0)))
		end
		for i = 0, N - 1 do
			local a = 2 * math.pi * i / N
			table.insert(loop1, mesh:AppendVertex(V(math.cos(a), math.sin(a), 1)))
		end
		local editor = G3.MeshEditor.new(mesh)
		local tris = editor:StitchLoop(loop0, loop1)
		h.truthy(tris ~= nil)
		h.eq(#tris, 2 * N)
		h.truthy(mesh:CheckValidity(false))
		-- cap the two ends with fans
		local c0 = mesh:AppendVertex(V(0, 0, 0))
		local c1 = mesh:AppendVertex(V(0, 0, 1))
		local loops = G3.MeshBoundaryLoops.new(mesh)
		h.eq(loops:Count(), 2)
		for _, loop in loops.Loops do
			local center = mesh:GetVertex(loop.Vertices[1]).z < 0.5 and c0 or c1
			local fan = editor:AddTriangleFan_OrderedEdgeLoop(center, loop.Edges)
			h.truthy(fan ~= nil and #fan == N)
		end
		h.truthy(mesh:IsClosed())
		h.truthy(mesh:CheckValidity(false))
		h.truthy(math.abs(signedVolume(mesh)) > 2)
		-- invalid fan (edges no longer boundary) fails and backs out
		local nt = mesh:TriangleCount()
		local bad = editor:AddTriangleFan_OrderedVertexLoop(c0, loop0)
		h.eq(bad, nil)
		h.eq(mesh:TriangleCount(), nt)

		-- SeparateTriangles
		local box = TM.Box(G3, 1, 1, 1, 2)
		local top = {}
		for tid in box:TriangleIndices() do
			if box:GetTriCentroid(tid).y > 0.49 then
				table.insert(top, tid)
			end
		end
		local ed = G3.MeshEditor.new(box)
		local ok, pairs_ = ed:SeparateTriangles(top, true)
		h.truthy(ok)
		h.eq(#pairs_, 8)
		h.truthy(box:CheckValidity(false))
		h.eq(G3.Util.Count(box:BoundaryEdgeIndices()), 16)
		-- stitch them back with StitchUnorderedEdges (adds a zero-height strip)
		local new_tris, incomplete = ed:StitchUnorderedEdges(pairs_)
		h.truthy(new_tris ~= nil and not incomplete)
		h.truthy(box:IsClosed())

		-- bowtie: two tetrahedra sharing one vertex
		local bt = G3.DMesh3.new()
		local p = { V(0, 0, 0), V(1, 0, 0), V(0, 1, 0), V(0, 0, 1), V(-1, 0, 0), V(0, -1, 0), V(0, 0, -1) }
		for _, v in p do
			bt:AppendVertex(v)
		end
		for _, t in { { 0, 2, 1 }, { 0, 1, 3 }, { 0, 3, 2 }, { 1, 2, 3 }, { 0, 4, 5 }, { 0, 6, 4 }, { 0, 5, 6 }, { 4, 6, 5 } } do
			bt:AppendTriangle(t[1], t[2], t[3])
		end
		h.truthy(bt:IsBowtieVertex(0))
		local remaining = G3.MeshEditor.new(bt):DisconnectAllBowties()
		h.eq(remaining, 0)
		h.eq(bt:VertexCount(), 8)
		h.truthy(bt:CheckValidity(false))

		-- RemoveIsolatedTriangles / RemoveFinTriangles / RemoveUnusedVertices / AppendMesh
		local m = TM.Grid(G3, 2, 1)
		local a = m:AppendVertex(V(5, 5, 5))
		local b = m:AppendVertex(V(6, 5, 5))
		local c = m:AppendVertex(V(5, 6, 5))
		m:AppendTriangle(a, b, c)
		m:AppendVertex(V(9, 9, 9))
		h.truthy(G3.MeshEditor.RemoveIsolatedTriangles(m))
		h.eq(m:TriangleCount(), 8)
		h.eq(G3.MeshEditor.RemoveUnusedVertices(m), 1)
		h.eq(m:VertexCount(), 9)
		local nfin = G3.MeshEditor.RemoveFinTriangles(m, nil, false)
		h.eq(nfin, 2)
		local target = G3.DMesh3.new()
		local okA, mapV = G3.MeshEditor.new(target):AppendMesh(m)
		h.truthy(okA)
		h.eq(target:TriangleCount(), m:TriangleCount())
		h.eq(mapV[1] ~= nil, true)
		G3.MeshEditor.AppendMesh(target, m, 7)
		h.eq(target:TriangleCount(), 2 * m:TriangleCount())
		local comb = G3.MeshEditor.Combine(m, m, m)
		h.eq(comb:TriangleCount(), 3 * m:TriangleCount())
		-- ReverseTriangles flips volume sign
		local box2 = TM.Box(G3, 1, 2, 3, 1)
		G3.MeshEditor.new(box2):ReverseTriangles(G3.Util.ToArray(box2:TriangleIndices()))
		h.near(signedVolume(box2), -6, 1e-9)
	end)

	h:test("MeshEditor AppendBox / AppendLine / RemoveSmallComponents", function()
		local mesh = G3.DMesh3.new()
		G3.MeshEditor.AppendBox(mesh, V(0, 0, 0), 0.5)
		h.eq(mesh:TriangleCount(), 12)
		h.eq(mesh:VertexCount(), 8)
		h.truthy(mesh:IsClosed())
		h.near(math.abs(signedVolume(mesh)), 1, 1e-9)
		local editor = G3.MeshEditor.new(mesh)
		editor:AppendBox(G3.Frame3d.new(V(5, 0, 0)), V(1, 2, 3))
		h.eq(mesh:TriangleCount(), 24)
		G3.MeshEditor.AppendBox(mesh, V(10, 0, 0), V(0, 0, 1), 0.25)
		G3.MeshEditor.AppendLine(mesh, G3.Segment3d.new(V(20, 0, 0), V(20, 0, 4)), 0.1)
		h.eq(mesh:TriangleCount(), 48)
		h.truthy(mesh:CheckValidity(false))
		local vol = signedVolume(mesh)
		h.near(math.abs(vol), 1 + 48 + 0.125 + 0.1 * 0.1 * 2 * 4 * 2, 1e-9)
		-- remove components with volume < 0.5 (keeps the unit box and the 2x4x6 box)
		if vol < 0 then
			mesh:ReverseOrientation()
		end
		local nRemoved = G3.MeshEditor.RemoveSmallComponents(mesh, 0.5, 0)
		h.eq(nRemoved, 2)
		h.eq(mesh:TriangleCount(), 24)
	end)

	h:test("MeshEditor misc (strip, span, nearest stitch, duplicate, append variants, bowtie removal)", function()
		local mesh = G3.DMesh3.new()
		local ed = G3.MeshEditor.new(mesh)
		local frames, spans = {}, {}
		for i = 0, 4 do
			table.insert(frames, G3.Frame3d.new(V(i, 0, 0)))
			table.insert(spans, G3.Interval1d.new(-1, 1))
		end
		local strip = ed:AddTriangleStrip(frames, spans)
		h.eq(#strip, 8)
		h.truthy(mesh:CheckValidity(false))
		-- StitchSpan between two parallel vertex rows
		local m2 = G3.DMesh3.new()
		local s1, s2 = {}, {}
		for i = 0, 5 do
			table.insert(s1, m2:AppendVertex(V(i, 0, 0)))
			table.insert(s2, m2:AppendVertex(V(i, 1, 0)))
		end
		local e2 = G3.MeshEditor.new(m2)
		local st = e2:StitchSpan(s1, s2)
		h.eq(#st, 10)
		h.truthy(m2:CheckValidity(false))
		-- StitchVertexLoops_NearestV with rotated second loop
		local m3 = G3.DMesh3.new()
		local l0, l1 = {}, {}
		for i = 0, 5 do
			local a = 2 * math.pi * i / 6
			table.insert(l0, m3:AppendVertex(V(math.cos(a), math.sin(a), 0)))
		end
		for i = 0, 5 do
			local a = 2 * math.pi * ((i + 2) % 6) / 6
			table.insert(l1, m3:AppendVertex(V(math.cos(a), math.sin(a), 1)))
		end
		local st3 = G3.MeshEditor.new(m3):StitchVertexLoops_NearestV(l0, l1)
		h.eq(#st3, 12)
		h.truthy(m3:CheckValidity(false))
		-- DuplicateTriangles
		local box = TM.Box(G3, 1, 1, 1, 1)
		local eb = G3.MeshEditor.new(box)
		local map = G3.IndexMap.new(true)
		local dup, map2 = eb:DuplicateTriangles(box:TriangleIndices(), map)
		h.eq(#dup, 12)
		h.eq(map2, map)
		h.eq(box:VertexCount(), 16)
		h.truthy(box:CheckValidity(false))
		-- AppendMeshMerge / AppendMeshFull with transform and remapped groups
		local src = TM.Box(G3, 1, 1, 1, 1)
		local dst = G3.DMesh3.new(true, false, false, true)
		local merge = G3.IndexMap.new(true)
		local okm, mapV = G3.MeshEditor.new(dst):AppendMeshMerge(src, merge)
		h.truthy(okm)
		h.eq(dst:VertexCount(), 8)
		h.eq(#mapV, 8)
		local tmap = table.create(src:MaxTriangleID(), -1)
		local okf, vmap, gmap = G3.MeshEditor.AppendMeshFull(dst, src, tmap, G3.MeshEditor.AppendGroupPolicy.RemapGroupIDs, -1, {
			TransformPosition = function(p)
				return p + V(10, 0, 0)
			end,
		})
		h.truthy(okf and vmap ~= nil and gmap ~= nil)
		h.eq(dst:TriangleCount(), 24)
		h.truthy(tmap[1] >= 12)
		h.near(dst:GetVertex(vmap[1]).x, src:GetVertex(0).x + 10, 1e-12)
		-- RemoveAllBowtieVertices
		local bt = G3.DMesh3.new()
		for _, v in { V(0, 0, 0), V(1, 0, 0), V(0, 1, 0), V(-1, 0, 0), V(0, -1, 0) } do
			bt:AppendVertex(v)
		end
		bt:AppendTriangle(0, 1, 2)
		bt:AppendTriangle(0, 3, 4)
		h.truthy(G3.MeshEditor.new(bt):RemoveAllBowtieVertices(true))
		h.eq(bt:TriangleCount(), 0)
	end)

	h:test("MeshValidation + FaceGroupUtil", function()
		local mesh = TM.Grid(G3, 3, 1)
		local loops = G3.MeshBoundaryLoops.new(mesh)
		h.eq(G3.MeshValidation.IsBoundaryLoop(mesh, loops:Get(0)), G3.ValidationStatus.Ok)
		h.eq(G3.MeshValidation.IsEdgeLoop(mesh, loops:Get(0)), G3.ValidationStatus.Ok)
		h.eq(G3.MeshValidation.HasDuplicateTriangles(mesh), G3.ValidationStatus.Ok)
		local rev = G3.EdgeLoop.FromCopy(loops:Get(0))
		rev:Reverse()
		h.eq(G3.MeshValidation.IsBoundaryLoop(mesh, rev), G3.ValidationStatus.IncorrectLoopOrientation)

		local g = G3.DMesh3.new(true, false, false, true)
		TM.Append(G3, g, TM.Grid(G3, 2, 1))
		for tid in g:TriangleIndices() do
			g:SetTriangleGroup(tid, tid % 2 == 0 and 3 or 5)
		end
		local counts = G3.FaceGroupUtil.CountAllGroups(g)
		h.eq(counts[3], 4)
		h.eq(counts[5], 4)
		local sets = G3.FaceGroupUtil.FindTriangleSetsByGroup(g)
		h.eq(#sets, 2)
		h.eq(#sets[1], 4)
		G3.FaceGroupUtil.SetGroupToGroup(g, 5, 3)
		h.eq(#G3.FaceGroupUtil.FindTrianglesByGroup(g, 3), 8)
		local parts, ids = G3.FaceGroupUtil.SeparateMeshByGroups(g)
		h.eq(#parts, 1)
		h.eq(ids[1], 3)
	end)

	h:test("SphericalFibonacciPointSet NearestPoint", function()
		local sfps = G3.SphericalFibonacciPointSet.new(256)
		h.eq(sfps:Count(), 256)
		for i = 0, 255, 7 do
			local p = sfps:Point(i)
			h.near(p:Length(), 1, 1e-12)
			h.eq(sfps:NearestPoint(p, true), i)
			h.eq(sfps:NearestPoint(p * 3), i)
		end
		-- random directions: result must be the true nearest point
		local seed = 1
		for k = 1, 50 do
			seed = (seed * 1103515245 + 12345) % 2147483648
			local x = (seed % 1000) / 500 - 1
			seed = (seed * 1103515245 + 12345) % 2147483648
			local y = (seed % 1000) / 500 - 1
			seed = (seed * 1103515245 + 12345) % 2147483648
			local z = (seed % 1000) / 500 - 1
			local p = V(x, y, z):Normalized()
			local best, bestd = -1, math.huge
			for i = 0, 255 do
				local d = sfps:Point(i):DistanceSquared(p)
				if d < bestd then
					best, bestd = i, d
				end
			end
			local j = sfps:NearestPoint(p, true)
			h.near(sfps:Point(j):DistanceSquared(p), bestd, 1e-9, "nearest point " .. k)
		end
	end)

	return h:results()
end
