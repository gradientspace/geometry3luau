-- Tests for MergeCoincidentEdges, RemoveDuplicateTriangles, PointSetHashtable, DynamicPriorityQueue (package D).

return function(G3, h)
	local TM = require(script.Parent.TestMeshes)

	local function signedVolume(mesh)
		local vol = 0
		for tid in mesh:TriangleIndices() do
			local a, b, c = mesh:GetTriVertices(tid)
			vol += a:Dot(b:Cross(c)) / 6
		end
		return vol
	end

	-- every triangle gets its own 3 vertices (triangle soup)
	local function unweld(mesh, jitter)
		local soup = G3.DMesh3.new()
		local n = 0
		for tid in mesh:TriangleIndices() do
			local a, b, c = mesh:GetTriVertices(tid)
			local ids = {}
			for k, v in { a, b, c } do
				if jitter then
					n += 1
					local s = jitter * math.sin(n * 12.9898)
					v = v + G3.Vector3d.new(s, -s, s * 0.5)
				end
				ids[k] = soup:AppendVertex(v)
			end
			soup:AppendTriangle(ids[1], ids[2], ids[3])
		end
		return soup
	end

	h:test("DynamicPriorityQueue orders by priority", function()
		local Q = G3.DynamicPriorityQueue.new()
		Q.EnableDebugChecks = true
		local vals = { 5, 3, 9, 1, 7, 2, 8, 2, 6 }
		local nodes = {}
		for i, p in vals do
			nodes[i] = { id = i }
			Q:Enqueue(nodes[i], p)
		end
		h.eq(Q:Count(), #vals)
		h.truthy(Q:IsValidQueue())
		Q:Update(nodes[3], 0) -- 9 -> 0
		h.eq(Q:First(), nodes[3])
		Q:Remove(nodes[5]) -- remove the 7
		local out = {}
		while Q:Count() > 0 do
			table.insert(out, Q:Dequeue().priority)
		end
		h.eq(table.concat(out, ","), "0,1,2,2,3,5,6,8")
	end)

	h:test("PointSetHashtable FindInBall", function()
		local mesh = TM.UVSphere(G3, 1, 16, 8)
		local hash = G3.PointSetHashtable.new(mesh)
		hash:Build(16)
		local buffer = {}
		for vid in mesh:VertexIndices() do
			local ok, n = hash:FindInBall(mesh:GetVertex(vid), 0.01, buffer)
			h.truthy(ok)
			h.eq(n, 1)
			h.eq(buffer[1], vid)
		end
		-- brute-force compare for a larger radius
		local r = 0.1
		local p = G3.Vector3d.new(0.3, 0.9, 0.2):Normalized()
		local ok, n = hash:FindInBall(p, r, buffer)
		h.truthy(ok)
		local count = 0
		for vid in mesh:VertexIndices() do
			if mesh:GetVertex(vid):Distance(p) < r then
				count += 1
			end
		end
		h.eq(n, count)
	end)

	h:test("merge unwelded box soup", function()
		local box = TM.Box(G3, 2, 3, 4, 2)
		local soup = unweld(box)
		h.eq(soup:VertexCount(), 3 * box:TriangleCount())
		h.truthy(not soup:IsClosed())
		local merge = G3.MergeCoincidentEdges.new(soup)
		h.truthy(merge:Apply())
		h.truthy(soup:CheckValidity(true))
		h.truthy(soup:IsClosed(), "merged box closed")
		h.eq(soup:TriangleCount(), box:TriangleCount())
		local compact = G3.DMesh3.FromCopy(soup, true)
		h.eq(compact:VertexCount(), box:VertexCount())
		h.near(signedVolume(soup), 24, 1e-9)
	end)

	h:test("merge unwelded sphere soup with tolerance", function()
		local sphere = TM.UVSphere(G3, 1, 24, 12)
		local soup = unweld(sphere, 1e-7)
		local merge = G3.MergeCoincidentEdges.new(soup)
		merge.MergeDistance = 1e-5
		merge:Apply()
		h.truthy(soup:CheckValidity(true))
		h.truthy(soup:IsClosed(), "merged sphere closed")
		h.truthy(signedVolume(soup) > 4)
	end)

	h:test("OnlyUniquePairs merges a simple crack", function()
		-- two grids sharing an edge line but with separate vertices
		local a = TM.Grid(G3, 4, 1)
		local b = TM.Grid(G3, 4, 1)
		TM.Translate(G3, b, 1, 0, 0)
		local mesh = G3.DMesh3.new()
		TM.Append(G3, mesh, a)
		TM.Append(G3, mesh, b)
		local nBdryBefore = G3.Util.Count(mesh:BoundaryEdgeIndices())
		local merge = G3.MergeCoincidentEdges.new(mesh)
		merge.OnlyUniquePairs = true
		merge:Apply()
		h.truthy(mesh:CheckValidity(true))
		local nBdryAfter = G3.Util.Count(mesh:BoundaryEdgeIndices())
		h.eq(nBdryBefore - nBdryAfter, 8)
	end)

	h:test("RemoveDuplicateTriangles", function()
		local mesh = TM.Box(G3, 1, 1, 1, 2)
		local soup = unweld(mesh)
		local nt = soup:TriangleCount()
		-- duplicate 5 triangles (same orientation), and one reversed
		for tid = 0, 4 do
			local a, b, c = soup:GetTriVertices(tid)
			soup:AppendTriangle(soup:AppendVertex(a), soup:AppendVertex(b), soup:AppendVertex(c))
		end
		do
			local a, b, c = soup:GetTriVertices(7)
			soup:AppendTriangle(soup:AppendVertex(a), soup:AppendVertex(c), soup:AppendVertex(b))
		end
		local dupe = G3.RemoveDuplicateTriangles.new(soup)
		h.truthy(dupe:Apply())
		h.eq(dupe.Removed, 5)
		h.eq(soup:TriangleCount(), nt + 1)
		dupe.CheckOrientation = false
		dupe:Apply()
		h.eq(dupe.Removed, 1)
		h.eq(soup:TriangleCount(), nt)
		h.truthy(soup:CheckValidity(true))
	end)

	return h:results()
end
