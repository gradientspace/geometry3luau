-- Tests for MeshIndexUtil and MeshOps (SetGroupBehavior)
return function(G3, h)
	local TM = require(script.Parent.TestMeshes)

	h:test("map edges and loop via vertex map", function()
		local meshA = TM.Grid(G3, 4, 4)
		-- MeshB = octahedron (6 vertices) + copy of MeshA, so A vertex v is B vertex v + 6
		local meshB = TM.Append(G3, TM.Octahedron(G3), meshA)
		local nShift = meshB:VertexCount() - meshA:VertexCount()
		h.eq(nShift, 6)
		local AtoBV = G3.IndexMap.ShiftIndexMap.new(nShift)

		local loopA = G3.MeshBoundaryLoops.new(meshA):Get(0)
		local loopB = G3.MeshIndexUtil.MapLoopViaVertexMap(AtoBV, meshA, meshB, loopA)
		h.eq(loopB.Mesh, meshB)
		h.eq(loopB:VertexCount(), loopA:VertexCount())
		h.eq(loopB:EdgeCount(), loopA:EdgeCount())
		for i = 1, loopA:VertexCount() do
			h.eq(loopB.Vertices[i], loopA.Vertices[i] + nShift)
		end
		h.truthy(loopB:CheckValidity())
		h.truthy(loopB:IsBoundaryLoop())

		local edgesB = G3.MeshIndexUtil.MapEdgesViaVertexMap(AtoBV, meshA, meshB, loopA.Edges)
		h.eq(#edgesB, #loopA.Edges)
		for i = 1, #edgesB do
			h.eq(edgesB[i], loopB.Edges[i])
			local ea, eb = meshA:GetEdgeV(loopA.Edges[i]), meshB:GetEdgeV(edgesB[i])
			h.eq(math.min(ea[1], ea[2]) + nShift, math.min(eb[1], eb[2]))
			h.eq(math.max(ea[1], ea[2]) + nShift, math.max(eb[1], eb[2]))
		end
	end)

	h:test("SetGroupBehavior", function()
		local SetGroupBehavior = G3.MeshOps.SetGroupBehavior
		local mesh = TM.Octahedron(G3)
		mesh:EnableTriangleGroups(0)
		h.eq(SetGroupBehavior.Ignore:GetGroupID(mesh), -1)
		h.eq(SetGroupBehavior.SetTo(7):GetGroupID(mesh), 7)
		local g0 = SetGroupBehavior.AutoGenerate:GetGroupID(mesh)
		local g1 = SetGroupBehavior.AutoGenerate:GetGroupID(mesh)
		h.truthy(g1 == g0 + 1, "AutoGenerate allocates new groups")
		h.eq(SetGroupBehavior.new(SetGroupBehavior.Modes.UseConstant, 3):GetGroupID(mesh), 3)
		h.eq(SetGroupBehavior.new(SetGroupBehavior.Modes.UseConstant).SetGroupID, 0)
	end)

	return h:results()
end
