-- Port of geometry3Sharp mesh/MeshIndexUtil.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Utility functions for manipulating sets/lists of mesh indices
--
-- Luau port notes (see PORTING.md):
--   * AtoBV is an IIndexMap (see IndexMap.luau): C# AtoBV[i] -> AtoBV:Get(i).
--   * edge lists are 1-based Luau arrays of 0-based edge ids (C# List<int>).

local EdgeLoop = require(script.Parent.EdgeLoop)
local Util = require(script.Parent.Util)

local InvalidID = -1

local MeshIndexUtil = {}

-- given list of edges of MeshA, and vertex map from A to B, map to list of edges on B
function MeshIndexUtil.MapEdgesViaVertexMap(AtoBV, MeshA, MeshB, edges: { number }): { number }
	local N = #edges
	local result = table.create(N)
	for i = 1, N do
		local eid_a = edges[i]
		local aev = MeshA:GetEdgeV(eid_a)
		local bev0 = AtoBV:Get(aev[1])
		local bev1 = AtoBV:Get(aev[2])
		local eid_b = MeshB:FindEdge(bev0, bev1)
		Util.gDevAssert(eid_b ~= InvalidID, "MeshIndexUtil.MapEdgesViaVertexMap: edge not found")
		table.insert(result, eid_b)
	end
	return result
end

-- given EdgeLoop on MeshA, and vertex map from A to B, map to EdgeLoop on B
function MeshIndexUtil.MapLoopViaVertexMap(AtoBV, MeshA, MeshB, loopIn)
	local NV, NE = loopIn:VertexCount(), loopIn:EdgeCount()
	local newVerts = table.create(NV)
	for i = 1, NV do
		newVerts[i] = AtoBV:Get(loopIn.Vertices[i])
	end

	local newEdges = table.create(NE)
	for i = 1, NE do
		local eid_a = loopIn.Edges[i]
		local aev = MeshA:GetEdgeV(eid_a)
		local bev0 = AtoBV:Get(aev[1])
		local bev1 = AtoBV:Get(aev[2])
		newEdges[i] = MeshB:FindEdge(bev0, bev1)
		Util.gDevAssert(newEdges[i] ~= InvalidID, "MeshIndexUtil.MapLoopViaVertexMap: edge not found")
	end

	return EdgeLoop.FromArrays(MeshB, newVerts, newEdges, false)
end

return MeshIndexUtil
