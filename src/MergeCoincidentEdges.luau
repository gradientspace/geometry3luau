--!native
-- Port of geometry3Sharp mesh_ops/MergeCoincidentEdges.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Merge coincident boundary edges (ie close cracks / weld edges of a triangle soup).
--
-- Luau port notes:
--   * MergeCoincidentEdges.new(mesh); fields MergeDistance, OnlyUniquePairs; :Apply() -> bool
--   * Equivalence sets are Luau arrays keyed by edge id; the `remaining` HashSet keeps C# insertion order
--     (an ordered list + membership set), so priority-queue tie-breaking matches the C# behaviour.
--   * The C# hash-query retry `buffer = new int[buffer.Length]` would loop forever if more than 1024
--     midpoints coincide; here the buffer capacity is doubled instead.
--   * When a merge fails, C# does `EquivSets[other_eid].Remove(e.eid)`, which throws if that set was already
--     cleared; here a missing set is skipped.

local MathUtil = require(script.Parent.MathUtil)
local MeshResult = require(script.Parent.MeshResult)
local PointSetHashtable = require(script.Parent.PointSetHashtable)
local MeshPointSets = require(script.Parent.MeshPointSets)
local DynamicPriorityQueue = require(script.Parent.DynamicPriorityQueue)

local MergeCoincidentEdges = {}
MergeCoincidentEdges.__index = MergeCoincidentEdges
MergeCoincidentEdges.ClassName = "MergeCoincidentEdges"

function MergeCoincidentEdges.new(mesh)
	local self = setmetatable({}, MergeCoincidentEdges)
	self.Mesh = mesh
	self.MergeDistance = MathUtil.ZeroTolerancef
	self.OnlyUniquePairs = false
	self.merge_r2 = 0
	return self
end

function MergeCoincidentEdges:is_same_edge(a, b, c, d): boolean
	local merge_r2 = self.merge_r2
	return (a:DistanceSquared(c) < merge_r2 and b:DistanceSquared(d) < merge_r2)
		or (a:DistanceSquared(d) < merge_r2 and b:DistanceSquared(c) < merge_r2)
end

function MergeCoincidentEdges:Apply(): boolean
	local Mesh = self.Mesh
	local MergeDistance = self.MergeDistance
	self.merge_r2 = MergeDistance * MergeDistance

	-- construct hash table for edge midpoints
	local pointset = MeshPointSets.MeshBoundaryEdgeMidpoints.new(Mesh)
	local hash = PointSetHashtable.new(pointset)
	local hashN = 64
	if Mesh:TriangleCount() > 100000 then
		hashN = 128
	end
	if Mesh:TriangleCount() > 1000000 then
		hashN = 256
	end
	hash:Build(hashN)

	-- find edge equivalence sets. First we find all other edges with same
	-- midpoint, and then we check if endpoints are the same in second loop
	local buffer = {}
	local capacity = 1024
	local EquivSets: { [number]: { number }? } = {}
	local remaining = {} -- set
	local remainingOrder = {} -- insertion order of `remaining`
	for eid in Mesh:BoundaryEdgeIndices() do
		local midpt = Mesh:GetEdgePoint(eid, 0.5)
		local ok, N = hash:FindInBall(midpt, MergeDistance, buffer, capacity)
		while ok == false do
			capacity *= 2
			ok, N = hash:FindInBall(midpt, MergeDistance, buffer, capacity)
		end
		if N == 1 and buffer[1] ~= eid then
			error("MergeCoincidentEdges.Apply: how could this happen?!")
		end
		if N <= 1 then
			continue -- unique edge
		end

		local _, a, b = Mesh:GetEdgeVPositions(eid)

		-- if same endpoints, add to equivalence set
		local equiv = {}
		for i = 1, N do
			if buffer[i] ~= eid then
				local _, c, d = Mesh:GetEdgeVPositions(buffer[i])
				if self:is_same_edge(a, b, c, d) then
					table.insert(equiv, buffer[i])
				end
			end
		end
		if #equiv > 0 then
			EquivSets[eid] = equiv
			if remaining[eid] == nil then
				remaining[eid] = true
				table.insert(remainingOrder, eid)
			end
		end
	end

	-- [TODO] could replace remaining hashset w/ PQ, and use conservative count?

	-- add potential duplicate edges to priority queue, sorted by
	-- number of possible matches.
	-- [TODO] Does this need to be a PQ? Not updating PQ below anyway...
	local Q = DynamicPriorityQueue.new()
	for _, i in remainingOrder do
		local set_i = EquivSets[i] :: { number }
		if self.OnlyUniquePairs then
			if #set_i ~= 1 then
				continue
			end
			-- (the C# code also loops over EquivSets[i] here, but its `continue` only affects that
			--  inner loop, so it has no effect)
		end

		Q:Enqueue({ eid = i }, #set_i)
	end

	while Q:Count() > 0 do
		local e = Q:Dequeue()
		local e_eid = e.eid
		if Mesh:IsEdge(e_eid) == false or EquivSets[e_eid] == nil or remaining[e_eid] == nil then
			continue -- dealt with this edge already
		end
		if Mesh:IsBoundaryEdge(e_eid) == false then
			continue
		end

		local equiv = EquivSets[e_eid] :: { number }

		-- find viable match
		-- [TODO] how to make good decisions here? prefer planarity?
		local merged = false
		local failed = 0
		local i = 1
		while i <= #equiv and merged == false do
			local other_eid = equiv[i]
			if Mesh:IsEdge(other_eid) == false or Mesh:IsBoundaryEdge(other_eid) == false then
				i += 1
				continue
			end

			local result = Mesh:MergeEdges(e_eid, other_eid)
			if result ~= MeshResult.Ok then
				table.remove(equiv, i)
				i -= 1

				local other_set = EquivSets[other_eid]
				if other_set ~= nil then
					local k = table.find(other_set, e_eid)
					if k ~= nil then
						table.remove(other_set, k)
					end
				end
				--Q.UpdatePriority(...);  // how need ref to queue node to do this...??
				--   maybe equiv set is queue node??

				failed += 1
			else
				-- ok we merged, other edge is no longer free
				merged = true
				EquivSets[other_eid] = nil
				remaining[other_eid] = nil
			end
			i += 1
		end

		-- (if not merged: should we do something else here? doesn't make sense to put
		--  back into Q, as it should be at the top, right?)
		EquivSets[e_eid] = nil
		remaining[e_eid] = nil
	end

	return true
end

return MergeCoincidentEdges
