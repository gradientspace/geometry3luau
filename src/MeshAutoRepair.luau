-- Port of geometry3Sharp mesh_ops/MeshAutoRepair.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Mesh Auto Repair top-level driver.
--
-- TODO:
--   - remove degenerate *faces*  (which may still have all edges > length)
--       - this is tricky, in many CAD meshes these faces can't just be collapsed. But can often remove via flipping...?
--
-- Luau port notes:
--   * MeshAutoRepair.new(mesh); options RepairTolerance, MinEdgeLengthTol, ErosionIterations,
--     RemoveMode (MeshAutoRepair.RemoveModes), Progress (ProgressCancel).
--   * Apply() -> bool. NOTE: like the C#, Apply() replaces .Mesh with a compacted copy at the end, so read
--     the result from repair.Mesh (the input mesh object is modified but not compacted).
--   * The C# goto-based control flow (repeat_all / all_done) is expressed with a loop.
--   * C# try/catch around MergeCoincidentEdges / RemoveDuplicateTriangles -> pcall.
--   * private helpers that had out-params return extra values:
--       fill_trivial_holes() -> nRemaining, saw_spans;  fill_any_holes() -> nRemaining, saw_spans
--       remove_duplicate_faces(tol) -> ok, nRemoved;     collapse_degenerate_edges(minLen, bBoundaryOnly) -> true, count
--       disconnect_bowties() -> true, nRemaining;        remove_interior()/remove_occluded() -> true, nRemoved

local MathUtil = require(script.Parent.MathUtil)
local MeshResult = require(script.Parent.MeshResult)
local DMesh3 = require(script.Parent.DMesh3)
local MeshIterators = require(script.Parent.MeshIterators)
local MeshNormals = require(script.Parent.MeshNormals)
local MeshBoundaryLoops = require(script.Parent.MeshBoundaryLoops)
local MeshEditor = require(script.Parent.MeshEditor)
local MeshFaceSelection = require(script.Parent.MeshFaceSelection)
local MergeCoincidentEdges = require(script.Parent.MergeCoincidentEdges)
local RemoveDuplicateTriangles = require(script.Parent.RemoveDuplicateTriangles)
local MeshRepairOrientation = require(script.Parent.MeshRepairOrientation)
local SimpleHoleFiller = require(script.Parent.SimpleHoleFiller)
local MinimalHoleFill = require(script.Parent.MinimalHoleFill)
local RemoveOccludedTriangles = require(script.Parent.RemoveOccludedTriangles)

local MeshAutoRepair = {}
MeshAutoRepair.__index = MeshAutoRepair
MeshAutoRepair.ClassName = "MeshAutoRepair"

-- [TODO] interior components?
MeshAutoRepair.RemoveModes = table.freeze({
	None = 0,
	Interior = 1,
	Occluded = 2,
})
local RemoveModes = MeshAutoRepair.RemoveModes

function MeshAutoRepair.new(mesh3)
	local self = setmetatable({}, MeshAutoRepair)
	self.RepairTolerance = MathUtil.ZeroTolerancef
	-- assume edges shorter than this are degenerate and should be collapsed
	self.MinEdgeLengthTol = 0.0001
	-- number of times we will delete border triangles and try again
	self.ErosionIterations = 5
	self.RemoveMode = RemoveModes.None
	-- Set this to be able to cancel running remesher
	self.Progress = nil
	self.Mesh = mesh3
	return self
end

-- if this returns true, abort computation.
function MeshAutoRepair:Cancelled(): boolean
	return (self.Progress ~= nil) and self.Progress:Cancelled() or false
end

function MeshAutoRepair:Apply(): boolean
	local do_checks = false

	if do_checks then
		self.Mesh:CheckValidity()
	end

	--[[
	 * Remove parts of the mesh we don't want before we bother with anything else
	 * TODO: maybe we need to repair orientation first? if we want to use MWN...
	--]]
	self:do_remove_inside()
	if self:Cancelled() then
		return false
	end

	local repeat_count = 0
	while true do -- repeat_all:
		--[[
		 * make sure orientation of connected components is consistent
		 * TODO: what about mobius strip problems?
		--]]
		self:repair_orientation(false)
		if self:Cancelled() then
			return false
		end

		--[[
		 *  Do safe close-cracks to handle easy cases
		--]]
		self:repair_cracks(true, self.RepairTolerance)
		if self.Mesh:IsClosed() then
			break -- goto all_done
		end
		if self:Cancelled() then
			return false
		end

		--[[
		 * Collapse tiny edges and then try easy cases again, and
		 * then allow for handling of ambiguous cases
		--]]
		self:collapse_all_degenerate_edges(self.RepairTolerance * 0.5, true)
		if self:Cancelled() then
			return false
		end
		self:repair_cracks(true, 2 * self.RepairTolerance)
		if self:Cancelled() then
			return false
		end
		self:repair_cracks(false, 2 * self.RepairTolerance)
		if self:Cancelled() then
			return false
		end
		if self.Mesh:IsClosed() then
			break -- goto all_done
		end

		--[[
		 * Possibly we have joined regions with different orientation (is it?), fix that
		 * TODO: mobius strips again
		--]]
		self:repair_orientation(false)
		if self:Cancelled() then
			return false
		end

		if do_checks then
			self.Mesh:CheckValidity()
		end

		-- get rid of any remaining single-triangles before we start filling holes
		self:remove_loners()

		--[[
		 * Ok, fill simple holes.
		--]]
		local nRemainingBowties = 0
		local nHoles, bSawSpans = self:fill_trivial_holes()
		if self:Cancelled() then
			return false
		end
		if self.Mesh:IsClosed() then
			break -- goto all_done
		end

		--[[
		 * Now fill harder holes. If we saw spans, that means boundary loops could
		 * not be resolved in some cases, do we disconnect bowties and try again.
		--]]
		nHoles, bSawSpans = self:fill_any_holes()
		if self:Cancelled() then
			return false
		end
		if bSawSpans then
			local _
			_, nRemainingBowties = self:disconnect_bowties()
			nHoles, bSawSpans = self:fill_any_holes()
		end
		if self:Cancelled() then
			return false
		end
		if self.Mesh:IsClosed() then
			break -- goto all_done
		end

		--[[
		 * We may have a closed mesh now but it might still have bowties (eg
		 * tetrahedra sharing vtx case). So disconnect those.
		--]]
		do
			local _
			_, nRemainingBowties = self:disconnect_bowties()
		end
		if self:Cancelled() then
			return false
		end

		--[[
		 * If the mesh is not closed, we will do one more round to try again.
		--]]
		if repeat_count == 0 and self.Mesh:IsClosed() == false then
			repeat_count += 1
			continue -- goto repeat_all
		end

		--[[
		 * Ok, we didn't get anywhere on our first repeat. If we are still not
		 * closed, we will try deleting boundary triangles and repeating.
		 * Repeat this N times.
		--]]
		if repeat_count <= self.ErosionIterations and self.Mesh:IsClosed() == false then
			repeat_count += 1
			local bdry_faces = MeshFaceSelection.new(self.Mesh)
			for eid in MeshIterators.BoundaryEdges(self.Mesh) do
				bdry_faces:SelectEdgeTris(eid)
			end
			MeshEditor.RemoveTriangles(self.Mesh, bdry_faces, true)
			continue -- goto repeat_all
		end

		break
	end

	-- all_done:

	--[[
	 * Remove tiny edges
	--]]
	if self.MinEdgeLengthTol > 0 then
		self:collapse_all_degenerate_edges(self.MinEdgeLengthTol, false)
	end
	if self:Cancelled() then
		return false
	end

	--[[
	 * finally do global orientation
	--]]
	self:repair_orientation(true)
	if self:Cancelled() then
		return false
	end

	if do_checks then
		self.Mesh:CheckValidity()
	end

	--[[
	 * Might as well compact output mesh...
	--]]
	self.Mesh = DMesh3.FromCopy(self.Mesh, true)
	MeshNormals.QuickCompute(self.Mesh)

	return true
end

-- returns nRemaining, saw_spans
function MeshAutoRepair:fill_trivial_holes(): (number, boolean)
	local Mesh = self.Mesh
	local loops = MeshBoundaryLoops.new(Mesh)
	local nRemaining = 0
	local saw_spans = loops.SawOpenSpans

	for _, loop in loops.Loops do
		if self:Cancelled() then
			break
		end
		local filled = false
		local nv = loop:VertexCount()
		if nv == 3 then
			local filler = SimpleHoleFiller.new(Mesh, loop)
			filled = filler:Fill()
		elseif nv == 4 then
			local filler = MinimalHoleFill.new(Mesh, loop)
			filled = filler:Apply()
			if filled == false then
				local fallback = SimpleHoleFiller.new(Mesh, loop)
				filled = fallback:Fill()
			end
		end

		if filled == false then
			nRemaining += 1
		end
	end
	return nRemaining, saw_spans
end

-- returns nRemaining, saw_spans (like the C#, nRemaining is never incremented here)
function MeshAutoRepair:fill_any_holes(): (number, boolean)
	local Mesh = self.Mesh
	local loops = MeshBoundaryLoops.new(Mesh)
	local nRemaining = 0
	local saw_spans = loops.SawOpenSpans

	for _, loop in loops.Loops do
		if self:Cancelled() then
			break
		end
		local filler = MinimalHoleFill.new(Mesh, loop)
		local filled = filler:Apply()
		if filled == false then
			if self:Cancelled() then
				break
			end
			local fallback = SimpleHoleFiller.new(Mesh, loop)
			filled = fallback:Fill()
		end
	end
	return nRemaining, saw_spans
end

function MeshAutoRepair:repair_cracks(bUniqueOnly: boolean, mergeDist: number): boolean
	local ok, result = pcall(function()
		local merge = MergeCoincidentEdges.new(self.Mesh)
		merge.OnlyUniquePairs = bUniqueOnly
		merge.MergeDistance = mergeDist
		return merge:Apply()
	end)
	if not ok then
		-- ??
		return false
	end
	return result
end

-- returns ok, nRemoved
function MeshAutoRepair:remove_duplicate_faces(vtxTolerance: number): (boolean, number)
	local nRemoved = 0
	local ok, bOK = pcall(function()
		local dupe = RemoveDuplicateTriangles.new(self.Mesh)
		dupe.VertexTolerance = vtxTolerance
		local r = dupe:Apply()
		nRemoved = dupe.Removed
		return r
	end)
	if not ok then
		return false, nRemoved
	end
	return bOK, nRemoved
end

-- returns true, collapseCount
function MeshAutoRepair:collapse_degenerate_edges(minLength: number, bBoundaryOnly: boolean): (boolean, number)
	local Mesh = self.Mesh
	local collapseCount = 0
	if Mesh:MaxEdgeID() == 0 then
		return true, 0 -- (C# ModuloIteration would throw DivideByZeroException on an empty edge range)
	end
	-- don't iterate sequentially because there may be pathological cases
	for eid in MathUtil.ModuloIteration(Mesh:MaxEdgeID()) do
		if self:Cancelled() then
			break
		end
		if Mesh:IsEdge(eid) == false then
			continue
		end
		local is_boundary_edge = Mesh:IsBoundaryEdge(eid)
		if bBoundaryOnly and is_boundary_edge == false then
			continue
		end
		local ev = Mesh:GetEdgeV(eid)
		local a, b = Mesh:GetVertex(ev[1]), Mesh:GetVertex(ev[2])
		if a:Distance(b) < minLength then
			local keep = Mesh:IsBoundaryVertex(ev[1]) and ev[1] or ev[2]
			local discard = (keep == ev[1]) and ev[2] or ev[1]
			local result = Mesh:CollapseEdge(keep, discard)
			if result == MeshResult.Ok then
				collapseCount += 1
				if Mesh:IsBoundaryVertex(keep) == false or is_boundary_edge then
					Mesh:SetVertex(keep, (a + b) * 0.5)
				end
			end
		end
	end
	return true, collapseCount
end

function MeshAutoRepair:collapse_all_degenerate_edges(minLength: number, bBoundaryOnly: boolean): boolean
	local rep = true
	while rep do
		if self:Cancelled() then
			break
		end
		local _, collapse_count = self:collapse_degenerate_edges(minLength, bBoundaryOnly)
		if collapse_count == 0 then
			rep = false
		end
	end
	return true
end

-- returns true, nRemaining
function MeshAutoRepair:disconnect_bowties(): (boolean, number)
	local editor = MeshEditor.new(self.Mesh)
	local nRemaining = editor:DisconnectAllBowties()
	return true, nRemaining
end

function MeshAutoRepair:repair_orientation(bGlobal: boolean)
	local orient = MeshRepairOrientation.new(self.Mesh)
	orient:OrientComponents()
	if self:Cancelled() then
		return
	end
	if bGlobal then
		orient:SolveGlobalOrientation()
	end
end

-- returns true, nRemoved
function MeshAutoRepair:remove_interior(): (boolean, number)
	local remove = RemoveOccludedTriangles.new(self.Mesh)
	remove.PerVertex = true
	remove.InsideMode = RemoveOccludedTriangles.CalculationMode.FastWindingNumber
	remove:Apply()
	return true, (remove.RemovedT ~= nil) and #remove.RemovedT or 0
end

-- returns true, nRemoved
function MeshAutoRepair:remove_occluded(): (boolean, number)
	local remove = RemoveOccludedTriangles.new(self.Mesh)
	remove.PerVertex = true
	remove.InsideMode = RemoveOccludedTriangles.CalculationMode.SimpleOcclusionTest
	remove:Apply()
	return true, (remove.RemovedT ~= nil) and #remove.RemovedT or 0
end

function MeshAutoRepair:do_remove_inside(): boolean
	if self.RemoveMode == RemoveModes.Interior then
		return (self:remove_interior())
	elseif self.RemoveMode == RemoveModes.Occluded then
		return (self:remove_occluded())
	end
	return true
end

function MeshAutoRepair:remove_loners(): boolean
	local _bOK = MeshEditor.RemoveIsolatedTriangles(self.Mesh)
	return true
end

return MeshAutoRepair
