-- Port of geometry3Sharp mesh/MeshConstraintUtil.cs
-- Copyright (c) Ryan Schmidt (rms@gradientspace.com) - All Rights Reserved
-- Distributed under the Boost Software License, Version 1.0. http://www.boost.org/LICENSE_1_0.txt
--
-- Luau port notes (see PORTING.md):
--   * C# overloads taking (MeshConstraints cons, DMesh3 mesh, ...) or (Remesher r, ...) are one function
--     that dispatches on the first argument: if it is a MeshConstraints the first form is used,
--     otherwise it is treated as a Remesher/MeshRefinerBase (constraints are created if missing):
--       FixAllBoundaryEdges(cons, mesh) / FixAllBoundaryEdges(r)
--       FixAllGroupBoundaryEdges(cons, mesh, bPinVertices) / FixAllGroupBoundaryEdges(r, bPinVertices)
--       ConstrainVtxLoopTo(cons, mesh, loopV, target, setID?) / ConstrainVtxLoopTo(r, loopV, target, setID?)
--       ConstrainVtxSpanTo(cons, mesh, spanV, target, setID?) / ConstrainVtxSpanTo(r, spanV, target, setID?)
--       PreserveBoundaryLoops(cons, mesh) / PreserveBoundaryLoops(r)
--   * Edge / vertex id lists (C# IEnumerable<int>, IList<int>, int[]) are 1-based Luau arrays;
--     FixEdges also accepts an iterator function or a set table {[eid] = true}.
--   * FixSubmeshBoundaryEdges(cons, sub) expects a DSubmesh3 with fields BaseBorderE (array, set
--     table or iterator of base edge ids), BaseMesh, SubMesh and method MapVerticesToSubmesh(Index2i).
--   * The Remesher overloads of ConstrainVtxLoopTo/ConstrainVtxSpanTo drop setID exactly like the C#.

local MeshConstraints = require(script.Parent.MeshConstraints)
local Util = require(script.Parent.Util)

local EdgeConstraint = MeshConstraints.EdgeConstraint
local VertexConstraint = MeshConstraints.VertexConstraint
local EdgeRefineFlags = MeshConstraints.EdgeRefineFlags

local InvalidID = -1

local MeshConstraintUtil = {}

local function isConstraints(o: any): boolean
	return MeshConstraints.IsMeshConstraints(o)
end

-- returns constraints of refiner r, creating them if needed
local function refiner_constraints(r)
	if r:Constraints() == nil then
		r:SetExternalConstraints(MeshConstraints.new())
	end
	return r:Constraints()
end

-- iterate ids from array / iterator / set
local function each_id(source: any, f: (number) -> ())
	if type(source) == "function" then
		for id in source do
			f(id)
		end
		return
	end
	local _, firstValue = next(source)
	if firstValue == nil then
		return
	end
	if type(firstValue) == "number" then
		-- 1-based array of ids
		for _, id in ipairs(source) do
			f(id)
		end
	else
		-- set table {[id] = true}; sort for deterministic order
		local ids = {}
		for id, v in source do
			if v then
				table.insert(ids, id)
			end
		end
		table.sort(ids)
		for _, id in ids do
			f(id)
		end
	end
end

-- for all edges, disable flip/split/collapse
-- for all vertices, pin in current position
function MeshConstraintUtil.FixEdges(cons, mesh, edges: any)
	each_id(edges, function(ei)
		if mesh:IsEdge(ei) then
			cons:SetOrUpdateEdgeConstraint(ei, EdgeConstraint.FullyConstrained)
			local ev = mesh:GetEdgeV(ei)
			cons:SetOrUpdateVertexConstraint(ev[1], VertexConstraint.Pinned)
			cons:SetOrUpdateVertexConstraint(ev[2], VertexConstraint.Pinned)
		end
	end)
end

-- for all mesh boundary edges, disable flip/split/collapse
-- for all mesh boundary vertices, pin in current position
function MeshConstraintUtil.FixAllBoundaryEdges(cons, mesh)
	if not isConstraints(cons) then
		local r = cons
		MeshConstraintUtil.FixAllBoundaryEdges(refiner_constraints(r), r:Mesh())
		return
	end
	local NE = mesh:MaxEdgeID()
	for ei = 0, NE - 1 do
		if mesh:IsEdge(ei) and mesh:IsBoundaryEdge(ei) then
			cons:SetOrUpdateEdgeConstraint(ei, EdgeConstraint.FullyConstrained)

			local ev = mesh:GetEdgeV(ei)
			cons:SetOrUpdateVertexConstraint(ev[1], VertexConstraint.Pinned)
			cons:SetOrUpdateVertexConstraint(ev[2], VertexConstraint.Pinned)
		end
	end
end

-- for all mesh boundary vertices, pin in current position, but allow collapses
function MeshConstraintUtil.FixAllBoundaryEdges_AllowCollapse(cons, mesh, setID: number)
	local edgeCons = EdgeConstraint.new(bit32.bor(EdgeRefineFlags.NoFlip, EdgeRefineFlags.NoSplit))
	local vertCons = VertexConstraint.new(true, setID)

	local NE = mesh:MaxEdgeID()
	for ei = 0, NE - 1 do
		if mesh:IsEdge(ei) and mesh:IsBoundaryEdge(ei) then
			cons:SetOrUpdateEdgeConstraint(ei, edgeCons)

			local ev = mesh:GetEdgeV(ei)
			cons:SetOrUpdateVertexConstraint(ev[1], vertCons)
			cons:SetOrUpdateVertexConstraint(ev[2], vertCons)
		end
	end
end

-- for all mesh boundary vertices, pin in current position, but allow splits
function MeshConstraintUtil.FixAllBoundaryEdges_AllowSplit(cons, mesh, setID: number)
	local edgeCons = EdgeConstraint.new(bit32.bor(EdgeRefineFlags.NoFlip, EdgeRefineFlags.NoCollapse))
	local vertCons = VertexConstraint.new(true, setID)

	local NE = mesh:MaxEdgeID()
	for ei = 0, NE - 1 do
		if mesh:IsEdge(ei) and mesh:IsBoundaryEdge(ei) then
			cons:SetOrUpdateEdgeConstraint(ei, edgeCons)

			local ev = mesh:GetEdgeV(ei)
			cons:SetOrUpdateVertexConstraint(ev[1], vertCons)
			cons:SetOrUpdateVertexConstraint(ev[2], vertCons)
		end
	end
end

-- loop through submesh border edges on basemesh, map to submesh, and
-- pin those edges / vertices
function MeshConstraintUtil.FixSubmeshBoundaryEdges(cons, sub)
	Util.gDevAssert(sub.BaseBorderE ~= nil)
	each_id(sub.BaseBorderE, function(base_eid)
		local base_ev = sub.BaseMesh:GetEdgeV(base_eid)
		local sub_ev = sub:MapVerticesToSubmesh(base_ev)
		local sub_eid = sub.SubMesh:FindEdge(sub_ev[1], sub_ev[2])
		Util.gDevAssert(sub_eid ~= InvalidID)
		Util.gDevAssert(sub.SubMesh:IsBoundaryEdge(sub_eid))

		cons:SetOrUpdateEdgeConstraint(sub_eid, EdgeConstraint.FullyConstrained)
		cons:SetOrUpdateVertexConstraint(sub_ev[1], VertexConstraint.Pinned)
		cons:SetOrUpdateVertexConstraint(sub_ev[2], VertexConstraint.Pinned)
	end)
end

-- for all mesh group boundary edges, disable flip/split/collapse
-- for all mesh group boundary vertices, pin in current position (if bPinVertices)
function MeshConstraintUtil.FixAllGroupBoundaryEdges(cons, mesh, bPinVertices: boolean?)
	if not isConstraints(cons) then
		local r = cons
		local bPin = mesh -- (r, bPinVertices)
		MeshConstraintUtil.FixAllGroupBoundaryEdges(refiner_constraints(r), r:Mesh(), bPin)
		return
	end
	local NE = mesh:MaxEdgeID()
	for ei = 0, NE - 1 do
		if mesh:IsEdge(ei) and mesh:IsGroupBoundaryEdge(ei) then
			cons:SetOrUpdateEdgeConstraint(ei, EdgeConstraint.FullyConstrained)

			if bPinVertices then
				local ev = mesh:GetEdgeV(ei)
				cons:SetOrUpdateVertexConstraint(ev[1], VertexConstraint.Pinned)
				cons:SetOrUpdateVertexConstraint(ev[2], VertexConstraint.Pinned)
			end
		end
	end
end

-- for all vertices in loopV, constrain to target
-- for all edges in loopV, disable flips and constrain to target
function MeshConstraintUtil.ConstrainVtxLoopTo(cons, mesh, loopV: any, target: any, setID: number?)
	if not isConstraints(cons) then
		-- (Remesher r, int[] loopV, IProjectionTarget target, int setID = -1); C# ignores setID here
		local r = cons
		MeshConstraintUtil.ConstrainVtxLoopTo(refiner_constraints(r), r:Mesh(), mesh, loopV)
		return
	end
	local vc = VertexConstraint.FromTarget(target)
	local N = #loopV
	for i = 1, N do
		cons:SetOrUpdateVertexConstraint(loopV[i], vc)
	end

	local ec = EdgeConstraint.new(EdgeRefineFlags.NoFlip, target)
	ec.TrackingSetID = setID or -1
	for i = 0, N - 1 do
		local v0 = loopV[i + 1]
		local v1 = loopV[((i + 1) % N) + 1]

		local eid = mesh:FindEdge(v0, v1)
		Util.gDevAssert(eid ~= InvalidID)
		if eid ~= InvalidID then
			cons:SetOrUpdateEdgeConstraint(eid, ec)
		end
	end
end

-- for all vertices in spanV, constrain to target (endpoints are pinned)
-- for all edges in spanV, disable flips and constrain to target
function MeshConstraintUtil.ConstrainVtxSpanTo(cons, mesh, spanV: any, target: any, setID: number?)
	if not isConstraints(cons) then
		-- (Remesher r, int[] spanV, IProjectionTarget target, int setID = -1); C# ignores setID here
		local r = cons
		MeshConstraintUtil.ConstrainVtxSpanTo(refiner_constraints(r), r:Mesh(), mesh, spanV)
		return
	end
	local vc = VertexConstraint.FromTarget(target)
	local N = #spanV
	for i = 2, N - 1 do
		cons:SetOrUpdateVertexConstraint(spanV[i], vc)
	end
	cons:SetOrUpdateVertexConstraint(spanV[1], VertexConstraint.Pinned)
	cons:SetOrUpdateVertexConstraint(spanV[N], VertexConstraint.Pinned)

	local ec = EdgeConstraint.new(EdgeRefineFlags.NoFlip, target)
	ec.TrackingSetID = setID or -1
	for i = 1, N - 1 do
		local v0 = spanV[i]
		local v1 = spanV[i + 1]

		local eid = mesh:FindEdge(v0, v1)
		Util.gDevAssert(eid ~= InvalidID)
		if eid ~= InvalidID then
			cons:SetOrUpdateEdgeConstraint(eid, ec)
		end
	end
end

function MeshConstraintUtil.PreserveBoundaryLoops(cons, mesh)
	if not isConstraints(cons) then
		local r = cons
		MeshConstraintUtil.PreserveBoundaryLoops(refiner_constraints(r), r:Mesh())
		return
	end
	local MeshBoundaryLoops = require(script.Parent.MeshBoundaryLoops)
	local MeshUtil = require(script.Parent.MeshUtil)
	local DCurveProjectionTarget = require(script.Parent.DCurveProjectionTarget)
	local loops = MeshBoundaryLoops.new(mesh)
	for _, loop in loops.Loops do
		local loopC = MeshUtil.ExtractLoopV(mesh, loop.Vertices)
		local target = DCurveProjectionTarget.new(loopC)
		MeshConstraintUtil.ConstrainVtxLoopTo(cons, mesh, loop.Vertices, target)
	end
end

function MeshConstraintUtil.AddTrackedEdges(cons, edges: { number }, setID: number)
	local ec = EdgeConstraint.Unconstrained:Clone()
	ec.TrackingSetID = setID
	for i = 1, #edges do
		cons:SetOrUpdateEdgeConstraint(edges[i], ec)
	end
end

return MeshConstraintUtil
